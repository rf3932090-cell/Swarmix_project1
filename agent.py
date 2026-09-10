"""Independent UAV agent with continuous gap and column coordination."""

import asyncio
from dataclasses import replace
import logging
import math
import time

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from communication import Communication
from config import (
    ALTITUDE_GAIN,
    BOTTLENECK_CONFIRM_SCANS,
    BOTTLENECK_EXIT_SCANS,
    BOTTLENECK_EXIT_WIDTH,
    BOTTLENECK_LOST_SCANS,
    BOTTLENECK_MAX_OPENING_WIDTH,
    COLUMN_RELEASE_TIME,
    CONTROL_PERIOD,
    GAP_RELEASE_SCANS,
    GAP_RELEASE_WIDTH_MARGIN,
    GOAL_CLEAR_DISTANCE,
    LIDAR_TIMEOUT,
    LOG_PERIOD,
    MAX_ALTITUDE_CORRECTION,
    POSITION_TOLERANCE,
    TAKEOFF_ALTITUDE,
    TARGET_YAW_DEG,
    WALL_CLEARANCE,
    WALL_STOP_DISTANCE,
)
from controller import (
    calculate_velocity_command,
    column_formation_weight,
    has_reached_target,
    resolve_bottleneck_leader,
    update_bottleneck_leader,
)
from lidar_interface import LidarInterface
from lidar_processor import process_scan
from state import UAVState
from telemetry import telemetry_loop


logger = logging.getLogger(__name__)


class UAVAgent:
    def __init__(self, uav_id, config, peers):
        self.uav_id = uav_id
        self.name = config["name"]
        self.formation_north = config["formation_north"]
        self.formation_east = config["formation_east"]
        self.formation_side = config["formation_side"]
        self.drone = System(port=config["mavsdk_port"])
        self.system_address = config["system_address"]
        self.state = UAVState()
        self.lidar = LidarInterface(config["lidar_topic"], LIDAR_TIMEOUT)
        self.communication = Communication(
            uav_id=uav_id,
            local_port=config["communication_port"],
            peers=peers,
        )
        self.telemetry_task = None
        self.communication_task = None
        self.last_diagnostic_log = 0.0

        self.gap_active = False
        self.locked_gap_width = math.inf
        self.locked_gap_lane_north = None
        self.gap_release_count = 0

        self.bottleneck_confirm_count = 0
        self.bottleneck_lost_count = 0
        self.bottleneck_active = False
        self.bottleneck_entered = False
        self.bottleneck_exit_count = 0
        self.locked_bottleneck_east = None
        self.locked_bottleneck_north = None
        self.locked_bottleneck_width = math.inf
        self.local_column_weight = 0.0
        self.group_column_weight = 0.0
        self.bottleneck_leader_id = None

    async def connect(self):
        print(f"{self.name}: connecting...")
        await self.drone.connect(system_address=self.system_address)
        async for connection in self.drone.core.connection_state():
            if connection.is_connected:
                print(f"{self.name}: connected")
                return

    async def communication_loop(self):
        while True:
            self.communication.send_state(self.state)
            await asyncio.sleep(CONTROL_PERIOD)

    async def prepare(self):
        while not self.state.is_valid():
            await asyncio.sleep(CONTROL_PERIOD)
        start_north = self.state.north
        start_east = self.state.east
        await self.drone.action.arm()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                start_north,
                start_east,
                -TAKEOFF_ALTITUDE,
                TARGET_YAW_DEG,
            )
        )
        await self.drone.offboard.start()
        await self.wait_for_altitude()
        await self.move_to_initial_formation()

    async def wait_for_altitude(self):
        while True:
            if (
                self.state.down is not None
                and -self.state.down >= TAKEOFF_ALTITUDE - 0.15
            ):
                return
            await asyncio.sleep(CONTROL_PERIOD)

    async def move_to_initial_formation(self):
        while True:
            if not self.state.is_valid():
                await asyncio.sleep(CONTROL_PERIOD)
                continue
            distance = math.sqrt(
                (self.state.north - self.formation_north) ** 2
                + (self.state.east - self.formation_east) ** 2
                + (self.state.down + TAKEOFF_ALTITUDE) ** 2
            )
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    self.formation_north,
                    self.formation_east,
                    -TAKEOFF_ALTITUDE,
                    TARGET_YAW_DEG,
                )
            )
            if distance <= POSITION_TOLERANCE:
                print(f"{self.name}: initial formation reached")
                return
            await asyncio.sleep(CONTROL_PERIOD)

    async def wait_for_both_ready(self, neighbor_id):
        self.communication.set_ready()
        while not (
            self.communication.is_ready()
            and self.communication.is_neighbor_ready(neighbor_id)
        ):
            await asyncio.sleep(CONTROL_PERIOD)

    def altitude_velocity(self):
        altitude = -self.state.down
        return (
            clamp(
                ALTITUDE_GAIN * (altitude - TAKEOFF_ALTITUDE),
                -MAX_ALTITUDE_CORRECTION,
                MAX_ALTITUDE_CORRECTION,
            ),
            altitude,
        )

    def fresh_neighbor(self, neighbor_id):
        if not self.communication.is_neighbor_state_fresh(neighbor_id):
            return None
        return self.communication.get_neighbor_state(neighbor_id)

    @staticmethod
    def is_inside_narrow(lidar):
        return (
            lidar.left_wall_seen
            and lidar.right_wall_seen
            and lidar.free_width <= BOTTLENECK_MAX_OPENING_WIDTH
        )

    def clear_bottleneck_lock(self):
        self.bottleneck_confirm_count = 0
        self.bottleneck_lost_count = 0
        self.bottleneck_active = False
        self.bottleneck_entered = False
        self.bottleneck_exit_count = 0
        self.locked_bottleneck_east = None
        self.locked_bottleneck_north = None
        self.locked_bottleneck_width = math.inf

    def apply_bottleneck_lock(self, lidar):
        """Confirm and hold the physical shoulder through temporary scan loss."""

        if lidar.bottleneck_found:
            candidate_east = self.state.east + lidar.bottleneck_distance
            candidate_north = self.state.north + lidar.bottleneck_lateral_error
            self.bottleneck_confirm_count += 1
            self.bottleneck_lost_count = 0
            if self.bottleneck_active:
                self.locked_bottleneck_east = (
                    0.8 * self.locked_bottleneck_east + 0.2 * candidate_east
                )
                self.locked_bottleneck_north = (
                    0.8 * self.locked_bottleneck_north + 0.2 * candidate_north
                )
                self.locked_bottleneck_width = (
                    0.8 * self.locked_bottleneck_width
                    + 0.2 * lidar.bottleneck_width
                )
            elif self.bottleneck_confirm_count >= BOTTLENECK_CONFIRM_SCANS:
                self.bottleneck_active = True
                self.locked_bottleneck_east = candidate_east
                self.locked_bottleneck_north = candidate_north
                self.locked_bottleneck_width = lidar.bottleneck_width
        elif self.bottleneck_active:
            self.bottleneck_lost_count += 1
        else:
            self.bottleneck_confirm_count = 0

        inside = self.is_inside_narrow(lidar)
        if self.bottleneck_active:
            if inside:
                self.bottleneck_entered = True
                self.bottleneck_exit_count = 0
            elif self.bottleneck_entered and lidar.free_width >= BOTTLENECK_EXIT_WIDTH:
                self.bottleneck_exit_count += 1
                if self.bottleneck_exit_count >= BOTTLENECK_EXIT_SCANS:
                    self.clear_bottleneck_lock()
            else:
                self.bottleneck_exit_count = 0

            if (
                self.bottleneck_active
                and not self.bottleneck_entered
                and self.bottleneck_lost_count >= BOTTLENECK_LOST_SCANS
                and self.state.east > self.locked_bottleneck_east + 0.5
            ):
                self.clear_bottleneck_lock()

        if not self.bottleneck_active:
            return replace(
                lidar,
                bottleneck_found=False,
                bottleneck_distance=math.inf,
                bottleneck_width=math.inf,
                bottleneck_angle=0.0,
                bottleneck_lateral_error=0.0,
            )

        distance = self.locked_bottleneck_east - self.state.east
        lateral_error = self.locked_bottleneck_north - self.state.north
        angle = math.atan2(lateral_error, max(0.1, distance))
        return replace(
            lidar,
            path_angle=angle,
            bottleneck_found=True,
            bottleneck_distance=distance,
            bottleneck_width=self.locked_bottleneck_width,
            bottleneck_angle=angle,
            bottleneck_lateral_error=lateral_error,
        )

    def apply_gap_lock(self, lidar):
        """Assign each UAV its own safe lane through a wide entrance."""

        wide_gap = (
            lidar.goal_blocked
            and lidar.gap_found
            and lidar.gap_width > BOTTLENECK_MAX_OPENING_WIDTH
        )
        if wide_gap:
            if not self.gap_active:
                centre_lateral = lidar.front_distance * math.tan(lidar.path_angle)
                centre_north = self.state.north + centre_lateral
                half_lane = max(0.0, lidar.gap_width / 2.0 - WALL_CLEARANCE)
                self.locked_gap_lane_north = (
                    centre_north + self.formation_side * half_lane
                )
                self.locked_gap_width = lidar.gap_width
                self.gap_active = True
            self.gap_release_count = 0
        elif self.gap_active:
            inside_gap = (
                lidar.left_wall_seen
                and lidar.right_wall_seen
                and lidar.free_width
                <= self.locked_gap_width + GAP_RELEASE_WIDTH_MARGIN
            )
            self.gap_release_count = (
                self.gap_release_count + 1 if inside_gap else 0
            )
            if self.gap_release_count >= GAP_RELEASE_SCANS:
                self.gap_active = False
                self.locked_gap_width = math.inf
                self.locked_gap_lane_north = None
                self.gap_release_count = 0

        if not self.gap_active:
            return replace(lidar, gap_active=False, gap_lateral_error=0.0)

        lateral_error = self.locked_gap_lane_north - self.state.north
        forward_reference = (
            clamp(
                lidar.front_distance,
                WALL_STOP_DISTANCE,
                GOAL_CLEAR_DISTANCE,
            )
            if lidar.goal_blocked
            else GOAL_CLEAR_DISTANCE
        )
        return replace(
            lidar,
            path_angle=math.atan2(lateral_error, forward_reference),
            gap_active=True,
            gap_width=self.locked_gap_width,
            gap_lateral_error=lateral_error,
        )

    def update_group_column_weight(self, neighbor_id, local_weight, inside):
        if self.communication.is_neighbor_state_fresh(neighbor_id):
            peer = self.communication.get_neighbor_coordination(neighbor_id)
        else:
            peer = {
                "column_weight": 0.0,
                "bottleneck_active": False,
                "inside_narrow": False,
                "leader_id": None,
            }
        target = max(local_weight, peer["column_weight"])
        if inside or peer["inside_narrow"]:
            target = 1.0
        if target >= self.group_column_weight:
            self.group_column_weight = target
        else:
            self.group_column_weight = max(
                target,
                self.group_column_weight
                - CONTROL_PERIOD / max(COLUMN_RELEASE_TIME, 1e-6),
            )

    async def send_velocity(self, north, east, down):
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, TARGET_YAW_DEG)
        )

    def log_snapshot(self, lidar, neighbor, vn, ve, vd, altitude, peer_distance):
        now = time.monotonic()
        if now - self.last_diagnostic_log < LOG_PERIOD:
            return
        self.last_diagnostic_log = now
        peer_text = "unavailable"
        if neighbor is not None and neighbor.is_valid():
            peer_text = (
                f"dN={neighbor.north-self.state.north:+.2f} "
                f"dE={neighbor.east-self.state.east:+.2f} "
                f"distance={format_value(peer_distance)}"
            )
        logger.info(
            "\n[%s]\n"
            "  STATE      N=%+.2f E=%+.2f Alt=%.2f\n"
            "  SEES       left=%s front=%.2f right=%s width=%.2f centre=%+.2f\n"
            "  GAP        found=%s active=%s width=%s lane_error=%+.2f\n"
            "  BOTTLENECK found=%s active=%s distance=%s width=%s lateral=%+.2f\n"
            "  COLUMN     local=%.2f group=%.2f inside=%s leader=%s\n"
            "  PEER       %s\n"
            "  COMMAND    N=%+.2f E=%+.2f D=%+.2f speed=%.2f",
            self.name,
            self.state.north,
            self.state.east,
            altitude,
            format_wall(lidar.left_distance, lidar.left_wall_seen),
            lidar.front_distance,
            format_wall(lidar.right_distance, lidar.right_wall_seen),
            lidar.free_width,
            lidar.center_error,
            yes_no(lidar.gap_found),
            yes_no(lidar.gap_active),
            format_value(lidar.gap_width),
            lidar.gap_lateral_error,
            yes_no(lidar.bottleneck_found),
            yes_no(self.bottleneck_active),
            format_value(lidar.bottleneck_distance),
            format_value(lidar.bottleneck_width),
            lidar.bottleneck_lateral_error,
            self.local_column_weight,
            self.group_column_weight,
            yes_no(self.is_inside_narrow(lidar)),
            (
                f"UAV {self.bottleneck_leader_id}"
                if self.bottleneck_leader_id is not None
                else "--"
            ),
            peer_text,
            vn,
            ve,
            vd,
            math.hypot(vn, ve),
        )

    async def control_loop(self, neighbor_id):
        print(f"{self.name}: reactive control started")
        while True:
            if not self.state.is_valid():
                await asyncio.sleep(CONTROL_PERIOD)
                continue
            velocity_down, altitude = self.altitude_velocity()
            scan = self.lidar.get_scan()
            if scan is None:
                await self.send_velocity(0.0, 0.0, velocity_down)
                await asyncio.sleep(CONTROL_PERIOD)
                continue
            if has_reached_target(self.state):
                await self.send_velocity(0.0, 0.0, velocity_down)
                print(f"{self.name}: target reached")
                return

            neighbor = self.fresh_neighbor(neighbor_id)
            lidar = self.apply_bottleneck_lock(process_scan(scan))
            lidar = self.apply_gap_lock(lidar)
            inside = self.is_inside_narrow(lidar)
            self.local_column_weight = column_formation_weight(
                lidar,
                self.state,
                neighbor,
            )
            self.update_group_column_weight(
                neighbor_id,
                self.local_column_weight,
                inside,
            )
            self.bottleneck_leader_id = update_bottleneck_leader(
                self.bottleneck_leader_id,
                self.group_column_weight,
                self.uav_id,
                self.state,
                neighbor_id,
                neighbor,
            )
            if self.communication.is_neighbor_state_fresh(neighbor_id):
                peer_leader_id = self.communication.get_neighbor_coordination(
                    neighbor_id
                )["leader_id"]
            else:
                peer_leader_id = None
            if self.group_column_weight > 0.0:
                self.bottleneck_leader_id = resolve_bottleneck_leader(
                    self.bottleneck_leader_id,
                    peer_leader_id,
                )
            self.communication.set_coordination(
                self.local_column_weight,
                self.bottleneck_active,
                inside,
                self.bottleneck_leader_id,
            )
            vn, ve, _, peer_distance = calculate_velocity_command(
                own=self.state,
                neighbor=neighbor,
                lidar=lidar,
                uav_id=self.uav_id,
                formation_side=self.formation_side,
                column_weight_override=self.group_column_weight,
                bottleneck_leader_id=self.bottleneck_leader_id,
            )
            await self.send_velocity(vn, ve, velocity_down)
            self.log_snapshot(
                lidar,
                neighbor,
                vn,
                ve,
                velocity_down,
                altitude,
                peer_distance,
            )
            await asyncio.sleep(CONTROL_PERIOD)

    async def land(self):
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        await self.drone.action.land()
        print(f"{self.name}: landing")

    async def run(self, neighbor_id):
        await self.connect()
        await self.communication.start()
        self.telemetry_task = asyncio.create_task(
            telemetry_loop(self.drone, self.state, self.name)
        )
        self.communication_task = asyncio.create_task(self.communication_loop())
        try:
            await self.prepare()
            await self.wait_for_both_ready(neighbor_id)
            await self.control_loop(neighbor_id)
            await self.land()
        finally:
            tasks = [
                task
                for task in (self.telemetry_task, self.communication_task)
                if task is not None
            ]
            for task in tasks:
                task.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
            await self.communication.close()


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def yes_no(value):
    return "yes" if value else "NO"


def format_value(value):
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:.2f} m"


def format_wall(value, seen):
    return format_value(value) if seen else "NO_WALL"

