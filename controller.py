"""Continuous decentralized controller for the SWARMIX tunnel scenario."""

import math
from typing import Optional

from config import (
    BOTTLENECK_ALIGNMENT_FULL_SPEED_METERS,
    BOTTLENECK_ALIGNMENT_STOP_METERS,
    BOTTLENECK_APPROACH_SPEED,
    BOTTLENECK_LOOKAHEAD_DISTANCE,
    BOTTLENECK_MAX_OPENING_WIDTH,
    BOTTLENECK_PREPARE_COMPLETE_DISTANCE,
    CENTER_GAIN,
    DISTANCE_TOLERANCE,
    FORMATION_GAIN,
    FORWARD_SPEED,
    GAP_ALIGNMENT_FULL_SPEED_METERS,
    GAP_ALIGNMENT_STOP_METERS,
    GAP_APPROACH_SPEED,
    GAP_LANE_GAIN,
    GAP_PEER_AVOIDANCE_WEIGHT,
    GOAL_CLEAR_DISTANCE,
    HARD_WALL_CLEARANCE,
    INITIAL_FORMATION_SPACING,
    LEADER_SELECTION_MARGIN,
    LONGITUDINAL_SYNC_GAIN,
    MAX_LATERAL_SPEED,
    MAX_VELOCITY,
    PEER_FOLLOW_DISTANCE,
    PEER_HARD_DISTANCE,
    PEER_SAFE_DISTANCE,
    SYNC_CATCHUP_MAX_PATH_ANGLE_DEG,
    SYNC_CATCHUP_MAX_SPEED,
    TARGET_EAST,
    TARGET_TOLERANCE,
    WALL_AVOIDANCE_GAIN,
    WALL_CLEARANCE,
    WALL_INFLUENCE_DISTANCE,
    WALL_STOP_DISTANCE,
)
from lidar_processor import LidarResult
from state import UAVState


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def column_formation_weight(
    lidar: LidarResult,
    own: Optional[UAVState] = None,
    neighbor: Optional[UAVState] = None,
) -> float:
    """Return continuous preparation progress for the 1 m column."""

    if (
        lidar.left_wall_seen
        and lidar.right_wall_seen
        and lidar.free_width <= BOTTLENECK_MAX_OPENING_WIDTH
    ):
        return 1.0
    if not lidar.bottleneck_found:
        return 0.0

    group_distance = lidar.bottleneck_distance
    if (
        own is not None
        and neighbor is not None
        and own.is_valid()
        and neighbor.is_valid()
    ):
        group_distance += max(0.0, own.east - neighbor.east)

    return clamp(
        (BOTTLENECK_LOOKAHEAD_DISTANCE - group_distance)
        / (
            BOTTLENECK_LOOKAHEAD_DISTANCE
            - BOTTLENECK_PREPARE_COMPLETE_DISTANCE
        ),
        0.0,
        1.0,
    )


def desired_spacing(free_width: float) -> float:
    usable_width = max(0.0, free_width - 2.0 * WALL_CLEARANCE)
    return min(INITIAL_FORMATION_SPACING, usable_width)


def corridor_seen(lidar: LidarResult) -> bool:
    return (
        lidar.left_wall_seen
        and lidar.right_wall_seen
        and lidar.free_width
        <= INITIAL_FORMATION_SPACING + 2.0 * WALL_CLEARANCE
    )


def lateral_correction(
    own: UAVState,
    neighbor: Optional[UAVState],
    lidar: LidarResult,
    formation_side: int,
    column_weight: float,
) -> float:
    """Track a real lateral target with a zero-error equilibrium."""

    if lidar.gap_active:
        return clamp(
            GAP_LANE_GAIN * lidar.gap_lateral_error,
            -MAX_LATERAL_SPEED,
            MAX_LATERAL_SPEED,
        )

    side_weight = 1.0 - clamp(column_weight, 0.0, 1.0)
    if corridor_seen(lidar):
        desired_offset = (
            formation_side
            * side_weight
            * desired_spacing(lidar.free_width)
            / 2.0
        )
        error = lidar.center_error + desired_offset
        return clamp(
            CENTER_GAIN * error,
            -MAX_LATERAL_SPEED,
            MAX_LATERAL_SPEED,
        )

    if neighbor is None or not neighbor.is_valid():
        return 0.0
    current_separation = own.north - neighbor.north
    desired_separation = (
        formation_side * side_weight * INITIAL_FORMATION_SPACING
    )
    error = 0.5 * (desired_separation - current_separation)
    return clamp(
        FORMATION_GAIN * error,
        -MAX_LATERAL_SPEED,
        MAX_LATERAL_SPEED,
    )


def longitudinal_synchronization_speed(
    own: UAVState,
    neighbor: Optional[UAVState],
    safe_speed: float,
    column_weight: float,
    catchup_allowed: bool,
) -> float:
    safe_speed = max(0.0, safe_speed)
    if neighbor is None or not neighbor.is_valid():
        return 0.0

    side_weight = 1.0 - column_weight
    if side_weight <= 0.0:
        return safe_speed

    error = own.east - neighbor.east
    if error > DISTANCE_TOLERANCE:
        synchronized = 0.0
    elif error < -DISTANCE_TOLERANCE and catchup_allowed:
        correction = LONGITUDINAL_SYNC_GAIN * (
            -error - DISTANCE_TOLERANCE
        )
        synchronized = clamp(
            safe_speed + correction,
            0.0,
            SYNC_CATCHUP_MAX_SPEED,
        )
    else:
        synchronized = safe_speed

    return clamp(
        side_weight * synchronized + column_weight * safe_speed,
        0.0,
        max(safe_speed, synchronized),
    )


def following_scale(
    uav_id: int,
    own: UAVState,
    neighbor: Optional[UAVState],
    column_weight: float,
    leader_id: Optional[int],
) -> float:
    if (
        column_weight <= 0.0
        or neighbor is None
        or not neighbor.is_valid()
        or leader_id is None
        or uav_id == leader_id
    ):
        return 1.0

    leader_gap = max(0.0, neighbor.east - own.east)
    desired_gap = column_weight * PEER_FOLLOW_DISTANCE
    if desired_gap <= 0.0:
        return 1.0
    completion = clamp(leader_gap / desired_gap, 0.0, 1.0)
    return 1.0 - column_weight * (1.0 - completion)


def select_bottleneck_leader(
    uav_id: int,
    own: UAVState,
    neighbor_id: int,
    neighbor: Optional[UAVState],
) -> Optional[int]:
    """Select the UAV that is physically farther along the East axis.

    The id is used only when the longitudinal difference is inside the
    selection margin.  Both agents can therefore reach the same result from
    the two positions without a central coordinator.
    """

    if (
        not own.is_valid()
        or neighbor is None
        or not neighbor.is_valid()
    ):
        return None

    east_difference = own.east - neighbor.east
    if east_difference > LEADER_SELECTION_MARGIN:
        return uav_id
    if east_difference < -LEADER_SELECTION_MARGIN:
        return neighbor_id
    return min(uav_id, neighbor_id)


def update_bottleneck_leader(
    current_leader_id: Optional[int],
    column_weight: float,
    uav_id: int,
    own: UAVState,
    neighbor_id: int,
    neighbor: Optional[UAVState],
) -> Optional[int]:
    """Lock one temporary leader while the group is in column transition."""

    if column_weight <= 0.0:
        return None
    if current_leader_id in (uav_id, neighbor_id):
        return current_leader_id
    return select_bottleneck_leader(
        uav_id,
        own,
        neighbor_id,
        neighbor,
    )


def resolve_bottleneck_leader(
    local_leader_id: Optional[int],
    peer_leader_id: Optional[int],
) -> Optional[int]:
    """Make simultaneous peer proposals converge deterministically.

    Different proposals mean the ordering was too close or too stale to be
    trusted.  In that exceptional case the smaller id is only a tie-breaker;
    it is not the normal, fixed leader.
    """

    if local_leader_id is None:
        return peer_leader_id
    if peer_leader_id is None or peer_leader_id == local_leader_id:
        return local_leader_id
    return min(local_leader_id, peer_leader_id)


def peer_avoidance(
    own: UAVState,
    neighbor: Optional[UAVState],
    formation_side: int,
):
    if neighbor is None or not neighbor.is_valid():
        return 0.0, 0.0, None

    dn = own.north - neighbor.north
    de = own.east - neighbor.east
    distance = math.hypot(dn, de)
    if distance == 0.0:
        return (
            formation_side * FORMATION_GAIN * PEER_SAFE_DISTANCE,
            0.0,
            distance,
        )
    if distance >= PEER_SAFE_DISTANCE:
        return 0.0, 0.0, distance

    strength = FORMATION_GAIN * (PEER_SAFE_DISTANCE - distance)
    return strength * dn / distance, strength * de / distance, distance


def apply_hard_peer_safety(
    velocity_north: float,
    velocity_east: float,
    own: UAVState,
    neighbor: Optional[UAVState],
):
    """Remove only motion that closes an already-critical peer distance."""
    if neighbor is None or not neighbor.is_valid():
        return velocity_north, velocity_east

    to_peer_north = neighbor.north - own.north
    to_peer_east = neighbor.east - own.east
    distance = math.hypot(to_peer_north, to_peer_east)
    if distance <= 1e-6:
        return 0.0, 0.0
    if distance > PEER_HARD_DISTANCE:
        return velocity_north, velocity_east

    unit_north = to_peer_north / distance
    unit_east = to_peer_east / distance
    closing_speed = (
        velocity_north * unit_north
        + velocity_east * unit_east
    )
    if closing_speed <= 0.0:
        return velocity_north, velocity_east

    velocity_north -= closing_speed * unit_north
    velocity_east -= closing_speed * unit_east
    return velocity_north, velocity_east


def alignment_scale(error: float, full_error: float, stop_error: float) -> float:
    error = abs(error)
    if error <= full_error:
        return 1.0
    if error >= stop_error:
        return 0.0
    return (stop_error - error) / (stop_error - full_error)


def apply_wall_safety(velocity_north: float, lidar: LidarResult) -> float:
    """Apply soft repulsion and then remove motion into a dangerous wall."""

    command = velocity_north
    span = max(1e-6, WALL_INFLUENCE_DISTANCE - HARD_WALL_CLEARANCE)
    if lidar.left_wall_seen and lidar.left_distance < WALL_INFLUENCE_DISTANCE:
        command -= WALL_AVOIDANCE_GAIN * clamp(
            (WALL_INFLUENCE_DISTANCE - lidar.left_distance) / span,
            0.0,
            1.0,
        )
    if lidar.right_wall_seen and lidar.right_distance < WALL_INFLUENCE_DISTANCE:
        command += WALL_AVOIDANCE_GAIN * clamp(
            (WALL_INFLUENCE_DISTANCE - lidar.right_distance) / span,
            0.0,
            1.0,
        )

    command = clamp(command, -MAX_LATERAL_SPEED, MAX_LATERAL_SPEED)
    if (
        lidar.left_wall_seen
        and lidar.left_distance <= HARD_WALL_CLEARANCE
        and command > 0.0
    ):
        command = 0.0
    if (
        lidar.right_wall_seen
        and lidar.right_distance <= HARD_WALL_CLEARANCE
        and command < 0.0
    ):
        command = 0.0
    return command


def calculate_velocity_command(
    own: UAVState,
    neighbor: Optional[UAVState],
    lidar: LidarResult,
    uav_id: int,
    formation_side: int,
    column_weight_override: Optional[float] = None,
    bottleneck_leader_id: Optional[int] = None,
):
    local_weight = column_formation_weight(lidar, own, neighbor)
    column_weight = local_weight
    if column_weight_override is not None:
        column_weight = max(
            column_weight,
            clamp(column_weight_override, 0.0, 1.0),
        )

    remaining = TARGET_EAST - own.east
    forward = clamp(0.5 * remaining, 0.0, FORWARD_SPEED)
    blocked_without_route = (
        lidar.goal_blocked
        and not lidar.gap_found
        and not lidar.gap_active
        and not lidar.bottleneck_found
    )
    if blocked_without_route:
        forward = 0.0
    elif lidar.gap_active:
        forward = min(forward, GAP_APPROACH_SPEED)
    elif column_weight > 0.0:
        forward = min(forward, BOTTLENECK_APPROACH_SPEED)

    velocity_north = lateral_correction(
        own,
        neighbor,
        lidar,
        formation_side,
        column_weight,
    )
    velocity_east = forward

    avoid_north, avoid_east, peer_distance = peer_avoidance(
        own,
        neighbor,
        formation_side,
    )

    # Solution 3: lane tracking has priority over SOFT peer avoidance only
    # when their North commands oppose each other.
    if (
        lidar.gap_active
        and peer_distance is not None
        and peer_distance > PEER_HARD_DISTANCE
        and velocity_north * avoid_north < 0.0
    ):
        avoid_north *= GAP_PEER_AVOIDANCE_WEIGHT

    velocity_north += avoid_north
    velocity_east += avoid_east

    if lidar.goal_blocked:
        velocity_east *= clamp(
            (lidar.front_distance - WALL_STOP_DISTANCE)
            / max(1e-6, GOAL_CLEAR_DISTANCE - WALL_STOP_DISTANCE),
            0.0,
            1.0,
        )
    velocity_east = max(0.0, velocity_east)

    if lidar.gap_active:
        velocity_east *= alignment_scale(
            lidar.gap_lateral_error,
            GAP_ALIGNMENT_FULL_SPEED_METERS,
            GAP_ALIGNMENT_STOP_METERS,
        )
    if column_weight > 0.0:
        centre_error = (
            lidar.bottleneck_lateral_error
            if lidar.bottleneck_found
            else lidar.center_error
        )
        narrow_scale = alignment_scale(
            centre_error,
            BOTTLENECK_ALIGNMENT_FULL_SPEED_METERS,
            BOTTLENECK_ALIGNMENT_STOP_METERS,
        )
        velocity_east *= (
            (1.0 - column_weight) + column_weight * narrow_scale
        )

    catchup_allowed = (
        not lidar.goal_blocked
        and abs(math.degrees(lidar.path_angle))
        <= SYNC_CATCHUP_MAX_PATH_ANGLE_DEG
    )
    velocity_east = longitudinal_synchronization_speed(
        own,
        neighbor,
        velocity_east,
        column_weight,
        catchup_allowed,
    )
    velocity_east *= following_scale(
        uav_id,
        own,
        neighbor,
        column_weight,
        bottleneck_leader_id,
    )

    if lidar.front_distance <= WALL_STOP_DISTANCE:
        velocity_east = 0.0

    velocity_north, velocity_east = apply_hard_peer_safety(
        velocity_north,
        velocity_east,
        own,
        neighbor,
    )
    velocity_east = max(0.0, velocity_east)
    velocity_north = apply_wall_safety(velocity_north, lidar)

    magnitude = math.hypot(velocity_north, velocity_east)
    if magnitude > MAX_VELOCITY:
        scale = MAX_VELOCITY / magnitude
        velocity_north *= scale
        velocity_east *= scale
    return velocity_north, velocity_east, 0.0, peer_distance


def has_reached_target(state: UAVState) -> bool:
    return state.is_valid() and state.east >= TARGET_EAST - TARGET_TOLERANCE


