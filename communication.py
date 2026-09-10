import asyncio
import json
import time

from state import UAVState
from config import COMMUNICATION_TIMEOUT


class UDPProtocol(asyncio.DatagramProtocol):

    def __init__(self, communication):
        self.communication = communication

    def datagram_received(self, data, addr):

        try:

            message = json.loads(
                data.decode("utf-8")
            )

            uav_id = message["uav_id"]

            # ------------------------------------------------
            # Receive UAV state
            # ------------------------------------------------

            state = UAVState(
                north=message["north"],
                east=message["east"],
                down=message["down"],
            )

            self.communication.neighbor_states[
                uav_id
            ] = state

            # Store reception time
            self.communication.last_received[
                uav_id
            ] = time.monotonic()

            # ------------------------------------------------
            # Receive formation readiness
            # ------------------------------------------------

            self.communication.neighbor_ready[
                uav_id
            ] = message.get(
                "ready",
                False,
            )
            self.communication.neighbor_coordination[uav_id] = {
                "column_weight": float(message.get("column_weight", 0.0)),
                "bottleneck_active": bool(
                    message.get("bottleneck_active", False)
                ),
                "inside_narrow": bool(message.get("inside_narrow", False)),
                "leader_id": normalize_leader_id(
                    message.get("leader_id")
                ),
            }

        except Exception as e:

            print(
                f"Communication receive error: {e}"
            )


class Communication:
    """
    Handles UAV-to-UAV communication using UDP.

    Each UAV periodically sends:
        - current position
        - readiness state

    The receiver also tracks the time of the
    last received packet to detect stale communication.
    """

    def __init__(
        self,
        uav_id,
        local_port,
        peers,
    ):

        self.uav_id = uav_id
        self.local_port = local_port
        self.peers = peers

        self.transport = None

        # Latest state received from neighbors
        self.neighbor_states = {}

        # Last packet reception time
        self.last_received = {}

        # Formation readiness
        self.neighbor_ready = {}

        # Local readiness
        self.ready = False
        self.local_coordination = {
            "column_weight": 0.0,
            "bottleneck_active": False,
            "inside_narrow": False,
            "leader_id": None,
        }
        self.neighbor_coordination = {}

    # ========================================================
    # Start communication
    # ========================================================

    async def start(self):

        loop = asyncio.get_running_loop()

        transport, _ = (
            await loop.create_datagram_endpoint(
                lambda: UDPProtocol(self),
                local_addr=(
                    "0.0.0.0",
                    self.local_port,
                ),
            )
        )

        self.transport = transport

        print(
            f"UAV {self.uav_id}: "
            f"communication listening on UDP "
            f"{self.local_port}"
        )

    # ========================================================
    # Formation readiness
    # ========================================================

    def set_ready(self):

        self.ready = True

    def is_ready(self):

        return self.ready

    def is_neighbor_ready(
        self,
        neighbor_id,
    ):

        return self.neighbor_ready.get(
            neighbor_id,
            False,
        )

    # ========================================================
    # Communication freshness
    # ========================================================

    def is_neighbor_state_fresh(
        self,
        neighbor_id,
    ):
        """
        Checks whether the latest state received
        from the neighbor is still fresh.
        """

        last_received = self.last_received.get(
            neighbor_id
        )

        if last_received is None:
            return False

        age = (
            time.monotonic()
            - last_received
        )

        return age <= COMMUNICATION_TIMEOUT

    def get_neighbor_state_age(
        self,
        neighbor_id,
    ):
        """
        Returns the age of the latest
        neighbor state in seconds.
        """

        last_received = self.last_received.get(
            neighbor_id
        )

        if last_received is None:
            return None

        return (
            time.monotonic()
            - last_received
        )

    # ========================================================
    # State communication
    # ========================================================

    def set_coordination(
        self,
        column_weight,
        bottleneck_active,
        inside_narrow,
        leader_id,
    ):
        self.local_coordination = {
            "column_weight": max(0.0, min(float(column_weight), 1.0)),
            "bottleneck_active": bool(bottleneck_active),
            "inside_narrow": bool(inside_narrow),
            "leader_id": normalize_leader_id(leader_id),
        }

    def get_neighbor_coordination(self, neighbor_id):
        return self.neighbor_coordination.get(
            neighbor_id,
            {
                "column_weight": 0.0,
                "bottleneck_active": False,
                "inside_narrow": False,
                "leader_id": None,
            },
        ).copy()

    def send_state(
        self,
        state: UAVState,
    ):

        if self.transport is None:
            return

        if not state.is_valid():
            return

        message = {
            "uav_id": self.uav_id,
            "timestamp": time.time(),

            # Position
            "north": state.north,
            "east": state.east,
            "down": state.down,

            # Formation synchronization
            "ready": self.ready,
            "column_weight": self.local_coordination["column_weight"],
            "bottleneck_active": self.local_coordination[
                "bottleneck_active"
            ],
            "inside_narrow": self.local_coordination["inside_narrow"],
            "leader_id": self.local_coordination["leader_id"],
        }

        payload = json.dumps(
            message
        ).encode("utf-8")

        for peer in self.peers:

            self.transport.sendto(
                payload,
                (
                    peer["host"],
                    peer["port"],
                ),
            )

    # ========================================================
    # Neighbor state
    # ========================================================

    def get_neighbor_state(
        self,
        neighbor_id,
    ):

        return self.neighbor_states.get(
            neighbor_id
        )

    # ========================================================
    # Close communication
    # ========================================================

    async def close(self):

        if self.transport:

            self.transport.close()

            self.transport = None


def normalize_leader_id(value):
    """Return a positive integer leader id or ``None`` for invalid input."""

    if isinstance(value, bool):
        return None
    try:
        leader_id = int(value)
    except (TypeError, ValueError):
        return None
    return leader_id if leader_id > 0 else None


