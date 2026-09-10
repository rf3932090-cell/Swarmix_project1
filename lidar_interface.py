"""Receive the latest 2-D LiDAR scan from Gazebo."""

from dataclasses import dataclass
import threading
import time
from typing import Optional, Tuple

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node


@dataclass(frozen=True)
class LidarScanData:
    ranges: Tuple[float, ...]
    angle_min: float
    angle_step: float
    range_min: float
    range_max: float


class LidarInterface:
    """Subscribe to one UAV LiDAR topic and keep its newest scan."""

    def __init__(self, topic: str, timeout: float = 0.5):
        if not topic:
            raise ValueError("LiDAR topic cannot be empty")
        if timeout <= 0.0:
            raise ValueError("LiDAR timeout must be greater than zero")

        self.topic = topic
        self.timeout = timeout
        self.node = Node()

        self._scan: Optional[LidarScanData] = None
        self._last_update: Optional[float] = None
        self._lock = threading.Lock()

        subscribed = self.node.subscribe(
            LaserScan,
            self.topic,
            self._callback,
        )
        if not subscribed:
            raise RuntimeError(
                f"Failed to subscribe to LiDAR topic: {self.topic}"
            )

    def _callback(self, message: LaserScan) -> None:
        scan = LidarScanData(
            ranges=tuple(float(value) for value in message.ranges),
            angle_min=float(message.angle_min),
            angle_step=float(message.angle_step),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
        )

        with self._lock:
            self._scan = scan
            self._last_update = time.monotonic()

    def get_scan(self) -> Optional[LidarScanData]:
        """Return the latest fresh scan, otherwise return ``None``."""

        with self._lock:
            scan = self._scan
            last_update = self._last_update

        if scan is None or last_update is None:
            return None
        if time.monotonic() - last_update > self.timeout:
            return None
        if not scan.ranges:
            return None

        return scan

    def has_scan(self) -> bool:
        """Return True after at least one scan has been received."""

        with self._lock:
            return self._scan is not None

    def is_fresh(self) -> bool:
        """Return True while a usable scan is newer than the timeout."""

        return self.get_scan() is not None
