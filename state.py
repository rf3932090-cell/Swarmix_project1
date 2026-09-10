"""State shared by telemetry, communication, controller, and agent."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class UAVState:
    """Latest NED position and velocity of one UAV."""

    north: Optional[float] = None
    east: Optional[float] = None
    down: Optional[float] = None

    velocity_north: Optional[float] = None
    velocity_east: Optional[float] = None
    velocity_down: Optional[float] = None

    def has_position(self) -> bool:
        return (
            self.north is not None
            and self.east is not None
            and self.down is not None
        )

    def has_velocity(self) -> bool:
        return (
            self.velocity_north is not None
            and self.velocity_east is not None
            and self.velocity_down is not None
        )

    def is_valid(self) -> bool:
        """Keep the phase-2 meaning: a valid state has a full position."""

        return self.has_position()

    def copy(self) -> "UAVState":
        return UAVState(
            north=self.north,
            east=self.east,
            down=self.down,
            velocity_north=self.velocity_north,
            velocity_east=self.velocity_east,
            velocity_down=self.velocity_down,
        )

    def horizontal_distance_to(self, other: "UAVState") -> Optional[float]:
        if not self.has_position() or not other.has_position():
            return None

        return math.hypot(
            self.north - other.north,
            self.east - other.east,
        )
