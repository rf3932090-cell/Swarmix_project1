"""State container shared by telemetry, communication, controller, and agent."""
import math
from dataclasses import dataclass
from typing import Optional

@dataclass
class UAVState:
    """Store Latest NED position and velocity of one UAV."""

    # Position in the shared NED frame.
    north: Optional[float] = None
    east: Optional[float] = None
    down: Optional[float] = None

    # Latest velocity reported by MAVSDK.
    velocity_north: Optional[float] = None
    velocity_east: Optional[float] = None
    velocity_down: Optional[float] = None

    def has_position(self) -> bool:
        """Return True when all three position components are available."""
        return (
            self.north is not None
            and self.east is not None
            and self.down is not None
        )

    def has_velocity(self) -> bool:
        """"Return True when all three velocity components are available."""

        return (
            self.velocity_north is not None
            and self.velocity_east is not None
            and self.velocity_down is not None
        )

    def is_valid(self) -> bool:
        return self.has_position()

    def copy(self) -> "UAVState":
        """ Return an indepandent copy of the current state."""

        return UAVState(
            north=self.north,
            east=self.east,
            down=self.down,
            velocity_north=self.velocity_north,
            velocity_east=self.velocity_east,
            velocity_down=self.velocity_down,
        )

    def horizontal_distance_to(self, other: "UAVState") -> Optional[float]:
        """Return horizontal N-E distance, or None if either position is incomplete."""
        
        if not self.has_position() or not other.has_position():
            return None

        return math.hypot(
            self.north - other.north,
            self.east - other.east,
        )
