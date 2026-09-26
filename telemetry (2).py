"""Small MAVSDK telemetry helpers for one UAV."""

import asyncio

from state import UAVState


def update_state(data, state: UAVState, origin_north: 0.0, origin_east: float = 0.0) -> None:
    """Copy one MAVSDK PositionVelocityNed message into ``state``."""

    position = data.position
    velocity = data.velocity

    state.north = position.north_m + origin_north
    state.east = position.east_m + origin_east
    state.down = position.down_m

    state.velocity_north = velocity.north_m_s
    state.velocity_east = velocity.east_m_s
    state.velocity_down = velocity.down_m_s


async def telemetry_loop(drone, state: UAVState, name: str, origin_north: float = 0.0, origin_east = 0.0) -> None:
    """Continuously keep the local state of one UAV up to date."""

    try:
        async for data in drone.telemetry.position_velocity_ned():
            update_state(data, state, origin_north, origin_east)

    except asyncio.CancelledError:
        raise

    except Exception as error:
        print(f"{name}: telemetry error -> {error}")


async def get_current_position(drone, state: UAVState):
    """Wait for the first sample, update ``state``, and return its position."""

    async for data in drone.telemetry.position_velocity_ned():
        update_state(data, state)
        return state.north, state.east, state.down

    return None

