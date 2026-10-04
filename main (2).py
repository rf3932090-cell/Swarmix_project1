"""Start and run all decentralized UAV agents."""

import asyncio
import logging

from agent import UAVAgent
from config import LOG_FILE, LOG_LEVEL, UAVS


def configure_logging():
    """Configure console logging and, optionally, file logging."""
    handlers = [logging.StreamHandler()]

    if LOG_FILE:
        handlers.append(logging.FileHandler(LOG_FILE, encoding="utf-8"))

    log_level = getattr(logging, LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers,
        force=True,
    )


def build_agents():
    """Create one UAVAgent per UAV and configure its peer list."""
    agents = {}

    for uav_id, uav_config in UAVS.items():
        peers = [
            {
                "uav_id": peer_id,
                "host": "127.0.0.1",
                "port": peer_config["communication_port"],
            }
            for peer_id, peer_config in UAVS.items()
            if peer_id != uav_id
        ]

        agents[uav_id] = UAVAgent(uav_id=uav_id, config=uav_config, peers=peers,)
    return agents

async def main():
    """Build the agents and run all UAV missions concurrently."""
    configure_logging()
    agents = build_agents()
    print(f"Starting {len(agents)} UAV agents")

    agent_tasks = []
    for uav_id, agent in agents.items():
        neighbor_id = next(
            other_id for other_id in agents if other_id != uav_id
        )
        agent_tasks.append(agent.run(neighbor_id))

    await asyncio.gather(*agent_tasks)
    print("Mission completed")


if __name__ == "__main__":
    asyncio.run(main())


