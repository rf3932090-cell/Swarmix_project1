"""Start both decentralized UAV agents."""

import asyncio
import logging

from agent import UAVAgent
from config import LOG_FILE, LOG_LEVEL, UAVS


def configure_logging():
    handlers = [logging.StreamHandler()]
    if LOG_FILE:
        handlers.append(logging.FileHandler(LOG_FILE, encoding="utf-8"))

    level = getattr(logging, LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers,
        force=True,
    )


def build_agents():
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

        agents[uav_id] = UAVAgent(
            uav_id=uav_id,
            config=uav_config,
            peers=peers,
        )

    return agents


async def main():
    configure_logging()
    agents = build_agents()
    print(f"Starting {len(agents)} UAV agents")

    tasks = []
    for uav_id, agent in agents.items():
        neighbor_id = next(
            other_id for other_id in agents if other_id != uav_id
        )
        tasks.append(agent.run(neighbor_id))

    await asyncio.gather(*tasks)
    print("Mission completed")


if __name__ == "__main__":
    asyncio.run(main())

