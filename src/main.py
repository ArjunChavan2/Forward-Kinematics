"""Project 3 runtime entry point (`make run`).

Wires the Registry, the TCP gateway, param_server, robot_state_publisher, and
robot_world_state_publisher -- the three required nodes.
"""
from __future__ import annotations

import asyncio
import signal

import param_server
import robot_state_publisher
import robot_world_state_publisher
import urdf
from gateway import Gateway, log
from registry import Registry


async def run() -> None:
    registry = Registry()
    param_server.register(registry)
    rsp = robot_state_publisher.RobotStatePublisher(registry, load_robot=urdf.parse_urdf)
    robot_world_state_publisher.register(registry)
    gateway = Gateway(registry)
    await gateway.start()

    description_task = asyncio.create_task(rsp.run())
    tf_task = asyncio.create_task(rsp.publish_tf_loop())

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()

    for task in (description_task, tf_task):
        task.cancel()
    await asyncio.gather(description_task, tf_task, return_exceptions=True)


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
