"""Project 3 runtime entry point (`make run`, and `make demo` with --demo).

Wires the Registry, the TCP gateway, param_server, robot_state_publisher, and
robot_world_state_publisher -- the three required nodes. --demo also starts
joint_state_publisher and finite_state_machine, which make run must never start.
"""
from __future__ import annotations

import asyncio
import signal
import sys

import finite_state_machine
import joint_state_publisher
import param_server
import robot_state_publisher
import robot_world_state_publisher
import urdf
from gateway import Gateway, log
from registry import Registry


async def run(demo: bool = False) -> None:
    registry = Registry()
    param_server.register(registry)
    rsp = robot_state_publisher.RobotStatePublisher(registry, load_robot=urdf.parse_urdf)
    robot_world_state_publisher.register(registry)
    gateway = Gateway(registry)
    await gateway.start()

    description_task = asyncio.create_task(rsp.run())
    tf_task = asyncio.create_task(rsp.publish_tf_loop())
    tasks = [description_task, tf_task]
    if demo:
        jsp = joint_state_publisher.JointStatePublisher(registry, load_robot=urdf.parse_urdf)
        fsm = finite_state_machine.FiniteStateMachine(registry)
        tasks += [asyncio.create_task(jsp.run()), asyncio.create_task(fsm.run())]

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()

    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def main() -> None:
    asyncio.run(run(demo="--demo" in sys.argv[1:]))


if __name__ == "__main__":
    main()
