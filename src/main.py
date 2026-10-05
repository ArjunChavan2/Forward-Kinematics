"""Project 3 runtime entry point (`make run`).

Wires the Registry and TCP gateway. Forward-kinematics services are not
registered yet.
"""
from __future__ import annotations

import asyncio
import signal

from gateway import Gateway, log
from registry import Registry


async def run() -> None:
    registry = Registry()
    gateway = Gateway(registry)
    await gateway.start()

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
