"""Description handling for robot_state_publisher: polls robot_description, processes each new version, and
reports /robot_state_publisher/description_status as a parameter.

The URDF loader is injected. It takes the description text and returns an object with a `root_link`
attribute, or raises ValueError with the reason the description is rejected.
"""
from __future__ import annotations

import asyncio
from typing import Any, Callable, Protocol

from param_server import GET_PARAM, SET_PARAM
from registry import Registry

DESCRIPTION_PARAM = "robot_description"
STATUS_PARAM = "/robot_state_publisher/description_status"
POLL_INTERVAL_S = 0.1


class LoadedRobot(Protocol):
    root_link: str


LoadRobot = Callable[[str], LoadedRobot]


class RobotStatePublisher:
    def __init__(self, registry: Registry, load_robot: LoadRobot) -> None:
        self.registry = registry
        self.load_robot = load_robot
        self.robot: LoadedRobot | None = None
        self.processed_version = 0
        self.loaded_version = 0
        self.root_link = ""

    def poll_once(self) -> bool:
        ok, values, _ = self.registry.call_service_sync(GET_PARAM, {"name": DESCRIPTION_PARAM})
        if not ok:
            return False
        version = values["version"]
        if version <= self.processed_version:
            return False
        self._process(values["value"], version)
        return True

    def _process(self, value: Any, version: int) -> None:
        accepted, error = True, ""
        robot = None
        if not isinstance(value, str):
            accepted, error = False, "robot_description must be a JSON string"
        else:
            try:
                robot = self.load_robot(value)
            except ValueError as exc:
                accepted, error = False, str(exc) or "invalid robot_description"
            except Exception as exc:  # a loader bug must not crash the node
                accepted, error = False, f"internal error: {exc}"

        if accepted:
            self.robot = robot
            self.loaded_version = version
            self.root_link = robot.root_link

        self.processed_version = version
        self.registry.call_service_sync(SET_PARAM, {
            "name": STATUS_PARAM,
            "value": {
                "version": version,
                "accepted": accepted,
                "error": error,
                "loaded_version": self.loaded_version,
                "root_link": self.root_link,
            },
        })

    async def run(self) -> None:
        while True:
            self.poll_once()
            await asyncio.sleep(POLL_INTERVAL_S)
