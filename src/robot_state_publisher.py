"""robot_state_publisher: polls robot_description, processes each new version, reports
/robot_state_publisher/description_status, subscribes /joint_states, and publishes /tf.

The URDF loader is injected. It takes the description text and returns an object with `root_link`
and `joints` (each with `name` and `joint_type`) attributes, or raises ValueError with the reason
the description is rejected. fk.joint_edges/fk computation is also used, but not injected: it is
read from fk.py directly, since that module is the hand-written forward-kinematics contract.
"""
from __future__ import annotations

import asyncio
import math
from typing import Any, Callable, Protocol

import fk
from param_server import GET_PARAM, SET_PARAM
from registry import Registry
from transform import transform_to_pose

DESCRIPTION_PARAM = "robot_description"
STATUS_PARAM = "/robot_state_publisher/description_status"
JOINT_STATES_TOPIC = "/joint_states"
TF_TOPIC = "/tf"
POLL_INTERVAL_S = 0.1
TF_INTERVAL_S = 0.1


class JointInfo(Protocol):
    name: str
    joint_type: str


class LoadedRobot(Protocol):
    root_link: str
    joints: list[JointInfo]


LoadRobot = Callable[[str], LoadedRobot]

ZERO_STAMP = {"sec": 0, "nanosec": 0}


def _all_finite(edges: list) -> bool:
    return all(math.isfinite(v) for _, _, matrix in edges for row in matrix for v in row)


class RobotStatePublisher:
    def __init__(self, registry: Registry, load_robot: LoadRobot) -> None:
        self.registry = registry
        self.load_robot = load_robot
        self.robot: LoadedRobot | None = None
        self.processed_version = 0
        self.loaded_version = 0
        self.root_link = ""
        self.stamp = ZERO_STAMP
        self.positions: dict[str, float] = {}
        self._last_names: list[str] = []
        self._last_positions: list[float] = []
        registry.subscribe(self, JOINT_STATES_TOPIC)

    # -- Connection protocol (for registry.subscribe) -----------------------

    def send(self, message: dict) -> None:
        if message.get("topic") == JOINT_STATES_TOPIC:
            self._on_joint_states(message["msg"])

    def _on_joint_states(self, msg: dict) -> None:
        names, positions = msg.get("name") or [], msg.get("position") or []
        resolved = self._positions_for(names, positions)
        # Huge positions (1e308) overflow to inf/NaN, which /tf must never carry: ignore that whole message.
        if self.robot is not None and not _all_finite(fk.joint_edges(self.robot, resolved)):
            return
        self._last_names, self._last_positions = names, positions
        self.stamp = msg.get("header", {}).get("stamp", self.stamp)
        self.positions = resolved

    def _resolve_positions(self) -> None:
        self.positions = self._positions_for(self._last_names, self._last_positions)

    def _positions_for(self, names: list, positions: list) -> dict[str, float]:
        """Positions for the current robot's movable joints: named value if given, 0 otherwise
        (spec: "a movable joint the message does not name is at 0"). Unknown names and fixed joints
        are ignored, since we only iterate the robot's own movable joints."""
        if self.robot is None:
            return {}
        given = dict(zip(names, positions))
        return {joint.name: float(given.get(joint.name, 0.0))
                for joint in self.robot.joints if joint.joint_type != "fixed"}

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
            self._resolve_positions()

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

    # -- /tf publishing -------------------------------------------------------

    def publish_tf(self) -> None:
        """Publish one /tf message: one entry per joint (fixed included, root excluded), at the
        current joint positions. No-op while no robot is loaded (spec: never publish /tf then)."""
        if self.robot is None:
            return
        edges = fk.joint_edges(self.robot, self.positions)
        transforms = []
        for parent, child, matrix in edges:
            translation, (w, x, y, z) = transform_to_pose(matrix)
            transforms.append({
                "header": {"stamp": self.stamp, "frame_id": parent},
                "child_frame_id": child,
                "transform": {
                    "translation": {"x": translation[0], "y": translation[1], "z": translation[2]},
                    "rotation": {"x": x, "y": y, "z": z, "w": w},
                },
            })
        self.registry.publish(TF_TOPIC, {"transforms": transforms})

    async def publish_tf_loop(self) -> None:
        # Sleep only until the next tick's deadline: sleeping a full interval after the work made the period
        # work + interval, which fell under 5 Hz on ~1 MiB descriptions.
        loop = asyncio.get_running_loop()
        deadline = loop.time()
        while True:
            self.publish_tf()
            deadline = max(deadline + TF_INTERVAL_S, loop.time())  # running late: don't burst to catch up
            await asyncio.sleep(deadline - loop.time())
