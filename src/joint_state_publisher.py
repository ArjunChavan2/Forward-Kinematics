"""joint_state_publisher (make demo only, never make run): holds the robot's joint positions, servos them
toward the latest /joint_trajectory setpoints, and publishes /joint_states at about 10 Hz.

Reads robot_description on its own, like robot_state_publisher. Setpoints are kept by joint name, including
names the loaded robot does not have yet, so a trajectory that arrives just before a new robot still applies.
"""
from __future__ import annotations

import asyncio
import time
from typing import Any, Callable

from param_server import GET_PARAM
from registry import Registry

DESCRIPTION_PARAM = "robot_description"
TRAJECTORY_TOPIC = "/joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"
RESET_SERVICE = "/joint_state_publisher/reset"
INTERVAL_S = 0.1
# The reference servos with gain 50/s, which at 10 Hz is effectively a snap. Capped speed and acceleration
# make motion visible on video instead, and smooth: joints speed up, slow down into a target, and keep
# their speed when a new target arrives mid-move, so a choreography flows from pose to pose.
MAX_SPEED = 12.0  # rad/s or m/s
MAX_ACCEL = 60.0  # rad/s^2 or m/s^2
SUBSTEP_S = 0.01  # integration step inside one publish interval


class JointStatePublisher:
    def __init__(self, registry: Registry, load_robot: Callable[[str], Any]) -> None:
        self.registry = registry
        self.load_robot = load_robot
        self.robot = None
        self.processed_version = 0
        self.positions: dict[str, float] = {}
        self.setpoints: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        registry.subscribe(self, TRAJECTORY_TOPIC)
        registry.register_handler(RESET_SERVICE, self.reset)

    # -- Connection protocol (for registry.subscribe) -----------------------

    def send(self, message: dict) -> None:
        if message.get("topic") != TRAJECTORY_TOPIC:
            return
        msg = message.get("msg") or {}
        points = msg.get("points") or []
        if not points:
            return
        for name, value in zip(msg.get("joint_names") or [], points[-1].get("positions") or []):
            self.setpoints[name] = float(value)

    # -- robot loading ----------------------------------------------------------

    def poll_once(self) -> None:
        ok, values, _ = self.registry.call_service_sync(GET_PARAM, {"name": DESCRIPTION_PARAM})
        if not ok or values["version"] <= self.processed_version:
            return
        self.processed_version = values["version"]
        try:
            robot = self.load_robot(values["value"])
        except Exception:  # robot_state_publisher reports the reason; keep the old robot here
            return
        # A joint the old and new robot share keeps its position, clamped to the new limits.
        self.robot = robot
        self.positions = {j.name: self._clamp(j, self.positions.get(j.name, 0.0)) for j in self._movable()}

    def _movable(self) -> list:
        return [j for j in self.robot.joints if j.joint_type != "fixed"] if self.robot else []

    @staticmethod
    def _clamp(joint, value: float) -> float:
        if joint.limit is None:  # continuous
            return value
        return min(max(value, joint.limit.lower), joint.limit.upper)

    # -- servo and publish ------------------------------------------------------

    def step(self, dt: float) -> None:
        while dt > 1e-9:
            h = min(SUBSTEP_S, dt)
            dt -= h
            for joint in self._movable():
                self._substep(joint, h)

    def _substep(self, joint, h: float) -> None:
        current = self.positions[joint.name]
        error = self._clamp(joint, self.setpoints.get(joint.name, current)) - current
        # fastest speed that can still stop at the target, then change speed by at most MAX_ACCEL * h
        wanted = min(MAX_SPEED, (2 * MAX_ACCEL * abs(error)) ** 0.5) * (1 if error > 0 else -1)
        v = self.velocities.get(joint.name, 0.0)
        v += max(-MAX_ACCEL * h, min(MAX_ACCEL * h, wanted - v))
        if abs(v * h) >= abs(error) and v * error >= 0:  # reaches the target this substep
            self.positions[joint.name], self.velocities[joint.name] = current + error, 0.0
        else:
            self.positions[joint.name], self.velocities[joint.name] = current + v * h, v

    def publish(self) -> None:
        if self.robot is None:
            return
        now = time.time()
        self.registry.publish(JOINT_STATES_TOPIC, {
            "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": ""},
            "name": list(self.positions),
            "position": list(self.positions.values()),
            "velocity": [],
            "effort": [],
        })

    def reset(self, args: Any) -> tuple[bool, dict, str]:
        """Setpoint every joint to the middle of its limits (0 for continuous joints)."""
        for joint in self._movable():
            self.setpoints[joint.name] = 0.0 if joint.limit is None else (joint.limit.lower + joint.limit.upper) / 2
        return True, {}, ""

    async def run(self) -> None:
        while True:
            self.poll_once()
            self.step(INTERVAL_S)
            self.publish()
            await asyncio.sleep(INTERVAL_S)
