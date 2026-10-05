"""URDF parsing and validation. Hand-written by the project owner.

parse_urdf takes the robot_description text and returns a RobotModel, or raises ValueError with the reason
the description is rejected (the reason becomes the description_status error).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from transform import Vec3


@dataclass
class Limit:
    lower: float
    upper: float


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: Vec3
    origin_rpy: Vec3
    axis: Vec3
    limit: Limit | None = None


@dataclass
class Link:
    name: str


@dataclass
class RobotModel:
    root_link: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)


def parse_urdf(text: str) -> RobotModel:
    raise NotImplementedError
