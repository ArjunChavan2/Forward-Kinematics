"""Forward kinematics over a RobotModel. Hand-written by the project owner.

Each joint contributes T_joint(q) = T_origin · T_motion(q), composed parent first.
"""
from __future__ import annotations

from transform import Matrix
from urdf import RobotModel


def joint_edges(model: RobotModel, positions: dict[str, float]) -> list[tuple[str, str, Matrix]]:
    raise NotImplementedError


def link_transforms(
    root: str,
    edges: list[tuple[str, str, Matrix]],
    root_pose: Matrix,
) -> dict[str, Matrix]:
    raise NotImplementedError


def identity_pose() -> Matrix:
    raise NotImplementedError


def root_of_edges(edges: list[tuple[str, str, Matrix]]) -> str:
    raise NotImplementedError

