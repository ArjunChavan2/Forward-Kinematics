"""Rotation and transform helpers for FK. Hand-written by the project owner.

Conventions: quaternions are (w, x, y, z) internally with w the scalar part; the wire format is
{x, y, z, w} and conversion happens at the message boundary. Matrices are 3x3 or 4x4 row-major
lists of floats. Angles are radians.
"""
from __future__ import annotations

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
Matrix = list[list[float]]


def normalize_axis(axis: Vec3) -> Vec3:
    raise NotImplementedError


def axis_angle_to_quat(axis: Vec3, angle: float) -> Quat:
    raise NotImplementedError


def quat_multiply(a: Quat, b: Quat) -> Quat:
    raise NotImplementedError


def quat_normalize(q: Quat) -> Quat:
    raise NotImplementedError


def quat_to_matrix(q: Quat) -> Matrix:
    raise NotImplementedError


def matrix_to_quat(m: Matrix) -> Quat:
    raise NotImplementedError


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    raise NotImplementedError


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    raise NotImplementedError


def make_transform(rotation: Matrix, translation: Vec3) -> Matrix:
    raise NotImplementedError


def matmul(a: Matrix, b: Matrix) -> Matrix:
    raise NotImplementedError


def transform_translation(t: Matrix) -> Vec3:
    raise NotImplementedError


def transform_rotation(t: Matrix) -> Matrix:
    raise NotImplementedError


def transform_to_column_major(t: Matrix) -> list[float]:
    raise NotImplementedError


def transform_to_pose(t: Matrix) -> tuple[Vec3, Quat]:
    raise NotImplementedError


def joint_motion(joint_type: str, axis: Vec3, q: float) -> Matrix:
    raise NotImplementedError
