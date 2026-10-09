"""TEMPORARY forward kinematics for the 3D demo viewer only.

This is NOT src/fk.py and is not meant to be graded work. The owner will write their own fk.py
later; this file exists only so tools/urdf_viewer_3d.py has poses to render in the meantime.
Delete this file once src/fk.py is built, and point the viewer at that instead.

Uses the already-implemented, tested transform.py (make_transform, matmul, joint_motion,
rpy_to_matrix) for the actual math. The only new code here is the tree traversal and a 4x4
matrix multiply (transform.matmul is 3x3-only).
"""
from __future__ import annotations

from transform import Matrix, joint_motion, make_transform, rpy_to_matrix
from urdf import RobotModel

IDENTITY_4: Matrix = [[1.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]]


def _matmul4(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def compute_link_poses(model: RobotModel, positions: dict[str, float] | None = None) -> dict[str, Matrix]:
    """Return {link_name: 4x4 world transform} for every link, at the given joint positions
    (default: all zero, i.e. "zero-configuration FK")."""
    positions = positions or {}
    joints_by_parent: dict[str, list] = {}
    for joint in model.joints:
        joints_by_parent.setdefault(joint.parent, []).append(joint)

    poses: dict[str, Matrix] = {}

    def visit(link_name: str, parent_pose: Matrix) -> None:
        poses[link_name] = parent_pose
        for joint in joints_by_parent.get(link_name, []):
            q = positions.get(joint.name, 0.0)
            origin = make_transform(rpy_to_matrix(*joint.origin_rpy), list(joint.origin_xyz))
            motion = joint_motion(joint.joint_type, joint.axis, q)
            t_joint = _matmul4(origin, motion)
            visit(joint.child, _matmul4(parent_pose, t_joint))

    visit(model.root_link, IDENTITY_4)
    return poses
