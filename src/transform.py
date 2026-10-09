"""Rotation and transform helpers for FK. Hand-written by the project owner.

Conventions: quaternions are (w, x, y, z) internally with w the scalar part; the wire format is
{x, y, z, w} and conversion happens at the message boundary. Matrices are 3x3 or 4x4 row-major
lists of floats. Angles are radians.
"""
from __future__ import annotations
from math import sqrt, sin, cos, asin, acos, atan

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
Matrix = list[list[float]]

IDENTITY_MAT_3 = [[1 if i == j else 0 for i in range(3)] for j in range(3)]

def normalize_axis(axis: Vec3) -> Vec3:
    """Return the axis scaled to unit length.

    A zero vector has no direction and raises ZeroDivisionError. Callers reject zero axes on movable
    joints before they reach this function.
    """
    mag = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    return (axis[0] / mag, axis[1] / mag, axis[2] / mag)


def axis_angle_to_quat(axis: Vec3, angle: float) -> Quat:
    """Return the unit quaternion (w, x, y, z) for a rotation of angle radians about axis.

    The axis is normalized first, so any non-zero length is accepted. The quaternion uses the half-angle:
    w = cos(angle / 2) and the vector part is sin(angle / 2) times the unit axis.
    """
    w = cos(angle / 2)
    x = sin(angle / 2) * axis[0]
    y = sin(angle / 2) * axis[1]
    z = sin(angle / 2) * axis[2]
    return (w, x, y, z)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Return the Hamilton product a * b as (w, x, y, z).

    When the quaternions represent rotations, a * b applies b first and then a. The product is not
    commutative, so the order of arguments must follow the order of the rotations.
    """
    w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    y = a[0] * b[2] + a[2] * b[0] - a[1] * b[3] + a[3] * b[1]
    z = a[3] * b[0] + a[0] * b[3] - a[2] * b[1] + a[1] * b[2]
    return (w, x, y, z)


def quat_normalize(q: Quat) -> Quat:
    """Return q scaled to unit length.

    The zero quaternion has no direction and raises ZeroDivisionError.
    """
    mag = sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    return (q[0] / mag, q[1] / mag, q[2] / mag, q[3] / mag)


def quat_to_matrix(q: Quat) -> Matrix:
    """Return the 3x3 rotation matrix for the unit quaternion q = (w, x, y, z).

    The result is a row-major list of lists. The input is assumed to have unit length, so the result is
    orthonormal with determinant +1.
    """
    w, x, y, z = q
    mat = [[0 for i in range(3)] for j in range(3)]
    mat[0][0] = 1 - 2*(y**2 + z**2)
    mat[0][1] = 2*(x*y - z*w)
    mat[0][2] = 2*(x*z + w*y)
    
    mat[1][0] = 2*(x*y + z*w)
    mat[1][1] = 1 - 2*(x**2 + z**2)
    mat[1][2] = 2*(y*z - w*x)
    
    mat[2][0] = 2*(x*z - w*y)
    mat[2][1] = 2*(y*z + w*x)
    mat[2][2] = 1 - 2*(x**2 + y**2)
    
    return mat



def matrix_to_quat(m: Matrix) -> Quat:
    """Return the unit quaternion (w, x, y, z) for the rotation matrix m.

    The branch is chosen from the largest of the trace and the diagonal entries, which keeps the divisions
    well away from zero for every rotation. The sign of the result is not fixed: q and -q are the same
    rotation, so either is correct.
    """
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        w = 0.5 * sqrt(1 + trace)
        x = (m[2][1] - m[1][2]) / (4 * w)
        y = (m[0][2] - m[2][0]) / (4 * w)
        z = (m[1][0] - m[0][1]) / (4 * w)
    else:
        max_diag = max(m[0][0], m[1][1], m[2][2])
        if max_diag == m[0][0]:
            s = 2 * sqrt(1 + m[0][0] - m[1][1] - m[2][2])
            x = s / 4
            w = (m[2][1] - m[1][2]) / (s)
            y = (m[0][1] + m[1][0]) / (s)
            z = (m[0][2] + m[2][0]) / (s)
        elif max_diag == m[1][1]:
            s = 2 * sqrt(1 + m[1][1] - m[0][0] - m[2][2])
            y = s / 4
            w = (m[0][2] - m[2][0]) / (s)
            z = (m[1][2] + m[2][1]) / (s)
            x = (m[0][1] + m[1][0]) / (s)
        else:
            s = 2 * sqrt(1 + m[2][2] - m[1][1] - m[0][0])
            z = s / 4
            w = (m[1][0] - m[0][1]) / (s)
            x = (m[0][2] + m[2][0]) / (s)
            y = (m[1][2] + m[2][1]) / (s)
    return (w, x, y, z)

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """Return R = Rz(yaw) · Ry(pitch) · Rx(roll) as a 3x3 matrix.

    The angles are in radians. The rotations are about the fixed parent axes, applied roll first. The
    order of the product is part of the convention, so changing it changes the result.
    """
    
    mRoll = [[1,            0,               0],
             [0,    cos(roll),      -sin(roll)],
             [0,    sin(roll),       cos(roll)]]
    
    mPitch =[[ cos(pitch),            0,      sin(pitch)],
             [          0,            1,               0],
             [-sin(pitch),            0,       cos(pitch)]]
    
    mYaw =  [[   cos(yaw),    -sin(yaw),               0],
             [   sin(yaw),     cos(yaw),               0],
             [          0,            0,               1]]

    return matmul(matmul(mYaw, mPitch), mRoll)


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    """Return the unit quaternion (w, x, y, z) for R = Rz(yaw) · Ry(pitch) · Rx(roll).

    The quaternion for R is the product of the three axis quaternions in the same order: z, then y, then
    x. It represents the same rotation as rpy_to_matrix for the same angles.
    """
    rq = axis_angle_to_quat((1,0,0), roll)
    pq = axis_angle_to_quat((0,1,0), pitch)
    yq = axis_angle_to_quat((0,0,1), yaw)
    q = quat_multiply((quat_multiply(yq, pq)), rq)
    
    return q


def make_transform(rotation: Matrix, translation: Vec3) -> Matrix:
    """Return the 4x4 homogeneous transform built from a rotation and a translation.

    The rotation fills the top-left 3x3 block, the translation fills the last column, and the bottom row is
    0 0 0 1. The inputs are not modified; the result is a new list of lists.
    """
    t = []
    for i in range(3):
        t.append([])
        for j in range(3):
            t[i].append(rotation[i][j])
    t[0].append(translation[0])
    t[1].append(translation[1])
    t[2].append(translation[2])
    t.append([0,0,0,1])
    return t


def matmul(a: Matrix, b: Matrix) -> Matrix:
    """Return the 3x3 matrix product a · b.

    Entry (i, j) of the result is the sum over k of a[i][k] * b[k][j]. The product is not commutative.
    """
    ret = [[0 for i in range(3)] for j in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                ret[i][j] += a[i][k] * b[k][j]
    return ret


def transform_translation(t: Matrix) -> Vec3:
    """Return the translation (x, y, z) from the last column of the 4x4 transform t.

    The result is a new list, so callers can modify it without changing t.
    """
    return [t[0][3], t[1][3], t[2][3]]


def transform_rotation(t: Matrix) -> Matrix:
    """Return the top-left 3x3 rotation block of the 4x4 transform t as a new matrix.

    Modifying the result does not change t.
    """
    rot = [[0 for i in range(3)] for j in range(3)]
    for i in range(3):
        for j in range(3):
            rot[i][j] = t[i][j]
    return rot


def transform_to_column_major(t: Matrix) -> list[float]:
    """Return the 16 entries of the 4x4 transform t in column-major order.

    Index 4 * c + r holds t[r][c], so the translation occupies indices 12 to 14 and the bottom row
    occupies indices 3, 7, 11, and 15. This is the layout the /xform_world message uses.
    """
    col = [0 for i in range(16)]
    for j in range(4):
        for i in range(4):
            col[j * 4 + i] = t[i][j]
    return col


def transform_to_pose(t: Matrix) -> tuple[Vec3, Quat]:
    """Return (translation, quaternion) for the 4x4 transform t.

    The translation is (x, y, z). The quaternion is (w, x, y, z), which is this module's internal order.
    Convert to the message order (x, y, z, w) at the message boundary.
    """
    return (transform_translation(t), matrix_to_quat(transform_rotation(t)))


def joint_motion(joint_type: str, axis: Vec3, q: float) -> Matrix:
    """Return T_motion(q) for a joint of the given type as a 4x4 transform.

    revolute and continuous rotate by q radians about the axis normalized to unit length. prismatic
    translates by q times the axis, with the axis used as written and not normalized. fixed returns the
    identity. Any other type raises ValueError.
    """
    if joint_type == "revolute" or joint_type == "continuous":
        axis = normalize_axis(axis)
        quat = axis_angle_to_quat(axis, q)
        mq = quat_to_matrix(quat)
        translation = [0,0,0]
        t = make_transform(mq, translation)
    elif joint_type == "prismatic":
        mq = IDENTITY_MAT_3
        translation = [q * axis[0], q * axis[1], q * axis[2]]
        t = make_transform(mq, translation)
    elif joint_type == "fixed":
        mq = IDENTITY_MAT_3
        translation = [0, 0, 0]
        t = make_transform(mq, translation)
    else:
        raise ValueError
    return t
        
