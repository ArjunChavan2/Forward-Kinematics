import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from transform import (  # noqa: E402
    axis_angle_to_quat,
    joint_motion,
    make_transform,
    matrix_to_quat,
    quat_to_matrix,
    rpy_to_matrix,
    transform_to_column_major,
    transform_to_pose,
)

TOL = 1e-9


def rotation(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    return quat_to_matrix(axis_angle_to_quat(tuple(a / n for a in axis), angle))


class TransformTestCase(unittest.TestCase):
    def assertMatAlmostEqual(self, actual, expected) -> None:
        for r in range(len(expected)):
            for c in range(len(expected[0])):
                self.assertAlmostEqual(actual[r][c], expected[r][c], delta=TOL, msg=f"[{r}][{c}]")


class MatrixToQuatTest(TransformTestCase):
    def assertRoundTrips(self, m) -> None:
        q = matrix_to_quat(m)
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in q)), 1.0, delta=TOL)
        self.assertMatAlmostEqual(quat_to_matrix(q), m)

    # One case per branch: trace > 0, then trace <= 0 with the largest diagonal entry at x, y, or z.
    def test_trace_positive_branch(self) -> None:
        self.assertRoundTrips(rotation((0, 0, 1), math.pi / 2))

    def test_x_diagonal_branch(self) -> None:
        self.assertRoundTrips(rotation((1, 0, 0), math.pi))
        self.assertRoundTrips(rotation((1, 0.2, 0.1), 2.8))

    def test_y_diagonal_branch(self) -> None:
        self.assertRoundTrips(rotation((0, 1, 0), math.pi))
        self.assertRoundTrips(rotation((0.1, 1, 0.2), 2.8))

    def test_z_diagonal_branch(self) -> None:
        self.assertRoundTrips(rotation((0, 0, 1), math.pi))
        self.assertRoundTrips(rotation((0.2, 0.1, 1), 2.8))

    def test_random_rotations(self) -> None:
        rng = random.Random(0)
        for _ in range(2000):
            axis = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
            self.assertRoundTrips(rotation(axis, rng.uniform(-math.pi, math.pi)))


class RpyTest(TransformTestCase):
    def test_rpy_is_rz_ry_rx(self) -> None:
        # R = Rz(yaw) · Ry(pitch) · Rx(roll), composed independently here from axis-angle rotations.
        roll, pitch, yaw = 0.3, -0.7, 1.9
        rz, ry, rx = rotation((0, 0, 1), yaw), rotation((0, 1, 0), pitch), rotation((1, 0, 0), roll)
        expected = [[sum(rz[i][k] * sum(ry[k][m] * rx[m][j] for m in range(3)) for k in range(3))
                     for j in range(3)] for i in range(3)]
        self.assertMatAlmostEqual(rpy_to_matrix(roll, pitch, yaw), expected)


class JointMotionTest(TransformTestCase):
    def test_revolute_normalizes_axis(self) -> None:
        self.assertMatAlmostEqual(joint_motion("revolute", (0, 0, 2), 0.5),
                                  make_transform(rotation((0, 0, 1), 0.5), [0, 0, 0]))

    def test_prismatic_uses_axis_as_written(self) -> None:
        # Spec example: axis 2 0 0 at q = 0.7 moves 1.4 m.
        t = joint_motion("prismatic", (2, 0, 0), 0.7)
        self.assertAlmostEqual(t[0][3], 1.4, delta=TOL)

    def test_fixed_is_identity(self) -> None:
        self.assertMatAlmostEqual(joint_motion("fixed", (0, 0, 0), 5.0), make_transform(rotation((1, 0, 0), 0), [0, 0, 0]))

    def test_unknown_type_raises(self) -> None:
        with self.assertRaises(ValueError):
            joint_motion("floating", (1, 0, 0), 0.0)


class LayoutTest(TransformTestCase):
    def test_column_major_puts_translation_at_12_to_14(self) -> None:
        flat = transform_to_column_major(make_transform(rotation((0, 0, 1), 0.4), [1.0, 2.0, 3.0]))
        self.assertEqual(flat[12:15], [1.0, 2.0, 3.0])
        self.assertEqual([flat[3], flat[7], flat[11], flat[15]], [0, 0, 0, 1])
        self.assertAlmostEqual(flat[1], math.sin(0.4), delta=TOL)  # M[1][0]

    def test_transform_to_pose(self) -> None:
        translation, (w, x, y, z) = transform_to_pose(make_transform(rotation((0, 0, 1), math.pi / 2), [1, 2, 3]))
        self.assertEqual(translation, [1, 2, 3])
        sign = 1 if w > 0 else -1
        for got, want in zip((sign * w, sign * x, sign * y, sign * z), (math.sqrt(0.5), 0, 0, math.sqrt(0.5))):
            self.assertAlmostEqual(got, want, delta=TOL)


if __name__ == "__main__":
    unittest.main()
