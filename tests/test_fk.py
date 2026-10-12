import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import fk  # noqa: E402
from transform import make_transform, rpy_to_matrix  # noqa: E402
from urdf import parse_urdf  # noqa: E402

TOL = 1e-6
S = math.sqrt(0.5)

IDENTITY_4 = [[1.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0]]

# base_link -j1 (revolute z, up 1)-> link1 -j2 (fixed, +x 1)-> link2 -j3 (prismatic, axis 0 0 2)-> link3.
# The prismatic axis is deliberately not unit length: it must be used as written.
CHAIN_URDF = """<robot name="chain">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="j3" type="prismatic">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 2"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"""


def translation(t):
    return [t[0][3], t[1][3], t[2][3]]


def rz(theta):
    return make_transform(rpy_to_matrix(0.0, 0.0, theta), [0.0, 0.0, 0.0])


class FkTestCase(unittest.TestCase):
    def assertMatrixAlmostEqual(self, actual, expected) -> None:
        self.assertEqual(len(actual), 4)
        for r in range(4):
            for c in range(4):
                self.assertAlmostEqual(actual[r][c], expected[r][c], delta=TOL, msg=f"[{r}][{c}] of {actual}")

    def assertVecAlmostEqual(self, actual, expected) -> None:
        for a, e in zip(actual, expected, strict=True):
            self.assertAlmostEqual(a, e, delta=TOL, msg=f"{actual} != {expected}")


class IdentityPoseTest(FkTestCase):
    def test_is_4x4_identity(self) -> None:
        self.assertMatrixAlmostEqual(fk.identity_pose(), IDENTITY_4)


class JointEdgesTest(FkTestCase):
    def setUp(self) -> None:
        self.model = parse_urdf(CHAIN_URDF)

    def test_one_edge_per_joint_in_model_order_including_fixed(self) -> None:
        edges = fk.joint_edges(self.model, {})
        self.assertEqual([(p, c) for p, c, _ in edges],
                         [("base_link", "link1"), ("link1", "link2"), ("link2", "link3")])

    def test_zero_configuration_is_just_the_origins(self) -> None:
        edges = fk.joint_edges(self.model, {})
        expected = [[0, 0, 1], [1, 0, 0], [0, 0, 0]]
        for (_, _, t), xyz in zip(edges, expected):
            self.assertMatrixAlmostEqual(t, make_transform(rpy_to_matrix(0, 0, 0), xyz))

    def test_revolute_rotates_about_axis_after_origin(self) -> None:
        _, _, t = fk.joint_edges(self.model, {"j1": math.pi / 2})[0]
        expected = rz(math.pi / 2)
        expected[2][3] = 1.0
        self.assertMatrixAlmostEqual(t, expected)

    def test_prismatic_axis_is_not_normalized(self) -> None:
        _, _, t = fk.joint_edges(self.model, {"j3": 0.5})[2]
        self.assertVecAlmostEqual(translation(t), [0, 0, 1.0])

    def test_missing_joint_is_at_zero_and_no_clamping(self) -> None:
        # j3's limit is [-1, 1]; 5.0 must be used as given.
        edges = fk.joint_edges(self.model, {"j3": 5.0})
        self.assertMatrixAlmostEqual(edges[0][2], fk.joint_edges(self.model, {})[0][2])
        self.assertVecAlmostEqual(translation(edges[2][2]), [0, 0, 10.0])

    def test_origin_is_applied_before_motion(self) -> None:
        # T_origin · T_motion: origin moves +x 1 and yaws 90°, then the joint slides along its own x,
        # which is world +y. The reversed product would give (2, 0, 0) instead.
        model = parse_urdf("""<robot name="r">
          <link name="a"/><link name="b"/>
          <joint name="p" type="prismatic">
            <parent link="a"/><child link="b"/>
            <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>""")
        _, _, t = fk.joint_edges(model, {"p": 1.0})[0]
        self.assertVecAlmostEqual(translation(t), [1, 1, 0])

    def test_robot_with_no_joints_has_no_edges(self) -> None:
        model = parse_urdf('<robot name="r"><link name="only"/></robot>')
        self.assertEqual(fk.joint_edges(model, {}), [])


class LinkTransformsTest(FkTestCase):
    def setUp(self) -> None:
        self.model = parse_urdf(CHAIN_URDF)

    def test_zero_configuration_chain(self) -> None:
        poses = fk.link_transforms("base_link", fk.joint_edges(self.model, {}), fk.identity_pose())
        self.assertEqual(set(poses), {"base_link", "link1", "link2", "link3"})
        self.assertMatrixAlmostEqual(poses["base_link"], IDENTITY_4)
        self.assertVecAlmostEqual(translation(poses["link1"]), [0, 0, 1])
        self.assertVecAlmostEqual(translation(poses["link2"]), [1, 0, 1])
        self.assertVecAlmostEqual(translation(poses["link3"]), [1, 0, 1])

    def test_moved_chain_composes_parent_first(self) -> None:
        # j1 yaws 90°, so link2's +x offset becomes world +y; j3 then lifts 0.5 * 2 = 1.
        edges = fk.joint_edges(self.model, {"j1": math.pi / 2, "j3": 0.5})
        poses = fk.link_transforms("base_link", edges, fk.identity_pose())
        self.assertVecAlmostEqual(translation(poses["link2"]), [0, 1, 1])
        self.assertVecAlmostEqual(translation(poses["link3"]), [0, 1, 2])
        expected = rz(math.pi / 2)
        expected[1][3], expected[2][3] = 1.0, 2.0
        self.assertMatrixAlmostEqual(poses["link3"], expected)

    def test_root_pose_premultiplies_everything(self) -> None:
        root_pose = rz(math.pi / 2)
        root_pose[0][3] = 10.0
        edges = fk.joint_edges(self.model, {"j1": math.pi / 2, "j3": 0.5})
        poses = fk.link_transforms("base_link", edges, root_pose)
        self.assertMatrixAlmostEqual(poses["base_link"], root_pose)
        # Rz(90°) · (0, 1, 2) + (10, 0, 0) = (9, 0, 2)
        self.assertVecAlmostEqual(translation(poses["link3"]), [9, 0, 2])

    def test_edge_order_does_not_matter(self) -> None:
        # URDF joints (and so /tf entries) need not be listed parent-first.
        edges = fk.joint_edges(self.model, {"j1": 0.3, "j3": 0.2})
        forward = fk.link_transforms("base_link", edges, fk.identity_pose())
        backward = fk.link_transforms("base_link", list(reversed(edges)), fk.identity_pose())
        for link in forward:
            self.assertMatrixAlmostEqual(backward[link], forward[link])

    def test_branching_tree(self) -> None:
        up = make_transform(rpy_to_matrix(0, 0, 0), [0, 0, 1])
        left = make_transform(rpy_to_matrix(0, 0, 0), [0, 1, 0])
        poses = fk.link_transforms("root", [("root", "a", up), ("root", "b", left), ("a", "c", left)],
                                   fk.identity_pose())
        self.assertVecAlmostEqual(translation(poses["a"]), [0, 0, 1])
        self.assertVecAlmostEqual(translation(poses["b"]), [0, 1, 0])
        self.assertVecAlmostEqual(translation(poses["c"]), [0, 1, 1])

    def test_loop_in_edges_terminates(self) -> None:
        # Wire /tf data could hold a loop beside the root; every link still gets exactly one pose.
        up = make_transform(rpy_to_matrix(0, 0, 0), [0, 0, 1])
        poses = fk.link_transforms("a", [("a", "b", up), ("b", "c", up), ("c", "b", up)], fk.identity_pose())
        self.assertVecAlmostEqual(translation(poses["b"]), [0, 0, 1])
        self.assertVecAlmostEqual(translation(poses["c"]), [0, 0, 2])

    def test_no_edges_gives_only_the_root(self) -> None:
        poses = fk.link_transforms("only", [], fk.identity_pose())
        self.assertEqual(list(poses), ["only"])


class RootOfEdgesTest(FkTestCase):
    def assertRejects(self, edges) -> None:
        with self.assertRaises(Exception) as ctx:
            fk.root_of_edges(edges)
        self.assertNotIsInstance(ctx.exception, NotImplementedError)

    def test_finds_the_root_regardless_of_order(self) -> None:
        edges = fk.joint_edges(parse_urdf(CHAIN_URDF), {})
        self.assertEqual(fk.root_of_edges(edges), "base_link")
        self.assertEqual(fk.root_of_edges(list(reversed(edges))), "base_link")

    def test_root_with_several_children_is_still_one_root(self) -> None:
        # Real robots branch at the root (e.g. PR2's base_link), so the root appears as parent more than once.
        edges = [("base", "left", IDENTITY_4), ("base", "right", IDENTITY_4), ("left", "hand", IDENTITY_4)]
        self.assertEqual(fk.root_of_edges(edges), "base")

    def test_empty_edges_raise(self) -> None:
        self.assertRejects([])

    def test_two_roots_raise(self) -> None:
        self.assertRejects([("a", "x", IDENTITY_4), ("b", "y", IDENTITY_4)])

    def test_cycle_with_no_root_raises(self) -> None:
        self.assertRejects([("a", "b", IDENTITY_4), ("b", "a", IDENTITY_4)])


if __name__ == "__main__":
    unittest.main()
