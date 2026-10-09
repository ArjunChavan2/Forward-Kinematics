import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from urdf import (  # noqa: E402
    Limit,
    _parse_joint,
    _parse_link,
    _parse_number,
    _parse_root,
    _parse_vector,
    _parse_visual,
    _validate_robot,
    parse_urdf,
)
import xml.etree.ElementTree as ET  # noqa: E402


def elem(xml_text: str) -> ET.Element:
    return ET.fromstring(xml_text)


class ParseRootTest(unittest.TestCase):
    def test_valid_robot_returns_root_element(self) -> None:
        root = _parse_root('<robot name="r"><link name="a"/></robot>')
        self.assertEqual(root.tag, "robot")

    def test_leading_whitespace_before_declaration_is_accepted(self) -> None:
        text = '\n<?xml version="1.0"?>\n<robot><link name="a"/></robot>'
        self.assertEqual(_parse_root(text).tag, "robot")

    def test_empty_and_whitespace_only_text_rejected(self) -> None:
        for bad in ("", "   ", "\t\n\t"):
            with self.assertRaises(ValueError):
                _parse_root(bad)

    def test_malformed_xml_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_root("not xml")

    def test_wrong_root_tag_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_root("<arm/>")


class NumberAndVectorTest(unittest.TestCase):
    def test_valid_number_forms(self) -> None:
        for token, value in [("1", 1.0), ("-0.5", -0.5), (".25", 0.25), ("3.", 3.0),
                              ("+1", 1.0), ("1.5e-1", 0.15), ("-2E+0", -2.0)]:
            self.assertAlmostEqual(_parse_number(token), value)

    def test_underscore_and_non_numbers_rejected(self) -> None:
        for bad in ("1_000", "two", "1,5", "nan", "inf"):
            with self.assertRaises(ValueError):
                _parse_number(bad)

    def test_vector_parses_three_numbers(self) -> None:
        self.assertEqual(_parse_vector("1 -0.5 .25"), (1.0, -0.5, 0.25))

    def test_vector_wrong_count_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_vector("1 2")


class ParseLinkTest(unittest.TestCase):
    def test_valid_link(self) -> None:
        self.assertEqual(_parse_link(elem('<link name="base"/>')).name, "base")

    def test_missing_name_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_link(elem("<link/>"))


class ParseJointTest(unittest.TestCase):
    def test_full_joint(self) -> None:
        j = _parse_joint(elem(
            '<joint name="j1" type="revolute">'
            '<parent link="a"/><child link="b"/>'
            '<origin xyz="0 0 1" rpy="0 0 0"/><axis xyz="0 0 2"/>'
            '<limit lower="-1" upper="1"/></joint>'
        ))
        self.assertEqual((j.name, j.joint_type, j.parent, j.child), ("j1", "revolute", "a", "b"))
        self.assertEqual(j.origin_xyz, (0.0, 0.0, 1.0))
        self.assertEqual(j.axis, (0.0, 0.0, 2.0))
        self.assertEqual(j.limit, Limit(-1.0, 1.0))

    def test_defaults_for_missing_origin_and_axis(self) -> None:
        j = _parse_joint(elem('<joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>'))
        self.assertEqual(j.origin_xyz, (0, 0, 0))
        self.assertEqual(j.origin_rpy, (0, 0, 0))
        self.assertEqual(j.axis, (1, 0, 0))
        self.assertIsNone(j.limit)

    def test_unknown_type_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_joint(elem('<joint name="j" type="floating"><parent link="a"/><child link="b"/></joint>'))

    def test_missing_parent_or_child_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_joint(elem('<joint name="j" type="fixed"><child link="b"/></joint>'))
        with self.assertRaises(ValueError):
            _parse_joint(elem('<joint name="j" type="fixed"><parent link="a"/></joint>'))

    def test_revolute_without_limit_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _parse_joint(elem('<joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>'))

    def test_continuous_ignores_malformed_limit(self) -> None:
        j = _parse_joint(elem(
            '<joint name="j" type="continuous"><parent link="a"/><child link="b"/>'
            '<limit lower="notanumber" upper="1"/></joint>'
        ))
        self.assertIsNone(j.limit)

    def test_fixed_joint_accepts_zero_axis(self) -> None:
        j = _parse_joint(elem(
            '<joint name="j" type="fixed"><parent link="a"/><child link="b"/><axis xyz="0 0 0"/></joint>'
        ))
        self.assertEqual(j.axis, (0.0, 0.0, 0.0))


class ParseVisualTest(unittest.TestCase):
    def test_no_visual_is_fine(self) -> None:
        _parse_visual(elem('<link name="a"/>'))

    def test_each_valid_shape(self) -> None:
        shapes = [
            '<box size="1 2 3"/>',
            '<cylinder radius="0.1" length="1"/>',
            '<sphere radius="0.2"/>',
            '<mesh filename="x.dae"/>',
            '<mesh filename="x.dae" scale="1 1 1"/>',
        ]
        for shape in shapes:
            _parse_visual(elem(f'<link name="a"><visual><geometry>{shape}</geometry></visual></link>'))

    def test_missing_required_attribute_rejected(self) -> None:
        for shape in ('<box/>', '<cylinder length="1"/>', '<sphere/>', '<mesh/>'):
            with self.assertRaises(ValueError):
                _parse_visual(elem(f'<link name="a"><visual><geometry>{shape}</geometry></visual></link>'))

    def test_missing_or_empty_or_unknown_geometry_rejected(self) -> None:
        for body in ('<visual></visual>', '<visual><geometry/></visual>',
                     '<visual><geometry><capsule radius="1" length="1"/></geometry></visual>'):
            with self.assertRaises(ValueError):
                _parse_visual(elem(f'<link name="a">{body}</link>'))

    def test_first_shape_wins_later_shapes_not_checked(self) -> None:
        _parse_visual(elem(
            '<link name="a"><visual><geometry>'
            '<box size="1 2 3"/><cylinder radius="bad" length="bad"/>'
            '</geometry></visual></link>'
        ))


class ValidateRobotAndParseUrdfTest(unittest.TestCase):
    def test_valid_robot_with_nested_gazebo_joint_ignored(self) -> None:
        model = parse_urdf(
            '<robot name="demo">'
            '<link name="base"/>'
            '<joint name="j1" type="revolute"><parent link="base"/><child link="arm"/>'
            '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
            '<link name="arm"/>'
            '<gazebo reference="arm"><joint name="ghost" type="fixed"/></gazebo>'
            "</robot>"
        )
        self.assertEqual(model.root_link, "base")
        self.assertEqual([l.name for l in model.links], ["base", "arm"])
        self.assertEqual([j.name for j in model.joints], ["j1"])

    def test_two_roots_rejected(self) -> None:
        with self.assertRaises(ValueError):
            parse_urdf('<robot><link name="a"/><link name="b"/></robot>')

    def test_cycle_with_no_root_rejected(self) -> None:
        with self.assertRaises(ValueError):
            parse_urdf(
                '<robot><link name="a"/><link name="b"/>'
                '<joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint>'
                '<joint name="j2" type="fixed"><parent link="b"/><child link="a"/></joint>'
                "</robot>"
            )

    def test_joint_to_missing_link_rejected(self) -> None:
        with self.assertRaises(ValueError):
            parse_urdf(
                '<robot><link name="a"/>'
                '<joint name="j1" type="fixed"><parent link="a"/><child link="missing"/></joint>'
                "</robot>"
            )

    def test_zero_axis_rejected_on_movable_but_not_fixed(self) -> None:
        with self.assertRaises(ValueError):
            parse_urdf(
                '<robot><link name="a"/><link name="b"/>'
                '<joint name="j1" type="revolute"><parent link="a"/><child link="b"/>'
                '<axis xyz="0 0 0"/><limit lower="0" upper="0"/></joint>'
                "</robot>"
            )
        model = parse_urdf(
            '<robot><link name="a"/><link name="b"/>'
            '<joint name="j1" type="fixed"><parent link="a"/><child link="b"/>'
            '<axis xyz="0 0 0"/></joint>'
            "</robot>"
        )
        self.assertEqual(model.root_link, "a")


if __name__ == "__main__":
    unittest.main()
