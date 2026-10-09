"""URDF parsing and validation. Hand-written by the project owner.

parse_urdf takes the robot_description text and returns a RobotModel, or raises ValueError with the reason
the description is rejected (the reason becomes the description_status error).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from transform import Vec3

import re
import xml.etree.ElementTree as ET

NUMBER_RE = re.compile(r"[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][+-]?[0-9]+)?")


@dataclass
class Limit:
    """Joint limits from a joint's <limit> element.

    The values are informational: FK never clamps joint positions to them.
    """
    lower: float
    upper: float


@dataclass
class Joint:
    """One joint of the robot, read from a direct child of <robot>.

    origin_xyz and origin_rpy describe T_origin in the parent link's frame. axis is kept as written in the
    URDF; revolute and continuous axes are normalized when motion is computed, prismatic axes are not.
    limit is None for continuous and fixed joints, and is required for revolute and prismatic joints.
    """
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
    """One link of the robot, read from a direct child of <robot>.

    name is the value of the name attribute after XML entity decoding.
    """
    name: str


@dataclass
class RobotModel:
    """A parsed, validated robot.

    root_link is the unique link that no joint names as its child. links and joints hold only the direct
    children of <robot>, in document order.
    """
    root_link: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)


def _parse_root(text: str) -> ET.Element:
    """Step 1-2: parse text into an element tree and check the root tag.

    Raises ValueError for empty or whitespace-only text, text that is not well-formed XML (catch
    ET.ParseError), or a root element whose tag is not "robot".
    """
    try:
        l = len(text)
        root = ET.fromstring(text.lstrip())
        if l == 0 or l == text.count(" ") or root.tag != "robot":
            raise ValueError
    except ET.ParseError:
        raise ValueError
    
    return root
        


def _parse_number(token: str) -> float:
    """Parse one token against the spec's number grammar, rejecting what float() would wrongly accept."""
    if NUMBER_RE.fullmatch(token) is None:
        raise ValueError
    return float(token)


def _parse_vector(text: str, count: int = 3) -> tuple:
    """Parse an attribute value holding exactly `count` whitespace-separated numbers.

    Raises ValueError if the token count is wrong, or any token is not a number matching the spec's
    grammar.
    """
    tokens = text.split()
    if len(tokens) != count:
        raise ValueError
    return tuple(_parse_number(t) for t in tokens)


def _parse_link(elem: ET.Element) -> Link:
    """Step 3-4: build a Link from a direct-child <link> element.

    Raises ValueError if the element has no name attribute.
    """
    if "name" not in elem.attrib:
        raise ValueError
    return Link(elem.attrib["name"])


def _parse_joint(elem: ET.Element) -> Joint:
    """Step 3-4: build a Joint from a direct-child <joint> element.

    Raises ValueError if name or type is missing, type is not one of revolute/continuous/prismatic/fixed,
    <parent> or <child> is missing or has no link attribute, a vector attribute is malformed, or a
    revolute/prismatic joint has no <limit> (or a present lower/upper is not a number). Applies the
    defaults for a missing <origin>/xyz/rpy (zeros) and <axis> (1 0 0). Ignores <limit> entirely for
    continuous and fixed joints, even if malformed. Does not validate the first <visual> here (see
    _parse_visual) or that parent/child name real links (see _validate_robot).
    """
    name = elem.get("name")
    joint_type = elem.get("type")
    if name is None or joint_type not in ["revolute", "continuous", "prismatic", "fixed"]:
        raise ValueError
    
    parent = elem.find("parent")
    if parent is None or parent.get("link") is None:
        raise ValueError    
    else:
        parent = parent.get("link")
    
    child = elem.find("child")
    if child is None or child.get("link") is None:
        raise ValueError
    else:
        child = child.get("link")
    
    origin = elem.find("origin")
    if origin is None:
        origin_xyz = (0, 0, 0)
        origin_rpy = (0, 0, 0)
    else:
        origin_xyz = _parse_vector(origin.get("xyz", "0 0 0"))
        origin_rpy = _parse_vector(origin.get("rpy", "0 0 0"))
    
    axis = elem.find("axis")
    if axis is None:
        axis = (1, 0, 0)
    else:
        axis = _parse_vector(axis.get("xyz", "1 0 0"))
        
    if joint_type in ("revolute", "prismatic"):
        limit_elem = elem.find("limit")
        if limit_elem is None:
            raise ValueError
        lower = _parse_number(limit_elem.get("lower", "0"))
        upper = _parse_number(limit_elem.get("upper", "0"))
        limit = Limit(lower, upper)
    else:
        limit = None

    new_joint = Joint(name, joint_type, parent, child, origin_xyz, origin_rpy, axis, limit)
    return new_joint
    
        


def _parse_visual(elem: ET.Element) -> None:
    """Step 4: validate a link's first <visual> element, if present.

    Raises ValueError if <visual> is present but its <geometry> is missing, empty, or not one of
    box/cylinder/sphere/mesh with the required attributes. A later <visual> on the same link is not
    read here. Does not return anything useful to FK: visuals never affect /tf or /xform_world.
    """
    visual = elem.find("visual")
    if visual is None:
        return
    geometry = visual.find("geometry")
    if geometry is None:
        raise ValueError("visual has no geometry")

    box = geometry.find("box")
    if box is not None:
        if box.get("size") is None:
            raise ValueError("box has no size")
        _parse_vector(box.get("size"))
        return

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        if cylinder.get("radius") is None or cylinder.get("length") is None:
            raise ValueError
        _parse_number(cylinder.get("radius"))
        _parse_number(cylinder.get("length"))
        return

    sphere = geometry.find("sphere")
    if sphere is not None:
        if sphere.get("radius") is None:
            raise ValueError
        _parse_number(sphere.get("radius"))
        return

    mesh = geometry.find("mesh")
    if mesh is not None:
        if mesh.get("filename") is None:
            raise ValueError
        scale = mesh.get("scale")
        if scale is not None:
            _parse_vector(scale)
        return

    raise ValueError



def _validate_robot(links: list[Link], joints: list[Joint]) -> str:
    """Step 5: validate a fully-parsed robot and return its root link's name.

    Raises ValueError if a joint's parent or child does not name a link in `links`, if there is not
    exactly one root (a link that is no joint's child), or if a movable (revolute/continuous/prismatic)
    joint's axis is zero-length (our own choice; the spec leaves this unspecified).
    """
    link_names = {link.name for link in links}
    for joint in joints:
        if joint.parent not in link_names or joint.child not in link_names:
            raise ValueError
        if joint.joint_type in ("revolute", "continuous", "prismatic") and all(c == 0 for c in joint.axis):
            raise ValueError
    child_names = {joint.child for joint in joints}
    roots = link_names - child_names
    if len(roots) != 1:
        raise ValueError
    return next(iter(roots))


def parse_urdf(text: str) -> RobotModel:
    """Parse the robot_description text and return a RobotModel.

    Raises ValueError with the reason when the description must be rejected: text that is not well-formed
    XML, a root element other than <robot>, a link or joint missing required attributes, an unknown joint
    type, a joint naming a link that does not exist, zero or several roots, a revolute or prismatic joint
    without a <limit>, a vector attribute that does not hold exactly three numbers, or an invalid first
    <visual>. Nested <link> and <joint> elements outside the direct children of <robot> are ignored.
    """
    root = _parse_root(text)
    links: list[Link] = []
    joints: list[Joint] = []
    for child in root:
        if child.tag == "link":
            links.append(_parse_link(child))
            _parse_visual(child)
        elif child.tag == "joint":
            joints.append(_parse_joint(child))

    root_link = _validate_robot(links, joints)
    return RobotModel(root_link, links, joints)

