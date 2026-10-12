"""Forward kinematics over a RobotModel. Hand-written by the project owner.

Each joint contributes T_joint(q) = T_origin · T_motion(q), composed parent first.
"""
from __future__ import annotations

from transform import Matrix, rpy_to_matrix, make_transform, joint_motion, matmul
from urdf import RobotModel


def joint_edges(model: RobotModel, positions: dict[str, float]) -> list[tuple[str, str, Matrix]]:
    """Return one (parent, child, T_joint) edge per joint in model.joints, in that order.

    T_joint = T_origin * T_motion(q), where T_origin comes from the joint's origin_xyz/origin_rpy and
    T_motion(q) is transform.joint_motion(joint_type, axis, q). q is positions.get(joint.name, 0.0), so a
    joint absent from positions is treated as being at 0 -- the caller (robot_state_publisher) is
    responsible for only including movable joints it actually knows about; a joint missing here still gets
    an edge, just at q=0. The root link itself has no edge, since it is nobody's child.
    """
    edges = []
    for joint in model.joints:
        if joint.name in positions:
            q = positions[joint.name]
        else:
            q = 0.0
        rotation = rpy_to_matrix(joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2])
        T_origin = make_transform(rotation, joint.origin_xyz)
        T_motion = joint_motion(joint.joint_type, joint.axis, q)
        T_joint = matmul(T_origin, T_motion)
        edges.append((joint.parent, joint.child, T_joint))
    return edges
        


def link_transforms(
    root: str,
    edges: list[tuple[str, str, Matrix]],
    root_pose: Matrix,
) -> dict[str, Matrix]:
    """Return {link_name: world_pose} for every link reachable from root via edges.

    world_pose[root] = root_pose. For every other link, world_pose[child] = world_pose[parent] * T_joint,
    composing parent-first down the tree. edges may come from joint_edges (a RobotModel's own joints) or
    be decoded from a wire /tf message (robot_world_state_publisher) -- this function doesn't care which;
    it only needs (parent, child, Matrix) triples and a starting pose for the root.
    """
    family = {}
    for edge in edges:
        parent, child, T = edge
        if parent not in family:
            family[parent] = []
        family[parent].append([child, T])
    
    
    world = {root: root_pose}
    stack = [root]
    while stack:
        link = stack.pop()
        for child, T in family.get(link, []):
            if child in world:  # a loop in wire /tf data; revisiting would never terminate
                continue
            stack.append(child)
            world[child] = matmul(world[link], T)
    
    return world
        
    
    


def identity_pose() -> Matrix:
    """Return the 4x4 identity transform: no rotation, no translation.

    Used as root_pose when there is no external pose to start from (e.g. a /global_pose that has not
    arrived yet).
    """
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],]


def root_of_edges(edges: list[tuple[str, str, Matrix]]) -> str:
    """Return the name of the one link that is some edge's parent and no edge's child.

    Raises an error (type and message are up to you) if edges is empty, or if there is not exactly one
    such link -- both are cases the URDF/robot_description validation should already have ruled out for a
    RobotModel's own edges, but a decoded /tf message is wire data and could in principle be malformed.
    """
    parents = set()
    children = set()
    uniques = set()
    for edge in edges:
        parents.add(edge[0])
        children.add(edge[1])
    
    for parent in parents:
        if parent not in children:
            uniques.add(parent)
    
    if not edges:
        raise ValueError("cannot find a root: no edges (an empty /tf has no parent/child pairs)")
    if len(uniques) != 1:
        if uniques:
            raise ValueError(f"expected exactly one root (a parent that is no edge's child), found "
                             f"{len(uniques)}: {sorted(uniques)} -- the edges form separate trees")
        raise ValueError("no root: every parent is also some edge's child, so the edges form a loop")
    return uniques.pop()

