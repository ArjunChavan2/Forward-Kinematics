"""robot_world_state_publisher: subscribes /tf and /global_pose, composes them into /xform_world.

Decodes each /tf TransformStamped back into a (parent, child, Matrix) edge, then hands those edges
to fk.root_of_edges/fk.link_transforms (the hand-written contract) to do the actual tree
composition -- this module does no FK math of its own, only wire decode/encode.

Per spec: compute from the latest /tf message alone, never merge in edges from an earlier message.
"""
from __future__ import annotations

from registry import Registry
from transform import make_transform, quat_to_matrix, transform_to_column_major

import fk

TF_TOPIC = "/tf"
GLOBAL_POSE_TOPIC = "/global_pose"
XFORM_WORLD_TOPIC = "/xform_world"
GLOBAL_FRAME = "global_frame"

_IDENTITY_4 = [[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0]]


class RobotWorldStatePublisher:
    def __init__(self, registry: Registry) -> None:
        self.registry = registry
        self.global_pose = _IDENTITY_4
        registry.subscribe(self, TF_TOPIC)
        registry.subscribe(self, GLOBAL_POSE_TOPIC)

    # -- Connection protocol (for registry.subscribe) -----------------------

    def send(self, message: dict) -> None:
        topic = message.get("topic")
        if topic == TF_TOPIC:
            self._on_tf(message["msg"])
        elif topic == GLOBAL_POSE_TOPIC:
            self._on_global_pose(message["msg"])

    def _on_global_pose(self, msg: dict) -> None:
        pos = msg["position"]
        ori = msg["orientation"]
        quat_wxyz = (ori["w"], ori["x"], ori["y"], ori["z"])
        self.global_pose = make_transform(quat_to_matrix(quat_wxyz), [pos["x"], pos["y"], pos["z"]])

    def _on_tf(self, msg: dict) -> None:
        entries = msg.get("transforms") or []
        if not entries:
            # A robot with no joints has an empty /tf; the spec leaves /xform_world UNSPECIFIED
            # here. Publish nothing rather than guess a root.
            return

        edges = []
        for entry in entries:
            parent = entry["header"]["frame_id"]
            child = entry["child_frame_id"]
            t = entry["transform"]["translation"]
            r = entry["transform"]["rotation"]
            quat_wxyz = (r["w"], r["x"], r["y"], r["z"])
            matrix = make_transform(quat_to_matrix(quat_wxyz), [t["x"], t["y"], t["z"]])
            edges.append((parent, child, matrix))

        root = fk.root_of_edges(edges)
        poses = fk.link_transforms(root, edges, self.global_pose)

        stamp = entries[0]["header"]["stamp"]
        out_entries = [
            {
                "header": {"stamp": stamp, "frame_id": GLOBAL_FRAME},
                "child_frame_id": link_name,
                "matrix": transform_to_column_major(pose),
            }
            for link_name, pose in poses.items()
        ]
        self.registry.publish(XFORM_WORLD_TOPIC, {"transforms": out_entries})


def register(registry: Registry) -> RobotWorldStatePublisher:
    return RobotWorldStatePublisher(registry)
