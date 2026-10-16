"""arm_angles inverts the robot's own FK: random arm angles -> FK directions -> arm_angles -> same directions."""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import fk  # noqa: E402
from urdf import parse_urdf  # noqa: E402
from video_to_choreo import arm_angles  # noqa: E402

URDF = os.path.join(os.path.dirname(__file__), "..", "robots", "dual_arm_robot.urdf")


class ArmAnglesTest(unittest.TestCase):
    def test_recovers_upper_arm_and_forearm_directions(self) -> None:
        with open(URDF, encoding="utf-8") as f:
            model = parse_urdf(f.read())
        rng = random.Random(1)

        def directions(side, q):
            T = fk.link_transforms("dance_floor", fk.joint_edges(model, q), fk.identity_pose())
            pos = lambda link: [T[link][i][3] for i in range(3)]  # noqa: E731
            torso = [[T["torso"][i][k] for i in range(3)] for k in range(3)]  # rows: torso axes in world
            s, e, w = pos(f"{side}_shoulder_roll_link"), pos(f"{side}_forearm"), pos(f"{side}_wrist_link")
            local = lambda v: [sum(torso[k][i] * v[i] for i in range(3)) for k in range(3)]  # noqa: E731
            unit = lambda v: [x / math.sqrt(sum(y * y for y in v)) for x in v]  # noqa: E731
            return local(unit([e[i] - s[i] for i in range(3)])), local(unit([w[i] - e[i] for i in range(3)]))

        for _ in range(40):
            side = rng.choice(["left", "right"])
            q = {f"{side}_shoulder_pitch": rng.uniform(-3, 3), f"{side}_shoulder_roll": rng.uniform(-1.3, 1.3),
                 f"{side}_shoulder_yaw": rng.uniform(-3, 3), f"{side}_elbow_pitch": rng.uniform(-2.5, -0.4),
                 "waist_yaw": rng.uniform(-1, 1)}
            upper, fore = directions(side, q)
            p, r, y, e = arm_angles(upper, fore)
            got = directions(side, {**q, f"{side}_shoulder_pitch": p, f"{side}_shoulder_roll": r,
                                    f"{side}_shoulder_yaw": y, f"{side}_elbow_pitch": e})
            for a, b in zip(upper + fore, got[0] + got[1]):
                self.assertAlmostEqual(a, b, places=6)


if __name__ == "__main__":
    unittest.main()
