"""Over-the-wire tests: start `src/main.py` (what `make run` runs) and talk to it on 127.0.0.1:9095."""
import math
import os
import socket
import subprocess
import sys
import time
import unittest

SRC = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, SRC)

import fk  # noqa: E402
from client_helper import HOST, PORT, Client  # noqa: E402
from test_fk import CHAIN_URDF, S  # noqa: E402
from urdf import RobotModel  # noqa: E402

TOL = 1e-4
STATUS_PARAM = "/robot_state_publisher/description_status"


def _fk_written() -> bool:
    try:
        root = fk.identity_pose()
        fk.joint_edges(RobotModel(root_link="a"), {})
        fk.link_transforms("a", [], root)
        fk.root_of_edges([("a", "b", root)])
    except NotImplementedError:
        return False
    return True


def _is_topic(topic):
    return lambda m: m.get("op") == "publish" and m.get("topic") == topic


@unittest.skipUnless(_fk_written(), "src/fk.py is not implemented yet")
class WireTest(unittest.TestCase):
    def setUp(self) -> None:
        self.started = time.monotonic()
        self.proc = subprocess.Popen([sys.executable, os.path.join(SRC, "main.py")],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(self._stop)
        deadline = time.monotonic() + 10
        while True:
            try:
                socket.create_connection((HOST, PORT), timeout=0.2).close()
                break
            except OSError:
                if time.monotonic() > deadline or self.proc.poll() is not None:
                    self.fail("gateway did not start listening")
                time.sleep(0.05)
        self.client = Client()
        self.addCleanup(self.client.close)

    def _stop(self) -> None:
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()

    def load_robot(self, urdf: str = CHAIN_URDF) -> None:
        reply = self.client.call_service("/param_server/set_param", {"name": "robot_description", "value": urdf})
        self.assertTrue(reply["result"], reply)

    def recv(self, topic: str, predicate=lambda msg: True, timeout: float = 3.0) -> dict:
        return self.client.recv_matching(lambda m: _is_topic(topic)(m) and predicate(m["msg"]),
                                         timeout=timeout, max_messages=1000)["msg"]

    def tf_by_child(self, msg: dict) -> dict:
        return {e["child_frame_id"]: e for e in msg["transforms"]}

    def xform_translation(self, msg: dict, link: str) -> list:
        entry = next(e for e in msg["transforms"] if e["child_frame_id"] == link)
        return entry["matrix"][12:15]

    def assertVecAlmostEqual(self, actual, expected) -> None:
        for a, e in zip(actual, expected, strict=True):
            self.assertAlmostEqual(a, e, delta=TOL, msg=f"{actual} != {expected}")

    # -- tests -----------------------------------------------------------------

    def test_ready_within_10s_of_start(self) -> None:
        self.client.subscribe("/xform_world")
        self.load_robot()
        self.recv("/xform_world")
        self.assertLess(time.monotonic() - self.started, 10.0)
        reply = self.client.call_service("/param_server/get_param", {"name": STATUS_PARAM})
        self.assertTrue(reply["values"]["value"]["accepted"], reply)

    def test_no_tf_without_a_robot(self) -> None:
        self.client.subscribe("/tf")
        with self.assertRaises((TimeoutError, socket.timeout)):
            self.recv("/tf", timeout=0.5)

    def test_tf_has_one_entry_per_joint_at_zero_configuration(self) -> None:
        self.client.subscribe("/tf")
        self.load_robot()
        entries = self.tf_by_child(self.recv("/tf"))
        self.assertEqual(len(entries), 3)
        self.assertEqual({c: e["header"]["frame_id"] for c, e in entries.items()},
                         {"link1": "base_link", "link2": "link1", "link3": "link2"})
        for child, xyz in (("link1", [0, 0, 1]), ("link2", [1, 0, 0]), ("link3", [0, 0, 0])):
            t, r = entries[child]["transform"]["translation"], entries[child]["transform"]["rotation"]
            self.assertVecAlmostEqual([t["x"], t["y"], t["z"]], xyz)
            self.assertVecAlmostEqual([r["x"], r["y"], r["z"], abs(r["w"])], [0, 0, 0, 1])
            self.assertEqual(entries[child]["header"]["stamp"], {"sec": 0, "nanosec": 0})

    def test_tf_rate_is_at_least_5hz_without_changes(self) -> None:
        self.client.subscribe("/tf")
        self.load_robot()
        self.recv("/tf")
        start, count = time.monotonic(), 0
        while time.monotonic() - start < 2.0:
            self.recv("/tf")
            count += 1
        self.assertGreaterEqual(count, 10)

    def test_joint_states_move_tf_and_stamp_is_copied_verbatim(self) -> None:
        self.client.subscribe("/tf")
        self.load_robot()
        self.recv("/tf")
        stamp = {"sec": 123, "nanosec": 456}
        self.client.advertise("/joint_states")
        self.client.publish("/joint_states", {
            "header": {"stamp": stamp, "frame_id": ""},
            "name": ["j1", "j3", "not_a_joint", "j2"],
            "position": [math.pi / 2, 0.5, 9.0, 9.0],
        })
        msg = self.recv("/tf", lambda m: m["transforms"] and m["transforms"][0]["header"]["stamp"] == stamp)
        self.assertTrue(all(e["header"]["stamp"] == stamp for e in msg["transforms"]))
        entries = self.tf_by_child(msg)
        r = entries["link1"]["transform"]["rotation"]
        sign = 1 if r["w"] > 0 else -1
        self.assertVecAlmostEqual([sign * r[k] for k in "xyzw"], [0, 0, S, S])
        t = entries["link3"]["transform"]["translation"]
        self.assertVecAlmostEqual([t["x"], t["y"], t["z"]], [0, 0, 1.0])

    def test_xform_world_composes_global_pose(self) -> None:
        self.client.subscribe("/xform_world")
        self.load_robot()
        self.recv("/xform_world")
        self.client.advertise("/joint_states")
        self.client.advertise("/global_pose")
        stamp = {"sec": 7, "nanosec": 8}
        self.client.publish("/global_pose", {"position": {"x": 10.0, "y": 0.0, "z": 0.0},
                                             "orientation": {"x": 0.0, "y": 0.0, "z": S, "w": S}})
        self.client.publish("/joint_states", {"header": {"stamp": stamp, "frame_id": ""},
                                              "name": ["j1", "j3"], "position": [math.pi / 2, 0.5]})
        # The global pose lands before the next /tf, so wait for one with the new stamp and the moved base.
        msg = self.recv("/xform_world", lambda m: m["transforms"][0]["header"]["stamp"] == stamp
                        and abs(self.xform_translation(m, "base_link")[0] - 10.0) < TOL)
        self.assertEqual({e["child_frame_id"] for e in msg["transforms"]} - {"global_frame"},
                         {"base_link", "link1", "link2", "link3"})
        self.assertTrue(all(e["header"]["frame_id"] == "global_frame" for e in msg["transforms"]))
        self.assertTrue(all(e["header"]["stamp"] == stamp for e in msg["transforms"]))
        # Global Rz(90°) · local (0, 1, 2) + (10, 0, 0) = (9, 0, 2).
        self.assertVecAlmostEqual(self.xform_translation(msg, "base_link"), [10, 0, 0])
        self.assertVecAlmostEqual(self.xform_translation(msg, "link3"), [9, 0, 2])


if __name__ == "__main__":
    unittest.main()
