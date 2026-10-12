"""Over-the-wire tests: start `src/main.py` (what `make run` runs) and talk to it on 127.0.0.1:9095."""
import json
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


def _big_urdf() -> str:
    """As many joints as fit in a 1 MiB set_param line (~4,700): the worst case for per-tick work."""
    links, joints, i = ['<link name="l0"/>'], [], 0
    while len(json.dumps("".join(links + joints))) < 1_000_000 - 4096:
        i += 1
        links.append(f'<link name="l{i}"/>')
        joints.append(f'<joint name="j{i}" type="revolute"><parent link="l{(i - 1) // 2}"/><child link="l{i}"/>'
                      f'<origin xyz="0.1 0 0.05" rpy="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>')
    return "<robot>" + "".join(links + joints) + "</robot>"


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

    def server_rss_mib(self) -> int:
        return int(subprocess.check_output(["ps", "-o", "rss=", "-p", str(self.proc.pid)])) // 1024

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

    def test_rates_hold_at_5hz_with_a_1mib_description(self) -> None:
        self.client.subscribe("/tf")
        self.client.subscribe("/xform_world")
        self.load_robot(_big_urdf())
        self.recv("/xform_world", timeout=10)
        counts = {"/tf": 0, "/xform_world": 0}
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            topic = self.client.recv_line(timeout=5).get("topic")
            if topic in counts:
                counts[topic] += 1
        self.assertGreaterEqual(counts["/tf"], 15, counts)
        self.assertGreaterEqual(counts["/xform_world"], 15, counts)

    def test_subscriber_that_stops_reading_does_not_grow_server_memory(self) -> None:
        stalled = socket.create_connection((HOST, PORT))
        self.addCleanup(stalled.close)
        for topic in ("/tf", "/xform_world"):
            stalled.sendall(json.dumps({"op": "subscribe", "topic": topic}).encode() + b"\n")
        self.load_robot(_big_urdf())
        time.sleep(1.0)  # let the socket buffers fill
        before = self.server_rss_mib()
        time.sleep(6.0)  # unbounded buffering grew ~20 MiB/s here
        growth = self.server_rss_mib() - before
        self.assertLess(growth, 40, f"server grew {growth} MiB in 6 s with one stalled subscriber")
        # The server still works for everyone else.
        self.client.subscribe("/tf")
        self.recv("/tf", timeout=5)

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

    def test_each_joint_states_message_replaces_the_last_and_is_not_clamped(self) -> None:
        self.client.subscribe("/tf")
        self.load_robot()
        self.recv("/tf")
        self.client.advertise("/joint_states")
        self.client.publish("/joint_states", {"header": {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": ""},
                                              "name": ["j1"], "position": [math.pi / 2]})
        self.recv("/tf", lambda m: m["transforms"][0]["header"]["stamp"]["sec"] == 1)
        # j3 = 5 is past its limit of 1: used as given (5 * axis 2 = 10). j1 is not named, so it goes back to 0.
        self.client.publish("/joint_states", {"header": {"stamp": {"sec": 2, "nanosec": 0}, "frame_id": ""},
                                              "name": ["j3"], "position": [5.0]})
        entries = self.tf_by_child(self.recv("/tf", lambda m: m["transforms"][0]["header"]["stamp"]["sec"] == 2))
        r = entries["link1"]["transform"]["rotation"]
        self.assertVecAlmostEqual([r["x"], r["y"], r["z"], abs(r["w"])], [0, 0, 0, 1])
        t = entries["link3"]["transform"]["translation"]
        self.assertVecAlmostEqual([t["x"], t["y"], t["z"]], [0, 0, 10.0])

    def test_late_subscriber_gets_a_full_message_within_one_period(self) -> None:
        self.client.subscribe("/xform_world")
        self.load_robot()
        self.recv("/xform_world")
        for topic, count in (("/tf", 3), ("/xform_world", 4)):
            late = Client()
            self.addCleanup(late.close)
            start = time.monotonic()
            late.subscribe(topic)
            msg = late.recv_matching(_is_topic(topic), timeout=2)["msg"]
            self.assertLess(time.monotonic() - start, 0.2, topic)  # 5 Hz floor = 0.2 s period
            self.assertEqual(len(msg["transforms"]), count, topic)

    def test_reload_replaces_the_robot_and_a_bad_description_keeps_it(self) -> None:
        self.client.subscribe("/tf")
        self.client.subscribe("/xform_world")
        self.load_robot()
        self.recv("/xform_world")
        # Same root name, different children: merging old and new edges would still form one valid tree.
        self.load_robot('<robot><link name="base_link"/><link name="b1"/>'
                        '<joint name="k1" type="fixed"><parent link="base_link"/><child link="b1"/></joint></robot>')
        robot_b = {"base_link", "b1"}
        msg = self.recv("/xform_world", lambda m: "b1" in {e["child_frame_id"] for e in m["transforms"]})
        self.assertEqual({e["child_frame_id"] for e in msg["transforms"]} - {"global_frame"}, robot_b)

        self.load_robot("<robot><link name=")
        status = self.client.call_service("/param_server/get_param", {"name": STATUS_PARAM})["values"]["value"]
        deadline = time.monotonic() + 3
        while status["accepted"] and time.monotonic() < deadline:  # wait for the node to process the bad one
            time.sleep(0.05)
            status = self.client.call_service("/param_server/get_param", {"name": STATUS_PARAM})["values"]["value"]
        self.assertFalse(status["accepted"], status)
        self.assertNotEqual(status["error"], "")
        self.assertLess(status["loaded_version"], status["version"])
        # /tf keeps flowing with robot B: far more messages than could have been queued before the rejection.
        children, start = [], time.monotonic()
        while time.monotonic() - start < 1.5:
            children.append(frozenset(self.tf_by_child(self.recv("/tf"))))
        self.assertGreaterEqual(len(children), 10)
        self.assertEqual(set(children), {frozenset({"b1"})})
        msg = self.recv("/xform_world")
        self.assertEqual({e["child_frame_id"] for e in msg["transforms"]} - {"global_frame"}, robot_b)

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
