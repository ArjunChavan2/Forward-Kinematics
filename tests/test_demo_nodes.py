import copy
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import param_server  # noqa: E402
from finite_state_machine import FiniteStateMachine  # noqa: E402
from joint_state_publisher import MAX_SPEED, JointStatePublisher  # noqa: E402
from registry import Registry  # noqa: E402
from test_fk import CHAIN_URDF  # noqa: E402  (j1 revolute [-3, 3], j2 fixed, j3 prismatic [-1, 1])
from urdf import parse_urdf  # noqa: E402


class Recorder:
    def __init__(self, registry, topic):
        self.msgs = []
        registry.subscribe(self, topic)

    def send(self, message):
        self.msgs.append(message["msg"])


def trajectory(names, positions):
    return {"joint_names": names, "points": [{"positions": positions, "velocities": [], "accelerations": [],
                                              "time_from_start": {"sec": 0, "nanosec": 0}}]}


class JointStatePublisherTest(unittest.TestCase):
    def setUp(self):
        self.registry = Registry()
        param_server.register(self.registry)
        self.jsp = JointStatePublisher(self.registry, parse_urdf)
        self.states = Recorder(self.registry, "/joint_states")

    def load(self, urdf=CHAIN_URDF):
        self.registry.call_service_sync("/param_server/set_param", {"name": "robot_description", "value": urdf})
        self.jsp.poll_once()

    def test_moves_at_capped_speed_and_clamps_to_limits(self):
        self.load()
        self.registry.publish("/joint_trajectory", trajectory(["j1", "j3"], [0.25, 5.0]))
        self.jsp.step(0.1)
        self.assertAlmostEqual(self.jsp.positions["j1"], MAX_SPEED * 0.1)
        for _ in range(50):
            self.jsp.step(0.1)
        self.assertEqual(self.jsp.positions, {"j1": 0.25, "j3": 1.0})  # j3 stops at its upper limit
        self.jsp.publish()
        self.assertEqual(dict(zip(self.states.msgs[-1]["name"], self.states.msgs[-1]["position"])),
                         {"j1": 0.25, "j3": 1.0})

    def test_setpoint_that_arrives_before_the_robot_still_applies(self):
        self.registry.publish("/joint_trajectory", trajectory(["j1"], [0.05]))
        self.jsp.publish()
        self.assertEqual(self.states.msgs, [])  # no robot, no /joint_states
        self.load()
        self.jsp.step(0.1)
        self.assertAlmostEqual(self.jsp.positions["j1"], 0.05)

    def test_reset_goes_to_middle_of_limits(self):
        self.load(CHAIN_URDF.replace('lower="-1" upper="1"', 'lower="0" upper="0.5"'))
        self.jsp.reset({})
        self.assertEqual(self.jsp.setpoints, {"j1": 0.0, "j3": 0.25})


TABLE = {"initial_state": "home", "states": [
    {"name": "home", "target": {"joint_names": ["j1"], "positions": [0.0]}, "epsilon": 0.02,
     "on_arrival": "to_wave", "transitions": [{"name": "to_wave", "next_state": "wave"}]},
    {"name": "wave", "target": {"joint_names": ["j1"], "positions": [0.8]}, "epsilon": 0.02,
     "transitions": [{"name": "to_home", "next_state": "home"}]}]}


class FiniteStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.registry = Registry()
        self.dir = tempfile.mkdtemp()
        self.fsm = FiniteStateMachine(self.registry, fsm_dir=self.dir)
        self.trajectories = Recorder(self.registry, "/joint_trajectory")

    def call(self, name, args):
        return self.registry.call_service_sync(f"/fsm/{name}", args)

    def joint_states(self, j1):
        self.registry.publish("/joint_states", {"name": ["j1"], "position": [j1]})

    def test_publishes_target_on_entry_and_follows_on_arrival(self):
        self.assertEqual(self.call("update_logic", TABLE), (True, {"current_state": "home"}, ""))
        self.assertEqual(self.trajectories.msgs[-1], trajectory(["j1"], [0.0]))
        self.joint_states(0.5)
        self.fsm.step()
        self.assertEqual(self.fsm.current["name"], "home")  # not there yet
        self.joint_states(0.01)
        self.fsm.step()
        self.assertEqual(self.fsm.current["name"], "wave")
        self.assertEqual(self.trajectories.msgs[-1], trajectory(["j1"], [0.8]))
        self.assertEqual(self.fsm.last_transition, "to_wave")

    def test_invalid_table_leaves_the_old_one_running(self):
        self.call("update_logic", TABLE)
        bad = copy.deepcopy(TABLE)
        bad["states"][1]["transitions"][0]["next_state"] = "nowhere"
        ok, _, status = self.call("update_logic", bad)
        self.assertFalse(ok)
        self.assertIn("nowhere", status)
        self.assertEqual(self.call("get_logic", {})[1], TABLE)
        for broken in ({"initial_state": "x", "states": TABLE["states"]}, {"initial_state": "home", "states": []},
                       {"initial_state": "home", "states": TABLE["states"] + TABLE["states"][:1]}):
            self.assertFalse(self.call("update_logic", broken)[0], broken)

    def test_fire_transition_by_hand_and_rejected_while_stopped(self):
        self.call("update_logic", TABLE)
        self.assertEqual(self.call("set_running", {"data": False})[1], {"running": False})
        self.assertFalse(self.call("fire_transition", {"transition": "to_wave"})[0])
        self.joint_states(0.0)
        self.fsm.step()
        self.assertEqual(self.fsm.current["name"], "home")  # stopped: on_arrival does not fire
        self.call("set_running", {"data": True})
        self.assertFalse(self.call("fire_transition", {"transition": "to_home"})[0])  # not a transition of home
        self.assertEqual(self.call("fire_transition", {"transition": "to_wave"})[1], {"current_state": "wave"})

    def test_save_load_list_and_identifiers_cannot_escape_the_directory(self):
        self.assertTrue(self.call("save_to_file", dict(TABLE, identifier="wave_1"))[0])
        self.assertEqual(self.call("load_from_file", {"identifier": "wave_1"})[1], TABLE)
        self.assertEqual(self.call("list_saved", {})[1], {"identifiers": ["wave_1"]})
        for bad in ("../escape", "a/b", "", "x" * 65, "dot.json", None):
            self.assertFalse(self.call("save_to_file", dict(TABLE, identifier=bad))[0], bad)
            self.assertFalse(self.call("load_from_file", {"identifier": bad})[0], bad)
        self.assertEqual(os.listdir(self.dir), ["wave_1.json"])
        self.assertFalse(os.path.exists(os.path.join(self.dir, "..", "escape.json")))


if __name__ == "__main__":
    unittest.main()
