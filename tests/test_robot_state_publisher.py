import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from param_server import GET_PARAM, SET_PARAM, register  # noqa: E402
from registry import Registry  # noqa: E402
from robot_state_publisher import DESCRIPTION_PARAM, STATUS_PARAM, RobotStatePublisher  # noqa: E402


class _Robot:
    def __init__(self, root_link: str) -> None:
        self.root_link = root_link


def _loader(text: str) -> _Robot:
    if not text.startswith("ok:"):
        raise ValueError("not a robot")
    return _Robot(text[3:])


class RobotStatePublisherTest(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = Registry()
        register(self.registry)
        self.rsp = RobotStatePublisher(self.registry, _loader)

    def set_description(self, value) -> None:
        ok, _, status = self.registry.call_service_sync(SET_PARAM, {"name": DESCRIPTION_PARAM, "value": value})
        self.assertTrue(ok, status)

    def status(self) -> dict:
        ok, values, _ = self.registry.call_service_sync(GET_PARAM, {"name": STATUS_PARAM})
        self.assertTrue(ok)
        return values["value"]

    def test_nothing_loaded_before_any_description(self) -> None:
        self.assertFalse(self.rsp.poll_once())
        self.assertEqual(self.rsp.loaded_version, 0)
        self.assertEqual(self.rsp.root_link, "")

    def test_accepted_description_replaces_robot_and_reports(self) -> None:
        self.set_description("ok:base_link")
        self.assertTrue(self.rsp.poll_once())
        self.assertEqual(self.status(), {
            "version": 1, "accepted": True, "error": "",
            "loaded_version": 1, "root_link": "base_link",
        })

    def test_rejected_description_keeps_previous_robot(self) -> None:
        self.set_description("ok:base_link")
        self.rsp.poll_once()
        self.set_description("broken")
        self.rsp.poll_once()
        status = self.status()
        self.assertFalse(status["accepted"])
        self.assertNotEqual(status["error"], "")
        self.assertEqual(status["version"], 2)
        self.assertEqual(status["loaded_version"], 1)
        self.assertEqual(status["root_link"], "base_link")
        self.assertEqual(self.rsp.root_link, "base_link")

    def test_rejection_with_nothing_loaded_reports_zero_and_empty(self) -> None:
        self.set_description("broken")
        self.rsp.poll_once()
        self.assertEqual(self.status()["loaded_version"], 0)
        self.assertEqual(self.status()["root_link"], "")

    def test_non_string_description_is_rejected(self) -> None:
        self.set_description({"not": "a string"})
        self.rsp.poll_once()
        self.assertFalse(self.status()["accepted"])

    def test_same_version_is_not_processed_twice(self) -> None:
        self.set_description("ok:base_link")
        self.assertTrue(self.rsp.poll_once())
        self.assertFalse(self.rsp.poll_once())

    def test_skipped_versions_process_only_the_latest(self) -> None:
        self.set_description("ok:first")
        self.set_description("ok:second")
        self.rsp.poll_once()
        self.assertEqual(self.status()["version"], 2)
        self.assertEqual(self.status()["root_link"], "second")

    def test_loader_crash_is_reported_not_raised(self) -> None:
        def crashing(_text: str):
            raise RuntimeError("boom")
        rsp = RobotStatePublisher(self.registry, crashing)
        self.set_description("anything")
        rsp.poll_once()
        self.assertFalse(self.status()["accepted"])
        self.assertIn("boom", self.status()["error"])


if __name__ == "__main__":
    unittest.main()
