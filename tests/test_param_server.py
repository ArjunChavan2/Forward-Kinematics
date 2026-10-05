import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from param_server import GET_PARAM, SET_PARAM, register  # noqa: E402
from registry import Registry  # noqa: E402


class ParamServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.registry = Registry()
        register(self.registry)

    def call(self, service: str, args):
        return self.registry.call_service_sync(service, args)

    def test_first_set_is_version_one(self) -> None:
        ok, values, status = self.call(SET_PARAM, {"name": "robot_description", "value": "<robot/>"})
        self.assertTrue(ok)
        self.assertEqual(values, {"version": 1})
        self.assertEqual(status, "")

    def test_versions_increment_per_name(self) -> None:
        self.call(SET_PARAM, {"name": "a", "value": 1})
        self.call(SET_PARAM, {"name": "a", "value": 2})
        _, values_b, _ = self.call(SET_PARAM, {"name": "b", "value": 3})
        self.assertEqual(values_b, {"version": 1})
        _, values_a, _ = self.call(SET_PARAM, {"name": "a", "value": 4})
        self.assertEqual(values_a, {"version": 3})

    def test_get_returns_value_and_version(self) -> None:
        self.call(SET_PARAM, {"name": "k", "value": {"x": [1, 2.5]}})
        ok, values, status = self.call(GET_PARAM, {"name": "k"})
        self.assertTrue(ok)
        self.assertEqual(values, {"value": {"x": [1, 2.5]}, "version": 1})
        self.assertEqual(status, "")

    def test_get_unset_name_fails_with_null_value_and_version_zero(self) -> None:
        ok, values, status = self.call(GET_PARAM, {"name": "missing"})
        self.assertFalse(ok)
        self.assertEqual(values, {"value": None, "version": 0})
        self.assertNotEqual(status, "")

    def test_non_string_or_missing_name_is_rejected(self) -> None:
        for args in ({}, {"name": 5, "value": 1}, {"value": 1}, None, "name"):
            ok, _, status = self.call(SET_PARAM, args)
            self.assertFalse(ok, args)
            self.assertNotEqual(status, "", args)
        for args in ({}, {"name": None}, None):
            ok, _, status = self.call(GET_PARAM, args)
            self.assertFalse(ok, args)
            self.assertNotEqual(status, "", args)

    def test_set_without_value_is_rejected_and_leaves_store_unchanged(self) -> None:
        self.call(SET_PARAM, {"name": "k", "value": 1})
        ok, _, status = self.call(SET_PARAM, {"name": "k"})
        self.assertFalse(ok)
        self.assertNotEqual(status, "")
        _, values, _ = self.call(GET_PARAM, {"name": "k"})
        self.assertEqual(values, {"value": 1, "version": 1})

    def test_set_null_value_is_a_valid_value(self) -> None:
        ok, values, _ = self.call(SET_PARAM, {"name": "k", "value": None})
        self.assertTrue(ok)
        self.assertEqual(values, {"version": 1})
        ok, values, _ = self.call(GET_PARAM, {"name": "k"})
        self.assertTrue(ok)
        self.assertEqual(values, {"value": None, "version": 1})


if __name__ == "__main__":
    unittest.main()
