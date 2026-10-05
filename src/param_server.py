"""In-process parameter server: /param_server/set_param and /param_server/get_param.

Each name keeps an independent version, starting at 0 and incremented by each set_param.
"""
from __future__ import annotations

from typing import Any

from registry import Registry

SET_PARAM = "/param_server/set_param"
GET_PARAM = "/param_server/get_param"


class ParamStore:
    def __init__(self) -> None:
        self._values: dict[str, Any] = {}
        self._versions: dict[str, int] = {}

    def set_param(self, args: Any) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "request must be an object"
        name = args.get("name")
        if not isinstance(name, str):
            return False, {}, "name must be a string"
        if "value" not in args:
            return False, {}, "missing value"
        self._values[name] = args["value"]
        self._versions[name] = self._versions.get(name, 0) + 1
        return True, {"version": self._versions[name]}, ""

    def get_param(self, args: Any) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "request must be an object"
        name = args.get("name")
        if not isinstance(name, str):
            return False, {}, "name must be a string"
        if name not in self._values:
            return False, {"value": None, "version": 0}, f"parameter {name!r} is not set"
        return True, {"value": self._values[name], "version": self._versions[name]}, ""


def register(registry: Registry) -> ParamStore:
    store = ParamStore()
    registry.register_handler(SET_PARAM, store.set_param)
    registry.register_handler(GET_PARAM, store.get_param)
    return store
