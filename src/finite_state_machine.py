"""finite_state_machine (make demo only, never make run): runs a choreography table of joint-space setpoints.

On entering a state it publishes the state's target on /joint_trajectory. It watches /joint_states, and once
every named joint is within the state's epsilon it follows the state's on_arrival transition, if any.
Services and the table format follow FK_API.md's reference finite_state_machine.
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import time
from typing import Any

from registry import Registry

JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_TOPIC = "/joint_trajectory"
STATUS_TOPIC = "/fsm/status"
INTERVAL_S = 0.1
FSM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fsm")
IDENTIFIER_RE = re.compile(r"[A-Za-z0-9_-]{1,64}")  # keeps every save/load inside FSM_DIR


def validate(doc: Any) -> str:
    """Return "" for a valid table, else the reason it is invalid."""
    if not isinstance(doc, dict) or not isinstance(doc.get("states"), list) or not doc["states"]:
        return "states must be a non-empty list"
    names = set()
    for state in doc["states"]:
        if not isinstance(state, dict) or not isinstance(state.get("name"), str):
            return "every state needs a string name"
        if state["name"] in names:
            return f"duplicate state {state['name']!r}"
        names.add(state["name"])
    for state in doc["states"]:
        where = f"state {state['name']!r}"
        target = state.get("target")
        if (not isinstance(target, dict) or not isinstance(target.get("joint_names"), list)
                or not isinstance(target.get("positions"), list)
                or len(target["joint_names"]) != len(target["positions"])
                or not all(isinstance(n, str) for n in target["joint_names"])
                or not all(isinstance(p, (int, float)) and not isinstance(p, bool) for p in target["positions"])):
            return f"{where}: target needs equal-length joint_names (strings) and positions (numbers)"
        epsilon = state.get("epsilon", 0.0)
        if not isinstance(epsilon, (int, float)) or isinstance(epsilon, bool) or epsilon < 0:
            return f"{where}: epsilon must be a number >= 0"
        transitions = state.get("transitions", [])
        if not isinstance(transitions, list):
            return f"{where}: transitions must be a list"
        seen = set()
        for t in transitions:
            if not isinstance(t, dict) or not isinstance(t.get("name"), str) or t["name"] in seen:
                return f"{where}: transition names must be unique strings"
            if t.get("next_state") not in names:
                return f"{where}: transition {t['name']!r} goes to unknown state {t.get('next_state')!r}"
            seen.add(t["name"])
        if state.get("on_arrival") is not None and state["on_arrival"] not in seen:
            return f"{where}: on_arrival {state['on_arrival']!r} is not one of its transitions"
    if doc.get("initial_state") not in names:
        return f"initial_state {doc.get('initial_state')!r} is not a state"
    return ""


class FiniteStateMachine:
    def __init__(self, registry: Registry, fsm_dir: str = FSM_DIR) -> None:
        self.registry = registry
        self.fsm_dir = fsm_dir
        self.table: dict | None = None
        self.current: dict | None = None
        self.entered_at = time.monotonic()
        self.last_transition: str | None = None
        self.running = False
        self.positions: dict[str, float] = {}
        registry.subscribe(self, JOINT_STATES_TOPIC)
        for name, handler in (("update_logic", self.update_logic), ("get_logic", self.get_logic),
                              ("set_running", self.set_running), ("fire_transition", self.fire_transition),
                              ("save_to_file", self.save_to_file), ("load_from_file", self.load_from_file),
                              ("list_saved", self.list_saved)):
            registry.register_handler(f"/fsm/{name}", handler)

    # -- Connection protocol (for registry.subscribe) -----------------------

    def send(self, message: dict) -> None:
        if message.get("topic") == JOINT_STATES_TOPIC:
            msg = message.get("msg") or {}
            self.positions = dict(zip(msg.get("name") or [], msg.get("position") or []))

    # -- running the table --------------------------------------------------------

    def _enter(self, name: str) -> None:
        self.current = next(s for s in self.table["states"] if s["name"] == name)
        self.entered_at = time.monotonic()
        target = self.current["target"]
        self.registry.publish(TRAJECTORY_TOPIC, {
            "joint_names": target["joint_names"],
            "points": [{"positions": target["positions"], "velocities": [], "accelerations": [],
                        "time_from_start": {"sec": 0, "nanosec": 0}}],
        })

    def _take(self, transition: str) -> bool:
        for t in self.current.get("transitions", []):
            if t["name"] == transition:
                self.last_transition = transition
                self._enter(t["next_state"])
                return True
        return False

    def _arrived(self) -> bool:
        target, epsilon = self.current["target"], self.current.get("epsilon", 0.0)
        return all(name in self.positions and abs(self.positions[name] - want) <= epsilon
                   for name, want in zip(target["joint_names"], target["positions"]))

    def step(self) -> None:
        if self.running and self.current is not None and self.current.get("on_arrival") and self._arrived():
            self._take(self.current["on_arrival"])

    def publish_status(self) -> None:
        self.registry.publish(STATUS_TOPIC, {
            "current_state": self.current["name"] if self.current else None,
            "time_in_state": round(time.monotonic() - self.entered_at, 3),
            "last_transition": self.last_transition,
            "running": self.running,
        })

    async def run(self) -> None:
        while True:
            self.step()
            self.publish_status()
            await asyncio.sleep(INTERVAL_S)

    # -- services -------------------------------------------------------------------

    def update_logic(self, args: Any) -> tuple[bool, dict, str]:
        error = validate(args)
        if error:
            return False, {}, error
        self.table = {"initial_state": args["initial_state"], "states": args["states"]}
        self.last_transition = None
        self.running = True
        self._enter(args["initial_state"])
        return True, {"current_state": self.current["name"]}, ""

    def get_logic(self, args: Any) -> tuple[bool, dict, str]:
        if self.table is None:
            return False, {}, "no table loaded"
        return True, self.table, ""

    def set_running(self, args: Any) -> tuple[bool, dict, str]:
        data = args.get("data") if isinstance(args, dict) else None
        if data is not None:
            if not isinstance(data, bool):
                return False, {"running": self.running}, "data must be true or false"
            self.running = data
        return True, {"running": self.running}, ""

    def fire_transition(self, args: Any) -> tuple[bool, dict, str]:
        if not self.running or self.current is None:
            return False, {}, "not running"
        name = args.get("transition") if isinstance(args, dict) else None
        if not self._take(name):
            return False, {}, f"state {self.current['name']!r} has no transition {name!r}"
        return True, {"current_state": self.current["name"]}, ""

    def _path(self, args: Any) -> str | None:
        identifier = args.get("identifier") if isinstance(args, dict) else None
        if not isinstance(identifier, str) or not IDENTIFIER_RE.fullmatch(identifier):
            return None
        return os.path.join(self.fsm_dir, identifier + ".json")

    def save_to_file(self, args: Any) -> tuple[bool, dict, str]:
        path = self._path(args)
        if path is None:
            return False, {}, "identifier must match [A-Za-z0-9_-]{1,64}"
        os.makedirs(self.fsm_dir, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"initial_state": args.get("initial_state"), "states": args.get("states")}, f, indent=2)
        return True, {"path": os.path.relpath(path)}, ""

    def load_from_file(self, args: Any) -> tuple[bool, dict, str]:
        path = self._path(args)
        if path is None:
            return False, {}, "identifier must match [A-Za-z0-9_-]{1,64}"
        try:
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
        except (OSError, ValueError) as exc:
            return False, {}, f"cannot load {args['identifier']!r}: {exc}"
        if not isinstance(doc, dict):
            return False, {}, f"{args['identifier']!r} does not hold an FSM document"
        return True, {"initial_state": doc.get("initial_state"), "states": doc.get("states")}, ""

    def list_saved(self, args: Any) -> tuple[bool, dict, str]:
        names = os.listdir(self.fsm_dir) if os.path.isdir(self.fsm_dir) else []
        ids = sorted(n[:-5] for n in names if n.endswith(".json") and IDENTIFIER_RE.fullmatch(n[:-5]))
        return True, {"identifiers": ids}, ""
