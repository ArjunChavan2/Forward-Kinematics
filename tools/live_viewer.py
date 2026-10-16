"""Live 3D viewer for `make demo`: draws the robot from /xform_world as it moves, with a slider per joint.

Not part of the graded submission. A browser cannot open the gateway's raw TCP socket, so this serves the
page (live_viewer.html) and bridges the gateway to it:
- a background thread subscribes /xform_world, /joint_states, and /fsm/status, and the page reads the
  latest of each through Server-Sent Events;
- the page's sliders POST setpoints here, which go out on /joint_trajectory (the choreography is stopped
  first, so it does not fight the sliders).
Every link pose comes from /xform_world. robot_description only supplies shapes, joint limits, and
joint origins (where each link's body reaches toward its children).

Usage (with `make demo` running):
    python3 tools/live_viewer.py                            # load the three-link arm + its choreography
    python3 tools/live_viewer.py robot.urdf fsm/table.json  # load another robot + choreography
    python3 tools/live_viewer.py 8804                       # on another port (default 8803)
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.join(HERE, "..", "tests"))

from client_helper import Client  # noqa: E402
from finite_state_machine import FSM_DIR, IDENTIFIER_RE, validate  # noqa: E402
from urdf import parse_urdf  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 8803
DEFAULT_ROBOT = os.path.join(HERE, "..", "robots", "dual_arm_robot.urdf")
DEFAULT_FSM = os.path.join(HERE, "..", "fsm", "OrangeJustice.json")
TOPICS = {"/xform_world": "xform", "/joint_states": "joints", "/fsm/status": "fsm"}
latest: dict[str, str] = {}  # topic -> latest msg as JSON text
current_choreo: str | None = None  # the identifier (fsm/<id>.json) last loaded from the picker or command line
unsaved: dict[str, dict] = {}  # identifier -> edited table not yet saved, so switching choreos keeps the edits
UNSAVED_PATH = os.path.join(HERE, "..", ".unsaved_choreos.json")  # `unsaved` on disk, so a restart keeps it too


def store_unsaved() -> None:
    tmp = UNSAVED_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(unsaved, f, indent=1)
    os.replace(tmp, UNSAVED_PATH)  # never leaves a half-written file


def loaded_table() -> dict:
    reply = control.service("/fsm/get_logic", {})
    return reply["values"] if reply.get("result") else {}


def rev_of(table) -> str:
    return hashlib.sha1(json.dumps(table, sort_keys=True).encode()).hexdigest()[:12]


def table_rev() -> str:
    """Fingerprint of the loaded choreography table; pages send back the one they edited, so a stale page
    cannot overwrite a change made elsewhere (another tab, a script)."""
    return rev_of(loaded_table())


def watch_table() -> None:
    """Publish table_rev() on the page stream whenever it changes, so open pages reload the table."""
    while True:
        try:
            rev = json.dumps(table_rev())
            if latest.get("table") != rev:  # a new string only on change, so the stream sends it once
                latest["table"] = rev
        except (OSError, ConnectionError, ValueError, KeyError):
            pass
        time.sleep(0.5)


STALE = {"result": False, "values": {"stale": True},
         "status": "the choreography changed elsewhere; reloaded it, so redo that change"}


def bridge() -> None:
    """Keep the latest message of each topic; reconnect if the gateway restarts."""
    while True:
        try:
            with Client() as client:
                for topic in TOPICS:
                    client.subscribe(topic)
                while True:
                    m = client.recv_line(timeout=30)
                    if m.get("op") == "publish":
                        latest[m["topic"]] = json.dumps(m["msg"])
        except (OSError, ConnectionError, ValueError):
            time.sleep(0.5)


class Control:
    """One gateway connection for the sliders, shared by the HTTP threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._client: Client | None = None

    def _call(self, fn):
        with self._lock:
            for attempt in range(2):  # reconnect once if the gateway restarted
                try:
                    if self._client is None:
                        self._client = Client()
                        self._client.advertise("/joint_trajectory")
                    return fn(self._client)
                except OSError:
                    self._client = None
                    if attempt:
                        raise

    def service(self, name: str, args: dict) -> dict:
        return self._call(lambda c: c.call_service(name, args))

    def setpoint(self, names: list, positions: list) -> None:
        self.service("/fsm/set_running", {"data": False})
        self._call(lambda c: c.publish("/joint_trajectory", {
            "joint_names": names,
            "points": [{"positions": positions, "velocities": [], "accelerations": [],
                        "time_from_start": {"sec": 0, "nanosec": 0}}]}))


control = Control()


def _three(text: str | None) -> list[float]:
    values = [float(v) for v in (text or "0 0 0").split()]
    return values if len(values) == 3 and all(math.isfinite(v) for v in values) else [0.0, 0.0, 0.0]


def _rgb(material, named: dict) -> list[float] | None:
    if material is None:
        return None
    color = material.find("color")
    if color is None:
        return named.get(material.get("name"))
    try:
        rgba = [float(v) for v in color.get("rgba", "").split()]
    except ValueError:
        return None
    return rgba[:3] if len(rgba) == 4 and all(0 <= v <= 1 for v in rgba) else None


def link_visuals(text: str) -> dict:
    """{link: [{"xyz", "rpy", "type", "size", "color"?}, ...]} for every box, cylinder, or sphere <visual>.
    color is the rgb of the visual's <material> (inline, or named at <robot> level). Meshes are skipped (their
    files are not available here); a link with none of these shapes gets a drawn body instead."""
    robot = ET.fromstring(text)
    named = {m.get("name"): rgb for m in robot.findall("material") if (rgb := _rgb(m, {})) is not None}
    visuals = {}
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            shape = geometry[0] if geometry is not None and len(geometry) else None
            if shape is None or shape.tag not in ("box", "cylinder", "sphere"):
                continue
            origin = visual.find("origin")
            try:
                size = (_three(shape.get("size")) if shape.tag == "box"
                        else [float(shape.get("radius")), float(shape.get("length", 0))])
            except (TypeError, ValueError):
                continue
            entry = {"xyz": _three(origin.get("xyz") if origin is not None else None),
                     "rpy": _three(origin.get("rpy") if origin is not None else None),
                     "type": shape.tag, "size": size}
            if (rgb := _rgb(visual.find("material"), named)) is not None:
                entry["color"] = rgb
            visuals.setdefault(link.get("name"), []).append(entry)
    return visuals


def choreos() -> dict:
    """{"current": id, "choreos": {id: [joint names it moves]}} for every valid table saved in fsm/."""
    found = {}
    for ident in control.service("/fsm/list_saved", {})["values"].get("identifiers", []):
        reply = control.service("/fsm/load_from_file", {"identifier": ident})
        if reply.get("result") and not validate(reply["values"]):
            found[ident] = sorted({n for s in reply["values"]["states"] for n in s["target"]["joint_names"]})
    return {"current": current_choreo, "choreos": found, "unsaved": sorted(unsaved)}


def load_choreo(ident) -> dict:
    """Load fsm/<ident>.json into the state machine; it starts running from its initial state."""
    global current_choreo
    if ident in unsaved:
        reply = control.service("/fsm/update_logic", unsaved[ident])
    else:
        reply = control.service("/fsm/load_from_file", {"identifier": ident})
        if reply.get("result"):
            reply = control.service("/fsm/update_logic", reply["values"])
    if reply.get("result"):
        current_choreo = ident
    return reply


def edit_choreo(table, state, fresh: bool) -> dict:
    """Load an edited table paused on `state`, so the robot holds the pose being edited."""
    global current_choreo
    reply = control.service("/fsm/update_logic", table)
    if reply.get("result"):
        control.service("/fsm/set_running", {"data": False})
        control.service("/fsm/set_state", {"state": state})
        if fresh:
            current_choreo = None
        elif current_choreo is not None:
            unsaved[current_choreo] = table
            store_unsaved()
    return reply


def save_choreo(ident, table) -> dict:
    """Write fsm/<ident>.json (save_to_file checks the identifier; it does not check the table, so this does)."""
    global current_choreo
    error = validate(table)
    if error:
        return {"result": False, "values": {}, "status": error}
    reply = control.service("/fsm/save_to_file", {**table, "identifier": ident})
    if reply.get("result"):
        current_choreo = ident
        if unsaved.pop(ident, None) is not None:
            store_unsaved()
    return reply


def delete_choreo(ident) -> dict:
    """Move fsm/<ident>.json into fsm/.deleted/ (recoverable; list_saved skips it). The loaded table stays."""
    global current_choreo
    if not isinstance(ident, str) or not IDENTIFIER_RE.fullmatch(ident):
        return {"result": False, "values": {}, "status": "identifier must match [A-Za-z0-9_-]{1,64}"}
    path = os.path.join(FSM_DIR, ident + ".json")
    if not os.path.isfile(path):
        return {"result": False, "values": {}, "status": f"no saved choreo {ident!r}"}
    trash = os.path.join(FSM_DIR, ".deleted")
    os.makedirs(trash, exist_ok=True)
    os.replace(path, os.path.join(trash, f"{ident}-{time.strftime('%Y%m%d-%H%M%S')}.json"))
    if unsaved.pop(ident, None) is not None:
        store_unsaved()
    if current_choreo == ident:
        current_choreo = None  # still loaded, now shown as "new (not saved)"
    return {"result": True, "values": {}, "status": ""}


def robot_json() -> bytes:
    reply = control.service("/param_server/get_param", {"name": "robot_description"})
    text = reply["values"].get("value")
    try:
        model = parse_urdf(text)
    except (ValueError, TypeError):
        return b'{"root": null, "links": [], "joints": [], "visuals": {}}'
    joints = [{"name": j.name, "type": j.joint_type, "parent": j.parent, "child": j.child, "xyz": list(j.origin_xyz), "axis": list(j.axis),
               "lower": j.limit.lower if j.limit else -math.pi, "upper": j.limit.upper if j.limit else math.pi}
              for j in model.joints]
    return json.dumps({"root": model.root_link, "links": [link.name for link in model.links],
                       "joints": joints, "visuals": link_visuals(text)}).encode()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/stream":
            return self.stream()
        if self.path == "/robot":
            return self.reply(robot_json(), "application/json")
        if self.path == "/choreos":
            return self.reply(json.dumps(choreos()).encode(), "application/json")
        if self.path == "/fsm":  # the loaded choreography table, {} when none; X-Table-Rev is its table_rev()
            table = loaded_table()
            return self.reply(json.dumps(table).encode(), "application/json", {"X-Table-Rev": rev_of(table)})
        with open(os.path.join(HERE, "live_viewer.html"), "rb") as f:
            self.reply(f.read(), "text/html; charset=utf-8")

    def do_POST(self) -> None:  # noqa: N802
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        if self.path == "/setpoint":  # {"name": [...], "position": [...]}
            control.setpoint(body["name"], [float(p) for p in body["position"]])
            result = {}
        elif self.path == "/running":  # {"data": true|false}
            result = control.service("/fsm/set_running", {"data": bool(body.get("data"))})
        elif self.path == "/speed":  # {"data": rate}, or {} to read it: playback rate of every move
            result = control.service("/joint_state_publisher/set_speed", body)
        elif self.path == "/state":  # {"state": name}: pause the choreography on that state
            control.service("/fsm/set_running", {"data": False})
            result = control.service("/fsm/set_state", {"state": body.get("state")})
        elif self.path == "/choreo":  # {"id": identifier}: switch to fsm/<identifier>.json
            result = load_choreo(body.get("id"))
        elif self.path == "/edit":  # {"table", "state", "fresh", "base_rev"}: load an edited table, paused on that state
            # base_rev (the X-Table-Rev the page loaded) is required unless fresh, so a page loaded before a change
            # elsewhere, or an old page without this check, cannot silently put back an older table
            stale = not body.get("fresh") and body.get("base_rev") != table_rev()
            result = STALE if stale else edit_choreo(body.get("table"), body.get("state"), bool(body.get("fresh")))
        elif self.path == "/save":  # {"id", "table", "base_rev"}: write fsm/<id>.json
            stale = body.get("base_rev") != table_rev()
            result = STALE if stale else save_choreo(body.get("id"), body.get("table"))
        elif self.path == "/delete":  # {"id"}: move fsm/<id>.json to fsm/.deleted/
            result = delete_choreo(body.get("id"))
        elif self.path == "/reset":
            control.service("/fsm/set_running", {"data": False})
            result = control.service("/joint_state_publisher/reset", {})
        else:
            self.send_error(404)
            return
        self.reply(json.dumps(result).encode(), "application/json")

    def reply(self, body: bytes, kind: str, headers: dict | None = None) -> None:
        self.send_response(200)
        self.send_header("Content-Type", kind)
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        sent: dict[str, str] = {}
        try:
            while True:
                for topic, event in [*TOPICS.items(), ("table", "table")]:
                    data = latest.get(topic)
                    if data is not None and data is not sent.get(topic):
                        self.wfile.write(f"event: {event}\ndata: {data}\n\n".encode())
                        sent[topic] = data
                self.wfile.flush()
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt: str, *args) -> None:
        pass


def main() -> None:
    args = sys.argv[1:]
    port = int(args.pop()) if args and args[-1].isdigit() else DEFAULT_PORT
    urdf_path, fsm_path = (args + [None, None])[:2] if args else (DEFAULT_ROBOT, DEFAULT_FSM)
    if urdf_path:
        with open(urdf_path, encoding="utf-8") as f:
            reply = control.service("/param_server/set_param", {"name": "robot_description", "value": f.read()})
        print("set robot_description:", reply["result"], flush=True)
        time.sleep(0.5)  # let joint_state_publisher pick up the robot before the first setpoint
    if fsm_path:
        with open(fsm_path, encoding="utf-8") as f:
            reply = control.service("/fsm/update_logic", json.load(f))
        if reply["result"]:
            global current_choreo
            current_choreo = os.path.splitext(os.path.basename(fsm_path))[0]
        print("update_logic:", reply["result"], reply["status"] or reply["values"], flush=True)
    if os.path.exists(UNSAVED_PATH):
        with open(UNSAVED_PATH, encoding="utf-8") as f:
            unsaved.update(json.load(f))
        print("unsaved edits kept from last run:", sorted(unsaved), flush=True)
    threading.Thread(target=bridge, daemon=True).start()
    threading.Thread(target=watch_table, daemon=True).start()
    server = ThreadingHTTPServer((HOST, port), Handler)
    print(f"Serving at http://{HOST}:{port}/  (Ctrl-C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
