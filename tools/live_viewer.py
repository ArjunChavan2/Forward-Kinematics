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
from urdf import parse_urdf  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 8803
DEFAULT_ROBOT = os.path.join(HERE, "..", "robots", "three_link_arm.urdf")
DEFAULT_FSM = os.path.join(HERE, "..", "fsm", "three_link_wave.json")
TOPICS = {"/xform_world": "xform", "/joint_states": "joints", "/fsm/status": "fsm"}
latest: dict[str, str] = {}  # topic -> latest msg as JSON text


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


def first_visuals(text: str) -> dict:
    """{link: {"xyz", "rpy", "type", "size"}} for each link whose first <visual> is a box, cylinder, or sphere.
    Meshes are skipped (their files are not available here); those links get a drawn body instead."""
    visuals = {}
    for link in ET.fromstring(text).findall("link"):
        visual = link.find("visual")
        geometry = visual.find("geometry") if visual is not None else None
        shape = geometry[0] if geometry is not None and len(geometry) else None
        if shape is None or shape.tag not in ("box", "cylinder", "sphere"):
            continue
        origin = visual.find("origin")
        try:
            size = (_three(shape.get("size")) if shape.tag == "box"
                    else [float(shape.get("radius")), float(shape.get("length", 0))])
        except (TypeError, ValueError):
            continue
        visuals[link.get("name")] = {"xyz": _three(origin.get("xyz") if origin is not None else None),
                                     "rpy": _three(origin.get("rpy") if origin is not None else None),
                                     "type": shape.tag, "size": size}
    return visuals


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
                       "joints": joints, "visuals": first_visuals(text)}).encode()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/stream":
            return self.stream()
        if self.path == "/robot":
            return self.reply(robot_json(), "application/json")
        with open(os.path.join(HERE, "live_viewer.html"), "rb") as f:
            self.reply(f.read(), "text/html; charset=utf-8")

    def do_POST(self) -> None:  # noqa: N802
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        if self.path == "/setpoint":  # {"name": [...], "position": [...]}
            control.setpoint(body["name"], [float(p) for p in body["position"]])
            result = {}
        elif self.path == "/running":  # {"data": true|false}
            result = control.service("/fsm/set_running", {"data": bool(body.get("data"))})
        elif self.path == "/reset":
            control.service("/fsm/set_running", {"data": False})
            result = control.service("/joint_state_publisher/reset", {})
        else:
            self.send_error(404)
            return
        self.reply(json.dumps(result).encode(), "application/json")

    def reply(self, body: bytes, kind: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", kind)
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
                for topic, event in TOPICS.items():
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
        print("update_logic:", reply["result"], reply["status"] or reply["values"], flush=True)
    threading.Thread(target=bridge, daemon=True).start()
    server = ThreadingHTTPServer((HOST, port), Handler)
    print(f"Serving at http://{HOST}:{port}/  (Ctrl-C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
