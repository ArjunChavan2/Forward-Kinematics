"""Live 3D viewer for `make demo`: draws every link from /xform_world as the robot moves.

Not part of the graded submission. A browser cannot open the gateway's raw TCP socket, so this serves the
page and bridges the gateway to it: a background thread subscribes /xform_world and /fsm/status, and the
page reads the latest of each through Server-Sent Events. Link shapes come from robot_description (a
sphere + axes per link, a line per joint); every pose comes from /xform_world.

Usage (with `make demo` running):
    python3 tools/live_viewer.py                                     # view whatever robot is loaded
    python3 tools/live_viewer.py robot.urdf fsm/fetch_wave.json      # load a robot + choreography first
    python3 tools/live_viewer.py robot.urdf fsm/fetch_wave.json 8804 # on another port (default 8803)
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.join(HERE, "..", "tests"))

from client_helper import Client  # noqa: E402
from urdf import parse_urdf  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 8803
latest: dict[str, str] = {}  # topic -> latest msg as JSON text


def bridge() -> None:
    """Keep the latest /xform_world and /fsm/status; reconnect if the gateway restarts."""
    while True:
        try:
            with Client() as client:
                client.subscribe("/xform_world")
                client.subscribe("/fsm/status")
                while True:
                    m = client.recv_line(timeout=30)
                    if m.get("op") == "publish":
                        latest[m["topic"]] = json.dumps(m["msg"])
        except (OSError, ConnectionError, ValueError):
            time.sleep(0.5)


def robot_json() -> bytes:
    with Client() as client:
        reply = client.call_service("/param_server/get_param", {"name": "robot_description"})
    try:
        model = parse_urdf(reply["values"]["value"])
    except (ValueError, TypeError, KeyError):
        return b'{"links": [], "bones": []}'
    return json.dumps({"links": [link.name for link in model.links],
                       "bones": [[j.parent, j.child, j.joint_type] for j in model.joints]}).encode()


PAGE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Live FK viewer</title>
<style>
  body { margin: 0; font-family: -apple-system, system-ui, sans-serif; background: #111; color: #eee; }
  #overlay { position: absolute; top: 0; left: 0; padding: 0.75rem 1rem; font-size: 0.85rem; }
  #overlay h1 { font-size: 1rem; margin: 0 0 0.25rem; }
  #overlay .note { color: #999; }
  #credit { position: absolute; bottom: 0; left: 0; padding: 0.5rem 1rem; font-size: 0.75rem; color: #888; }
  canvas { display: block; }
</style>
</head>
<body>
<div id="overlay">
  <h1>Live FK viewer</h1>
  <div id="state">waiting for /xform_world&hellip;</div>
  <div class="note">every link drawn from /xform_world &mdash; drag to orbit, scroll to zoom</div>
</div>
<div id="credit"></div>
<script type="importmap">
{ "imports": { "three": "https://cdn.jsdelivr.net/npm/three@0.149.0/build/three.module.js" } }
</script>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.149.0/examples/jsm/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.01, 100);
camera.position.set(2.2, 1.6, 2.2);
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.6, 0);

scene.add(new THREE.GridHelper(4, 40, 0x444444, 0x2a2a2a));
scene.add(new THREE.AmbientLight(0xffffff, 0.8));
const sun = new THREE.DirectionalLight(0xffffff, 0.6);
sun.position.set(1, 2, 1);
scene.add(sun);

// global_frame is z-up; three.js is y-up. The correction lives here in the viewer, never in /xform_world.
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);

const boneColors = {revolute: 0xffaa00, continuous: 0xffaa00, prismatic: 0x55ff55, fixed: 0x888888};
let links = {}, bones = [], linkKey = "";

async function loadRobot() {
  const robot = await (await fetch("/robot")).json();
  world.clear();
  links = {};
  for (const name of robot.links) {
    const obj = new THREE.Object3D();
    obj.matrixAutoUpdate = false;
    obj.add(new THREE.Mesh(new THREE.SphereGeometry(0.025, 12, 12),
                           new THREE.MeshStandardMaterial({color: 0x4da6ff})));
    obj.add(new THREE.AxesHelper(0.07));
    world.add(obj);
    links[name] = obj;
  }
  bones = robot.bones.filter(([p, c]) => links[p] && links[c]).map(([parent, child, type]) => {
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
                                new THREE.LineBasicMaterial({color: boneColors[type] || 0xffffff}));
    world.add(line);
    return {parent, child, line};
  });
  document.getElementById("credit").textContent = robot.links.includes("shoulder_pan_link")
    ? "Fetch robot description \\u00a9 Fetch Robotics, Inc., CC BY-NC-SA 4.0." : "";
}

const a = new THREE.Vector3(), b = new THREE.Vector3();
let loading = false;
async function onXform(msg) {
  if (loading) return;  // drop frames while a new robot's links are being built
  const key = msg.transforms.map(e => e.child_frame_id).sort().join(",");
  if (key !== linkKey) {  // a new robot was loaded
    loading = true;
    try { await loadRobot(); linkKey = key; } finally { loading = false; }
  }
  for (const e of msg.transforms) {
    const obj = links[e.child_frame_id];
    if (obj) { obj.matrix.fromArray(e.matrix); obj.matrixWorldNeedsUpdate = true; }  // column-major, as sent
  }
  for (const {parent, child, line} of bones) {
    a.setFromMatrixPosition(links[parent].matrix);
    b.setFromMatrixPosition(links[child].matrix);
    line.geometry.setFromPoints([a, b]);
  }
}

let fsm = null;
const events = new EventSource("/stream");
events.addEventListener("xform", e => onXform(JSON.parse(e.data)));
events.addEventListener("fsm", e => { fsm = JSON.parse(e.data); });
setInterval(() => {
  const n = Object.keys(links).length;
  document.getElementById("state").textContent = fsm
    ? `${n} links | state: ${fsm.current_state} (${fsm.time_in_state.toFixed(1)} s) | ${fsm.running ? "running" : "stopped"}`
    : `${n} links | no FSM status (is make demo running?)`;
}, 100);

addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
(function animate() { requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); })();
</script>
</body>
</html>
""".encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/stream":
            return self.stream()
        body, kind = (robot_json(), "application/json") if self.path == "/robot" else (PAGE, "text/html; charset=utf-8")
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
                for topic, event in (("/xform_world", "xform"), ("/fsm/status", "fsm")):
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
    urdf_path, fsm_path = (args + [None, None])[:2]
    with Client() as client:
        if urdf_path:
            with open(urdf_path, encoding="utf-8") as f:
                print("set robot_description:", client.call_service(
                    "/param_server/set_param", {"name": "robot_description", "value": f.read()})["result"])
            time.sleep(0.5)  # let joint_state_publisher pick up the robot before the first setpoint
        if fsm_path:
            with open(fsm_path, encoding="utf-8") as f:
                reply = client.call_service("/fsm/update_logic", json.load(f))
            print("update_logic:", reply["result"], reply["status"] or reply["values"])
    threading.Thread(target=bridge, daemon=True).start()
    server = ThreadingHTTPServer((HOST, port), Handler)
    print(f"Serving at http://{HOST}:{port}/  (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
