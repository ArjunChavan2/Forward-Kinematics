"""Local 3D web viewer: parse a URDF, compute poses with the TEMPORARY tools/demo_fk.py, and
render the robot's links/joints with three.js.

Not part of the graded submission. Poses come from demo_fk.py, not src/fk.py (which the owner
will write themselves) -- replace the import below once that exists.

Usage:
    python3 tools/urdf_viewer_3d.py              # built-in sample robot, serves on 127.0.0.1:8801
    python3 tools/urdf_viewer_3d.py path/to.urdf
    python3 tools/urdf_viewer_3d.py path/to.urdf 8802
"""
from __future__ import annotations

import html
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from demo_fk import compute_link_poses  # noqa: E402
from transform import transform_to_pose  # noqa: E402
from urdf import RobotModel, parse_urdf  # noqa: E402
from urdf_demo import SAMPLE_ROBOT  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 8801


def build_scene_data(model: RobotModel) -> dict:
    """{"links": [{"name", "position" [x,y,z], "quaternion" [x,y,z,w]}], "bones": [[parent, child]]}"""
    poses = compute_link_poses(model)
    links = []
    for link in model.links:
        translation, quat_wxyz = transform_to_pose(poses[link.name])
        w, x, y, z = quat_wxyz
        links.append({"name": link.name, "position": list(translation), "quaternion": [x, y, z, w]})
    bones = [[j.parent, j.child, j.name, j.joint_type] for j in model.joints]
    return {"links": links, "bones": bones}


PAGE_TEMPLATE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>URDF 3D viewer</title>
<style>
  body {{ margin: 0; font-family: -apple-system, system-ui, sans-serif; background: #111; color: #eee; }}
  #overlay {{ position: absolute; top: 0; left: 0; padding: 0.75rem 1rem; font-size: 0.8rem; }}
  #overlay h1 {{ font-size: 1rem; margin: 0 0 0.25rem; }}
  #overlay .note {{ color: #999; }}
  canvas {{ display: block; }}
</style>
</head>
<body>
<div id="overlay">
  <h1>{title}</h1>
  <div class="note">zero-configuration FK, demo_fk.py (temporary, not src/fk.py) &mdash; drag to orbit, scroll to zoom</div>
</div>
<script type="importmap">
{{
  "imports": {{
    "three": "https://cdn.jsdelivr.net/npm/three@0.149.0/build/three.module.js"
  }}
}}
</script>
<script type="module">
import * as THREE from "three";
import {{ OrbitControls }} from "https://cdn.jsdelivr.net/npm/three@0.149.0/examples/jsm/controls/OrbitControls.js";

const data = {data_json};

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
const renderer = new THREE.WebGLRenderer({{antialias: true}});
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

scene.add(new THREE.GridHelper(2, 20, 0x444444, 0x2a2a2a));
scene.add(new THREE.AxesHelper(0.3));
scene.add(new THREE.AmbientLight(0xffffff, 0.8));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
dirLight.position.set(1, 2, 1);
scene.add(dirLight);

const positions = {{}};
for (const link of data.links) {{
  positions[link.name] = link.position;
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 16, 16),
    new THREE.MeshStandardMaterial({{color: 0x4da6ff}})
  );
  sphere.position.set(link.position[0], link.position[1], link.position[2]);
  scene.add(sphere);

  const axes = new THREE.AxesHelper(0.08);
  axes.position.copy(sphere.position);
  axes.quaternion.set(link.quaternion[0], link.quaternion[1], link.quaternion[2], link.quaternion[3]);
  scene.add(axes);
}}

const boneColors = {{revolute: 0xffaa00, continuous: 0xffaa00, prismatic: 0x55ff55, fixed: 0x888888}};
for (const [parent, child, name, type] of data.bones) {{
  const a = positions[parent], b = positions[child];
  const geom = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(a[0], a[1], a[2]),
    new THREE.Vector3(b[0], b[1], b[2]),
  ]);
  const line = new THREE.Line(geom, new THREE.LineBasicMaterial({{color: boneColors[type] || 0xffffff}}));
  scene.add(line);
}}

// fit the camera to the robot's extent
let maxDist = 0.3;
for (const p of Object.values(positions)) {{
  maxDist = Math.max(maxDist, Math.hypot(p[0], p[1], p[2]));
}}
camera.position.set(maxDist * 1.5, maxDist * 1.2, maxDist * 1.5);
camera.lookAt(0, 0, 0);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);

window.addEventListener("resize", () => {{
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}});

function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}}
animate();
</script>
</body>
</html>
"""


def render_page(model: RobotModel, title: str) -> bytes:
    data = build_scene_data(model)
    return PAGE_TEMPLATE.format(title=html.escape(title), data_json=json.dumps(data)).encode("utf-8")


def make_handler(page: bytes):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def log_message(self, fmt: str, *args) -> None:
            pass

    return Handler


def main() -> None:
    args = sys.argv[1:]
    port = DEFAULT_PORT
    path = None
    for arg in args:
        if arg.isdigit():
            port = int(arg)
        else:
            path = arg

    if path is None:
        text = SAMPLE_ROBOT
        title = "URDF 3D (built-in sample robot)"
    else:
        with open(path, encoding="utf-8") as f:
            text = f.read()
        title = f"URDF 3D ({path})"

    try:
        model = parse_urdf(text)
    except ValueError as exc:
        print(f"rejected: {exc}")
        sys.exit(1)

    page = render_page(model, title)
    server = ThreadingHTTPServer((HOST, port), make_handler(page))
    print(f"Serving at http://{HOST}:{port}/  (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
