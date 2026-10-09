"""Local web viewer: parse a URDF and show its link/joint tree in a browser.

Not part of the graded submission. Shows parse_urdf's output as a structure (no poses, no FK math --
fk.py isn't built yet, so this never computes a transform, only displays what urdf.py parsed).

Usage:
    python3 tools/urdf_viewer.py              # built-in sample robot, serves on 127.0.0.1:8800
    python3 tools/urdf_viewer.py path/to.urdf
    python3 tools/urdf_viewer.py path/to.urdf 8801
"""
from __future__ import annotations

import html
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from urdf import RobotModel, parse_urdf  # noqa: E402
from urdf_demo import SAMPLE_ROBOT  # noqa: E402

HOST = "127.0.0.1"
DEFAULT_PORT = 8800


def build_tree(model: RobotModel) -> dict:
    """Build a nested {link: {joints: [...]}} structure from parent/child names. No FK math: this is
    bookkeeping over names only, the same role as KinEval's initRobotLinks/initRobotJoints."""
    joints_by_parent: dict[str, list] = {}
    for joint in model.joints:
        joints_by_parent.setdefault(joint.parent, []).append(joint)

    def node(link_name: str) -> dict:
        children = []
        for joint in joints_by_parent.get(link_name, []):
            children.append({
                "joint": {
                    "name": joint.name,
                    "type": joint.joint_type,
                    "origin_xyz": joint.origin_xyz,
                    "origin_rpy": joint.origin_rpy,
                    "axis": joint.axis,
                    "limit": None if joint.limit is None else [joint.limit.lower, joint.limit.upper],
                },
                "link": node(joint.child),
            })
        return {"name": link_name, "children": children}

    return node(model.root_link)


PAGE_TEMPLATE = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>URDF tree viewer</title>
<style>
  body {{ font-family: -apple-system, system-ui, sans-serif; margin: 2rem; color: #1a1a1a; background: #fafafa; }}
  h1 {{ font-size: 1.1rem; margin-bottom: 0.25rem; }}
  .note {{ color: #666; font-size: 0.85rem; margin-bottom: 1.5rem; }}
  ul {{ list-style: none; padding-left: 1.25rem; border-left: 1px solid #ddd; }}
  li {{ margin: 0.4rem 0; }}
  .link {{ font-weight: 600; }}
  .joint {{ color: #555; font-size: 0.85rem; margin: 0.15rem 0 0.15rem 0; }}
  .joint-type {{ display: inline-block; padding: 0 0.4rem; border-radius: 3px; background: #eef; margin-right: 0.3rem; }}
  code {{ background: #f0f0f0; padding: 0 0.25rem; border-radius: 3px; }}
</style>
</head>
<body>
<h1>{title}</h1>
<p class="note">Link/joint tree only &mdash; no poses. fk.py is not built yet.</p>
<div id="tree"></div>
<script>
const data = {data_json};
function renderLink(link) {{
  const li = document.createElement("li");
  const nameEl = document.createElement("div");
  nameEl.className = "link";
  nameEl.textContent = link.name;
  li.appendChild(nameEl);
  if (link.children.length > 0) {{
    const ul = document.createElement("ul");
    for (const child of link.children) {{
      const jointLi = document.createElement("li");
      const j = child.joint;
      const jointEl = document.createElement("div");
      jointEl.className = "joint";
      const limitText = j.limit ? ` limit=[${{j.limit[0]}}, ${{j.limit[1]}}]` : "";
      jointEl.innerHTML = `<span class="joint-type">${{j.type}}</span>` +
        `<code>${{j.name}}</code> origin_xyz=${{JSON.stringify(j.origin_xyz)}} ` +
        `origin_rpy=${{JSON.stringify(j.origin_rpy)}} axis=${{JSON.stringify(j.axis)}}${{limitText}}`;
      jointLi.appendChild(jointEl);
      const childUl = document.createElement("ul");
      childUl.appendChild(renderLink(child.link));
      jointLi.appendChild(childUl);
      ul.appendChild(jointLi);
    }}
    li.appendChild(ul);
  }}
  return li;
}}
const root = document.createElement("ul");
root.appendChild(renderLink(data));
document.getElementById("tree").appendChild(root);
</script>
</body>
</html>
"""


def render_page(model: RobotModel, title: str) -> bytes:
    tree = build_tree(model)
    return PAGE_TEMPLATE.format(
        title=html.escape(title),
        data_json=json.dumps(tree),
    ).encode("utf-8")


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
        title = "URDF tree (built-in sample robot)"
    else:
        with open(path, encoding="utf-8") as f:
            text = f.read()
        title = f"URDF tree ({path})"

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
