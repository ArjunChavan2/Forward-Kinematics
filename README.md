# Project 3 — Forward Kinematics

EECS 367 Project 3 in Python (standard library only). A single asyncio process runs a rosbridge-style
TCP/JSON gateway on `127.0.0.1:9095` (the Project 1 transport) plus three nodes: a parameter server that
holds a URDF `robot_description`, `robot_state_publisher` (joint angles to per-joint transforms on `/tf`),
and `robot_world_state_publisher` (every link's pose in the global frame on `/xform_world`). `make demo`
adds a joint servo and a choreography state machine so a dual-arm robot can dance in a live 3D viewer.

## Running

```
make build   # python3 -m py_compile src/*.py (offline, noninteractive)
make run     # python3 src/main.py: gateway + the three graded nodes, no robot loaded; Ctrl-C / SIGTERM stops it
make demo    # python3 src/main.py --demo: make run + joint_state_publisher + finite_state_machine (not graded)
make test    # python3 tests/run_tests.py
make clean   # removes src/__pycache__ and tests/__pycache__
sh make_submission.sh   # builds submission.tar.gz (see Submitting)
```

`make run` and `make demo` start one process (`src/main.py`) that logs `gateway listening on 127.0.0.1:9095`.
The port can be overridden with `GATEWAY_PORT=<port>`; that exists for tests only, the spec port is 9095.

### End-to-end flow

All traffic is newline-delimited JSON over TCP (`spec/ROSBRIDGE_PROTOCOL.md`). Every node shares one
in-process `Registry`, so internal nodes and TCP clients use the same topics and services.

1. A client calls `/param_server/set_param` with `{"name": "robot_description", "value": "<robot>...</robot>"}`
   and gets back a per-name version.
2. `robot_state_publisher` polls `get_param` every 0.1 s. On a new version it parses and validates the URDF
   (`urdf.parse_urdf`). If it is valid it swaps the robot in; either way it then sets the parameter
   `/robot_state_publisher/description_status` (`version`, `accepted`, `error`, `loaded_version`, `root_link`).
   A rejected description leaves the previous robot in effect.
3. Clients publish `sensor_msgs/JointState` on `/joint_states`. Each message replaces the joint state.
4. Every 0.1 s `robot_state_publisher` publishes `/tf`: one parent-to-child transform per joint, from
   `fk.joint_edges`.
5. On each `/tf`, `robot_world_state_publisher` composes the edges with the latest `/global_pose`
   (identity until one arrives) using `fk.root_of_edges` / `fk.link_transforms`, and publishes
   `/xform_world`: every link's 4x4 pose in `global_frame`.

Both publishers cache their last message and the input objects it came from, so an unchanged tick
republishes it instead of redoing FK. That keeps `/tf` at about 10 Hz even with a ~4,700-joint, 1 MiB robot.

| Name | Kind | Provided by |
|---|---|---|
| `/param_server/set_param`, `/param_server/get_param` | services | `param_server.py` |
| `/robot_state_publisher/description_status` | parameter | `robot_state_publisher.py` |
| `/joint_states` | topic, subscribed (published by `joint_state_publisher` in demo) | `robot_state_publisher.py` |
| `/tf` | topic, published | `robot_state_publisher.py` |
| `/global_pose` | topic, subscribed | `robot_world_state_publisher.py` |
| `/xform_world` | topic, published | `robot_world_state_publisher.py` |
| `/joint_trajectory` | topic (demo) | `finite_state_machine.py` publishes, `joint_state_publisher.py` subscribes |
| `/joint_state_publisher/reset`, `/joint_state_publisher/set_speed` | services (demo) | `joint_state_publisher.py` |
| `/fsm/status`; `/fsm/{update_logic,get_logic,set_running,fire_transition,set_state,save_to_file,load_from_file,list_saved}` | topic + services (demo) | `finite_state_machine.py` |

### What `make demo` adds

- `joint_state_publisher` reads `robot_description` itself, servos joints toward the latest `/joint_trajectory`
  setpoints (quintic moves, `src/quintic.py`, with a playback speed set by `set_speed`), and publishes
  `/joint_states` at about 10 Hz.
- `finite_state_machine` runs a choreography table of joint-space states. On entering a state it publishes the
  target on `/joint_trajectory`; when every joint is within the state's epsilon it follows `on_arrival`. Tables
  are saved and loaded as `fsm/<identifier>.json`.
- Neither loads a robot or a table on its own. Something (normally the live viewer) must `set_param`
  `robot_description` and send a table with `/fsm/update_logic`.

### Tools (not graded)

- Live viewer, with `make demo` running: `python3 tools/live_viewer.py` loads `robots/dual_arm_robot.urdf` and
  `fsm/OrangeJustice.json`, then serves http://127.0.0.1:8803/. It draws every link from `/xform_world`, has a
  slider per joint (sent on `/joint_trajectory`), a Choreography menu over `fsm/*.json`, an editor for states,
  and a Speed slider. Other forms: `python3 tools/live_viewer.py robot.urdf fsm/table.json`, or a trailing port
  number. Known issue: its gateway client comes from `tests/client_helper.py`, whose default port has been a
  random free port since tests stopped using 9095, so on the current code it exits with
  `ConnectionRefusedError` instead of reaching `make demo` on 9095.
- `python3 tools/urdf_demo.py [path.urdf]` prints the parsed `RobotModel`.
- `python3 tools/urdf_viewer.py [path.urdf] [port]` shows the link/joint tree (default port 8800).
- `python3 tools/urdf_viewer_3d.py [path.urdf] [port]` renders zero-configuration poses with three.js (default 8801).
- `tools/video_to_choreo.py` (MediaPipe dance video to `fsm/<name>.json`) is a shelved experiment. It needs a
  separate `.venv-pose` (Python 3.12 + mediapipe, plus `pose_landmarker_heavy.task` in `.venv-pose/models/`):
  `.venv-pose/bin/python tools/video_to_choreo.py VIDEO NAME [--start S] [--end S] [--once] [--force]`. It stays
  in the repo because `tests/test_video_to_choreo.py` checks its `arm_angles` against `fk.py`.

## Layout

- `src/main.py` — entry point; wires the registry, gateway, and nodes; `--demo` adds the two demo nodes.
- `src/gateway.py` — asyncio TCP/JSON gateway (Project 1), port from `GATEWAY_PORT` or 9095.
- `src/registry.py` — in-process topic/service registry shared by the gateway and every node.
- `src/param_server.py` — `set_param` / `get_param`, versioned per name.
- `src/urdf.py` — parses and validates `robot_description` into a `RobotModel` (owner-written).
- `src/transform.py` — rotation/transform helpers and quaternion conventions (owner-written, graded exercise).
- `src/fk.py` — forward kinematics: `joint_edges`, `link_transforms`, `identity_pose`, `root_of_edges`
  (owner-written, graded exercise; contract in `spec/FK_API.md`).
- `src/robot_state_publisher.py` — `robot_description` to `description_status` and `/joint_states` to `/tf`.
- `src/robot_world_state_publisher.py` — `/tf` + `/global_pose` to `/xform_world`; only decodes and encodes, the
  composition is `fk.py`'s.
- `src/joint_state_publisher.py`, `src/finite_state_machine.py` — demo nodes; `src/quintic.py` — quintic move math.
- `tools/` — viewers and helpers above. `robots/dual_arm_robot.urdf` — the demo dance robot (27 joints).
- `fsm/` — choreographies (`OrangeJustice.json`, `scuba.json`, `sample_swing.json`); `fsm/backups/` holds earlier
  versions; the viewer's delete moves a table to `fsm/.deleted/` (not in git).
- `tests/` — unit tests per module, over-the-wire `test_integration.py`, `client_helper.py`, `run_tests.py`.
- `spec/` — project spec, wire contract, URDF format, protocol, submission notes.
- `agent-notes/` — plan and handover notes. `prompts/` — Plan/Implement/Audit/Test agent instructions.

## Testing

`make test` runs `tests/run_tests.py`, which discovers every `tests/test_*.py`. Each file also runs on its own,
e.g. `python3 tests/test_fk.py`. The grader runs the tests while its own `make run` holds 9095, so no test
binds 9095: `client_helper.PORT` is a free OS port, and `test_integration.py` starts `src/main.py` (and
`--demo`) as a subprocess with `GATEWAY_PORT` set to it. Each test file must finish in under 30 s on the grader;
keep files under ~15 s locally and check timing-sensitive ones on efficiency cores with
`taskpolicy -c background python3 tests/test_x.py` (`spec/submission_layout.md`, rules A-D).

## Submitting

`sh make_submission.sh` writes `submission.tar.gz` with `Makefile` at the archive root. It lists entries by name
(`Makefile README.md .gitignore src tests tools robots fsm`) instead of tarring `.`, which the grader rejects,
excludes caches, and checks the result. `tools/`, `robots/`, and `fsm/` are included because tests import or read
them. `prompts/`, `agent-notes/`, and `spec/` are left out. Details: `spec/submission_layout.md`.

## Status

Complete and submitted: the graded runtime, the demo nodes, and the tests are all done and passing
(`agent-notes/HANDOVER.md`'s "Next step" predates `fk.py` being written).

## Where to read more

| File | For |
|---|---|
| `spec/PROJECT3_FORWARD_KINEMATICS.md` | The assignment: architecture, Makefile rules, parameter/topic behavior, grading |
| `spec/FK_API.md` | Wire contract for every node, message format, and the demo nodes' reference behavior |
| `spec/ROBOT_DESCRIPTION.md` | Accepted URDF subset and rejection rules |
| `spec/ROSBRIDGE_PROTOCOL.md` | The TCP/JSON transport |
| `spec/submission_layout.md` | Packaging and grader test rules (free ports, `run_tests.py`, 30 s, rate on slow CPUs) |
| `agent-notes/PLAN.md` | Original plan, requirements, and design decisions |
| `agent-notes/HANDOVER.md` | Grader failures and fixes (timeouts, memory test, 5 Hz caching) |
