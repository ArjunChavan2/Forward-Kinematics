# Project 3 — Forward Kinematics

Python implementation of URDF-driven forward kinematics and a multi-node pub/sub system, running on
the Project 1 rosbridge-style TCP/JSON gateway (`127.0.0.1:9095`). Spec:
`spec/PROJECT3_FORWARD_KINEMATICS.md` (summary; verbatim spec pending), `spec/ROSBRIDGE_PROTOCOL.md`.

## Running

```
make build   # offline, noninteractive, compiles all of src/
make run     # foreground, listens on 127.0.0.1:9095, Ctrl-C / SIGTERM to stop cleanly
make test    # unit + integration tests
make demo    # make run + joint_state_publisher + finite_state_machine (not graded)
make clean   # removes __pycache__
```

## Layout

- `src/registry.py`, `src/gateway.py` — generic topic/service registry and asyncio TCP/JSON gateway, from Project 1.
- `src/main.py` — `make run`'s entry point: wires the gateway, `param_server`, `robot_state_publisher`, and
  `robot_world_state_publisher` in one asyncio process.
- `src/param_server.py` — `/param_server/set_param` and `/param_server/get_param`, versioned per name.
- `src/urdf.py`, `src/transform.py`, `src/fk.py` — URDF parsing, transform math, forward kinematics (owner-written).
- `src/robot_state_publisher.py` — loads `robot_description`, reports `/robot_state_publisher/description_status`,
  turns `/joint_states` into `/tf` at 10 Hz.
- `src/robot_world_state_publisher.py` — `/tf` + `/global_pose` → `/xform_world`.
- `tests/` — unit tests per module, `test_integration.py` over the wire against `src/main.py`, and
  `client_helper.py` (raw-socket TCP/JSON client).
- `tools/` — non-graded URDF demo and viewers. With `make demo` running,
  `python3 tools/live_viewer.py` loads the demo arm and its choreography and serves a live viewer at
  http://127.0.0.1:8803/ that draws every link from `/xform_world`, with a slider per joint.
- `robots/dual_arm_robot.urdf` — the demo robot (drives around a dance floor on x/y/spin joints; ankle roll above the treads; prismatic torso lift, waist pitch, roll and twist, two 9-joint arms: shoulder pitch, roll and yaw (so the forearms can cross), elbow pitch, roll and yaw, wrist roll, pitch and yaw; every joint name ends in how it moves; every revolute joint turns a full circle). `fsm/` — choreographies (`OrangeJustice.json`, the demo dance, posed from screenshots of the Fortnite emote; `scuba.json`: twists ±30° while one hand fans the air; `sample_swing.json`). The live viewer's Choreography menu lists every `fsm/*.json` and greys out ones that move joints the loaded robot lacks. To make one, pick "+ Create new choreo" (or "+ Duplicate" to copy the loaded one) in the Choreography menu or click a state to pause on it, pose the robot with the sliders, and use the Edit choreo box (Record pose after, Update pose, Duplicate, Delete, then, Save); drag a state onto another to swap their places. The Speed slider next to Play sets the playback rate of the whole choreography (0.25×–3×; it scales every move's duration in `joint_state_publisher`).
- `tools/video_to_choreo.py` — turns a dance video into `fsm/<name>.json` (MediaPipe pose tracking; arms solved
  from the robot's arm geometry, waist from shoulders vs hips, thigh bend as torso lift, keyframes where the
  movement pauses). One-time setup, kept out of the graded code and the system Python:
  `uv venv -p python3.12 .venv-pose && uv pip install -p .venv-pose mediapipe`, then download
  `pose_landmarker_heavy.task` from https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task
  into `.venv-pose/models/`. Run `.venv-pose/bin/python tools/video_to_choreo.py VIDEO NAME [--start S] [--end S] [--once]`.
  The delete button in the viewer moves a saved choreo to `fsm/.deleted/` (not in git); `fsm/backups/` keeps
  earlier versions of the OrangeJustice and scuba choreos.
- `prompts/` — Plan/Implement/Audit/Test agent instructions, reused across sessions.
- `agent-notes/` — handoff notes (`PLAN.md`, `HANDOVER.md`).
- `spec/` — project spec summary, protocol reference, submission packaging notes.

## Workflow

Plan → Implement → Audit → Test, one phase per session, each reading `prompts/<PHASE>.md` and
`agent-notes/`. Current status and remaining steps live in `agent-notes/PLAN.md`.

Not yet built: the `make demo` nodes (`joint_state_publisher`, `finite_state_machine`).
