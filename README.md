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
- `robots/three_link_arm.urdf` — the demo robot. `fsm/` — choreographies (`three_link_wave.json`, `sample_swing.json`).
- `prompts/` — Plan/Implement/Audit/Test agent instructions, reused across sessions.
- `agent-notes/` — handoff notes (`PLAN.md`, `HANDOVER.md`).
- `spec/` — project spec summary, protocol reference, submission packaging notes.

## Workflow

Plan → Implement → Audit → Test, one phase per session, each reading `prompts/<PHASE>.md` and
`agent-notes/`. Current status and remaining steps live in `agent-notes/PLAN.md`.

Not yet built: the `make demo` nodes (`joint_state_publisher`, `finite_state_machine`).
