# Project 3 — Forward Kinematics

Python implementation of URDF-driven forward kinematics and a multi-node pub/sub system, running on
the Project 1 rosbridge-style TCP/JSON gateway (`127.0.0.1:9095`). Spec:
`spec/PROJECT3_FORWARD_KINEMATICS.md` (summary; verbatim spec pending), `spec/ROSBRIDGE_PROTOCOL.md`.

## Running

```
make build   # offline, noninteractive, compiles all of src/
make run     # foreground, listens on 127.0.0.1:9095, Ctrl-C / SIGTERM to stop cleanly
make test    # unit + integration tests
make clean   # removes __pycache__
```

## Layout

- `src/registry.py`, `src/gateway.py` — generic topic/service registry and asyncio TCP/JSON gateway, from Project 1.
- `src/main.py` — wires `Registry` + `Gateway`; `make run`'s entry point. No Project 3 nodes yet.
- `tests/client_helper.py` — raw-socket TCP/JSON test client.
- `prompts/` — Plan/Implement/Audit/Test agent instructions, reused across sessions.
- `agent-notes/` — handoff artifacts (`PLAN.md`, `IMPLEMENTATION.md`, `AUDIT.md`, `TEST_RESULTS.md`) written by each phase.
- `spec/` — project spec summary, protocol reference, submission packaging notes.

## Workflow

Plan → Implement → Audit → Test, one phase per session, each reading `prompts/<PHASE>.md` and the
prior phase's `agent-notes/` file. See each prompt's "Project context" section for current status.

Not yet built: URDF parsing, FK math, `param_server`, `robot_state_publisher`,
`robot_world_state_publisher`, and `make demo`.
