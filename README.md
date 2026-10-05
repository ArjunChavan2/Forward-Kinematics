# Project 3 — Forward Kinematics

Python implementation of forward kinematics for a planar n-link rotational arm, served over the
rosbridge-style TCP/JSON gateway from Project 1 (`127.0.0.1:9095`), with the transport reused from
`~/Pendularm` (`src/registry.py`, `src/gateway.py`). Spec: `spec/ROSBRIDGE_PROTOCOL.md`.
The Project 3 spec (`spec/PROJECT3_FORWARD_KINEMATICS.md`) is not yet in this repo.

## Running

```
make build   # offline, noninteractive, compiles all of src/
make run     # foreground, listens on 127.0.0.1:9095, Ctrl-C / SIGTERM to stop cleanly
make test    # unit + integration tests
make clean   # removes __pycache__
```

## Layout

- `src/registry.py`, `src/gateway.py` — generic topic/service registry and asyncio TCP/JSON gateway, ported from Project 2.
- `src/main.py` — wires `Registry` + `Gateway`; `make run`'s entry point.
- `tests/client_helper.py` — raw-socket TCP/JSON test client.
- `spec/` — protocol reference and submission packaging notes.

Not yet built: the forward-kinematics module and its services.
