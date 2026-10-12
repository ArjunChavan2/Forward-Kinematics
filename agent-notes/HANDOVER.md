# Handover — Project 3 (Forward Kinematics)

For a fresh session starting directly in this repo. Full plan/history: `PLAN.md` (this directory).

## P1/P2 context this project depends on

- Transport is Project 1's rosbridge-style gateway: `src/registry.py` (topic/service registry) +
  `src/gateway.py` (asyncio TCP/JSON on `127.0.0.1:9095`, newline-delimited JSON, lines up to 4 MiB).
  Reused as-is, with Project 2's fixes folded in (pushed to Project 1 as `51d7dd8`).
- Advertise rule from Project 1, carried into P2/P3: external wire `publish` is dropped unless the
  connection advertised that topic first. Internal/in-process `registry.publish` is not gated.
- Project 2 fix `a34ffc8`: shutdown closes connected clients. Also already in this gateway.
- None of this needs further work — it's done, tested transport. Only touch `gateway.py`/`registry.py`
  if a P3 integration test turns up a real transport bug.

## P3 state (current)

- **Blocker: `src/fk.py`.** All four functions (`joint_edges`, `link_transforms`, `identity_pose`,
  `root_of_edges`) are `NotImplementedError` stubs with full docstrings already written — this file is
  owner-written (see PLAN.md open question #2: agents must not write the URDF/FK math). Everything else
  (`robot_state_publisher.py`, `robot_world_state_publisher.py`, `main.py`) is wired against it and will
  work once it's filled in.
- Done: `param_server.py`, `transform.py`, `urdf.py`, `robot_state_publisher.py`,
  `robot_world_state_publisher.py`, `main.py` wiring. All per PLAN.md steps 2–4, 6–8.
- Not yet checked: `make run` readiness-within-10s against a real robot (needs `fk.py`), integration
  tests over the wire (step 9), `make demo` nodes (steps 10–12, ungraded).
- Checkpoint was due 2026-10-12 (categories A–B, excluding large descriptions) — check if that's passed
  and what's actually graded before prioritizing further.
- `README.md`'s "Layout"/"Workflow" sections are stale (say "no Project 3 nodes yet") — trust PLAN.md's
  "Repository observations" and commit log over README for current status.

## Next step

Implement `src/fk.py` per its existing docstrings and PLAN.md's math section (`T_joint = T_origin · T_motion(q)`,
parent-first composition). Then run `tests/` and check `/tf` cadence/content against PLAN.md's topic
behavior requirements.
