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

## Autograder test rules (2026-10-27)

- The grader runs our tests while its own `make run` holds 9095. `tests/test_integration.py` now
  starts `src/main.py` on a free port (`client_helper.PORT` passed as `GATEWAY_PORT`; gateway
  default stays 9095). `make test` runs `tests/run_tests.py`, documented in `tests/README.md`
  (the grader requires it). Same fix as Projects 1 and 2; see `spec/submission_layout.md`,
  "The grader runs your tests while `make run` is up". Keep both rules for any new test.

## Autograder fixes, round 2 (2026-10-27): timeout, memory test, 5 Hz rate

What failed on the grader, in order, and the fix that stuck:

1. `test_integration.py` timed out (30 s per-file limit; it took ~27 s). `_big_urdf()` re-serialized
   the whole growing string every loop (2.8 s per call); now incremental + cached. Rate/memory
   windows shortened.
2. My shortened memory test then failed (21 MiB vs 20): server RSS ramps ~25 -> ~74 MiB for ~1.5 s
   while the 1 MiB robot loads, and a 0.5 s settle put the baseline on that ramp. Now settle 2 s,
   measure 2 s. Lesson: sample the curve before choosing a settle time.
3. The 5 Hz rate test failed (6 /tf in 1.5 s). Real server bug, not the window: each 0.1 s tick redid
   FK for all ~4,700 joints (~0.15 s on an M3), so the rate depended on CPU speed: ~6.5 Hz on the
   M3, ~4 Hz on the grader, ~2 Hz on efficiency cores. Fix: `robot_state_publisher` and
   `robot_world_state_publisher` keep their last message plus the input objects it came from
   (robot, positions, stamp / /tf msg, global_pose). Those are replaced, never mutated, on change,
   so an unchanged tick republishes the cached message. Now ~10 Hz on efficiency cores.
   Not covered: a huge robot *moving* every tick still recomputes FK each tick (fk.py, hand-written,
   left alone).

How to avoid it next time:
- Reproduce a grader-only failure locally before fixing it:
  `taskpolicy -c background python3 tests/test_x.py` (efficiency cores, slower than the grader).
  The pre-fix code failed there with the grader's exact assertion; the fix passes there.
- Periodic publishers must not redo expensive work for unchanged inputs; a rate test passing on a
  fast laptop proves nothing about the grader.
- Don't loosen timing tests blind: every shortened window or settle time must pass on efficiency
  cores first.

## Next step

Implement `src/fk.py` per its existing docstrings and PLAN.md's math section (`T_joint = T_origin · T_motion(q)`,
parent-first composition). Then run `tests/` and check `/tf` cadence/content against PLAN.md's topic
behavior requirements.
