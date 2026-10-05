# Plan — Project 3 (Forward Kinematics)

Status: provisional. Based on `spec/PROJECT3_FORWARD_KINEMATICS.md`, which is a summary from
autorob.org, not the verbatim spec. Resolve the open questions below before implementation.

## Goal

Provide a multi-node system on the Project 1 rosbridge gateway that loads a URDF robot description,
computes link poses from joint states by forward kinematics, and publishes them as `/tf` and
`/xform_world`. The graded core is three nodes: `param_server`, `robot_state_publisher`, and
`robot_world_state_publisher`. The ungraded demo adds `joint_state_publisher` and
`finite_state_machine` for choreography.

## Relevant requirements

Protocol and runtime:
- Reuse the Project 1 gateway and middleware on `127.0.0.1:9095`, newline-delimited JSON.
- Every hop carries lines of at least 1 MiB. The gateway accepts up to 4 MiB (done).
- `make run` starts gateway, `param_server`, `robot_state_publisher`, `robot_world_state_publisher`
  in the foreground. Ready within 10 s, no robot loaded.
- `make run` must not start anything publishing `/joint_states`, `/joint_trajectory`, or `/global_pose`.
- `make build` compiles offline within 180 s. `make clean` removes artifacts. `make demo` is ungraded.

Topics and services:
- `/param_server/set_param`, `/param_server/get_param`; `robot_description` stored as a parameter.
- `robot_state_publisher` subscribes `/joint_states`, publishes `/tf`.
- `/tf`: at least 5 messages/s even when nothing changed; exactly one entry per joint, including
  `fixed`; every entry's stamp is the latest `/joint_states` `header.stamp`, verbatim.
- `robot_world_state_publisher` subscribes `/tf` and `/global_pose`, publishes `/xform_world`.

URDF:
- Only direct children of `<robot>` count. Joint types: `revolute`, `continuous`, `prismatic`, `fixed`.
- Handle comments, entity references, CRLF line endings.
- Revolute/continuous axes normalized; prismatic axes as written. `<limit>` is informational, never clamps.
- Invalid descriptions are reported with a status (exact behavior unspecified in the summary).

Math:
- `R = Rz(yaw) · Ry(pitch) · Rx(roll)`; `T_joint(q) = T_origin · T_motion(q)`;
  `T_root_child = T_root_parent · T_joint`.
- Accuracy: translation within 1e-4 m, rotation within 1e-4 rad, quaternion norm within 1e-4 of 1.
- No kinematics, transform, or URDF-model library.

Grading weights: parameter server and description handling 15%, zero-configuration FK 25%,
joint-state FK 25%, topic behavior and global pose 10%, FSM video 15% (staff), portfolio 10% (staff).
Checkpoint due 2026-10-12: categories A–B, excluding large descriptions.

## Repository observations

- Transport is in place: `src/registry.py`, `src/gateway.py` (Project 1 gateway with the Project 2
  fixes, pushed to Project 1 as `51d7dd8`), `src/main.py` (registry + gateway only).
- `src/gateway.py` already has in-process service handlers (`registry.call_service_sync`,
  `has_in_process_handler`) and TCP provider routing with a 5 s timeout. Nodes can be in-process
  or separate TCP clients; the summary does not require either.
- `Makefile` has `build`, `run`, `test`, `clean`. No `demo` yet.
- `tests/client_helper.py` is the raw-socket client.
- No URDF, FK, or node code exists.

## Proposed architecture

Single asyncio process, same as Project 1 and 2: `main.py` wires `Registry`, `Gateway`, and the
required node registrations, then runs. Each node is a module that registers its services and
subscriptions on the registry:

- `src/param_server.py` — in-memory dict; `set_param`/`get_param` services.
- `src/urdf.py` — XML parsing into a model of links and joints (direct `<robot>` children only).
  Parser choice is open (see questions). Pure data, no ROS types.
- `src/transform.py` — hand-written 3x3/4x4 matrix or quaternion helpers, RPY, axis-angle, normalize.
- `src/fk.py` — kinematic tree build (root link, parent/child map), traversal, `T_root_child`
  composition from `q`, quaternion output.
- `src/robot_state_publisher.py` — on `robot_description` set, parse and validate, store model;
  subscribe `/joint_states`, compute, publish `/tf` at least 5 Hz via a timer task.
- `src/robot_world_state_publisher.py` — subscribe `/tf` and `/global_pose`, compose, publish `/xform_world`.
- `src/main.py` — wiring, starts the required three nodes plus gateway.

The `make demo` nodes (`joint_state_publisher`, `finite_state_machine`) come after the graded core.

## Interfaces and data flow

- `param_server`: `set_param{name, value}` → `{result, status}`; `get_param{name}` → `{result, values:{value}}`.
  (Exact arg/response shapes are not in the summary — confirm against the verbatim spec.)
- `/joint_states` (subscribed by robot_state_publisher) → `fk.compute(model, q)` → `/tf`.
- `/tf` (subscribed by robot_world_state_publisher) plus `/global_pose` → `/xform_world`.
- Parsed model stored in `robot_state_publisher` state; `robot_description` is read from `param_server`.

## Implementation steps

1. Get the verbatim spec and fill the open questions below.
2. `param_server.py` with `set_param`/`get_param`, plus a unit test.
3. `transform.py` with unit tests against hand-computed values.
4. `urdf.py` parser with tests for direct-child rule, CRLF, comments, entity refs, joint types.
5. `fk.py` tree build and composition with tests for fixed joints and a 2-link chain.
6. `robot_state_publisher.py`: description validation, `/joint_states` subscription, `/tf` timer.
7. `robot_world_state_publisher.py`: `/tf` and `/global_pose` → `/xform_world`.
8. `main.py` wiring; `make run` readiness check within 10 s.
9. Integration tests over the wire using `client_helper.py`.
10. `make demo` (ungraded), then the FSM and portfolio.

## Edge cases and failure modes

- URDF with nested `<joint>`/`<link>` elements outside `<robot>` must be ignored.
- Missing or invalid `robot_description`: nodes stay up and report an error status.
- `/joint_states` with missing joints or wrong stamp type: no crash; behavior to be specified.
- Joint with missing `<limit>` for revolute/prismatic: informational only, FK still computed.
- Zero-length axis or unnormalizable axis: undefined in summary; needs a rule.
- `/tf` must include `fixed` joints; a node stamping with its own clock violates the spec.
- Gateway with a client that never advertises: publish behavior follows Project 1 (advertise-gated).
- Shutdown with connected clients: gateway already closes them (Project 2 fix).
- Large URDF (at least 1 MiB lines): handled by the 4 MiB limit; "large descriptions" are excluded
  from the checkpoint.

## Open questions and assumptions

Project 2 precedents are noted where they apply. Each is an assumption until the verbatim spec confirms it.

1. Verbatim spec (blocking for steps 2 and 7): the summary may differ on exact argument and response
   field names, error statuses, and the `/global_pose` format. Decided: service responses use the
   Project 1 envelope `{"values", "result", "status"}` (`service_response` on the wire, `id` echoed).
2. Who writes the FK math (decided): the user (project owner) hand-writes `urdf.py`, `transform.py`, and
   `fk.py`. Agents build the transport, node wiring, `param_server`, and tests around them, following
   Project 2's split. Agents must not write the URDF parsing or FK math.
3. Advertise rule (decided): Project 2 and Project 3 rely on Project 1's advertisement rules. The gateway
   gates external wire `publish` on an active advertisement, and an unadvertised publish is silently
   dropped. Project 2's current gateway relaxes this, and its spec says so, so Project 2 needs to change
   to match. Internal nodes publish in-process via `registry.publish`, which is not gated (Project 2 precedent).
4. Node topology: assumed in-process on one asyncio loop, as in Projects 1 and 2 (Project 2 precedent).
5. Quaternion convention (`w,x,y,z` vs `x,y,z,w`) in `/tf` and `/xform_world`: no precedent. Open.
6. Message shapes: assumed ROS-style, matching Project 2: `header: {stamp: {sec, nanosec}, frame_id}`
   with parallel arrays for `/joint_states`. `/tf`, `/xform_world`, and `/global_pose` shapes: no
   precedent. Open.
7. Stdlib only, no numpy (Project 2 precedent).
8. Startup probe risk: Project 2's grader probed a parameter service at startup, and a missing provider
   failed the whole run. Build `param_server` first (step 2) and check whether the probe calls
   `/param_server/get_param` or `set_param` with `{}`.

## Verification strategy

- Unit tests for transforms, URDF parsing, and FK against hand-computed values (1e-4 tolerances).
- Integration tests over the real gateway: `make run` readiness within 10 s; `/tf` cadence and
  per-joint coverage; stamp passthrough; `/xform_world` composition with `/global_pose`.
- Probe as an autograder would: non-default URDFs, CRLF input, connections left open at shutdown.
- Checkpoint check: categories A–B, excluding large descriptions, before 2026-10-12.
