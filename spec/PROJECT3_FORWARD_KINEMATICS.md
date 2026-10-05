# Project 3 — Forward Kinematics (summary)

Source: https://autorob.org/projects/project3/#starter-projects. This is a summary, not the verbatim
spec. Replace it with the official text before relying on exact wording or numbers.

## Goals

- Parse URDF robot descriptions and build kinematic trees.
- Implement 3D rigid-body transforms using unit quaternions.
- Compute FK by tree traversal and transform composition.
- Build a multi-node ROS-like pub/sub system.
- Demonstrate robot motion through choreography.

## Required nodes (graded)

- `param_server`: named parameters, including `robot_description` via `/param_server/set_param` and
  `/param_server/get_param`.
- `robot_state_publisher`: reads the URDF, validates it, reports status, subscribes to
  `/joint_states`, publishes `/tf` (joint transforms).
- `robot_world_state_publisher`: composes `/tf` and `/global_pose` into `/xform_world` (link poses in
  the global frame).

## Recommended nodes (ungraded)

- `joint_state_publisher`: servos joints toward `/joint_trajectory` setpoints.
- `finite_state_machine`: runs choreographed joint sequences.

## Protocol

- Reuse the Project 1 rosbridge gateway and middleware.
- Transport: newline-delimited JSON over plain TCP on `127.0.0.1:9095`.
- Every hop must carry lines of at least 1 MiB.
- Publish `/tf` at least 5 times per second, even when nothing changed.
- Each `/tf` message has exactly one entry per joint, including `fixed` joints.
- The latest message's `header.stamp` is copied verbatim to every `/tf` entry.

## URDF rules

- Only direct children of `<robot>` count. Nested elements elsewhere are ignored.
- Supported joint types: `revolute`, `continuous`, `prismatic`, `fixed`.
- Revolute and continuous axes are normalized. Prismatic axes are used as written.
- Revolute and prismatic joints must have a `<limit>`. Limits are informational and never clamp FK.
- Real-world XML must parse: comments, entity references, CRLF line endings.
- No kinematics, transform, or URDF-model library. Implement the math and traversal yourself.

## Math conventions

- `R = Rz(yaw) · Ry(pitch) · Rx(roll)`
- `T_joint(q) = T_origin · T_motion(q)`
- `T_root_child = T_root_parent · T_joint`
- Accuracy: translations within 1e-4 m, rotations within 1e-4 rad, quaternion norm within 1e-4 of 1.

## Makefile targets

- `make build`: compiles within 180 s offline, using only the course toolchain.
- `make run`: starts the gateway, `param_server`, `robot_state_publisher`, and
  `robot_world_state_publisher` in the foreground. Must be ready within 10 s, with no robot loaded.
  Must not start `joint_state_publisher`, `finite_state_machine`, or anything publishing
  `/joint_states`, `/joint_trajectory`, or `/global_pose`.
- `make demo`: adds `joint_state_publisher` and `finite_state_machine` for choreography (ungraded).
- `make clean`: removes generated artifacts.

## Grading

| Category | Weight | Method |
|---|---|---|
| Parameter server and description handling | 15% | Autograder.io |
| Zero-configuration FK | 25% | Autograder.io |
| Joint-state FK | 25% | Autograder.io |
| Topic behavior and global pose | 10% | Autograder.io |
| FSM choreography video | 15% | Staff |
| Portfolio page | 10% | Staff |

Checkpoint due 2026-10-12: categories A–B, excluding large descriptions. All three required nodes
must work for runtime readiness.

Submission: one language (Python here), a single project root with a top-level `Makefile`, packaged
as `submission.tar.gz` with no subdirectories. See `spec/submission_layout.md`.
