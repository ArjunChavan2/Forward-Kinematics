# Project 3 — Forward Kinematics

Source: https://autorob.org/projects/project3/

Fetched through a tool that reproduces the page's text. Check exact field names and numbers against
the official page before relying on them, since the fetch tool may paraphrase.

## Parameter server

### `/param_server/set_param`

Request: `{"name": "<name>", "value": <any JSON>}`

Response: `{"values": {"version": <n>}, "result": <bool>, "status": <string>}`

Each name keeps its own version, starting at 0. The first `set_param` makes it 1. A missing or
non-string name gets `result: false` with a descriptive status.

### `/param_server/get_param`

Request: `{"name": "<name>"}`

Response: `{"values": {"value": <stored value>, "version": <n>}, "result": <bool>, "status": <string>}`

For an unset name: `result: false`, `value: null`, `version: 0`.

## Robot state publisher status

Published on `/robot_state_publisher/description_status`:

```json
{"version": 7, "accepted": true, "error": "", "loaded_version": 7, "root_link": "base_link"}
```

- `version`: the processed `robot_description` version.
- `accepted`: true when the load succeeded.
- `error`: empty if accepted, otherwise a non-empty reason.
- `loaded_version`, `root_link`: describe the active robot.

Write status only after the robot is fully operational on `/tf` and `/xform_world`.

## `/joint_states` (subscribe)

`sensor_msgs/JointState`:

```json
{"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
 "name": [...], "position": [...], "velocity": [...], "effort": [...]}
```

Each message replaces the joint state. Unnamed movable joints default to position 0. Unknown names and
fixed joints are ignored. `header.stamp` is copied verbatim to `/tf` entries.

## `/tf` (publish)

`geometry_msgs/TransformStamped` array:

```json
{"transforms": [
  {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "<parent link>"},
   "child_frame_id": "<child link>",
   "transform": {"translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                 "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}
]}
```

- At least 5 Hz.
- Exactly one entry per joint, including fixed joints.
- All entries share one stamp.
- A late subscriber gets a complete current message within one publishing cycle.

## `/xform_world` (publish)

Array of matrix transforms:

```json
{"transforms": [
  {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "global_frame"},
   "child_frame_id": "<link name>",
   "matrix": [m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33]}
]}
```

- `matrix` is a 4x4 in column-major order. Translation is `matrix[12..14]`.
- Include every link, including the root.
- Compute from the latest `/tf` message alone. Do not merge old edges.
- At least 5 Hz.

## `/global_pose` (subscribe)

`geometry_msgs/Pose`:

```json
{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
```

The root link's pose in `global_frame`. Identity until the first message arrives.

## Math conventions

- RPY: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
- `T_joint(q) = T_origin · T_motion(q)`, with `T_origin = Trans(xyz) · Rot(rpy)` in the parent frame.
- Revolute and continuous: rotate by `q` about the normalized `axis`.
- Prismatic: translate by `q · axis`. The axis is not normalized.
- Fixed: identity.
- Quaternions are `(x, y, z, w)` with `w` scalar.

## Accuracy

- Translations within 1e-4 m.
- Rotations within 1e-4 rad (angle between matrices).
- Quaternion norm within 1e-4 of 1.
- `/xform_world` rotation blocks orthonormal within 1e-4, determinant +1.
- All numeric fields are JSON numbers.

## Errors

Invalid service names or missing required fields get `result: false` and a non-empty `status`.
A rejected robot description leaves the previous robot unchanged, and `/tf` and `/xform_world` are
unaffected.


## Summary (earlier, pre-verbatim)

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
