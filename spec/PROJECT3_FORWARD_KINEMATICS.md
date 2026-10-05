# Project 3 — Forward Kinematics

Source: https://autorob.org/projects/project3/ (page text extracted in a browser, not a model summary).
The page links two further specs, `spec/FK_API.md` and `spec/ROBOT_DESCRIPTION.md`. They are not in this
repo yet. The page says that if the handout and those specs disagree, the specs win.

## Architecture

- `param_server`: `/param_server/set_param`, `/param_server/get_param`. Holds `robot_description`.
- `robot_state_publisher`: reads `robot_description` through `get_param` (polling is fine), validates it,
  reports status, subscribes `/joint_states`, publishes `/tf`.
- `robot_world_state_publisher`: subscribes `/tf` and `/global_pose`, publishes `/xform_world`.
- Recommended, not graded, `make demo` only: `joint_state_publisher`, `finite_state_machine`.
- Transport: rosbridge TCP/JSON on `127.0.0.1:9095`. Reuse the Project 1 gateway and middleware.
- Every hop carries lines of at least 1 MiB. A URDF travels as one JSON string on one line.

## Makefile and runtime

- `make build`: offline, noninteractive, within 180 s.
- `make run`: gateway, `param_server`, `robot_state_publisher`, `robot_world_state_publisher`. No files,
  env vars, or arguments. Starts with no robot loaded. Ready within 10 s, meaning a robot set through
  `param_server` reaches `/tf` and `/xform_world`.
- `make run` must not start `joint_state_publisher`, `finite_state_machine`, or anything publishing
  `/joint_states`, `/joint_trajectory`, or `/global_pose`, or setting `robot_description`.
- `make demo`: `make run` plus the two recommended nodes. Not graded.
- `make clean`.

## Parameter server

Every response: `{"values": {...}, "result": <bool>, "status": <string>}`.

- `set_param` request `{"name": "<name>", "value": <any JSON>}` → `{"version": <n>}`. Per-name versions
  start at 0; each set adds 1. Missing or non-string name → `result: false`, non-empty status. The value is
  never interpreted.
- `get_param` request `{"name": "<name>"}` → `{"value": <stored>, "version": <n>}`. Unset name →
  `result: false`, `{"value": null, "version": 0}`.

## Robot description status (corrected)

robot_state_publisher reports its status by SETTING the parameter `/robot_state_publisher/description_status`,
not by publishing a topic. Clients read it with `get_param`:

    {"version": 7, "accepted": true, "error": "", "loaded_version": 7, "root_link": "base_link"}

- After a `set_param` of `robot_description` returns version V, report a status for some version >= V
  within 2 s.
- `error` is `""` when accepted, otherwise a non-empty reason.
- `loaded_version` and `root_link` describe the robot now in effect. After a rejection they still describe
  the previous robot (0 and `""` if none).
- Write the status only after an accepted robot is fully in effect, so every later `/tf` reflects it.
- Validate the whole description before changing the current robot. A rejected description leaves the
  previous robot, its joint state, and `/tf` unchanged.

## `/joint_states` (subscribe)

`sensor_msgs/JointState`: `header`, `name`, `position`, `velocity`, `effort`.

- Each message replaces the joint state. A movable joint the message does not name goes to 0.
- Names that are not movable joints (unknown, or fixed) are ignored. The rest of the message applies.
- `velocity` and `effort` may be absent or empty. Ignore them.
- The latest `header.stamp` is copied verbatim to every `/tf` entry. Before any message it is `{"sec": 0, "nanosec": 0}`.
- No clamping: use positions exactly as given.
- Promptly: show up on `/tf` within a few seconds. The same message may arrive more than once.

## `/tf` (publish)

`{"transforms": [TransformStamped, ...]}`, each entry:

    {"header": {"stamp": {...}, "frame_id": "<parent>"}, "child_frame_id": "<child>",
     "transform": {"translation": {"x","y","z"}, "rotation": {"x","y","z","w"}}}

- While a robot is loaded: at least 5 Hz, whether or not anything changed. Not published with no robot loaded.
- Exactly one entry per joint, including fixed joints. Every entry carries the same stamp.
- A late subscriber gets a complete current message within one period.

## `/xform_world` (publish) and `/global_pose` (subscribe)

`/xform_world`: `{"transforms": [MatrixTransform, ...]}`, each entry:

    {"header": {"stamp": {...}, "frame_id": "global_frame"}, "child_frame_id": "<link>",
     "matrix": [m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33]}

- `matrix` is the 4x4 pose `T_global_root · T_root_link`, column-major (`matrix[4*c + r] = M[r][c]`), so
  translation is `matrix[12..14]`.
- Compute from the most recent `/tf` only. Find the root (a parent that is no entry's child). Do not merge older
  edges.
- One entry per link, including the root, stamped with the `/tf` stamp.
- At least 5 Hz once a `/tf` has been received. Serve late subscribers within one period.
- `global_frame` is z-up. Do not add a rendering correction.

`/global_pose`: `geometry_msgs/Pose`, the root link's pose in `global_frame`. Identity until the first message.

## Robot description (URDF)

From the page; full rules in `spec/ROBOT_DESCRIPTION.md`.

- Flat, already xacro-expanded URDF. Never expand xacro.
- Only direct children of `<robot>` count. Nested `<link>`/`<joint>` (e.g. in `<gazebo>`, `<transmission>`)
  are ignored.
- Read: joint type, `<parent>`, `<child>`, `<origin>`, `<axis>`, `<limit>`, and each link's first `<visual>`.
  Everything else (collision, inertial, materials, gazebo, transmission, mimic, safety_controller, unknown
  attributes) is ignored and must never cause rejection.
- Defaults: missing `<origin>`, `xyz`, or `rpy` → zeros. Missing `<axis>` → `1 0 0`. `name` on `<robot>` optional.
- Joint types: `revolute`, `continuous`, `prismatic`, `fixed`. Anything else (including `floating`, `planar`)
  is rejected.
- Revolute and prismatic joints must have a `<limit>`. The limit is informational: FK never clamps to it.
- A link's first `<visual>`, if present, must hold a valid box, cylinder, sphere, or mesh.
- `xyz`, `rpy`, and axis `xyz` hold exactly three decimal numbers, any XML whitespace, any ordinary spelling
  (`1`, `-0.5`, `.25`, `3.`, `1.5e-1`). Wrong count or a non-number is rejected.
- Real-world XML must load: XML declaration, comments (even with markup inside), `xmlns:xacro` on `<robot>`,
  single or double quotes, entity and character references, CDATA, self-closing tags, CRLF line endings,
  joints before their links. Names are compared after entity decoding.
- Validity: the numbered list in `spec/ROBOT_DESCRIPTION.md` is the authority. A description that passes every
  check must be accepted. Duplicate names, cycles, and zero-length axes on movable joints are unspecified:
  do anything except crash or hang.

## Kinematics

- Unit quaternions for rotation. Denavit-Hartenberg parameters are not allowed.
- `rpy`: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, fixed axes.
- `T_joint(q) = T_origin · T_motion(q)`, `T_origin = Trans(xyz) · Rot(rpy)` in the parent frame.
- `T_motion`: revolute and continuous rotate by `q` about the normalized axis. Prismatic translates by
  `q · axis` with the axis as written (not normalized). Fixed is identity.
- Root: the unique link that is no joint's child. Not necessarily the first `<link>`.
- `T_root_child = T_root_parent · T_joint`.
- Mimic joints are driven by their own names, like any other joint.
- Quaternion `(x, y, z, w)`, `w` scalar. Sign does not matter.

## Accuracy

- Translations within 1e-4 m. Rotations within 1e-4 rad (angle between matrices).
- Quaternion norm within 1e-4 of 1.
- `/xform_world` rotation blocks orthonormal within 1e-4, determinant +1.
- Every numeric field is a JSON number. Never emit NaN or Infinity.

## Checkpoint (due Mon 2026-10-12)

Graded as a separate hosted project. Covers: parameter server; description handling and status; zero-configuration
FK on `/tf` and `/xform_world` (every joint at 0), including real robot URDFs (PR2 is well over 64 KiB).
Not graded at checkpoint: `/joint_states` motion, `/global_pose`, rate and late-subscriber timing, near-1 MiB descriptions.
A malformed axis, or a revolute or prismatic joint with no `<limit>`, is rejected even at checkpoint.

## Grading

| Category | Weight | Graded by |
|---|---|---|
| A. Parameter server and description handling | 15% | Autograder.io |
| B. Zero-configuration FK | 25% | Autograder.io |
| C. Joint-state FK | 25% | Autograder.io |
| D. Topic behavior and global pose | 10% | Autograder.io |
| E. FSM choreography video | 15% | Course staff |
| F. Portfolio page | 10% | Course staff |

## Starter and kit

Starters for Python, C, C++, and Rust, each with a supplied XML parser (Python: `xml.etree.ElementTree`).
Student kit: includes `robots/fetch/fetch.urdf` (CC BY-NC-SA 4.0, Fetch Robotics; keep the license with it).
Submission: one language, project root with top-level `Makefile`, `submission.tar.gz` extracts to that root.
See `spec/submission_layout.md`.

## Portfolio

Video of an FSM choreography from `make demo`, with a self-built viewer that draws each link from `/xform_world`.
A portfolio page describes the robot, the choreography, and the viewer. Due date TBD on the page.
