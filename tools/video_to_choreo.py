"""Turn a dance video into a choreography: fsm/<name>.json for the dual-arm robot.

    .venv-pose/bin/python tools/video_to_choreo.py VIDEO NAME [--start S] [--end S] [--once] [--force]

MediaPipe finds the dancer's 3D body points in each frame (ffmpeg decodes the video). The arms are
solved exactly from the robot's arm geometry, the torso from the shoulder line against the hips, and
thigh bend becomes torso_lift_z. Keyframes are where the movement pauses. Needs .venv-pose (Python 3.12
with mediapipe) and .venv-pose/models/pose_landmarker_heavy.task; the README says how to set them up.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from finite_state_machine import FSM_DIR, IDENTIFIER_RE, validate  # noqa: E402

MODEL = os.path.join(HERE, "..", ".venv-pose", "models", "pose_landmarker_heavy.task")
SAMPLE_FPS = 15
JOINTS = ["drive_x", "drive_y", "drive_yaw", "ankle_roll", "torso_lift_z", "waist_pitch", "waist_roll", "waist_yaw"] + [
    f"{side}_{j}" for side in ("left", "right") for j in
    ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll", "elbow_yaw", "wrist_roll",
     "wrist_pitch", "wrist_yaw")]
# MediaPipe pose landmark indices ("left" is the dancer's left)
L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST, L_HIP, R_HIP, L_KNEE, R_KNEE = 11, 12, 13, 14, 15, 16, 23, 24, 25, 26


def sub(a, b): return [a[i] - b[i] for i in range(3)]
def add(a, b): return [a[i] + b[i] for i in range(3)]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def unit(a):
    n = math.sqrt(dot(a, a)) or 1.0
    return [x / n for x in a]


def frame(side_axis, up_hint):
    """Rows x (forward), y (left), z (up) of a body frame whose y follows side_axis."""
    y = unit(side_axis)
    x = unit(cross(y, up_hint))
    return [x, y, cross(x, y)]


def local(F, v): return [dot(F[0], v), dot(F[1], v), dot(F[2], v)]


def arm_angles(upper, fore, prev_yaw=0.0):
    """(shoulder pitch, roll, yaw, elbow pitch) pointing the robot's upper arm along `upper` and forearm along
    `fore` (unit vectors in the torso frame). Chain: pitch(y) -> roll(x) -> yaw(z) -> elbow pitch(y), arm along -z;
    elbow roll stays 0. A straight arm leaves the yaw free, so it keeps prev_yaw."""
    roll = math.asin(max(-1.0, min(1.0, upper[1])))
    pitch = math.atan2(-upper[0], -upper[2])
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    # forearm in the frame after pitch and roll: transpose of Ry(pitch) Rx(roll) applied to fore
    x = cp * fore[0] - sp * fore[2]
    y = sr * sp * fore[0] + cr * fore[1] + sr * cp * fore[2]
    z = cr * sp * fore[0] - sr * fore[1] + cr * cp * fore[2]
    elbow = -math.acos(max(-1.0, min(1.0, -z)))  # bends forward, like the hand-made choreos
    if math.hypot(x, y) < 0.15:  # nearly straight: yaw barely matters
        return pitch, roll, prev_yaw, elbow
    yaw = math.atan2(y, x)
    return pitch, roll, yaw, elbow


def robot_axes(p):
    """MediaPipe world point (x right in the image, y down, z away from the camera) -> robot axes (x forward
    toward the camera, y the dancer's left, z up)."""
    return [-p[2], p[0], -p[1]]


def rot(axis, a):
    """Rows of the rotation by `a` about axis "x", "y" or "z"."""
    c, s = math.cos(a), math.sin(a)
    return {"x": [[1, 0, 0], [0, c, -s], [0, s, c]], "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]


def matmul(A, B): return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def apply(A, v): return [dot(row, v) for row in A]
def apply_t(A, v): return [sum(A[k][i] * v[k] for k in range(3)) for i in range(3)]


def body_angles(world):
    """Raw base and waist angles plus thigh bend for one frame (world: 33 robot-axis points)."""
    hip_mid = [(world[L_HIP][i] + world[R_HIP][i]) / 2 for i in range(3)]
    sh_mid = [(world[L_SHOULDER][i] + world[R_SHOULDER][i]) / 2 for i in range(3)]
    pelvis = frame(sub(world[L_HIP], world[R_HIP]), [0, 0, 1])
    torso = frame(sub(world[L_SHOULDER], world[R_SHOULDER]), sub(sh_mid, hip_mid))
    up = local(pelvis, unit(sub(sh_mid, hip_mid)))
    tx = local(pelvis, torso[0])
    thighs = [unit(sub(world[k], world[h])) for h, k in ((L_HIP, L_KNEE), (R_HIP, R_KNEE))]
    return {"drive_yaw": math.atan2(pelvis[0][1], pelvis[0][0]),
            "ankle_roll": math.atan2(-pelvis[1][2], math.hypot(pelvis[1][0], pelvis[1][1])) * 0.5,  # hip tilt, toned down
            "waist_pitch": math.atan2(up[0], up[2]), "waist_roll": math.atan2(-up[1], up[2]),
            "waist_yaw": math.atan2(tx[1], tx[0]),
            "bend": sum(math.acos(max(-1.0, min(1.0, -t[2]))) for t in thighs) / 2}  # 0: thighs straight down


def pose_angles(world, raw, facing, waist_center, prev):
    """Joint positions for one frame. facing: the median shoulder heading, taken as "toward the camera";
    waist_center: MediaPipe's steady hip-vs-shoulder twist, which is tracking bias rather than dance."""
    q = dict.fromkeys(JOINTS, 0.0)
    q["waist_yaw"] = raw["waist_yaw"] - waist_center
    q["drive_yaw"] = raw["drive_yaw"] + raw["waist_yaw"] - facing - q["waist_yaw"]
    q["ankle_roll"], q["waist_roll"] = raw["ankle_roll"], raw["waist_roll"]
    q["waist_pitch"] = raw["waist_pitch"] * 0.5  # a front camera judges lean poorly; half keeps it subtle
    q["torso_lift_z"] = 0.3 * max(0.0, min(1.0, 1 - (raw["bend"] - 0.15) / 0.7))  # straight legs: full height
    # the robot's torso orientation from those angles (chain: drive_yaw, ankle_roll, waist pitch, roll, yaw), in a
    # world turned so `facing` is forward; the arms are solved against it so the hands land where the video has them
    R = rot("z", q["drive_yaw"])
    for axis, j in (("x", "ankle_roll"), ("y", "waist_pitch"), ("x", "waist_roll"), ("z", "waist_yaw")):
        R = matmul(R, rot(axis, q[j]))
    unturn = rot("z", -facing)
    for side, s, e, w in (("left", L_SHOULDER, L_ELBOW, L_WRIST), ("right", R_SHOULDER, R_ELBOW, R_WRIST)):
        upper = apply_t(R, apply(unturn, unit(sub(world[e], world[s]))))
        fore = apply_t(R, apply(unturn, unit(sub(world[w], world[e]))))
        p, r, y, el = arm_angles(upper, fore, prev.get(f"{side}_shoulder_yaw", 0.0))
        q.update({f"{side}_shoulder_pitch": p, f"{side}_shoulder_roll": r, f"{side}_shoulder_yaw": y,
                  f"{side}_elbow_pitch": el})
    return q


def track(video, start, end):
    """[33 world points (robot axes) or None] per sampled frame, via ffmpeg + MediaPipe."""
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision

    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                            "-of", "csv=p=0", video], capture_output=True, text=True, check=True)
    w, h = (int(v) for v in probe.stdout.strip().split(",")[:2])
    w, h = 480, round(480 * h / w / 2) * 2  # smaller frames track just as well and decode faster
    cmd = ["ffmpeg", "-v", "error", "-ss", str(start)] + (["-to", str(end)] if end else []) + [
        "-i", video, "-vf", f"fps={SAMPLE_FPS},scale={w}:{h}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    options = vision.PoseLandmarkerOptions(base_options=BaseOptions(model_asset_path=MODEL),
                                           running_mode=vision.RunningMode.VIDEO)
    frames = []
    with vision.PoseLandmarker.create_from_options(options) as landmarker, \
            subprocess.Popen(cmd, stdout=subprocess.PIPE) as ff:
        import numpy as np
        size = w * h * 3
        while (buf := ff.stdout.read(size)) and len(buf) == size:
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.frombuffer(buf, np.uint8).reshape(h, w, 3))
            result = landmarker.detect_for_video(image, round(len(frames) * 1000 / SAMPLE_FPS))
            if result.pose_world_landmarks:
                frames.append([robot_axes([p.x, p.y, p.z]) for p in result.pose_world_landmarks[0]])
            else:
                frames.append(None)
    return frames


def smooth(rows, k=2):
    """Moving average over +-k samples of each joint."""
    return [{j: sum(r[j] for r in rows[max(0, i - k):i + k + 1]) / len(rows[max(0, i - k):i + k + 1]) for j in JOINTS}
            for i in range(len(rows))]


def keyframes(rows, min_gap=4):
    """Indices where joint-space speed has a local minimum (the dancer hits a pose), at least min_gap apart."""
    speed = [0.0] + [math.sqrt(sum((rows[i][j] - rows[i - 1][j]) ** 2 for j in JOINTS)) for i in range(1, len(rows))]
    picks = [0]
    for i in range(1, len(rows) - 1):
        if speed[i] <= speed[i - 1] and speed[i] <= speed[i + 1] and i - picks[-1] >= min_gap:
            picks.append(i)
    return picks


def table(poses, loop):
    names = [f"move_{i + 1}" for i in range(len(poses))]
    states = []
    for i, (name, q) in enumerate(zip(names, poses)):
        s = {"name": name, "target": {"joint_names": JOINTS, "positions": [round(q[j], 4) for j in JOINTS]},
             "epsilon": 0.1, "transitions": []}
        nxt = names[i + 1] if i + 1 < len(names) else (names[0] if loop else None)
        if nxt:
            s["transitions"] = [{"name": f"to_{nxt}", "next_state": nxt}]
            s["on_arrival"] = f"to_{nxt}"
        states.append(s)
    return {"initial_state": names[0], "states": states}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("video")
    ap.add_argument("name", help="saved as fsm/<name>.json")
    ap.add_argument("--start", type=float, default=0.0, help="seconds to skip at the start")
    ap.add_argument("--end", type=float, help="stop at this many seconds")
    ap.add_argument("--once", action="store_true", help="play once and hold the last pose (default: loop)")
    ap.add_argument("--force", action="store_true", help="overwrite an existing fsm/<name>.json")
    args = ap.parse_args()
    if not IDENTIFIER_RE.fullmatch(args.name):
        sys.exit("name must match [A-Za-z0-9_-]{1,64}")
    out = os.path.join(FSM_DIR, args.name + ".json")
    if os.path.exists(out) and not args.force:
        sys.exit(f"{out} exists; pass --force to overwrite it")

    frames = track(args.video, args.start, args.end)
    seen = [f for f in frames if f]
    print(f"tracked {len(seen)} of {len(frames)} frames", flush=True)
    if len(seen) < 2:
        sys.exit("no body found in the video")
    raws = [body_angles(f) for f in seen]
    median = lambda xs: sorted(xs)[len(xs) // 2]  # noqa: E731
    facing = median([r["drive_yaw"] + r["waist_yaw"] for r in raws])
    waist_center = median([r["waist_yaw"] for r in raws])
    rows, prev = [], {}
    for f, raw in zip(seen, raws):
        prev = pose_angles(f, raw, facing, waist_center, prev)
        rows.append(prev)
    rows = smooth(rows)
    picks = keyframes(rows)
    doc = table([rows[i] for i in picks], loop=not args.once)
    error = validate(doc)
    if error:
        sys.exit(f"generated table is invalid: {error}")
    tmp = out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
    os.replace(tmp, out)
    print(f"wrote {out}: {len(picks)} states from {len(seen) / SAMPLE_FPS:.1f} s of tracked video")


if __name__ == "__main__":
    main()
