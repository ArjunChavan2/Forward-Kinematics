"""Demo: parse a URDF and print the resulting RobotModel.

Not part of the graded submission. Shows parse_urdf working end to end: direct-child iteration,
link/joint/visual parsing with URDF defaults, root-finding, and validation.

Usage:
    python3 tools/urdf_demo.py              # uses the sample robot below
    python3 tools/urdf_demo.py path/to.urdf # parses a real URDF file
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from urdf import parse_urdf  # noqa: E402

SAMPLE_ROBOT = """
<?xml version="1.0"?>
<robot name="two_link_arm">
  <!-- links may come before or after the joints that use them -->
  <link name="base_link"/>
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="upper_arm">
    <visual><geometry><cylinder radius="0.05" length="0.5"/></geometry></visual>
  </link>
  <joint name="elbow_mount" type="fixed">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0.5 0 0" rpy="0 1.5707963267948966 0"/>
  </joint>
  <link name="forearm"/>
  <gazebo reference="forearm"><material>Gazebo/Grey</material></gazebo>
</robot>
"""


def describe(model) -> None:
    print(f"root link: {model.root_link}")
    print(f"links ({len(model.links)}): {[link.name for link in model.links]}")
    print(f"joints ({len(model.joints)}):")
    for j in model.joints:
        print(f"  {j.name} ({j.joint_type}): {j.parent} -> {j.child}")
        print(f"    origin_xyz={j.origin_xyz} origin_rpy={j.origin_rpy} axis={j.axis}")
        print(f"    limit={j.limit}")


def main() -> None:
    if len(sys.argv) > 1:
        with open(sys.argv[1], encoding="utf-8") as f:
            text = f.read()
    else:
        print("No file given; using the built-in sample robot.\n")
        text = SAMPLE_ROBOT

    try:
        model = parse_urdf(text)
    except ValueError as exc:
        print(f"rejected: {exc}")
        sys.exit(1)

    describe(model)


if __name__ == "__main__":
    main()
