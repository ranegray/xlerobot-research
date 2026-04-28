"""Command the left arm to a previously captured pose YAML.

Reads ~/.xle/poses/<name>.yaml, builds a single-point JointTrajectory, and
publishes it once to /left_arm_controller/joint_trajectory. The
joint_trajectory_guard validates against URDF limits before the bridge
sees it. Joints in the pose YAML that aren't in the bridge's command set
(left arm 1-6) are silently skipped.

Usage:
    ros2 run xle_hardware goto_pose stow
    ros2 run xle_hardware goto_pose stow --duration 5.0
    ros2 run xle_hardware goto_pose stow --pose-dir ~/.xle/poses
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from xle_hardware.trajectory_guard import JOINT_LIMITS, LEFT_ARM_JOINTS

# Arm trajectories exclude Jaw_L; gripper control gets its own interface.
ALLOWED_JOINTS = LEFT_ARM_JOINTS


SCHEMA = "xle.pose.v0"
DEFAULT_POSE_DIR = Path.home() / ".xle" / "poses"
DEFAULT_TOPIC = "/left_arm_controller/joint_trajectory"
DEFAULT_DURATION_S = 3.0


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Command the left arm to a saved pose.")
    parser.add_argument("name", help="Pose name (file <pose-dir>/<name>.yaml).")
    parser.add_argument(
        "--pose-dir",
        type=Path,
        default=DEFAULT_POSE_DIR,
        help=f"Directory holding pose YAMLs (default: {DEFAULT_POSE_DIR}).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"Time-from-start for the trajectory point in seconds (default {DEFAULT_DURATION_S}).",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="Output trajectory topic.",
    )
    return parser.parse_args(argv)


def load_pose(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(
            f"pose file not found: {path}. capture it first with "
            f"`ros2 run xle_hardware capture_pose {path.stem}`."
        )
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        raise SystemExit(
            f"{path} is not a {SCHEMA} document (got schema={data.get('schema')!r})."
        )
    joints = data.get("joints") or {}
    if not joints:
        raise SystemExit(f"{path} has no joints to command.")
    return data


def _clamp_to_guard(joint_name: str, value: float) -> tuple[float, Optional[str]]:
    """Clamp to the guard's per-joint limits (cal-derived, URDF fallback).

    Handles stale poses after calibration limits change. Usually a no-op.
    """
    limits = JOINT_LIMITS.get(joint_name)
    if limits is None:
        return value, None
    lo, hi = limits
    if value < lo:
        return lo, f"clamped {value:+.4f} -> {lo:+.4f} (guard min)"
    if value > hi:
        return hi, f"clamped {value:+.4f} -> {hi:+.4f} (guard max)"
    return value, None


def build_trajectory(pose: dict, duration_s: float) -> tuple[JointTrajectory, List[str], List[str], List[str]]:
    """Build a single-point trajectory with only commandable joints, clamped to URDF limits.

    Returns (msg, commanded, skipped, clamp_notes).
    """
    pose_joints = pose["joints"]
    commanded: List[str] = []
    skipped: List[str] = []
    positions: List[float] = []
    clamp_notes: List[str] = []
    for joint_name in ALLOWED_JOINTS:
        if joint_name in pose_joints:
            raw = float(pose_joints[joint_name])
            value, note = _clamp_to_guard(joint_name, raw)
            commanded.append(joint_name)
            positions.append(value)
            if note is not None:
                clamp_notes.append(f"{joint_name}: {note}")
    for joint_name in pose_joints:
        if joint_name not in ALLOWED_JOINTS:
            skipped.append(joint_name)

    msg = JointTrajectory()
    msg.joint_names = commanded
    point = JointTrajectoryPoint()
    point.positions = positions
    sec = int(duration_s)
    nanosec = int((duration_s - sec) * 1e9)
    point.time_from_start = Duration(sec=sec, nanosec=nanosec)
    msg.points = [point]
    return msg, commanded, skipped, clamp_notes


class _Publisher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("goto_pose")
        self._pub = self.create_publisher(JointTrajectory, topic, 10)

    def publish_when_ready(self, msg: JointTrajectory, deadline_s: float = 2.0) -> bool:
        """Wait for at least one subscriber, then publish once. Returns True if published."""
        deadline = self.get_clock().now().nanoseconds + int(deadline_s * 1e9)
        while self._pub.get_subscription_count() == 0:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        self._pub.publish(msg)
        # Keep the publisher alive so late DDS matches, including rosbag, see it.
        end = self.get_clock().now().nanoseconds + int(1.0 * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return True


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))
    pose_path = args.pose_dir / f"{args.name}.yaml"
    pose = load_pose(pose_path)
    msg, commanded, skipped, clamp_notes = build_trajectory(pose, args.duration)

    if not commanded:
        raise SystemExit(
            f"{pose_path} has no joints in the bridge's command set "
            f"({ALLOWED_JOINTS}). Nothing to do."
        )

    print(f"goto_pose: {args.name}  ({pose_path})")
    print(f"  commanding {len(commanded)} joint(s) over {args.duration}s:")
    for name, pos in zip(commanded, msg.points[0].positions):
        print(f"    {name:<18s}  {pos:+.4f} rad")
    if skipped:
        print(f"  skipped non-commandable joints: {skipped}")
    for note in clamp_notes:
        print(f"  WARN  {note}", file=sys.stderr)

    rclpy.init()
    node = _Publisher(args.topic)
    try:
        published = node.publish_when_ready(msg)
        if not published:
            print(
                f"no subscriber on {args.topic} within timeout. "
                f"is joint_trajectory_guard_node running?",
                file=sys.stderr,
            )
            return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"OK  trajectory published. monitor with `ros2 topic echo /joint_states` or RViz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
