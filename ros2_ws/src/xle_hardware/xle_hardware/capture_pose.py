"""Capture the current /joint_states as a named pose YAML.

Subscribes briefly to /joint_states, takes the latest message, writes
~/.xle/poses/<name>.yaml. Use to record stow, ready, retracted, or any
other reproducible pose for later replay via goto_pose.

Usage:
    ros2 run xle_hardware capture_pose stow
    ros2 run xle_hardware capture_pose ready --output ~/my_pose.yaml
    ros2 run xle_hardware capture_pose stow --pose-dir ~/.xle/poses
"""

from __future__ import annotations

import argparse
import datetime as dt
import socket
import sys
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


SCHEMA = "xle.pose.v0"
DEFAULT_POSE_DIR = Path.home() / ".xle" / "poses"
WAIT_TIMEOUT_S = 5.0


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture current /joint_states to a named pose YAML.")
    parser.add_argument("name", help="Pose name, e.g. 'stow' or 'ready'.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Explicit output path. Default: <pose-dir>/<name>.yaml",
    )
    parser.add_argument(
        "--pose-dir",
        type=Path,
        default=DEFAULT_POSE_DIR,
        help=f"Directory for pose YAMLs (default: {DEFAULT_POSE_DIR}).",
    )
    parser.add_argument(
        "--topic",
        default="/joint_states",
        help="Source topic.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=WAIT_TIMEOUT_S,
        help=f"Seconds to wait for a message (default {WAIT_TIMEOUT_S}).",
    )
    return parser.parse_args(argv)


class _Snapshot(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("capture_pose")
        self.last: Optional[JointState] = None
        self._sub = self.create_subscription(JointState, topic, self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        self.last = msg


def write_pose_yaml(output: Path, name: str, source_topic: str, joint_state: JointState) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    lines: List[str] = []
    lines.append(f"schema: {SCHEMA}")
    lines.append(f"name: {name}")
    lines.append(f"captured_at: '{timestamp}'")
    lines.append(f"host: {socket.gethostname()}")
    lines.append(f"source_topic: {source_topic}")
    lines.append("joints:")
    for joint_name, position in zip(joint_state.name, joint_state.position):
        lines.append(f"  {joint_name}: {float(position):.6f}")
    output.write_text("\n".join(lines) + "\n")


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))
    output = args.output if args.output else (args.pose_dir / f"{args.name}.yaml")

    if output.exists():
        confirm = input(f"{output} exists. overwrite? [y/N] ")
        if confirm.strip().lower() not in {"y", "yes"}:
            print("aborted.")
            return 0

    rclpy.init()
    node = _Snapshot(args.topic)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while node.last is None and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.last is None:
            print(
                f"no message on {args.topic} within {args.timeout}s. "
                f"is the bridge or fake hardware running?",
                file=sys.stderr,
            )
            return 1
        write_pose_yaml(output, args.name, args.topic, node.last)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"OK  wrote {output}")
    print(f"    {len(node.last.name)} joint(s): {list(node.last.name)}")
    print(f"next: ros2 run xle_hardware goto_pose {args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
