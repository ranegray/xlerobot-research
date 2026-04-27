"""Record a guarded left-arm run to a timestamped rosbag.

Captures the topics listed in the Phase 2 done-when contract:
    /joint_states /tf /tf_static /robot_description
    /left_arm_controller/joint_trajectory
    /left_arm_controller/guarded_joint_trajectory
    /harness/events

Output: <output-dir>/<name>_<UTC-timestamp>/

Usage:
    ros2 run xle_hardware record_left_arm_run                  # name=run, dir=./bags
    ros2 run xle_hardware record_left_arm_run stow_test
    ros2 run xle_hardware record_left_arm_run reach1 --output-dir ~/runs

Press Ctrl-C to stop the recording.
"""

from __future__ import annotations

import argparse
import datetime as dt
import subprocess
import sys
from pathlib import Path
from typing import List


DEFAULT_TOPICS = [
    "/joint_states",
    "/tf",
    "/tf_static",
    "/robot_description",
    "/left_arm_controller/joint_trajectory",
    "/left_arm_controller/guarded_joint_trajectory",
    "/harness/events",
]


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record a guarded left-arm run.")
    parser.add_argument("name", nargs="?", default="run", help="Run name prefix.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.cwd() / "bags",
        help="Directory under which to create the timestamped bag.",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=DEFAULT_TOPICS,
        help="Topics to record (default: Phase 2 contract topics).",
    )
    parser.add_argument(
        "--storage",
        default="sqlite3",
        choices=["sqlite3", "mcap"],
        help="Bag storage backend. mcap requires `ros-humble-rosbag2-storage-mcap`; "
             "sqlite3 is the default and always available.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))

    ts = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = args.output_dir / f"{args.name}_{ts}"
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2", "bag", "record",
        "-o", str(out),
        "--storage", args.storage,
        *args.topics,
    ]
    print(f"recording -> {out}")
    print(f"topics: {' '.join(args.topics)}")
    print("press Ctrl-C to stop")
    print()
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
