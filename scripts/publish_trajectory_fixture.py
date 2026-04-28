#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


SCHEMA = "xle.trajectory_fixture.v0"
MESSAGE_TYPE = "trajectory_msgs/msg/JointTrajectory"


def load_fixture(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    if data.get("schema") != SCHEMA:
        raise ValueError(f"{path} must use schema {SCHEMA!r}")
    if not data.get("topic"):
        raise ValueError(f"{path} is missing topic")
    message = data.get("message")
    if not isinstance(message, dict):
        raise ValueError(f"{path} is missing message mapping")
    if not message.get("joint_names"):
        raise ValueError(f"{path} message is missing joint_names")
    if not message.get("points"):
        raise ValueError(f"{path} message is missing points")
    return data


def fixture_message_yaml(fixture: dict[str, Any]) -> str:
    return yaml.safe_dump(
        fixture["message"],
        default_flow_style=True,
        sort_keys=False,
        width=4096,
    ).strip()


def build_ros2_command(fixture: dict[str, Any]) -> list[str]:
    return [
        "ros2",
        "topic",
        "pub",
        "--once",
        str(fixture["topic"]),
        MESSAGE_TYPE,
        fixture_message_yaml(fixture),
    ]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a trajectory fixture with `ros2 topic pub --once`."
    )
    parser.add_argument("fixture", type=Path, help="Trajectory fixture YAML path.")
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print the ros2 command instead of running it.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    fixture = load_fixture(args.fixture)
    cmd = build_ros2_command(fixture)
    if args.print:
        print(shlex.join(cmd))
        return 0
    return subprocess.run(cmd, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())

