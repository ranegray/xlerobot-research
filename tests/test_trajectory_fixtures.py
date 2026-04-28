from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from scripts.publish_trajectory_fixture import build_ros2_command, load_fixture
from xle_hardware.trajectory_guard import CONTROLLERS, load_joint_limits, validate_trajectory


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "trajectories"


@dataclass(frozen=True)
class Duration:
    sec: int
    nanosec: int = 0


@dataclass(frozen=True)
class Point:
    positions: list[float]
    time_from_start: Duration


@dataclass(frozen=True)
class Trajectory:
    joint_names: list[str]
    points: list[Point]


def _left_arm_controller():
    return next(controller for controller in CONTROLLERS if controller.name == "left_arm")


def _trajectory_from_fixture(fixture: dict) -> Trajectory:
    message = fixture["message"]
    points = []
    for raw_point in message["points"]:
        raw_duration = raw_point.get("time_from_start") or {}
        points.append(
            Point(
                positions=[float(value) for value in raw_point["positions"]],
                time_from_start=Duration(
                    sec=int(raw_duration.get("sec", 0)),
                    nanosec=int(raw_duration.get("nanosec", 0)),
                ),
            )
        )
    return Trajectory(joint_names=list(message["joint_names"]), points=points)


def _fixtures() -> list[Path]:
    return sorted(FIXTURE_DIR.glob("*.yaml"))


def test_trajectory_fixture_files_are_guard_compatible(tmp_path: Path) -> None:
    limits, sources = load_joint_limits(tmp_path / "missing-calibration.yaml")

    for path in _fixtures():
        fixture = load_fixture(path)
        result = validate_trajectory(
            _left_arm_controller(),
            _trajectory_from_fixture(fixture),
            limits,
            sources,
        )
        expect = fixture["expect"]
        if expect["guard_event"] == "command_accepted":
            assert result.ok is True, path
            assert result.reason == expect["reason"], path
        else:
            assert result.ok is False, path
            if "reason" in expect:
                assert result.reason == expect["reason"], path
            else:
                assert result.reason.startswith(expect["reason_prefix"]), path


def test_fixture_publisher_builds_ros2_topic_pub_command() -> None:
    fixture = load_fixture(FIXTURE_DIR / "valid_left_arm_small.yaml")

    cmd = build_ros2_command(fixture)

    assert cmd[:4] == ["ros2", "topic", "pub", "--once"]
    assert cmd[4] == "/left_arm_controller/joint_trajectory"
    assert cmd[5] == "trajectory_msgs/msg/JointTrajectory"
    assert yaml.safe_load(cmd[6]) == fixture["message"]
