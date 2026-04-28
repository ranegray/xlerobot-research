from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pytest

from xle_hardware.trajectory_guard import (
    CONTROLLERS,
    LEFT_ARM_JOINTS,
    load_joint_limits,
    validate_trajectory,
)


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


def _controller(name: str = "left_arm"):
    return next(controller for controller in CONTROLLERS if controller.name == name)


def _limits(tmp_path: Path):
    return load_joint_limits(tmp_path / "missing-calibration.yaml")


def _trajectory(
    joint_names: list[str] | None = None,
    positions: list[float] | None = None,
    sec: int = 2,
) -> Trajectory:
    names = list(LEFT_ARM_JOINTS if joint_names is None else joint_names)
    values = [0.05, 0.10, 0.10, 0.0, 0.0] if positions is None else positions
    return Trajectory(names, [Point(values, Duration(sec=sec))])


def test_valid_left_arm_trajectory_is_accepted(tmp_path: Path) -> None:
    limits, sources = _limits(tmp_path)

    result = validate_trajectory(_controller(), _trajectory(), limits, sources)

    assert result.ok is True
    assert result.reason == "ok"


@pytest.mark.parametrize(
    ("trajectory", "reason"),
    [
        (
            Trajectory([], [Point([], Duration(sec=1))]),
            "missing joint_names",
        ),
        (
            Trajectory(["Rotation_L", "Rotation_L"], [Point([0.0, 0.0], Duration(sec=1))]),
            "duplicate joint names",
        ),
        (
            Trajectory(["Rotation_L"], []),
            "missing trajectory points",
        ),
        (
            Trajectory(["Rotation_R"], [Point([0.0], Duration(sec=1))]),
            "unexpected joints ['Rotation_R']",
        ),
        (
            Trajectory(["Rotation_L", "Pitch_L"], [Point([0.0], Duration(sec=1))]),
            "point 0 has 1 positions for 2 joints",
        ),
        (
            Trajectory(
                ["Rotation_L"],
                [
                    Point([0.0], Duration(sec=2)),
                    Point([0.1], Duration(sec=1)),
                ],
            ),
            "point 1 time_from_start moves backward",
        ),
        (
            Trajectory(["Rotation_L"], [Point([float("nan")], Duration(sec=1))]),
            "Rotation_L position is not finite",
        ),
    ],
)
def test_invalid_trajectories_are_rejected(
    tmp_path: Path,
    trajectory: Trajectory,
    reason: str,
) -> None:
    limits, sources = _limits(tmp_path)

    result = validate_trajectory(_controller(), trajectory, limits, sources)

    assert result.ok is False
    assert result.reason == reason


def test_out_of_limit_position_is_rejected(tmp_path: Path) -> None:
    limits, sources = _limits(tmp_path)
    trajectory = _trajectory(joint_names=["Pitch_L"], positions=[4.0])

    result = validate_trajectory(_controller(), trajectory, limits, sources)

    assert result.ok is False
    assert result.reason.startswith("Pitch_L position 4.0000 outside")
    assert "limit source: urdf" in result.reason


def test_calibration_limits_override_urdf_limits(tmp_path: Path) -> None:
    calibration = tmp_path / "bus1_calibration.yaml"
    calibration.write_text(
        "\n".join(
            [
                "motors:",
                "  Rotation_L:",
                "    raw_min: 0",
                "    raw_max: 4096",
                "    homing_offset_steps: 2048",
                "    sign: 1",
            ]
        )
    )

    limits, sources = load_joint_limits(calibration)

    assert sources["Rotation_L"] == "calibration"
    assert limits["Rotation_L"] == pytest.approx((-math.pi, math.pi))
