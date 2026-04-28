from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

from xle_hardware.harness_events import (
    HARNESS_EVENT_SCHEMA,
    build_guard_event,
    encode_harness_event,
)
from xle_hardware.trajectory_guard import CONTROLLERS


@dataclass(frozen=True)
class Point:
    positions: list[float]


@dataclass(frozen=True)
class Trajectory:
    joint_names: list[str]
    points: list[Point]


def _controller():
    return next(controller for controller in CONTROLLERS if controller.name == "left_arm")


def test_guard_event_schema_is_json_serializable() -> None:
    event = build_guard_event(
        controller=_controller(),
        msg=Trajectory(["Rotation_L"], [Point([0.1])]),
        event_type="command_accepted",
        reason="ok",
        source="joint_trajectory_guard_node",
        stamp_sec=10,
        stamp_nanosec=20,
    )

    assert event == {
        "schema": HARNESS_EVENT_SCHEMA,
        "event_type": "command_accepted",
        "source": "joint_trajectory_guard_node",
        "controller": "left_arm",
        "input_topic": "/left_arm_controller/joint_trajectory",
        "output_topic": "/left_arm_controller/guarded_joint_trajectory",
        "reason": "ok",
        "joint_names": ["Rotation_L"],
        "point_count": 1,
        "stamp": {"sec": 10, "nanosec": 20},
    }
    assert json.loads(encode_harness_event(event)) == event


def test_guard_event_rejects_unknown_event_type() -> None:
    with pytest.raises(ValueError, match="unsupported guard event_type"):
        build_guard_event(
            controller=_controller(),
            msg=Trajectory(["Rotation_L"], [Point([0.1])]),
            event_type="command_forwarded",
            reason="ok",
            source="joint_trajectory_guard_node",
            stamp_sec=10,
            stamp_nanosec=20,
        )

