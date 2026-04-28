from __future__ import annotations

import json
from typing import Any

from xle_hardware.trajectory_guard import GuardedController, JointTrajectoryLike


HARNESS_EVENT_SCHEMA = "xle.harness.event.v0"
GUARD_EVENT_TYPES = {"command_accepted", "command_rejected"}


def build_guard_event(
    *,
    controller: GuardedController,
    msg: JointTrajectoryLike,
    event_type: str,
    reason: str,
    source: str,
    stamp_sec: int,
    stamp_nanosec: int,
) -> dict[str, Any]:
    if event_type not in GUARD_EVENT_TYPES:
        raise ValueError(f"unsupported guard event_type: {event_type}")
    return {
        "schema": HARNESS_EVENT_SCHEMA,
        "event_type": event_type,
        "source": source,
        "controller": controller.name,
        "input_topic": controller.input_topic,
        "output_topic": controller.output_topic,
        "reason": reason,
        "joint_names": list(msg.joint_names),
        "point_count": len(msg.points),
        "stamp": {
            "sec": int(stamp_sec),
            "nanosec": int(stamp_nanosec),
        },
    }


def encode_harness_event(event: dict[str, Any]) -> str:
    return json.dumps(event, sort_keys=True)

