from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


LEFT_ARM_JOINTS = [
    "Rotation_L",
    "Pitch_L",
    "Elbow_L",
    "Wrist_Pitch_L",
    "Wrist_Roll_L",
]

RIGHT_ARM_JOINTS = [
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
]

# Fallback limits when calibration is missing or incomplete.
URDF_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "Rotation_L": (-2.16, 2.16),
    "Pitch_L": (-0.22, 3.37),
    "Elbow_L": (-0.22, 3.14),
    "Wrist_Pitch_L": (-1.6580628, 1.6580627),
    "Wrist_Roll_L": (-2.7438473, 2.8412063),
    "Rotation_R": (-2.16, 2.16),
    "Pitch_R": (-0.22, 3.37),
    "Elbow_R": (-0.22, 3.14),
    "Wrist_Pitch_R": (-1.6580628, 1.6580627),
    "Wrist_Roll_R": (-2.7438473, 2.8412063),
}

DEFAULT_CALIBRATION_PATH = Path.home() / ".xle" / "bus1_calibration.yaml"
_STEPS_PER_RAD = 4096 / (2 * math.pi)  # STS3215 resolution


def load_joint_limits(
    cal_path: Optional[Path] = None,
) -> Tuple[Dict[str, Tuple[float, float]], Dict[str, str]]:
    """Build per-joint URDF-radian limits, preferring calibration over URDF.

    Returns (limits, sources) where sources[joint] is "calibration" or "urdf".
    Joints in calibration with both raw_min and raw_max set become cal-derived
    limits; everything else falls back to URDF_JOINT_LIMITS.
    """
    if cal_path is None:
        cal_path = DEFAULT_CALIBRATION_PATH

    limits: Dict[str, Tuple[float, float]] = dict(URDF_JOINT_LIMITS)
    sources: Dict[str, str] = {name: "urdf" for name in URDF_JOINT_LIMITS}

    if not cal_path.exists():
        return limits, sources

    try:
        data = yaml.safe_load(cal_path.read_text()) or {}
    except Exception:
        return limits, sources

    for joint, m in (data.get("motors") or {}).items():
        raw_min = m.get("raw_min")
        raw_max = m.get("raw_max")
        if raw_min is None or raw_max is None:
            continue
        sign = m.get("sign", 1)
        homing = m["homing_offset_steps"]
        a = sign * (raw_min - homing) / _STEPS_PER_RAD
        b = sign * (raw_max - homing) / _STEPS_PER_RAD
        limits[joint] = (min(a, b), max(a, b))
        sources[joint] = "calibration"

    return limits, sources


# Import-time limits for older helpers. Long-running nodes should call
# load_joint_limits() so calibration changes are picked up explicitly.
JOINT_LIMITS = load_joint_limits()[0]


@dataclass(frozen=True)
class GuardedController:
    name: str
    input_topic: str
    output_topic: str
    allowed_joints: List[str]


CONTROLLERS = [
    GuardedController(
        name="left_arm",
        input_topic="/left_arm_controller/joint_trajectory",
        output_topic="/left_arm_controller/guarded_joint_trajectory",
        allowed_joints=LEFT_ARM_JOINTS,
    ),
    GuardedController(
        name="right_arm",
        input_topic="/right_arm_controller/joint_trajectory",
        output_topic="/right_arm_controller/guarded_joint_trajectory",
        allowed_joints=RIGHT_ARM_JOINTS,
    ),
]


def _duration_to_nanoseconds(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


class JointTrajectoryGuardNode(Node):
    """Validate public arm trajectories before hardware-facing nodes consume them."""

    def __init__(self) -> None:
        super().__init__("joint_trajectory_guard_node")

        self.declare_parameter("calibration_path", str(DEFAULT_CALIBRATION_PATH))
        cal_path = Path(str(self.get_parameter("calibration_path").value))
        self._joint_limits, self._limit_sources = load_joint_limits(cal_path)

        self._harness_events_pub = self.create_publisher(
            String,
            "/harness/events",
            10,
        )
        self._guarded_publishers = {}
        for controller in CONTROLLERS:
            self._guarded_publishers[controller.name] = self.create_publisher(
                JointTrajectory,
                controller.output_topic,
                10,
            )
            self.create_subscription(
                JointTrajectory,
                controller.input_topic,
                self._callback(controller),
                10,
            )

        cal_count = sum(1 for src in self._limit_sources.values() if src == "calibration")
        self.get_logger().info(
            f"joint trajectory guard ready; limits from calibration for "
            f"{cal_count}/{len(self._limit_sources)} joints "
            f"(cal: {cal_path})"
        )
        for name in sorted(self._limit_sources):
            lo, hi = self._joint_limits[name]
            self.get_logger().info(
                f"  {name:<18s} [{lo:+.4f}, {hi:+.4f}] ({self._limit_sources[name]})"
            )

    def _callback(self, controller: GuardedController):
        def callback(msg: JointTrajectory) -> None:
            ok, reason = self._validate(controller, msg)
            if not ok:
                self.get_logger().warning(
                    f"rejected {controller.name} trajectory: {reason}"
                )
                self._publish_harness_event(
                    controller=controller,
                    msg=msg,
                    event_type="command_rejected",
                    reason=reason,
                )
                return

            self._guarded_publishers[controller.name].publish(msg)
            self._publish_harness_event(
                controller=controller,
                msg=msg,
                event_type="command_accepted",
                reason=reason,
            )
            self.get_logger().info(
                f"accepted {controller.name} trajectory with "
                f"{len(msg.joint_names)} joints and {len(msg.points)} point(s)"
            )

        return callback

    def _validate(
        self,
        controller: GuardedController,
        msg: JointTrajectory,
    ) -> Tuple[bool, str]:
        if not msg.joint_names:
            return False, "missing joint_names"
        if len(set(msg.joint_names)) != len(msg.joint_names):
            return False, "duplicate joint names"
        if not msg.points:
            return False, "missing trajectory points"

        allowed = set(controller.allowed_joints)
        unexpected = [name for name in msg.joint_names if name not in allowed]
        if unexpected:
            return False, f"unexpected joints {unexpected}"

        last_time_ns = -1
        for index, point in enumerate(msg.points):
            if len(point.positions) != len(msg.joint_names):
                return (
                    False,
                    f"point {index} has {len(point.positions)} positions for "
                    f"{len(msg.joint_names)} joints",
                )

            time_ns = _duration_to_nanoseconds(
                point.time_from_start.sec,
                point.time_from_start.nanosec,
            )
            if time_ns < last_time_ns:
                return False, f"point {index} time_from_start moves backward"
            last_time_ns = time_ns

            for name, position in zip(msg.joint_names, point.positions):
                if not math.isfinite(position):
                    return False, f"{name} position is not finite"
                lower, upper = self._joint_limits[name]
                if position < lower or position > upper:
                    source = self._limit_sources.get(name, "?")
                    return (
                        False,
                        f"{name} position {position:.4f} outside "
                        f"[{lower:.4f}, {upper:.4f}] (limit source: {source})",
                    )

        return True, "ok"

    def _publish_harness_event(
        self,
        controller: GuardedController,
        msg: JointTrajectory,
        event_type: str,
        reason: str,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        event = {
            "schema": "xle.harness.event.v0",
            "event_type": event_type,
            "source": self.get_name(),
            "controller": controller.name,
            "input_topic": controller.input_topic,
            "output_topic": controller.output_topic,
            "reason": reason,
            "joint_names": list(msg.joint_names),
            "point_count": len(msg.points),
            "stamp": {
                "sec": stamp.sec,
                "nanosec": stamp.nanosec,
            },
        }

        self._harness_events_pub.publish(String(data=json.dumps(event, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
