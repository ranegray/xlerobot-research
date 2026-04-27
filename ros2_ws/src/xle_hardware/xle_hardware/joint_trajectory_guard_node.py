from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
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

JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
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

        self.get_logger().info("joint trajectory guard ready")

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
                lower, upper = JOINT_LIMITS[name]
                if position < lower or position > upper:
                    return (
                        False,
                        f"{name} position {position:.4f} outside "
                        f"[{lower:.4f}, {upper:.4f}]",
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
