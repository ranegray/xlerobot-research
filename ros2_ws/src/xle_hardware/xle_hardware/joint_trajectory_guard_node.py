from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from xle_hardware.harness_events import build_guard_event, encode_harness_event
from xle_hardware.trajectory_guard import (
    CONTROLLERS,
    DEFAULT_CALIBRATION_PATH,
    GuardedController,
    load_joint_limits,
    validate_trajectory,
)


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
            result = validate_trajectory(
                controller,
                msg,
                self._joint_limits,
                self._limit_sources,
            )
            if not result.ok:
                self.get_logger().warning(
                    f"rejected {controller.name} trajectory: {result.reason}"
                )
                self._publish_harness_event(
                    controller=controller,
                    msg=msg,
                    event_type="command_rejected",
                    reason=result.reason,
                )
                return

            self._guarded_publishers[controller.name].publish(msg)
            self._publish_harness_event(
                controller=controller,
                msg=msg,
                event_type="command_accepted",
                reason=result.reason,
            )
            self.get_logger().info(
                f"accepted {controller.name} trajectory with "
                f"{len(msg.joint_names)} joints and {len(msg.points)} point(s)"
            )

        return callback

    def _publish_harness_event(
        self,
        controller: GuardedController,
        msg: JointTrajectory,
        event_type: str,
        reason: str,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        event = build_guard_event(
            controller=controller,
            msg=msg,
            event_type=event_type,
            reason=reason,
            source=self.get_name(),
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
        )

        self._harness_events_pub.publish(String(data=encode_harness_event(event)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
