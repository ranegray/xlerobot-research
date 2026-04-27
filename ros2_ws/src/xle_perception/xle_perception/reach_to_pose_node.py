"""Subscribe to /detected_object/pose and reach the left arm to it via IK.

Pipeline:
    /detected_object/pose (in world frame, from color_object_detector_node)
        -> TF transform to Base frame
        -> apply optional pregrasp Z offset
        -> ikpy on the URDF chain Base -> Fixed_Jaw with current /joint_states as seed
        -> publish JointTrajectory to /left_arm_controller/joint_trajectory

The guard validates against per-joint limits before the bridge writes goals,
so a bad IK solution (out of joint range) gets rejected at the guard.

Throttled: only processes a new target if it moves more than
target_change_threshold_m from the last accepted or rejected target. This avoids
spamming the bus or harness events while the detector reports near-identical
positions every frame.

Parameters:
    pregrasp_offset_z_m: float (default 0.05)   approach pose this far above target
    duration_s: float (default 3.0)             trajectory time_from_start
    target_change_threshold_m: float (default 0.02)
    max_target_distance_m: float (default 0.5)  refuse targets beyond this from base
    max_ik_error_m: float (default 0.025)       refuse IK solves farther than this
    enable: bool (default false)                 if false, computes IK + logs but does not publish
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from xle_perception.arm_ik import ARM_JOINTS, ArmIK, BASE_LINK


class ReachToPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("reach_to_pose_node")

        self.declare_parameter("pregrasp_offset_z_m", 0.05)
        self.declare_parameter("duration_s", 3.0)
        self.declare_parameter("target_change_threshold_m", 0.02)
        self.declare_parameter("max_target_distance_m", 0.5)
        self.declare_parameter("max_ik_error_m", 0.025)
        self.declare_parameter("enable", False)

        self._pregrasp_z = float(self.get_parameter("pregrasp_offset_z_m").value)
        self._duration_s = float(self.get_parameter("duration_s").value)
        self._target_change_threshold = float(self.get_parameter("target_change_threshold_m").value)
        self._max_target_distance = float(self.get_parameter("max_target_distance_m").value)
        self._max_ik_error = float(self.get_parameter("max_ik_error_m").value)
        self._enable = self._coerce_bool(self.get_parameter("enable").value)

        urdf_path = (
            Path(get_package_share_directory("xle_description")) / "urdf" / "xlerobot.urdf"
        )
        self._ik = ArmIK(urdf_path)
        self.get_logger().info(f"loaded URDF chain {BASE_LINK} -> Fixed_Jaw from {urdf_path}")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._latest_joint_state: Dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(PoseStamped, "/detected_object/pose", self._on_pose, 10)

        self._traj_pub = self.create_publisher(
            JointTrajectory, "/left_arm_controller/joint_trajectory", 10
        )
        self._harness_events_pub = self.create_publisher(String, "/harness/events", 10)
        self._last_target_base: Optional[np.ndarray] = None

        mode = (
            "ENABLED (will publish trajectories)"
            if self._enable
            else "DISABLED (dry-run, log only)"
        )
        self.get_logger().info(
            f"reach_to_pose ready; mode={mode}; pregrasp_z={self._pregrasp_z}m "
            f"max_ik_error={self._max_ik_error}m"
        )

    @staticmethod
    def _coerce_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "y", "on")
        return bool(value)

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._latest_joint_state[name] = float(position)

    def _on_pose(self, msg: PoseStamped) -> None:
        try:
            self._process_pose(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"unhandled error in reach callback: {exc!r}")

    def _process_pose(self, msg: PoseStamped) -> None:
        # Transform target into the arm's base frame
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_LINK,
                msg.header.frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"TF lookup failed {msg.header.frame_id} -> {BASE_LINK}: {exc}"
            )
            return

        target_in_base_pose = do_transform_pose(msg.pose, tf)
        x = target_in_base_pose.position.x
        y = target_in_base_pose.position.y
        z = target_in_base_pose.position.z + self._pregrasp_z
        target_in_base = np.array([x, y, z])

        distance = float(np.linalg.norm(target_in_base))
        if distance > self._max_target_distance:
            self.get_logger().info(
                f"target {target_in_base.round(3)} is {distance:.3f}m from base; "
                f"exceeds max_target_distance={self._max_target_distance}m. ignoring."
            )
            return

        # Throttle: skip if the target hasn't meaningfully moved.
        if (
            self._last_target_base is not None
            and np.linalg.norm(target_in_base - self._last_target_base)
            < self._target_change_threshold
        ):
            return

        # Solve IK seeded from current joint state
        if not self._latest_joint_state:
            self.get_logger().info("no /joint_states yet; skipping IK.")
            return

        result = self._ik.solve(target_in_base, current_q=self._latest_joint_state)
        self.get_logger().info(
            f"target_in_base={target_in_base.round(3)}  "
            f"achieved={result.achieved_xyz.round(3)}  err={result.error_m * 1000:.1f}mm  "
            f"q={ {k: round(v, 3) for k, v in result.joint_positions.items()} }"
        )

        if result.error_m > self._max_ik_error:
            reason = (
                f"ik_error {result.error_m:.4f}m exceeds "
                f"max_ik_error_m {self._max_ik_error:.4f}m"
            )
            self.get_logger().warning(f"rejecting target: {reason}")
            self._publish_harness_event(
                event_type="target_rejected",
                reason=reason,
                target_in_base=target_in_base,
                achieved_xyz=result.achieved_xyz,
                ik_error_m=result.error_m,
            )
            self._last_target_base = target_in_base
            return

        if not self._enable:
            return

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [result.joint_positions[j] for j in ARM_JOINTS]
        sec = int(self._duration_s)
        nanosec = int((self._duration_s - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [point]
        self._traj_pub.publish(traj)
        self._last_target_base = target_in_base

    def _publish_harness_event(
        self,
        *,
        event_type: str,
        reason: str,
        target_in_base: np.ndarray,
        achieved_xyz: np.ndarray,
        ik_error_m: float,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        event = {
            "schema": "xle.harness.event.v0",
            "event_type": event_type,
            "source": self.get_name(),
            "reason": reason,
            "target_in_base_m": [float(v) for v in target_in_base],
            "achieved_xyz_m": [float(v) for v in achieved_xyz],
            "ik_error_m": float(ik_error_m),
            "max_ik_error_m": self._max_ik_error,
            "stamp": {
                "sec": stamp.sec,
                "nanosec": stamp.nanosec,
            },
        }
        self._harness_events_pub.publish(String(data=json.dumps(event, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachToPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
