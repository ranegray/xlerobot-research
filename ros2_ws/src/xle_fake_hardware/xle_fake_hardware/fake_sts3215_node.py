from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


DEFAULT_JOINT_NAMES = [
    "left_wheel_joint",
    "right_wheel_joint",
    "Rotation_L",
    "Pitch_L",
    "Elbow_L",
    "Wrist_Pitch_L",
    "Wrist_Roll_L",
    "Jaw_L",
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
    "Jaw_R",
    "head_pan_joint",
    "head_tilt_joint",
]

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

JOINT_LIMITS = {
    "left_wheel_joint": (None, None),
    "right_wheel_joint": (None, None),
    "Rotation_L": (-2.16, 2.16),
    "Pitch_L": (-0.22, 3.37),
    "Elbow_L": (-0.22, 3.14),
    "Wrist_Pitch_L": (-1.6580628, 1.6580627),
    "Wrist_Roll_L": (-2.7438473, 2.8412063),
    "Jaw_L": (-0.374533, 1.7453292),
    "Rotation_R": (-2.16, 2.16),
    "Pitch_R": (-0.22, 3.37),
    "Elbow_R": (-0.22, 3.14),
    "Wrist_Pitch_R": (-1.6580628, 1.6580627),
    "Wrist_Roll_R": (-2.7438473, 2.8412063),
    "Jaw_R": (-0.374533, 1.7453292),
    "head_pan_joint": (-3.2, 3.2),
    "head_tilt_joint": (-0.76, 1.45),
}


@dataclass(frozen=True)
class ControllerSpec:
    topic: str
    default_joints: List[str]


CONTROLLERS = [
    ControllerSpec(
        topic="/left_arm_controller/guarded_joint_trajectory",
        default_joints=LEFT_ARM_JOINTS,
    ),
    ControllerSpec(
        topic="/right_arm_controller/guarded_joint_trajectory",
        default_joints=RIGHT_ARM_JOINTS,
    ),
]


def _coerce_string_array(value: Iterable[str]) -> List[str]:
    return [str(item) for item in value]


def _clamp_joint(name: str, value: float) -> float:
    lower, upper = JOINT_LIMITS.get(name, (None, None))
    if lower is not None and value < lower:
        return lower
    if upper is not None and value > upper:
        return upper
    return value


class FakeSts3215Node(Node):
    """Interface-compatible fake STS3215 state and command node."""

    def __init__(self) -> None:
        super().__init__("fake_sts3215_node")

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_velocity_rad_s", 0.75)

        self._joint_names = _coerce_string_array(
            self.get_parameter("joint_names").value
        )
        self._max_velocity = float(self.get_parameter("max_velocity_rad_s").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._positions: Dict[str, float] = {name: 0.0 for name in self._joint_names}
        self._targets: Dict[str, float] = dict(self._positions)
        self._last_update_time = self.get_clock().now()

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        for controller in CONTROLLERS:
            self.create_subscription(
                JointTrajectory,
                controller.topic,
                self._trajectory_callback(controller),
                10,
            )

        self.create_timer(1.0 / publish_rate_hz, self._tick)
        self.get_logger().info(
            f"fake STS3215 publishing {len(self._joint_names)} joints at "
            f"{publish_rate_hz:.1f} Hz"
        )

    def _trajectory_callback(self, controller: ControllerSpec):
        def callback(msg: JointTrajectory) -> None:
            if not msg.points:
                self.get_logger().warning(f"ignored empty trajectory on {controller.topic}")
                return

            point = msg.points[-1]
            if len(point.positions) != len(msg.joint_names):
                self.get_logger().warning(
                    f"ignored malformed trajectory on {controller.topic}: "
                    f"{len(msg.joint_names)} names, {len(point.positions)} positions"
                )
                return

            command = dict(zip(msg.joint_names, point.positions))
            unknown = [name for name in command if name not in self._positions]
            if unknown:
                self.get_logger().warning(
                    f"ignored unknown joints on {controller.topic}: {unknown}"
                )
                return

            allowed = set(controller.default_joints)
            unexpected = [name for name in command if name not in allowed]
            if unexpected:
                self.get_logger().warning(
                    f"accepted trajectory on {controller.topic} with non-default joints: "
                    f"{unexpected}"
                )

            for name, value in command.items():
                self._targets[name] = _clamp_joint(name, float(value))

        return callback

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds / 1_000_000_000.0
        self._last_update_time = now

        max_step = max(self._max_velocity * dt, 0.0)
        for name in self._joint_names:
            position = self._positions[name]
            target = self._targets[name]
            delta = target - position
            if abs(delta) <= max_step:
                self._positions[name] = target
            elif delta > 0.0:
                self._positions[name] = position + max_step
            else:
                self._positions[name] = position - max_step

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self._joint_names)
        msg.position = [self._positions[name] for name in self._joint_names]
        msg.velocity = [0.0 for _ in self._joint_names]
        msg.effort = [0.0 for _ in self._joint_names]
        self._joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSts3215Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
