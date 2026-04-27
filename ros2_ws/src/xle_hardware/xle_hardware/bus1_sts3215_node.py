"""Bus1 STS3215 hardware bridge node.

Owns the serial port to /dev/ttyACM0 and is the only process that talks to
the bus while the node is running. Behavior:

    Reads (always on):
        Sync-reads Present_Position from all 8 motors at publish_rate_hz.
        Converts raw steps -> URDF radians using the bus1 calibration YAML.
        Publishes /joint_states with all 8 joint names.

    Writes (gated by enable_torque param):
        Subscribes /left_arm_controller/guarded_joint_trajectory.
        On each accepted trajectory, takes the LAST point's positions,
        clamps to per-joint limits, converts to raw steps, sync-writes to
        motors 1-6 (left arm only). Head motors are read-only on this node
        until a head controller is added to joint_trajectory_guard_node.

    Safety:
        - Refuses to start without a calibration YAML.
        - On torque enable, seeds Goal_Position from current Present_Position
          per motor before writing Torque_Enable=1, so each motor holds where
          it is rather than jumping to whatever stale Goal_Position was in
          its register from a previous session. This makes torque-enable
          safe at any pose; the bridge does not gate on calibration distance.
        - Disables torque on shutdown, even on Ctrl-C / SIGTERM.
        - Sets Acceleration to acceleration_value (slow by default) when
          enabling torque.
        - The enable_torque param is coerced strictly: "false"/"true"/"0"/"1"
          etc are accepted, anything ambiguous raises rather than being
          silently truthy.
"""

from __future__ import annotations

import math
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from xle_hardware._sdk import load_sdk

scs = load_sdk()

from xle_hardware.bus1_layout import (
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    BUS1_MOTORS,
    BY_JOINT,
    COMMAND_JOINTS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DEFAULT_PROTOCOL_VERSION,
    STS3215_MODEL_NUMBER,
    STS3215_RESOLUTION,
)


SCHEMA = "xle.bus1_calibration.v0"
STEPS_PER_RAD = STS3215_RESOLUTION / (2.0 * math.pi)


_TRUE_STRS = {"true", "1", "yes", "y", "on"}
_FALSE_STRS = {"false", "0", "no", "n", "off", ""}


def _coerce_bool(value, name: str) -> bool:
    """Strict bool coercion. Refuses ambiguity rather than silently guessing.

    The naive `bool(value)` returns True for any non-empty string including
    "false", which is a safety hole for params like enable_torque. ROS 2
    launch arguments arrive as strings unless wrapped with ParameterValue.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in _TRUE_STRS:
            return True
        if normalized in _FALSE_STRS:
            return False
        raise ValueError(
            f"parameter '{name}' has ambiguous string value {value!r}; "
            f"expected one of {sorted(_TRUE_STRS | _FALSE_STRS)}"
        )
    raise TypeError(
        f"parameter '{name}' has unsupported type {type(value).__name__}; "
        f"expected bool or string"
    )


@dataclass(frozen=True)
class JointCal:
    motor_id: int
    sign: int
    homing_offset_steps: int
    raw_min: Optional[int]
    raw_max: Optional[int]

    def steps_to_rad(self, raw: int) -> float:
        return self.sign * (raw - self.homing_offset_steps) / STEPS_PER_RAD

    def rad_to_steps(self, rad: float) -> int:
        return int(round(self.homing_offset_steps + self.sign * rad * STEPS_PER_RAD))

    def clamp_steps(self, raw: int) -> int:
        lo = 0 if self.raw_min is None else self.raw_min
        hi = STS3215_RESOLUTION - 1 if self.raw_max is None else self.raw_max
        return max(lo, min(hi, raw))


def load_calibration(path: Path) -> Dict[str, JointCal]:
    if not path.exists():
        raise FileNotFoundError(
            f"calibration file not found at {path}. "
            f"run: ros2 run xle_hardware calibrate_bus1"
        )
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        raise ValueError(
            f"{path} is not a {SCHEMA} document (got schema={data.get('schema')!r})."
        )
    motors = data.get("motors") or {}
    cal: Dict[str, JointCal] = {}
    for joint_name, motor in BY_JOINT.items():
        entry = motors.get(joint_name)
        if not entry:
            raise ValueError(f"calibration is missing entry for {joint_name}")
        if entry.get("motor_id") != motor.motor_id:
            raise ValueError(
                f"{joint_name} calibration has motor_id {entry.get('motor_id')}, "
                f"expected {motor.motor_id}"
            )
        cal[joint_name] = JointCal(
            motor_id=motor.motor_id,
            sign=int(entry.get("sign", motor.default_sign)),
            homing_offset_steps=int(entry["homing_offset_steps"]),
            raw_min=entry.get("raw_min"),
            raw_max=entry.get("raw_max"),
        )
    return cal


class Bus1Sts3215Node(Node):
    """Hardware bridge for bus1 (left arm + head)."""

    def __init__(self) -> None:
        super().__init__("bus1_sts3215_node")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)
        self.declare_parameter("protocol_version", DEFAULT_PROTOCOL_VERSION)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("enable_torque", False)
        self.declare_parameter("calibration_path", str(Path.home() / ".xle" / "bus1_calibration.yaml"))
        self.declare_parameter("acceleration_value", 20)

        self._port_path = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._protocol_version = int(self.get_parameter("protocol_version").value)
        self._publish_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._enable_torque = _coerce_bool(
            self.get_parameter("enable_torque").value, "enable_torque"
        )
        self._calibration_path = Path(str(self.get_parameter("calibration_path").value))
        self._acceleration_value = int(self.get_parameter("acceleration_value").value)

        self._cal = load_calibration(self._calibration_path)
        self.get_logger().info(f"loaded calibration from {self._calibration_path}")

        self._port = scs.PortHandler(self._port_path)
        if not self._port.openPort():
            raise RuntimeError(f"could not open {self._port_path}")
        if not self._port.setBaudRate(self._baudrate):
            self._port.closePort()
            raise RuntimeError(f"could not set baudrate {self._baudrate}")
        self._packet = scs.PacketHandler(self._protocol_version)

        self._verify_bus()

        if self._enable_torque:
            self._enable_torque_holding_current_pose()
        else:
            self._torque_off_all()
            self.get_logger().info("torque DISABLED (enable_torque param is false)")

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._traj_sub = self.create_subscription(
            JointTrajectory,
            "/left_arm_controller/guarded_joint_trajectory",
            self._on_trajectory,
            10,
        )

        self._timer = self.create_timer(self._publish_period, self._read_and_publish)
        self._sigint_installed = False
        self._install_signal_handler()
        self.get_logger().info(
            f"bus1 bridge ready on {self._port_path} @ {self._baudrate} baud, "
            f"publishing {len(BUS1_MOTORS)} joints at {1.0 / self._publish_period:.1f} Hz"
        )

    def _verify_bus(self) -> None:
        for m in BUS1_MOTORS:
            model_number, comm, _ = self._packet.ping(self._port, m.motor_id)
            if comm != scs.COMM_SUCCESS:
                raise RuntimeError(
                    f"id {m.motor_id} ({m.joint_name}) did not respond to ping. "
                    f"run scan_bus1 to diagnose."
                )
            if model_number != STS3215_MODEL_NUMBER:
                raise RuntimeError(
                    f"id {m.motor_id} reports model {model_number}, expected "
                    f"{STS3215_MODEL_NUMBER}"
                )

    def _enable_torque_holding_current_pose(self) -> None:
        """Enable torque so each motor holds its current position.

        Goal_Position is seeded from Present_Position before Torque_Enable=1
        per motor, so no motor jumps when torque comes on. This is true at
        any pose; the bridge does not gate torque-enable on calibration
        distance because (a) the seed-from-present makes torque-enable
        intrinsically safe, and (b) any "are we in a known-good pose" check
        belongs against a defined safe pose like stow, not against the
        calibration zero (which is just wherever the operator stood during
        calibrate_bus1).
        """
        self.get_logger().warning("ENABLING TORQUE on left arm (motors 1-6)")
        for m in BUS1_MOTORS:
            if m.joint_name not in COMMAND_JOINTS:
                continue
            self._write_byte(m.motor_id, ADDR_ACCELERATION, self._acceleration_value)
            current_pos = self._read_position(m.motor_id)
            self._write_word(m.motor_id, ADDR_GOAL_POSITION, current_pos)
            self._write_byte(m.motor_id, ADDR_TORQUE_ENABLE, 1)

    def _torque_off_all(self) -> None:
        for m in BUS1_MOTORS:
            try:
                self._write_byte(m.motor_id, ADDR_TORQUE_ENABLE, 0)
            except RuntimeError as exc:
                self.get_logger().warning(f"could not disable torque on {m.joint_name}: {exc}")

    def _read_position(self, motor_id: int) -> int:
        value, comm, _ = self._packet.read2ByteTxRx(
            self._port, motor_id, ADDR_PRESENT_POSITION
        )
        if comm != scs.COMM_SUCCESS:
            raise RuntimeError(f"failed to read position from id={motor_id}")
        return value

    def _write_byte(self, motor_id: int, address: int, value: int) -> None:
        comm, err = self._packet.write1ByteTxRx(self._port, motor_id, address, value)
        if comm != scs.COMM_SUCCESS or err != 0:
            raise RuntimeError(
                f"write byte addr={address} on id={motor_id} failed (comm={comm}, err={err})"
            )

    def _write_word(self, motor_id: int, address: int, value: int) -> None:
        comm, err = self._packet.write2ByteTxRx(self._port, motor_id, address, value)
        if comm != scs.COMM_SUCCESS or err != 0:
            raise RuntimeError(
                f"write word addr={address} on id={motor_id} failed (comm={comm}, err={err})"
            )

    def _read_and_publish(self) -> None:
        positions: List[float] = []
        for m in BUS1_MOTORS:
            try:
                raw = self._read_position(m.motor_id)
            except RuntimeError as exc:
                self.get_logger().warning(f"read fail {m.joint_name}: {exc}")
                positions.append(float("nan"))
                continue
            positions.append(self._cal[m.joint_name].steps_to_rad(raw))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [m.joint_name for m in BUS1_MOTORS]
        msg.position = positions
        self._joint_state_pub.publish(msg)

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not self._enable_torque:
            self.get_logger().warning(
                "trajectory received but enable_torque is false; ignoring."
            )
            return
        if not msg.points:
            self.get_logger().warning("trajectory has no points; ignoring.")
            return
        unknown = [n for n in msg.joint_names if n not in COMMAND_JOINTS]
        if unknown:
            self.get_logger().warning(
                f"trajectory contains non-commandable joints {unknown}; ignoring."
            )
            return

        target = msg.points[-1]
        if len(target.positions) != len(msg.joint_names):
            self.get_logger().warning("trajectory point shape mismatch; ignoring.")
            return

        writes: List[Tuple[int, int]] = []
        for name, urdf_rad in zip(msg.joint_names, target.positions):
            cal = self._cal[name]
            raw = cal.rad_to_steps(float(urdf_rad))
            clamped = cal.clamp_steps(raw)
            if clamped != raw:
                self.get_logger().warning(
                    f"{name} target {urdf_rad:.4f} rad = raw {raw} clamped to {clamped}"
                )
            writes.append((cal.motor_id, clamped))

        for motor_id, raw in writes:
            try:
                self._write_word(motor_id, ADDR_GOAL_POSITION, raw)
            except RuntimeError as exc:
                self.get_logger().error(f"goal write fail id={motor_id}: {exc}")

    def _install_signal_handler(self) -> None:
        if self._sigint_installed:
            return
        prev_int = signal.getsignal(signal.SIGINT)
        prev_term = signal.getsignal(signal.SIGTERM)

        def handler(signum, frame):  # noqa: ANN001
            self.get_logger().info(f"signal {signum} received; disabling torque.")
            self.shutdown()
            if signum == signal.SIGINT and callable(prev_int):
                prev_int(signum, frame)
            elif signum == signal.SIGTERM and callable(prev_term):
                prev_term(signum, frame)

        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)
        self._sigint_installed = True

    def shutdown(self) -> None:
        try:
            self._torque_off_all()
        finally:
            if self._port is not None:
                self._port.closePort()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[Bus1Sts3215Node] = None
    try:
        node = Bus1Sts3215Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
