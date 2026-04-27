"""Single source of truth for the bus1 motor inventory.

Bus1 carries the left arm (IDs 1-6) and the head pan/tilt (IDs 7-8) on a single
1 Mbps Feetech STS3215 daisy chain via /dev/ttyACM0.

Joint names match the URDF in xle_description and the joint name lists in
xle_hardware.joint_trajectory_guard_node and xle_fake_hardware.fake_sts3215_node.
Add a new joint here only if it is also added to those.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_PROTOCOL_VERSION = 0

STS3215_MODEL_NUMBER = 777
STS3215_RESOLUTION = 4096

ADDR_FIRMWARE_MAJOR = 0
ADDR_FIRMWARE_MINOR = 1
ADDR_MODEL_NUMBER = 3
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63

BAUDRATE_INDEX = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}


@dataclass(frozen=True)
class Bus1Motor:
    motor_id: int
    joint_name: str
    role: str
    default_sign: int = 1
    """+1 if motor's increasing step direction matches URDF positive direction.

    -1 if the motor is mounted such that increasing motor steps move the joint
    in the URDF-negative direction. Inherited from the pincer reference setup.
    Re-derive at calibration time if motors are remounted with different polarity.
    """


BUS1_MOTORS: List[Bus1Motor] = [
    Bus1Motor(1, "Rotation_L", "left arm shoulder pan", default_sign=1),
    Bus1Motor(2, "Pitch_L", "left arm shoulder lift", default_sign=-1),
    Bus1Motor(3, "Elbow_L", "left arm elbow flex", default_sign=1),
    Bus1Motor(4, "Wrist_Pitch_L", "left arm wrist flex", default_sign=1),
    Bus1Motor(5, "Wrist_Roll_L", "left arm wrist roll", default_sign=1),
    Bus1Motor(6, "Jaw_L", "left arm gripper", default_sign=1),
    Bus1Motor(7, "head_pan_joint", "head pan", default_sign=-1),
    Bus1Motor(8, "head_tilt_joint", "head tilt", default_sign=1),
]


COMMAND_JOINTS = [m.joint_name for m in BUS1_MOTORS if m.joint_name.endswith("_L")]
"""Joint names that the bridge accepts trajectory commands for.

Initially: left arm only. Head is read-only on bus1 until a head controller
exists in joint_trajectory_guard_node.
"""

BY_ID: Dict[int, Bus1Motor] = {m.motor_id: m for m in BUS1_MOTORS}
BY_JOINT: Dict[str, Bus1Motor] = {m.joint_name: m for m in BUS1_MOTORS}

LEFT_ARM_IDS: List[int] = [m.motor_id for m in BUS1_MOTORS if m.joint_name.endswith("_L")]
HEAD_IDS: List[int] = [m.motor_id for m in BUS1_MOTORS if "head" in m.joint_name]
EXPECTED_IDS: List[int] = [m.motor_id for m in BUS1_MOTORS]
