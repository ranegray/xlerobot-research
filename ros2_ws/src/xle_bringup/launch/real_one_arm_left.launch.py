"""Real-hardware bring-up for the left arm only.

Composition:
    - robot_state_publisher          URDF + /tf for non-moving links
    - joint_trajectory_guard_node    validates /left_arm_controller/joint_trajectory
                                     against URDF joint names + limits, republishes
                                     to /left_arm_controller/guarded_joint_trajectory,
                                     emits /harness/events
    - bus1_sts3215_node              opens /dev/ttyACM0, reads positions for all 8
                                     bus1 motors, publishes /joint_states for all 8
                                     joints (left arm + head), subscribes
                                     /left_arm_controller/guarded_joint_trajectory
                                     and writes goal positions only for left arm
                                     motors. Refuses to start without calibration.

The right arm is intentionally absent. It will live on a second bus (bus2) on a
separate USB device when added.

Pre-flight:
    1. ros2 run xle_hardware scan_bus1            # confirm IDs 1-8 healthy
    2. ros2 run xle_hardware calibrate_bus1       # interactive; writes ~/.xle/bus1_calibration.yaml

Launch arguments:
    enable_torque:=true                    # default false; required to actually move motors
    port:=/dev/ttyACM0
    publish_rate_hz:=30.0
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("xle_description"))
    urdf_path = description_share / "urdf" / "xlerobot.urdf"

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial device for bus1 (left arm + head).",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="30.0",
        description="Joint-state publish rate from the hardware bridge.",
    )
    enable_torque_arg = DeclareLaunchArgument(
        "enable_torque",
        default_value="false",
        description="Enable torque on left arm motors at startup. False = read-only.",
    )
    calibration_arg = DeclareLaunchArgument(
        "calibration_path",
        default_value=str(Path.home() / ".xle" / "bus1_calibration.yaml"),
        description="Path to the bus1 calibration YAML.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_path.read_text()}],
    )

    joint_trajectory_guard = Node(
        package="xle_hardware",
        executable="joint_trajectory_guard_node",
        name="joint_trajectory_guard_node",
        output="screen",
    )

    bus1_bridge = Node(
        package="xle_hardware",
        executable="bus1_sts3215_node",
        name="bus1_sts3215_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "enable_torque": ParameterValue(
                    LaunchConfiguration("enable_torque"), value_type=bool
                ),
                "calibration_path": LaunchConfiguration("calibration_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            publish_rate_arg,
            enable_torque_arg,
            calibration_arg,
            robot_state_publisher,
            joint_trajectory_guard,
            bus1_bridge,
        ]
    )
