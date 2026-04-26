from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("xle_description"))
    bringup_share = Path(get_package_share_directory("xle_bringup"))

    urdf_path = description_share / "urdf" / "xlerobot.urdf"
    fake_hardware_params = bringup_share / "config" / "fake_hardware.yaml"

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

    fake_sts3215 = Node(
        package="xle_fake_hardware",
        executable="fake_sts3215_node",
        name="fake_sts3215_node",
        output="screen",
        parameters=[str(fake_hardware_params)],
    )

    return LaunchDescription(
        [robot_state_publisher, joint_trajectory_guard, fake_sts3215]
    )
