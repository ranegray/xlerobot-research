from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("xle_description"))
    urdf_path = description_share / "urdf" / "xlerobot.urdf"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_path.read_text()}],
    )

    return LaunchDescription([robot_state_publisher])
