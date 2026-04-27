"""Real-hardware bring-up plus RViz so the URDF tracks the physical arm.

Includes real_one_arm_left.launch.py and adds RViz with the xlerobot config.
All launch args are forwarded to the inner launch (port, publish_rate_hz,
enable_torque, calibration_path).

Examples:
    ros2 launch xle_bringup view_real_arm.launch.py
    ros2 launch xle_bringup view_real_arm.launch.py enable_torque:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("xle_bringup"))
    description_share = Path(get_package_share_directory("xle_description"))

    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyACM0")
    publish_rate_arg = DeclareLaunchArgument("publish_rate_hz", default_value="30.0")
    enable_torque_arg = DeclareLaunchArgument("enable_torque", default_value="false")
    calibration_arg = DeclareLaunchArgument(
        "calibration_path",
        default_value=str(Path.home() / ".xle" / "bus1_calibration.yaml"),
    )

    real_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "real_one_arm_left.launch.py")
        ),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "enable_torque": LaunchConfiguration("enable_torque"),
            "calibration_path": LaunchConfiguration("calibration_path"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(description_share / "rviz" / "xlerobot.rviz")],
    )

    return LaunchDescription(
        [
            port_arg,
            publish_rate_arg,
            enable_torque_arg,
            calibration_arg,
            real_arm,
            rviz,
        ]
    )
