"""Fake-hardware bring-up plus RViz for visual joint motion.

Brings up fake_one_arm + RViz so trajectories sent to
/left_arm_controller/joint_trajectory animate the URDF.

Usage:
    ros2 launch xle_bringup view_fake_arm.launch.py

Then in another terminal:
    ros2 topic pub --once /left_arm_controller/joint_trajectory \\
        trajectory_msgs/msg/JointTrajectory '{joint_names:["Rotation_L","Pitch_L","Elbow_L","Wrist_Pitch_L","Wrist_Roll_L"], points:[{positions:[1.0,1.0,1.0,0.0,0.0], time_from_start:{sec:2}}]}'
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("xle_bringup"))
    description_share = Path(get_package_share_directory("xle_description"))

    fake_one_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "fake_one_arm.launch.py"))
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(description_share / "rviz" / "xlerobot.rviz")],
    )

    return LaunchDescription([fake_one_arm, rviz])
