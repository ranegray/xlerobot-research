from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = Path(get_package_share_directory("xle_perception"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))

    default_config = perception_share / "config" / "realsense_d435if.yaml"
    rs_launch = realsense_share / "launch" / "rs_launch.py"

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=str(default_config),
        description="YAML file with realsense2_camera parameters.",
    )
    serial_no_arg = DeclareLaunchArgument(
        "serial_no",
        default_value="''",
        description="Camera serial number. Empty selects the first device.",
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(rs_launch)),
        launch_arguments={
            "camera_namespace": "",
            "camera_name": "camera",
            "config_file": LaunchConfiguration("config_file"),
            "serial_no": LaunchConfiguration("serial_no"),
        }.items(),
    )

    # Bridge the URDF chain (which ends at head_camera_link, the camera mount)
    # to realsense's frame tree (rooted at camera_link). Zero offset because
    # head_camera_link is defined in the URDF at the camera body mount point.
    # If the physical camera is offset from the URDF mount point, fix the URDF
    # joint origin (xle_description/urdf/xlerobot.urdf, fixed_head_camera_link)
    # rather than this static TF.
    head_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="head_camera_link_to_camera_link",
        arguments=["0", "0", "0", "0", "0", "0", "head_camera_link", "camera_link"],
        output="screen",
    )

    return LaunchDescription(
        [config_file_arg, serial_no_arg, realsense, head_to_camera_tf]
    )
