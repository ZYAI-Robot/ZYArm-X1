from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("zyarm_hardware"))
    default_config = share_dir / "config" / "teleop_pair_real.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Path to the arm-system YAML config.",
            ),
            Node(
                package="zyarm_hardware",
                executable="arm_system",
                name="zyarm_arm_system",
                output="screen",
                parameters=[{"config_file": LaunchConfiguration("config_file")}],
            ),
        ]
    )
