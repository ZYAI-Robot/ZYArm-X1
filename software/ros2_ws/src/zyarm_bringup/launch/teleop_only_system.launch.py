from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hardware_share = Path(get_package_share_directory("zyarm_hardware"))
    default_config = hardware_share / "config" / "teleop_pair_real.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Path to the zyarm_hardware arm_system YAML config.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(hardware_share / "launch" / "arm_system.launch.py")
                ),
                launch_arguments={"config_file": LaunchConfiguration("config_file")}.items(),
            ),
        ]
    )
