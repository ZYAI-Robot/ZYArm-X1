from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("zyarm_description"))
    rviz_config = (
        Path(get_package_share_directory("zyarm_dataset_replay"))
        / "rviz/quality_check.rviz"
    )
    xacro_file = description_share / "urdf" / "2b" / "robot.urdf.xacro"

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
                " ",
                "use_ros2_control:=false",
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root", default_value=""),
            DeclareLaunchArgument("dataset_repo", default_value=""),
            DeclareLaunchArgument("dataset_id", default_value=""),
            DeclareLaunchArgument("episode_index", default_value="0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="zyarm_quality_check_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="zyarm_dataset_replay",
                executable="dataset_replay",
                name="zyarm_dataset_replay",
                output="screen",
                parameters=[
                    {
                        "dataset_root": LaunchConfiguration("dataset_root"),
                        "dataset_repo": LaunchConfiguration("dataset_repo"),
                        "dataset_id": LaunchConfiguration("dataset_id"),
                        "episode_index": LaunchConfiguration("episode_index"),
                        "_quality_check_mode": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_quality_check",
                output="screen",
                arguments=["-d", str(rviz_config)],
            ),
        ]
    )
