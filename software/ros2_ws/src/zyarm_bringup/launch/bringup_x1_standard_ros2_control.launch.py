from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("zyarm_description"))
    control_share = Path(get_package_share_directory("zyarm_control"))

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    rviz_file = description_share / "rviz" / "display_x1_standard.rviz"
    controllers_file = control_share / "config" / "zyarm_x1_standard_controllers.yaml"

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 when true.",
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
                " ",
                "use_ros2_control:=true",
            ]
        ),
        value_type=str,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="zyarm_x1_standard_controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            str(controllers_file),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="zyarm_x1_standard_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="zyarm_x1_standard_joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="zyarm_x1_standard_arm_controller_spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="zyarm_x1_standard_gripper_controller_spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="zyarm_x1_standard_rviz2",
        output="screen",
        arguments=["-d", str(rviz_file)],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    delay_arm_controller_until_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            controller_manager,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_arm_controller_until_jsb,
            rviz,
        ]
    )
