from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root")


def _package_share(package_name: str) -> Path:
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return _repo_root() / "software/ros2_ws" / "src" / package_name


def _default_controllers_file() -> Path:
    return _package_share("zyarm_control") / "config" / "zyarm_x1_standard_real_controllers.yaml"


def _real_hardware_launch_arguments():
    return [
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="230400"),
        DeclareLaunchArgument("read_timeout_ms", default_value="20"),
        DeclareLaunchArgument("write_timeout_ms", default_value="20"),
        DeclareLaunchArgument("activation_status_timeout_ms", default_value="1000"),
        DeclareLaunchArgument("status_stale_warn_ms", default_value="100"),
        DeclareLaunchArgument("status_stale_error_ms", default_value="1000"),
        DeclareLaunchArgument("stale_log_period_ms", default_value="2000"),
        DeclareLaunchArgument("reset_rts_dtr", default_value="false"),
        DeclareLaunchArgument("reset_rts_dtr_quiet_ms", default_value="0"),
        DeclareLaunchArgument("arm_hw_offsets_deg", default_value="0,-180,90,0,0,0"),
        DeclareLaunchArgument("arm_hw_signs", default_value="1,1,1,1,1,1"),
        DeclareLaunchArgument("claw_travel_m", default_value="0.034"),
        DeclareLaunchArgument("claw_command_max", default_value="100"),
    ]


def _real_hardware_xacro_arguments():
    return [
        " ",
        "use_ros2_control:=true",
        " ",
        "use_gazebo:=false",
        " ",
        "use_real_hardware_interface:=true",
        " ",
        "real_hardware_port:=",
        LaunchConfiguration("serial_port"),
        " ",
        "real_hardware_baud_rate:=",
        LaunchConfiguration("baud_rate"),
        " ",
        "real_hardware_read_timeout_ms:=",
        LaunchConfiguration("read_timeout_ms"),
        " ",
        "real_hardware_write_timeout_ms:=",
        LaunchConfiguration("write_timeout_ms"),
        " ",
        "real_hardware_activation_status_timeout_ms:=",
        LaunchConfiguration("activation_status_timeout_ms"),
        " ",
        "real_hardware_status_stale_warn_ms:=",
        LaunchConfiguration("status_stale_warn_ms"),
        " ",
        "real_hardware_status_stale_error_ms:=",
        LaunchConfiguration("status_stale_error_ms"),
        " ",
        "real_hardware_stale_log_period_ms:=",
        LaunchConfiguration("stale_log_period_ms"),
        " ",
        "real_hardware_reset_rts_dtr:=",
        LaunchConfiguration("reset_rts_dtr"),
        " ",
        "real_hardware_reset_rts_dtr_quiet_ms:=",
        LaunchConfiguration("reset_rts_dtr_quiet_ms"),
        " ",
        "real_hardware_arm_hw_offsets_deg:=",
        LaunchConfiguration("arm_hw_offsets_deg"),
        " ",
        "real_hardware_arm_hw_signs:=",
        LaunchConfiguration("arm_hw_signs"),
        " ",
        "real_hardware_claw_travel_m:=",
        LaunchConfiguration("claw_travel_m"),
        " ",
        "real_hardware_claw_command_max:=",
        LaunchConfiguration("claw_command_max"),
    ]


def generate_launch_description():
    description_share = _package_share("zyarm_description")

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    rviz_file = description_share / "rviz" / "display_x1_standard.rviz"
    controllers_file = _default_controllers_file()

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
                *_real_hardware_xacro_arguments(),
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
            *_real_hardware_launch_arguments(),
            controller_manager,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_arm_controller_until_jsb,
            rviz,
        ]
    )
