from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


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


def _common_runtime_parameters():
    return [{"use_sim_time": False}]


def _real_hardware_launch_arguments():
    return [
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


def _real_hardware_bringup_arguments():
    return {
        "use_rviz": "false",
        "serial_port": LaunchConfiguration("serial_port"),
        "baud_rate": LaunchConfiguration("baud_rate"),
        "read_timeout_ms": LaunchConfiguration("read_timeout_ms"),
        "write_timeout_ms": LaunchConfiguration("write_timeout_ms"),
        "activation_status_timeout_ms": LaunchConfiguration("activation_status_timeout_ms"),
        "status_stale_warn_ms": LaunchConfiguration("status_stale_warn_ms"),
        "status_stale_error_ms": LaunchConfiguration("status_stale_error_ms"),
        "stale_log_period_ms": LaunchConfiguration("stale_log_period_ms"),
        "reset_rts_dtr": LaunchConfiguration("reset_rts_dtr"),
        "reset_rts_dtr_quiet_ms": LaunchConfiguration("reset_rts_dtr_quiet_ms"),
        "arm_hw_offsets_deg": LaunchConfiguration("arm_hw_offsets_deg"),
        "arm_hw_signs": LaunchConfiguration("arm_hw_signs"),
        "claw_travel_m": LaunchConfiguration("claw_travel_m"),
        "claw_command_max": LaunchConfiguration("claw_command_max"),
    }


def generate_launch_description():
    description_share = _package_share("zyarm_description")
    moveit_share = _package_share("zyarm_moveit_config")
    bringup_share = _package_share("zyarm_bringup")

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    srdf_file = moveit_share / "srdf" / "zyarm_x1_standard.srdf"
    rviz_file = moveit_share / "rviz" / "moveit_x1_standard.rviz"
    kinematics_file = moveit_share / "config" / "kinematics.yaml"
    ompl_file = moveit_share / "config" / "ompl_planning.yaml"
    joint_limits_file = moveit_share / "config" / "joint_limits.yaml"
    moveit_controllers_file = moveit_share / "config" / "moveit_real_controllers.yaml"
    trajectory_execution_file = moveit_share / "config" / "trajectory_execution.yaml"
    planning_scene_file = moveit_share / "config" / "planning_scene_monitor_parameters.yaml"

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

    robot_description_semantic = srdf_file.read_text(encoding="utf-8")
    robot_description_kinematics = load_yaml(kinematics_file)
    ompl_planning = load_yaml(ompl_file)
    joint_limits = load_yaml(joint_limits_file)
    moveit_controllers = load_yaml(moveit_controllers_file)
    trajectory_execution = load_yaml(trajectory_execution_file)
    planning_scene_monitor_parameters = load_yaml(planning_scene_file)

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "bringup_x1_standard_real_ros2_control.launch.py")
        ),
        launch_arguments=_real_hardware_bringup_arguments().items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=_common_runtime_parameters()
        + [
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": robot_description_kinematics},
            {"robot_description_planning": joint_limits},
            {"planning_pipelines": ["ompl"]},
            {"default_planning_pipeline": "ompl"},
            {"ompl": ompl_planning},
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"capabilities": ""},
            {"disable_capabilities": ""},
            {"monitor_dynamics": False},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_file)],
        parameters=_common_runtime_parameters()
        + [
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": robot_description_kinematics},
            {"robot_description_planning": joint_limits},
        ],
    )

    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zyarm_world_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription(
        [
            *_real_hardware_launch_arguments(),
            static_world_tf,
            bringup,
            move_group,
            rviz,
        ]
    )
