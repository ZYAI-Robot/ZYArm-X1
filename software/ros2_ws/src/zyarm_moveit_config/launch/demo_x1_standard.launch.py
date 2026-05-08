from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # MoveIt 启动时需要同时装配模型、SRDF、规划器参数和控制器映射。
    description_share = Path(get_package_share_directory("zyarm_description"))
    moveit_share = Path(get_package_share_directory("zyarm_moveit_config"))
    bringup_share = Path(get_package_share_directory("zyarm_bringup"))

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    srdf_file = moveit_share / "srdf" / "zyarm_x1_standard.srdf"
    rviz_file = moveit_share / "rviz" / "moveit_x1_standard.rviz"
    kinematics_file = moveit_share / "config" / "kinematics.yaml"
    ompl_file = moveit_share / "config" / "ompl_planning.yaml"
    joint_limits_file = moveit_share / "config" / "joint_limits.yaml"
    moveit_controllers_file = moveit_share / "config" / "moveit_controllers.yaml"
    trajectory_execution_file = moveit_share / "config" / "trajectory_execution.yaml"
    planning_scene_file = moveit_share / "config" / "planning_scene_monitor_parameters.yaml"

    # 控制链版本使用 ros2_control 模式展开 xacro。
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

    robot_description_semantic = srdf_file.read_text(encoding="utf-8")
    robot_description_kinematics = load_yaml(kinematics_file)
    ompl_planning = load_yaml(ompl_file)
    joint_limits = load_yaml(joint_limits_file)
    moveit_controllers = load_yaml(moveit_controllers_file)
    trajectory_execution = load_yaml(trajectory_execution_file)
    planning_scene_monitor_parameters = load_yaml(planning_scene_file)

    # 先启动底层 ros2_control，再叠加 move_group 和 RViz。
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "bringup_x1_standard_ros2_control.launch.py")
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
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

    # RViz 使用与 move_group 同一套模型和规划参数，保证显示一致。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(rviz_file)],
        parameters=[
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
            static_world_tf,
            bringup,
            move_group,
            rviz,
        ]
    )
