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
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_world_robot_base_pose(world_config_path: Path):
    return list(load_yaml(world_config_path)["robot_base_pose"])


def _build_world_tf_arguments(robot_base_pose):
    x, y, z, roll, pitch, yaw = robot_base_pose
    return [
        "--x",
        str(x),
        "--y",
        str(y),
        "--z",
        str(z),
        "--roll",
        str(roll),
        "--pitch",
        str(pitch),
        "--yaw",
        str(yaw),
        "--frame-id",
        "world",
        "--child-frame-id",
        "base_link",
    ]


def _common_runtime_parameters():
    return [{"use_sim_time": True}]


def generate_launch_description():
    description_share = Path(get_package_share_directory("zyarm_description"))
    control_share = Path(get_package_share_directory("zyarm_control"))
    moveit_share = Path(get_package_share_directory("zyarm_moveit_config"))
    gazebo_share = Path(get_package_share_directory("zyarm_gazebo"))

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    camera_config_file = description_share / "config" / "x1_standard" / "gazebo_camera_frames.yaml"
    gazebo_controllers_file = control_share / "config" / "zyarm_x1_standard_gazebo_controllers.yaml"
    srdf_file = moveit_share / "srdf" / "zyarm_x1_standard_gazebo.srdf"
    rviz_file = moveit_share / "rviz" / "moveit_x1_standard.rviz"
    kinematics_file = moveit_share / "config" / "kinematics.yaml"
    ompl_file = moveit_share / "config" / "ompl_planning.yaml"
    joint_limits_file = moveit_share / "config" / "joint_limits.yaml"
    moveit_controllers_file = moveit_share / "config" / "moveit_gazebo_controllers.yaml"
    trajectory_execution_file = moveit_share / "config" / "trajectory_execution.yaml"
    planning_scene_file = moveit_share / "config" / "planning_scene_monitor_parameters.yaml"
    world_config_file = gazebo_share / "config" / "world.yaml"

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
                " ",
                "use_ros2_control:=true",
                " ",
                "use_gazebo:=true",
                " ",
                "use_sim_cameras:=true",
                " ",
                f"gazebo_camera_config_file:={camera_config_file}",
                " ",
                f"gazebo_controller_config_file:={gazebo_controllers_file}",
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

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "bringup_pick_place_world.launch.py")
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=_common_runtime_parameters() + [
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
        parameters=_common_runtime_parameters() + [
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
        arguments=_build_world_tf_arguments(_load_world_robot_base_pose(world_config_file)),
    )

    return LaunchDescription(
        [
            static_world_tf,
            gazebo_bringup,
            move_group,
            rviz,
        ]
    )
