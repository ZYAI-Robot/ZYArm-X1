from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description():
    # 模型和 RViz 预设都从 description 包内取，方便跨机器复用。
    package_share = Path(get_package_share_directory("zyarm_description"))
    default_model = package_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    default_rviz = package_share / "rviz" / "display_x1_standard.rviz"

    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Process the xacro with ros2_control tags enabled.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=str(default_rviz),
        description="Absolute path to the RViz config file.",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start joint_state_publisher_gui when true.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 when true.",
    )

    use_ros2_control = LaunchConfiguration("use_ros2_control")
    # 通过 xacro 参数决定是否把 ros2_control 相关标签展开进模型。
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(default_model),
                " ",
                "use_ros2_control:=",
                use_ros2_control,
            ]
        ),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="zyarm_x1_standard_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="zyarm_x1_standard_joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    jsp_headless = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="zyarm_x1_standard_joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="zyarm_x1_standard_rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # 只要 RViz 退出，就把整套显示链一起收掉，避免留下孤儿节点。
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="rviz2 exited"))],
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_ros2_control_arg,
        rviz_arg,
        gui_arg,
        use_rviz_arg,
        jsp,
        jsp_headless,
        rsp,
        rviz,
        shutdown_on_rviz_exit,
    ])
