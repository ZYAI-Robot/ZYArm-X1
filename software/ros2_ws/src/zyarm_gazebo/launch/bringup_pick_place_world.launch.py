import atexit
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from string import Template

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LEFT_CONTACT_TOPIC = (
    "/world/pick_place_world/model/zyarm_x1_standard/link/claw1/sensor/claw1_touch/contact"
)
_RIGHT_CONTACT_TOPIC = (
    "/world/pick_place_world/model/zyarm_x1_standard/link/claw2/sensor/claw2_touch/contact"
)
_GRASP_ATTACH_TOPIC = "/zyarm/grasp/attach"
_GRASP_DETACH_TOPIC = "/zyarm/grasp/detach"
_GRASP_STATE_TOPIC = "/zyarm/grasp/state"
_DETACHABLE_CHILD_MODEL = "pickup_cube"
_DETACHABLE_CHILD_LINK = "cube_link"
_CLAW_CONTACT_FRICTION = "5.0"


def _format_values(values):
    return " ".join(str(value) for value in values)


def _render_template(template_path, destination_path, substitutions):
    template = Template(template_path.read_text(encoding="utf-8"))
    destination_path.write_text(template.safe_substitute(substitutions), encoding="utf-8")


def _load_world_config(share_dir):
    config_path = share_dir / "config" / "world.yaml"
    with config_path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _prepare_rendered_assets(share_dir):
    config = _load_world_config(share_dir)
    render_root = Path(tempfile.mkdtemp(prefix="zyarm_gazebo_"))
    atexit.register(lambda: shutil.rmtree(render_root, ignore_errors=True))

    substitutions = {
        "TABLE_SIZE": _format_values(config["table"]["size"]),
        "TABLE_POSE": _format_values(config["table"]["pose"]),
        "CUBE_SIZE": _format_values(config["cube"]["size"]),
        "CUBE_POSE": _format_values(config["cube"]["pose"]),
        "CUBE_MASS": str(config["cube"]["mass"]),
        "GOAL_ZONE_SIZE": _format_values(config["goal_zone"]["size"]),
        "GOAL_ZONE_POSE": _format_values(config["goal_zone"]["pose"]),
        "ROBOT_MODEL_NAME": "zyarm_x1_standard",
        "ROBOT_BASE_POSE": _format_values(config["robot_base_pose"]),
    }

    world_render_path = render_root / "pick_place_world.sdf"
    _render_template(share_dir / "worlds" / "pick_place_world.sdf", world_render_path, substitutions)

    models_root = render_root / "models"
    for model_name in ("pickup_cube", "goal_zone"):
        source_model_dir = share_dir / "models" / model_name
        destination_model_dir = models_root / model_name
        destination_model_dir.mkdir(parents=True, exist_ok=True)
        _render_template(
            source_model_dir / "model.sdf",
            destination_model_dir / "model.sdf",
            substitutions,
        )
        shutil.copyfile(source_model_dir / "model.config", destination_model_dir / "model.config")

    return render_root, world_render_path, config


def _build_gz_resource_path(*paths, existing_resource_path=None):
    ordered_paths = [str(path) for path in paths if path]
    if existing_resource_path:
        ordered_paths.append(existing_resource_path)
    return os.pathsep.join(ordered_paths)


def _build_spawn_arguments(model_file, robot_base_pose):
    x, y, z, roll, pitch, yaw = robot_base_pose
    return [
        "-file",
        str(model_file),
        "-x",
        str(x),
        "-y",
        str(y),
        "-z",
        str(z),
        "-R",
        str(roll),
        "-P",
        str(pitch),
        "-Y",
        str(yaw),
    ]


def _build_bridge_arguments():
    return [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera_fixed/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera_wrist/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        f"{_LEFT_CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        f"{_RIGHT_CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        f"{_GRASP_ATTACH_TOPIC}@std_msgs/msg/Empty]gz.msgs.Empty",
        f"{_GRASP_DETACH_TOPIC}@std_msgs/msg/Empty]gz.msgs.Empty",
        f"{_GRASP_STATE_TOPIC}@std_msgs/msg/String[gz.msgs.StringMsg",
    ]


def _build_bridge_parameters():
    return [
        {
            f"qos_overrides.{_GRASP_STATE_TOPIC}.publisher.durability": "transient_local",
        }
    ]


def _build_detachable_joint_attach_allowed_targets():
    return [f"{_DETACHABLE_CHILD_MODEL}::{_DETACHABLE_CHILD_LINK}"]


def _run_command(command):
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout


def _model_config_contents():
    return """<?xml version="1.0"?>
<model>
  <name>zyarm_x1_standard</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <description>ZYArm-X1 standard Gazebo simulation model rendered from xacro.</description>
</model>
"""


def _inject_dual_touch_features(sdf_text):
    root = ET.fromstring(sdf_text)
    model = root.find("./model")
    if model is None:
        raise ValueError("Rendered robot SDF does not contain a top-level model element")

    for link_name, sensor_name in (("claw1", "claw1_touch"), ("claw2", "claw2_touch")):
        link = model.find(f"./link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"Rendered robot SDF does not contain required link {link_name}")
        collision = link.find("./collision")
        if collision is None:
            raise ValueError(f"Rendered robot SDF does not contain a collision for {link_name}")
        collision_name = collision.attrib.get("name")
        if not collision_name:
            raise ValueError(f"Rendered robot SDF collision for {link_name} does not have a name")

        surface = collision.find("surface")
        if surface is None:
            surface = ET.SubElement(collision, "surface")
        friction = surface.find("friction")
        if friction is None:
            friction = ET.SubElement(surface, "friction")
        ode = friction.find("ode")
        if ode is None:
            ode = ET.SubElement(friction, "ode")
        mu = ode.find("mu")
        if mu is None:
            mu = ET.SubElement(ode, "mu")
        mu.text = _CLAW_CONTACT_FRICTION
        mu2 = ode.find("mu2")
        if mu2 is None:
            mu2 = ET.SubElement(ode, "mu2")
        mu2.text = _CLAW_CONTACT_FRICTION

        sensor = ET.SubElement(link, "sensor", {"name": sensor_name, "type": "contact"})
        ET.SubElement(sensor, "always_on").text = "1"
        ET.SubElement(sensor, "update_rate").text = "200"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = collision_name

    _inject_detachable_joint_plugin(model)
    return ET.tostring(root, encoding="unicode")


def _find_detachable_joint_parent_link(model):
    for link_name in ("ee_link", "link6"):
        link = model.find(f"./link[@name='{link_name}']")
        if link is not None:
            return link_name
    raise ValueError(
        "Rendered robot SDF does not contain required link ee_link or link6"
    )


def _inject_detachable_joint_plugin(model):
    parent_link_name = _find_detachable_joint_parent_link(model)

    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-detachable-joint-system",
            "name": "gz::sim::systems::DetachableJoint",
        },
    )
    ET.SubElement(plugin, "parent_link").text = parent_link_name
    ET.SubElement(plugin, "child_model").text = _DETACHABLE_CHILD_MODEL
    ET.SubElement(plugin, "child_link").text = _DETACHABLE_CHILD_LINK
    ET.SubElement(plugin, "attach_topic").text = _GRASP_ATTACH_TOPIC
    ET.SubElement(plugin, "detach_topic").text = _GRASP_DETACH_TOPIC
    ET.SubElement(plugin, "output_topic").text = _GRASP_STATE_TOPIC


def _render_robot_model(render_root, description_share, control_share):
    model_dir = render_root / "models" / "zyarm_x1_standard"
    model_dir.mkdir(parents=True, exist_ok=True)
    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    camera_config_file = description_share / "config" / "x1_standard" / "gazebo_camera_frames.yaml"
    gazebo_controllers_file = control_share / "config" / "zyarm_x1_standard_gazebo_controllers.yaml"
    urdf_text = _run_command(
        [
            "xacro",
            str(xacro_file),
            "use_ros2_control:=true",
            "use_gazebo:=true",
            "use_sim_cameras:=true",
            f"gazebo_camera_config_file:={camera_config_file}",
            f"gazebo_controller_config_file:={gazebo_controllers_file}",
        ]
    )
    urdf_path = render_root / "zyarm_x1_standard.urdf"
    urdf_path.write_text(urdf_text, encoding="utf-8")
    sdf_text = _run_command(["gz", "sdf", "-p", str(urdf_path)])
    sdf_text = _inject_dual_touch_features(sdf_text)
    model_sdf_path = model_dir / "model.sdf"
    model_sdf_path.write_text(sdf_text, encoding="utf-8")
    (model_dir / "model.config").write_text(_model_config_contents(), encoding="utf-8")
    return model_sdf_path


def _build_grasp_manager_node():
    return Node(
        package="zyarm_gazebo",
        executable="grasp_manager.py",
        name="zyarm_grasp_manager",
        output="screen",
        parameters=_build_grasp_manager_parameters(),
    )


def _build_grasp_manager_parameters():
    return [
        {
            "left_contact_topic": _LEFT_CONTACT_TOPIC,
            "right_contact_topic": _RIGHT_CONTACT_TOPIC,
            "grasp_state_topic": _GRASP_STATE_TOPIC,
            "grasp_mode": "attach",
            "attach_allowed_targets": _build_detachable_joint_attach_allowed_targets(),
            "attach_contact_debounce_sec": 0.03,
            "detach_open_threshold": 0.028,
            "detach_open_debounce_sec": 0.05,
            "contact_debounce_sec": 0.05,
            "single_touch_max_total_width": 0.05,
            "hold_preload": 0.001,
            "joint6_name": "joint6",
            "joint7_name": "joint7",
        }
    ]


def generate_launch_description():
    share_dir = Path(get_package_share_directory("zyarm_gazebo"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    description_share = Path(get_package_share_directory("zyarm_description"))
    control_share = Path(get_package_share_directory("zyarm_control"))
    render_root, rendered_world_path, config = _prepare_rendered_assets(share_dir)
    _render_robot_model(render_root, description_share, control_share)
    resource_path = _build_gz_resource_path(
        render_root / "models",
        description_share.parent,
        existing_resource_path=os.environ.get("GZ_SIM_RESOURCE_PATH"),
    )

    xacro_file = description_share / "urdf" / "x1_standard" / "robot.urdf.xacro"
    camera_config_file = description_share / "config" / "x1_standard" / "gazebo_camera_frames.yaml"
    gazebo_controllers_file = control_share / "config" / "zyarm_x1_standard_gazebo_controllers.yaml"
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

    controller_type_waiter = Node(
        package="zyarm_gazebo",
        executable="wait_for_controller_types.py",
        name="zyarm_x1_standard_controller_type_waiter",
        output="screen",
        arguments=[
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
            "--required-type",
            "joint_state_broadcaster/JointStateBroadcaster",
            "--required-type",
            "joint_trajectory_controller/JointTrajectoryController",
            "--timeout-seconds",
            "30.0",
            "--sleep-interval-seconds",
            "0.25",
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

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zyarm_x1_standard_camera_bridge",
        output="screen",
        arguments=_build_bridge_arguments(),
        parameters=_build_bridge_parameters(),
    )
    grasp_manager = _build_grasp_manager_node()

    delay_jsb_until_controller_types_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_type_waiter,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_controllers_until_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r {rendered_world_path}"}.items(),
            ),
            robot_state_publisher,
            controller_type_waiter,
            delay_jsb_until_controller_types_ready,
            delay_controllers_until_jsb,
            grasp_manager,
            camera_bridge,
        ]
    )
