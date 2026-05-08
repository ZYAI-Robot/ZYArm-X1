import shlex
import shutil
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = (
            parent
            / "software/ros2_ws"
            / "src"
            / "zyarm_description"
            / "urdf"
            / "x1_standard"
            / "robot.urdf.xacro"
        )
        if candidate.is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root from test path")


def _is_missing_xacro_error(completed: subprocess.CompletedProcess[str]) -> bool:
    stderr = completed.stderr or ""
    return (
        "xacro: command not found" in stderr
        or "No module named xacro" in stderr
        or "can't open file" in stderr
        or "package 'zyarm_description' not found" in stderr
    )


def _run_xacro_command(command: list[str], setup_file: Path | None = None) -> subprocess.CompletedProcess[str]:
    if setup_file is None:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )

    return subprocess.run(
        [
            "bash",
            "-lc",
            f"source {shlex.quote(str(setup_file))} >/dev/null 2>&1 && {shlex.join(command)}",
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def _run_xacro(xacro_file: Path, *extra_args: str) -> str:
    candidate_commands: list[tuple[list[str], Path | None]] = []
    repo_root = _find_repo_root()
    workspace_setup = repo_root / "software/ros2_ws" / "install" / "setup.bash"

    if workspace_setup.is_file():
        candidate_commands.append((["xacro", str(xacro_file), *extra_args], workspace_setup))
        candidate_commands.append((["python3", "-m", "xacro", str(xacro_file), *extra_args], workspace_setup))

    xacro_executable = shutil.which("xacro")
    if xacro_executable is not None:
        candidate_commands.append(([xacro_executable, str(xacro_file), *extra_args], None))

    candidate_commands.append(([sys.executable, "-m", "xacro", str(xacro_file), *extra_args], None))

    for setup_file in sorted(Path("/opt/ros").glob("*/setup.bash")):
        candidate_commands.append((["xacro", str(xacro_file), *extra_args], setup_file))
        candidate_commands.append((["python3", "-m", "xacro", str(xacro_file), *extra_args], setup_file))

    for command, setup_file in candidate_commands:
        completed = _run_xacro_command(command, setup_file=setup_file)
        if completed.returncode == 0:
            return completed.stdout
        if not _is_missing_xacro_error(completed):
            completed.check_returncode()

    raise AssertionError(
        "xacro unavailable: no executable in PATH and no importable Python module in the current "
        "environment or any /opt/ros/*/setup.bash environment"
    )


def test_x1_standard_xacro_emits_gazebo_camera_and_ros2_control_tags():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )
    camera_config = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "config"
        / "x1_standard"
        / "gazebo_camera_frames.yaml"
    )
    controller_config = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_control"
        / "config"
        / "zyarm_x1_standard_gazebo_controllers.yaml"
    )
    camera_frames = yaml.safe_load(camera_config.read_text(encoding="utf-8"))["camera_frames"]
    fixed_link = camera_frames["fixed"]["link_frame"]
    fixed_optical = camera_frames["fixed"]["optical_frame"]
    fixed_topic = camera_frames["fixed"]["image_topic"]
    wrist_link = camera_frames["wrist"]["link_frame"]
    wrist_optical = camera_frames["wrist"]["optical_frame"]
    wrist_topic = camera_frames["wrist"]["image_topic"]

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=true",
        f"gazebo_camera_config_file:={camera_config}",
        f"gazebo_controller_config_file:={controller_config}",
    )
    xml_without_ros2_control = _run_xacro(
        xacro_file,
        "use_ros2_control:=false",
        "use_gazebo:=true",
        "use_sim_cameras:=true",
        f"gazebo_camera_config_file:={camera_config}",
        f"gazebo_controller_config_file:={controller_config}",
    )
    xml_without_sim_cameras = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
        f"gazebo_camera_config_file:={camera_config}",
        f"gazebo_controller_config_file:={controller_config}",
    )
    xml_without_gazebo = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=false",
        "use_sim_cameras:=true",
        f"gazebo_camera_config_file:={camera_config}",
        f"gazebo_controller_config_file:={controller_config}",
    )
    assert "ee_link" in xml
    assert fixed_link in xml
    assert fixed_optical in xml
    assert wrist_link in xml
    assert wrist_optical in xml
    assert fixed_topic in xml
    assert wrist_topic in xml
    assert "gazebo_ros2_control" in xml or "gz_ros2_control" in xml
    assert "gazebo_ros2_control" not in xml_without_ros2_control
    assert "gz_ros2_control" not in xml_without_ros2_control
    assert fixed_link not in xml_without_sim_cameras
    assert fixed_optical not in xml_without_sim_cameras
    assert wrist_link not in xml_without_sim_cameras
    assert wrist_optical not in xml_without_sim_cameras
    assert fixed_topic not in xml_without_sim_cameras
    assert wrist_topic not in xml_without_sim_cameras
    assert "<gazebo" not in xml_without_gazebo
    assert "gazebo_ros2_control" not in xml_without_gazebo
    assert "gz_ros2_control" not in xml_without_gazebo
    assert fixed_topic not in xml_without_gazebo
    assert wrist_topic not in xml_without_gazebo


def test_x1_standard_xacro_declares_gazebo_system_plugin_and_controller_manager_name():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    source = xacro_file.read_text(encoding="utf-8")

    assert "<plugin>gz_ros2_control/GazeboSimSystem</plugin>" in source
    assert "<controller_manager_name>zyarm_x1_standard_controller_manager</controller_manager_name>" in source


def test_x1_standard_xacro_declares_real_hardware_interface_position_only():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=false",
        "use_real_hardware_interface:=true",
        "real_hardware_port:=/tmp/ttyZYARM",
        "real_hardware_baud_rate:=230400",
    )
    root = ET.fromstring(xml)

    hardware = root.find(".//ros2_control/hardware")
    assert hardware is not None
    plugin = hardware.find("plugin")
    assert plugin is not None
    assert plugin.text == "zyarm_hardware_interface/ZyArmSystemHardware"

    params = {param.attrib["name"]: param.text for param in hardware.findall("param")}
    assert params["port"] == "/tmp/ttyZYARM"
    assert params["baud_rate"] == "230400"
    assert params["read_timeout_ms"] == "20"
    assert params["write_timeout_ms"] == "20"
    assert params["activation_status_timeout_ms"] == "1000"
    assert params["status_stale_warn_ms"] == "100"
    assert params["status_stale_error_ms"] == "1000"
    assert params["arm_hw_offsets_deg"] == "0 -180 90 0 0 0"
    assert params["arm_hw_signs"] == "1 1 1 1 1 1"
    assert params["claw_travel_m"] == "0.034"
    assert params["claw_command_max"] == "100"

    for joint_index in range(7):
        joint = root.find(f".//ros2_control/joint[@name='joint{joint_index}']")
        assert joint is not None
        assert [iface.attrib["name"] for iface in joint.findall("command_interface")] == ["position"]
        assert [iface.attrib["name"] for iface in joint.findall("state_interface")] == ["position"]

    assert root.find(".//ros2_control/joint[@name='joint7']") is None
    assert "joint_io_fast" not in xml


def test_x1_standard_xacro_does_not_embed_world_anchor_even_for_gazebo():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml_with_gazebo = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    xml_without_gazebo = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=false",
        "use_sim_cameras:=false",
    )

    assert '<link name="world"/>' not in xml_with_gazebo
    assert '<joint name="world_fixed_joint" type="fixed">' not in xml_with_gazebo
    assert '<link name="world"/>' not in xml_without_gazebo
    assert '<joint name="world_fixed_joint" type="fixed">' not in xml_without_gazebo


def test_x1_standard_xacro_uses_positive_motion_limits_for_simulated_joints():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    root = ET.fromstring(xml)

    for joint_name in ("joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["effort"]) > 0.0
        assert float(limit.attrib["velocity"]) > 0.0


def test_x1_standard_xacro_keeps_gazebo_zero_start_state_inside_arm_limits():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    root = ET.fromstring(xml)

    for joint_name in ("joint1", "joint2"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) < 0.0
        assert float(limit.attrib["upper"]) > 0.0


def test_x1_standard_xacro_uses_dual_finger_gripper_in_gazebo_without_mimic_constraint():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    root = ET.fromstring(xml)

    joint7 = root.find("./joint[@name='joint7']")
    assert joint7 is not None
    assert joint7.find("mimic") is None

    ros2_control_joint7 = root.find(".//ros2_control/joint[@name='joint7']")
    assert ros2_control_joint7 is not None
    assert ros2_control_joint7.find("./command_interface[@name='position']") is not None


def test_x1_standard_xacro_uses_higher_effort_limits_for_gazebo_tracking_under_contact():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    root = ET.fromstring(xml)

    for joint_name in ("joint0", "joint1", "joint2", "joint3", "joint4", "joint5"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["effort"]) >= 60.0

    for joint_name in ("joint6", "joint7"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["effort"]) >= 160.0


def test_x1_standard_xacro_uses_higher_velocity_limits_for_faster_sim_tracking():
    repo_root = _find_repo_root()
    xacro_file = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_standard"
        / "robot.urdf.xacro"
    )

    xml = _run_xacro(
        xacro_file,
        "use_ros2_control:=true",
        "use_gazebo:=true",
        "use_sim_cameras:=false",
    )
    root = ET.fromstring(xml)

    for joint_name in ("joint0", "joint1", "joint2"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["velocity"]) >= 3.0

    for joint_name in ("joint3", "joint4", "joint5"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["velocity"]) >= 4.0

    for joint_name in ("joint6", "joint7"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["velocity"]) >= 0.12
