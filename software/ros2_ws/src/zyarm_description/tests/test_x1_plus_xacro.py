import xml.etree.ElementTree as ET

from test_x1_standard_xacro_gazebo import _find_repo_root, _run_xacro


def _x1_plus_xacro_file():
    return (
        _find_repo_root()
        / "software/ros2_ws"
        / "src"
        / "zyarm_description"
        / "urdf"
        / "x1_plus"
        / "robot.urdf.xacro"
    )


def test_x1_plus_xacro_declares_plus_identity_and_model_data():
    source = _x1_plus_xacro_file().read_text(encoding="utf-8")

    assert 'name="zyarm_x1_plus"' in source
    assert "package://zyarm_description/meshes/x1_plus/link4.STL" in source
    assert "package://ZYARM_X1_PLUS_URDF" not in source
    assert "use_gazebo" not in source
    assert "<gazebo" not in source

    xml = _run_xacro(_x1_plus_xacro_file(), "use_ros2_control:=false")
    root = ET.fromstring(xml)

    assert root.attrib["name"] == "zyarm_x1_plus"
    link4_inertial = root.find("./link[@name='link4']/inertial")
    assert link4_inertial is not None
    assert link4_inertial.find("origin").attrib["xyz"] == "0.0024199 -3.8275E-05 0.079447"
    assert link4_inertial.find("mass").attrib["value"] == "0.14836"

    joint3 = root.find("./joint[@name='joint3']")
    assert joint3 is not None
    assert joint3.find("origin").attrib["xyz"] == "0.039913 0.23 0"


def test_x1_plus_xacro_declares_real_hardware_interface_position_only():
    xml = _run_xacro(
        _x1_plus_xacro_file(),
        "use_ros2_control:=true",
        "use_real_hardware_interface:=true",
        "real_hardware_port:=/tmp/ttyZYARM_PLUS",
        "real_hardware_baud_rate:=230400",
    )
    root = ET.fromstring(xml)

    hardware = root.find(".//ros2_control/hardware")
    assert hardware is not None
    plugin = hardware.find("plugin")
    assert plugin is not None
    assert plugin.text == "zyarm_hardware_interface/ZyArmSystemHardware"

    params = {param.attrib["name"]: param.text for param in hardware.findall("param")}
    assert params["port"] == "/tmp/ttyZYARM_PLUS"
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

    joint7 = root.find("./joint[@name='joint7']")
    assert joint7 is not None
    assert joint7.find("mimic").attrib["joint"] == "joint6"
    assert root.find(".//ros2_control/joint[@name='joint7']") is None
    assert "<gazebo" not in xml
    assert "joint_io_fast" not in xml


def test_x1_plus_xacro_mock_control_uses_generic_system_and_positive_limits():
    xml = _run_xacro(
        _x1_plus_xacro_file(),
        "use_ros2_control:=true",
        "use_real_hardware_interface:=false",
    )
    root = ET.fromstring(xml)

    hardware = root.find(".//ros2_control/hardware")
    assert hardware is not None
    assert hardware.find("plugin").text == "mock_components/GenericSystem"

    for joint_index in range(7):
        joint = root.find(f".//ros2_control/joint[@name='joint{joint_index}']")
        assert joint is not None
        assert [iface.attrib["name"] for iface in joint.findall("state_interface")] == [
            "position",
            "velocity",
        ]

    assert root.find(".//ros2_control/joint[@name='joint7']") is None

    for joint_name in ("joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["effort"]) > 0.0
        assert float(limit.attrib["velocity"]) > 0.0
