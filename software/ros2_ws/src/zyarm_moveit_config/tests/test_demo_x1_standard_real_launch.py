from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root from test path")


def _launch_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "launch"


def _real_launch_path() -> Path:
    return _launch_dir() / "demo_x1_standard_real.launch.py"


def _load_launch_module():
    spec = spec_from_file_location("zyarm_moveit_real_launch", _real_launch_path())
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_real_launch_runtime_parameters_disable_sim_time():
    module = _load_launch_module()

    assert module._common_runtime_parameters() == [{"use_sim_time": False}]


def test_real_launch_exposes_hardware_interface_arguments(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    module = _load_launch_module()
    declarations = module._real_hardware_launch_arguments()
    names = {declaration.name for declaration in declarations}

    assert {
        "serial_port",
        "baud_rate",
        "read_timeout_ms",
        "write_timeout_ms",
        "activation_status_timeout_ms",
        "status_stale_warn_ms",
        "status_stale_error_ms",
        "stale_log_period_ms",
        "reset_rts_dtr",
        "reset_rts_dtr_quiet_ms",
        "arm_hw_offsets_deg",
        "arm_hw_signs",
        "claw_travel_m",
        "claw_command_max",
    } <= names


def test_real_launch_uses_ros2_control_bringup_without_adapter():
    source = _real_launch_path().read_text(encoding="utf-8")

    assert "bringup_x1_standard_real_ros2_control.launch.py" in source
    assert "moveit_real_controllers.yaml" in source
    assert "use_real_hardware_interface:=true" in source
    assert "zyarm_control_adapter" not in source
    assert "arm_system.launch.py" not in source
    assert "joint_io_fast" not in source


def test_legacy_real_ros2_control_alias_launch_is_removed():
    assert not list(_launch_dir().glob("demo_*_real_ros2_control.launch.py"))


def test_moveit_readme_describes_single_real_moveit_entry():
    readme = (
        _find_repo_root() / "software/ros2_ws" / "src" / "zyarm_moveit_config" / "README.md"
    ).read_text(encoding="utf-8")

    assert "demo_x1_standard_real.launch.py" in readme
    assert "zyarm_hardware_interface" in readme
    assert "real_ros2_control.launch.py" not in readme
    assert "zyarm_control_adapter" not in readme
