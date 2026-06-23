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


def _launch_path(name: str) -> Path:
    return _launch_dir() / name


def _load_launch_module(name: str):
    spec = spec_from_file_location(name.replace(".", "_"), _launch_path(name))
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_x1_plus_mock_moveit_launch_uses_x1_plus_resources():
    source = _launch_path("demo_x1_plus.launch.py").read_text(encoding="utf-8")

    assert 'description_share / "urdf" / "x1_plus" / "robot.urdf.xacro"' in source
    assert "zyarm_x1_plus.srdf" in source
    assert "moveit_x1_plus.rviz" in source
    assert "moveit_controllers.yaml" in source
    assert "bringup_x1_plus_ros2_control.launch.py" in source
    assert "use_real_hardware_interface:=true" not in source
    assert "gazebo" not in source.lower()


def test_x1_plus_real_moveit_launch_uses_real_bringup_without_adapter():
    source = _launch_path("demo_x1_plus_real.launch.py").read_text(encoding="utf-8")

    assert 'description_share / "urdf" / "x1_plus" / "robot.urdf.xacro"' in source
    assert "zyarm_x1_plus.srdf" in source
    assert "moveit_x1_plus.rviz" in source
    assert "moveit_real_controllers.yaml" in source
    assert "bringup_x1_plus_real_ros2_control.launch.py" in source
    assert "use_real_hardware_interface:=true" in source
    assert "use_gazebo:=false" not in source
    assert "zyarm_control_adapter" not in source
    assert "arm_system.launch.py" not in source
    assert "joint_io_fast" not in source


def test_x1_plus_real_moveit_launch_exposes_hardware_arguments(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    module = _load_launch_module("demo_x1_plus_real.launch.py")
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
    assert module._common_runtime_parameters() == [{"use_sim_time": False}]


def test_x1_plus_moveit_readme_and_resources_do_not_claim_gazebo_support():
    repo_root = _find_repo_root()
    moveit_dir = repo_root / "software/ros2_ws" / "src" / "zyarm_moveit_config"
    readme = (moveit_dir / "README.md").read_text(encoding="utf-8")

    assert "demo_x1_plus.launch.py" in readme
    assert "demo_x1_plus_real.launch.py" in readme
    assert "x1_plus` 首版暂不提供 Gazebo" in readme
    assert not (moveit_dir / "launch" / "demo_x1_plus_gazebo.launch.py").exists()
    assert not (moveit_dir / "srdf" / "zyarm_x1_plus_gazebo.srdf").exists()
