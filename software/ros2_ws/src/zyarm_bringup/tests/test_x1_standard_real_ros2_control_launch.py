from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root")


def _launch_path(model: str = "x1_standard") -> Path:
    return (
        _find_repo_root()
        / "software/ros2_ws"
        / "src"
        / "zyarm_bringup"
        / "launch"
        / f"bringup_{model}_real_ros2_control.launch.py"
    )


def _load_launch_module(model: str = "x1_standard"):
    spec = spec_from_file_location(f"zyarm_bringup_{model}_real_ros2_control_launch", _launch_path(model))
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_real_ros2_control_bringup_uses_real_controller_yaml():
    module = _load_launch_module()

    assert module._default_controllers_file().name == "zyarm_x1_standard_real_controllers.yaml"


def test_x1_plus_real_ros2_control_bringup_uses_real_controller_yaml():
    module = _load_launch_module("x1_plus")

    assert module._default_controllers_file().name == "zyarm_x1_plus_real_controllers.yaml"


def test_real_ros2_control_bringup_uses_real_hardware_xacro_args():
    module = _load_launch_module()
    args = module._real_hardware_xacro_arguments()
    rendered = "".join(str(item) for item in args)

    assert "use_real_hardware_interface:=true" in rendered
    assert "use_gazebo:=false" in rendered
    assert "real_hardware_status_stale_error_ms:=" in rendered
    assert "real_hardware_arm_hw_offsets_deg:=" in rendered


def test_x1_plus_real_ros2_control_bringup_uses_real_hardware_xacro_args():
    module = _load_launch_module("x1_plus")
    args = module._real_hardware_xacro_arguments()
    rendered = "".join(str(item) for item in args)

    assert "use_real_hardware_interface:=true" in rendered
    assert "use_gazebo:=false" not in rendered
    assert "real_hardware_status_stale_error_ms:=" in rendered
    assert "real_hardware_arm_hw_offsets_deg:=" in rendered
    assert "real_hardware_claw_travel_m:=" in rendered


def test_real_ros2_control_bringup_does_not_start_adapter():
    source = _launch_path().read_text(encoding="utf-8")

    assert "zyarm_x1_standard_real_controllers.yaml" in source
    assert "zyarm_control_adapter" not in source
    assert "joint_io_fast" not in source


def test_x1_plus_real_ros2_control_bringup_does_not_start_adapter():
    source = _launch_path("x1_plus").read_text(encoding="utf-8")

    assert "zyarm_x1_plus_real_controllers.yaml" in source
    assert "zyarm_control_adapter" not in source
    assert "arm_system.launch.py" not in source
    assert "joint_io_fast" not in source
