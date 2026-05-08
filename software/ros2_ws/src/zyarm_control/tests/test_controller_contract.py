from pathlib import Path
import yaml


def _repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root")


def _load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_controller_contract_defines_default_and_gazebo_gripper_variants():
    root = _repo_root()
    contract = _load_yaml(
        root / "software/ros2_ws" / "src" / "zyarm_control" / "config" / "controller_contract.yaml"
    )

    arm = contract["controllers"]["arm_controller"]
    gripper = contract["controllers"]["gripper_controller"]

    assert arm["joints"]["default"] == ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
    assert arm["joints"]["gazebo"] == ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
    assert arm["joints"]["real_ros2_control"] == [
        "joint0",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
    ]
    assert gripper["joints"]["default"] == ["joint6"]
    assert gripper["joints"]["gazebo"] == ["joint6", "joint7"]
    assert gripper["joints"]["real_ros2_control"] == ["joint6"]
    assert arm["action_type"] == "FollowJointTrajectory"
    assert gripper["action_type"] == "FollowJointTrajectory"
    assert contract["controller_manager"]["real_update_rate"] == 50
    assert contract["real_hardware_standard_control"] == {
        "backend": "zyarm_hardware_interface",
        "moveit_launch": "demo_x1_standard_real.launch.py",
        "bringup_launch": "bringup_x1_standard_real_ros2_control.launch.py",
    }
    assert arm["real_state_interfaces"] == ["position"]
    assert gripper["real_state_interfaces"] == ["position"]


def test_ros2_control_controller_files_match_contract_projections():
    root = _repo_root()
    control_dir = root / "software/ros2_ws" / "src" / "zyarm_control" / "config"
    contract = _load_yaml(control_dir / "controller_contract.yaml")
    default_config = _load_yaml(control_dir / "zyarm_x1_standard_controllers.yaml")
    gazebo_config = _load_yaml(control_dir / "zyarm_x1_standard_gazebo_controllers.yaml")
    real_config = _load_yaml(control_dir / "zyarm_x1_standard_real_controllers.yaml")

    arm_contract = contract["controllers"]["arm_controller"]
    gripper_contract = contract["controllers"]["gripper_controller"]

    assert (
        default_config["arm_controller"]["ros__parameters"]["joints"]
        == arm_contract["joints"]["default"]
    )
    assert (
        default_config["gripper_controller"]["ros__parameters"]["joints"]
        == gripper_contract["joints"]["default"]
    )
    assert (
        gazebo_config["arm_controller"]["ros__parameters"]["joints"]
        == arm_contract["joints"]["gazebo"]
    )
    assert (
        gazebo_config["gripper_controller"]["ros__parameters"]["joints"]
        == gripper_contract["joints"]["gazebo"]
    )
    assert (
        real_config["arm_controller"]["ros__parameters"]["joints"]
        == arm_contract["joints"]["real_ros2_control"]
    )
    assert (
        real_config["gripper_controller"]["ros__parameters"]["joints"]
        == gripper_contract["joints"]["real_ros2_control"]
    )
    assert (
        default_config["zyarm_x1_standard_controller_manager"]["ros__parameters"]["arm_controller"]["type"]
        == arm_contract["type"]
    )
    assert (
        gazebo_config["zyarm_x1_standard_controller_manager"]["ros__parameters"]["gripper_controller"]["type"]
        == gripper_contract["type"]
    )
    assert (
        real_config["zyarm_x1_standard_controller_manager"]["ros__parameters"]["update_rate"]
        == contract["controller_manager"]["real_update_rate"]
    )
    assert real_config["arm_controller"]["ros__parameters"]["command_interfaces"] == ["position"]
    assert real_config["arm_controller"]["ros__parameters"]["state_interfaces"] == ["position"]
    assert real_config["gripper_controller"]["ros__parameters"]["command_interfaces"] == ["position"]
    assert real_config["gripper_controller"]["ros__parameters"]["state_interfaces"] == ["position"]


def test_moveit_controller_mappings_match_contract():
    root = _repo_root()
    contract = _load_yaml(
        root / "software/ros2_ws" / "src" / "zyarm_control" / "config" / "controller_contract.yaml"
    )
    moveit_dir = root / "software/ros2_ws" / "src" / "zyarm_moveit_config" / "config"
    moveit_default = _load_yaml(moveit_dir / "moveit_controllers.yaml")
    moveit_gazebo = _load_yaml(moveit_dir / "moveit_gazebo_controllers.yaml")
    moveit_real = _load_yaml(moveit_dir / "moveit_real_controllers.yaml")

    arm_default = contract["controllers"]["arm_controller"]["joints"]["default"]
    gripper_default = contract["controllers"]["gripper_controller"]["joints"]["default"]
    gripper_gazebo = contract["controllers"]["gripper_controller"]["joints"]["gazebo"]

    assert moveit_default["moveit_simple_controller_manager"]["arm_controller"]["joints"] == arm_default
    assert (
        moveit_default["moveit_simple_controller_manager"]["gripper_controller"]["joints"]
        == gripper_default
    )
    assert moveit_real["moveit_simple_controller_manager"]["arm_controller"]["joints"] == arm_default
    assert moveit_real["moveit_simple_controller_manager"]["gripper_controller"]["joints"] == gripper_default
    assert (
        moveit_gazebo["moveit_simple_controller_manager"]["gripper_controller"]["joints"]
        == gripper_gazebo
    )
