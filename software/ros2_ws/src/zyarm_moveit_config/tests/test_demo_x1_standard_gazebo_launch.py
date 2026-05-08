from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import yaml


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root from test path")


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "demo_x1_standard_gazebo.launch.py"
    )
    spec = spec_from_file_location("zyarm_moveit_gazebo_launch", launch_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_load_world_robot_base_pose_from_gazebo_config():
    module = _load_launch_module()

    pose = module._load_world_robot_base_pose(
        Path(__file__).resolve().parents[2] / "zyarm_gazebo" / "config" / "world.yaml"
    )

    assert pose == [0.12, 0.0, 0.5038, 0.0, 0.0, 0.0]


def test_build_world_tf_arguments_use_robot_base_pose():
    module = _load_launch_module()

    arguments = module._build_world_tf_arguments([0.12, 0.0, 0.5038, 0.0, 0.0, 0.0])

    assert arguments == [
        "--x",
        "0.12",
        "--y",
        "0.0",
        "--z",
        "0.5038",
        "--roll",
        "0.0",
        "--pitch",
        "0.0",
        "--yaw",
        "0.0",
        "--frame-id",
        "world",
        "--child-frame-id",
        "base_link",
    ]


def test_moveit_readme_mentions_gazebo_validation_launch():
    readme = (
        _find_repo_root() / "software/ros2_ws" / "src" / "zyarm_moveit_config" / "README.md"
    ).read_text(encoding="utf-8")

    assert "demo_x1_standard_gazebo.launch.py" in readme
    assert "Gazebo" in readme


def test_workspace_readme_lists_gazebo_moveit_command():
    readme = (_find_repo_root() / "software/ros2_ws" / "README.md").read_text(encoding="utf-8")

    assert "ros2 launch zyarm_moveit_config demo_x1_standard_gazebo.launch.py" in readme


def test_common_runtime_parameters_enable_sim_time():
    module = _load_launch_module()

    assert module._common_runtime_parameters() == [{"use_sim_time": True}]


def test_gazebo_moveit_configs_use_dual_joint_gripper():
    repo_root = _find_repo_root()
    srdf_path = repo_root / "software/ros2_ws" / "src" / "zyarm_moveit_config" / "srdf" / "zyarm_x1_standard_gazebo.srdf"
    controllers_path = (
        repo_root
        / "software/ros2_ws"
        / "src"
        / "zyarm_moveit_config"
        / "config"
        / "moveit_gazebo_controllers.yaml"
    )

    srdf_text = srdf_path.read_text(encoding="utf-8")
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))

    assert '<joint name="joint6"/>' in srdf_text
    assert '<joint name="joint7"/>' in srdf_text
    assert '<joint name="joint7" value="0.034"/>' in srdf_text
    assert '<joint name="joint7" value="0.0"/>' in srdf_text
    assert controllers["moveit_simple_controller_manager"]["gripper_controller"]["joints"] == [
        "joint6",
        "joint7",
    ]
