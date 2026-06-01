from pathlib import Path


def _repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "software/ros2_ws" / "README.md").is_file():
            return parent
    raise FileNotFoundError("Could not locate repository root")



def test_quality_check_launch_uses_quality_mode_and_rviz_config():
    launch_path = (
        _repo_root()
        / "software"
        / "ros2_ws"
        / "src"
        / "zyarm_dataset_replay"
        / "launch"
        / "dataset_quality_check.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")

    assert "_quality_check_mode" in source
    assert "quality_check.rviz" in source
    assert "robot_state_publisher" in source
    assert "use_ros2_control:=false" in source
    # 确认不再依赖 bringup
    assert "bringup_2b_ros2_control.launch.py" not in source

