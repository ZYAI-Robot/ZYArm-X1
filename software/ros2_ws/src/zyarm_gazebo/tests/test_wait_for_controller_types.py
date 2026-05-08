from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


def _load_wait_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "wait_for_controller_types.py"
    )
    spec = spec_from_file_location("zyarm_gazebo_wait_for_controller_types", script_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_missing_controller_types_keeps_required_order():
    module = _load_wait_module()

    missing = module._missing_controller_types(
        [
            "joint_state_broadcaster/JointStateBroadcaster",
            "joint_trajectory_controller/JointTrajectoryController",
        ],
        ["joint_trajectory_controller/JointTrajectoryController"],
    )

    assert missing == ["joint_state_broadcaster/JointStateBroadcaster"]


def test_wait_for_required_controller_types_retries_until_all_types_available():
    module = _load_wait_module()
    attempts = iter(
        [
            [],
            ["joint_state_broadcaster/JointStateBroadcaster"],
            [
                "joint_state_broadcaster/JointStateBroadcaster",
                "joint_trajectory_controller/JointTrajectoryController",
            ],
        ]
    )
    sleeps = []
    now = {"value": 0.0}

    def fetch_available_types():
        return next(attempts)

    def fake_sleep(seconds):
        sleeps.append(seconds)
        now["value"] += seconds

    def fake_monotonic():
        return now["value"]

    module.wait_for_required_controller_types(
        fetch_available_types=fetch_available_types,
        required_types=[
            "joint_state_broadcaster/JointStateBroadcaster",
            "joint_trajectory_controller/JointTrajectoryController",
        ],
        timeout_seconds=5.0,
        sleep_interval_seconds=0.5,
        monotonic=fake_monotonic,
        sleep=fake_sleep,
    )

    assert sleeps == [0.5, 0.5]


def test_wait_for_required_controller_types_raises_timeout_with_missing_types():
    module = _load_wait_module()
    now = {"value": 0.0}

    def fetch_available_types():
        return []

    def fake_sleep(seconds):
        now["value"] += seconds

    def fake_monotonic():
        return now["value"]

    with pytest.raises(TimeoutError) as excinfo:
        module.wait_for_required_controller_types(
            fetch_available_types=fetch_available_types,
            required_types=["joint_state_broadcaster/JointStateBroadcaster"],
            timeout_seconds=1.0,
            sleep_interval_seconds=0.5,
            monotonic=fake_monotonic,
            sleep=fake_sleep,
        )

    assert "joint_state_broadcaster/JointStateBroadcaster" in str(excinfo.value)


def test_parse_args_ignores_launch_ros_extra_arguments():
    module = _load_wait_module()

    args = module._parse_args(
        [
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
            "--required-controller",
            "joint_state_broadcaster",
            "--ros-args",
            "-r",
            "__node:=zyarm_x1_standard_controller_type_waiter",
        ]
    )

    assert args.controller_manager == "/zyarm_x1_standard_controller_manager"
    assert args.required_controllers == ["joint_state_broadcaster"]


def test_parse_args_accepts_required_types_without_controller_names():
    module = _load_wait_module()

    args = module._parse_args(
        [
            "--controller-manager",
            "/zyarm_x1_standard_controller_manager",
            "--required-type",
            "joint_state_broadcaster/JointStateBroadcaster",
            "--required-type",
            "joint_trajectory_controller/JointTrajectoryController",
        ]
    )

    assert args.controller_manager == "/zyarm_x1_standard_controller_manager"
    assert args.required_types == [
        "joint_state_broadcaster/JointStateBroadcaster",
        "joint_trajectory_controller/JointTrajectoryController",
    ]
    assert args.required_controllers is None


def test_wait_for_required_controller_names_retries_until_all_names_available():
    module = _load_wait_module()
    attempts = iter(
        [
            [],
            ["arm_controller"],
            ["joint_state_broadcaster", "arm_controller", "gripper_controller"],
        ]
    )
    sleeps = []
    now = {"value": 0.0}

    def fetch_available_controllers():
        return next(attempts)

    def fake_sleep(seconds):
        sleeps.append(seconds)
        now["value"] += seconds

    def fake_monotonic():
        return now["value"]

    module.wait_for_required_controller_names(
        fetch_available_controllers=fetch_available_controllers,
        required_controllers=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
        ],
        timeout_seconds=5.0,
        sleep_interval_seconds=0.5,
        monotonic=fake_monotonic,
        sleep=fake_sleep,
    )

    assert sleeps == [0.5, 0.5]
