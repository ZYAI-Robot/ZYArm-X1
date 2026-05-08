from __future__ import annotations

from types import SimpleNamespace

import pytest
import rclpy
import yaml

from zyarm_hardware.device import CommandResult, JointIoFastResult
import zyarm_hardware.node as node_module


class FakeArmDevice:
    def __init__(self, config, logger, *, publish_joint_state) -> None:
        self.config = config
        self.logger = logger
        self.publish_joint_state = publish_joint_state
        self.is_connected = False
        self.connect_calls = 0
        self.close_calls = 0
        self.status_report_calls = []
        self.dispatch_calls = []
        self.fast_io_calls = []
        self.enter_calls = []
        self.stop_calls = 0
        self.authoritative = False
        self.master_data_listeners = []
        self.published_master_positions = []

    def connect(self) -> None:
        self.connect_calls += 1
        self.is_connected = True

    def close(self) -> None:
        self.close_calls += 1
        self.is_connected = False

    def add_master_data_listener(self, listener) -> None:
        self.master_data_listeners.append(listener)

    def dispatch_joint_target(self, positions, apply_mask) -> CommandResult:
        self.dispatch_calls.append((list(positions), list(apply_mask)))
        return CommandResult(True, f"{self.config.name} dispatched")

    def dispatch_joint_io_fast(self, positions, apply_mask) -> JointIoFastResult:
        self.fast_io_calls.append((list(positions), list(apply_mask)))
        return JointIoFastResult(
            True,
            f"{self.config.name} fast_io completed",
            list(positions),
            True,
        )

    def set_status_report(self, enabled: bool, frequency_hz: float) -> CommandResult:
        self.status_report_calls.append((bool(enabled), float(frequency_hz)))
        return CommandResult(True, f"{self.config.name} status report updated")

    def enter_master_slave(self, role: int, frequency_hz: float) -> CommandResult:
        self.enter_calls.append((int(role), float(frequency_hz)))
        return CommandResult(True, f"{self.config.name} entered role {role}")

    def stop_master_slave(self) -> CommandResult:
        self.stop_calls += 1
        return CommandResult(True, f"{self.config.name} stopped")

    def set_master_data_authoritative(self, enabled: bool) -> None:
        self.authoritative = bool(enabled)

    def publish_master_data_joint_state(self, positions) -> None:
        self.published_master_positions.append(list(positions))


@pytest.fixture
def ros_context():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _write_config(tmp_path, payload: dict) -> str:
    config_path = tmp_path / "system.yaml"
    config_path.write_text(yaml.safe_dump(payload, sort_keys=True), encoding="utf-8")
    return str(config_path)


def _service_names(node) -> set[str]:
    return {service["name"] for service in node._managed_services}


def _topic_names(node) -> set[str]:
    return {name for name, _types in node.get_topic_names_and_types()}


def _make_response():
    return SimpleNamespace(accepted=False, message="")


def _patch_services(monkeypatch):
    def fake_create_service(self, srv_type, srv_name, callback, *args, **kwargs):
        return {"type": srv_type, "name": f"/{srv_name}", "callback": callback}

    monkeypatch.setattr(node_module.ArmSystemNode, "create_service", fake_create_service, raising=False)


def test_arm_system_node_smoke_starts_single_arm_instance(monkeypatch, tmp_path, ros_context):
    monkeypatch.setattr(node_module, "ArmDevice", FakeArmDevice)
    _patch_services(monkeypatch)
    config_file = _write_config(
        tmp_path,
        {
            "arms": {
                "slave": {
                    "port": "/dev/ttyUSB0",
                    "status_report_frequency_hz": 50.0,
                }
            },
            "relationships": {},
        },
    )

    node = node_module.ArmSystemNode(config_file=config_file)
    try:
        assert sorted(node._devices) == ["slave"]
        assert node._devices["slave"].connect_calls == 1
        assert sorted(node._relations) == []
        assert "/slave/joint_io_fast" in _service_names(node)
        assert "/slave/set_joint_target" in _service_names(node)
        assert "/slave/set_status_report" in _service_names(node)
        assert "/slave/joint_state" in _topic_names(node)
        assert "/slave/joint_command" not in _topic_names(node)
    finally:
        node.destroy_node()


def test_arm_system_node_smoke_starts_multiple_independent_instances(monkeypatch, tmp_path, ros_context):
    monkeypatch.setattr(node_module, "ArmDevice", FakeArmDevice)
    _patch_services(monkeypatch)
    config_file = _write_config(
        tmp_path,
        {
            "arms": {
                "left_master": {"port": "/dev/ttyUSB0"},
                "left_slave": {"port": "/dev/ttyUSB1"},
                "right_master": {"port": "/dev/ttyUSB2"},
                "right_slave": {"port": "/dev/ttyUSB3"},
            },
            "relationships": {},
        },
    )

    node = node_module.ArmSystemNode(config_file=config_file)
    try:
        assert sorted(node._devices) == ["left_master", "left_slave", "right_master", "right_slave"]
        assert all(device.connect_calls == 1 for device in node._devices.values())
        assert sorted(node._joint_command_publishers) == []
        service_names = _service_names(node)
        for arm_name in node._devices:
            assert f"/{arm_name}/joint_io_fast" in service_names
            assert f"/{arm_name}/set_joint_target" in service_names
            assert f"/{arm_name}/set_status_report" in service_names
    finally:
        node.destroy_node()


def test_arm_system_node_smoke_runs_two_pairs_and_relation_enable_disable(monkeypatch, tmp_path, ros_context):
    monkeypatch.setattr(node_module, "ArmDevice", FakeArmDevice)
    _patch_services(monkeypatch)
    config_file = _write_config(
        tmp_path,
        {
            "arms": {
                "left_master": {"port": "/dev/ttyUSB0"},
                "left_slave": {"port": "/dev/ttyUSB1"},
                "right_master": {"port": "/dev/ttyUSB2"},
                "right_slave": {"port": "/dev/ttyUSB3"},
            },
            "relationships": {
                "left_pair": {
                    "mode": "teleop_follow",
                    "leader": "left_master",
                    "follower": "left_slave",
                    "follow_frequency_hz": 50.0,
                },
                "right_pair": {
                    "mode": "teleop_follow",
                    "leader": "right_master",
                    "follower": "right_slave",
                    "follow_frequency_hz": 45.0,
                },
            },
        },
    )

    node = node_module.ArmSystemNode(config_file=config_file)
    try:
        assert sorted(node._relations) == ["left_pair", "right_pair"]
        assert sorted(node._joint_command_publishers) == ["left_slave", "right_slave"]
        service_names = _service_names(node)
        assert "/left_pair/set_enabled" in service_names
        assert "/right_pair/set_enabled" in service_names
        assert "/left_slave/joint_command" in _topic_names(node)
        assert "/right_slave/joint_command" in _topic_names(node)

        left_handler = node._make_relation_handler(node._relations["left_pair"])
        right_handler = node._make_relation_handler(node._relations["right_pair"])

        left_response = left_handler(SimpleNamespace(enabled=True), _make_response())
        right_response = right_handler(SimpleNamespace(enabled=True), _make_response())
        assert left_response.accepted is True
        assert right_response.accepted is True

        assert node._devices["left_master"].enter_calls == [(1, 50.0)]
        assert node._devices["left_slave"].enter_calls == []
        assert node._devices["right_master"].enter_calls == [(1, 45.0)]
        assert node._devices["right_slave"].enter_calls == []

        node._relations["left_pair"]._on_leader_master_data([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02])
        node._relations["right_pair"]._on_leader_master_data([0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.01])

        assert node._devices["left_slave"].fast_io_calls == [
            ([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02], [True, True, True, True, True, True, True])
        ]
        assert node._devices["right_slave"].fast_io_calls == [
            ([0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.01], [True, True, True, True, True, True, True])
        ]

        disable_response = left_handler(SimpleNamespace(enabled=False), _make_response())
        assert disable_response.accepted is True
        assert node._devices["left_master"].stop_calls == 1
        assert node._devices["left_slave"].stop_calls == 0
        assert node._devices["right_master"].stop_calls == 0
        assert node._devices["right_slave"].stop_calls == 0
    finally:
        node.destroy_node()
