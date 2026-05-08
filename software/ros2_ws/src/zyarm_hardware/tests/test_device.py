import math
from dataclasses import replace

from zyarm_hardware.config import ArmConfig, ArmKinematicsConfig
from zyarm_hardware.device import ArmDevice, NO_CHANGE_SENTINEL
from zyarm_hardware.protocol import CommandId, StatusFrame


class FakeLogger:
    def __init__(self) -> None:
        self.warnings = []

    def info(self, _message: str) -> None:
        pass

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeProtocol:
    def __init__(self, connected: bool = True) -> None:
        self.is_connected = connected
        self.commands = []
        self.connect_calls = 0
        self.send_command_result = True
        self.status_received_at = float("-inf")
        self.has_fresh_status = False
        self.waited_for_status_since = []
        self.status_frame = None

    def connect(self):
        self.connect_calls += 1
        self.is_connected = True

    def send_command(self, command_id, params=None, *, wait_ack=True):
        self.commands.append((command_id, list(params or []), wait_ack))
        return self.send_command_result

    def latest_status_received_at(self):
        return self.status_received_at

    def has_status_since(self, received_at):
        return self.has_fresh_status and received_at == self.status_received_at

    def wait_for_status_since(self, received_at, *, timeout_s=None):
        self.waited_for_status_since.append((received_at, timeout_s))
        return self.status_frame

    def close(self):
        pass


def _arm_config() -> ArmConfig:
    return ArmConfig(
        name="slave",
        port="/dev/ttyUSB0",
        baudrate=230400,
        timeout_s=0.1,
        write_timeout_s=0.1,
        ack_timeout_s=0.2,
        reset_rts_dtr=False,
        reset_rts_dtr_quiet_s=3.0,
        log_serial=False,
        log_dir="/tmp",
        startup_reset=False,
        status_report_frequency_hz=50.0,
        kinematics=ArmKinematicsConfig(
            arm_hw_offsets_deg=[0.0] * 6,
            arm_hw_signs=[1.0] * 6,
            claw_travel_m=0.04,
            claw_command_max=100.0,
        ),
    )


def test_dispatch_joint_target_uses_no_change_sentinel_for_masked_joints():
    published = []
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=published.append)
    device._protocol = FakeProtocol()

    result = device.dispatch_joint_target(
        [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02],
        [True, False, False, False, False, False, True],
    )

    assert result.accepted is True
    command_id, params, wait_ack = device._protocol.commands[0]
    assert command_id == CommandId.JOINT_SYNC
    assert wait_ack is False
    assert math.isclose(params[0], math.degrees(0.5), rel_tol=1e-6, abs_tol=1e-6)
    assert params[1:6] == [NO_CHANGE_SENTINEL] * 5
    assert math.isclose(params[6], 50.0, rel_tol=1e-6, abs_tol=1e-6)
    assert params[7:] == [0.0, 0.0, 1.0]


def test_set_status_report_sends_enable_flag_and_frequency():
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=lambda _positions: None)
    device._protocol = FakeProtocol()

    result = device.set_status_report(True, 25.0)

    assert result.accepted is True
    assert device._protocol.commands == [(CommandId.STATUS_REPORT, [1.0, 25.0], True)]


def test_connect_only_sends_startup_reset():
    config = replace(_arm_config(), startup_reset=True)
    device = ArmDevice(config, FakeLogger(), publish_joint_state=lambda _positions: None)
    device._protocol = FakeProtocol()

    device.connect()

    assert device._protocol.connect_calls == 1
    assert device._protocol.commands == [(CommandId.RESET, [], True)]


def test_set_status_report_accepts_fresh_status_frame_when_ack_times_out():
    logger = FakeLogger()
    device = ArmDevice(_arm_config(), logger, publish_joint_state=lambda _positions: None)
    device._protocol = FakeProtocol()
    device._protocol.send_command_result = False
    device._protocol.status_received_at = 12.0
    device._protocol.has_fresh_status = True

    result = device.set_status_report(True, 25.0)

    assert result.accepted is True
    assert logger.warnings


def test_set_status_report_still_fails_without_ack_or_fresh_status_frame():
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=lambda _positions: None)
    device._protocol = FakeProtocol()
    device._protocol.send_command_result = False
    device._protocol.status_received_at = 12.0
    device._protocol.has_fresh_status = False

    result = device.set_status_report(True, 25.0)

    assert result.accepted is False


def test_status_frame_publishes_joint_state_until_master_data_becomes_authoritative():
    published = []
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=published.append)

    device._on_status_frame(StatusFrame([10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0], 0.0, ""))
    device.set_master_data_authoritative(True)
    device._on_status_frame(StatusFrame([11.0, 21.0, 31.0, 41.0, 51.0, 61.0, 71.0], 0.0, ""))

    assert len(published) == 1
    assert len(published[0]) == 7


def test_dispatch_joint_io_fast_returns_latest_cached_measured_state_without_waiting():
    published = []
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=published.append)
    device._protocol = FakeProtocol()
    device._on_status_frame(StatusFrame([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], 4.0, ""))
    published.clear()

    result = device.dispatch_joint_io_fast([0.0] * 7, [False] * 7)

    assert result.accepted is True
    assert result.measured_valid is True
    assert len(result.measured_positions) == 7
    command_id, params, wait_ack = device._protocol.commands[0]
    assert command_id == CommandId.JOINT_IO_FAST
    assert params == [NO_CHANGE_SENTINEL] * 7
    assert wait_ack is False
    assert published == []


def test_dispatch_joint_io_fast_succeeds_even_without_cached_status():
    device = ArmDevice(_arm_config(), FakeLogger(), publish_joint_state=lambda _positions: None)
    device._protocol = FakeProtocol()

    result = device.dispatch_joint_io_fast([0.0] * 7, [False] * 7)

    assert result.accepted is True
    assert result.measured_valid is False
