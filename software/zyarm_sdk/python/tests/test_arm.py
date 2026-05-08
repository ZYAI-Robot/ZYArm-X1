import math
from typing import Optional

from zyarm_sdk import StateSource, ZyArm, ZyArmConfig
from zyarm_sdk.protocol import CommandId, MasterSlaveRole
from zyarm_sdk.transport import MemoryTransport


def make_arm() -> tuple[ZyArm, MemoryTransport]:
    config = ZyArmConfig(port="memory")
    transport = MemoryTransport(config)
    arm = ZyArm(config, transport=transport).connect()
    return arm, transport


def test_default_baudrate_matches_current_firmware_default() -> None:
    assert ZyArmConfig(port="memory").baudrate == 230_400


def test_fast_io_is_non_blocking_by_default() -> None:
    arm, transport = make_arm()
    result = arm.fast_io([0, 0, 0, 0, 0, 0, 0.5])
    assert result.accepted
    assert result.measured_snapshot is None
    assert transport.written_lines[-1].startswith("[CMD][36]")
    assert "-999.900" not in transport.written_lines[-1]


def test_query_state_and_latest_cache_sources() -> None:
    arm, transport = make_arm()
    before = transport.status_sequence
    transport.feed_line_for_test("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50")
    assert transport.status_sequence == before + 1

    latest = arm.get_latest_state()
    assert latest is not None
    assert latest.source == StateSource.CACHE
    assert latest.positions[-1] == 0.5

    transport.feed_line_for_test("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:25")
    queried = arm.query_state(timeout_ms=1)
    assert queried is None
    assert transport.written_lines[-1] == "[CMD][6]\n"


def test_fast_io_wait_state_labels_cmd36_snapshot() -> None:
    arm, transport = make_arm()

    def feed_after_write() -> None:
        transport.feed_line_for_test("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:10")

    original_send = transport.send_command

    def send_and_feed(command_id, params=None, *, wait_ack=False, timeout_s=None):
        ok = original_send(command_id, params, wait_ack=wait_ack, timeout_s=timeout_s)
        if int(command_id) == int(CommandId.JOINT_IO_FAST):
            feed_after_write()
        return ok

    transport.send_command = send_and_feed  # type: ignore[method-assign]
    result = arm.fast_io([0, 0, 0, 0, 0, 0, 0.1], wait_state=True, timeout_ms=10)
    assert result.measured_snapshot is not None
    assert result.measured_snapshot.source == StateSource.CMD36_MEASURED_SNAPSHOT
    assert math.isclose(result.measured_snapshot.positions[-1], 0.1)


def test_master_slave_lpf_command() -> None:
    arm, transport = make_arm()

    result = arm.set_master_slave_lpf(0.15)

    assert result.accepted
    assert transport.written_lines[-1] == "[CMD][34][0.150]\n"


def test_master_slave_mode_commands_use_semantic_roles() -> None:
    arm, transport = make_arm()

    result = arm.enter_master_mode(frequency_hz=60.0)
    assert result.accepted
    assert transport.written_lines[-1] == "[CMD][32][1 60]\n"

    result = arm.enter_slave_mode()
    assert result.accepted
    assert transport.written_lines[-1] == "[CMD][32][2 50]\n"

    result = arm.enter_master_slave_mode(MasterSlaveRole.SLAVE, frequency_hz=25.0)
    assert result.accepted
    assert transport.written_lines[-1] == "[CMD][32][2 25]\n"


def test_frame_stats_track_status_and_master_data_rx() -> None:
    arm, transport = make_arm()

    transport.feed_line_for_test("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50")
    transport.feed_line_for_test("[MD][0][0 -180 90 0 0 0 50]")
    transport.feed_line_for_test("[MD][2][0 -180 90 0 0 0 60]")

    stats = arm.get_frame_stats()
    assert stats.status_received == 1
    assert stats.status_rate_hz == 1.0
    assert stats.master_data_received == 2
    assert stats.master_data_gap_count == 1
    assert stats.master_data_rate_hz == 2.0

    arm.reset_frame_stats()
    reset_stats = arm.get_frame_stats()
    assert reset_stats.status_received == 0
    assert reset_stats.status_rate_hz == 0.0
    assert reset_stats.master_data_received == 0
    assert reset_stats.master_data_gap_count == 0
    assert reset_stats.master_data_rate_hz == 0.0


def test_ack_commands_use_semantic_timeouts() -> None:
    config = ZyArmConfig(
        port="memory",
        ack_timeout_s=0.1,
        action_timeout_s=2.5,
        play_record_timeout_s=181.0,
    )
    transport = MemoryTransport(config)
    arm = ZyArm(config, transport=transport).connect()
    seen: list[tuple[int, bool, Optional[float]]] = []
    original_send = transport.send_command

    def send_and_record(command_id, params=None, *, wait_ack=False, timeout_s=None):
        seen.append((int(command_id), bool(wait_ack), timeout_s))
        return original_send(command_id, params, wait_ack=wait_ack, timeout_s=timeout_s)

    transport.send_command = send_and_record  # type: ignore[method-assign]

    arm.reset()
    arm.move_ik(200, 0, 100)
    arm.set_gripper(1.0, sync=True)
    arm.play_record(1)
    arm.set_remote_mode(True)

    assert seen[0] == (int(CommandId.RESET), True, 2.5)
    assert seen[1] == (int(CommandId.IK_INVERSE), True, 2.5)
    assert seen[2] == (int(CommandId.SET_CLAW), True, 2.5)
    assert seen[3] == (int(CommandId.RECORD_PLAYER), True, 181.0)
    assert seen[4] == (int(CommandId.REMOTE_MODE), True, None)
