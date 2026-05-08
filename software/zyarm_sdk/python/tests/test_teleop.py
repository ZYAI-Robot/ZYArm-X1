import time

from zyarm_sdk import ZyArm, ZyArmConfig
from zyarm_sdk.teleop import ZyArmTeleopPair
from zyarm_sdk.transport import MemoryTransport


def wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    assert predicate()


def make_arm(port: str) -> tuple[ZyArm, MemoryTransport]:
    config = ZyArmConfig(port=port)
    transport = MemoryTransport(config)
    arm = ZyArm(config, transport=transport).connect()
    return arm, transport


def test_teleop_step_uses_master_data_and_fast_io() -> None:
    leader, leader_transport = make_arm("leader")
    follower, follower_transport = make_arm("follower")
    pair = ZyArmTeleopPair(leader, follower)
    leader_transport.feed_line_for_test("[MD][4][0 -180 90 0 0 0 50]")
    result = pair.step()
    assert result is not None
    assert result.action.positions[-1] == 0.5
    assert follower_transport.written_lines[-1].startswith("[CMD][36]")


def test_auto_follow_starts_follower_slave_mode_and_relays_events() -> None:
    leader, leader_transport = make_arm("leader")
    follower, follower_transport = make_arm("follower")
    pair = ZyArmTeleopPair(leader, follower)
    leader_transport.feed_line_for_test("[MD][4][0 -180 90 0 0 0 50]")
    pair.start_auto_follow()
    assert follower_transport.written_lines[0] == "[CMD][34][0.150]\n"
    assert follower_transport.written_lines[1] == "[CMD][32][2 50]\n"
    assert leader_transport.written_lines[-1] == "[CMD][32][1 50]\n"

    leader_transport.feed_line_for_test("[MD][5][0 -180 90 0 0 0 60]")
    wait_until(lambda: len(follower_transport.written_lines) == 3)
    assert follower_transport.written_lines[-1].startswith("[CMD][36]")
    pair.stop()

    assert leader_transport.written_lines[-1] == "[CMD][33]\n"
    assert follower_transport.written_lines[-1] == "[CMD][33]\n"
