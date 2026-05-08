from zyarm_hardware.config import RelationConfig
from zyarm_hardware.device import CommandResult
from zyarm_hardware.relations import ArmRelationCoordinator


class FakeLogger:
    def __init__(self) -> None:
        self.warnings = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeArmDevice:
    def __init__(self, name: str) -> None:
        self.name = name
        self.is_connected = True
        self.enter_calls = []
        self.stop_calls = 0
        self.dispatch_calls = []
        self.authoritative = False
        self.published_master_positions = []
        self._listener = None

    def add_master_data_listener(self, listener):
        self._listener = listener

    def enter_master_slave(self, role: int, frequency_hz: float) -> CommandResult:
        self.enter_calls.append((role, frequency_hz))
        return CommandResult(True, "ok")

    def stop_master_slave(self) -> CommandResult:
        self.stop_calls += 1
        return CommandResult(True, "ok")

    def set_master_data_authoritative(self, enabled: bool) -> None:
        self.authoritative = enabled

    def publish_master_data_joint_state(self, positions):
        self.published_master_positions.append(list(positions))

    def dispatch_joint_target(self, positions, apply_mask) -> CommandResult:
        raise AssertionError("dispatch_joint_target should not be used in fast_io teleop mode")

    def dispatch_joint_io_fast(self, positions, apply_mask) -> CommandResult:
        self.dispatch_calls.append((list(positions), list(apply_mask)))
        return CommandResult(True, "ok")


def test_relation_enable_enters_roles_and_relays_master_data():
    logger = FakeLogger()
    leader = FakeArmDevice("leader")
    follower = FakeArmDevice("follower")
    published_commands = []
    relation = ArmRelationCoordinator(
        RelationConfig(
            name="teleop_pair",
            mode="teleop_follow",
            leader="leader",
            follower="follower",
            follow_frequency_hz=50.0,
        ),
        leader,
        follower,
        logger,
        publish_follower_command=published_commands.append,
    )

    result = relation.set_enabled(True)
    relation._on_leader_master_data([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02])

    assert result.accepted is True
    assert leader.enter_calls == [(1, 50.0)]
    assert follower.enter_calls == []
    assert leader.authoritative is True
    assert leader.published_master_positions == [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02]]
    assert follower.dispatch_calls == [
        ([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02], [True, True, True, True, True, True, True])
    ]
    assert published_commands == [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02]]


def test_relation_disable_stops_both_devices():
    logger = FakeLogger()
    leader = FakeArmDevice("leader")
    follower = FakeArmDevice("follower")
    relation = ArmRelationCoordinator(
        RelationConfig(
            name="teleop_pair",
            mode="teleop_follow",
            leader="leader",
            follower="follower",
            follow_frequency_hz=50.0,
        ),
        leader,
        follower,
        logger,
        publish_follower_command=lambda _positions: None,
    )
    relation.set_enabled(True)

    result = relation.set_enabled(False)

    assert result.accepted is True
    assert leader.authoritative is False
    assert leader.stop_calls == 1
    assert follower.stop_calls == 0
