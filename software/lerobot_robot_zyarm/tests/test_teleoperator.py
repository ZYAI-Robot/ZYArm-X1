from dataclasses import dataclass

from lerobot_robot_zyarm.config import ZyArmLeaderTeleoperatorConfig
from lerobot_robot_zyarm.features import ACTION_KEYS
from lerobot_robot_zyarm.teleoperator import ZyArmLeaderTeleoperator
from zyarm_sdk.types import StateSource


@dataclass(frozen=True)
class _Action:
    positions: tuple[float, ...]
    source: StateSource = StateSource.MASTER_DATA
    timestamp: float = 0.0
    sequence: int = 1
    raw_line: str = ""


class _Arm:
    def __init__(self):
        self.is_connected = False
        self.closed = False

    def connect(self):
        self.is_connected = True
        return self

    def close(self):
        self.is_connected = False
        self.closed = True


class _Leader:
    def __init__(self):
        self.arm = _Arm()
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def get_action(self, *, wait=False, timeout_ms=None):
        assert wait is True
        assert timeout_ms == 50.0
        return _Action((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4))


def test_teleoperator_lifecycle_and_action() -> None:
    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _Leader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    assert teleop.action_features == {key: float for key in ACTION_KEYS}
    assert teleop.feedback_features == {}

    teleop.connect()
    assert leader.started
    assert teleop.is_connected

    action = teleop.get_action()
    assert action["joint6.pos"] == 0.4

    teleop.send_feedback({})
    teleop.disconnect()
    assert leader.stopped
    assert leader.arm.closed
