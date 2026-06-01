from dataclasses import dataclass
from time import perf_counter

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

    @property
    def age_ms(self):
        return max(0.0, (perf_counter() - self.timestamp) * 1000.0)


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
        self.timeout_ms_calls = []

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def get_action(self, *, wait=False, timeout_ms=None):
        assert wait is True
        self.timeout_ms_calls.append(timeout_ms)
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
    assert leader.timeout_ms_calls == [config.startup_timeout_ms]

    action = teleop.get_action()
    assert action["joint6.pos"] == 0.4
    assert leader.timeout_ms_calls[-1] == config.action_max_age_ms

    teleop.prepare_episode()
    action = teleop.get_action()
    assert action["joint6.pos"] == 0.4
    assert leader.timeout_ms_calls[-1] == config.startup_timeout_ms

    action = teleop.get_action()
    assert action["joint6.pos"] == 0.4
    assert leader.timeout_ms_calls[-1] == config.action_max_age_ms

    teleop.send_feedback({})
    teleop.disconnect()
    assert leader.stopped
    assert leader.arm.closed


def test_teleoperator_runtime_action_wait_uses_action_max_age_after_idle_gap() -> None:
    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _Leader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    teleop.connect()
    teleop._last_action_monotonic = None

    action = teleop.get_action()

    assert action["joint6.pos"] == 0.4
    assert leader.action_calls[-1] == (True, config.action_max_age_ms)

    teleop.get_action()
    assert leader.action_calls[-1] == (True, config.action_max_age_ms)


def test_teleoperator_reuses_latest_fresh_action_when_next_frame_wait_times_out() -> None:
    class _CachedLeader(_Leader):
        def __init__(self):
            super().__init__()
            self.calls_after_connect = 0

        def get_action(self, *, wait=False, timeout_ms=None):
            self.action_calls.append((wait, timeout_ms))
            if not self.started:
                return super().get_action(wait=wait, timeout_ms=timeout_ms)
            self.calls_after_connect += 1
            if self.calls_after_connect == 1:
                return _Action((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4), timestamp=perf_counter())
            if wait:
                return None
            return _Action((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5), timestamp=perf_counter(), sequence=2)

    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _CachedLeader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    teleop.connect()
    action = teleop.get_action()

    assert action["joint6.pos"] == 0.5
    assert leader.action_calls[-2:] == [(True, config.action_max_age_ms), (False, None)]


def test_teleoperator_prepare_episode_uses_startup_wait_once() -> None:
    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _Leader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    teleop.connect()
    teleop.prepare_episode()

    action = teleop.get_action()

    assert action["joint6.pos"] == 0.4
    assert leader.action_calls[-1] == (True, config.startup_timeout_ms)

    teleop.get_action()
    assert leader.action_calls[-1] == (True, config.action_max_age_ms)


def test_teleoperator_diagnostic_action_reads_fresh_action_without_dataset_side_effect() -> None:
    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _Leader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    teleop.connect()
    diagnostic = teleop.get_diagnostic_action()

    assert diagnostic.action["joint6.pos"] == 0.4
    assert diagnostic.age_ms <= config.action_max_age_ms
    assert diagnostic.sequence == 1
    assert leader.action_calls[-1] == (True, config.action_max_age_ms)


def test_teleoperator_startup_fails_before_recording_when_no_initial_action() -> None:
    class _NoActionLeader(_Leader):
        def get_action(self, *, wait=False, timeout_ms=None):
            self.action_calls.append((wait, timeout_ms))
            return None

    config = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB0")
    leader = _NoActionLeader()
    teleop = ZyArmLeaderTeleoperator(config, leader=leader)

    try:
        teleop.connect()
    except RuntimeError as exc:
        assert "during startup" in str(exc)
    else:
        raise AssertionError("connect should fail when no startup action is available")

    assert leader.action_calls == [(True, config.startup_timeout_ms)]
    assert leader.stopped
    assert leader.arm.closed
