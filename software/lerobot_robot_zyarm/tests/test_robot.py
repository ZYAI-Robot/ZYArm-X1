from dataclasses import dataclass

from lerobot.cameras import CameraConfig
from lerobot_robot_zyarm.config import ZyArmFollowerRobotConfig
from lerobot_robot_zyarm.features import ACTION_KEYS
from lerobot_robot_zyarm.robot import ZyArmFollowerRobot


@dataclass(frozen=True)
class _State:
    positions: tuple[float, ...]


@dataclass(frozen=True)
class _Result:
    accepted: bool = True


class _Arm:
    def __init__(self):
        self.is_connected = False
        self.latest_state = _State((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5))
        self.fast_io_calls = []
        self.lpf_calls = []
        self.enter_slave_mode_calls = 0
        self.stop_master_mode_calls = 0
        self.closed = False

    def connect(self):
        self.is_connected = True
        return self

    def close(self):
        self.is_connected = False
        self.closed = True

    def query_state(self, timeout_ms=1000.0):
        del timeout_ms
        return self.latest_state

    def get_latest_state(self, max_age_ms=None):
        del max_age_ms
        return self.latest_state

    def set_master_slave_lpf(self, alpha):
        self.lpf_calls.append(alpha)
        return _Result()

    def enter_slave_mode(self):
        self.enter_slave_mode_calls += 1
        return _Result()

    def stop_master_mode(self):
        self.stop_master_mode_calls += 1
        return _Result()

    def fast_io(self, positions):
        self.fast_io_calls.append(list(positions))


def test_robot_connect_observation_and_disconnect() -> None:
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    arm = _Arm()
    robot = ZyArmFollowerRobot(config, arm=arm)

    assert robot.action_features == {key: float for key in ACTION_KEYS}
    assert robot.observation_features["front"] == (240, 320, 3)

    robot.connect()
    assert robot.is_connected
    assert arm.lpf_calls == [config.slave_filter_lpf_alpha]
    assert arm.enter_slave_mode_calls == 1

    robot.connect()
    assert arm.lpf_calls == [config.slave_filter_lpf_alpha]
    assert arm.enter_slave_mode_calls == 1

    observation = robot.get_observation()
    assert observation["joint6.pos"] == 0.5
    assert observation["front"] == "front-image"

    robot.disconnect()
    assert arm.stop_master_mode_calls == 1
    assert arm.closed


def test_robot_send_action_uses_non_blocking_fast_io_and_returns_sent_action() -> None:
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={},
    )
    arm = _Arm()
    robot = ZyArmFollowerRobot(config, arm=arm)
    action = {key: 0.0 for key in ACTION_KEYS}
    action["joint6.pos"] = 1.2

    sent = robot.send_action(action)

    assert arm.fast_io_calls == [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    assert sent["joint6.pos"] == 1.0
