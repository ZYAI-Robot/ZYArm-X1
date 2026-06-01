from dataclasses import dataclass
from time import perf_counter

from lerobot.cameras import CameraConfig
from lerobot_robot_zyarm.config import ZyArmFollowerRobotConfig
from lerobot_robot_zyarm.features import ACTION_KEYS
from lerobot_robot_zyarm.robot import ZyArmFollowerRobot
from zyarm_sdk.errors import StaleStateError


@dataclass(frozen=True)
class _State:
    positions: tuple[float, ...]


@dataclass(frozen=True)
class _Result:
    accepted: bool = True


class _Arm:
    def __init__(self, events=None):
        self.is_connected = False
        self.latest_state = _State((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5))
        self.fast_io_calls = []
        self.lpf_calls = []
        self.enter_slave_mode_calls = 0
        self.stop_master_mode_calls = 0
        self.closed = False
        self.events = events
        self.query_state_calls = 0
        self.latest_state_calls = 0

    def connect(self):
        if self.events is not None:
            self.events.append("arm.connect")
        self.is_connected = True
        return self

    def close(self):
        self.is_connected = False
        self.closed = True

    def query_state(self, timeout_ms=1000.0):
        del timeout_ms
        self.query_state_calls += 1
        if self.events is not None:
            self.events.append("arm.query_state")
        return self.latest_state

    def get_latest_state(self, max_age_ms=None):
        del max_age_ms
        self.latest_state_calls += 1
        return self.latest_state

    def set_master_slave_lpf(self, alpha):
        self.lpf_calls.append(alpha)
        return _Result()

    def enter_slave_mode(self):
        if self.events is not None:
            self.events.append("arm.enter_slave_mode")
        self.enter_slave_mode_calls += 1
        return _Result()

    def stop_master_mode(self):
        self.stop_master_mode_calls += 1
        return _Result()

    def fast_io(self, positions):
        self.fast_io_calls.append(list(positions))


class _Camera:
    def __init__(self, image="front-image", events=None):
        self.is_connected = False
        self.image = image
        self.events = events
        self.disconnect_calls = 0

    def connect(self):
        if self.events is not None:
            self.events.append("camera.connect")
        self.is_connected = True

    def disconnect(self):
        if self.events is not None:
            self.events.append("camera.disconnect")
        self.is_connected = False
        self.disconnect_calls += 1

    def read_latest(self):
        return self.image


def test_robot_connect_observation_and_disconnect() -> None:
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    arm = _Arm()
    robot = ZyArmFollowerRobot(config, arm=arm, cameras={"front": _Camera()})

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


def test_robot_connects_cameras_before_arm_and_queries_state_before_slave_mode() -> None:
    events = []
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    arm = _Arm(events)
    robot = ZyArmFollowerRobot(config, arm=arm, cameras={"front": _Camera(events=events)})

    robot.connect()

    assert events[:4] == [
        "camera.connect",
        "arm.connect",
        "arm.query_state",
        "arm.enter_slave_mode",
    ]


def test_robot_first_observation_queries_follower_state_after_slave_mode() -> None:
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    arm = _Arm()
    robot = ZyArmFollowerRobot(config, arm=arm, cameras={"front": _Camera()})

    robot.connect()
    assert arm.query_state_calls == 1
    assert arm.latest_state_calls == 0

    observation = robot.get_observation()
    assert observation["joint6.pos"] == 0.5
    assert arm.query_state_calls == 2
    assert arm.latest_state_calls == 0

    robot.get_observation()
    assert arm.latest_state_calls == 1


def test_robot_observation_refreshes_after_idle_gap_between_record_loops() -> None:
    class _StaleFirstObservationArm(_Arm):
        def __init__(self):
            super().__init__()
            self.raise_stale = False

        def get_latest_state(self, max_age_ms=None):
            self.latest_state_calls += 1
            if self.raise_stale:
                self.raise_stale = False
                raise StaleStateError("Cached data is stale: 244.3 ms > 100.0 ms")
            return super().get_latest_state(max_age_ms)

    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    arm = _StaleFirstObservationArm()
    robot = ZyArmFollowerRobot(config, arm=arm, cameras={"front": _Camera()})

    robot.connect()
    arm.raise_stale = True

    observation = robot.get_observation()

    assert observation["joint6.pos"] == 0.5
    assert arm.query_state_calls == 2
    assert arm.latest_state_calls == 0

    robot._last_observation_monotonic = perf_counter() - 1.0
    arm.raise_stale = True

    robot.get_observation()
    assert arm.query_state_calls == 3
    assert arm.latest_state_calls == 0


def test_robot_runtime_stale_after_startup_refresh_still_fails() -> None:
    class _AlwaysStaleArm(_Arm):
        def get_latest_state(self, max_age_ms=None):
            raise StaleStateError("Cached data is stale: 244.3 ms > 100.0 ms")

    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    robot = ZyArmFollowerRobot(config, arm=_AlwaysStaleArm(), cameras={"front": _Camera()})

    robot.connect()
    robot.get_observation()

    try:
        robot.get_observation()
    except StaleStateError as exc:
        assert "Cached data is stale" in str(exc)
    else:
        raise AssertionError("runtime stale state should still fail after startup refresh is used")


def test_robot_connect_cleans_up_camera_when_arm_startup_fails() -> None:
    class _FailingArm(_Arm):
        def query_state(self, timeout_ms=1000.0):
            super().query_state(timeout_ms)
            return None

    events = []
    config = ZyArmFollowerRobotConfig(
        port="/dev/ttyUSB1",
        cameras={"front": CameraConfig(width=320, height=240, fps=30, image="front-image")},
    )
    camera = _Camera(events=events)
    robot = ZyArmFollowerRobot(config, arm=_FailingArm(events), cameras={"front": camera})

    try:
        robot.connect()
    except RuntimeError as exc:
        assert "initial zyarm follower state" in str(exc)
    else:
        raise AssertionError("connect should fail when follower state is unavailable")

    assert camera.disconnect_calls == 1
    assert not camera.is_connected


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


def test_robot_refresh_state_for_diagnostics_queries_state_without_camera_read() -> None:
    config = ZyArmFollowerRobotConfig(port="/dev/ttyUSB1", cameras={})
    arm = _Arm()
    robot = ZyArmFollowerRobot(config, arm=arm)

    diagnostic = robot.refresh_state_for_diagnostics()

    assert arm.query_state_calls == 1
    assert diagnostic.positions[-1] == 0.5
    assert diagnostic.age_ms == 0.0
