from __future__ import annotations

import time
import warnings
from dataclasses import dataclass
from typing import Optional

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from zyarm_sdk import MappingConfig, SafetyConfig, ZyArm, ZyArmConfig
from zyarm_sdk.safety import SafetyController

from .config import ZyArmFollowerRobotConfig
from .conversion import action_to_positions, positions_to_action
from .features import joint_features, observation_features
from .profile import once_profiler


@dataclass(frozen=True)
class FollowerStateDiagnostic:
    positions: tuple[float, ...]
    age_ms: float
    source: object
    sequence: object


@dataclass(frozen=True)
class FollowerStateDiagnostic:
    positions: tuple[float, ...]
    age_ms: float
    source: object
    sequence: object


class ZyArmFollowerRobot(Robot):
    config_class = ZyArmFollowerRobotConfig
    name = "zyarm_follower"

    def __init__(
        self,
        config: ZyArmFollowerRobotConfig,
        *,
        arm: Optional[ZyArm] = None,
        cameras: Optional[dict] = None,
    ) -> None:
        super().__init__(config)
        self.config = config
        self.arm = arm or ZyArm(self._make_sdk_config(config))
        self.safety = SafetyController(config.safety)
        self.cameras = cameras if cameras is not None else make_cameras_from_configs(config.cameras)
        self._slave_filter_started = False
        self._last_observation_monotonic: float | None = None
        self.last_observation_state_timestamp: float | None = None
        self.last_observation_state_age_ms: float | None = None

    @property
    def observation_features(self) -> dict:
        return observation_features(self.config.cameras)

    @property
    def action_features(self) -> dict[str, type]:
        return joint_features()

    @property
    def is_connected(self) -> bool:
        arm_connected = bool(getattr(self.arm, "is_connected", False))
        cameras_connected = all(bool(getattr(camera, "is_connected", False)) for camera in self.cameras.values())
        return arm_connected and cameras_connected

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            for camera in self.cameras.values():
                if not getattr(camera, "is_connected", False):
                    camera.connect()

            if not getattr(self.arm, "is_connected", False):
                self.arm.connect()
            state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
            if state is None:
                raise RuntimeError("Failed to read initial zyarm follower state after camera startup")
            if not self._slave_filter_started:
                self._start_slave_filter()
            self._last_observation_monotonic = None
            self.configure()
        except Exception:
            self.disconnect()
            raise

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_observation(self) -> RobotObservation:
        profile_token = once_profiler.begin("get_observation")
        profile_sampled = bool(profile_token and profile_token[2])
        state = self._get_observation_state()
        if state is None and self.config.query_state_on_missing_cache:
            state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
        if state is None:
            raise RuntimeError("No zyarm follower state available")
        self._last_observation_monotonic = time.perf_counter()
        self.last_observation_state_timestamp = getattr(state, "timestamp", None)
        self.last_observation_state_age_ms = getattr(state, "age_ms", None)

        observation: RobotObservation = positions_to_action(state.positions)
        for name, camera in self.cameras.items():
            if profile_sampled:
                camera_start = time.perf_counter()
                observation[name] = camera.read_latest()
                once_profiler.add_camera_value(name, (time.perf_counter() - camera_start) * 1000.0)
            else:
                observation[name] = camera.read_latest()
        sampled = once_profiler.end(profile_token)
        if sampled:
            once_profiler.add_value("state_age_ms", getattr(state, "age_ms", None))
            once_profiler.update_frame_stats(self.arm)
            once_profiler.maybe_print()
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        profile_token = once_profiler.begin("send_action")
        positions = action_to_positions(action)
        safe_positions = self.safety.sanitize_positions(positions)
        self.arm.fast_io(safe_positions)
        if once_profiler.end(profile_token):
            once_profiler.maybe_print()
        return positions_to_action(safe_positions)

    def refresh_state_for_diagnostics(self) -> FollowerStateDiagnostic:
        state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
        if state is None:
            raise RuntimeError("No zyarm follower state available for B_ENTER_DIAGNOSTIC")
        return FollowerStateDiagnostic(
            positions=tuple(state.positions),
            age_ms=float(getattr(state, "age_ms", 0.0)),
            source=getattr(state, "source", None),
            sequence=getattr(state, "sequence", None),
        )

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if getattr(camera, "is_connected", False):
                camera.disconnect()
        self._last_observation_monotonic = None
        self.last_observation_state_timestamp = None
        self.last_observation_state_age_ms = None
        self._stop_slave_filter()
        if getattr(self.arm, "is_connected", False):
            self.arm.close()

    def _start_slave_filter(self) -> None:
        result = self.arm.set_master_slave_lpf(self.config.slave_filter_lpf_alpha)
        if not result.accepted:
            raise RuntimeError("Failed to configure zyarm follower slave filter LPF")
        result = self.arm.enter_slave_mode()
        if not result.accepted:
            raise RuntimeError("Failed to enter zyarm follower slave filter mode")
        self._slave_filter_started = True

    def _stop_slave_filter(self) -> None:
        if not self._slave_filter_started or not getattr(self.arm, "is_connected", False):
            return
        try:
            result = self.arm.stop_master_mode()
            if not result.accepted:
                warnings.warn("Failed to stop zyarm follower slave filter mode", RuntimeWarning)
        except Exception as exc:
            warnings.warn(
                f"Failed to stop zyarm follower slave filter mode: {exc}",
                RuntimeWarning,
            )
        finally:
            self._slave_filter_started = False

    def _get_observation_state(self):
        if self._observation_needs_refresh_after_idle():
            return self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
        return self.arm.get_latest_state(self.config.state_max_age_ms)

    def _observation_needs_refresh_after_idle(self) -> bool:
        if self._last_observation_monotonic is None:
            return True
        if self.config.state_max_age_ms is None:
            return False
        idle_s = time.perf_counter() - self._last_observation_monotonic
        return idle_s > (self.config.state_max_age_ms / 1000.0)

    @staticmethod
    def _make_sdk_config(config: ZyArmFollowerRobotConfig) -> ZyArmConfig:
        return ZyArmConfig(
            port=config.port,
            baudrate=config.baudrate,
            timeout_s=config.timeout_s,
            write_timeout_s=config.write_timeout_s,
            ack_timeout_s=config.ack_timeout_s,
            action_timeout_s=config.action_timeout_s,
            play_record_timeout_s=config.play_record_timeout_s,
            reset_rts_dtr=config.reset_rts_dtr,
            reset_quiet_s=config.reset_quiet_s,
            mapping=config.mapping if config.mapping is not None else MappingConfig(),
            safety=SafetyConfig(),
        )
