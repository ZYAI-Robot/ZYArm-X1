from __future__ import annotations

import warnings
from typing import Optional

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from zyarm_sdk import MappingConfig, SafetyConfig, ZyArm, ZyArmConfig
from zyarm_sdk.safety import SafetyController

from .config import ZyArmFollowerRobotConfig
from .conversion import action_to_positions, positions_to_action
from .features import joint_features, observation_features


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
        if not getattr(self.arm, "is_connected", False):
            self.arm.connect()
        state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
        if state is None:
            raise RuntimeError("Failed to read initial zyarm follower state")
        if not self._slave_filter_started:
            self._start_slave_filter()
        try:
            for camera in self.cameras.values():
                if not getattr(camera, "is_connected", False):
                    camera.connect()
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
        state = self.arm.get_latest_state(self.config.state_max_age_ms)
        if state is None and self.config.query_state_on_missing_cache:
            state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)
        if state is None:
            raise RuntimeError("No zyarm follower state available")

        observation: RobotObservation = positions_to_action(state.positions)
        for name, camera in self.cameras.items():
            observation[name] = camera.read_latest()
        return observation

    def send_action(self, action: RobotAction) -> RobotAction:
        positions = action_to_positions(action)
        safe_positions = self.safety.sanitize_positions(positions)
        self.arm.fast_io(safe_positions)
        return positions_to_action(safe_positions)

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            if getattr(camera, "is_connected", False):
                camera.disconnect()
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
