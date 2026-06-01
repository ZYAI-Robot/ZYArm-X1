from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Optional

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction
from zyarm_sdk import MappingConfig, TeleopConfig, ZyArm, ZyArmConfig
from zyarm_sdk.retarget import Retargeter
from zyarm_sdk.teleop import ZyArmLeader

from .config import ZyArmLeaderTeleoperatorConfig
from .conversion import positions_to_action
from .features import joint_features
from .profile import once_profiler


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LeaderActionDiagnostic:
    action: RobotAction
    age_ms: float
    source: object
    sequence: object


class ZyArmLeaderTeleoperator(Teleoperator):
    config_class = ZyArmLeaderTeleoperatorConfig
    name = "zyarm_leader"

    def __init__(
        self,
        config: ZyArmLeaderTeleoperatorConfig,
        *,
        arm: Optional[ZyArm] = None,
        leader: Optional[ZyArmLeader] = None,
    ) -> None:
        super().__init__(config)
        self.config = config
        if leader is not None:
            self.leader = leader
            self.arm = leader.arm
        else:
            self.arm = arm or ZyArm(self._make_sdk_config(config))
            self.leader = ZyArmLeader(self.arm, self._make_teleop_config(config))
        self.retargeter = Retargeter(config.retarget)
        self._last_action_monotonic: float | None = None
        self._episode_start_action_refresh = False

    @property
    def action_features(self) -> dict[str, type]:
        return joint_features()

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return bool(getattr(self.leader.arm, "is_connected", False))

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            if not getattr(self.leader.arm, "is_connected", False):
                self.leader.arm.connect()
            self.leader.start()
            action = self.leader.get_action(wait=True, timeout_ms=self.config.startup_timeout_ms)
            if action is None:
                raise RuntimeError("No zyarm leader action available during startup")
            self._last_action_monotonic = time.perf_counter()
            logger.info(
                "ZYArm leader startup action ready: sequence=%s age_ms=%.1f timeout_ms=%.1f",
                getattr(action, "sequence", None),
                getattr(action, "age_ms", -1.0),
                self.config.startup_timeout_ms,
            )
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

    def get_action(self) -> RobotAction:
        idle_ms = self._action_idle_ms()
        timeout_ms = (
            self.config.startup_timeout_ms
            if self._episode_start_action_refresh
            else self.config.action_max_age_ms
        )
        profile_token = once_profiler.begin("get_action")
        action = self.leader.get_action(wait=True, timeout_ms=timeout_ms)
        sampled = once_profiler.end(profile_token)
        if action is None:
            action = self.leader.get_action(wait=False)
            if action is None:
                logger.warning(
                    "ZYArm leader action unavailable: timeout_ms=%.1f idle_ms=%s sequence_now=%s",
                    timeout_ms,
                    "unknown" if idle_ms is None else f"{idle_ms:.1f}",
                    self._master_data_sequence(),
                )
                raise RuntimeError("No zyarm leader action available")
            logger.debug(
                "ZYArm leader reused latest fresh action after waiting for next frame timed out: "
                "sequence=%s age_ms=%.1f timeout_ms=%.1f",
                getattr(action, "sequence", None),
                getattr(action, "age_ms", -1.0),
                timeout_ms,
            )
            raise RuntimeError("No zyarm leader action available")
        if sampled:
            once_profiler.add_value("action_age_ms", getattr(action, "age_ms", None))
            once_profiler.update_frame_stats(self.leader.arm)
            once_profiler.maybe_print()
        self._last_action_monotonic = time.perf_counter()
        if self._episode_start_action_refresh:
            logger.info(
                "ZYArm leader episode action ready: sequence=%s age_ms=%.1f timeout_ms=%.1f",
                getattr(action, "sequence", None),
                getattr(action, "age_ms", -1.0),
                timeout_ms,
            )
            self._episode_start_action_refresh = False
        positions = self.retargeter.apply(action)
        return positions_to_action(positions)

    def get_diagnostic_action(self) -> LeaderActionDiagnostic:
        action = self.leader.get_action(wait=True, timeout_ms=self.config.action_max_age_ms)
        if action is None:
            logger.warning(
                "ZYArm leader diagnostic action unavailable: timeout_ms=%.1f sequence_now=%s",
                self.config.action_max_age_ms,
                self._master_data_sequence(),
            )
            raise RuntimeError("No zyarm leader action available for B_ENTER_DIAGNOSTIC")
        self._last_action_monotonic = time.perf_counter()
        positions = self.retargeter.apply(action)
        return LeaderActionDiagnostic(
            action=positions_to_action(positions),
            age_ms=float(getattr(action, "age_ms", 0.0)),
            source=getattr(action, "source", None),
            sequence=getattr(action, "sequence", None),
        )

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        return None

    def disconnect(self) -> None:
        self._last_action_monotonic = None
        self._episode_start_action_refresh = False
        if getattr(self.leader.arm, "is_connected", False):
            self.leader.stop()
            self.leader.arm.close()

    def prepare_episode(self) -> None:
        self._episode_start_action_refresh = True

    def _action_idle_ms(self) -> float | None:
        if self._last_action_monotonic is None:
            return None
        return (time.perf_counter() - self._last_action_monotonic) * 1000.0

    def _master_data_sequence(self):
        transport = getattr(self.leader.arm, "transport", None)
        if transport is None:
            return None
        return getattr(transport, "master_data_sequence", None)

    @staticmethod
    def _make_sdk_config(config: ZyArmLeaderTeleoperatorConfig) -> ZyArmConfig:
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
        )

    @staticmethod
    def _make_teleop_config(config: ZyArmLeaderTeleoperatorConfig) -> TeleopConfig:
        return TeleopConfig(
            leader_hz=config.leader_hz,
            action_max_age_ms=config.action_max_age_ms,
            wait_timeout_ms=config.wait_timeout_ms,
        )
