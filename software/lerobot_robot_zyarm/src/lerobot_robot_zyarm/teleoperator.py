from __future__ import annotations

from typing import Any, Optional

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction
from zyarm_sdk import MappingConfig, TeleopConfig, ZyArm, ZyArmConfig
from zyarm_sdk.retarget import Retargeter
from zyarm_sdk.teleop import ZyArmLeader

from .config import ZyArmLeaderTeleoperatorConfig
from .conversion import positions_to_action
from .features import joint_features


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
        if not getattr(self.leader.arm, "is_connected", False):
            self.leader.arm.connect()
        self.leader.start()
        self.configure()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_action(self) -> RobotAction:
        action = self.leader.get_action(wait=True, timeout_ms=self.config.wait_timeout_ms)
        if action is None:
            raise RuntimeError("No zyarm leader action available")
        positions = self.retargeter.apply(action)
        return positions_to_action(positions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        return None

    def disconnect(self) -> None:
        if getattr(self.leader.arm, "is_connected", False):
            self.leader.stop()
            self.leader.arm.close()

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
