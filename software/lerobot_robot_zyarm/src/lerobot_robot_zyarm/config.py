from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig
from zyarm_sdk.config import MappingConfig, SafetyConfig, TeleopConfig
from zyarm_sdk.retarget import RetargetConfig


@RobotConfig.register_subclass("zyarm_follower")
@dataclass
class ZyArmFollowerRobotConfig(RobotConfig):
    port: str = ""
    baudrate: int = 230_400
    timeout_s: float = 0.02
    write_timeout_s: float = 0.05
    ack_timeout_s: float = 1.0
    action_timeout_s: float = 10.0
    play_record_timeout_s: float = 190.0
    reset_rts_dtr: bool = False
    reset_quiet_s: float = 0.0
    state_max_age_ms: Optional[float] = 100.0
    initial_state_timeout_ms: float = 1000.0
    query_state_on_missing_cache: bool = True
    slave_filter_lpf_alpha: float = 0.15
    cameras: Dict[str, CameraConfig] = field(default_factory=dict)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def __post_init__(self) -> None:
        post_init = getattr(super(), "__post_init__", None)
        if callable(post_init):
            post_init()


@TeleoperatorConfig.register_subclass("zyarm_leader")
@dataclass
class ZyArmLeaderTeleoperatorConfig(TeleoperatorConfig):
    port: str = ""
    baudrate: int = 230_400
    timeout_s: float = 0.02
    write_timeout_s: float = 0.05
    ack_timeout_s: float = 1.0
    action_timeout_s: float = 10.0
    play_record_timeout_s: float = 190.0
    reset_rts_dtr: bool = False
    reset_quiet_s: float = 0.0
    leader_hz: float = 50.0
    action_max_age_ms: float = 100.0
    wait_timeout_ms: float = 50.0
    mapping: MappingConfig = field(default_factory=MappingConfig)
    retarget: RetargetConfig = field(default_factory=RetargetConfig)


ZyArmFollowerConfig = ZyArmFollowerRobotConfig
ZyArmLeaderConfig = ZyArmLeaderTeleoperatorConfig
