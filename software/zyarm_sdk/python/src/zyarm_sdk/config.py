from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

from .types import ARM_JOINT_COUNT, JOINT_COUNT


def _tuple(values: Sequence[float], expected: int, name: str) -> Tuple[float, ...]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    return parsed


@dataclass(frozen=True)
class MappingConfig:
    arm_offsets_deg: Tuple[float, ...] = (0.0, -180.0, 90.0, 0.0, 0.0, 0.0)
    arm_signs: Tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    gripper_command_max: float = 100.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "arm_offsets_deg",
            _tuple(self.arm_offsets_deg, ARM_JOINT_COUNT, "arm_offsets_deg"),
        )
        object.__setattr__(
            self,
            "arm_signs",
            _tuple(self.arm_signs, ARM_JOINT_COUNT, "arm_signs"),
        )
        if self.gripper_command_max <= 0.0:
            raise ValueError("gripper_command_max must be positive")


@dataclass(frozen=True)
class SafetyConfig:
    min_positions: Optional[Tuple[float, ...]] = None
    max_positions: Optional[Tuple[float, ...]] = None
    max_delta: Optional[Tuple[float, ...]] = None

    def __post_init__(self) -> None:
        for name in ("min_positions", "max_positions", "max_delta"):
            values = getattr(self, name)
            if values is not None:
                object.__setattr__(self, name, _tuple(values, JOINT_COUNT, name))


@dataclass(frozen=True)
class ZyArmConfig:
    port: str
    baudrate: int = 230_400
    timeout_s: float = 0.02
    write_timeout_s: float = 0.05
    # 普通 ACK 通常来自配置/模式切换命令，应快速暴露串口、波特率或协议错误。
    ack_timeout_s: float = 1.0
    # 动作 ACK 表示固件报告动作执行完成，reset/move_ik/同步夹爪可能需要更久。
    action_timeout_s: float = 10.0
    # 录制动作最长可达 3 分钟，回放完成 ACK 需要预留少量收尾和串口调度余量。
    play_record_timeout_s: float = 190.0
    reset_rts_dtr: bool = False
    reset_quiet_s: float = 0.0
    mapping: MappingConfig = field(default_factory=MappingConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)


@dataclass(frozen=True)
class TeleopConfig:
    leader_hz: float = 50.0
    action_max_age_ms: float = 100.0
    state_max_age_ms: Optional[float] = None
    wait_timeout_ms: float = 50.0
    slave_filter_lpf_alpha: float = 0.15
    verbose: bool = False
