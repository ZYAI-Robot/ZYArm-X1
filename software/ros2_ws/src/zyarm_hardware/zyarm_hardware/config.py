from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping

import yaml

from zyarm_hardware.protocol import ProtocolConfig


JOINT_COUNT = 7
ARM_JOINT_COUNT = 6

DEFAULT_BAUDRATE = 230400
DEFAULT_TIMEOUT_S = 0.1
DEFAULT_WRITE_TIMEOUT_S = 0.1
DEFAULT_ACK_TIMEOUT_S = 0.2
DEFAULT_LOG_DIR = "/tmp"
DEFAULT_RESET_RTS_DTR_QUIET_S = 3.0
DEFAULT_CLAW_TRAVEL_M = 0.034
DEFAULT_CLAW_COMMAND_MAX = 100.0
DEFAULT_ARM_HW_OFFSETS_DEG = [0.0, -180.0, 90.0, 0.0, 0.0, 0.0]
DEFAULT_ARM_HW_SIGNS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
DEFAULT_STATUS_REPORT_FREQUENCY_HZ = 50.0
DEFAULT_TELEOP_FOLLOW_FREQUENCY_HZ = 50.0


@dataclass(frozen=True)
class ArmKinematicsConfig:
    arm_hw_offsets_deg: List[float]
    arm_hw_signs: List[float]
    claw_travel_m: float
    claw_command_max: float

    def hw_to_ros(self, hw_positions_deg: List[float]) -> List[float]:
        arm = [
            math.radians((float(value) - float(offset)) / float(sign))
            for value, offset, sign in zip(
                hw_positions_deg[:ARM_JOINT_COUNT],
                self.arm_hw_offsets_deg,
                self.arm_hw_signs,
            )
        ]
        claw = max(0.0, min(float(hw_positions_deg[ARM_JOINT_COUNT]), self.claw_command_max))
        arm.append(claw / self.claw_command_max * self.claw_travel_m)
        return arm

    def ros_to_hw(self, ros_positions: List[float]) -> List[float]:
        arm = [
            math.degrees(float(value)) * float(sign) + float(offset)
            for value, offset, sign in zip(
                ros_positions[:ARM_JOINT_COUNT],
                self.arm_hw_offsets_deg,
                self.arm_hw_signs,
            )
        ]
        claw = max(0.0, min(self.claw_travel_m, float(ros_positions[ARM_JOINT_COUNT])))
        arm.append(claw / self.claw_travel_m * self.claw_command_max)
        return arm


@dataclass(frozen=True)
class ArmConfig:
    name: str
    port: str
    baudrate: int
    timeout_s: float
    write_timeout_s: float
    ack_timeout_s: float
    reset_rts_dtr: bool
    reset_rts_dtr_quiet_s: float
    log_serial: bool
    log_dir: str
    startup_reset: bool
    status_report_frequency_hz: float
    kinematics: ArmKinematicsConfig

    def protocol_config(self) -> ProtocolConfig:
        return ProtocolConfig(
            port=self.port,
            baudrate=self.baudrate,
            timeout_s=self.timeout_s,
            write_timeout_s=self.write_timeout_s,
            ack_timeout_s=self.ack_timeout_s,
            reset_rts_dtr=self.reset_rts_dtr,
            reset_rts_dtr_quiet_s=self.reset_rts_dtr_quiet_s,
            log_serial=self.log_serial,
            log_dir=self.log_dir,
        )


@dataclass(frozen=True)
class RelationConfig:
    name: str
    mode: str
    leader: str
    follower: str
    follow_frequency_hz: float


@dataclass(frozen=True)
class SystemConfig:
    arms: Dict[str, ArmConfig]
    relationships: Dict[str, RelationConfig]


class ConfigError(ValueError):
    pass


def load_system_config(path: str | Path) -> SystemConfig:
    config_path = Path(path)
    raw_config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw_config, Mapping):
        raise ConfigError("Top-level config must be a mapping")

    arms_raw = raw_config.get("arms")
    if not isinstance(arms_raw, Mapping) or not arms_raw:
        raise ConfigError("Config must define a non-empty 'arms' mapping")

    arms: Dict[str, ArmConfig] = {}
    for arm_name, arm_payload in arms_raw.items():
        if not isinstance(arm_payload, Mapping):
            raise ConfigError(f"Arm '{arm_name}' must be a mapping")
        arms[str(arm_name)] = _parse_arm_config(str(arm_name), arm_payload)

    relationships_raw = raw_config.get("relationships") or {}
    if not isinstance(relationships_raw, Mapping):
        raise ConfigError("'relationships' must be a mapping when provided")

    relationships: Dict[str, RelationConfig] = {}
    occupied_arms: Dict[str, str] = {}
    for relation_name, relation_payload in relationships_raw.items():
        if not isinstance(relation_payload, Mapping):
            raise ConfigError(f"Relationship '{relation_name}' must be a mapping")
        relation = _parse_relation_config(str(relation_name), relation_payload)
        if relation.leader not in arms:
            raise ConfigError(
                f"Relationship '{relation.name}' references unknown leader '{relation.leader}'"
            )
        if relation.follower not in arms:
            raise ConfigError(
                f"Relationship '{relation.name}' references unknown follower '{relation.follower}'"
            )
        if relation.leader == relation.follower:
            raise ConfigError(f"Relationship '{relation.name}' cannot reuse the same arm twice")
        for arm_name in (relation.leader, relation.follower):
            previous_relation = occupied_arms.get(arm_name)
            if previous_relation is not None:
                raise ConfigError(
                    f"Arm '{arm_name}' is reused by relationships '{previous_relation}' and '{relation.name}'"
                )
            occupied_arms[arm_name] = relation.name
        relationships[relation.name] = relation

    return SystemConfig(arms=arms, relationships=relationships)


def _parse_arm_config(name: str, payload: Mapping[str, Any]) -> ArmConfig:
    port = str(payload.get("port", "")).strip()
    if not port:
        raise ConfigError(f"Arm '{name}' must define a non-empty 'port'")
    arm_hw_offsets_deg = _float_list(
        payload.get("arm_hw_offsets_deg", DEFAULT_ARM_HW_OFFSETS_DEG),
        expected_length=ARM_JOINT_COUNT,
        field_name=f"arms.{name}.arm_hw_offsets_deg",
    )
    arm_hw_signs = _float_list(
        payload.get("arm_hw_signs", DEFAULT_ARM_HW_SIGNS),
        expected_length=ARM_JOINT_COUNT,
        field_name=f"arms.{name}.arm_hw_signs",
    )
    return ArmConfig(
        name=name,
        port=port,
        baudrate=int(payload.get("baudrate", DEFAULT_BAUDRATE)),
        timeout_s=float(payload.get("timeout_s", DEFAULT_TIMEOUT_S)),
        write_timeout_s=float(payload.get("write_timeout_s", DEFAULT_WRITE_TIMEOUT_S)),
        ack_timeout_s=float(payload.get("ack_timeout_s", DEFAULT_ACK_TIMEOUT_S)),
        reset_rts_dtr=bool(payload.get("reset_rts_dtr", False)),
        reset_rts_dtr_quiet_s=_parse_reset_rts_dtr_quiet_s(payload),
        log_serial=bool(payload.get("log_serial", False)),
        log_dir=str(payload.get("log_dir", DEFAULT_LOG_DIR)),
        startup_reset=bool(payload.get("startup_reset", True)),
        status_report_frequency_hz=max(
            0.0,
            float(payload.get("status_report_frequency_hz", DEFAULT_STATUS_REPORT_FREQUENCY_HZ)),
        ),
        kinematics=ArmKinematicsConfig(
            arm_hw_offsets_deg=arm_hw_offsets_deg,
            arm_hw_signs=arm_hw_signs,
            claw_travel_m=float(payload.get("claw_travel_m", DEFAULT_CLAW_TRAVEL_M)),
            claw_command_max=float(payload.get("claw_command_max", DEFAULT_CLAW_COMMAND_MAX)),
        ),
    )


def _parse_reset_rts_dtr_quiet_s(payload: Mapping[str, Any]) -> float:
    if "reset_rts_dtr_quiet_s" in payload:
        value = payload["reset_rts_dtr_quiet_s"]
    else:
        value = payload.get("startup_reset_quiet_s", DEFAULT_RESET_RTS_DTR_QUIET_S)
    return max(0.0, float(value))


def _parse_relation_config(name: str, payload: Mapping[str, Any]) -> RelationConfig:
    mode = str(payload.get("mode", "")).strip()
    if mode != "teleop_follow":
        raise ConfigError(
            f"Relationship '{name}' uses unsupported mode '{mode}', only 'teleop_follow' is supported"
        )
    leader = str(payload.get("leader", "")).strip()
    follower = str(payload.get("follower", "")).strip()
    if not leader or not follower:
        raise ConfigError(f"Relationship '{name}' must define 'leader' and 'follower'")
    follow_frequency_hz = float(payload.get("follow_frequency_hz", DEFAULT_TELEOP_FOLLOW_FREQUENCY_HZ))
    if follow_frequency_hz <= 0.0:
        raise ConfigError(
            f"Relationship '{name}' must define a positive 'follow_frequency_hz', got {follow_frequency_hz}"
        )
    return RelationConfig(
        name=name,
        mode=mode,
        leader=leader,
        follower=follower,
        follow_frequency_hz=follow_frequency_hz,
    )


def _float_list(values: Any, *, expected_length: int, field_name: str) -> List[float]:
    if not isinstance(values, list):
        raise ConfigError(f"Field '{field_name}' must be a list")
    if len(values) != expected_length:
        raise ConfigError(
            f"Field '{field_name}' must contain {expected_length} values, got {len(values)}"
        )
    return [float(value) for value in values]
