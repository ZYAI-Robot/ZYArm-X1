from __future__ import annotations

import re
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, List, Optional, Sequence

from .errors import ProtocolError
from .types import JOINT_COUNT, NO_CHANGE_SENTINEL


ACK_RE = re.compile(r"ACK_COMPLETED:\s*CMD_ID=(\d+),\s*(SUCCESS|ERROR)")
STATUS_RE = re.compile(
    r"\[STATUS\]\s*J0:([-\d.]+)\s*J1:([-\d.]+)\s*J2:([-\d.]+)\s*J3:([-\d.]+)\s*J4:([-\d.]+)\s*J5:([-\d.]+)\s*CLAW:([-\d.]+)"
)
MASTER_DATA_RE = re.compile(
    r"\[MD\]\[(\d+)\]\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]"
)


class CommandId(IntEnum):
    IK_INVERSE = 0
    RESET = 1
    STOP = 2
    JOINT_SYNC = 3
    STATUS = 6
    SET_CLAW = 8
    SET_JOINT_SPEED = 11
    RECORD_PLAYER = 14
    STATUS_REPORT = 17
    REMOTE_MODE = 24
    REMOTE_RESET = 25
    REMOTE = 26
    MASTER_SLAVE = 32
    MASTER_SLAVE_STOP = 33
    MASTER_SLAVE_SET_LPF = 34
    JOINT_IO_FAST = 36


class MasterSlaveRole(IntEnum):
    MASTER = 1
    SLAVE = 2


@dataclass(frozen=True)
class AckFrame:
    command_id: int
    success: bool
    raw_line: str


@dataclass(frozen=True)
class StatusFrame:
    values: List[float]
    received_at: float
    sequence: int
    raw_line: str


@dataclass(frozen=True)
class MasterDataFrame:
    frame_id: int
    values: List[float]
    received_at: float
    sequence: int
    raw_line: str


def format_number(value: float) -> str:
    numeric = float(value)
    if numeric.is_integer():
        return str(int(numeric))
    return f"{numeric:.3f}"


def format_command(command_id: int, params: Optional[Sequence[float]] = None) -> str:
    if not params:
        return f"[CMD][{int(command_id)}]\n"
    return f"[CMD][{int(command_id)}][{' '.join(format_number(value) for value in params)}]\n"


def format_joint_io_fast_command(hardware_positions: Sequence[float]) -> str:
    values = [float(value) for value in hardware_positions]
    if len(values) != JOINT_COUNT:
        raise ProtocolError(f"CMD36 requires {JOINT_COUNT} values, got {len(values)}")
    return format_command(CommandId.JOINT_IO_FAST, values)


def parse_ack(line: str) -> Optional[AckFrame]:
    match = ACK_RE.search(line)
    if not match:
        return None
    return AckFrame(
        command_id=int(match.group(1)),
        success=match.group(2).upper() == "SUCCESS",
        raw_line=line,
    )


def _parse_float_fields(fields: Iterable[str], *, raw_line: str) -> List[float]:
    values: List[float] = []
    for field in fields:
        try:
            values.append(float(field))
        except ValueError as exc:
            raise ProtocolError(f"Malformed numeric field in line: {raw_line!r}") from exc
    return values


def parse_status_line(
    line: str,
    *,
    sequence: int = 0,
    received_at: Optional[float] = None,
) -> Optional[StatusFrame]:
    match = STATUS_RE.search(line)
    if not match:
        return None
    return StatusFrame(
        values=_parse_float_fields(
            (match.group(index) for index in range(1, JOINT_COUNT + 1)),
            raw_line=line,
        ),
        received_at=time.perf_counter() if received_at is None else float(received_at),
        sequence=int(sequence),
        raw_line=line,
    )


def parse_master_data_line(
    line: str,
    *,
    sequence: int = 0,
    received_at: Optional[float] = None,
) -> Optional[MasterDataFrame]:
    match = MASTER_DATA_RE.search(line)
    if not match:
        return None
    return MasterDataFrame(
        frame_id=int(match.group(1)),
        values=_parse_float_fields(
            (match.group(index) for index in range(2, JOINT_COUNT + 2)),
            raw_line=line,
        ),
        received_at=time.perf_counter() if received_at is None else float(received_at),
        sequence=int(sequence),
        raw_line=line,
    )


__all__ = [
    "NO_CHANGE_SENTINEL",
    "AckFrame",
    "CommandId",
    "MasterSlaveRole",
    "MasterDataFrame",
    "StatusFrame",
    "format_command",
    "format_joint_io_fast_command",
    "parse_ack",
    "parse_master_data_line",
    "parse_status_line",
]
