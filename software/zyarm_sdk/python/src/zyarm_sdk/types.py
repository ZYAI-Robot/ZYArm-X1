from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


ARM_JOINT_COUNT = 6
JOINT_COUNT = 7
NO_CHANGE_SENTINEL = -999.9


class StateSource(str, Enum):
    CACHE = "cache"
    CMD6_QUERY = "cmd6_query"
    CMD36_MEASURED_SNAPSHOT = "cmd36_measured_snapshot"
    STATUS_REPORT = "status_report"
    MASTER_DATA = "master_data"
    UNKNOWN = "unknown"


def _age_ms(timestamp: float) -> float:
    return max(0.0, (time.perf_counter() - timestamp) * 1000.0)


@dataclass(frozen=True)
class ArmState:
    positions: Tuple[float, ...]
    source: StateSource
    timestamp: float
    sequence: int
    raw_line: str = ""

    @property
    def age_ms(self) -> float:
        return _age_ms(self.timestamp)


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    message: str
    command_id: int
    dispatched_at: float


@dataclass(frozen=True)
class FastIoResult:
    accepted: bool
    message: str
    command_id: int
    dispatched_at: float
    measured_snapshot: Optional[ArmState] = None
    state_waited: bool = False


@dataclass(frozen=True)
class TeleopAction:
    positions: Tuple[float, ...]
    source: StateSource
    timestamp: float
    sequence: int
    raw_line: str = ""

    @property
    def age_ms(self) -> float:
        return _age_ms(self.timestamp)


@dataclass(frozen=True)
class TeleopStepResult:
    action: TeleopAction
    command: FastIoResult
    observation: Optional[ArmState]


@dataclass(frozen=True)
class ArmFrameStats:
    master_data_received: int = 0
    master_data_gap_count: int = 0
    master_data_rate_hz: float = 0.0
    status_received: int = 0
    status_rate_hz: float = 0.0
