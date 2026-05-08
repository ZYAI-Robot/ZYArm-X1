from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .safety import validate_apply_mask, validate_positions
from .types import JOINT_COUNT, TeleopAction


@dataclass(frozen=True)
class RetargetConfig:
    signs: Tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    offsets: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    deadband: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    min_positions: Optional[Tuple[float, ...]] = None
    max_positions: Optional[Tuple[float, ...]] = None
    max_delta: Optional[Tuple[float, ...]] = None
    apply_mask: Tuple[bool, ...] = (True, True, True, True, True, True, True)


class Retargeter:
    def __init__(self, config: Optional[RetargetConfig] = None) -> None:
        self.config = config or RetargetConfig()
        validate_positions(self.config.signs)
        validate_positions(self.config.offsets)
        validate_positions(self.config.deadband)
        validate_apply_mask(self.config.apply_mask)
        self._last: Optional[list[float]] = None

    @property
    def apply_mask(self) -> Tuple[bool, ...]:
        return tuple(bool(value) for value in self.config.apply_mask)

    def apply(self, action: TeleopAction) -> list[float]:
        source = validate_positions(action.positions)
        output: list[float] = []
        for index, value in enumerate(source):
            mapped = value * self.config.signs[index] + self.config.offsets[index]
            if self._last is not None and abs(mapped - self._last[index]) < self.config.deadband[index]:
                mapped = self._last[index]
            if self.config.max_delta is not None and self._last is not None:
                delta = mapped - self._last[index]
                step = self.config.max_delta[index]
                if delta > step:
                    mapped = self._last[index] + step
                elif delta < -step:
                    mapped = self._last[index] - step
            if self.config.min_positions is not None:
                mapped = max(mapped, self.config.min_positions[index])
            if self.config.max_positions is not None:
                mapped = min(mapped, self.config.max_positions[index])
            output.append(mapped)
        self._last = list(output)
        return output
