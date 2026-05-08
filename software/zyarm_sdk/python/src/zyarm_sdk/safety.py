from __future__ import annotations

from typing import List, Optional, Sequence

from .config import SafetyConfig
from .errors import SafetyError, StaleStateError
from .types import JOINT_COUNT


def validate_positions(positions: Sequence[float]) -> List[float]:
    values = [float(value) for value in positions]
    if len(values) != JOINT_COUNT:
        raise SafetyError(f"Expected {JOINT_COUNT} joint values, got {len(values)}")
    return values


def validate_apply_mask(apply_mask: Optional[Sequence[bool]]) -> List[bool]:
    if apply_mask is None:
        return [True] * JOINT_COUNT
    values = [bool(value) for value in apply_mask]
    if len(values) != JOINT_COUNT:
        raise SafetyError(f"Expected {JOINT_COUNT} apply flags, got {len(values)}")
    return values


class SafetyController:
    def __init__(self, config: Optional[SafetyConfig] = None) -> None:
        self.config = config or SafetyConfig()
        self._last_positions: Optional[List[float]] = None

    def sanitize_positions(self, positions: Sequence[float]) -> List[float]:
        values = validate_positions(positions)
        if self.config.min_positions is not None:
            values = [max(value, self.config.min_positions[index]) for index, value in enumerate(values)]
        if self.config.max_positions is not None:
            values = [min(value, self.config.max_positions[index]) for index, value in enumerate(values)]
        if self.config.max_delta is not None and self._last_positions is not None:
            limited = []
            for index, value in enumerate(values):
                delta = value - self._last_positions[index]
                step = self.config.max_delta[index]
                if delta > step:
                    value = self._last_positions[index] + step
                elif delta < -step:
                    value = self._last_positions[index] - step
                limited.append(value)
            values = limited
        self._last_positions = list(values)
        return values

    def assert_fresh(self, item, max_age_ms: Optional[float]) -> None:
        if max_age_ms is not None and item.age_ms > float(max_age_ms):
            raise StaleStateError(f"Cached data is stale: {item.age_ms:.1f} ms > {max_age_ms:.1f} ms")
