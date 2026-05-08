from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence

from zyarm_sdk.safety import validate_positions

from .features import ACTION_KEYS

GRIPPER_INDEX = 6


def _to_float(value: Any) -> float:
    if hasattr(value, "item"):
        value = value.item()
    return float(value)


def action_to_positions(action: Mapping[str, Any]) -> list[float]:
    missing = [key for key in ACTION_KEYS if key not in action]
    if missing:
        raise ValueError(f"Missing zyarm action keys: {missing}")
    return normalize_public_positions(_to_float(action[key]) for key in ACTION_KEYS)


def positions_to_action(positions: Sequence[float]) -> Dict[str, float]:
    values = normalize_public_positions(positions)
    return {key: float(value) for key, value in zip(ACTION_KEYS, values)}


def normalize_public_positions(positions: Sequence[float]) -> list[float]:
    values = validate_positions(positions)
    values[GRIPPER_INDEX] = min(1.0, max(0.0, values[GRIPPER_INDEX]))
    return values
