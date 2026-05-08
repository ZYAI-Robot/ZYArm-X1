from __future__ import annotations

import math
from typing import List, Optional, Sequence

from .config import MappingConfig
from .errors import SafetyError
from .types import ARM_JOINT_COUNT, JOINT_COUNT, NO_CHANGE_SENTINEL


class JointMapping:
    def __init__(self, config: Optional[MappingConfig] = None) -> None:
        self.config = config or MappingConfig()

    def public_to_hardware(
        self,
        positions: Sequence[float],
        apply_mask: Optional[Sequence[bool]] = None,
    ) -> List[float]:
        values = _validate_positions(positions)
        mask = [True] * JOINT_COUNT if apply_mask is None else _validate_apply_mask(apply_mask)
        hardware: List[float] = []
        for index in range(ARM_JOINT_COUNT):
            if not mask[index]:
                hardware.append(NO_CHANGE_SENTINEL)
                continue
            hardware.append(
                math.degrees(values[index]) * self.config.arm_signs[index]
                + self.config.arm_offsets_deg[index]
            )
        if mask[ARM_JOINT_COUNT]:
            gripper = min(1.0, max(0.0, values[ARM_JOINT_COUNT]))
            hardware.append(gripper * self.config.gripper_command_max)
        else:
            hardware.append(NO_CHANGE_SENTINEL)
        return hardware

    def hardware_to_public(self, positions: Sequence[float]) -> List[float]:
        values = _validate_positions(positions)
        public: List[float] = []
        for index in range(ARM_JOINT_COUNT):
            public.append(
                math.radians(
                    (values[index] - self.config.arm_offsets_deg[index])
                    / self.config.arm_signs[index]
                )
            )
        gripper = min(self.config.gripper_command_max, max(0.0, values[ARM_JOINT_COUNT]))
        public.append(gripper / self.config.gripper_command_max)
        return public


def _validate_positions(positions: Sequence[float]) -> List[float]:
    values = [float(value) for value in positions]
    if len(values) != JOINT_COUNT:
        raise SafetyError(f"Expected {JOINT_COUNT} joint values, got {len(values)}")
    return values


def _validate_apply_mask(apply_mask: Sequence[bool]) -> List[bool]:
    values = [bool(value) for value in apply_mask]
    if len(values) != JOINT_COUNT:
        raise SafetyError(f"Expected {JOINT_COUNT} apply flags, got {len(values)}")
    return values
