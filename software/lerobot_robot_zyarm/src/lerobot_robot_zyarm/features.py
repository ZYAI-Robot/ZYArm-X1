from __future__ import annotations

from typing import Dict, Mapping, Tuple


JOINT_NAMES: Tuple[str, ...] = tuple(f"joint{index}" for index in range(7))
ACTION_KEYS: Tuple[str, ...] = tuple(f"{name}.pos" for name in JOINT_NAMES)


def joint_features() -> Dict[str, type]:
    return {key: float for key in ACTION_KEYS}


def camera_features(camera_configs: Mapping[str, object]) -> Dict[str, Tuple[int, int, int]]:
    features: Dict[str, Tuple[int, int, int]] = {}
    for name, config in camera_configs.items():
        height = getattr(config, "height", None)
        width = getattr(config, "width", None)
        if height is None or width is None:
            continue
        features[str(name)] = (int(height), int(width), 3)
    return features


def observation_features(camera_configs: Mapping[str, object]) -> Dict[str, object]:
    features: Dict[str, object] = joint_features()
    features.update(camera_features(camera_configs))
    return features
