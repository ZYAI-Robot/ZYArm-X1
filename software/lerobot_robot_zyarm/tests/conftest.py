from __future__ import annotations

import sys
import types
from dataclasses import dataclass
from pathlib import Path


class _ChoiceRegistry:
    @classmethod
    def register_subclass(cls, name):
        def decorator(subclass):
            subclass._choice_name = name
            return subclass

        return decorator

    def get_choice_name(self, subclass):
        return getattr(subclass, "_choice_name", subclass.__name__)


@dataclass(kw_only=True)
class _RobotConfig(_ChoiceRegistry):
    id: str | None = None
    calibration_dir: Path | None = None

    @property
    def type(self):
        return self.get_choice_name(self.__class__)

    def __post_init__(self):
        if hasattr(self, "cameras") and self.cameras:
            for config in self.cameras.values():
                for attr in ("width", "height", "fps"):
                    if getattr(config, attr, None) is None:
                        raise ValueError(f"Specifying '{attr}' is required")


@dataclass(kw_only=True)
class _TeleoperatorConfig(_ChoiceRegistry):
    id: str | None = None
    calibration_dir: Path | None = None

    @property
    def type(self):
        return self.get_choice_name(self.__class__)


class _Robot:
    config_class = _RobotConfig
    name = "fake_robot"

    def __init__(self, config):
        self.robot_type = self.name
        self.id = config.id


class _Teleoperator:
    config_class = _TeleoperatorConfig
    name = "fake_teleoperator"

    def __init__(self, config):
        self.id = config.id


@dataclass
class _CameraConfig:
    width: int
    height: int
    fps: int
    image: object = "image"


class _Camera:
    def __init__(self, config):
        self.config = config
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def read_latest(self):
        return self.config.image


def _make_cameras_from_configs(configs):
    return {name: _Camera(config) for name, config in configs.items()}


def pytest_configure():
    lerobot = types.ModuleType("lerobot")
    cameras = types.ModuleType("lerobot.cameras")
    cameras.CameraConfig = _CameraConfig
    cameras_utils = types.ModuleType("lerobot.cameras.utils")
    cameras_utils.make_cameras_from_configs = _make_cameras_from_configs

    robots = types.ModuleType("lerobot.robots")
    robots_config = types.ModuleType("lerobot.robots.config")
    robots_config.RobotConfig = _RobotConfig
    robots_robot = types.ModuleType("lerobot.robots.robot")
    robots_robot.Robot = _Robot

    teleoperators = types.ModuleType("lerobot.teleoperators")
    teleoperators_config = types.ModuleType("lerobot.teleoperators.config")
    teleoperators_config.TeleoperatorConfig = _TeleoperatorConfig
    teleoperators_teleoperator = types.ModuleType("lerobot.teleoperators.teleoperator")
    teleoperators_teleoperator.Teleoperator = _Teleoperator

    lerobot_types = types.ModuleType("lerobot.types")
    lerobot_types.RobotAction = dict
    lerobot_types.RobotObservation = dict

    sys.modules.setdefault("lerobot", lerobot)
    sys.modules.setdefault("lerobot.cameras", cameras)
    sys.modules.setdefault("lerobot.cameras.utils", cameras_utils)
    sys.modules.setdefault("lerobot.robots", robots)
    sys.modules.setdefault("lerobot.robots.config", robots_config)
    sys.modules.setdefault("lerobot.robots.robot", robots_robot)
    sys.modules.setdefault("lerobot.teleoperators", teleoperators)
    sys.modules.setdefault("lerobot.teleoperators.config", teleoperators_config)
    sys.modules.setdefault("lerobot.teleoperators.teleoperator", teleoperators_teleoperator)
    sys.modules.setdefault("lerobot.types", lerobot_types)
