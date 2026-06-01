"""数据加载抽象层：统一接口，解耦 LeRobot"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ReplayFrame:
    """单帧回放数据"""
    timestamp: float
    joints: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class EpisodeData:
    """Episode 回放数据"""
    episode_index: int
    frames: tuple[ReplayFrame, ...]
    images: dict[str, list[np.ndarray]]  # {"front": [...], "wrist": [...]}


def _load_episode_base(dataset_root: Path, episode_index: int):
    from .replay_node import load_episode

    return load_episode(dataset_root, episode_index)


def _load_images(dataset_root: Path, episode_index: int, frames):
    from .lerobot_video_adapter import load_images_for_episode

    return load_images_for_episode(dataset_root, episode_index, frames)


def load_images_for_replay_best_effort(
    dataset_root: Path,
    episode_index: int,
    frames,
    log_error,
) -> dict[str, list[np.ndarray]]:
    try:
        return _load_images(dataset_root, episode_index, frames)
    except Exception as exc:
        log_error(f"Image loading disabled for episode {episode_index}: {exc}")
        return {}


def load_episode_for_replay(
    dataset_root: Path,
    episode_index: int,
    load_images: bool = False,
) -> EpisodeData:
    """
    加载 episode 用于回放质检

    Args:
        dataset_root: 数据集根目录
        episode_index: Episode 索引
        load_images: 是否加载图像（质检模式用 True）

    Returns:
        EpisodeData: 包含 frames 和 images（如果启用）
    """
    episode_replay_data = _load_episode_base(dataset_root, episode_index)
    frames = episode_replay_data.frames

    images: dict[str, list[np.ndarray]] = {}
    if load_images:
        images = _load_images(dataset_root, episode_index, frames)

    return EpisodeData(
        episode_index=episode_index,
        frames=frames,
        images=images,
    )
