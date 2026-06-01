"""LeRobot 视频解码适配层：隔离 LeRobot API 变化"""
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd

if TYPE_CHECKING:
    from .dataset_loader import ReplayFrame


class VideoDecodingError(RuntimeError):
    """视频解码错误（用于日志报告，不中断回放）"""
    pass


def load_images_for_episode(
    dataset_root: Path,
    episode_index: int,
    frames: tuple[ReplayFrame, ...],
) -> dict[str, list[np.ndarray]]:
    """
    适配 LeRobot 视频解码到产品接口

    Args:
        dataset_root: 数据集根目录
        episode_index: Episode 索引
        frames: 已加载的关节帧（用于提取 timestamp）

    Returns:
        dict[str, list[np.ndarray]]: {"front": [...], "wrist": [...]}

    Raises:
        VideoDecodingError: 视频文件不存在、解码失败、或帧数不匹配（调用方应捕获并记录日志）
    """
    # 读取 LeRobot episode 元数据
    episodes_meta_path = dataset_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    if not episodes_meta_path.exists():
        raise VideoDecodingError(f"Episode metadata not found: {episodes_meta_path}")

    episodes_meta = pd.read_parquet(episodes_meta_path)
    ep_rows = episodes_meta[episodes_meta["episode_index"] == episode_index]
    if ep_rows.empty:
        raise VideoDecodingError(f"Episode {episode_index} not found in metadata")

    ep_row = ep_rows.iloc[0]

    # 构建视频路径（LeRobot 约定）
    video_paths = {
        "front": dataset_root / f"videos/observation.images.front/chunk-{ep_row['videos/observation.images.front/chunk_index']:03d}/file-{ep_row['videos/observation.images.front/file_index']:03d}.mp4",
        "wrist": dataset_root / f"videos/observation.images.wrist/chunk-{ep_row['videos/observation.images.wrist/chunk_index']:03d}/file-{ep_row['videos/observation.images.wrist/file_index']:03d}.mp4",
    }

    # 提取时间戳
    timestamps = [frame.timestamp for frame in frames]

    # 调用 LeRobot 解码
    images: dict[str, list[np.ndarray]] = {}
    for camera_name, video_path in video_paths.items():
        if not video_path.exists():
            raise VideoDecodingError(f"Video file not found: {video_path}")

        try:
            # LeRobot API 调用
            from lerobot.datasets.video_utils import decode_video_frames

            frames_tensor = decode_video_frames(
                video_path=video_path,
                timestamps=timestamps,
                tolerance_s=0.01,
                backend="pyav",
            )

            # 转换为标准 numpy (CHW → HWC, torch → numpy)
            images[camera_name] = [
                frame.permute(1, 2, 0).numpy()  # CHW → HWC
                for frame in frames_tensor
            ]

        except Exception as e:
            raise VideoDecodingError(
                f"Failed to decode video {video_path}: {e}"
            ) from e

    # 验证图像帧数与关节帧数一致
    for camera_name, image_list in images.items():
        if len(image_list) != len(frames):
            raise VideoDecodingError(
                f"Image-joint frame count mismatch for {camera_name}: "
                f"got {len(image_list)} images but {len(frames)} joint frames. "
                f"This may indicate dataset corruption or incomplete recording."
            )

    return images
