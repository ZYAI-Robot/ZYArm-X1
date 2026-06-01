from pathlib import Path
from unittest.mock import MagicMock, patch

from zyarm_dataset_replay.dataset_loader import EpisodeData, ReplayFrame, load_episode_for_replay


def test_load_episode_for_replay_without_images():
    """验证 load_images=False 时不加载图像"""
    mock_episode_data = MagicMock()
    mock_episode_data.frames = (
        ReplayFrame(timestamp=0.0, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ReplayFrame(timestamp=0.02, joints=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)),
    )

    with patch("zyarm_dataset_replay.dataset_loader._load_episode_base", return_value=mock_episode_data):
        result = load_episode_for_replay(
            dataset_root=Path("/fake/dataset"),
            episode_index=0,
            load_images=False,
        )

    assert isinstance(result, EpisodeData)
    assert result.episode_index == 0
    assert len(result.frames) == 2
    assert result.images == {}


def test_load_episode_for_replay_with_images():
    """验证 load_images=True 时加载图像"""
    import numpy as np

    mock_episode_data = MagicMock()
    mock_episode_data.frames = (
        ReplayFrame(timestamp=0.0, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    )

    mock_images = {
        "front": [np.zeros((480, 640, 3), dtype=np.uint8)],
        "wrist": [np.zeros((480, 640, 3), dtype=np.uint8)],
    }

    with patch("zyarm_dataset_replay.dataset_loader._load_episode_base", return_value=mock_episode_data):
        with patch(
            "zyarm_dataset_replay.dataset_loader._load_images",
            return_value=mock_images,
        ):
            result = load_episode_for_replay(
                dataset_root=Path("/fake/dataset"),
                episode_index=0,
                load_images=True,
            )

    assert isinstance(result, EpisodeData)
    assert result.episode_index == 0
    assert len(result.frames) == 1
    assert "front" in result.images
    assert "wrist" in result.images
    assert result.images["front"][0].shape == (480, 640, 3)
