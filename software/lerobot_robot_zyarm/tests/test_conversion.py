import pytest

from lerobot_robot_zyarm.conversion import action_to_positions, positions_to_action
from lerobot_robot_zyarm.features import ACTION_KEYS, joint_features, observation_features


def test_joint_features_are_stable() -> None:
    assert tuple(joint_features()) == ACTION_KEYS
    assert ACTION_KEYS[0] == "joint0.pos"
    assert ACTION_KEYS[-1] == "joint6.pos"


def test_action_conversion_uses_fixed_order() -> None:
    action = {key: index for index, key in enumerate(ACTION_KEYS)}
    assert action_to_positions(action) == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 1.0]
    assert positions_to_action([0, 1, 2, 3, 4, 5, 0.5])["joint6.pos"] == 0.5
    assert positions_to_action([0, 1, 2, 3, 4, 5, -0.5])["joint6.pos"] == 0.0


def test_action_conversion_rejects_missing_keys() -> None:
    with pytest.raises(ValueError, match="Missing zyarm action keys"):
        action_to_positions({"joint0.pos": 0.0})


def test_observation_features_include_cameras() -> None:
    class CameraConfig:
        width = 640
        height = 480

    features = observation_features({"front": CameraConfig()})
    assert features["joint0.pos"] is float
    assert features["front"] == (480, 640, 3)
