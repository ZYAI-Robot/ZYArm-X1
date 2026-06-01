from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[5]
NODE_PATH = (
    REPO_ROOT
    / "software"
    / "ros2_ws"
    / "src"
    / "zyarm_dataset_replay"
    / "zyarm_dataset_replay"
    / "replay_node.py"
)


@pytest.fixture(scope="module")
def replay_module():
    spec = spec_from_file_location("zyarm_dataset_replay.replay_node", NODE_PATH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_load_episode_reads_observation_state_and_timestamps(replay_module, monkeypatch):
    dataset_root = REPO_ROOT / "data" / "zyarm_demo"

    def fake_read_episode_rows(_parquet_files, _episode_index):
        rows = []
        base = 10.0
        for idx in range(12):
            rows.append(
                {
                    "timestamp": base + idx * 0.02,
                    "observation.state": [0.1 * idx] * 7,
                }
            )
        return rows

    monkeypatch.setattr(replay_module, "_read_episode_rows", fake_read_episode_rows)
    episode = replay_module.load_episode(dataset_root=dataset_root, episode_index=0)

    assert episode.episode_index == 0
    assert len(episode.frames) == 12
    assert len(episode.frames[0].joints) == 7


def test_build_trajectory_keeps_raw_joint_values(replay_module, monkeypatch):
    dataset_root = REPO_ROOT / "data" / "zyarm_demo"

    def fake_read_episode_rows(_parquet_files, _episode_index):
        return [
            {"timestamp": 1.00, "observation.state": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]},
            {"timestamp": 1.02, "observation.state": [1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 7.1]},
            {"timestamp": 1.04, "observation.state": [1.2, 2.2, 3.2, 4.2, 5.2, 6.2, 7.2]},
            {"timestamp": 1.06, "observation.state": [1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]},
            {"timestamp": 1.08, "observation.state": [1.4, 2.4, 3.4, 4.4, 5.4, 6.4, 7.4]},
        ]

    monkeypatch.setattr(replay_module, "_read_episode_rows", fake_read_episode_rows)
    episode = replay_module.load_episode(dataset_root=dataset_root, episode_index=0)
    chunk = episode.frames[:5]

    traj = replay_module._build_trajectory(
        replay_module.ARM_JOINT_NAMES,
        chunk,
        range(0, 6),
    )

    for frame, point in zip(chunk, traj.points):
        assert list(point.positions) == list(frame.joints[:6])


def test_split_frames_creates_ordered_chunks(replay_module, monkeypatch):
    dataset_root = REPO_ROOT / "data" / "zyarm_demo"

    def fake_read_episode_rows(_parquet_files, _episode_index):
        rows = []
        base = 2.0
        for idx in range(30):
            rows.append({"timestamp": base + idx * 0.02, "observation.state": [0.0] * 7})
        return rows

    monkeypatch.setattr(replay_module, "_read_episode_rows", fake_read_episode_rows)
    episode = replay_module.load_episode(dataset_root=dataset_root, episode_index=0)

    chunks = replay_module._split_frames(episode.frames, chunk_duration_sec=0.2)

    assert len(chunks) > 1
    assert chunks[0][0].timestamp <= chunks[0][-1].timestamp
    assert chunks[-1][0].timestamp <= chunks[-1][-1].timestamp


def test_validate_monotonic_rejects_non_monotonic(replay_module):
    frames = (
        replay_module.ReplayFrame(timestamp=0.0, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        replay_module.ReplayFrame(timestamp=0.01, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        replay_module.ReplayFrame(timestamp=0.01, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    )

    with pytest.raises(replay_module.DatasetValidationError):
        replay_module._validate_monotonic_timestamps(frames)


def test_resolve_dataset_root_from_repo_and_id(replay_module):
    resolved = replay_module.resolve_dataset_root("", "C:/tmp/repo", "subset")
    assert str(resolved).replace("\\", "/").endswith("/tmp/repo/subset")


def test_validate_chunk_boundaries_rejects_overlap(replay_module):
    a = replay_module.ReplayFrame(timestamp=0.00, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    b = replay_module.ReplayFrame(timestamp=0.10, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    c = replay_module.ReplayFrame(timestamp=0.10, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    with pytest.raises(replay_module.DatasetValidationError):
        replay_module._validate_chunk_boundaries(((a, b), (c,)))


def test_extract_frames_rejects_missing_required_fields(replay_module):
    rows = [{"timestamp": 0.0}]
    with pytest.raises(replay_module.DatasetValidationError, match="missing observation.state"):
        replay_module._extract_frames(rows)


def test_validate_50hz_baseline_rejects_mismatch(replay_module):
    frames = (
        replay_module.ReplayFrame(timestamp=0.00, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        replay_module.ReplayFrame(timestamp=0.02, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        replay_module.ReplayFrame(timestamp=0.08, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    )
    with pytest.raises(replay_module.DatasetValidationError, match="50Hz baseline mismatch"):
        replay_module._validate_50hz_baseline(frames)


def test_build_trajectory_arm_and_gripper_pack_rule(replay_module):
    frames = (
        replay_module.ReplayFrame(timestamp=1.00, joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        replay_module.ReplayFrame(timestamp=1.02, joints=(1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 7.1)),
    )
    arm = replay_module._build_trajectory(replay_module.ARM_JOINT_NAMES, frames, range(0, 6))
    gripper = replay_module._build_trajectory(replay_module.GRIPPER_JOINT_NAMES, frames, [6])

    assert list(arm.points[0].positions) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert list(gripper.points[0].positions) == [7.0]
    assert arm.points[1].time_from_start.nanosec == gripper.points[1].time_from_start.nanosec


def test_replay_stops_after_failed_chunk(replay_module):
    chunk0 = (
        replay_module.ReplayFrame(timestamp=0.00, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        replay_module.ReplayFrame(timestamp=0.02, joints=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)),
    )
    chunk1 = (
        replay_module.ReplayFrame(timestamp=0.04, joints=(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)),
        replay_module.ReplayFrame(timestamp=0.06, joints=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3)),
    )

    calls = []

    class DummyNode:
        def __init__(self):
            self._arm_client = object()
            self._gripper_client = object()
            self._gripper_normalized_input = False
            self._gripper_travel_m = 0.034

        def _send_goal(self, client, trajectory, chunk_index, controller_name):
            calls.append((chunk_index, controller_name, "send"))
            if chunk_index == 1 and controller_name == "arm_controller":
                raise RuntimeError("chunk 1: arm_controller failed")
            return object(), 0  # 返回 (future, accept_time)

        def _wait_goal_result(self, result_future, accept_time, chunk_index, controller_name):
            calls.append((chunk_index, controller_name, "wait"))

        class _Logger:
            def info(self, _msg):
                return None

        def get_logger(self):
            return self._Logger()

        class _Clock:
            def now(self):
                class _Time:
                    nanoseconds = 0
                return _Time()

        def get_clock(self):
            return self._Clock()

    dummy = DummyNode()
    with pytest.raises(RuntimeError, match="chunk 1"):
        replay_module.DatasetReplayNode._replay_chunks(dummy, (chunk0, chunk1))

    assert calls == [
        (0, "arm_controller", "send"),
        (0, "gripper_controller", "send"),
        (0, "arm_controller", "wait"),
        (0, "gripper_controller", "wait"),
        (1, "arm_controller", "send"),
    ]


def test_apply_gripper_mapping_scales_joint6_when_enabled(replay_module):
    frames = (
        replay_module.ReplayFrame(timestamp=0.00, joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5)),
        replay_module.ReplayFrame(timestamp=0.02, joints=(1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 0.8)),
    )
    mapped = replay_module._apply_gripper_mapping(frames, gripper_normalized_input=True, gripper_travel_m=0.034)

    assert len(mapped) == 2
    assert mapped[0].joints[6] == pytest.approx(0.5 * 0.034)
    assert mapped[1].joints[6] == pytest.approx(0.8 * 0.034)
    # arm joints 不变
    assert mapped[0].joints[:6] == frames[0].joints[:6]
    assert mapped[1].joints[:6] == frames[1].joints[:6]


def test_apply_gripper_mapping_passthrough_when_disabled(replay_module):
    frames = (
        replay_module.ReplayFrame(timestamp=0.00, joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.025)),
        replay_module.ReplayFrame(timestamp=0.02, joints=(1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 0.030)),
    )
    mapped = replay_module._apply_gripper_mapping(frames, gripper_normalized_input=False, gripper_travel_m=0.034)

    assert len(mapped) == 2
    assert mapped[0].joints[6] == 0.025  # 不变
    assert mapped[1].joints[6] == 0.030  # 不变
    assert mapped[0].joints[:6] == frames[0].joints[:6]
    assert mapped[1].joints[:6] == frames[1].joints[:6]

