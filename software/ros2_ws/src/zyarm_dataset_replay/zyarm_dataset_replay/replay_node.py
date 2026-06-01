from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Sequence, TYPE_CHECKING

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    import numpy as np
    from sensor_msgs.msg import Image
    from visualization_msgs.msg import Marker, MarkerArray

JOINT_NAMES = tuple(f"joint{i}" for i in range(7))
ARM_JOINT_NAMES = JOINT_NAMES[:6]
GRIPPER_JOINT_NAMES = JOINT_NAMES[6:]
EXPECTED_HZ = 50.0
EXPECTED_DT = 1.0 / EXPECTED_HZ
MAX_EPISODE_DURATION_FOR_SINGLE_GOAL = 60.0  # 超过此秒数则回退到分块模式
DEFAULT_GRIPPER_TRAVEL_M = 0.034


@dataclass(frozen=True)
class ReplayFrame:
    timestamp: float
    joints: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class EpisodeReplayData:
    episode_index: int
    frames: tuple[ReplayFrame, ...]


class DatasetValidationError(RuntimeError):
    pass


def _load_json(path: Path) -> dict:
    if not path.is_file():
        raise DatasetValidationError(f"missing required file: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _resolve_data_files(dataset_root: Path, info: dict) -> tuple[Path, ...]:
    data_path_pattern = info.get("data_path")
    if not isinstance(data_path_pattern, str):
        raise DatasetValidationError("meta/info.json missing data_path")

    pattern = Path(data_path_pattern)
    anchor = "chunk-{"
    data_root = dataset_root / pattern.parent
    if anchor in str(pattern.parent):
        parent_text = str(pattern.parent)
        data_root = dataset_root / Path(parent_text[: parent_text.index(anchor)].rstrip("/"))

    if not data_root.is_dir():
        raise DatasetValidationError(f"data directory not found: {data_root}")
    files = tuple(sorted(data_root.glob("chunk-*/file-*.parquet")))
    if not files:
        raise DatasetValidationError(f"no parquet files found under: {data_root}")
    return files


def _read_episode_rows(parquet_files: Sequence[Path], episode_index: int) -> list[dict]:
    import pandas as pd
    import pyarrow  # noqa: F401

    rows: list[dict] = []
    for file_path in parquet_files:
        frame = pd.read_parquet(file_path)
        if "episode_index" not in frame.columns:
            raise DatasetValidationError(f"missing episode_index column in {file_path}")
        sub = frame[frame["episode_index"] == episode_index]
        if sub.empty:
            continue
        rows.extend(sub.to_dict(orient="records"))
    if not rows:
        raise DatasetValidationError(f"episode {episode_index} has no rows in dataset")
    return rows


def _coerce_state_vector(raw_state: object, row_idx: int) -> tuple[float, ...]:
    if not isinstance(raw_state, Iterable):
        raise DatasetValidationError(f"row {row_idx}: observation.state is not iterable")
    values = tuple(float(v) for v in raw_state)
    if len(values) != 7:
        raise DatasetValidationError(f"row {row_idx}: observation.state must have 7 joints")
    return values


def _extract_frames(rows: Sequence[dict]) -> tuple[ReplayFrame, ...]:
    frames: list[ReplayFrame] = []
    for row_idx, row in enumerate(rows):
        if "timestamp" not in row:
            raise DatasetValidationError(f"row {row_idx}: missing timestamp")
        if "observation.state" not in row:
            raise DatasetValidationError(f"row {row_idx}: missing observation.state")
        timestamp = float(row["timestamp"])
        joints = _coerce_state_vector(row["observation.state"], row_idx)
        frames.append(ReplayFrame(timestamp=timestamp, joints=joints))
    frames.sort(key=lambda item: item.timestamp)
    return tuple(frames)


def _validate_monotonic_timestamps(frames: Sequence[ReplayFrame]) -> None:
    if len(frames) < 2:
        raise DatasetValidationError("episode requires at least 2 frames")
    for idx in range(1, len(frames)):
        prev = frames[idx - 1].timestamp
        curr = frames[idx].timestamp
        if curr <= prev:
            raise DatasetValidationError(
                f"timestamp non-monotonic at index {idx}: {curr} <= {prev}"
            )


def _validate_50hz_baseline(frames: Sequence[ReplayFrame], tolerance_sec: float = 0.005) -> None:
    deltas = [frames[idx].timestamp - frames[idx - 1].timestamp for idx in range(1, len(frames))]
    mismatch = [delta for delta in deltas if abs(delta - EXPECTED_DT) > tolerance_sec]
    if mismatch:
        sample = mismatch[0]
        raise DatasetValidationError(
            f"50Hz baseline mismatch: expected ~{EXPECTED_DT:.4f}s, observed {sample:.4f}s"
        )


def resolve_dataset_root(dataset_root: str, dataset_repo: str, dataset_id: str) -> Path:
    root = Path(dataset_root) if dataset_root else None
    if root and str(root).strip():
        return root

    if not dataset_repo:
        raise DatasetValidationError("dataset_root or dataset_repo is required")

    repo_path = Path(dataset_repo)
    if dataset_id:
        return repo_path / dataset_id
    return repo_path


def load_episode(dataset_root: Path, episode_index: int) -> EpisodeReplayData:
    info = _load_json(dataset_root / "meta" / "info.json")
    _load_json(dataset_root / "meta" / "stats.json")

    features = info.get("features")
    if not isinstance(features, dict):
        raise DatasetValidationError("meta/info.json missing features")
    if "observation.state" not in features:
        raise DatasetValidationError("meta/info.json missing observation.state feature")
    if "timestamp" not in features:
        raise DatasetValidationError("meta/info.json missing timestamp feature")

    parquet_files = _resolve_data_files(dataset_root, info)
    rows = _read_episode_rows(parquet_files, episode_index)
    frames = _extract_frames(rows)
    _validate_monotonic_timestamps(frames)
    _validate_50hz_baseline(frames)
    return EpisodeReplayData(episode_index=episode_index, frames=frames)


def _build_time_axis(frames: Sequence[ReplayFrame]) -> tuple[float, ...]:
    base = frames[0].timestamp
    return tuple(frame.timestamp - base for frame in frames)


def _apply_gripper_mapping(
    frames: Sequence[ReplayFrame],
    gripper_normalized_input: bool,
    gripper_travel_m: float,
) -> tuple[ReplayFrame, ...]:
    if not gripper_normalized_input:
        return tuple(frames)

    mapped_frames: list[ReplayFrame] = []
    for frame in frames:
        mapped_joint6 = frame.joints[6] * gripper_travel_m
        mapped_joints = (
            frame.joints[0],
            frame.joints[1],
            frame.joints[2],
            frame.joints[3],
            frame.joints[4],
            frame.joints[5],
            mapped_joint6,
        )
        mapped_frames.append(ReplayFrame(timestamp=frame.timestamp, joints=mapped_joints))
    return tuple(mapped_frames)


def _build_trajectory(
    joint_names: Sequence[str],
    frames: Sequence[ReplayFrame],
    joint_indices: Sequence[int],
    log_info: Callable[[str], None] | None = None,
) -> JointTrajectory:
    timeline = _build_time_axis(frames)
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    points: list[JointTrajectoryPoint] = []
    for t_from_start, frame in zip(timeline, frames):
        point = JointTrajectoryPoint()
        point.positions = [frame.joints[idx] for idx in joint_indices]
        sec = int(t_from_start)
        nsec = int((t_from_start - sec) * 1_000_000_000)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
        points.append(point)
    trajectory.points = points

    if log_info is not None and tuple(joint_names) == GRIPPER_JOINT_NAMES and points:
        gripper_positions = [point.positions[0] for point in points]
        log_info(
            "[GRIPPER DEBUG] trajectory range: "
            f"min={min(gripper_positions):.6f}, max={max(gripper_positions):.6f}"
        )

    return trajectory


def _split_frames(frames: Sequence[ReplayFrame], chunk_duration_sec: float) -> tuple[tuple[ReplayFrame, ...], ...]:
    if chunk_duration_sec <= 0.0:
        raise DatasetValidationError("chunk_duration_sec must be > 0")

    chunks: list[list[ReplayFrame]] = []
    current: list[ReplayFrame] = [frames[0]]
    chunk_start = frames[0].timestamp

    for frame in frames[1:]:
        if frame.timestamp - chunk_start > chunk_duration_sec:
            chunks.append(current)
            current = [frame]
            chunk_start = frame.timestamp
        else:
            current.append(frame)
    chunks.append(current)

    # 边界无缝衔接：每个 chunk 的最后一帧作为下一个 chunk 的起点（重叠帧策略）
    seamless_chunks: list[tuple[ReplayFrame, ...]] = []
    for idx, chunk in enumerate(chunks):
        if idx < len(chunks) - 1:
            # 将下一个 chunk 的第一帧追加到当前 chunk 末尾，确保时间轴连续
            next_first = chunks[idx + 1][0]
            seamless_chunks.append(tuple(chunk) + (next_first,))
        else:
            seamless_chunks.append(tuple(chunk))

    return tuple(seamless_chunks)


def _validate_chunk_boundaries(chunks: Sequence[Sequence[ReplayFrame]]) -> None:
    for idx in range(1, len(chunks)):
        prev_last = chunks[idx - 1][-1]
        curr_first = chunks[idx][0]
        if curr_first.timestamp <= prev_last.timestamp:
            raise DatasetValidationError(
                f"chunk boundary timestamp invalid at index {idx}: {curr_first.timestamp} <= {prev_last.timestamp}"
            )


class DatasetReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("zyarm_dataset_replay")

        self.declare_parameter("dataset_root", "")
        self.declare_parameter("dataset_repo", "")
        self.declare_parameter("dataset_id", "")
        self.declare_parameter("episode_index", 0)
        self.declare_parameter("chunk_duration_sec", 8.0)
        self.declare_parameter("arm_action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action_name", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_normalized_input", True)
        self.declare_parameter("gripper_travel_m", DEFAULT_GRIPPER_TRAVEL_M)
        self.declare_parameter("_quality_check_mode", False)

        dataset_root_param = str(self.get_parameter("dataset_root").value)
        dataset_repo = str(self.get_parameter("dataset_repo").value)
        dataset_id = str(self.get_parameter("dataset_id").value)
        dataset_root = resolve_dataset_root(dataset_root_param, dataset_repo, dataset_id)
        episode_index = int(self.get_parameter("episode_index").value)
        chunk_duration_sec = float(self.get_parameter("chunk_duration_sec").value)
        arm_action_name = str(self.get_parameter("arm_action_name").value)
        gripper_action_name = str(self.get_parameter("gripper_action_name").value)
        gripper_normalized_input = bool(self.get_parameter("gripper_normalized_input").value)
        gripper_travel_m = float(self.get_parameter("gripper_travel_m").value)
        quality_check_mode = bool(self.get_parameter("_quality_check_mode").value)

        if not dataset_root:
            raise DatasetValidationError("dataset_root is required")

        # 质检模式：加载图像数据并创建发布器
        if quality_check_mode:
            from .dataset_loader import load_episode_for_replay, load_images_for_replay_best_effort
            from sensor_msgs.msg import Image, JointState
            from visualization_msgs.msg import Marker, MarkerArray

            self.get_logger().info("Quality check mode enabled: loading images and creating publishers")
            episode_data = load_episode_for_replay(dataset_root, episode_index, load_images=False)

            self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self._front_image_pub = self.create_publisher(Image, "/replay/front/image_raw", 10)
            self._wrist_image_pub = self.create_publisher(Image, "/replay/wrist/image_raw", 10)
            self._marker_pub = self.create_publisher(MarkerArray, "/replay/frame_marker", 10)

            replay_data = EpisodeReplayData(
                episode_index=episode_data.episode_index,
                frames=tuple(episode_data.frames)
            )
            self._episode_images = load_images_for_replay_best_effort(
                dataset_root=dataset_root,
                episode_index=episode_index,
                frames=replay_data.frames,
                log_error=self.get_logger().error,
            )
        else:
            replay_data = load_episode(dataset_root=dataset_root, episode_index=episode_index)
            self._episode_images = None

        episode_duration = replay_data.frames[-1].timestamp - replay_data.frames[0].timestamp
        use_single_goal = episode_duration <= MAX_EPISODE_DURATION_FOR_SINGLE_GOAL

        self.get_logger().info(
            "Raw Replay enabled: node forwards observation.state values without denoise/smoothing/interpolation/resampling. "
            "Execution-layer behavior is controlled by existing ros2_control controllers."
        )

        if use_single_goal:
            self.get_logger().info(
                f"Episode duration {episode_duration:.1f}s <= {MAX_EPISODE_DURATION_FOR_SINGLE_GOAL}s: using single-goal mode for seamless playback."
            )
        else:
            self.get_logger().info(
                f"Episode duration {episode_duration:.1f}s > {MAX_EPISODE_DURATION_FOR_SINGLE_GOAL}s: falling back to chunked mode with duration {chunk_duration_sec}s."
            )

        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action_name)
        self._gripper_client = ActionClient(self, FollowJointTrajectory, gripper_action_name)

        if not quality_check_mode:
            if not self._arm_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"arm action server not ready: {arm_action_name}")
            if not self._gripper_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"gripper action server not ready: {gripper_action_name}")

        self._gripper_normalized_input = gripper_normalized_input
        self._gripper_travel_m = gripper_travel_m
        self._quality_check_mode = quality_check_mode

        if quality_check_mode:
            self._replay_quality_check(replay_data.frames)
        elif use_single_goal:
            self._replay_single_goal(replay_data.frames)
        else:
            chunks = _split_frames(replay_data.frames, chunk_duration_sec)
            _validate_chunk_boundaries(chunks)
            self._replay_chunks(chunks)

    def _replay_quality_check(self, frames: Sequence[ReplayFrame]) -> None:
        """质检模式：逐帧发布 joint_states + 图像 + marker，按 timestamp 驱动节奏"""
        from sensor_msgs.msg import JointState
        import time

        self.get_logger().info(f"Quality check replay started with {len(frames)} frames")

        mapped_frames = _apply_gripper_mapping(frames, self._gripper_normalized_input, self._gripper_travel_m)

        start_wall_time = time.time()
        start_dataset_time = mapped_frames[0].timestamp

        for frame_index, frame in enumerate(mapped_frames):
            # 计算目标发布时间（相对于回放开始时间）
            target_dataset_time = frame.timestamp - start_dataset_time
            elapsed_wall_time = time.time() - start_wall_time
            sleep_duration = target_dataset_time - elapsed_wall_time

            if sleep_duration > 0:
                time.sleep(sleep_duration)

            # 发布 joint_states
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self._timestamp_to_ros_time(frame.timestamp)
            joint_state_msg.header.frame_id = ""
            joint_state_msg.name = list(ARM_JOINT_NAMES) + list(GRIPPER_JOINT_NAMES)
            joint_state_msg.position = list(frame.joints)
            self._joint_state_pub.publish(joint_state_msg)

            # 发布图像和 marker
            self._publish_quality_check_data(frame_index, frame)

            if (frame_index + 1) % 50 == 0:
                self.get_logger().info(f"Quality check: published frame {frame_index + 1}/{len(mapped_frames)}")

        self.get_logger().info(f"Quality check replay completed with {len(mapped_frames)} frames")

    def _send_goal(
        self, client: ActionClient, trajectory: JointTrajectory, chunk_index: int, controller_name: str
    ):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_time_ns = self.get_clock().now().nanoseconds
        send_future = client.send_goal_async(goal)
        self._spin_until_future_done(send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"chunk {chunk_index}: {controller_name} goal rejected")

        accept_time_ns = self.get_clock().now().nanoseconds
        accept_delay_ms = (accept_time_ns - send_time_ns) / 1_000_000.0
        self.get_logger().info(
            f"chunk {chunk_index}: {controller_name} goal accepted in {accept_delay_ms:.2f} ms"
        )

        result_future = goal_handle.get_result_async()
        return result_future, accept_time_ns

    def _wait_goal_result(self, result_future, accept_time_ns: int, chunk_index: int, controller_name: str) -> None:
        self._spin_until_future_done(result_future)
        wrapped_result = result_future.result()
        complete_time_ns = self.get_clock().now().nanoseconds
        execution_time_ms = (complete_time_ns - accept_time_ns) / 1_000_000.0

        if wrapped_result is None:
            raise RuntimeError(f"chunk {chunk_index}: {controller_name} missing action result")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"chunk {chunk_index}: {controller_name} failed with status {wrapped_result.status}"
            )

        self.get_logger().info(
            f"chunk {chunk_index}: {controller_name} execution completed in {execution_time_ms:.2f} ms"
        )

    def _replay_single_goal(self, frames: Sequence[ReplayFrame]) -> None:
        joint6_values = [frame.joints[6] for frame in frames]
        self.get_logger().info(
            "[GRIPPER MAPPING] dataset joint6 range (before mapping): "
            f"min={min(joint6_values):.6f}, max={max(joint6_values):.6f}, "
            f"mean={sum(joint6_values)/len(joint6_values):.6f}"
        )

        mapped_frames = _apply_gripper_mapping(frames, self._gripper_normalized_input, self._gripper_travel_m)

        if self._gripper_normalized_input:
            mapped_joint6_values = [frame.joints[6] for frame in mapped_frames]
            self.get_logger().info(
                f"[GRIPPER MAPPING] joint6 after normalized->meters mapping (travel={self._gripper_travel_m}m): "
                f"min={min(mapped_joint6_values):.6f}, max={max(mapped_joint6_values):.6f}"
            )

        arm_traj = _build_trajectory(ARM_JOINT_NAMES, mapped_frames, range(0, 6), self.get_logger().info)
        gripper_traj = _build_trajectory(GRIPPER_JOINT_NAMES, mapped_frames, [6], self.get_logger().info)

        arm_result_future, arm_accept_time = self._send_goal(self._arm_client, arm_traj, 0, "arm_controller")
        gripper_result_future, gripper_accept_time = self._send_goal(self._gripper_client, gripper_traj, 0, "gripper_controller")

        self._wait_goal_result(arm_result_future, arm_accept_time, 0, "arm_controller")
        self._wait_goal_result(gripper_result_future, gripper_accept_time, 0, "gripper_controller")

        self.get_logger().info(f"single-goal replay completed with {len(frames)} frames")

    def _replay_chunks(self, chunks: Sequence[Sequence[ReplayFrame]]) -> None:
        chunk_start_wall_time = None
        for chunk_index, chunk in enumerate(chunks):
            mapped_chunk = _apply_gripper_mapping(chunk, self._gripper_normalized_input, self._gripper_travel_m)
            arm_traj = _build_trajectory(ARM_JOINT_NAMES, mapped_chunk, range(0, 6), self.get_logger().info)
            gripper_traj = _build_trajectory(GRIPPER_JOINT_NAMES, mapped_chunk, [6], self.get_logger().info)

            now_ns = self.get_clock().now().nanoseconds
            if chunk_start_wall_time is not None:
                boundary_delay_ms = (now_ns - chunk_start_wall_time) / 1_000_000.0
                self.get_logger().info(
                    f"chunk {chunk_index + 1}/{len(chunks)} boundary delay: {boundary_delay_ms:.2f} ms"
                )

            arm_result_future, arm_accept_time = self._send_goal(
                self._arm_client, arm_traj, chunk_index, "arm_controller"
            )
            gripper_result_future, gripper_accept_time = self._send_goal(
                self._gripper_client,
                gripper_traj,
                chunk_index,
                "gripper_controller",
            )

            self._wait_goal_result(arm_result_future, arm_accept_time, chunk_index, "arm_controller")
            self._wait_goal_result(gripper_result_future, gripper_accept_time, chunk_index, "gripper_controller")

            chunk_end_wall_time = self.get_clock().now().nanoseconds
            chunk_start_wall_time = chunk_end_wall_time
            self.get_logger().info(
                f"chunk {chunk_index + 1}/{len(chunks)} replayed with {len(chunk)} frames"
            )

    def _spin_until_future_done(self, future) -> None:
        import rclpy

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _publish_quality_check_data(self, frame_index: int, frame: ReplayFrame) -> None:
        """在质检模式下发布图像和帧标记"""
        if self._episode_images is None:
            return

        from sensor_msgs.msg import Image
        from visualization_msgs.msg import Marker, MarkerArray
        from builtin_interfaces.msg import Time

        # 发布 front 相机图像
        if "front" in self._episode_images and frame_index < len(self._episode_images["front"]):
            front_image = self._episode_images["front"][frame_index]
            front_msg = self._numpy_to_image_msg(front_image, frame.timestamp, "front_camera")
            self._front_image_pub.publish(front_msg)

        # 发布 wrist 相机图像
        if "wrist" in self._episode_images and frame_index < len(self._episode_images["wrist"]):
            wrist_image = self._episode_images["wrist"][frame_index]
            wrist_msg = self._numpy_to_image_msg(wrist_image, frame.timestamp, "wrist_camera")
            self._wrist_image_pub.publish(wrist_msg)

        # 发布帧标记
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = self._timestamp_to_ros_time(frame.timestamp)
        marker.header.frame_id = "base_link"
        marker.ns = "frame_info"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"Frame {frame_index} | t={frame.timestamp:.2f}s"
        marker_array.markers.append(marker)
        self._marker_pub.publish(marker_array)

    def _timestamp_to_ros_time(self, timestamp: float) -> "Time":
        """将浮点时间戳转换为 ROS Time 消息"""
        from builtin_interfaces.msg import Time
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        return Time(sec=sec, nanosec=nanosec)

    def _numpy_to_image_msg(self, image: "np.ndarray", timestamp: float, frame_id: str) -> "Image":
        """手动构造 sensor_msgs/Image（绕过 cv_bridge 的 NumPy ABI 冲突）"""
        from sensor_msgs.msg import Image
        import numpy as np

        # 规范化：float[0,1] → uint8[0,255]
        if image.dtype in (np.float32, np.float64):
            image = (image * 255.0).clip(0, 255).astype(np.uint8)
        elif image.dtype != np.uint8:
            image = image.astype(np.uint8)

        # 规范化：确保 HWC uint8 连续内存
        if image.ndim == 2:
            # 灰度图
            image = np.ascontiguousarray(image, dtype=np.uint8)
            encoding = "mono8"
            channels = 1
        elif image.ndim == 3:
            # 彩色图：CHW → HWC
            if image.shape[0] == 3 or image.shape[0] == 1:
                image = np.transpose(image, (1, 2, 0))
            image = np.ascontiguousarray(image, dtype=np.uint8)
            channels = image.shape[2]
            if channels == 3:
                encoding = "rgb8"
            elif channels == 1:
                encoding = "mono8"
                image = image.squeeze(axis=2)
            else:
                raise ValueError(f"Unsupported channel count: {channels}")
        else:
            raise ValueError(f"Unsupported image shape: {image.shape}")

        msg = Image()
        msg.header.stamp = self._timestamp_to_ros_time(timestamp)
        msg.header.frame_id = frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = image.strides[0] if image.ndim >= 2 else image.shape[0]
        msg.data = image.tobytes()

        # 验证一致性
        expected_size = msg.step * msg.height
        actual_size = len(msg.data)
        if expected_size != actual_size:
            raise RuntimeError(
                f"Image message size mismatch: expected {expected_size} bytes "
                f"(step={msg.step} × height={msg.height}), got {actual_size} bytes"
            )

        return msg


def main() -> None:
    import rclpy

    rclpy.init()
    node = None
    try:
        node = DatasetReplayNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
