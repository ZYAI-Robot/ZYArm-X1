# zyarm_dataset_replay

用于回放 LeRobot 数据集并在 RViz 中可视化，验证数据质量。

## 功能

同步回放关节状态和相机图像（front/wrist），在 RViz 中显示机械臂姿态与图像，用于数据质量检查。

## 环境依赖

**ROS 2 依赖**：
- ROS 2 Jazzy
- `sensor_msgs`, `visualization_msgs`
- `robot_state_publisher`

**Python 依赖**：
- `lerobot` (用于解码 h264 视频)
- `pandas`, `pyarrow` (读取 parquet 数据)

安装 lerobot：
```bash
# 在 Python 虚拟环境中
pip install lerobot
```

## 话题

质检回放会发布以下话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 机械臂关节状态 |
| `/replay/front/image_raw` | `sensor_msgs/Image` | front 相机图像 (rgb8) |
| `/replay/wrist/image_raw` | `sensor_msgs/Image` | wrist 相机图像 (rgb8) |
| `/replay/frame_marker` | `visualization_msgs/MarkerArray` | 当前帧号标记 |

## 启动

```bash
# 启动质检回放（自动打开 RViz）
ros2 launch zyarm_dataset_replay dataset_quality_check.launch.py \
  dataset_root:=/path/to/your/dataset \
  episode_index:=0

示例：
ros2 launch zyarm_dataset_replay dataset_quality_check.launch.py     dataset_root:=/media/sjy/Windows/ZY_ZYArm/zyarmv1/data/1     episode_index:=0
```

**参数说明**：
- `dataset_root`: LeRobot 数据集根目录（包含 `meta/` 和 `videos/` 子目录）
- `episode_index`: 要回放的 episode 索引（默认 0）

## 测试

```bash
# 构建包
cd software/ros2_ws
colcon build --packages-select zyarm_dataset_replay

# 运行测试
colcon test --packages-select zyarm_dataset_replay

# 查看测试结果
colcon test-result --verbose
```
