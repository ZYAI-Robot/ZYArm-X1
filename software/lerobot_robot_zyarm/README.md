# lerobot_robot_zyarm

`lerobot_robot_zyarm` 是 zyarm 面向 LeRobot 的原生适配插件。它不依赖 ROS 2，也不维护 LeRobot 子仓；机械臂通讯由 `zyarm_sdk` 直接访问串口完成。

```text
LeRobot 命令
  -> lerobot_robot_zyarm
  -> zyarm_sdk Python
  -> 串口 / 固件
```

这条路径主要用于主从臂遥操、LeRobot 数据采集、回放和策略评估。MoveIt、ros2_control 和 `zyarm_hardware_interface` 仍然走 ROS 2 路径，两条路径互不依赖。

## 支持版本

第一版固定支持 PyPI 正式版：

```bash
pip install lerobot==0.5.1
```

后续升级 LeRobot 时，需要先验证 Robot/Teleoperator API、camera 配置、record/replay 命令和 dataset 写入行为，再更新这里的支持版本。

## 安装

在仓库根目录执行：

```bash
pip install lerobot==0.5.1
pip install -e software/zyarm_sdk/python
pip install -e software/lerobot_robot_zyarm
```

安装后可以用下面的命令确认插件能被导入：

```bash
python -c "import lerobot_robot_zyarm; print('zyarm LeRobot plugin ok')"
```

## 硬件连接

典型连接方式：

- leader 主臂：接到一个独立串口，例如 Linux `/dev/ttyUSB0`，Windows `COM3`。
- follower 从臂：接到另一个独立串口，例如 Linux `/dev/ttyUSB1`，Windows `COM4`。
- 相机：由 LeRobot camera 配置管理，常用双摄像头为 `front` 和 `wrist`，例如 OpenCV camera 的 `index_or_path: 0`、`index_or_path: 1`。

默认波特率为 `230400`，需要和固件串口配置保持一致。Linux 下如果串口没有权限，可先把当前用户加入 `dialout` 组并重新登录。

## 单位约定

插件对 LeRobot 暴露 7 个稳定特征：

```text
joint0.pos
joint1.pos
joint2.pos
joint3.pos
joint4.pos
joint5.pos
joint6.pos
```

其中 `joint0.pos` 到 `joint5.pos` 是 SDK/ROS 公共角度表达，单位为弧度，初始姿态附近为 0；它不是固件内部舵机角度。`joint6.pos` 是夹爪归一化位置，范围为 `0.0..1.0`。

## 遥操

先空载、低速确认 leader/follower 串口没有接反，再运行：

```bash
lerobot-teleoperate \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

如果要同时看双摄像头画面，可以给 follower 增加 LeRobot camera 配置：

```bash
lerobot-teleoperate \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

## 数据采集

采集时由 LeRobot 负责 dataset、视频编码、Rerun 可视化和 episode 管理，zyarm 插件只提供 robot/teleoperator 适配：

```bash
lerobot-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --dataset.repo_id=<user>/zyarm_demo \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=60 \
  --dataset.single_task="Pick up the object" \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --display_data=true
```

如果画面或控制周期明显卡顿，优先降低相机分辨率、降低 fps、减少 `encoder_threads`，或关闭 `--display_data`。

## 回放

回放会读取 dataset 中保存的 action，并发送给 follower：

```bash
lerobot-replay \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --dataset.repo_id=<user>/zyarm_demo \
  --dataset.episode=0
```

首次回放建议空载、远离限位区域，并随时准备断电。

## 策略评估

LeRobot 使用 `lerobot-record` 运行策略评估。和人工采集相比，只需要把 teleop 换成 policy：

```bash
lerobot-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=<user>/zyarm_eval \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=60 \
  --dataset.single_task="Evaluate the trained policy" \
  --policy.path=<user>/zyarm_policy \
  --display_data=true
```

## 常用配置

LeRobot follower 连接后会固定进入固件 slave filter 模式，并在该模式下使用 `CMD36 fast_io` 下发动作。数据采集、回放和策略评估都使用同一条滤波控制路径；当前插件不再保留 raw fast_io 模式。

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `robot.port` | `""` | follower 串口 |
| `robot.baudrate` | `230400` | follower 波特率 |
| `robot.state_max_age_ms` | `100.0` | observation 状态缓存最大年龄 |
| `robot.slave_filter_lpf_alpha` | `0.15` | follower slave filter 的低通滤波系数 |
| `robot.cameras` | `{}` | LeRobot camera 配置 |
| `teleop.port` | `""` | leader 串口 |
| `teleop.baudrate` | `230400` | leader 波特率 |
| `teleop.leader_hz` | `50.0` | leader 动作读取频率 |

## 安全注意事项

- 首次运行先拆除负载，确认急停或断电方式可用。
- 确认 leader 和 follower 串口没有接反。
- 不要在机械臂接近限位、碰撞物体或人员时回放数据集。
- `send_action()` 走 `zyarm_sdk.fast_io()` 非阻塞下发，follower 端固定由固件 slave filter 平滑执行。
