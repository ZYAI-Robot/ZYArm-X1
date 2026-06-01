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

## 源码结构

`src/lerobot_robot_zyarm/` 下的文件职责如下：

| 文件 | 作用 |
| --- | --- |
| `__init__.py` | 对外导出 ZYArm follower、leader 和配置类，使 LeRobot 插件注册后可以通过 `--robot.type=zyarm_follower`、`--teleop.type=zyarm_leader` 使用。 |
| `config.py` | 定义 LeRobot 配置类，包括 follower 串口、相机、状态新鲜度、slave filter、leader 频率、启动超时和重定向参数。 |
| `features.py` | 定义 LeRobot dataset/action/observation 使用的稳定特征名和 shape，例如 7 个 `joint*.pos` 以及相机 observation。 |
| `conversion.py` | 在 LeRobot action/observation dict 和 SDK 内部关节位置数组之间做转换。 |
| `robot.py` | 实现 `ZyArmFollowerRobot`，负责 follower 连接、相机连接、状态读取、进入 slave filter、`get_observation()` 和 `send_action()`。 |
| `teleoperator.py` | 实现 `ZyArmLeaderTeleoperator`，负责 leader 连接、启动首帧等待、episode 起点刷新、`get_action()` 和 leader 到 follower 的动作重定向。 |
| `recording.py` | 实现 `zyarm-record` 使用的录制编排，复用 LeRobot dataset/camera/policy/encoder，但由本包控制 pre-roll、active recording loop、reset follow 和 episode 保存顺序。 |
| `cli.py` | `zyarm-record` 控制台入口，负责设置 Windows 控制台编码并调用 `recording.main()`。 |
| `profile.py` | 默认关闭的实机数据质量 profiler，用代码内开关采样 20ms/50Hz 热路径耗时、新鲜度、主从帧率和相机读取耗时。 |

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
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 50}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 50}}" \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

## 数据采集

产品采集推荐使用 `zyarm-record`，不要直接运行 LeRobot 原生 `lerobot-record`。`zyarm-record` 不是重新实现数据采集系统；它仍然复用 LeRobot 的 dataset、相机、策略、processor、视频编码和保存能力，只是把 ZYArm 必须定制的 pre-roll、active recording loop、reset follow 和 profiling 编排放在本包内，避免修改 LeRobot 源码。

```bash
zyarm-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 50}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 50}}" \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --dataset.repo_id=<user>/zyarm_demo \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=60 \
  --dataset.fps=50 \
  --dataset.single_task="Pick up the object" \
  --dataset.vcodec=h264 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --display_data=true
```

`zyarm-record` 的采集阶段按 `get_observation()` -> `get_action()` -> `send_action()` -> `dataset.add_frame()` 执行；episode 之间的 reset 阶段不写 dataset，但会继续执行 `teleop.get_action()` -> `robot.send_action()`，保持主从臂跟随控制在线。

`zyarm-record` 默认 `--dataset.fps=50`，对应 20ms 采集周期，这是当前和 ZYArm 固件、主从臂链路及已验证数据质量匹配的产品默认值。示例中显式写出 `--dataset.fps=50` 是为了强调推荐采集频率；用户仍可按相机能力、任务需求或机器性能显式覆盖为其他 FPS。

产品默认实时采集编码使用 `--dataset.vcodec=h264`，这是当前已经验证过的稳定路径。`--dataset.vcodec=auto` 仍然保留给用户显式选择：选择 `auto` 时会走 LeRobot/FFmpeg 原生逻辑自动搜索硬件编码；用户也可以显式指定 `h264_nvenc`、`h264_qsv`、`libsvtav1` 等编码器。如果明确使用 `--dataset.vcodec=libsvtav1`，建议搭配 `--dataset.streaming_encoding=false`，让 AV1 软件编码发生在 episode 结束后的 save 阶段，避免把重编码压力放进 50Hz 采集 loop。

如果画面或控制周期明显卡顿，优先降低相机分辨率、降低 fps、减少 `encoder_threads`、关闭 `--display_data`，或关闭 streaming encoding。不要通过放宽 `--robot.state_max_age_ms`、`--teleop.action_max_age_ms` 或 `--teleop.wait_timeout_ms` 掩盖实时性问题。

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

策略评估同样推荐使用 `zyarm-record`。和人工采集相比，只需要把 teleop 换成 policy：

```bash
zyarm-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=<user>/zyarm_eval \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=60 \
  --dataset.fps=50 \
  --dataset.single_task="Evaluate the trained policy" \
  --dataset.vcodec=h264 \
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
| `teleop.startup_timeout_ms` | `1000.0` | leader 进入 master mode 后等待首帧 master-data 的启动超时 |

## 安全注意事项

- 首次运行先拆除负载，确认急停或断电方式可用。
- 确认 leader 和 follower 串口没有接反。
- 不要在机械臂接近限位、碰撞物体或人员时回放数据集。
- `send_action()` 走 `zyarm_sdk.fast_io()` 非阻塞下发，follower 端固定由固件 slave filter 平滑执行。
