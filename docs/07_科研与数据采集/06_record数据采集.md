# record 数据采集

`lerobot-record` 用于把遥操过程保存成 dataset。采集时 LeRobot 负责 dataset、视频编码、显示和 episode 管理，ZYArm 插件负责把 leader 动作和 follower 状态接入 LeRobot。

## 先理解 dataset 和 episode

| 术语 | 含义 |
| --- | --- |
| dataset | 一组采集数据，通常包含多个 episode |
| episode | 一次完整任务过程，例如一次抓取或一次摆放 |
| action | 发送给 follower 的动作 |
| observation | follower 状态和相机画面等观测数据 |
| task description | 当前 episode 的任务描述，用于后续训练或检索 |

正式采集前，建议先录制 1 个很短的 episode，再 replay 验证。

## 采集命令

双臂加双摄像头示例：

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

Windows 端口只需要把 `/dev/ttyUSB0`、`/dev/ttyUSB1` 替换成 `COM3`、`COM4` 这类端口名。

## 参数含义

| 参数 | 含义 |
| --- | --- |
| `--robot.type=zyarm_follower` | 采集 follower 的状态并向 follower 下发动作 |
| `--robot.port` | follower 串口 |
| `--robot.cameras` | 相机配置，决定 dataset 中保存哪些图像 |
| `--teleop.type=zyarm_leader` | 使用 leader 作为人工输入 |
| `--teleop.port` | leader 串口 |
| `--dataset.repo_id` | 数据集名称，示例中用 `<user>/zyarm_demo` 占位 |
| `--dataset.num_episodes` | 采集多少个 episode |
| `--dataset.episode_time_s` | 每个 episode 的最长采集时间 |
| `--dataset.single_task` | 本次采集的任务描述 |
| `--dataset.streaming_encoding` | 边采集边编码视频，减少采集后的等待 |
| `--dataset.encoder_threads` | 视频编码线程数 |
| `--display_data` | 是否显示数据和相机画面 |

## 第一次采集建议

先用短采集验证：

```bash
lerobot-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --dataset.repo_id=<user>/zyarm_smoke_test \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=10 \
  --dataset.single_task="Move the arm slowly" \
  --display_data=true
```

如果短采集成功，再增加相机、时长和 episode 数量。

## 成功现象

- 命令可以打开 leader、follower 和相机。
- follower 跟随动作稳定，没有明显跳动或延迟堆积。
- 每个 episode 能正常开始和结束。
- 终端没有持续报错。
- dataset 能被后续 replay 命令读取。

## 性能建议

如果画面或控制周期明显卡顿，优先尝试：

- 降低相机分辨率，例如从 `1280x720` 降到 `640x480`。
- 降低相机 fps。
- 减少 `--dataset.encoder_threads`。
- 关闭 `--display_data`。
- 先不接入腕部相机，只保留一个 `front` 相机。

## 待项目方补充

> 待项目方补充：请提供官方推荐 dataset 命名规则、标准任务描述模板、推荐 episode 时长、推荐采集数量、数据集样例、采集窗口截图和演示视频。
