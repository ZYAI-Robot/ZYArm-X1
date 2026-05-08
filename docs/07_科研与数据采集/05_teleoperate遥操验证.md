# teleoperate 遥操验证

`lerobot-teleoperate` 用于先验证遥操链路。它会读取 leader 的动作输入，并让 follower 跟随运动。这个步骤不录制数据，适合在正式采集前确认端口、动作、相机和显示都正常。

## 运行前检查

- leader 和 follower 端口已经确认，没有接反。
- 两台机械臂都能单独读取状态并复位。
- follower 周围没有障碍物，首次运行保持空载。
- 摄像头已经完成视角确认。
- 可以快速切断 follower 电源。

## 不带摄像头的遥操

先运行最小命令：

```bash
lerobot-teleoperate \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

Windows 端口示例：

```bash
lerobot-teleoperate \
  --robot.type=zyarm_follower \
  --robot.port=COM4 \
  --teleop.type=zyarm_leader \
  --teleop.port=COM3 \
  --display_data=true
```

参数含义：

| 参数 | 含义 |
| --- | --- |
| `--robot.type=zyarm_follower` | 使用 ZYArm follower 适配，follower 是执行动作的机械臂 |
| `--robot.port` | follower 串口 |
| `--teleop.type=zyarm_leader` | 使用 ZYArm leader 适配，leader 是人工输入来源 |
| `--teleop.port` | leader 串口 |
| `--display_data=true` | 显示 LeRobot 数据和画面，便于观察 |

## 带摄像头的遥操

如果要同时显示双摄像头画面，可以添加 camera 配置：

```bash
lerobot-teleoperate \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=zyarm_leader \
  --teleop.port=/dev/ttyUSB0 \
  --display_data=true
```

如果显示窗口或控制明显卡顿，先关闭 `--display_data` 或降低相机分辨率/fps。正式采集前，建议至少运行一次带显示的遥操，确认画面和角色都正确。

## 成功现象

- 命令能打开 leader 和 follower 串口。
- follower 能跟随 leader 小幅运动。
- 运动方向和预期一致，没有突然大幅跳动。
- 如果打开相机，`front` 和 `wrist` 画面对应正确。
- 退出命令后，follower 不会继续接收遥操动作。

## 异常处理

| 现象 | 处理 |
| --- | --- |
| follower 不动 | 停止程序，确认 follower 串口、供电和状态读取 |
| leader 在动 | 端口可能接反，立即停止并重新确认角色 |
| 运动方向或幅度异常 | 停止程序，检查 leader/follower 摆放、固件名称和配置 |
| 显示卡顿 | 降低相机分辨率/fps，或关闭 `--display_data` |
| 机械臂接近障碍物 | 优先直接断电，不要继续观察 |

## 下一步

遥操稳定后，再进入 [record 数据采集](06_record数据采集.md)。如果遥操不稳定，不建议直接采集数据。

## 待项目方补充

> 待项目方补充：请提供标准 teleoperate 演示视频、推荐 leader/follower 摆放照片、推荐低通滤波参数、正常显示窗口截图和异常动作示例。
