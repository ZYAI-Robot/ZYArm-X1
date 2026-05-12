# LeRobot 开发

LeRobot 插件用于把 ZYArm 接入 LeRobot 的 robot / teleoperator 机制。它不依赖 ROS 2，机械臂通信由 `zyarm_sdk` 直接访问串口完成。

如果只是想运行遥操、数据采集、replay 或 policy eval，请先阅读 [科研与数据采集](../../07_科研与数据采集/README.md)。本节主要说明插件源码怎么工作、要改哪里、改完怎么验证。

```text
LeRobot 命令
  -> lerobot_robot_zyarm
  -> zyarm_sdk Python
  -> 串口 / 固件
```

源码入口见 [software/lerobot_robot_zyarm/README.md](../../../software/lerobot_robot_zyarm/README.md)。

## 本节页面

- [插件结构](01_插件结构.md)
- [config 配置与注册机制](02_config配置与注册机制.md)
- [features 与 conversion 数据字段](03_features与conversion数据字段.md)
- [robot follower 适配](04_robot_follower适配.md)
- [teleoperator leader 适配](05_teleoperator_leader适配.md)
- [常见扩展场景](06_常见扩展场景.md)
- [测试与兼容性检查](07_测试与兼容性检查.md)

## 源码阅读顺序

```text
01_插件结构
  -> 02_config配置与注册机制
  -> 03_features与conversion数据字段
  -> 04_robot_follower适配
  -> 05_teleoperator_leader适配
  -> 06_常见扩展场景
  -> 07_测试与兼容性检查
```

先读 `config.py`，理解 LeRobot 如何找到 `zyarm_follower` 和 `zyarm_leader`。再读 `features.py` / `conversion.py`，理解数据字段。最后分别读 follower 和 leader 的生命周期。

## 和科研章节的分工

| 目标 | 阅读位置 |
| --- | --- |
| 安装并验证 LeRobot 插件 | [科研与数据采集](../../07_科研与数据采集/02_LeRobot环境与插件验证.md) |
| 运行 `lerobot-teleoperate` | [teleoperate 遥操验证](../../07_科研与数据采集/05_teleoperate遥操验证.md) |
| 运行 `lerobot-record` 采集数据 | [record 数据采集](../../07_科研与数据采集/06_record数据采集.md) |
| replay 数据集或做 policy eval | [数据集检查与 replay 回放](../../07_科研与数据采集/07_数据集检查与replay回放.md)、[policy eval 策略评估](../../07_科研与数据采集/08_policy_eval策略评估.md) |
| 修改插件源码、字段、单位或兼容性 | 本节 |

## 核心数据流

action 路径：

```text
LeRobot
  -> ZyArmLeaderTeleoperator.get_action()
  -> Retargeter.apply()
  -> positions_to_action()
  -> ZyArmFollowerRobot.send_action()
  -> action_to_positions()
  -> SafetyController.sanitize_positions()
  -> zyarm_sdk.fast_io()
```

observation 路径：

```text
ZyArmFollowerRobot.get_observation()
  -> arm.get_latest_state()
  -> positions_to_action()
  -> camera.read_latest()
  -> LeRobot dataset
```

## 开发前确认

- 当前插件支持的 LeRobot 版本。
- SDK 的单位和特征表达是否变化。
- leader/follower 串口和相机配置是否清楚。
- 数据集中的 observation/action 字段是否保持兼容。
- 修改后需要同步 [科研与数据采集](../../07_科研与数据采集/README.md) 中的使用说明吗。

## 待项目方补充

- LeRobot 版本升级策略。
- 官方兼容矩阵。
- 数据集字段兼容性策略。
- 推荐扩展和发布规则。
