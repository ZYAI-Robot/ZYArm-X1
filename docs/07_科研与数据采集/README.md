# 科研与数据采集

本章面向希望使用 ZYArm 做遥操、数据采集、数据集回放、策略评估和算法实验的用户。它默认你使用项目已经提供的 LeRobot、Python SDK 和固件能力，不要求你修改源码。

如果你的目标是修改 LeRobot 插件、调整 observation/action 字段、扩展相机配置或升级 LeRobot 版本，请阅读 [LeRobot 开发](../09_开发者指南/06_LeRobot开发/README.md)。本章回答“怎么用现有能力做实验”，开发者指南回答“怎么改和维护插件”。

## 阅读前提

开始本章前，建议先完成：

- [快速上手](../02_快速上手/README.md)：能上电、连接串口、读取状态并完成一次安全小动作。
- [基础操作](../04_基础操作/README.md)：理解 `CMD6` 状态读取、`CMD1` 复位、`CMD21`/`CMD22` 设备命名等基础指令。
- [设备角色与端口确认](03_设备角色与端口确认.md)：能区分多台机械臂串口、固件名称和物理标签。
- [摄像头与视角配置](04_摄像头与视角配置.md)：如果要记录图像数据，先确认相机编号和物理视角。
- [主从臂遥操](../05_常用玩法/04_主从臂遥操.md)：如果要做 leader/follower 数据采集，先确认双臂遥操链路稳定。

数据采集、回放和策略评估都会真实驱动机械臂。首次运行请空载、低速、远离限位区域，并确认可以快速切断机械臂电源。

## 先认识这些词

| 术语 | 本章里的含义 |
| --- | --- |
| LeRobot | 用于遥操、数据采集、回放、训练和评估的机器人学习工具链 |
| `lerobot_robot_zyarm` | ZYArm 面向 LeRobot 的设备适配插件 |
| leader / teleoperator | 提供人工动作输入的主臂 |
| follower / robot | 接收动作并执行的从臂 |
| teleoperate | 人工遥控，让 follower 跟随 leader 或输入设备 |
| record | 采集 episode，并保存成 dataset |
| replay | 读取 dataset 中保存的 action，再发送给 follower |
| policy eval | 让训练好的策略模型接管 follower 并记录评估过程 |
| dataset / episode | 数据集 / 一次完整任务过程 |
| camera | LeRobot 管理的图像输入，例如 `front`、`wrist` |
| 100Hz | 高频控制和状态读取的目标量级，不等同于所有数据集都以 100Hz 存储 |

更多解释见 [术语与工具速查](../术语与工具速查.md)。

## 推荐阅读顺序

```text
01_科研路线与实验准备
  -> 02_LeRobot环境与插件验证
  -> 03_设备角色与端口确认
  -> 04_摄像头与视角配置
  -> 05_teleoperate遥操验证
  -> 06_record数据采集
  -> 07_数据集检查与replay回放
  -> 08_policy_eval策略评估
  -> 09_100Hz控制与数据质量
  -> 10_常见问题
```

这条路线里，`teleoperate` 用来先验证人工遥控链路，`record` 用来把演示保存成数据集，`replay` 用来检查数据能否复现，`policy eval` 用来让策略模型真正接管机械臂。

## 本章页面

| 页面 | 目标 |
| --- | --- |
| [科研路线与实验准备](01_科研路线与实验准备.md) | 明确实验路线、硬件软件准备和安全检查 |
| [LeRobot 环境与插件验证](02_LeRobot环境与插件验证.md) | 安装 LeRobot、SDK 和 ZYArm 插件，并确认能导入 |
| [设备角色与端口确认](03_设备角色与端口确认.md) | 区分 leader/follower 串口、固件名称和物理标签 |
| [摄像头与视角配置](04_摄像头与视角配置.md) | 配置 `front`、`wrist` 等相机，并确认视角正确 |
| [teleoperate 遥操验证](05_teleoperate遥操验证.md) | 先运行遥操，不录制数据，确认动作和画面稳定 |
| [record 数据采集](06_record数据采集.md) | 使用 LeRobot 采集 episode 和 dataset |
| [数据集检查与 replay 回放](07_数据集检查与replay回放.md) | 检查数据质量，并用 replay 验证 action 可复现 |
| [policy eval 策略评估](08_policy_eval策略评估.md) | 使用训练好的策略运行评估 |
| [100Hz 控制与数据质量](09_100Hz控制与数据质量.md) | 区分控制频率、相机帧率、编码和数据写入质量 |
| [常见问题](10_常见问题.md) | 排查端口接反、相机错位、卡顿、权限和危险动作 |

## 使用和开发的边界

| 你想做的事 | 主要阅读 |
| --- | --- |
| 跑通 LeRobot 数据采集 | 本章 |
| 录制一个可 replay 的数据集 | 本章 |
| 检查相机视角和 episode 质量 | 本章 |
| 评估训练好的 policy | 本章 |
| 修改 `zyarm_follower` 或 `zyarm_leader` 的源码行为 | [LeRobot 开发](../09_开发者指南/06_LeRobot开发/README.md) |
| 修改 observation/action 字段、单位或转换逻辑 | [LeRobot 开发](../09_开发者指南/06_LeRobot开发/README.md) |
| 升级 LeRobot 版本或适配新的 API | [LeRobot 开发](../09_开发者指南/06_LeRobot开发/README.md) |

## 总体链路

```text
LeRobot 命令
  -> lerobot_robot_zyarm
  -> zyarm_sdk Python
  -> 串口 / 固件
  -> 真实机械臂
```

这条路径不依赖 ROS 2、MoveIt 或 Gazebo。ROS 2 相关内容请阅读 [仿真与框架接入](../06_仿真与框架接入/README.md)。

## 待项目方补充

> 待项目方补充：请提供科研数据采集推荐套装照片、双臂摆放示意图、标准任务样例、推荐相机型号、数据集样例、100Hz 实测结果、策略评估截图和完整演示视频。
