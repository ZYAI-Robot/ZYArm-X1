# 仿真与框架接入

本章说明如何使用项目已经提供的 Serial API、Python SDK、C++ SDK、官方 Web、ROS 2、MoveIt 和 Gazebo 路径，把机械臂接入不同技术栈并跑起来。

第一次上手不需要先读本章；当你已经完成 [快速上手](../02_快速上手/README.md)，能够读取状态并让机械臂安全执行小动作后，再根据目标选择对应路径。

如果你的目标是修改源码、扩展 SDK、维护 ROS 2 硬件接口或调整 MoveIt/Gazebo 配置，请阅读 [开发者指南](../09_开发者指南/README.md)。本章优先回答“怎么用现有能力跑起来”，不展开“源码怎么改”。Web 控制台以官网在线页面为公开使用入口。

## 先认识这些工具

| 工具/术语 | 本章里的作用 |
| --- | --- |
| Serial API | 直接使用固件串口协议，是所有上层控制的底层基础 |
| Python SDK | 用 Python 程序控制机械臂，适合脚本、诊断和轻量二次开发 |
| C++ SDK | 用 C++ 程序控制机械臂，适合性能敏感程序、系统集成和 C++ 示例验证 |
| 官方 Web / URDF | 在官网在线页面中显示机械臂模型、连接串口并做可视化交互 |
| ROS 2 / ros2_control | 机器人系统和控制框架，用于真机桥接和控制器管理 |
| MoveIt / RViz | 运动规划和可视化工具 |
| Gazebo | 仿真环境，用于在虚拟场景中验证机械臂 |

更基础的解释见 [术语与工具速查](../术语与工具速查.md)。

## 总体关系

```text
基础操作 / Serial API
   |
   +--> Python SDK
   |      |
   |      +--> LeRobot
   |
   +--> C++ SDK
   |      |
   |      +--> C++ 示例 / 系统集成
   |
   +--> 官方 Web 可视化 / 仿真
   |
   +--> ROS 2
          |
          +--> ros2_control
          +--> MoveIt
          +--> Gazebo
```

Serial API 是底层协议，Python SDK、C++ SDK、官方 Web、ROS 2 和 LeRobot 都会以不同方式封装或复用它。使用上层框架前，建议先完成基础串口状态读取和小动作验证。

## 真机接入前检查

进入 Web、SDK、ROS 2、MoveIt 或 Gazebo 的真机路径前，请先确认：

- 单臂或双臂都能读取 `[STATUS]`。
- 多机械臂场景下，`CMD22` 返回的名称和物理标签一致。
- `CMD1` 复位正常。
- 你知道如何直接切断机械臂电源。
- 摄像头可以打开并看到任务区域。
- 设备端口、相机编号和物理标签已经记录。
- 电源和线材不会干扰运动。

## 入口

- [Serial API](Serial_API/README.md)：外部程序直接接入固件文本串口协议。
- [Python](Python/README.md)：通过 Python SDK 控制机械臂。
- [C++](Cpp/README.md)：通过 C++ SDK 控制机械臂。
- [Web](Web/README.md)：官方 Web 控制台、URDF 显示、串口连接和网页仿真。
- [ROS2](ROS2/README.md)：ROS 2、ros2_control、MoveIt、Gazebo 和真机桥接。

## 路线选择

| 目标 | 推荐路径 |
| --- | --- |
| 只验证底层串口链路 | [Serial API](Serial_API/README.md) |
| 写教学脚本或轻量控制程序 | [Python](Python/README.md) |
| 写 C++ 控制程序或做系统集成 | [C++](Cpp/README.md) |
| 使用官方 Web 做网页可视化和交互控制 | [Web](Web/README.md) |
| 做 ROS 2 控制和 MoveIt 规划 | [ROS2](ROS2/README.md) |
| 做 Gazebo 仿真 | [ROS2](ROS2/README.md) -> Gazebo |
| 做 LeRobot 数据采集 | [科研与数据采集](../07_科研与数据采集/README.md) |
| 修改源码或扩展能力 | [开发者指南](../09_开发者指南/README.md) |

## 使用和开发的边界

| 你想做的事 | 主要阅读 |
| --- | --- |
| 跑一个现成示例 | 本章 |
| 找到某条 launch 命令 | 本章 |
| 看某个框架适合什么场景 | 本章 |
| 使用现成 C++ SDK 示例 | [C++ 接入](Cpp/README.md) |
| 修改 SDK API | [SDK 开发](../09_开发者指南/04_SDK开发/README.md) |
| 修改 ROS 2 / MoveIt / Gazebo 源码和配置 | [ROS 2 开发](../09_开发者指南/05_ROS2开发/README.md) |
| 扩展 LeRobot 插件 | [LeRobot 开发](../09_开发者指南/06_LeRobot开发/README.md) |

如果只是使用 LeRobot 跑遥操、record、replay 或 policy eval，不需要先读开发者指南，直接进入 [科研与数据采集](../07_科研与数据采集/README.md)。

## 命名提醒

当前 ROS 2、URDF、mesh 和 launch 入口统一使用 `x1_standard` 作为 `ZYArm-X1` 的模型资源名。
