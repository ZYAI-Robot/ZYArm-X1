# 开发者指南

本章面向维护者和二次开发者，帮助你判断“要改哪一层、代码在哪里、改完还要同步哪些文档和验证”。如果你只是第一次使用机械臂，请先阅读 [快速上手](../02_快速上手/README.md) 和 [基础操作](../04_基础操作/README.md)。

如果你只是想运行现有 Python、Web、ROS 2、MoveIt 或 Gazebo 示例，请先阅读 [仿真与框架接入](../06_仿真与框架接入/README.md)。本章主要面向源码修改、接口扩展、测试验证和交付维护。

基础术语见 [术语与工具速查](../术语与工具速查.md)。

## 按目标选择入口

| 我想做什么 | 优先阅读 | 主要代码位置 |
| --- | --- | --- |
| 了解开发全貌 | [开发者总览](01_开发者总览.md) | 全仓库 |
| 判断一个功能该改哪一层 | [仓库结构与模块边界](02_仓库结构与模块边界.md) | `firmware/`、`software/`、`web/` |
| 修改或新增固件 `[CMD]` | [固件开发](03_固件开发/README.md) | `firmware/` |
| 扩展 Python/C++ API | [SDK 开发](04_SDK开发/README.md) | `software/zyarm_sdk/` |
| 修改 ROS 2、MoveIt 或 Gazebo 集成 | [ROS 2 开发](05_ROS2开发/README.md) | `software/ros2_ws/` |
| 修改 Web 仿真或交互 | [Web 开发](06_Web开发/README.md) | `web/` |
| 修改 LeRobot 插件、数据字段或适配逻辑 | [LeRobot 开发](07_LeRobot开发/README.md) | `software/lerobot_robot_zyarm/` |
| 增加示例、工具或压测脚本 | [示例工具与诊断开发](08_示例工具与诊断开发.md) | `software/examples/`、`software/tools/`、`software/diagnostics/` |
| 准备提交、发布或交付 | [测试发布与贡献检查](09_测试发布与贡献检查.md) | 全仓库 |

## 本章阅读顺序

第一次参与开发时，建议按下面顺序读：

```text
01_开发者总览
  -> 02_仓库结构与模块边界
  -> 选择对应模块子章节
  -> 09_测试发布与贡献检查
```

如果你已经知道自己要改哪一层，可以直接跳到对应子目录。

## 开发边界速记

- `firmware/` 决定控制板上的真实行为，包括串口命令、运动执行、状态上报和录制回放。
- `software/zyarm_sdk/` 封装上位机 API，负责把串口协议变成 Python/C++ 可调用接口。
- `software/ros2_ws/` 承载 ROS 2、MoveIt、Gazebo 和 ros2_control 接入。
- `web/` 承载浏览器端模型加载、仿真交互和后续真机控制入口。
- `software/lerobot_robot_zyarm/` 承载 LeRobot 原生适配，不依赖 ROS 2；使用教程见 [科研与数据采集](../07_科研与数据采集/README.md)，源码修改见 [LeRobot 开发](07_LeRobot开发/README.md)。
- `software/examples/` 是教学和应用示例，`software/tools/` 是日常工具，`software/diagnostics/` 是诊断和压测。

不要只看自己修改的那一层。只要固件协议、单位、状态字段或控制语义发生变化，就需要检查 SDK、Serial API、基础操作、工具、诊断脚本和上层框架是否同步。

## 代码目录 README

本章讲开发路径、模块边界和跨层检查；更贴近源码的说明仍保留在代码目录 README：

- [software/README.md](../../software/README.md)
- [software/zyarm_sdk/README.md](../../software/zyarm_sdk/README.md)
- [software/ros2_ws/src/README.md](../../software/ros2_ws/src/README.md)
- [software/lerobot_robot_zyarm/README.md](../../software/lerobot_robot_zyarm/README.md)
- [software/examples/README.md](../../software/examples/README.md)
- [software/tools/README.md](../../software/tools/README.md)
- [software/diagnostics/README.md](../../software/diagnostics/README.md)
- [web/README.md](../../web/README.md)
- [hardware/README.md](../../hardware/README.md)

## 待项目方补充

- 官方固件发布流程、版本命名和回滚规则。
- 官方烧录工具版本、接线照片、BOOT 设置截图和烧录成功截图。
- 开发分支、代码评审、测试记录和发布说明模板。

