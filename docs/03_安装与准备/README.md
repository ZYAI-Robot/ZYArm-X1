# 安装与准备

本章说明快速上手之后，不同使用路线需要继续准备哪些环境。第一次上手不需要一次装完所有东西，先按自己的目标选择即可。

如果你的目标只是让机械臂第一次动起来，请优先阅读 [快速上手](../02_快速上手/README.md)。串口驱动、CH340、COM 口、`/dev/ttyUSBx`、串口工具参数和第一次状态读取都在那里说明；本章不再重复基础串口连接流程。

## 先认识这些工具

| 工具 | 本章里的作用 |
| --- | --- |
| Chrome / Edge / Web Serial | 使用官方 Web 控制台连接本地机械臂串口 |
| Python / 虚拟环境 / pip | 运行 SDK、tools 和 LeRobot 插件，并隔离依赖 |
| ROS 2 / MoveIt / Gazebo | 机器人通信、运动规划和仿真路线 |
| LeRobot | 遥操、数据采集、回放和策略评估路线 |
| Keil / STM32 | 固件编译、烧录和底层开发路线 |

术语细节见 [术语与工具速查](../术语与工具速查.md)。

## 环境分层

| 目标 | 推荐系统 | 本章准备 |
| --- | --- | --- |
| 第一次使用机械臂 | Windows / Ubuntu / macOS | 不需要额外环境，直接看 [快速上手](../02_快速上手/README.md)，使用官方 Web 控制台 |
| 使用官方 Web 控制台 | Windows / Ubuntu / macOS | Chrome/Edge、Web Serial、USB 串口和网络访问 |
| 使用 Python SDK 或 tools | Windows / Ubuntu | Python、虚拟环境、`software/zyarm_sdk/python` |
| 使用 ROS 2 / MoveIt / Gazebo | Ubuntu 推荐 | ROS 2 Jazzy、colcon、Gazebo 相关集成 |
| 使用 LeRobot 数据采集 | Ubuntu 或已验证 Python 环境 | `lerobot==0.5.1`、SDK、LeRobot 插件、摄像头 |
| 开发固件 | Windows 常见 | Keil 相关工具链 |

## 阅读顺序

按目标选择，不需要全部阅读：

1. [Python SDK 环境](01_Python_SDK环境.md)
2. [Web 控制台使用环境](02_Web仿真环境.md)
3. [ROS 2 / MoveIt / Gazebo 环境](03_ROS2_MoveIt_Gazebo环境.md)
4. [LeRobot 环境](04_LeRobot环境.md)
5. [固件开发环境](05_固件开发环境.md)
6. [安装完成检查](06_安装完成检查.md)

## 路线选择

```text
第一次使用机械臂
  -> 快速上手
  -> 官方 Web 控制台
  -> 常用玩法

写 Python 脚本或运行 tools
  -> Python SDK 环境
  -> 仿真与框架接入 / 常用玩法

使用官方 Web 控制台
  -> Web 控制台使用环境
  -> Web 接入

使用 ROS 2 / MoveIt / Gazebo
  -> 快速上手完成基础串口验证
  -> ROS 2 / MoveIt / Gazebo 环境
  -> 仿真与框架接入

使用 LeRobot
  -> 快速上手完成基础串口验证
  -> Python SDK 环境
  -> LeRobot 环境
  -> 科研与数据采集
```

## 安装原则

- 先装当前目标必需环境，不要一次装完所有框架。
- 每装完一层环境，都执行对应页面里的验证命令。
- 官方在线 Web 控制台使用 `https://arm.zyairobot.com/#/zy`，第一次使用先确认浏览器、串口、供电和安全空间。
- 涉及真机运动前，先确认 Web 或串口工具能连接设备、复位正常、断电方式可用。
- 复杂框架遇到问题时，先回到 [快速上手](../02_快速上手/README.md) 的最小串口链路，验证机械臂本身是否正常。
- 不确定的驱动链接、软件版本、截图和硬件资料会保留 `待项目方补充` 占位。

## 总检查清单

- [ ] 已确认自己的目标路线。
- [ ] 已完成 [快速上手](../02_快速上手/README.md) 中的官方 Web 第一次连接和小幅动作。
- [ ] 已确认目标路线需要安装哪些额外环境。
- [ ] 已完成目标框架的安装验证。
- [ ] 复杂框架只在需要时安装。

更细的检查项见 [安装完成检查](06_安装完成检查.md)。
