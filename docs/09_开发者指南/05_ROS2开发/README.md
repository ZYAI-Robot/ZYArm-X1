# ROS 2 开发

ROS 2 章节面向需要维护模型、控制器、MoveIt、Gazebo、ros2_control 或真机硬件接口的开发者。ROS 2 环境准备见 [ROS2 MoveIt Gazebo 环境](../../03_安装与准备/03_ROS2_MoveIt_Gazebo环境.md)。

如果只是想启动模型显示、MoveIt 真机入口或 Gazebo 仿真，请先阅读 [ROS2 接入](../../06_仿真与框架接入/ROS2/README.md)。本节主要说明 ROS 2 工作区源码、硬件接口、MoveIt/Gazebo 配置和维护检查。

## 先认识这些工具

| 工具 | 作用 |
| --- | --- |
| ROS 2 | 机器人软件通信和运行框架 |
| colcon | ROS 2 工作区构建工具 |
| ros2_control | 控制器和硬件接口框架 |
| MoveIt | 运动规划和 RViz 交互规划 |
| Gazebo | 物理仿真和场景运行环境 |
| launch | ROS 2 的启动编排文件 |

源码入口见 [software/ros2_ws/src/README.md](../../../software/ros2_ws/src/README.md)。

## 本节页面

- [工作区与包职责](01_工作区与包职责.md)
- [ros2_control 硬件接口](02_ros2_control硬件接口.md)
- [MoveIt 配置](03_MoveIt配置.md)
- [Gazebo 与 launch 配置](04_Gazebo与launch配置.md)

## 最小构建检查

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 待项目方补充

- 官方推荐 Ubuntu 版本和 ROS 2 Jazzy 安装镜像。
- 真机 MoveIt 演示视频和标准验证流程。
- Gazebo 场景的课程/比赛任务模板。
