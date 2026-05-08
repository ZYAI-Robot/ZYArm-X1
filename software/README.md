# software

这里放运行在电脑或 SBC 上、并与机械臂通信的软件。

## 子目录

```text
software/
├── zyarm_sdk/       # 独立 Python/C++ 机械臂 SDK，面向二开、教学、诊断和 LeRobot 适配
├── ros2_ws/         # ROS 2 工作空间
├── examples/        # 示例和演示程序
├── diagnostics/     # 压测、可靠性验证和诊断脚本
└── tools/           # 日常调试和辅助工具
```

后续 LeRobot 原生适配层的位置预留为：

```text
software/lerobot_robot_zyarm/
```

`zyarm_sdk` 与 ROS 2 真机 MoveIt 路径保持独立：`zyarm_hardware_interface` 继续直接持有串口并作为 ros2_control 插件运行，不依赖 SDK。
