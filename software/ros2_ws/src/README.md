# ROS2 工作区源码导读

`software/ros2_ws/src` 是 ROS 2 工作区的源码目录。工作区内的 `build/`、`install/`、`log/` 是 `colcon` 生成物，阅读和修改代码时应优先关注 `src/`。

当前顶层目录可按两类理解：

- 源码与文档目录
  - `zyarm_description`
    机械臂模型、网格、URDF/Xacro、RViz 配置，以及历史资料。
  - `zyarm_control`
    `ros2_control` 控制器配置和基础控制链 launch。
  - `zyarm_interfaces`
    共享 ROS IDL 包，只提供 srv/msg/action 定义。
  - `zyarm_hardware`
    Python 硬件运行时，负责多机械臂实例、串口服务、`joint_io_fast` 和纯 ROS 主从遥操。
  - `zyarm_hardware_interface`
    C++ ros2_control 真机硬件插件，负责 MoveIt 真机控制链路。
  - `zyarm_moveit_config`
    MoveIt 的 SRDF、运动学、规划器和 RViz 配置。
  - `zyarm_bringup`
    系统级启动入口，统一暴露控制、真机硬件调试和纯 ROS 遥操场景。
- 工作区根目录中的构建产物
  - `build/`
  - `install/`
  - `log/`

从包关系上看，当前系统大致分为七层：

- 模型层：`zyarm_description`
- 控制器契约层：`zyarm_control`
- 共享接口层：`zyarm_interfaces`
- 服务式真机接入层：`zyarm_hardware`
- MoveIt 真机硬件插件层：`zyarm_hardware_interface`
- 规划层：`zyarm_moveit_config`
- 系统编排层：`zyarm_bringup`

环境设置与编译：

- 加载 ROS 2 环境：
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```
- 在工作区根目录编译：
  ```bash
  cd /home/xg/workspace/zyarmv1/software/ros2_ws
  colcon build --symlink-install
  ```
- 编译完成后加载工作区环境：
  ```bash
  source install/setup.bash
  ```
- 如果只想编译当前这几个核心包，可执行：
  ```bash
  cd /home/xg/workspace/zyarmv1/software/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --packages-select zyarm_description zyarm_control zyarm_hardware zyarm_hardware_interface zyarm_moveit_config zyarm_bringup zyarm_interfaces
  source install/setup.bash
  ```
- 新开一个终端通常不需要重新编译，只需要重新加载环境：
  ```bash
  cd /home/xg/workspace/zyarmv1/software/ros2_ws
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ```

常用入口速查：

- 只看模型显示：
  `ros2 launch zyarm_description display_x1_standard.launch.py`
- 启动 `ros2_control` 基础控制链：
  `ros2 launch zyarm_bringup bringup_x1_standard_ros2_control.launch.py`
- 启动真机实例运行时：
  `ros2 launch zyarm_hardware arm_system.launch.py`
- 启动纯 ROS 主从遥操：
  `ros2 launch zyarm_bringup teleop_only_system.launch.py`
- 启动 MoveIt + `ros2_control`：
  `ros2 launch zyarm_moveit_config demo_x1_standard.launch.py`
- 启动 MoveIt + 真机：
  `ros2 launch zyarm_moveit_config demo_x1_standard_real.launch.py serial_port:=/dev/ttyUSB0`

几个总入口之间的区别：

- `bringup_x1_standard_ros2_control`
  面向基础控制链联调，核心是 `robot_state_publisher`、`ros2_control_node` 和控制器。
- `teleop_only_system`
  启动 `zyarm_hardware/arm_system.launch.py`，默认使用 `zyarm_hardware/config/teleop_pair_real.yaml`。

包名相近但职责不同：

- `zyarm_interfaces` 是共享 ROS IDL 包。
- `zyarm_hardware` 是 Python 硬件运行时，用于非 MoveIt 的硬件服务和纯 ROS 遥操。
- `zyarm_hardware_interface` 是 ros2_control `hardware_interface::SystemInterface` 插件，用于 MoveIt 真机控制。

说明：

- `demo_x1_standard_real.launch.py` 是唯一真机 MoveIt 入口，通过 `zyarm_hardware_interface`、controller manager 和标准 `joint_trajectory_controller` 控制真机。
- `bringup_x1_standard_real_ros2_control.launch.py` 是不带 MoveIt 的底层真机 ros2_control bringup，适合单独调试 controller manager 和 `/joint_states`。

按阅读目标选择入口：

1. 想理解整机怎么拼起来，先看 `zyarm_bringup`。
2. 想理解真机控制和主从遥操，继续看 `zyarm_hardware`。
3. 想看模型和基础控制配置，分别看 `zyarm_description`、`zyarm_control`。
4. 想看规划链路，再看 `zyarm_moveit_config`。
