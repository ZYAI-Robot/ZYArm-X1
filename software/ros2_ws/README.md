# software/ros2_ws

这是本项目的 ROS 2 工作区根目录，主要承载模型显示、ros2_control、MoveIt 真机控制和 ROS 侧硬件调试。

## 编译

首次进入工作区后，先加载 ROS 2 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

然后在 `software/ros2_ws` 顶层编译：

```bash
cd software/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

如果只编译核心包：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select zyarm_description zyarm_control zyarm_hardware zyarm_hardware_interface zyarm_moveit_config zyarm_bringup zyarm_interfaces
source install/setup.bash
```

新开终端通常只需要重新加载环境：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 真机串口配置

真机场景分成两类入口：

- MoveIt 真机控制使用 `zyarm_hardware_interface`，通过 `demo_x1_standard_real.launch.py` 的 `serial_port` 等参数直接配置串口。
- 主从遥操和服务式硬件访问使用 `zyarm_hardware`，从 YAML 配置读取串口和实例关系。

默认配置：

- 单臂执行：`zyarm_hardware/config/single_slave_real.yaml`
- 一组主从遥操：`zyarm_hardware/config/teleop_pair_real.yaml`

如果串口号变了，直接修改对应 `arms.<name>.port`，例如：

```bash
ls /dev/ttyUSB*
```

也可以直接运行统一硬件入口并传入自定义配置：

```bash
ros2 launch zyarm_hardware arm_system.launch.py config_file:=/abs/path/to/system.yaml
```

YAML 基本结构：

```yaml
arms:
  master:
    port: /dev/ttyUSB0
  slave:
    port: /dev/ttyUSB1
relationships:
  teleop_pair:
    mode: teleop_follow
    leader: master
    follower: slave
    follow_frequency_hz: 50.0
```

## 常用启动命令

以下命令默认都在已经完成编译并执行过 `source install/setup.bash` 的前提下运行。

只看模型显示：

```bash
ros2 launch zyarm_description display_x1_standard.launch.py
```

启动 `ros2_control` 基础控制链：

```bash
ros2 launch zyarm_bringup bringup_x1_standard_ros2_control.launch.py
```

直接启动真机硬件实例运行时：

```bash
ros2 launch zyarm_hardware arm_system.launch.py

# 指定自定义 YAML
ros2 launch zyarm_hardware arm_system.launch.py config_file:=/abs/path/to/system.yaml
```

启动纯 ROS 主从遥操：

```bash
ros2 launch zyarm_bringup teleop_only_system.launch.py
```

默认读取 `zyarm_hardware/config/teleop_pair_real.yaml`。如果要换串口或关系配置，直接改这份 YAML，或传入自定义配置：

```bash
ros2 launch zyarm_bringup teleop_only_system.launch.py config_file:=/abs/path/to/teleop_pair.yaml
```

启动 MoveIt + `ros2_control`：

```bash
ros2 launch zyarm_moveit_config demo_x1_standard.launch.py
```

启动 Gazebo + MoveIt + RViz 仿真验证：

```bash
ros2 launch zyarm_moveit_config demo_x1_standard_gazebo.launch.py
```

该入口依赖 `ros_gz_sim`、`ros_gz_bridge` 和 Gazebo 相关 ROS 集成环境；若缺失，launch 会在解析或启动阶段失败。

启动 MoveIt 真机入口：

```bash
ros2 launch zyarm_moveit_config demo_x1_standard_real.launch.py serial_port:=/dev/ttyUSB0
```

这个入口会启动：

- `zyarm_bringup/bringup_x1_standard_real_ros2_control.launch.py`
- `zyarm_hardware_interface`
- `joint_state_broadcaster`、`arm_controller`、`gripper_controller`
- `move_group`
- RViz

`zyarm_hardware_interface` 在 `ros2_control_node` 进程内直接持有串口并发送 CMD36。MoveIt 真机路径不启动 `zyarm_hardware`。

## 目录说明

- `src/`
  工作区源码。
- `build/`
  编译中间产物。
- `install/`
  安装后的运行环境。
- `log/`
  `colcon` 构建日志。

更详细的包说明见 [src/README.md](/home/xg/workspace/zyarmv1/software/ros2_ws/src/README.md)。

## 包职责边界

- `zyarm_interfaces`：共享 ROS IDL 包，只提供 srv/msg/action 定义。
- `zyarm_hardware`：Python 硬件运行时，负责真机实例、串口服务、`joint_io_fast` 和纯 ROS 主从遥操。
- `zyarm_hardware_interface`：C++ ros2_control 真机硬件插件，负责 MoveIt 真机控制链路。
