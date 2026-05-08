# ros2_control 硬件接口

`ros2_control` 是 ROS 2 中连接“控制器”和“真实硬件”的框架。对 ZYArm 来说，它的作用是把 MoveIt 或控制器发出的关节轨迹，转换成固件能理解的串口指令；同时把固件返回的 `[STATUS]` 转换成 ROS 2 的 `/joint_states`。

本页重点说明 C++ `ros2_control` 硬件插件 `zyarm_hardware_interface`。它主要服务 MoveIt 真机控制链路，不负责主从遥操和多设备服务式运行时。

## 先认识这些组件

| 组件 | 简短说明 |
| --- | --- |
| `ros2_control` | ROS 2 控制框架，负责把控制器和硬件接口接起来 |
| `controller_manager` | 控制器管理器，负责加载、配置和启动各个 controller |
| `ros2_control_node` | 运行 `controller_manager` 和硬件插件的节点 |
| `hardware_interface::SystemInterface` | C++ 硬件插件接口，真实硬件要实现它 |
| `joint_state_broadcaster` | 把硬件状态发布成 `/joint_states` |
| `joint_trajectory_controller` | 接收 `FollowJointTrajectory` 轨迹，并把目标关节位置写给硬件接口 |
| `robot_description` | 由 URDF/Xacro 生成的机器人模型，里面包含 `<ros2_control>` 配置 |

可以先把它理解成下面这条链：

```text
MoveIt / 轨迹控制器
  -> joint_trajectory_controller
  -> controller_manager
  -> ZyArmSystemHardware
  -> 串口 CMD36
  -> 固件 STATUS
  -> /joint_states
```

## 两条真机路径

仓库里有两条真机接入路径，调试时不要混在一起。

| 路径 | 位置 | 适合场景 | 是否给 MoveIt 真机使用 |
| --- | --- | --- | --- |
| Python 硬件运行时 | `zyarm_hardware` | 多机械臂实例、串口服务、纯 ROS 主从遥操 | 否 |
| C++ ros2_control 插件 | `zyarm_hardware_interface` | 标准 controller 链路、MoveIt 真机控制 | 是 |

同一条真实机械臂串口不能同时被两条路径占用。如果已经启动 `zyarm_hardware/arm_system.launch.py`，再启动 MoveIt 真机入口时可能会因为串口被占用而失败。

## C++ 插件入口

| 文件 | 作用 |
| --- | --- |
| `include/zyarm_hardware_interface/zyarm_system_hardware.hpp` | `SystemInterface` 声明 |
| `src/zyarm_system_hardware.cpp` | 生命周期、read/write、接口导出和参数加载 |
| `src/shell_protocol.cpp` | `[CMD]` 格式化和 `[STATUS]` 解析 |
| `src/serial_transport.cpp` | Linux 串口打开、读写和后台接收线程 |
| `src/joint_mapping.cpp` | ROS 关节单位和固件角度表达之间的转换 |
| `plugin/zyarm_hardware_interface.xml` | 插件导出，供 `pluginlib` 加载 |
| `test/test_system_hardware_fake.cpp` | fake 串口测试，适合理解生命周期行为 |

## 生命周期做了什么

`ZyArmSystemHardware` 实现了 `hardware_interface::SystemInterface`。启动真机 ros2_control 时，它会按照生命周期逐步运行。

| 阶段 | 代码入口 | 做什么 |
| --- | --- | --- |
| 初始化 | `on_init()` | 校验 joint 名称和接口类型，读取硬件参数 |
| 配置 | `on_configure()` | 打开串口，启动接收线程 |
| 激活 | `on_activate()` | 发送 no-change `CMD36` 获取当前状态，把 command 初始化为当前 state |
| 读取 | `read()` | 消费后台线程缓存的最新 `[STATUS]` |
| 写入 | `write()` | 把 controller 写入的目标位置转换为固件角度，并发送 `CMD36` |
| 停用 | `on_deactivate()` | 保持当前位置命令，不主动复位或停止 |
| 清理 | `on_cleanup()` | 关闭串口，清理状态 |

`on_activate()` 不会复位机械臂。它只是读取当前状态作为初始值，避免 controller 一启动就因为 command 和 state 差异过大而突然跳变。

## 控制语义

真机插件只暴露 `position` 接口：

- command interfaces：`joint0` 到 `joint6` 的 `position`
- state interfaces：`joint0` 到 `joint6` 的 `position`
- 不暴露 `velocity`、`acceleration`、`effort`

其中 `joint0` 到 `joint5` 是机械臂 6 个旋转关节，ROS 侧单位是弧度。`joint6` 是夹爪主驱动关节，URDF 中是 prismatic joint，ROS 侧单位是米，默认行程由 `claw_travel_m` 表示，当前默认值为 `0.034`。

固件侧不是直接使用这些 ROS 单位：

- 机械臂关节会从弧度转换为固件角度。
- 默认固件角度偏移为 `0 -180 90 0 0 0`。
- 夹爪会从 `0.0..claw_travel_m` 映射到固件 `0..claw_command_max`，当前默认 `claw_command_max=100`。

## CMD36 状态语义

真机插件使用固件 `CMD36` 做高频控制。这个命令可以理解为“下发一帧关节目标，同时让固件返回状态快照”。

运行时不是每次 `write()` 都同步等待 `[STATUS]`：

- 后台接收线程持续解析 `[STATUS]`。
- `read()` 有新状态就更新，没有新状态就沿用上一帧有效状态。
- `write()` 只负责发送一帧 `CMD36`。
- 状态陈旧会通过日志和计数器观察，避免在 50Hz 循环里刷屏。

这意味着 `[STATUS]` 是状态快照，不是“轨迹已经执行完成”的确认。

## 相关配置

真机 ros2_control 参数来自 Xacro 和 launch：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `serial_port` | `/dev/ttyUSB0` | 机械臂串口 |
| `baud_rate` | `230400` | 串口波特率 |
| `read_timeout_ms` | `20` | 串口读取超时 |
| `write_timeout_ms` | `20` | 串口写入超时 |
| `activation_status_timeout_ms` | `1000` | 激活阶段等待初始 `[STATUS]` 的时间 |
| `status_stale_warn_ms` | `100` | 状态陈旧 warning 阈值 |
| `status_stale_error_ms` | `1000` | 状态陈旧 error 阈值 |
| `arm_hw_offsets_deg` | `0,-180,90,0,0,0` | 固件角度偏移 |
| `arm_hw_signs` | `1,1,1,1,1,1` | 固件角度方向 |
| `claw_travel_m` | `0.034` | ROS 侧夹爪行程 |
| `claw_command_max` | `100` | 固件侧夹爪命令最大值 |

这些参数在 `bringup_x1_standard_real_ros2_control.launch.py` 和 `demo_x1_standard_real.launch.py` 中都会暴露。

## 首次运行

运行前请确认：

- 已完成真机快速上手，串口软件可以读取 `[STATUS]`。
- 当前串口没有被其他程序占用。
- 机械臂周围无障碍，动作前可以随时断电。
- 已在 Ubuntu ROS 2 环境中完成构建并执行 `source install/setup.bash`。

启动不带 MoveIt 的真机 ros2_control：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_bringup bringup_x1_standard_real_ros2_control.launch.py serial_port:=/dev/ttyUSB0 use_rviz:=true
```

启动 MoveIt + 真机：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_moveit_config demo_x1_standard_real.launch.py serial_port:=/dev/ttyUSB0
```

## 调试命令

查看控制器是否加载并激活：

```bash
ros2 control list_controllers -c /zyarm_x1_standard_controller_manager
```

查看硬件接口：

```bash
ros2 control list_hardware_interfaces -c /zyarm_x1_standard_controller_manager
```

查看关节状态是否持续发布：

```bash
ros2 topic echo /joint_states --once
```

查看 action 接口是否存在：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 串口打开失败 | `serial_port` 是否正确，是否被串口软件、SDK、`zyarm_hardware` 占用 |
| 激活失败并等待 STATUS 超时 | 固件是否运行、波特率是否为 `230400`、机械臂是否持续输出 `[STATUS]` |
| controller 一直 inactive | `joint_state_broadcaster` 是否先启动，controller 配置是否加载成功 |
| `/joint_states` 没有 `joint0..joint6` | URDF、硬件接口、controller 的 joint 名称是否一致 |
| MoveIt 可以规划但不能执行 | `arm_controller` 和 `gripper_controller` 是否 active，MoveIt controller 映射是否正确 |
| 状态陈旧 warning | 串口链路、固件状态上报、USB 线材和系统负载是否稳定 |

## 修改检查

修改 ros2_control 链路时，建议按下面顺序检查：

1. joint 名称是否仍是 `joint0..joint6`，顺序是否一致。
2. 真机路径是否只暴露 `position` state 和 command interface。
3. Gazebo/mock 路径是否仍与自己的 controller 配置匹配。
4. `JointMapping` 的偏移、符号和夹爪映射是否与固件一致。
5. `CMD36` 格式是否仍与固件协议一致。
6. `bringup_x1_standard_real_ros2_control.launch.py` 和 `demo_x1_standard_real.launch.py` 是否同步暴露新参数。
7. fake transport 测试是否覆盖新增行为。

## 待项目方补充

- 真机 ros2_control 的标准演示动作。
- controller manager 调试截图。
- 真实机械臂串口、波特率和安全摆放标准。
