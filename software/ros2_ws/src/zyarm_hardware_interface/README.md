# zyarm_hardware_interface

`zyarm_hardware_interface` 是 ZyArm 真机的标准 `ros2_control` 硬件插件包。它实现
`hardware_interface::SystemInterface`，由 `ros2_control_node` 加载，并在控制循环中直接
通过串口发送固件 `CMD36`。

这个包只负责一件事：把真实机械臂翻译成 ros2_control 的 position-only 关节接口。

## 与现有路径的边界

- 不替代 `zyarm_hardware` Python node。`zyarm_hardware` 仍用于实例服务、遥操和现有 fast_io 服务路径。
- 真机 MoveIt 默认且唯一使用本插件路径。
- 不通过 `/slave/joint_io_fast` ROS service 控制真机。该插件直接持有串口，避免把高频控制重新绕回 Python node 和 ROS service。
- 同一条真实机械臂串口不能同时被 MoveIt ros2_control 路径和 `zyarm_hardware` 服务式路径占用。

## 控制语义

- command interfaces: `joint0` 到 `joint6` 的 `position`
- state interfaces: `joint0` 到 `joint6` 的 `position`
- 不暴露 velocity、acceleration 或 effort 接口
- 默认目标控制频率由真机 controller 配置设为 50Hz

## CMD36 状态语义

固件 `CMD36` 是“先读状态，再下发目标”的复合命令。返回的 `[STATUS]` 表示当前/上一周期测量快照，
不是目标完成确认。

运行期 `read()` 和 `write()` 不等待 `[STATUS]`：

- 后台接收线程解析 `[STATUS]` 并更新缓存。
- `read()` 有新状态就更新 state，没有新状态就沿用上一帧有效状态。
- `write()` 只负责写一帧 CMD36，不等待状态返回。
- 状态陈旧和串口错误通过节流日志与计数器观测，避免在 50Hz 循环里刷屏。

## 生命周期

- `on_activate()` 使用 no-change CMD36 获取当前状态，并把 command 初始化为当前 state。
- `on_activate()` 不 reset 机械臂。
- `on_deactivate()` 保持当前位置命令。
- `on_deactivate()` 不默认发送 reset 或 stop。

## 首次运行

先确认没有启动 `zyarm_hardware/arm_system` 等服务式真机入口去抢同一个串口。

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_moveit_config demo_x1_standard_real.launch.py serial_port:=/dev/ttyUSB0
```

如果启动失败，优先检查：

1. `serial_port` 是否是当前机械臂串口，且没有被其他进程占用。
2. `zyarm_x1_standard_controller_manager` 是否成功 configure/activate 硬件组件。
3. `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 是否都 loaded/active。
4. `/joint_states` 是否能持续发布 `joint0` 到 `joint6`。
5. 日志里是否出现状态陈旧 warning。单次缺 `[STATUS]` 不代表失败，持续陈旧才需要检查串口和固件状态。
