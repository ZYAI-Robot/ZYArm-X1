`zyarm_hardware` 现在只负责机械臂硬件实例运行时，不再保留旧的 `serial_bridge` / `teleop_bridge` 双入口模型。

当前架构：

- `zyarm_hardware/config.py`
  负责多实例 YAML schema、配置解析和启动前校验。
- `zyarm_hardware/protocol.py`
  负责串口收发、ACK 等待、`[STATUS]` / `[MD]` 解析和日志。
- `zyarm_hardware/device.py`
  负责单个机械臂实例的串口生命周期、关节目标下发、`joint_io_fast` 调用和状态控制。
- `zyarm_hardware/relations.py`
  负责实例关系编排，当前 `teleop_follow` 关系保留 leader 的固件 `[MD]` 输出，并让 follower 通过 `joint_io_fast` 执行命令。
- `zyarm_hardware/node.py`
  负责统一 ROS 2 节点入口，按实例和关系创建服务与话题。

对外接口：

- 每个机械臂实例：
  服务：`<arm>/joint_io_fast`、`<arm>/set_joint_target`、`<arm>/set_status_report`
  话题：`<arm>/joint_state`
- 每个关系实例：
  服务：`<relation>/set_enabled`
  话题：`<follower>/joint_command`

默认配置文件：

- `config/single_slave_real.yaml`
  单机械臂实例，适合策略评估等单臂执行场景。
- `config/teleop_pair_real.yaml`
  一组主从机械臂实例和一个 `teleop_pair` 关系，适合 ROS 2 fast_io 遥操 / 采集场景。

启动方式：

- 直接运行节点：
  `ros2 run zyarm_hardware arm_system --ros-args -p config_file:=/abs/path/to/system.yaml`
- 使用 launch：
  `ros2 launch zyarm_hardware arm_system.launch.py config_file:=/abs/path/to/system.yaml`

YAML 结构：

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

约束：

- `relationships` 中的 `leader` 和 `follower` 必须引用已存在的 `arms`。
- 同一个机械臂实例不能同时参与多个 `teleop_follow` 关系。
- `joint_io_fast` 的 `apply_mask` 全 `false` 时表示只读模式，可用于初始状态种子获取。
- `set_joint_target` 的返回语义是“本地已受理并已下发到串口”，不是“机械臂已经完成动作”。
- `joint_io_fast` 的返回语义是“本次命令已受理，并返回了一帧对应的测量状态快照”。
- `-999.9` no-change 哨兵值只存在于串口适配层，不会暴露到 ROS 服务契约。
- ROS 2 只保留 fast_io 版遥操；需要固件从臂滤波手感的演示路径，请使用 `software/tools/filtered_master_slave_teleop.py`。

固件命令参考：

- `firmware/Core/Inc/arm_shell.h`
- `firmware/Core/Src/arm_shell_cmd.c`
