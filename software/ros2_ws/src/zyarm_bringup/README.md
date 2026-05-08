`zyarm_bringup` 负责把描述、仿真、硬件实例运行时和 ros2_control 控制链整理成系统入口。

当前保留的入口：

- `bringup_x1_standard_ros2_control.launch.py`
  启动基础 ros2_control 控制链，适合验证模型、controller manager 和控制器配置。
- `bringup_x1_standard_real_ros2_control.launch.py`
  启动真机 ros2_control 底层控制链，不启动 MoveIt；适合单独验证 controller manager、控制器和 `/joint_states`。
- `teleop_only_system.launch.py`
  启动 `zyarm_hardware/arm_system.launch.py`，默认读取 `zyarm_hardware/config/teleop_pair_real.yaml`，用于纯 ROS 主从遥操调试。

职责边界：

- `zyarm_hardware`
  负责多机械臂实例、实例关系和硬件服务/话题契约。
- `zyarm_hardware_interface`
  负责 MoveIt 真机 ros2_control 控制链路中的硬件插件。
- `zyarm_bringup`
  只负责场景级 launch 组合，不承载学习、数据集或策略评估逻辑。
