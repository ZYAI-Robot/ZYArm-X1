`zyarm_moveit_config` 负责规划侧配置，把模型、运动学、规划器和控制器映射组织成 MoveIt 可直接启动的形态。

当前主要内容：

- `srdf/zyarm_x1_standard.srdf`
  MoveIt 语义模型，定义规划组、末端执行器等概念。
- `srdf/zyarm_x1_standard_gazebo.srdf`
  Gazebo 专用语义模型，夹爪组同时包含 `joint6` 和 `joint7`。
- `config/kinematics.yaml`
  各规划组的逆解求解器参数。
- `config/ompl_planning.yaml`
  OMPL 规划器和默认请求/响应适配器配置。
- `config/joint_limits.yaml`
  MoveIt 规划时采用的速度、加速度限制。
- `config/moveit_controllers*.yaml`
  MoveIt 到控制器 action 的映射。
- `launch/demo_x1_standard.launch.py`
  基于 `ros2_control` 的 MoveIt 启动入口。
- `launch/demo_x1_standard_gazebo.launch.py`
  基于 Gazebo 抓取场景的 MoveIt 启动入口，用于在 RViz MotionPlanning 中设置关节目标，并在 Gazebo 中观察碰撞和夹取效果。
- `launch/demo_x1_standard_real.launch.py`
  唯一真实硬件 MoveIt 启动入口。该链路会启动 `zyarm_hardware_interface` 插件、controller manager、`move_group` 和 RViz，并通过标准 `FollowJointTrajectory` 动作接口执行真机轨迹。

使用 `demo_x1_standard_gazebo.launch.py` 前，请先确认当前环境已具备 `ros_gz_sim`、`ros_gz_bridge` 和 Gazebo 相关 ROS 集成；若缺失，launch 会在解析或启动阶段失败。

建议阅读顺序：

1. 先看 `demo_x1_standard.launch.py`、`demo_x1_standard_gazebo.launch.py` 和 `demo_x1_standard_real.launch.py`，理解 MoveIt 如何接到 mock、Gazebo 和真机 ros2_control 路径。
2. 再看 `zyarm_x1_standard.srdf` 与 `zyarm_x1_standard_gazebo.srdf`，理解普通链路和 Gazebo 双指夹爪链的差异。
3. 再看 `kinematics.yaml`、`joint_limits.yaml`、`ompl_planning.yaml`，理解规划本身的边界条件。
4. 最后看 `moveit_controllers.yaml`、`moveit_gazebo_controllers.yaml` 和 `moveit_real_controllers.yaml`，理解规划结果如何落到动作接口。

真机路径注意事项：

- `demo_x1_standard_real.launch.py` 是唯一真机 MoveIt 入口，会独占串口并由 `zyarm_hardware_interface` 直接发送 CMD36。
- 该入口支持 `serial_port`、`baud_rate`、状态超时、关节映射和夹爪映射等真机硬件参数。
- `zyarm_hardware` 仍用于非 MoveIt 的硬件服务、遥操和学习链路。
