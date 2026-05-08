# MoveIt 配置

MoveIt 负责运动规划、碰撞检查和 RViz 交互。对 ZYArm 来说，它把“我希望末端或关节到某个位置”转换成一段关节轨迹，再通过 `joint_trajectory_controller` 执行到 mock、Gazebo 或真实机械臂。

ZYArm 的 MoveIt 配置位于 `software/ros2_ws/src/zyarm_moveit_config/`。

## 先认识这些文件

| 文件或目录 | 简短说明 |
| --- | --- |
| `srdf/` | MoveIt 语义模型，定义规划组、末端执行器、预设姿态和禁用碰撞 |
| `config/kinematics.yaml` | 逆解求解器配置，当前使用 KDL |
| `config/joint_limits.yaml` | MoveIt 规划时使用的速度和加速度限制 |
| `config/ompl_planning.yaml` | OMPL 采样规划器配置 |
| `config/moveit_controllers.yaml` | mock / 基础 ros2_control 路径的控制器映射 |
| `config/moveit_real_controllers.yaml` | 真机路径的控制器映射 |
| `config/moveit_gazebo_controllers.yaml` | Gazebo 路径的控制器映射 |
| `launch/demo_x1_standard.launch.py` | MoveIt + mock / 基础 ros2_control 演示入口 |
| `launch/demo_x1_standard_real.launch.py` | MoveIt + 真机入口 |
| `launch/demo_x1_standard_gazebo.launch.py` | MoveIt + Gazebo 入口 |

`joint_limits.yaml` 里的限制主要影响 MoveIt 规划和时间参数化，不等价于固件内部极限。真机安全边界仍需要同时考虑固件、机械结构、控制器配置和实际摆放环境。

## 规划到执行的链路

MoveIt 配置最终会落到控制器 action：

```text
URDF / Xacro
  -> robot_description
SRDF / kinematics / joint_limits / OMPL
  -> move_group
RViz MotionPlanning
  -> Plan
  -> FollowJointTrajectory
  -> arm_controller / gripper_controller
  -> ros2_control / Gazebo / 真机
```

MoveIt 不直接打开串口。真机串口由 `zyarm_hardware_interface` 在 `ros2_control_node` 中持有。

## 三个 MoveIt 启动入口

| 启动入口 | 使用场景 | 关键差异 |
| --- | --- | --- |
| `demo_x1_standard.launch.py` | 本地 MoveIt 和基础 ros2_control 联调 | 不接真实硬件，不启 Gazebo |
| `demo_x1_standard_real.launch.py` | 真机 MoveIt 控制 | `use_sim_time=false`，加载 `zyarm_hardware_interface` |
| `demo_x1_standard_gazebo.launch.py` | Gazebo + MoveIt 仿真 | `use_sim_time=true`，夹爪使用 `joint6` + `joint7` |

真机运行：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_moveit_config demo_x1_standard_real.launch.py serial_port:=/dev/ttyUSB0
```

Gazebo 仿真运行：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_moveit_config demo_x1_standard_gazebo.launch.py
```

## SRDF 里定义了什么

`srdf/zyarm_x1_standard.srdf` 是真机和普通 MoveIt 演示使用的语义模型，当前主要包含：

| 配置 | 当前含义 |
| --- | --- |
| `group name="arm"` | 从 `base_link` 到 `link6` 的机械臂规划组 |
| `group name="gripper"` | 夹爪规划组，真机路径只包含 `joint6` |
| `group_state home` | 机械臂 home 预设姿态 |
| `group_state open/closed` | 夹爪打开和闭合预设 |
| `end_effector gripper_eef` | 末端执行器声明 |
| `virtual_joint world_joint` | `world` 到 `base_link` 的固定关系 |
| `disable_collisions` | 禁用相邻连杆等不需要检查的碰撞对 |

`srdf/zyarm_x1_standard_gazebo.srdf` 是 Gazebo 专用版本。因为仿真夹爪把双指分成两个可控关节，所以 `gripper` 组会同时包含 `joint6` 和 `joint7`。

## 控制器映射

MoveIt 通过 `moveit_simple_controller_manager` 找到控制器 action。当前主要有两个控制器：

| 控制器 | action 类型 | 关节 |
| --- | --- | --- |
| `arm_controller` | `FollowJointTrajectory` | `joint0..joint5` |
| `gripper_controller` | `FollowJointTrajectory` | 真机 `joint6`，Gazebo `joint6` + `joint7` |

这里的控制器名称必须和 `zyarm_control/config/*.yaml` 中启动的 controller 名称一致。否则 RViz 里能规划，但执行时会找不到 action 或 controller。

## RViz 最小操作流程

启动 `demo_x1_standard.launch.py`、`demo_x1_standard_real.launch.py` 或 `demo_x1_standard_gazebo.launch.py` 后，RViz 中通常按下面流程验证：

1. 在 MotionPlanning 面板选择 `Planning Group`。
2. 先选择 `arm`，确认当前状态和模型显示正常。
3. 设置一个离当前姿态较近的目标状态。
4. 点击 `Plan`，确认能生成轨迹。
5. 真机模式下先不要急着执行，确认机械臂周围无障碍，并且可以随时断电。
6. 点击 `Execute`，观察 `/joint_states`、RViz 模型和真实机械臂或 Gazebo 是否同步变化。
7. 再切换到 `gripper` 组验证夹爪打开和闭合。

如果只是调试配置，优先在 `demo_x1_standard.launch.py` 或 `demo_x1_standard_gazebo.launch.py` 中验证。真机执行前，应先确认底层 ros2_control 链路稳定。

## 修改场景

| 修改目标 | 需要同步检查 |
| --- | --- |
| 改 joint 名称 | URDF/Xacro、SRDF、controller 配置、MoveIt controller 映射、RViz 配置 |
| 改 link 或末端名称 | SRDF 的 planning group、end effector、RViz fixed frame |
| 改夹爪结构 | `zyarm_x1_standard.srdf`、`zyarm_x1_standard_gazebo.srdf`、MoveIt controller、Gazebo controller |
| 改速度或加速度限制 | URDF limit、`joint_limits.yaml`、controller 约束和真机安全验证 |
| 改控制器名称 | `zyarm_control/config/*.yaml` 和 `moveit_*_controllers.yaml` |
| 改真机硬件参数 | `demo_x1_standard_real.launch.py`、`bringup_x1_standard_real_ros2_control.launch.py`、Xacro `<ros2_control>` 参数 |
| 改 Gazebo 场景 | `demo_x1_standard_gazebo.launch.py`、`zyarm_gazebo/config/world.yaml`、Gazebo controller 映射 |

## 真机和 Gazebo 的差异

| 项目 | 真机 | Gazebo |
| --- | --- | --- |
| 时间 | `use_sim_time=false` | `use_sim_time=true` |
| 硬件插件 | `zyarm_hardware_interface/ZyArmSystemHardware` | `gz_ros2_control/GazeboSimSystem` |
| 夹爪关节 | `joint6` | `joint6` 和 `joint7` |
| gripper controller | 单关节 | 双关节 |
| 状态来源 | 固件 `[STATUS]` | Gazebo 物理仿真 |
| 风险 | 会驱动真实机械臂 | 主要是仿真资源和模型稳定性 |

这个差异是很多配置问题的来源。看到夹爪不能执行时，先确认当前用的是 `moveit_real_controllers.yaml` 还是 `moveit_gazebo_controllers.yaml`。

## 调试命令

查看 MoveIt 是否暴露 action：

```bash
ros2 action list | grep follow_joint_trajectory
```

查看控制器状态：

```bash
ros2 control list_controllers -c /zyarm_x1_standard_controller_manager
```

查看关节状态：

```bash
ros2 topic echo /joint_states --once
```

查看当前 TF：

```bash
ros2 run tf2_tools view_frames
```

`view_frames` 会生成 TF 关系图，适合检查 `world`、`base_link`、`link6` 和 `ee_link` 是否存在。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| RViz 中没有机器人模型 | `robot_description` 是否生成成功，Xacro 路径是否正确 |
| 能显示模型但不能规划 | SRDF planning group、kinematics 配置、joint limits 是否正确 |
| 能规划但不能执行 | MoveIt controller 映射和 controller action 是否匹配 |
| 真机模式执行失败 | `arm_controller` / `gripper_controller` 是否 active，串口是否稳定 |
| Gazebo 模式夹爪异常 | 是否使用 Gazebo 专用 SRDF 和 `moveit_gazebo_controllers.yaml` |
| RViz 模型和 Gazebo 模型位置不一致 | `world` 到 `base_link` 的静态 TF 是否和 `world.yaml` 中底座姿态一致 |

## 验证建议

修改 MoveIt 配置后建议按这个顺序验证：

1. 先启动 `zyarm_description display_x1_standard.launch.py`，确认模型、mesh 和 TF。
2. 再启动 `demo_x1_standard.launch.py`，确认 MoveIt 能规划。
3. Gazebo 相关修改再启动 `demo_x1_standard_gazebo.launch.py`。
4. 真机修改最后启动 `demo_x1_standard_real.launch.py`。
5. 真机执行前，先让机械臂处于安全初始姿态，空载、低风险、小范围验证。

## 待项目方补充

- 官方推荐规划组和末端执行器命名。
- 真机 MoveIt 的标准规划目标和安全验证视频。
- `x1_standard` MoveIt 配置的标准验证动作和命名维护规则。
