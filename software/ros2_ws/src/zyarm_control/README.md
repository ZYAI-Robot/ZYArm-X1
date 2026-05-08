`zyarm_control` 负责 `ros2_control` 这一层的控制器配置，重点是把模型里的关节接口映射成可运行的控制器。

当前主要文件：

- `config/controller_contract.yaml`
  控制器契约的单一事实来源。这里统一定义控制器身份、标准动作语义、默认关节分组和 Gazebo 投影。
- `config/zyarm_x1_standard_controllers.yaml`
  `x1_standard` 机械臂在非 Gazebo 模式下的控制器投影。
- `config/zyarm_x1_standard_gazebo_controllers.yaml`
  Gazebo 双指夹爪模式下的控制器投影。
- `config/zyarm_x1_standard_real_controllers.yaml`
  真机 ros2_control 控制器投影，默认 50Hz，state interface 只使用 position。

这层的作用：

- 给 `ros2_control_node` 提供控制器类型和参数。
- 给 MoveIt 和其他上层节点提供统一的控制器身份与动作接口名字。
- 把机械臂 6 个旋转关节和夹爪关节拆成两个独立控制器，并显式区分 default / Gazebo 两种关节投影。
- 真实硬件标准控制语义只通过 `zyarm_hardware_interface`、controller manager 和标准 `joint_trajectory_controller` 提供。

建议阅读顺序：

1. 先看 `controller_contract.yaml`，理解控制器身份、动作语义和不同运行模式下的关节分组。
2. 再看 `zyarm_x1_standard_controllers.yaml`，理解 non-gazebo/default 模式如何把契约投影成 `ros2_control` 参数。
3. 再看 `zyarm_x1_standard_gazebo_controllers.yaml`，理解 Gazebo 双指夹爪投影与默认模式的差异。
4. 最后看 `zyarm_x1_standard_real_controllers.yaml`，理解真机 50Hz position-only 控制器投影。
