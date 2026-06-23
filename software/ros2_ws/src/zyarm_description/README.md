`zyarm_description` 负责机械臂模型本体，是控制、规划、可视化等所有上层功能的基础。

目录说明：

- `urdf/x1_standard`
  当前 ZYArm-X1 标准模型的 ROS 2 xacro 入口和源 URDF 参考。
- `urdf/x1_plus`
  ZYArm-X1 Plus 模型的 ROS 2 xacro 入口和源 URDF 参考，控制语义参考 `x1_standard`。
- `meshes/x1_standard`
  当前标准模型的 STL 网格资源。
- `meshes/x1_plus`
  Plus 模型的 STL 网格资源。
- `config/x1_standard`
  与当前标准模型绑定的关节名称、Gazebo 相机 frame 等辅助配置。
- `config/x1_plus`
  与 Plus 模型绑定的关节名称配置；Plus 首版暂不提供 Gazebo 相机配置。
- `launch/display_x1_standard.launch.py`
  当前标准模型的独立显示入口，适合只验证模型、关节和 TF。
- `launch/display_x1_plus.launch.py`
  Plus 模型的独立显示入口，适合只验证模型、关节和 TF。
- `rviz/display_x1_standard.rviz`
  模型显示时使用的 RViz 预设。
- `rviz/display_x1_plus.rviz`
  Plus 模型显示时使用的 RViz 预设。

当前重点：

- `urdf/x1_standard/robot.urdf.xacro` 与 `urdf/x1_plus/robot.urdf.xacro` 是当前两个 ROS 2 运行模型入口。
- 两个 xacro 都预留 `use_ros2_control` 开关，可在显示模式和控制模式之间切换。
- `x1_plus` 首版支持 display、mock ros2_control 和真实硬件 ros2_control，不提供 Gazebo 语义。
- `launch/display_x1_standard.launch.py` 和 `launch/display_x1_plus.launch.py` 可直接启动 `joint_state_publisher_gui + robot_state_publisher + RViz2`。

建议阅读顺序：

1. 先看 `launch/display_x1_standard.launch.py` 或 `launch/display_x1_plus.launch.py`，理解模型如何被加载和显示。
2. 再看对应型号的 `robot.urdf.xacro`，理解关节链、夹爪和 `ros2_control` 预留接口。
