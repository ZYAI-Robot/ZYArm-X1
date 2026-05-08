# ROS 2 / MoveIt / Gazebo 环境

ROS 2、MoveIt 和 Gazebo 推荐在 Ubuntu 上使用。本页说明 ROS 2 Jazzy 安装、ZYArm ROS 2 工作区依赖和编译验证；具体模型显示、MoveIt 控制、Gazebo 仿真和真机启动入口见 [仿真与框架接入](../06_仿真与框架接入/README.md)。

如果你第一次接触 ROS，可以先把它理解成一套机器人软件基础设施：ROS 2 负责节点通信和工具生态，MoveIt 负责机械臂运动规划，Gazebo 负责仿真环境，ros2_control 负责把控制器和真实或仿真的机械臂连接起来。

## 适用场景

- 使用 ROS 2 查看模型和 `/joint_states`。
- 使用 `ros2_control` 连接真机。
- 使用 MoveIt 做规划和执行。
- 使用 Gazebo 做仿真验证。
- 开发或调试 ROS 2 包。

## 先认识这些工具

| 工具 | 简短说明 | 在 ZYArm 中的作用 |
| --- | --- | --- |
| ROS 2 Jazzy | 机器人软件框架的发行版 | 提供节点、话题、服务、launch、包管理等基础能力 |
| Ubuntu 24.04 Noble | 推荐的 Linux 系统版本 | ROS 2 Jazzy 的官方 deb 安装目标平台 |
| colcon | ROS 2 工作区构建工具 | 编译 `software/ros2_ws` 中的多个 ROS 2 包 |
| rosdep | ROS 依赖安装工具 | 根据 `package.xml` 自动安装缺失的系统依赖 |
| xacro | URDF 模型模板工具 | 生成机械臂模型描述 |
| RViz | ROS 可视化工具 | 查看机械臂模型、TF、规划结果和状态 |
| ros2_control | ROS 2 控制框架 | 管理控制器，并连接仿真或真机硬件接口 |
| MoveIt | 机械臂运动规划框架 | 做关节规划、路径规划和执行 |
| Gazebo / ros_gz | 仿真器和 ROS 2 桥接 | 在仿真世界里验证机械臂和相机等场景 |

这一页的目标不是让你完全学会这些工具，而是先把它们安装好，并确认 ZYArm 的 ROS 2 工作区能编译、能启动最低风险的模型显示。

## 推荐环境

当前仓库 README 以 ROS 2 Jazzy 为主，推荐系统为 Ubuntu 24.04 Noble。ROS 2 Jazzy 官方安装说明见 [Ubuntu deb packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

本页默认使用 apt deb 包安装，不覆盖源码编译 ROS 2 的路径。

工作区位于：

```text
software/ros2_ws/
```

## 安装 ROS 2 Jazzy

这一节先安装 ROS 2 本体。安装完成后，系统会得到 `ros2`、`rviz2`、基础消息包、launch 工具和常用示例。

设置 locale：

```bash
locale
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

启用 Universe 仓库：

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

添加 ROS 2 apt 源：

```bash
sudo apt update
sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

安装 ROS 2 Jazzy Desktop 和开发工具：

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop ros-dev-tools
```

加载环境：

```bash
source /opt/ros/jazzy/setup.bash
```

可以把这行加入 `~/.bashrc`，避免每次开新终端都手动执行：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

验证 ROS 2：

```bash
ros2 pkg list | head
```

也可以打开两个终端运行 talker/listener 示例，确认 C++ 和 Python 节点都能工作。

## 安装项目依赖

ROS 2 Desktop 只提供基础能力。ZYArm 还会用到机械臂控制、规划、仿真和串口桥接，所以还需要安装一些额外包。

从 `software/ros2_ws/src/*/package.xml` 和当前 launch 入口看，ZYArm 工作区会用到 ros2_control、MoveIt、Gazebo/ros_gz、RViz、xacro、接口生成、串口和 YAML 等依赖。建议先安装下面这组包：

```bash
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  python3-serial \
  python3-yaml \
  python3-pytest
```

其中：

- `ros-jazzy-ros2-control` / `ros-jazzy-ros2-controllers`：让 ROS 2 能加载控制器，例如关节状态广播器和轨迹控制器。
- `ros-jazzy-moveit`：安装 MoveIt 2，用于机械臂运动规划和 RViz MotionPlanning 插件。
- `ros-jazzy-ros-gz`：安装 ROS 2 和 Gazebo 之间的桥接工具。
- `ros-jazzy-gz-ros2-control`：让 Gazebo 里的机械臂也能接 ros2_control 控制链。
- `python3-serial` / `python3-yaml`：供 `zyarm_hardware` 访问串口和读取 YAML 配置。

ros2_control 安装说明见 [ros2_control Jazzy](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)，Gazebo ros2_control 安装说明见 [gz_ros2_control Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。

## 使用 rosdep 兜底

`package.xml` 是每个 ROS 2 包声明依赖的文件。`rosdep` 会读取这些文件，并尝试用 apt 安装对应依赖。

手工 apt 清单可能会随着仓库演进而变化。进入工作区后，建议再用 `rosdep` 按 `package.xml` 自动补齐依赖：

```bash
cd software/ros2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

如果 `rosdep` 提示某些本仓库包找不到，例如 `zyarm_description`、`zyarm_control`，这是正常的本地源码包，不需要通过 apt 安装。

## 依赖来源

下面这张表不是新手必须背下来的内容，只是说明“为什么要装上面那些包”。当前 ROS 2 工作区中比较关键的外部依赖包括：

| 类别 | package.xml 依赖 |
| --- | --- |
| ROS 2 基础 | `ament_cmake`、`ament_python`、`launch`、`launch_ros`、`rclpy`、`rclcpp`、`rclcpp_lifecycle` |
| 模型与可视化 | `xacro`、`robot_state_publisher`、`joint_state_publisher`、`joint_state_publisher_gui`、`rviz2`、`tf2_ros` |
| ros2_control | `controller_manager`、`ros2controlcli`、`hardware_interface`、`joint_state_broadcaster`、`joint_trajectory_controller` |
| MoveIt | `moveit_configs_utils`、`moveit_kinematics`、`moveit_planners_ompl`、`moveit_ros_move_group`、`moveit_ros_visualization`、`moveit_simple_controller_manager` |
| Gazebo / ros_gz | `ros_gz_sim`、`ros_gz_bridge`、`ros_gz_interfaces`、`gz_ros2_control` |
| 消息与接口 | `sensor_msgs`、`std_msgs`、`trajectory_msgs`、`control_msgs`、`controller_manager_msgs`、`rosidl_default_generators`、`rosidl_default_runtime` |
| Python / 测试 | `python3-yaml`、`python3-pytest`、`ament_cmake_pytest`、`ament_cmake_gtest` |

> 待项目方确认：`zyarm_hardware` 当前通过 `setup.py` 声明 `pyserial` 和 `PyYAML`，后续是否也需要在 `package.xml` 中显式补充 `python3-serial` 和 `python3-yaml`。

## 基础工具检查

安装完成后，先不要急着启动真机，先确认工具链都在。

确认 colcon：

```bash
colcon --help
```

确认关键包可见：

```bash
ros2 pkg list | grep -E "moveit|ros_gz|gz_ros2_control|controller_manager"
```

## 编译工作区

`software/ros2_ws` 是一个 ROS 2 工作区。里面有 `zyarm_description`、`zyarm_control`、`zyarm_hardware`、`zyarm_moveit_config` 等多个包，需要先编译后再使用。

在仓库根目录进入 ROS 2 工作区：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译核心包：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select zyarm_description zyarm_control zyarm_hardware zyarm_hardware_interface zyarm_moveit_config zyarm_bringup zyarm_interfaces
source install/setup.bash
```

新开终端后通常需要重新加载：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 环境验证

编译完成并 `source install/setup.bash` 后，先验证 ZYArm 包是否可见：

```bash
ros2 pkg list | grep zyarm
```

再选择低风险入口，例如只看模型显示。这个入口不会直接驱动真实机械臂，适合作为 ROS 环境的第一步验证：

```bash
ros2 launch zyarm_description display_x1_standard.launch.py
```

涉及真机的 launch 之前，请先完成 [快速上手](../02_快速上手/README.md)，确认串口可访问、状态可读取、机械臂可复位。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `ros2` 命令不可用 | 是否 `source /opt/ros/jazzy/setup.bash` |
| `colcon` 命令不可用 | 是否安装 `ros-dev-tools` |
| 找不到 zyarm 包 | 是否完成 `colcon build` 和 `source install/setup.bash` |
| MoveIt 入口失败 | 是否安装 `ros-jazzy-moveit` |
| Gazebo 入口失败 | 是否安装 `ros-jazzy-ros-gz` 和 `ros-jazzy-gz-ros2-control` |
| 真机入口打不开串口 | `/dev/ttyUSBx` 是否正确、是否有权限 |
| launch 找不到 `x1_standard` 入口 | 是否完成最新代码构建并执行 `source install/setup.bash` |

## 下一步

- 框架入口和 launch 说明：[仿真与框架接入](../06_仿真与框架接入/README.md)
- ROS 2 工作区说明：[software/ros2_ws/README.md](../../software/ros2_ws/README.md)
- ROS 2 包职责：[software/ros2_ws/src/README.md](../../software/ros2_ws/src/README.md)

## 待补充

> 待项目方补充：请提供已验证 Ubuntu 镜像、ROS 2 Jazzy 安装录屏、MoveIt / Gazebo / ros_gz 实测版本、显卡/仿真建议、常见编译错误截图和一键安装脚本。
