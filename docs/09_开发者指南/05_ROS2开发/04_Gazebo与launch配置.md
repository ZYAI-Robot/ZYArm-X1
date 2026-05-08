# Gazebo 与 launch 配置

Gazebo 用于物理仿真、任务场景和算法调试。launch 文件负责把模型、控制器、仿真世界、桥接节点、MoveIt 和 RViz 组合成一个可运行系统。

本页重点说明 ZYArm 当前的 Gazebo 抓取场景和相关 launch 结构。它适合在不连接真实机械臂的情况下验证模型、控制器、相机、夹爪和 MoveIt 规划链路。

## 先认识这些工具

| 工具或概念 | 简短说明 |
| --- | --- |
| Gazebo | 物理仿真器，负责运行 world、模型、碰撞、传感器和插件 |
| SDF | Gazebo 使用的场景和模型描述格式 |
| Xacro | 生成 URDF 的模板语言，适合给模型传入参数 |
| `gz_ros2_control` | Gazebo 和 ros2_control 的连接插件 |
| `ros_gz_bridge` | Gazebo topic 和 ROS 2 topic 之间的桥接工具 |
| contact sensor | 接触传感器，用来判断夹爪是否碰到物体 |
| detachable joint | Gazebo 插件能力，用来模拟抓取后物体附着到末端 |
| launch | ROS 2 启动编排文件，负责一次性拉起多个节点和参数 |

如果只是第一次学习 Gazebo，可以先把它理解成“不会伤到真实机械臂的实验场”。MoveIt 负责规划，Gazebo 负责模拟物体和机械臂是否真的动起来。

## 当前仿真链路

当前 Gazebo + MoveIt 入口会组合下面这些内容：

```text
world.yaml
  -> 渲染 pick_place_world.sdf
  -> 渲染 pickup_cube / goal_zone 模型
  -> Xacro 生成 zyarm_x1_standard URDF
  -> gz sdf 转成 zyarm_x1_standard SDF
  -> 注入双指 contact sensor 和 detachable joint
  -> 启动 Gazebo world
  -> 启动 gz_ros2_control controllers
  -> 桥接相机、接触、抓取状态
  -> 启动 MoveIt 和 RViz
```

这条链路的关键点是：场景不是完全静态写死的，launch 会根据 `world.yaml` 动态渲染一部分 world 和 model 文件到临时目录。

## 主要文件

| 位置 | 作用 |
| --- | --- |
| `zyarm_gazebo/config/world.yaml` | 桌面、方块、目标区和机械臂底座姿态 |
| `zyarm_gazebo/worlds/pick_place_world.sdf` | Gazebo world 模板 |
| `zyarm_gazebo/models/pickup_cube/` | 可抓取方块模型 |
| `zyarm_gazebo/models/goal_zone/` | 目标区域模型 |
| `zyarm_gazebo/launch/bringup_pick_place_world.launch.py` | Gazebo 场景启动入口 |
| `zyarm_gazebo/scripts/grasp_manager.py` | 抓取状态管理脚本 |
| `zyarm_description/urdf/x1_standard/robot.urdf.xacro` | 机械臂模型，支持 Gazebo、相机和 ros2_control 参数 |
| `zyarm_moveit_config/launch/demo_x1_standard_gazebo.launch.py` | MoveIt + Gazebo + RViz 总入口 |

## 两个启动入口

只启动 Gazebo 抓取场景：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_gazebo bringup_pick_place_world.launch.py
```

启动 Gazebo + MoveIt + RViz：

```bash
cd software/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zyarm_moveit_config demo_x1_standard_gazebo.launch.py
```

第二个入口会包含第一个入口，并额外启动 `move_group`、RViz 和 `world` 到 `base_link` 的静态 TF。

## world.yaml 怎么读

`world.yaml` 是调整仿真任务最直接的入口：

| 字段 | 作用 |
| --- | --- |
| `table.size` | 桌面尺寸 |
| `table.pose` | 桌面位姿 |
| `cube.size` | 方块尺寸 |
| `cube.pose` | 方块初始位姿 |
| `cube.mass` | 方块质量 |
| `goal_zone.size` | 目标区域尺寸 |
| `goal_zone.pose` | 目标区域位姿 |
| `robot_base_pose` | 机械臂底座在 world 中的位姿 |

这些值会被 `bringup_pick_place_world.launch.py` 渲染进 world 和模型文件。修改后重新启动 launch 即可生效。

## Gazebo 里的夹爪为什么是双关节

真实机械臂文档里通常把夹爪看成一个主控制量，但 Gazebo 物理仿真里双指接触更敏感。当前 Gazebo 路径会让夹爪使用 `joint6` 和 `joint7` 两个关节：

- `joint6` 控制一侧夹爪。
- `joint7` 控制另一侧夹爪。
- Gazebo 专用 SRDF 和 controller 都包含这两个关节。
- 真机路径仍只使用 `joint6`。

所以 MoveIt 配置分成了：

- 真机：`zyarm_x1_standard.srdf` + `moveit_real_controllers.yaml`
- Gazebo：`zyarm_x1_standard_gazebo.srdf` + `moveit_gazebo_controllers.yaml`

## 相机和桥接 topic

Gazebo 场景当前包含固定相机和腕部相机。launch 会通过 `ros_gz_bridge` 把 Gazebo 图像桥接到 ROS 2：

| ROS 2 topic | 含义 |
| --- | --- |
| `/camera_fixed/color/image_raw` | 外置固定相机图像 |
| `/camera_wrist/color/image_raw` | 腕部相机图像 |
| `/clock` | 仿真时钟 |
| `/zyarm/grasp/state` | 抓取状态 |
| `/zyarm/grasp/attach` | 请求附着物体 |
| `/zyarm/grasp/detach` | 请求释放物体 |

确认 topic 是否存在：

```bash
ros2 topic list | grep camera
ros2 topic list | grep grasp
```

查看抓取状态：

```bash
ros2 topic echo /zyarm/grasp/state
```

## 抓取逻辑

当前 Gazebo 抓取逻辑不是单纯靠物理摩擦硬夹，而是混合使用接触传感器和 detachable joint：

1. 左右夹爪 contact sensor 判断是否接触方块。
2. `grasp_manager.py` 监听接触和夹爪开合状态。
3. 满足条件后，通过 detachable joint 把 `pickup_cube::cube_link` 附着到末端。
4. 夹爪打开到阈值后，发送 detach 释放方块。

这套逻辑适合课程和算法验证，因为它比纯物理抓取更稳定，也更容易复现实验结果。

## launch 文件做了什么

`bringup_pick_place_world.launch.py` 主要做这些事情：

| 步骤 | 作用 |
| --- | --- |
| 读取 `world.yaml` | 获取桌面、方块、目标区和底座位姿 |
| 创建临时渲染目录 | 避免直接修改源码中的 world/model |
| 渲染 world 和任务模型 | 把 YAML 参数写入 SDF |
| 生成机器人 SDF | 由 Xacro 生成 URDF，再用 `gz sdf -p` 转换 |
| 注入接触传感器 | 给 `claw1` 和 `claw2` 增加 contact sensor |
| 注入 detachable joint | 支持抓取附着和释放 |
| 设置 `GZ_SIM_RESOURCE_PATH` | 让 Gazebo 找到临时模型和 description 包 |
| 启动 controller spawner | 拉起 `joint_state_broadcaster`、`arm_controller`、`gripper_controller` |
| 启动 bridge 和 grasp manager | 桥接图像/接触/抓取 topic，并管理抓取状态 |

`demo_x1_standard_gazebo.launch.py` 会在此基础上再启动 MoveIt 和 RViz。

## 修改场景

| 想改什么 | 优先修改 |
| --- | --- |
| 桌面位置或尺寸 | `zyarm_gazebo/config/world.yaml` |
| 方块位置、尺寸或质量 | `zyarm_gazebo/config/world.yaml` |
| 目标区域 | `zyarm_gazebo/config/world.yaml` 和 `models/goal_zone/` |
| 相机位姿 | `zyarm_description/urdf/x1_standard/robot.urdf.xacro` 和相机 frame 配置 |
| 相机 topic | Xacro 中的相机配置和 `bringup_pick_place_world.launch.py` 桥接参数 |
| 抓取判定 | `scripts/grasp_manager.py` 参数和 contact topic |
| 控制器频率 | `zyarm_control/config/zyarm_x1_standard_gazebo_controllers.yaml` |
| MoveIt 夹爪配置 | `zyarm_moveit_config/srdf/zyarm_x1_standard_gazebo.srdf` 和 `moveit_gazebo_controllers.yaml` |

## 验证顺序

修改 Gazebo 相关内容后，建议按下面顺序验证：

1. 只启动 `zyarm_gazebo bringup_pick_place_world.launch.py`，确认 Gazebo world 能打开。
2. 查看 `ros2 topic list`，确认相机、抓取和 `/clock` topic 存在。
3. 查看 `/joint_states`，确认 controller 已经发布状态。
4. 再启动 `demo_x1_standard_gazebo.launch.py`，确认 RViz 和 MoveIt 能连接到同一套 controller。
5. 在 RViz 中先规划小范围关节动作，再验证夹爪和抓取。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| launch 解析失败 | Gazebo、`ros_gz_sim`、`ros_gz_bridge`、`xacro` 是否安装 |
| Gazebo 打开但没有机器人 | `GZ_SIM_RESOURCE_PATH`、临时模型渲染、description 包路径 |
| RViz 和 Gazebo 位置不一致 | `world.yaml` 的 `robot_base_pose` 和静态 TF 是否一致 |
| 没有相机 topic | `use_sim_cameras:=true` 是否生效，bridge 参数是否启动 |
| 夹爪能动但抓不起方块 | contact sensor、`grasp_manager.py`、detach/attach topic 和方块碰撞体 |
| controller 没有 active | `wait_for_controller_types.py`、controller yaml 和 `gz_ros2_control` 插件 |
| 运行变慢 | Gazebo 物理步进、相机分辨率、系统显卡和 CPU 负载 |

## 待项目方补充

- Gazebo 任务场景的官方验收标准。
- 仿真中夹爪、摄像头、桌面物体的推荐参数。
- 面向课程或比赛的标准 world 和评分规则。
