# zyarm_gazebo

这个包提供 ZyArm ACT 流程验证所需的 Gazebo 仿真场景、模型模板和场景入口。

当前场景包含：

- 一张桌面
- 一个抓取方块
- 一个固定放置区域
- 一台固定到底座的 `zyarm_x1_standard` 仿真机械臂
- 两路仿真 RGB 相机：
  `/camera_fixed/color/image_raw`
  `/camera_wrist/color/image_raw`

## 主要文件

- `launch/bringup_pick_place_world.launch.py`
  启动 Gazebo 抓取场景、`robot_state_publisher`、`ros_gz_bridge` 和仿真控制器链。
- `worlds/pick_place_world.sdf`
  Gazebo 世界模板。当前通过 world-level `include + fixed joint` 把机械臂固定到世界坐标，避免模型闪烁或掉出场景。
- `config/world.yaml`
  场景源配置。当前桌面、方块、目标区域和机械臂固定基座位姿都以这里为准。

## 启动前准备

先 source ROS 和工作区：

```bash
source /opt/ros/jazzy/setup.bash
source /home/xg/workspace/zyarmv1/software/ros2_ws/install/setup.bash
```

如果之前起过多个 Gazebo，建议先清理旧进程，避免旧 world 残留影响当前场景：

```bash
pkill -f 'gz sim' || true
```

## 常用启动命令

只启动 Gazebo 抓取场景：

```bash
ros2 launch zyarm_gazebo bringup_pick_place_world.launch.py
```

当前 Gazebo 包只保留仿真世界、相机桥接和控制器链验证入口。

## 当前验证状态

2026-04-18 已验证：

- Gazebo 世界可正常启动
- `zyarm_x1_standard` 固定基座在 world 中稳定，不再出现“闪一下就消失”或掉出场景的问题
- `/camera_fixed/color/image_raw` 与 `/camera_wrist/color/image_raw` 桥接配置已接通
- `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 能进入 `active`
- `/arm_controller/follow_joint_trajectory` 与 `/gripper_controller/follow_joint_trajectory` 可见

这轮修复后的关键实现：

- 机械臂不再通过 `ros_gz_sim create -file` 动态插入 world
- 改为先渲染模型，再由 `pick_place_world.sdf` 直接 `include`
- 通过 world-level fixed joint 把 `zyarm_x1_standard::base_link` 固定到世界坐标
- `config/world.yaml` 中 `robot_base_pose` 当前为 `[0.12, 0.0, 0.5038, 0.0, 0.0, 0.0]`

## 夹爪说明

- Gazebo 路径不再依赖 `joint7` 的 mimic constraint。
- 当前改为由 `gripper_controller` 同时控制 `joint6` 和 `joint7`，避免当前物理引擎不支持 mimic constraint 导致单侧夹爪不动。
