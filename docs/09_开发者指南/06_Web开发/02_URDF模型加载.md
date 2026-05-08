# URDF 模型加载

URDF 是机器人模型描述文件，定义连杆、关节、mesh 和坐标关系。Web 工程通过 URDF Loader 在浏览器中加载机械臂模型。

## 当前资源位置

```text
web/public/URDF_XG_Robot_Arm_Step_Urdf_V1/
├── urdf/
├── meshes/
├── config/
└── launch/
```

Web 中的模型资源和 ROS 2 `zyarm_description` 中的模型资源可能存在历史差异。修改模型前需要确认两边是否应该同步。

## 修改模型时检查

- URDF 中的 mesh 路径是否能被浏览器访问。
- joint 名称是否和控制逻辑一致。
- 关节轴向、零位和限制是否符合实际控制语义。
- 模型缩放和坐标方向是否正确。
- 资源文件名是否和当前 `ZYArm-X1` / `x1_standard` 命名保持一致。

## 与 ROS 2 模型的关系

ROS 2 模型入口在：

```text
software/ros2_ws/src/zyarm_description/
```

如果 Web 和 ROS 2 都依赖同一套机械臂模型，建议建立明确的同步规则，不要长期维护两套含义不同的 URDF。

## 待项目方补充

- Web 与 ROS 2 是否共享同一套正式 URDF 的决策。
- Web 与 ROS 2 之间的正式模型资源同步规则。
- 模型导出流程、mesh 精度和材质规范。
