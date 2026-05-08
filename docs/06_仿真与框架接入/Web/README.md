# Web 接入

本页说明如何使用 Web 控制端进行模型展示、URDF 加载、网页仿真和可视化体验。Web 路线适合给学习者提供直观交互入口，也适合后续扩展成浏览器端控制台。

如果你要修改 Web 项目结构、URDF 加载逻辑、前端交互或真机控制接入，请阅读 [Web 开发](../../09_开发者指南/06_Web开发/README.md)。本页只保留使用入口。

## Web 接入能做什么

- 在浏览器中查看机械臂模型。
- 观察 URDF 模型、关节和连杆关系。
- 做 Web 驱动仿真或拖拽交互体验。
- 后续通过 WebSocket、MQTT、HTTP 等链路接入上位机服务。

当前 Web 项目使用 Vue、Three.js、URDF Loader、Pinia、Element Plus 等依赖。

| 工具 | 简短说明 |
| --- | --- |
| Vue | Web 前端框架，用于组织页面和组件 |
| Three.js | 浏览器 3D 渲染库，用于显示机械臂模型 |
| URDF Loader | 加载机器人 URDF 模型的工具 |
| Pinia | 前端状态管理工具 |
| Element Plus | 前端 UI 组件库 |
| pnpm | 前端包管理和脚本运行工具 |

## 最小启动

```bash
cd web
pnpm install
pnpm run dev
```

启动后，根据终端输出打开本地开发地址。常见地址类似 `http://localhost:5173`，实际端口以终端输出为准。

## 使用前确认

- 已安装 Node.js 和 pnpm。
- `pnpm install` 能正常安装依赖。
- 浏览器支持 WebGL。
- 如果加载模型失败，先检查 Web 项目中的 URDF 和 mesh 路径。

## 和其他章节的关系

| 目标 | 阅读位置 |
| --- | --- |
| 只想启动 Web 页面 | 本页 |
| 想理解 Web 项目源码结构 | [Web 开发](../../09_开发者指南/06_Web开发/README.md) |
| 想理解 URDF / ROS 2 模型来源 | [ROS2 接入](../ROS2/README.md) |
| 想连接真实机械臂 | 先完成 [快速上手](../../02_快速上手/README.md) 和 [基础操作](../../04_基础操作/README.md) |

## 待项目方补充

- Web 界面截图。
- Web 仿真演示视频。
- 推荐浏览器和典型操作流程截图。
- Web 真机控制链路的正式安全说明。
