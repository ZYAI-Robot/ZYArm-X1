# Web 开发

Web 工程负责浏览器端模型展示、仿真交互和后续真机控制入口。当前项目使用 Vue、Three.js、URDF Loader、Pinia 和 Element Plus。

如果只是想启动 Web 页面、查看模型或体验网页仿真，请先阅读 [Web 接入](../../06_仿真与框架接入/Web/README.md)。本节主要说明 Web 源码结构、URDF 加载、交互实现和真机控制接入。

源码入口见 [web/README.md](../../../web/README.md)。

## 先认识这些工具

| 工具 | 作用 |
| --- | --- |
| Vue | 前端组件框架 |
| Three.js | 浏览器 3D 渲染 |
| URDF Loader | 在网页中加载机械臂 URDF 模型 |
| Pinia | 前端状态管理 |
| Element Plus | UI 组件库 |
| pnpm | 前端包管理和脚本运行工具 |

## 本节页面

- [Web 项目结构](01_Web项目结构.md)
- [URDF 模型加载](02_URDF模型加载.md)
- [交互与真机控制接入](03_交互与真机控制接入.md)

## 最小运行

```bash
cd web
pnpm install
pnpm run dev
```

## 待项目方补充

- Web 控制端正式支持的浏览器版本。
- 真机控制接入的安全策略。
- Web 仿真演示截图和课程说明。
