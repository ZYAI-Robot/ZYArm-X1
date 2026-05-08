# Web 项目结构

Web 工程位于 `web/`，面向浏览器运行。不要把固件协议细节直接散落在 UI 组件里，后续接入真机时应保留清晰的通信边界。

## 关键目录

| 目录或文件 | 作用 |
| --- | --- |
| `web/public/URDF_XG_Robot_Arm_Step_Urdf_V1/` | 当前 Web 使用的 URDF 和 mesh 资源 |
| `web/src/components/Menu.vue` | 控制面板组件 |
| `web/src/components/ScreenContainer.vue` | 响应式容器组件 |
| `web/src/views/home/index.vue` | 主页面 |
| `web/src/views/serial/index.vue` | 串口相关页面入口 |
| `web/src/utils/RobotDragControls.js` | 机械臂拖拽控制逻辑 |
| `web/src/utils/IKManager.js` | 逆解或姿态计算相关逻辑 |
| `web/src/utils/websocket.js` | WebSocket 通信工具 |
| `web/src/stores/` | Pinia 状态 |

## 修改建议

- UI 展示逻辑放在组件中。
- 3D 交互和模型控制放在 `utils/` 中。
- 全局状态放在 `stores/` 中。
- 通信逻辑放在独立工具中，避免直接写在页面组件里。
- 新增控件时同步检查移动端布局和按钮文本是否溢出。

## 验证建议

- 本地启动后检查首屏是否能看到模型。
- 拖拽、按钮和关节显示不会造成布局抖动。
- 控制面板在常见窗口大小下文字不重叠。
- 如果新增真机通信，必须在界面上区分仿真和真机状态。

## 待项目方补充

- Web 项目正式 UI 规范。
- Web 真机控制入口是否默认开启。
- 产品渲染图、模型资源和品牌素材的使用规范。
