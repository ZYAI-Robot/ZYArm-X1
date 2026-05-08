# Assets

本目录用于存放文档配套资源。所有面向用户文档的图片、图表、视频、动图、PDF、速查表等资源都应放在这里，不要散落在各章节目录里。

## 分类

- `Images/`：实物图、截图、界面图
- `Diagrams/`：架构图、流程图、接线图
- `Videos/`：演示视频、GIF、录屏素材
- `Downloads/`：课程 PDF、速查表、清单、可下载资料

## 文档图示原则

文档图片首先用于帮助读者理解内容，而不是作为装饰。只有当图片能更清楚地表达结构、流程、模块关系、硬件布局、使用场景、数据流、决策路径或安全边界时，才建议加入文档。

常见图示类型：

- 总览图：帮助读者快速理解一篇文档或一个章节的核心模型。
- 流程图：表达操作顺序、检查点和分支条件。
- 架构图：表达固件、SDK、Web、ROS 2、MoveIt、Gazebo、LeRobot 等层级关系。
- 硬件/接口图：表达接口、线缆、摄像头、设备角色和安装关系。
- 决策图：帮助读者按身份、目标或下一步选择阅读路径。
- 场景图：表达抓取、分拣、主从遥操、数据采集等使用场景。
- 数据流图：表达 record、replay、policy eval、数据字段和质量检查链路。
- 安全边界图：表达风险、禁止动作、急停和排障顺序。

图片不能替代正文。命令、参数、安全提示、表格和可搜索说明仍应保留在 Markdown 中；图片只负责降低理解成本。

## 图示源文件与导出

结构化图示优先使用可编辑源文件，并导出 Markdown 友好的图片：

```text
docs/assets/Diagrams/example-name.svg
docs/assets/Diagrams/example-name.png
```

- `.svg`：可编辑源文件，后续修改文字、颜色、间距和连线时使用。
- `.png`：文档默认引用的导出图，兼容更多 Markdown 查看环境。

修改 SVG 后，需要重新导出同名 PNG，避免源文件和页面展示不一致。真实照片、截图和界面图放在 `Images/`，不需要 SVG 源文件。

## 命名建议

资源文件名应使用稳定的英文 kebab-case，并体现章节和用途。例如：

```text
readme-learning-roadmap.svg
readme-learning-roadmap.png
product-positioning-overview.svg
product-positioning-overview.png
quickstart-serial-status.png
quickstart-first-motion.mp4
hardware-interface-labels.png
ros2-moveit-architecture.png
lerobot-teleop-flow.png
camera-wrist-mount.jpg
```

## 资料占位

如果文档需要真实照片或规格资料，但当前仓库没有，请在对应文档中使用：

```text
> 待项目方补充：请提供……
```

同时把资料项记录到 [资料补充清单](../资料补充清单.md)。

