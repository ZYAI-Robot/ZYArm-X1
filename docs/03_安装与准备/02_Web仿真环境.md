# Web 仿真环境

Web 环境用于运行网页控制端、URDF 模型加载、Three.js 可视化和网页仿真。它适合先观察机械臂结构，也适合后续做 Web 控制端开发。

## 先认识这些工具

| 工具 | 本页用它做什么 |
| --- | --- |
| Node.js | 运行前端开发工具链 |
| pnpm | 安装 Web 项目依赖、启动开发服务器 |
| 浏览器 | 打开本地 Web 页面并显示机械臂模型 |
| WebGL | 浏览器里的 3D 渲染能力，用于显示机械臂模型 |
| URDF / mesh | 描述机械臂结构和外观资源 |

## 适用场景

- 在浏览器中查看机械臂模型。
- 体验 Web 拖拽或仿真交互。
- 开发 Web 控制端或可视化页面。
- 课程演示时提供更直观的机械臂界面。

## 需要准备

| 项目 | 说明 |
| --- | --- |
| Node.js | 当前 [web/README.md](../../web/README.md) 标注使用 Node `22.17.0` |
| pnpm | Web 项目包管理工具 |
| 浏览器 | 需要支持 WebGL |
| 仓库代码 | Web 项目位于 `web/` |

## 安装 pnpm

如果系统还没有 pnpm：

```bash
npm install -g pnpm
```

`npm` 是 Node.js 自带的包管理工具，这里只用它安装 `pnpm`。后续进入 `web/` 后，统一使用 `pnpm install` 和 `pnpm run dev`。

确认版本：

```bash
node --version
pnpm --version
```

## 启动 Web 项目

进入 Web 项目：

```bash
cd web
pnpm install
pnpm run dev
```

启动后，根据终端输出打开本地地址。常见地址类似：

```text
http://localhost:5173/
```

## 验证内容

启动后建议检查：

- 页面能正常打开。
- 机械臂模型能完整加载。
- 浏览器控制台没有持续资源加载错误。
- 页面交互没有明显卡死。

Web 玩法路线见 [Web 仿真体验](../05_常用玩法/01_Web仿真体验.md)。Web 接入说明见 [Web 接入](../06_仿真与框架接入/Web/README.md)。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `pnpm` 命令不可用 | 是否安装 pnpm、终端 PATH 是否刷新 |
| 依赖安装失败 | 网络、npm 源、Node 版本 |
| 页面能打开但模型缺失 | URDF、mesh 或 public 资源路径 |
| 浏览器显示异常 | 浏览器 WebGL 支持、显卡驱动 |

## 待补充

> 待项目方补充：请提供已验证 Node/pnpm 版本组合、推荐浏览器、Web 界面截图、模型加载成功截图和常见错误截图。
