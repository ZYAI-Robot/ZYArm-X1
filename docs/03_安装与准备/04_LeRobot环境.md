# LeRobot 环境

LeRobot 环境用于主从遥操、数据采集、数据集回放和策略评估。ZYArm 的 LeRobot 插件不依赖 ROS 2，底层通过 Python SDK 直接访问串口。

## 先认识这些工具

| 工具/术语 | 本页用它做什么 |
| --- | --- |
| LeRobot | 提供遥操、数据采集、回放和策略评估流程 |
| `lerobot_robot_zyarm` | ZYArm 适配 LeRobot 的本地插件 |
| dataset | 保存演示数据、相机画面、状态和动作的数据集 |
| camera index | 系统给摄像头分配的编号，采集前需要确认 |
| `ffplay` | Linux 下快速预览摄像头画面 |

## 适用场景

- 使用 LeRobot `teleoperate` 做遥操。
- 使用 LeRobot `record` 采集数据集。
- 使用 LeRobot `replay` 回放数据集。
- 使用策略进行评估。
- 配置前置摄像头、腕部摄像头等数据输入。

具体采集和回放流程见 [科研与数据采集](../07_科研与数据采集/README.md) 和 [software/lerobot_robot_zyarm/README.md](../../software/lerobot_robot_zyarm/README.md)。

## 版本要求

当前插件固定支持：

```text
lerobot==0.5.1
Python >= 3.10
```

`software/lerobot_robot_zyarm/pyproject.toml` 依赖：

```text
lerobot==0.5.1
zyarm-sdk==0.1.0
```

建议使用独立虚拟环境。

## 安装

在仓库根目录执行：

```bash
python -m venv .venv-lerobot
source .venv-lerobot/bin/activate
python -m pip install --upgrade pip
pip install lerobot==0.5.1
pip install -e software/zyarm_sdk/python
pip install -e software/lerobot_robot_zyarm
```

Windows 如果只做插件导入或基础验证，也可以尝试同样流程；涉及摄像头、编码、长时间数据采集时，建议优先使用已验证的 Ubuntu 环境。

## 导入验证

```bash
python -c "import zyarm_sdk; print('zyarm_sdk ok')"
python -c "import lerobot_robot_zyarm; print('zyarm LeRobot plugin ok')"
```

## 硬件前置检查

运行 LeRobot 命令前，请先确认：

- leader 和 follower 串口没有接反。
- 每台机械臂都能单独读取 `[CMD][6]` 状态。
- 双臂名称和物理标签已经记录。
- 摄像头编号和物理位置一一对应。
- 可以快速切断机械臂电源。

多设备组织见 [设备角色与端口确认](../07_科研与数据采集/03_设备角色与端口确认.md) 和 [摄像头与视角配置](../07_科研与数据采集/04_摄像头与视角配置.md)。

## 相机准备

LeRobot camera 配置通常需要确认：

- 摄像头能被系统打开。
- OpenCV index 或设备路径和物理相机一致。
- 分辨率和帧率不会让机器明显卡顿。
- 前置相机和腕部相机名称保持稳定，例如 `front`、`wrist`。

Linux 下可以先用：

```bash
ls /dev/video*
ffplay -f v4l2 /dev/video0
```

`/dev/video0` 是 Linux 给摄像头分配的设备名；如果有多个摄像头，可能还会出现 `/dev/video1`、`/dev/video2`。`ffplay` 只用于确认画面和物理摄像头是否对应，不负责采集数据集。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `import lerobot_robot_zyarm` 失败 | 是否安装插件、是否激活虚拟环境 |
| LeRobot 命令找不到 zyarm 类型 | 插件是否安装到当前环境 |
| 摄像头打开失败 | index、权限、是否被其他软件占用 |
| 采集画面或控制卡顿 | 降低分辨率、降低 fps、减少编码线程、关闭显示 |
| 遥操方向不符合预期 | leader/follower 是否接反、名称记录是否正确 |

## 下一步

- LeRobot 遥操、采集、回放和策略评估：[科研与数据采集](../07_科研与数据采集/README.md)
- 插件命令参考：[software/lerobot_robot_zyarm/README.md](../../software/lerobot_robot_zyarm/README.md)

## 待补充

> 待项目方补充：请提供已验证 Python 版本、系统镜像、相机型号、视频编码依赖、数据目录建议、双摄像头配置截图和完整环境安装视频。
