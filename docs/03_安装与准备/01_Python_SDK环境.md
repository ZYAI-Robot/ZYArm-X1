# Python SDK 环境

Python SDK 适合教学脚本、诊断工具、简单二次开发、常用玩法脚本和 LeRobot 插件。SDK 不依赖 ROS 2。

## 先认识这些工具

| 工具 | 本页用它做什么 |
| --- | --- |
| Python | 运行 SDK、tools 脚本和验证命令 |
| 虚拟环境 | 隔离当前项目依赖，避免影响系统 Python |
| `pip` | 安装 SDK 和 tools 依赖 |
| `pyserial` | 让 Python 程序能打开串口并收发文本 |
| editable install / `-e` | 以可编辑方式安装本地 SDK，方便后续开发 |

## 适用场景

- 运行 `software/tools/` 中的工具脚本。
- 编写 Python 控制程序。
- 安装 LeRobot 插件前准备底层机械臂 SDK。
- 做串口诊断、状态读取和高频控制链路验证。

## 版本要求

`software/zyarm_sdk/python/pyproject.toml` 当前要求：

```text
Python >= 3.9
pyserial >= 3.5
```

建议使用虚拟环境，避免和系统 Python 或其他项目依赖混在一起。

## 创建虚拟环境

Windows：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

Ubuntu：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

如果 Windows PowerShell 阻止激活脚本，可以临时调整当前终端会话的执行策略，或直接使用 `.venv\Scripts\python.exe` 执行命令。

## 安装 SDK

在仓库根目录执行：

```bash
pip install -e software/zyarm_sdk/python
```

这里的 `-e` 表示 editable install。它不会把源码复制成一份固定安装包，而是让当前虚拟环境直接引用仓库中的 SDK 源码，适合本项目这种边使用边开发的场景。

验证安装：

```bash
python -c "import zyarm_sdk; print('zyarm_sdk ok')"
```

## 安装 tools 依赖

如果要运行 Joy-Con、遥操或可视化相关工具，还需要安装 tools 依赖：

```bash
pip install -r software/tools/requirements.txt
```

如果只运行 `get_arm_info.py` 这类基础脚本，通常只需要 `pyserial`。

## 真机最小验证

先确认机械臂已经完成 [快速上手](../02_快速上手/README.md)，再使用工具读取名称和版本：

Windows：

```powershell
python software\tools\get_arm_info.py COM3 -b 230400
```

Ubuntu：

```bash
python3 software/tools/get_arm_info.py /dev/ttyUSB0 -b 230400
```

如果这个脚本能读到名称或版本信息，说明 Python 串口环境和机械臂基础通信已经可用。

## 单位提醒

SDK 和固件文本指令的单位表达不完全相同：

- 固件 `[STATUS]` 是固件角度表达。
- SDK 对外使用公共角度表达，6 个关节为弧度。
- 夹爪在 SDK 中使用归一化 `0.0..1.0`。

具体 API 和单位说明见 [software/zyarm_sdk/README.md](../../software/zyarm_sdk/README.md)。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `import zyarm_sdk` 失败 | 是否激活虚拟环境、是否执行 `pip install -e` |
| `get_arm_info.py` 打不开串口 | 串口号、权限、端口占用 |
| Windows tools 依赖安装失败 | Python 版本、编译依赖、是否只需要基础工具 |
| 角度数值看起来和固件不同 | 是否混用了 SDK 单位和固件 `[STATUS]` 表达 |

## 下一步

- 写 Python 控制程序：[仿真与框架接入 / Python](../06_仿真与框架接入/Python/README.md)
- 使用手柄或遥操工具：[常用玩法与项目案例](../05_常用玩法/README.md)
- 安装 LeRobot 插件：[LeRobot 环境](04_LeRobot环境.md)
