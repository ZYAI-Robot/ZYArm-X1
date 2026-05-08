# Python 接入

本页说明如何通过 Python SDK 快速控制机械臂。Python 路线适合教学脚本、诊断工具、轻量二次开发、主从遥操脚本和 LeRobot 相关实验。

Python SDK 是对底层 Serial API 的封装。你不需要手工拼 `[CMD]` 文本，而是用 Python 对象调用 `query_state()`、`reset()`、`move_ik()`、`fast_io()` 等接口。

如果你要修改 SDK 类、扩展 API 或补测试，请阅读 [SDK 开发](../../09_开发者指南/04_SDK开发/README.md)。本页只保留使用入口和最小示例。

## 开发环境

本页只覆盖 Ubuntu/Linux 下的 Python SDK 使用流程。

在仓库根目录安装 Python SDK：

```bash
pip install -e software/zyarm_sdk/python
```

这条命令会把本地 SDK 以可编辑模式安装到当前 Python 环境，适合直接运行仓库里的示例脚本。

建议准备：

- Python 3.9 或更高版本。
- 已完成 [快速上手](../../02_快速上手/README.md)，并确认机械臂串口可用。
- 记录机械臂串口设备名，例如 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。

## 运行示例

安装完成后，可以先运行状态读取示例：

```bash
python3 software/zyarm_sdk/python/examples/read_state.py
```

当前示例里的端口默认按 Ubuntu 设备名编写，例如 `/dev/ttyUSB0`。如果你的端口不同，请先把示例中的端口改成实际设备名。

第一次接入建议先运行 `read_state.py`，确认 Python SDK 能读取真实机械臂状态，再尝试动作类脚本。

## 新增自己的 Python 脚本

如果要新增一个 `.py` 文件并调用 Python SDK，可以先把文件放在 `software/zyarm_sdk/python/examples/` 下，例如：

```text
software/zyarm_sdk/python/examples/my_read_state.py
```

脚本中直接导入 SDK：

```python
from zyarm_sdk import ZyArm, ZyArmConfig

with ZyArm(ZyArmConfig(port="/dev/ttyUSB0")) as arm:
    state = arm.query_state(timeout_ms=1000)
    print(state.positions if state else "No fresh state received")
```

运行：

```bash
python3 software/zyarm_sdk/python/examples/my_read_state.py
```

第一次新增脚本建议只做 `query_state()` 状态读取。`reset()`、`move_ik()`、`fast_io()` 都可能驱动真实机械臂，运行前必须确认机械臂周围没有障碍物，并且可以随时断电。

## 重新运行

Python 脚本没有 C++ 那样的编译产物。如果只是修改已有 `.py` 文件，直接重新运行脚本即可：

```bash
python3 software/zyarm_sdk/python/examples/my_read_state.py
```

由于前面使用的是 `pip install -e` 可编辑安装，修改 `software/zyarm_sdk/python/src/zyarm_sdk/` 里的 SDK 代码后，也通常不需要重新安装，重新运行脚本即可看到变化。

如果更换了 Python 环境、修改了依赖，或调整了 `pyproject.toml`，建议重新安装一次：

```bash
pip install -e software/zyarm_sdk/python
```

## 推荐入口

| 入口 | 作用 |
| --- | --- |
| [software/zyarm_sdk](../../../software/zyarm_sdk/README.md) | SDK 源码目录说明 |
| `software/zyarm_sdk/python/examples/read_state.py` | 状态读取示例 |
| `software/zyarm_sdk/python/examples/fast_io_loop.py` | 高频下发示例 |
| `software/zyarm_sdk/python/examples/teleop_step.py` | 遥操 step 示例 |
| `software/tools/get_arm_info.py` | 读取机械臂名称、版本等信息 |

## 最小状态读取

```python
from zyarm_sdk import ZyArm, ZyArmConfig

with ZyArm(ZyArmConfig(port="/dev/ttyUSB0")) as arm:
    state = arm.query_state(timeout_ms=1000)
    print(state.positions if state else "No fresh state received")
```

运行前请先完成 [快速上手](../../02_快速上手/README.md) 中的串口确认。

`timeout_ms=1000` 表示最多等待 1000 毫秒获取新状态；如果超时返回空值，通常要先检查串口号、权限、机械臂供电和串口是否被其他程序占用。

## 最小动作示例

动作会驱动真实机械臂。运行前请确认周围没有障碍物，并且可以随时断电。

```python
from zyarm_sdk import ZyArm, ZyArmConfig

with ZyArm(ZyArmConfig(port="/dev/ttyUSB0")) as arm:
    print(arm.reset())
    print(arm.move_ik(200, 0, 0, 0, 0, 0))
    state = arm.query_state(timeout_ms=1000)
    print(state.positions if state else "No fresh state received")
```

`reset()` 对应固件复位能力，`move_ik()` 对应运动学逆解动作。第一次运行时建议空载、低风险、小范围验证。

## 单位约定

- 六个机械臂关节使用弧度。
- 夹爪使用 `0.0..1.0` 归一化值。
- SDK/ROS 使用用户角度表达，和固件原始角度不同。

弧度是程序和机器人框架里常用的角度单位。`0.0..1.0` 归一化值表示把夹爪开合映射到一个固定范围。不要把 SDK 数值直接当成固件 `[STATUS]` 里的角度复制回串口命令。

## 常见下一步

| 目标 | 下一步 |
| --- | --- |
| 只读取状态 | 运行 `read_state.py` |
| 做连续控制 | 运行 `fast_io_loop.py`，再理解 `fast_io()` |
| 做主从或遥操 | 先阅读 [主从臂遥操](../../05_常用玩法/04_主从臂遥操.md)，再使用工具脚本 |
| 做 LeRobot 数据采集 | 阅读 [科研与数据采集](../../07_科研与数据采集/README.md) |
| 修改 SDK 实现 | 阅读 [SDK 开发](../../09_开发者指南/04_SDK开发/README.md) |

## 待项目方补充

- 推荐 Python 版本和虚拟环境创建方式。
- 面向课程的 Python 示例清单。
- 常见异常和串口权限问题截图。
