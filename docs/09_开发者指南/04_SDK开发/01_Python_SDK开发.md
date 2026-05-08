# Python SDK 开发

Python SDK 更适合快速脚本、教学实验、诊断工具和 LeRobot 插件。它把底层串口文本命令封装成 Python 类，使用者不用手工拼 `[CMD][6]`、等待 ACK、解析 `[STATUS]`，也不用在每个脚本里重复处理单位换算和超时。

源码位于 `software/zyarm_sdk/python/src/zyarm_sdk/`。如果只是写上层应用，通常先看 `ZyArmConfig` 和 `ZyArm`；如果要修改 SDK，再继续看 `protocol.py`、`mapping.py`、`transport.py` 和 `teleop.py`。

## 开发环境

在仓库根目录安装可编辑包：

```bash
pip install -e software/zyarm_sdk/python
```

这条命令会把本地源码以开发模式安装到当前 Python 环境。后续修改 `software/zyarm_sdk/python/src/zyarm_sdk/` 里的源码后，不需要重新打包安装，脚本就能直接引用最新代码。

Python SDK 依赖 `pyserial` 打开串口；包配置里要求 Python `>=3.9`。

## 公开类速览

| 类或数据结构 | 位置 | 作用 |
| --- | --- | --- |
| `ZyArmConfig` | `config.py` | SDK 配置入口，至少需要填写 `port`，默认波特率为 `230400` |
| `ZyArm` | `arm.py` | 最常用的机械臂控制类，封装复位、IK、夹爪、状态读取、`fast_io` 和主从模式 |
| `ArmState` | `types.py` | 机械臂状态快照，包含 7 个位置值、来源、时间戳和原始回包 |
| `CommandResult` | `types.py` | 命令发送结果，告诉你命令是否被 ACK、对应 CMD 编号和返回说明 |
| `FastIoResult` | `types.py` | `fast_io()` 的返回结果，可选携带一次测量状态 |
| `MappingConfig` / `JointMapping` | `config.py` / `mapping.py` | 负责 SDK 公共单位和固件角度表达之间的转换 |
| `SafetyConfig` / `SafetyController` | `config.py` / `safety.py` | 负责上位机侧限幅、步进限制和状态新鲜度检查 |
| `TeleopConfig` | `config.py` | 主从遥操、连续控制的频率、超时和滤波配置 |
| `ZyArmLeader` | `teleop.py` | 把主臂上报的 `[MD]` 数据转换成 `TeleopAction` |
| `ZyArmFollower` | `teleop.py` | 把 `TeleopAction` 发送给从臂，内部调用 `fast_io()` |
| `ZyArmTeleopPair` | `teleop.py` | 同时管理 leader 和 follower，适合脚本里快速搭建主从跟随 |

SDK 里的公共关节表达是 7 个值：前 6 个为机械臂关节，单位是弧度；第 7 个为夹爪开合，范围是 `0.0..1.0`。固件 `[STATUS]` 回来的角度会先经过 `JointMapping.hardware_to_public()` 转换，再出现在 `ArmState.positions` 里。

## 最小示例：读取状态

这个示例等价于“发送 `CMD6` 并解析 `[STATUS]`”，适合确认 SDK 已经能连上真实机械臂。

```python
from zyarm_sdk import ZyArm, ZyArmConfig

with ZyArm(ZyArmConfig(port="COM3")) as arm:
    state = arm.query_state(timeout_ms=1000)
    if state is None:
        print("没有收到 STATUS，请检查端口、供电和串口占用")
    else:
        print("positions:", state.positions)
        print("source:", state.source)
        print("age_ms:", state.age_ms)
```

Windows 端口通常写成 `COM3`、`COM4`；Ubuntu 端口通常写成 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。运行前请先完成快速上手里的串口确认流程。

## 最小示例：复位和 IK 小动作

这个示例展示 `ZyArm` 的常用动作类 API。动作会驱动真实机械臂，运行前必须确认周围没有障碍物，并且可以随时断电。

```python
from zyarm_sdk import ZyArm, ZyArmConfig

with ZyArm(ZyArmConfig(port="COM3")) as arm:
    reset_result = arm.reset()
    print(reset_result.accepted, reset_result.message)

    move_result = arm.move_ik(200, 0, 0, 0, 0, 0)
    print(move_result.accepted, move_result.message)

    state = arm.query_state(timeout_ms=1000)
    if state:
        print(state.positions)
```

`reset()` 对应固件 `CMD1`，`move_ik()` 对应固件 `CMD0`。这类动作命令的 ACK 通常表示固件报告动作执行完成，所以 SDK 默认等待时间会比普通配置命令更长。

## 最小示例：关节级 fast_io

`fast_io()` 面向连续控制、遥操、模仿学习或强化学习等场景，底层对应固件 `CMD36`。它不会等待普通动作完成 ACK，适合高频发送目标关节状态。

```python
import time

from zyarm_sdk import ZyArm, ZyArmConfig

target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

with ZyArm(ZyArmConfig(port="COM3")) as arm:
    for _ in range(100):
        result = arm.fast_io(target)
        if not result.accepted:
            print(result.message)
            break
        time.sleep(0.02)
```

`target` 必须包含 7 个值。前 6 个关节使用弧度，第 7 个夹爪使用 `0.0..1.0`。如果只想让部分关节生效，可以传入 `apply_mask`，未生效的关节会由 SDK 转换为固件约定的“不改变”值。

```python
arm.fast_io(
    [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
    apply_mask=[True, False, False, False, False, False, False],
)
```

## 结果对象怎么读

`CommandResult.accepted` 表示 SDK 是否收到了期望的 ACK，或者命令是否已经成功写入串口。它不代表机械臂一定处于你预期的位置；动作后仍建议调用 `query_state()` 或读取缓存状态确认。

`ArmState.positions` 是 SDK 公共单位，不是固件原始角度。需要调试协议时可以看 `ArmState.raw_line`，它保留了原始 `[STATUS]` 文本。

`get_latest_state(max_age_ms=...)` 读取缓存状态，适合高频循环；`query_state(timeout_ms=...)` 会主动发送 `CMD6`，适合确认当前真实状态。

## 遥操相关类

主从遥操不建议直接在业务代码里手工循环发送 `CMD32`、`CMD33`、`CMD36`。SDK 已经提供了更高层的封装：

```python
from zyarm_sdk import ZyArm, ZyArmConfig, TeleopConfig
from zyarm_sdk.teleop import ZyArmTeleopPair

leader = ZyArm(ZyArmConfig(port="COM3"))
follower = ZyArm(ZyArmConfig(port="COM4"))
pair = ZyArmTeleopPair(leader, follower, config=TeleopConfig(leader_hz=50.0))

try:
    pair.connect()
    pair.start_auto_follow()
    input("按回车停止遥操...")
finally:
    pair.close()
```

真实项目里优先参考 `software/tools/fast_io_teleop_pair.py`、`software/tools/filtered_master_slave_teleop.py` 等脚本，因为这些脚本会额外处理参数、日志和异常退出。

## 常见入口文件

| 文件 | 适合什么时候看 |
| --- | --- |
| `arm.py` | 想知道 `ZyArm` 对外提供了哪些控制方法 |
| `config.py` | 想调整串口、超时、单位映射、安全限幅或遥操频率 |
| `types.py` | 想理解 SDK 返回值的数据结构 |
| `protocol.py` | 固件新增 CMD、ACK、STATUS 或 `[MD]` 格式变化时 |
| `mapping.py` | 关节零位、符号、角度单位或夹爪范围需要适配时 |
| `safety.py` | 需要限制关节范围、最大步进或状态过期时间时 |
| `transport.py` | 串口打开、读写、后台接收线程或超时行为异常时 |
| `teleop.py` | 主从遥操、连续控制或 LeRobot 接入需要复用时 |

## 新增 Python API

建议流程：

1. 确认固件命令已经稳定，或明确这是仅 SDK 层封装。
2. 如果新增或修改 CMD 编号，在 `protocol.py` 更新 `CommandId`。
3. 如果回包格式变化，在 `protocol.py` 增加解析函数和测试。
4. 在 `arm.py` 增加面向用户的 `ZyArm` 方法。
5. 如果涉及单位、零位或夹爪范围，补 `mapping.py`。
6. 如果涉及安全边界，补 `SafetyConfig` 或 `safety.py`。
7. 在 `python/tests/` 增加不依赖真机的测试。
8. 在 `python/examples/` 或 `software/tools/` 增加最小示例。
9. 更新用户文档和开发者文档。

## 测试建议

优先测试不依赖真机的部分：

- 命令字符串是否正确。
- ACK、STATUS、`[MD]` 解析是否稳定。
- 单位转换是否符合公共约定。
- 超时和错误是否给出可理解异常。
- `fast_io()` 是否正确处理 `apply_mask`。

真机测试建议放到 tools 或 diagnostics，并明确端口参数和风险。

## 待项目方补充

- Python SDK 发布到 PyPI 或内部包源的流程。
- 支持的 Python 版本范围。
- 真机集成测试的标准设备和测试记录模板。
