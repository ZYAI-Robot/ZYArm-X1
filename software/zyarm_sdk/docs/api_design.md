# ZYArm SDK API 设计

`zyarm_sdk` 是面向 Python/C++ 程序、示例脚本、教学代码、诊断工具和后续 LeRobot 原生适配的独立机械臂 SDK。它不依赖 ROS 2、MoveIt、`ros2_control` 或 LeRobot；这些框架可以在上层使用 SDK，也可以各自维护自己的硬件接入路径。

## 核心概念

- `ZyArmConfig`：串口、波特率、超时、角度映射和安全限制配置。当前默认波特率为 `230400`。
- `ZyArm`：单臂生命周期和控制入口，包括 `connect()`、`close()`、`get_latest_state()`、`query_state()`、`fast_io()`、`reset()`、`standby()`、`stop()` 等。
- `ArmState`：机械臂状态，使用 SDK 对外单位，包含 `positions`、状态来源、时间戳、序号和 `age_ms`。
- `FastIoResult`：快速关节 I/O 调用结果。默认只表示目标已经交给传输层；需要时可携带一次测量快照。
- `TeleopAction`：从主臂 `[MD]` 数据转换出的遥操作动作，带有时间戳和来源信息。
- `ArmFrameStats`：SDK 观察到的 `[MD]` 和 `[STATUS]` 接收统计。丢帧诊断基于固件帧号估算，适合观察遥操作链路质量，不等价于严格的传输层审计。
- `ZyArmTeleopPair`：主从遥操作入口，支持显式 step 模式和可选自动跟随示例模式。

## 状态接口和 fast_io 行为

- `query_state(timeout_ms=...)` 会主动向机械臂请求一次新状态，并等待返回。它适合启动确认、低频诊断、动作前后检查。
- `get_latest_state(max_age_ms=None)` 只读取 SDK 后台接收线程维护的内存缓存，不发送串口命令。它适合高频循环中读取最近测量状态。

`fast_io()` 是高频状态读写接口，用于快速修改机械臂 6 个关节目标和 1 个夹爪目标，同时从快速链路获得角度快照。调用方传入目标后，SDK 会完成安全限制、单位映射和串口写入。

默认情况下，`fast_io()` 写入目标后立即返回，不等待机械臂到位或动作完成 ACK。SDK 后台接收线程会继续接收状态帧并更新状态缓存，调用方可以通过 `get_latest_state()` 获取最近状态。`FastIoResult.accepted=True` 只表示这次目标已经成功交给传输层，不表示机械臂已经运动到目标位置。

如果调用方希望这次调用同步拿一次快照，可以使用 `fast_io(..., wait_state=True)`；快照会放入 `FastIoResult.measured_snapshot`。这个快照仍然只能当成“本轮快速 I/O 附带拿到的测量状态”，不能当成动作完成后的最终状态。

高频状态读写推荐组合是：启动时用一次 `query_state()` 建立初始状态，然后在循环里用 `get_latest_state(max_age_ms=...)` 读取最近缓存状态，用 `fast_io()` 下发下一帧目标。不要在高频循环里每帧调用 `query_state()`。

缓存状态可能比最近一次 `fast_io()` 写入滞后一帧或数帧。调用方应把它当成“最新测量观测”，不要当成“最新目标已经执行完成”的确认。

## 角度和单位

固件角度表达和 SDK/ROS 角度表达是两个不同层级：

- 固件角度表达：固件 `[STATUS]`、`[MD]` 和底层命令中的原始角度，包含硬件零偏、符号约定和夹爪命令范围。
- SDK/ROS 角度表达：面向应用开发的统一表达。机械臂初始姿态下 6 个关节都表示为 `0` 弧度，夹爪使用 `0.0..1.0` 归一化值。
- `mapping` 层是两种表达之间的转换边界。应用代码、示例和 LeRobot 适配默认使用 SDK/ROS 表达。
- 调试固件原始日志时，不要把 `[STATUS]` 中的 degree 数值直接和 SDK 的 `ArmState.positions` 对比，应先通过 `MappingConfig` 转换。

## 控制路径默认策略

- 高频热路径只完成目标校验、单位映射和串口写入，不等待 ACK，不做逐帧文件日志。
- 推荐高频循环：先 `query_state()` 确认链路，再重复 `get_latest_state()`、计算目标、`fast_io()` 下发目标。
- 自动遥操作跟随由新的 `[MD]` 帧触发。SDK 不额外按固定频率重采样缓存动作。
- `TeleopConfig.leader_hz` 配置主臂固件输出 `[MD]` 的频率。SDK 侧没有额外的 follower 采样频率。
- `ZyArm.get_frame_stats()` 返回 `[MD]` 和 `[STATUS]` 的接收计数与近似 1 秒速率；`reset_frame_stats()` 只清除统计值，不重置底层传输序号。
- `ZyArmTeleopPair.start_step_mode()` 和 `start_auto_follow()` 会在启动主臂前让从臂进入固件 slave filter 模式，使 SDK 和 LeRobot 遥操作使用一致的执行语义。
- 每个串口连接只有一个 RX owner，负责维护 ACK、状态和动作缓存。不要用多个 SDK 对象或多个线程同时读取同一个串口。

## 超时语义

- 普通 ACK 默认等待 1 秒，主要用于配置和模式切换命令，便于快速暴露串口、波特率或协议错误。
- 动作完成 ACK 默认等待 10 秒，用于 `reset()`、`standby()`、`move_ik()` 和同步夹爪命令。这类 ACK 表示固件报告动作完成，不只是串口写入成功。
- `standby()` 对应固件 `CMD38`，会移动到低功耗待机姿态 `[0 -105 90 0 0 0 0]` 并保持锁定；它不是卸力或外部断电。
- 动作录制回放 ACK 默认等待 190 秒，用于覆盖最长 3 分钟动作和少量通信余量。
- 显式状态等待和动作等待都必须带明确超时，避免控制流程无限阻塞。

## 平台策略

- Python 使用 `pyserial`，兼容 Linux 和 Windows 串口名称。
- C++ 通过 `transport` 隐藏 POSIX 和 Win32 串口差异。
- Windows C++ 第一版采用阻塞读写加显式超时，先保证可读、可靠；只有实测性能不足时再引入 overlapped I/O。

## C++ 示例

```bash
cmake -S software/zyarm_sdk/cpp -B /tmp/zyarm_sdk_cpp_build -DZYARM_SDK_BUILD_EXAMPLES=ON
cmake --build /tmp/zyarm_sdk_cpp_build -j
```

示例目标包括 `read_state`、`fast_io_loop` 和 `teleop_step`。
