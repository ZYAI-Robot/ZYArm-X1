# ZYArm SDK 性能说明

高频控制路径的目标是把每一帧做短：少等待、少分配、少日志、单一串口接收者。SDK 的默认设计是让 `fast_io()` 承担快速下发，让后台 RX 线程持续维护状态缓存。

## 高频读写模式

推荐流程：

1. 启动时调用一次 `query_state(timeout_ms=...)`，确认串口链路正常并获得初始状态。
2. 控制循环中调用 `get_latest_state(max_age_ms=...)`，读取后台 RX 缓存中的最新测量状态。
3. 根据最新状态计算下一帧目标。
4. 调用 `fast_io(target)` 下发目标。

不要在高频循环中每一帧调用 `query_state()`。它会发送 `CMD6` 并等待新的 `[STATUS]`，适合启动确认、低频诊断和动作前后检查，不适合作为热路径里的逐帧状态读取。

## 状态时序

- `fast_io()` 默认只做校验、单位映射、`CMD36` 格式化和串口写入，不等待 ACK 或新的 `[STATUS]`。
- `get_latest_state()` 返回最近一次被 SDK 接收线程解析到的 `[STATUS]`，可能比当前 `fast_io()` 写入滞后一帧或数帧。
- 控制逻辑必须把 `get_latest_state()` 理解成“最新测量缓存”，不要理解成“本次目标已经执行完成后的状态”。
- 确实需要在某次 `CMD36` 后等待状态对齐时，可以显式使用 `fast_io(..., wait_state=True, timeout_ms=...)`。返回状态仍必须按 `CMD36` measured snapshot/pre-command state 理解。
- 高频循环中不要混用 `CMD6` 主动查询、`CMD17` 周期状态和 `CMD36` 状态，除非调用方明确标注来源并接受时序差异。

## 串口和日志

- 每个串口连接只允许一个 SDK RX owner。不要用多个线程或多个对象同时读取同一个串口。
- 默认不做逐帧文件日志。诊断日志应放在专门工具里，并避免进入控制热路径。
- ACK、`[STATUS]` 和 `[MD]` 解析都在内存中完成，不依赖串口日志文件。

## Python 与 C++

- Python 适合教学、数据采集、诊断脚本和一般控制。
- 更严格的实时性、低抖动或高频控制场景优先使用 C++ SDK。
- Windows C++ 串口第一版使用阻塞读写加超时，先保证可读、可靠；只有实测不足时再引入 overlapped I/O。

## 典型循环

```python
state = arm.query_state(timeout_ms=1000)  # 启动时确认链路和初始状态
if state is None:
    raise RuntimeError("No fresh state received")

target = list(state.positions)
while running:
    latest = arm.get_latest_state(max_age_ms=100)
    if latest is not None:
        observation = latest.positions
        target = compute_next_target(observation)

    arm.fast_io(target)
```

如果控制策略必须知道状态是否新鲜，应给 `get_latest_state(max_age_ms=...)` 设置合理阈值，并在状态陈旧时减速、保持目标或退出控制循环。
