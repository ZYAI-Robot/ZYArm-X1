# Performance guide

高频控制路径应尽量短：

- `fast_io()` 默认只做校验、单位映射、CMD36 格式化和串口写入，不等待 ACK 或新 `[STATUS]`。
- 需要状态对齐时显式使用 `wait_state=True` 和短超时；返回状态必须按 CMD36 measured snapshot/pre-command state 理解。
- 高频循环中不要混用 CMD6 主动查询、CMD17 周期状态和 CMD36 状态，除非调用方明确标注来源并接受时序差异。
- 每个串口连接只允许一个 SDK RX owner。不要用多个线程或多个对象同时读同一个串口。
- 默认不做逐帧文件日志。诊断日志应放在专门工具里，并避免进入控制热路径。
- Python 适合教学、数据采集和一般控制；更严格的实时/低抖动场景优先使用 C++ SDK。
- Windows C++ 串口第一版使用阻塞读写加超时，先保证可读、可靠；只有实测不足时再引入 overlapped I/O。
