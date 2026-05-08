# SDK 开发

SDK 把底层串口协议封装成 Python/C++ API，是示例、工具、诊断和 LeRobot 的公共基础。开发 SDK 时，核心目标是让上层调用者不需要直接处理 `[CMD]` 字符串、ACK、状态缓存和单位转换。

如果只是想使用 Python SDK 跑状态读取、动作或遥操示例，请先阅读 [Python 接入](../../06_仿真与框架接入/Python/README.md)。本节主要说明 SDK 源码结构、公开类、接口扩展和测试。

SDK 的主要价值不是“把串口命令换个名字”，而是把协议、单位、状态缓存、异常和常用控制链路变成稳定接口。

## 先认识这些概念

| 概念 | 含义 |
| --- | --- |
| transport | 串口收发层，负责把文本命令发给固件并接收回包 |
| protocol | 命令和回包的格式化、解析和契约测试 |
| mapping | 固件角度表达与用户角度表达之间的转换 |
| safety | 上位机侧的限幅、检查和安全约束 |
| teleop | 主从遥操、动作重定向和连续控制封装 |

## 目录结构

```text
software/zyarm_sdk/
├── contracts/              # 协议 golden cases
├── docs/                   # 协议、API 和性能说明
├── python/                 # Python SDK
│   ├── src/zyarm_sdk/
│   ├── examples/
│   └── tests/
└── cpp/                    # C++ SDK
    ├── include/zyarm_sdk/
    ├── src/
    ├── examples/
    └── tests/
```

旧入口 `robot_actor.py`、`RobotActor.cpp`、`include/RobotActor.hpp` 仍在仓库中，但不作为新功能主路径。

## 公共单位约定

| 数据 | SDK/ROS/LeRobot 公共表达 | 注意 |
| --- | --- | --- |
| 机械臂 6 个关节 | 弧度 | 初始姿态附近为 `0` |
| 夹爪 | 归一化 `0.0..1.0` | 不直接暴露固件内部角度 |
| 固件 `[STATUS]` | 固件角度表达 | 包含硬件零偏和符号约定 |

业务代码默认使用 SDK 公共表达，不要直接混用固件角度表达和用户角度表达。

源码入口见 [software/zyarm_sdk/README.md](../../../software/zyarm_sdk/README.md)。

## 本节页面

- [Python SDK 开发](01_Python_SDK开发.md)
- [C++ SDK 开发](02_Cpp_SDK开发.md)
- [新增 SDK 接口与测试](03_新增SDK接口与测试.md)

## 开发前判断

| 需求 | 建议位置 |
| --- | --- |
| 新固件命令需要 Python 调用 | `software/zyarm_sdk/python/src/zyarm_sdk/arm.py` |
| 新固件命令需要 C++ 调用 | `software/zyarm_sdk/cpp/include/zyarm_sdk/arm.hpp` 和 `cpp/src/arm.cpp` |
| 回包格式变化 | `protocol.py`、`protocol.cpp`、contracts |
| 单位或零位变化 | `mapping.py`、`mapping.cpp` |
| 主从或连续控制策略变化 | `teleop.py`、`teleop.cpp` |

## 最小验证

- Python tests：`software/zyarm_sdk/python/tests/`
- C++ tests：`software/zyarm_sdk/cpp/tests/`
- 协议契约：`software/zyarm_sdk/contracts/protocol_cases.yaml`
- 示例脚本：`software/zyarm_sdk/python/examples/`、`software/zyarm_sdk/cpp/examples/`

## 待项目方补充

- SDK 对外版本号和发布方式。
- Python/C++ API 稳定性承诺。
- SDK 性能基准和 100Hz 控制链路的正式验收指标。
