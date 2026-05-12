# 新增 SDK 接口与测试

新增 SDK 接口时，要让调用者获得稳定语义，而不是暴露固件内部细节。

## 接口设计清单

- API 名称是否表达用户意图。
- 参数单位是否符合 SDK 公共约定。
- 默认值是否安全。
- 同步/异步语义是否清楚。
- 返回值是否能判断成功、失败或状态。
- 异常是否能帮助定位串口、协议、超时或设备问题。

## 代码位置判断

新增接口前先判断改动落在哪一层：

| 改动内容 | 优先位置 |
| --- | --- |
| 固件协议文本变化 | `protocol.py`、`protocol.cpp`、contracts |
| 单位或零位变化 | `mapping.py`、`mapping.cpp` |
| 面向用户的新动作 API | `arm.py`、`arm.hpp`、`arm.cpp` |
| 主从或连续控制策略 | `teleop.py`、`teleop.cpp` |
| 需要 LeRobot、tools 或 diagnostics 复用的能力 | SDK 内部，而不是某个脚本内部 |

新 API 应同时考虑 Python 和 C++ 是否需要保持一致。即使第一版只实现 Python，也要在文档里说明 C++ 是否暂不支持，避免调用者误以为两边已经对齐。

## 单位和 mapping

SDK 对上层默认使用公共表达：机械臂 6 个关节使用弧度，夹爪使用归一化 `0.0..1.0`。固件 `[STATUS]` 里的角度可能包含硬件零偏和符号约定，不应直接暴露给上层业务代码。

如果新增接口涉及角度、夹爪或位姿，优先复用 `mapping` 层，不要让 examples、tools、LeRobot 或 ROS 2 各自写一套转换逻辑。

## 协议契约

如果接口依赖新的命令格式或回包格式，建议同步更新：

```text
software/zyarm_sdk/contracts/protocol_cases.yaml
```

这个文件适合记录 golden cases，帮助 Python 和 C++ 保持一致。

## 测试分层

| 测试类型 | 目标 |
| --- | --- |
| 协议测试 | 不连真机，确认命令和回包解析 |
| mapping 测试 | 确认单位和零位转换 |
| fake transport 测试 | 模拟串口回包，验证 API 行为 |
| 示例运行 | 确认用户能按文档跑通 |
| 诊断脚本 | 验证真机稳定性或性能 |

## 文档同步

新增接口后检查：

- `software/zyarm_sdk/README.md`
- `software/zyarm_sdk/docs/`
- [仿真与框架接入](../../06_仿真与框架接入/README.md)
- [示例工具与诊断开发](../07_示例工具与诊断开发.md)
- 依赖 SDK 的 LeRobot、tools、diagnostics

## 待项目方补充

- SDK API 文档生成方式。
- 真机测试是否需要固定测试工装。
- 100Hz 控制相关接口的验收阈值和记录格式。
