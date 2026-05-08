# ZYArm SDK

`software/zyarm_sdk` 是面向二次开发、教学脚本、诊断工具和后续 LeRobot 原生适配的独立机械臂 SDK。它不依赖 ROS 2、MoveIt、ros2_control 或 LeRobot；MoveIt 真机路径继续由 `software/ros2_ws/src/zyarm_hardware_interface/` 独立维护。

## Layout

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

## Public Defaults

- 默认串口波特率为 `230400`，与当前固件默认配置保持一致。
- 6 个机械臂关节使用弧度。
- 夹爪使用归一化 `0.0..1.0`。
- SDK/ROS 使用“用户角度表达”：机械臂初始姿态下 6 个关节都为 `0`。固件 `[STATUS]` 和 `[MD]` 中的角度是“固件角度表达”，包含硬件零偏和符号约定。两者通过 `mapping` 层转换，业务代码默认不要直接混用。
- `get_latest_state()` 只读缓存，不发串口命令。
- `query_state()` 发送 CMD6 并等待 fresh `[STATUS]`。
- `fast_io()` 发送 CMD36，默认不等待 ACK 或新状态。
- CMD36 返回的状态只表示 measured snapshot/pre-command state，不表示动作完成后的状态。
- 普通 ACK 默认等待 1 秒，主要用于配置/模式切换命令，便于快速暴露串口、波特率或协议错误。
- 动作完成 ACK 默认等待 10 秒，用于 `reset()`、`move_ik()` 和同步夹爪等可能真实运动的命令。
- 录制动作回放 ACK 默认等待 190 秒，用于覆盖最长 3 分钟动作以及少量通信/收尾余量。

## Build C++ Examples

```bash
cmake -S software/zyarm_sdk/cpp -B /tmp/zyarm_sdk_cpp_build -DZYARM_SDK_BUILD_EXAMPLES=ON
cmake --build /tmp/zyarm_sdk_cpp_build -j
```

示例可执行文件会生成在 `/tmp/zyarm_sdk_cpp_build/` 下，包括 `read_state`、`fast_io_loop` 和 `teleop_step`。

## Legacy Notes

`robot_actor.py`、`RobotActor.cpp` 和 `include/RobotActor.hpp` 是旧入口，仓库内调用方已迁移到新 SDK API。它们不再作为主路径维护。

`URPT8B0.py` 是继电器协议，主要服务上下电诊断脚本，不属于机械臂 SDK 主 API；后续可以移出或删除。
