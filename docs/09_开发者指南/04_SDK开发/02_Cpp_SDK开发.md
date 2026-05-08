# C++ SDK 开发

C++ SDK 面向性能敏感应用、C++ 示例、ROS 2 底层集成，以及后续需要静态链接或系统集成的场景。它和 Python SDK 的目标一致：把串口协议、状态解析、单位映射、超时和连续控制封装成稳定 API。

源码位于 `software/zyarm_sdk/cpp/`。如果只是调用机械臂，先看 `include/zyarm_sdk/arm.hpp`；如果要扩展 SDK，再看 `src/arm.cpp`、`protocol.hpp/.cpp`、`mapping.hpp/.cpp`、`transport.hpp/.cpp` 和 `teleop.hpp/.cpp`。

## 构建示例

在仓库根目录执行：

```bash
cmake -S software/zyarm_sdk/cpp -B build/zyarm_sdk_cpp -DZYARM_SDK_BUILD_EXAMPLES=ON
cmake --build build/zyarm_sdk_cpp -j
```

`CMake` 是 C++ 项目的构建工具，负责生成本机编译工程；`ZYARM_SDK_BUILD_EXAMPLES=ON` 会额外编译示例程序。当前 SDK 使用 C++17。

示例包括：

| 示例 | 作用 |
| --- | --- |
| `read_state` | 连接串口，主动查询一次机械臂状态 |
| `fast_io_loop` | 循环发送关节目标，用于连续控制链路验证 |
| `teleop_step` | 演示主从遥操的一步式控制流程 |

## 公开类速览

| 类或数据结构 | 位置 | 作用 |
| --- | --- | --- |
| `ZyArmConfig` | `config.hpp` | SDK 配置入口，至少需要填写 `port`，默认波特率为 `230400` |
| `ZyArm` | `arm.hpp` | 最常用的机械臂控制类，封装复位、IK、夹爪、状态读取、`fast_io` 和主从模式 |
| `ArmState` | `types.hpp` | 机械臂状态快照，包含 7 个位置值、来源、时间戳和原始回包 |
| `CommandResult` | `types.hpp` | 命令发送结果，告诉你命令是否被 ACK、对应 CMD 编号和返回说明 |
| `FastIoResult` | `types.hpp` | `fast_io()` 的返回结果，可选携带一次测量状态 |
| `JointArray` | `types.hpp` | 长度为 7 的关节数组类型，前 6 个为机械臂关节，第 7 个为夹爪 |
| `MappingConfig` / `JointMapping` | `config.hpp` / `mapping.hpp` | 负责 SDK 公共单位和固件角度表达之间的转换 |
| `SafetyConfig` / `SafetyController` | `config.hpp` / `safety.hpp` | 负责上位机侧限幅、步进限制和状态新鲜度检查 |
| `Transport` | `transport.hpp` | 串口传输抽象，测试时可以替换成 fake transport |
| `SerialTransport` | `transport.hpp` | 真实串口实现，内部处理收发线程、ACK、STATUS 和 `[MD]` 缓存 |
| `ZyArmLeader` / `ZyArmFollower` / `ZyArmTeleopPair` | `teleop.hpp` | 主从遥操和连续控制封装 |

SDK 里的公共关节表达是 `JointArray`，一共 7 个值：前 6 个为机械臂关节，单位是弧度；第 7 个为夹爪开合，范围是 `0.0..1.0`。固件 `[STATUS]` 回来的角度会先经过 `JointMapping::hardware_to_public()` 转换，再出现在 `ArmState::positions` 里。

## 最小示例：读取状态

这个示例等价于“发送 `CMD6` 并解析 `[STATUS]`”，适合确认 C++ SDK 已经能连上真实机械臂。

```cpp
#include <chrono>
#include <iostream>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "COM3";

  zyarm_sdk::ZyArm arm(config);
  arm.connect();

  auto state = arm.query_state(std::chrono::milliseconds(1000));
  if (!state.has_value()) {
    std::cout << "No fresh state received\n";
    arm.close();
    return 1;
  }

  for (double value : state->positions) {
    std::cout << value << " ";
  }
  std::cout << "\n";

  arm.close();
  return 0;
}
```

Windows 端口通常写成 `COM3`、`COM4`；Ubuntu 端口通常写成 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。运行前请先完成快速上手里的串口确认流程。

## 最小示例：复位和 IK 小动作

这个示例展示 `ZyArm` 的常用动作类 API。动作会驱动真实机械臂，运行前必须确认周围没有障碍物，并且可以随时断电。

```cpp
#include <chrono>
#include <iostream>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "COM3";

  zyarm_sdk::ZyArm arm(config);
  arm.connect();

  auto reset_result = arm.reset();
  std::cout << reset_result.accepted << " " << reset_result.message << "\n";

  auto move_result = arm.move_ik(200, 0, 0, 0, 0, 0);
  std::cout << move_result.accepted << " " << move_result.message << "\n";

  auto state = arm.query_state(std::chrono::milliseconds(1000));
  if (state.has_value()) {
    for (double value : state->positions) {
      std::cout << value << " ";
    }
    std::cout << "\n";
  }

  arm.close();
  return 0;
}
```

`reset()` 对应固件 `CMD1`，`move_ik()` 对应固件 `CMD0`。这类动作命令的 ACK 通常表示固件报告动作执行完成，所以 SDK 默认等待时间会比普通配置命令更长。

## 最小示例：关节级 fast_io

`fast_io()` 面向连续控制、遥操、模仿学习或强化学习等场景，底层对应固件 `CMD36`。它不会等待普通动作完成 ACK，适合高频发送目标关节状态。

```cpp
#include <chrono>
#include <thread>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "COM3";

  zyarm_sdk::ZyArm arm(config);
  arm.connect();

  zyarm_sdk::JointArray target{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
  for (int index = 0; index < 100; ++index) {
    auto result = arm.fast_io(target);
    if (!result.accepted) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  arm.close();
  return 0;
}
```

`target` 必须包含 7 个值。前 6 个关节使用弧度，第 7 个夹爪使用 `0.0..1.0`。如果只想让部分关节生效，可以传入 `apply_mask`，未生效的关节会由 SDK 转换为固件约定的“不改变”值。

```cpp
zyarm_sdk::JointArray target{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
std::array<bool, zyarm_sdk::kJointCount> mask{true, false, false, false, false, false, false};
arm.fast_io(target, mask);
```

## 结果对象怎么读

`CommandResult::accepted` 表示 SDK 是否收到了期望的 ACK，或者命令是否已经成功写入串口。它不代表机械臂一定处于你预期的位置；动作后仍建议调用 `query_state()` 或读取缓存状态确认。

`ArmState::positions` 是 SDK 公共单位，不是固件原始角度。需要调试协议时可以看 `ArmState::raw_line`，它保留了原始 `[STATUS]` 文本。

`get_latest_state(max_age_ms)` 读取缓存状态，适合高频循环；`query_state(timeout)` 会主动发送 `CMD6`，适合确认当前真实状态。

## 遥操相关类

主从遥操不建议直接在业务代码里手工循环发送 `CMD32`、`CMD33`、`CMD36`。C++ SDK 已经提供了更高层的封装：

```cpp
#include <iostream>
#include <memory>

#include "zyarm_sdk/arm.hpp"
#include "zyarm_sdk/teleop.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig leader_config;
  leader_config.port = "COM3";

  zyarm_sdk::ZyArmConfig follower_config;
  follower_config.port = "COM4";

  auto leader = std::make_shared<zyarm_sdk::ZyArm>(leader_config);
  auto follower = std::make_shared<zyarm_sdk::ZyArm>(follower_config);

  zyarm_sdk::TeleopConfig teleop_config;
  teleop_config.leader_hz = 50.0;

  zyarm_sdk::ZyArmTeleopPair pair(leader, follower, teleop_config);
  pair.connect();
  pair.start_auto_follow();

  std::cout << "Press Enter to stop...\n";
  std::cin.get();

  pair.close();
  return 0;
}
```

真实项目里优先参考 `software/tools/fast_io_teleop_pair.py`、`software/tools/filtered_master_slave_teleop.py` 的流程和参数设计；C++ 侧如果要补遥操工具，也应保持与 Python 工具的端口、频率、滤波和日志语义一致。

## 常见入口文件

| 文件 | 适合什么时候看 |
| --- | --- |
| `include/zyarm_sdk/arm.hpp` | 想知道 `ZyArm` 对外提供了哪些控制方法 |
| `src/arm.cpp` | 想知道公开 API 如何映射到具体 CMD |
| `include/zyarm_sdk/config.hpp` | 想调整串口、超时、单位映射、安全限幅或遥操频率 |
| `include/zyarm_sdk/types.hpp` | 想理解 SDK 返回值的数据结构 |
| `protocol.hpp` / `protocol.cpp` | 固件新增 CMD、ACK、STATUS 或 `[MD]` 格式变化时 |
| `mapping.hpp` / `mapping.cpp` | 关节零位、符号、角度单位或夹爪范围需要适配时 |
| `safety.hpp` / `safety.cpp` | 需要限制关节范围、最大步进或状态过期时间时 |
| `transport.hpp` / `transport.cpp` | 串口打开、读写、后台接收线程或超时行为异常时 |
| `teleop.hpp` / `teleop.cpp` | 主从遥操、连续控制或 LeRobot 接入需要复用时 |

## 新增 C++ API

建议流程：

1. 在 `include/zyarm_sdk/` 头文件中声明公开 API。
2. 在 `src/` 中实现。
3. 如果 Python SDK 已有同名能力，尽量保持参数命名、单位和语义一致。
4. 如果新增或修改 CMD 编号，在 `protocol.hpp` / `protocol.cpp` 更新协议层。
5. 如果涉及单位、零位或夹爪范围，补 `mapping.hpp` / `mapping.cpp`。
6. 如果涉及安全边界，补 `SafetyConfig` 或 `safety.cpp`。
7. 在 `cpp/tests/` 增加 fake transport 或协议测试。
8. 必要时补 `cpp/examples/` 示例。
9. 更新 SDK README 或本章文档。

## C++ 特别注意

- 公开头文件中的类型和异常会影响外部编译依赖，新增 API 前要先想清楚参数和返回值是否稳定。
- 协议解析应尽量保持可测试，不依赖真实串口。
- 示例不要硬编码用户本地串口，正式工具应通过参数或配置传入端口。
- Windows 串口 `COM10` 及以上会由 `SerialTransport` 自动转换为 `\\.\COM10` 形式；调用方仍按 `COM10` 填写即可。
- C++ 与 Python SDK 应尽量保持同一套单位约定：关节为弧度，夹爪为 `0.0..1.0`。

## 待项目方补充

- C++ SDK 的编译器和平台支持范围。
- 是否需要安装包、动态库或静态库发布。
- C++ ABI 兼容性和版本规则。
