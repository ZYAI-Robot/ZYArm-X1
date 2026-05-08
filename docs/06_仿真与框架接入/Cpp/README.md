# C++ 接入

本页说明如何通过 C++ SDK 快速控制机械臂。C++ 路线适合性能敏感控制程序、系统集成、C++ 示例验证，以及后续和 ROS 2 底层组件协同时复用同一套串口协议封装。

C++ SDK 是对底层 Serial API 的封装。你不需要手工拼 `[CMD]` 文本，而是通过 `ZyArm`、`ZyArmConfig`、`ArmState`、`fast_io()` 等 C++ 接口完成状态读取、复位、连续控制和主从遥操相关能力。

如果你要修改 C++ SDK 源码、扩展 API 或补测试，请阅读 [C++ SDK 开发](../../09_开发者指南/04_SDK开发/02_Cpp_SDK开发.md)。本页只保留使用入口和最小示例。

## 开发环境

本页只覆盖 Ubuntu/Linux 下的 C++ SDK 使用流程。

C++ SDK 位于 `software/zyarm_sdk/cpp/`，当前使用 C++17 和 CMake。

建议准备：

- CMake 3.16 或更高版本。
- 可用的 C++17 编译器，例如 GCC 或 Clang。
- 已完成 [快速上手](../../02_快速上手/README.md)，并确认机械臂串口可用。
- 记录机械臂串口设备名，例如 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。

## 构建示例

在仓库根目录执行：

```bash
cmake -S software/zyarm_sdk/cpp -B build/zyarm_sdk_cpp -DZYARM_SDK_BUILD_EXAMPLES=ON -DZYARM_SDK_BUILD_TESTS=OFF
cmake --build build/zyarm_sdk_cpp -j
```

`ZYARM_SDK_BUILD_EXAMPLES=ON` 会编译示例程序；`ZYARM_SDK_BUILD_TESTS=OFF` 用于先减少学习者第一次构建时的干扰。如果你要验证 SDK 实现，再打开测试。

编译完成后，示例可执行文件通常位于 `build/zyarm_sdk_cpp/`，例如：

```bash
./build/zyarm_sdk_cpp/read_state
./build/zyarm_sdk_cpp/fast_io_loop
./build/zyarm_sdk_cpp/teleop_step
```

第一次接入建议先运行 `read_state`，确认 C++ SDK 能读取真实机械臂状态：

```bash
./build/zyarm_sdk_cpp/read_state
```

当前示例里的端口默认按 Ubuntu 设备名编写，例如 `/dev/ttyUSB0`。如果你的端口不同，请先把示例中的端口改成实际设备名。

## 新增自己的 C++ 程序

如果要新增一个 `.cpp` 文件并调用 C++ SDK，可以先把文件放在 `software/zyarm_sdk/cpp/examples/` 下，例如：

```text
software/zyarm_sdk/cpp/examples/my_read_state.cpp
```

然后在 `software/zyarm_sdk/cpp/CMakeLists.txt` 的 `if(ZYARM_SDK_BUILD_EXAMPLES)` 代码块中增加一个可执行目标：

```cmake
add_executable(my_read_state examples/my_read_state.cpp)
target_link_libraries(my_read_state PRIVATE zyarm_sdk)
```

这两行的含义是：

- `add_executable()` 告诉 CMake 把 `my_read_state.cpp` 编译成名为 `my_read_state` 的可执行文件。
- `target_link_libraries()` 把这个程序链接到 C++ SDK，否则程序无法使用 `zyarm_sdk/arm.hpp` 里的实现。

新增目标后，重新配置并编译：

```bash
cmake -S software/zyarm_sdk/cpp -B build/zyarm_sdk_cpp -DZYARM_SDK_BUILD_EXAMPLES=ON -DZYARM_SDK_BUILD_TESTS=OFF
cmake --build build/zyarm_sdk_cpp -j
```

编译完成后运行：

```bash
./build/zyarm_sdk_cpp/my_read_state
```

第一次新增程序建议只做 `query_state()` 状态读取。`reset()`、`move_ik()`、`fast_io()` 都可能驱动真实机械臂，运行前必须确认机械臂周围没有障碍物，并且可以随时断电。

## 重新编译

如果只是修改已有 `.cpp` 文件，不需要删除 `build/zyarm_sdk_cpp/`，也通常不需要重新执行 `cmake -S ...`，直接重新编译即可：

```bash
cmake --build build/zyarm_sdk_cpp -j
```

如果只想重新编译某一个程序，可以指定目标名：

```bash
cmake --build build/zyarm_sdk_cpp --target my_read_state -j
```

如果新增了 `.cpp` 对应的 `add_executable()`、修改了 `CMakeLists.txt`，或调整了 CMake 选项，建议先重新配置，再编译：

```bash
cmake -S software/zyarm_sdk/cpp -B build/zyarm_sdk_cpp -DZYARM_SDK_BUILD_EXAMPLES=ON -DZYARM_SDK_BUILD_TESTS=OFF
cmake --build build/zyarm_sdk_cpp -j
```

## 推荐入口

| 入口 | 作用 |
| --- | --- |
| [software/zyarm_sdk](../../../software/zyarm_sdk/README.md) | SDK 源码目录说明 |
| `software/zyarm_sdk/cpp/examples/read_state.cpp` | 状态读取示例 |
| `software/zyarm_sdk/cpp/examples/fast_io_loop.cpp` | 高频下发示例 |
| `software/zyarm_sdk/cpp/examples/teleop_step.cpp` | 主从遥操 step 示例 |
| `software/zyarm_sdk/cpp/include/zyarm_sdk/arm.hpp` | 最常用的机械臂控制类声明 |

## 最小状态读取

这个示例等价于“发送 `CMD6` 并解析 `[STATUS]`”，适合确认 C++ SDK 已经能连上真实机械臂。

```cpp
#include <chrono>
#include <iostream>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "/dev/ttyUSB0";

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

`query_state()` 会主动查询一次机械臂状态，并等待 fresh `[STATUS]`。如果超时返回空值，通常要先检查串口号、权限、机械臂供电和串口是否被其他程序占用。

## 单位约定

- 六个机械臂关节使用弧度。
- 夹爪使用 `0.0..1.0` 归一化值。
- SDK/ROS 使用用户角度表达，和固件原始角度不同。
- `fast_io()` 适合连续控制链路验证，但第一次运行前必须先确认状态读取和复位稳定。

不要把 SDK 里的关节值直接当成固件 `[STATUS]` 里的角度复制回串口命令。需要直接发底层命令时，请回到 [Serial API 接入](../Serial_API/README.md)。

## 常见下一步

| 目标 | 下一步 |
| --- | --- |
| 只读取状态 | 构建并运行 `read_state` |
| 验证连续控制链路 | 先阅读 [主从臂遥操](../../05_常用玩法/04_主从臂遥操.md)，再理解 `fast_io_loop.cpp` |
| 做 C++ 主从遥操封装 | 阅读 `teleop_step.cpp` 和 [C++ SDK 开发](../../09_开发者指南/04_SDK开发/02_Cpp_SDK开发.md) |
| 直接调试底层协议 | 阅读 [Serial API 接入](../Serial_API/README.md) |
| 修改 SDK 实现 | 阅读 [SDK 开发](../../09_开发者指南/04_SDK开发/README.md) |

## 待项目方补充

- 推荐 Ubuntu 下的 CMake 生成器和编译器组合。
- 面向课程的 C++ 示例清单。
- 常见串口权限、编译错误和运行错误截图。
