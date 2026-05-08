# software/examples

这里存放机械臂的示例和演示程序，适合学习如何基于 ZYArm SDK、相机 SDK 和串口协议实现具体应用。

## 目录结构

```text
software/examples/
├── robot_arm_pick/          # 基础机械臂抓取示例
├── camera_pick/
│   ├── orbbec/              # Orbbec 相机抓取示例
│   └── deptrum/             # Deptrum 相机触摸/抓取参考示例
└── build.sh                 # Orbbec 示例构建脚本
```

## 构建 Orbbec 示例

```bash
cd software/examples
./build.sh
```

只构建指定示例：

```bash
./build.sh --example robot_arm_pick
./build.sh --example camera_pick/orbbec
```

清理后重新构建：

```bash
./build.sh --clean
```

该脚本会：

1. 自动将 Orbbec SDK 克隆到 `software/examples/third_party/OrbbecSDK/`。
2. 构建 Orbbec SDK。
3. 构建 `robot_arm_pick` 和 `camera_pick/orbbec` 示例。
4. 将示例可执行文件复制到 `software/examples/build/bin/`。

## 构建 Deptrum 示例

Deptrum 示例依赖外部相机 SDK，使用独立脚本：

```bash
cd software/examples/camera_pick/deptrum
./build.sh /path/to/deptrum-stream-sdk
```

构建完成后，产物位于：

```text
software/examples/camera_pick/deptrum/build/
```

## SDK 依赖

示例中的机械臂访问代码来自新 SDK：

```text
software/zyarm_sdk/python/
software/zyarm_sdk/cpp/
```

旧 `RobotActor` 示例入口已迁移到 `zyarm_sdk::ZyArm` 或 Python `zyarm_sdk.ZyArm`。C++ 示例通过 `software/zyarm_sdk/cpp` 构建和链接。
