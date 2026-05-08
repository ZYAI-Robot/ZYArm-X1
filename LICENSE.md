# ZYArmV1 开源协议说明

本仓库采用分层授权策略。原因是 ZYArmV1 同时包含软件、固件、文档、机器人描述资源、mesh 资产和品牌材料，它们的开放目标和商业边界不同。

本文件用于说明本项目公开发布时的授权边界，不构成法律意见。如果某个文件或目录内有更具体的许可证声明，以更具体的声明为准。

## 授权摘要

| 内容范围 | 适用协议 | 说明 |
| --- | --- | --- |
| `firmware/` 之外的软件源码，包括 SDK、ROS 2 包、Web 应用、示例、工具、诊断脚本、构建文件等 | Apache License 2.0 | 协议正文见 [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)。 |
| `firmware/` 下项目原创固件源码 | Mozilla Public License 2.0 | 协议正文见 [LICENSES/MPL-2.0.txt](LICENSES/MPL-2.0.txt)。STM32CubeMX 生成文件、ST HAL、CMSIS、FreeRTOS、舵机/硬件厂商适配文件和其他第三方材料继续遵循其自身协议。 |
| URDF、xacro、机器人描述配置、launch 文件、控制器配置、运动学模型数据 | Apache License 2.0 | 用于开放仿真、ROS、MoveIt、Gazebo 和兼容开发所需的机器人描述接口。 |
| 明确作为公开资产发布的脱敏 mesh，用于可视化、仿真、教学或文档展示 | Creative Commons Attribution-ShareAlike 4.0 International | 协议正文见 [LICENSES/CC-BY-SA-4.0.txt](LICENSES/CC-BY-SA-4.0.txt)。生产级或高精度 mesh 不属于公开发布资产。 |
| `docs/` 下文档和教程 | Creative Commons Attribution-ShareAlike 4.0 International | 文档中的代码片段默认遵循其所描述代码的对应协议，除非另有说明。 |
| `ZYArm`、`ZYArmV1`、`ZYArm-X1`、Logo、产品名称、品牌资产和商标 | 保留所有权利 | 开源协议不授予商标或品牌使用权。详见 [TRADEMARKS.md](TRADEMARKS.md)。 |
| 生产 CAD、高精生产 mesh、BOM、制造工艺、工厂标定数据、产测工具、私有密钥、未发布硬件设计 | 不公开，不授权 | 这些材料不属于公开仓库范围。 |

## 公开硬件边界

本公开仓库的目标是支持学习、研究、教育、仿真和围绕 ZYArm 硬件的兼容软件开发。

本公开仓库不包含，也不授权使用，生产 ZYArm 机械臂本体所需的生产 CAD、生产级 mesh、BOM、制造工艺文件、工厂标定流程、质检流程、私有签名密钥或其他生产资料。

在公开发布前，任何可能暴露生产几何细节的 mesh 或 CAD 衍生资产，都应替换为脱敏后的公开版本，或从公开仓库及公开历史中移除。

## 固件边界

`firmware/` 下项目原创固件源码计划以 MPL-2.0 发布。用户可以学习、修改、编译和刷写固件；如果分发修改过的 MPL 覆盖文件，需要继续按 MPL-2.0 提供相应源码。

`firmware/` 中并非所有文件都属于项目原创固件源码。STM32CubeMX 生成文件、ST HAL、CMSIS、FreeRTOS、舵机驱动、硬件驱动、厂商 SDK 或示例代码等第三方或厂商来源文件，应保留其原始版权声明和许可证。

修改固件可能影响设备运动、安全、性能、标定和质保状态。项目方可以提供官方恢复固件和推荐刷机说明，但除非另有书面协议，修改版固件由使用者自行承担风险。

## 第三方材料

仓库中可能包含第三方软件、厂商文件、生成文件或工具链组件。这些材料继续遵循其原始许可证和声明。相关说明见 [NOTICE.md](NOTICE.md)，具体许可证文件通常位于对应第三方组件附近。

## 贡献授权

贡献默认按被修改文件或目录适用的协议授权，除非维护者与贡献者另有书面约定。详见 [CONTRIBUTING.md](CONTRIBUTING.md)。
