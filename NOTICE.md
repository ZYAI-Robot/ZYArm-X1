# ZYArmV1 声明

Copyright (c) ZYArm project contributors.

ZYArmV1 由项目原创代码与资产、第三方组件、厂商提供的固件支持文件、生成的机器人描述资源以及开源依赖共同组成。

## 项目声明

- `firmware/` 之外的软件源码默认按 Apache License 2.0 授权，除非更具体的文件或目录声明另有规定。
- `firmware/` 下项目原创固件源码默认按 Mozilla Public License 2.0 授权，除非更具体的文件或目录声明另有规定。STM32CubeMX 生成文件、ST HAL、CMSIS、FreeRTOS、舵机/硬件厂商适配文件和其他第三方材料继续遵循其自身协议。
- 文档和脱敏后的公开 mesh 资产默认按 Creative Commons Attribution-ShareAlike 4.0 International 授权，除非更具体的文件或目录声明另有规定。
- 产品名称、Logo 和品牌标识不随开源协议或 Creative Commons 协议授权。详见 [TRADEMARKS.md](TRADEMARKS.md)。

## 第三方组件

本仓库可能包含或引用第三方材料，包括但不限于：

- `firmware/Drivers/` 下的 STM32 HAL、CMSIS 和芯片支持文件。
- `firmware/Middlewares/Third_Party/FreeRTOS/` 下的 FreeRTOS 文件。
- `firmware/Core/` 下由 STM32CubeMX 或厂商工具生成的启动、外设初始化和配置文件。
- ROS 2、MoveIt、Gazebo、LeRobot、Python、C++ 和 Web 生态依赖。
- 摄像头、舵机、串口或硬件集成示例中使用的厂商 SDK 或示例文件。

第三方文件继续受其自身许可证约束。若上游项目或厂商提供许可证文件，本仓库应尽量将其保留在对应组件附近。

## 公开发布提醒

生产 CAD、高精生产 mesh、BOM、制造工艺文件、工厂标定数据、产测工具、私有密钥和未发布硬件设计不属于公开开源发布范围。

在将本仓库公开前，应将生产级 mesh 资产替换为脱敏后的公开 mesh，或从公开发布历史中移除。
