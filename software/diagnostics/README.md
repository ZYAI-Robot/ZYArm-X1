# software/diagnostics

这里存放压测、可靠性验证和诊断脚本。

当前脚本主要覆盖：

- 随机移动和固定移动压力测试
- fast joint IO 压测
- 上下电循环测试
- 录制动作和远程命令相关验证
- 固件 ACK/help 输出检查

这些脚本面向调试和验证，默认依赖 `software/zyarm_sdk/python/src/zyarm_sdk` 的新 SDK API。

`power_cycle_test.py` 仍会临时导入旧的 `URPT8B0.py` 继电器协议；该协议不属于机械臂 SDK 主 API。
