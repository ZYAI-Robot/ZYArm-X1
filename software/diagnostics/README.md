# software/diagnostics

这里存放压测、可靠性验证和诊断脚本。

当前脚本主要覆盖：

- 随机移动和固定移动压力测试
- 出厂短时老化测试
- fast joint IO 压测
- 上下电循环测试
- 录制动作和远程命令相关验证
- 固件 ACK/help 输出检查

这些脚本面向调试和验证，默认依赖 `software/zyarm_sdk/python/src/zyarm_sdk` 的新 SDK API。

`power_cycle_test.py` 仍会临时导入旧的 `URPT8B0.py` 继电器协议；该协议不属于机械臂 SDK 主 API。

## 出厂短时老化测试

`factory_burn_in_test.py` 用于商品出厂前的短时稳定性循环测试。标准运行命令：

```bash
python software/diagnostics/factory_burn_in_test.py --port COM3 --sn ZA000001
```

默认行为：

- 测试时长 30 分钟，可用 `--duration-min` 覆盖。
- 环境温度默认 25°C，可用 `--ambient-temp-c` 手动填写。
- 固定点位顺序为 `P1 -> P2 -> P3 -> P4 -> P5 -> P6 -> P1`。
- 每个点位 `move_ik()` 完成后默认等待 1 秒，可用 `--point-delay-s` 覆盖。
- 温度默认每 20 秒采样一次，warn 阈值 60°C，fail 阈值 70°C。
- 默认开启 SDK 串口日志落盘，日志写入本次报告目录，默认 1 秒普通 flush；如不需要可加 `--no-serial-log`。
- 输出 JSON 总结报告、CSV 过程记录和串口日志路径。

产线导入前建议确认：

- 默认 warn/fail 温度阈值是否匹配最终舵机规格和工厂环境。
- 默认 6 个点位是否覆盖最终产品的典型运动范围。
- 默认 30 分钟节拍是否满足产线吞吐要求。

无硬件验证报告格式时可使用 dry run：

```bash
python software/diagnostics/factory_burn_in_test.py --port COM3 --sn DRYRUN001 --dry-run --duration-min 0.05 --point-delay-s 0
```
