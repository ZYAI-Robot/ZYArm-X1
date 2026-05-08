# 新增 CMD 指令

新增 `[CMD]` 是跨层改动。它不仅是固件代码变化，还可能影响基础操作、Serial API、SDK、工具、诊断和上层框架。

## 常见影响链

新增公开命令时，通常按这条链路检查：

```text
firmware/Core/Inc/arm_shell.h
  -> firmware/Core/Src/arm_shell_cmd.c
  -> docs/04_基础操作
  -> docs/06_仿真与框架接入/Serial_API
  -> software/zyarm_sdk
  -> software/tools 或 software/diagnostics
  -> 上层框架按需接入
```

如果命令只用于开发诊断，不一定要进入快速上手或常用玩法，但仍应在基础操作或开发者指南里说明“开发/诊断用途”。

## 新增前先确认

- 现有命令是否已经能满足需求。
- 这个命令是用户功能、上位机内部能力，还是开发诊断能力。
- 是否会真实运动、写 Flash、改变设备名称、改变控制模式或改变状态上报。
- 是否需要 SDK 封装，还是只在开发者文档里说明。
- 是否需要保留向后兼容。

## 固件实现步骤

1. 在 `firmware/Core/Inc/arm_shell.h` 的 `CMD_ID` 枚举中选择新 ID。
2. 在 `firmware/Core/Src/arm_shell_cmd.c` 中实现新的 `handle_*` 函数。
3. 在 `g_shell_cmd_list` 中注册名称、help、handler 和解析格式。
4. 如果逻辑较复杂，把业务代码放入独立模块，不要把所有逻辑堆在 handler 内。
5. 如果新增 `.c` 文件，确认 Keil 工程包含该文件。
6. 编译并烧录。
7. 用 `help`、新命令、`CMD6` 和相关诊断脚本验证。

## 参考例子：CMD35 get_transport_stats

新增命令时，建议先学习只读诊断命令。`CMD35 get_transport_stats` 是一个比较好的参考：它读取通信和解析统计，不会驱动机械臂运动，风险比运动类命令低。

第一步，`firmware/Core/Inc/arm_shell.h` 中为命令分配 ID。新增命令通常放在已有公开命令之后、`CMD_ID_MAX_NUM` 之前：

```c
CMD_ID_MASTER_SLAVE_SET_LPF,
CMD_ID_GET_TRANSPORT_STATS,
CMD_ID_JOINT_IO_FAST,
CMD_ID_MAX_NUM,
```

第二步，在 `firmware/Core/Src/arm_shell_cmd.c` 中实现 handler。这个 handler 先读取统计数据，再通过 `send_stream_response` 输出多行诊断信息，最后发送完成 ACK：

```c
static void handle_get_transport_stats(const ArmShellCmdPackage *cmd)
{
    ArmShellTransportStats stats = {0};
    ArmShellParserStats parser_stats = {0};

    arm_shell_get_transport_stats(&stats);
    arm_shell_get_parser_stats(&parser_stats);
    send_ack_received(cmd->cmd_id);
    send_stream_response(cmd->cmd_id, "CMD_PARSER_STATS:OK=%lu,ERROR=%lu,OVERFLOW=%lu\n",
        (unsigned long)parser_stats.parse_success_count,
        (unsigned long)parser_stats.parse_error_count,
        (unsigned long)parser_stats.overflow_count);
    send_ack_completed(cmd->cmd_id, 0);
}
```

第三步，在 `g_shell_cmd_list` 中注册命令。注册项要写清名称、help、handler 和参数解析格式：

```c
[CMD_ID_GET_TRANSPORT_STATS] = {
    "get_transport_stats",
    "Get firmware UART1/UART6 communication health snapshot",
    handle_get_transport_stats,
    CMD_PARSE_FORMAT_FLOAT
},
```

这条命令输入为：

```text
[CMD][35]
```

它的设计重点是：

- 只读查询，不产生真实运动。
- 不需要字符串参数，所以使用 `CMD_PARSE_FORMAT_FLOAT`。
- 输出文本带稳定前缀，例如 `CMD_PARSER_STATS`，方便工具脚本解析。
- 执行结束必须调用 `send_ack_completed(cmd->cmd_id, ret)`。

## 命令 ID 和名称

命令 ID 应保持稳定。不要复用已经公开使用过的 ID，除非明确这是破坏性版本。

命令名称建议：

- 使用小写英文和下划线。
- 名称表达动作，不表达实现细节。
- help 中写清参数数量、顺序、单位和可选参数。

## 参数和单位

参数文档必须写清：

| 内容 | 示例 |
| --- | --- |
| 参数数量 | `x y z rx ry rz` |
| 单位 | 角度、弧度、毫米、归一化 `0.0..1.0` |
| 范围 | `0/1`、`0.0-1.0`、频率上限 |
| 默认值 | 省略可选参数时的行为 |
| 同步语义 | 是否等待动作完成 |

如果参数会传到 SDK、ROS 2 或 LeRobot，优先使用已有单位约定，不要让同一字段在不同层表达不同含义。

## 新增只读命令的推荐模板

如果你要新增一个类似“查询当前配置/诊断信息”的命令，可以先按这个模板设计：

```c
static void handle_get_xxx(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    send_string_response(cmd->cmd_id, "XXX:FIELD=%d\n", value);
    send_ack_completed(cmd->cmd_id, 0);
}
```

然后在命令表中注册：

```c
[CMD_ID_GET_XXX] = {
    "get_xxx",
    "Get xxx information",
    handle_get_xxx,
    CMD_PARSE_FORMAT_FLOAT
},
```

如果命令需要名称、文件名或动作名这类文本参数，再改用 `CMD_PARSE_FORMAT_STRING`，并从 `cmd->str_param` 读取参数。

## 安全语义

新增命令如果会让机械臂运动，文档必须说明：

- 是否需要先复位。
- 是否建议空载测试。
- 是否会卸力。
- 是否可能接近限位。
- 异常时是否优先断电。

不要把未经验证的停止、急停、保护逻辑写成安全承诺。

## 上层同步

根据命令用途决定同步范围：

- 用户会手工发送：更新 [基础操作](../../04_基础操作/README.md)。
- SDK 会调用：更新 [SDK 开发](../04_SDK开发/README.md) 和 SDK 文档/测试。
- 工具会调用：更新 `software/tools/README.md`。
- 用于可靠性验证：新增或更新 `software/diagnostics/`。
- 影响 ROS 2、Web、LeRobot：同步对应开发章节和用户章节。