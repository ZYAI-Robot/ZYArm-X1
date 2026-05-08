# 修改已有 CMD 指令

已有固件命令的编号、名称、参数和回包会被文档、SDK、工具和上层框架依赖。修改前先确认这是兼容性修改还是破坏性修改。

## 先定位命令

1. 在 `firmware/Core/Inc/arm_shell.h` 中查找 `CMD_ID_*`。
2. 在 `firmware/Core/Src/arm_shell_cmd.c` 的 `g_shell_cmd_list` 中查找命令名称和 help。
3. 找到对应 handler，例如 `handle_status`、`handle_record`、`handle_master_slave`。
4. 阅读 handler 调用的业务模块，例如 `arm_robot.c`、`arm_robot_recorder.c`、`arm_remote.c`。

`g_shell_cmd_list` 是命令对外展示的关键来源，文档中的命令名称和 help 应与这里保持一致。

## 先用 CMD4 建立直观印象

`CMD4` 是读取固件版本的命令，输入形式是：

```text
[CMD][4]
```

它适合作为第一次阅读固件命令的例子，因为它不驱动机械臂运动，只返回版本信息。

第一步，在 `firmware/Core/Inc/arm_shell.h` 里可以看到命令 ID 的顺序。`CMD_ID_GET_VERSION` 前面有 4 个枚举项，所以它对应 `[CMD][4]`：

```c
typedef enum {
    CMD_ID_IK_INVERSE = 0,
    CMD_ID_RESET,
    CMD_ID_STOP,
    CMD_ID_JOINT_SYNC,
    CMD_ID_GET_VERSION,
    ...
} CMD_ID;
```

第二步，在 `firmware/Core/Src/arm_shell_cmd.c` 里找到 handler。handler 决定命令真正做什么、返回什么：

```c
static void handle_get_version(const ArmShellCmdPackage *cmd)
{
    send_string_response(cmd->cmd_id,
        "\nHardware Version: %s \nSoftware Version: %s \nBuild Date: %s\n",
        HW_VERSION, SW_VERSION, BUILD_DATE);
    send_ack_completed(cmd->cmd_id, 0);
}
```

第三步，继续看命令表注册。这里把 ID、命令名称、help 文案、handler 和参数解析格式绑在一起：

```c
[CMD_ID_GET_VERSION] = {
    "version",
    "Get firmware version information",
    handle_get_version,
    CMD_PARSE_FORMAT_FLOAT
},
```

把这三处连起来看，链路就是：

```text
[CMD][4]
  -> CMD_ID_GET_VERSION
  -> handle_get_version
  -> send_string_response + send_ack_completed
```

修改已有命令时，优先按这条链路确认“编号有没有变、handler 有没有变、help 有没有变、回包格式有没有变”。

## 参数解析注意

固件当前支持两种命令参数格式：浮点参数命令格式和字符串参数命令格式。两种格式都以 `[CMD][id]` 开头，区别在于第三段参数如何解析。

### 浮点参数命令格式

浮点参数命令用于数字类参数，例如关节角度、末端位姿、频率、开关值和阈值。输入格式是：

```text
[CMD][id][p0 p1 p2 ...]
```

例如 `CMD5 set_joint` 控制单个关节：

```text
[CMD][5][0 30]
```

它在命令表中注册为 `CMD_PARSE_FORMAT_FLOAT`。解析后，handler 通过 `cmd->params[index]` 读取参数，通过 `cmd->param_count` 判断参数数量：

```c
int joint_id = cmd->params[0];
int angle = cmd->params[1];
```

### 字符串参数命令格式

字符串参数命令用于名称类参数，例如机械臂名称、录制动作名称。输入格式是：

```text
[CMD][id][text]
```

例如 `CMD21 set_name` 设置机械臂名称：

```text
[CMD][21][master]
```

它在命令表中注册为 `CMD_PARSE_FORMAT_STRING`。解析后，handler 通过 `cmd->str_param` 读取字符串：

```c
strncpy(g_arm_robot.cfg.name, cmd->str_param, ARM_NAME_MAX_LEN - 1);
```

字符串参数长度受 `ARM_SHELL_CMD_STRING_PARAM_MAX_LEN` 限制。修改字符串类命令时，需要确认名称长度、非法字符、存储位置和掉电后的持久化结果。

### 如何从命令表判断格式

看 `g_shell_cmd_list` 最后一列即可判断命令使用哪种格式：

| 注册格式 | 用户输入示例 | handler 中读取方式 |
| --- | --- | --- |
| `CMD_PARSE_FORMAT_FLOAT` | `[CMD][5][0 30]` | `cmd->params[0]`、`cmd->params[1]` |
| `CMD_PARSE_FORMAT_STRING` | `[CMD][21][master]` | `cmd->str_param` |

例如 `CMD21 set_name` 的注册项应使用 `CMD_PARSE_FORMAT_STRING`，否则 `[CMD][21][master]` 不会按字符串解析。

```c
[CMD_ID_SET_NAME] = {
    "set_name",
    "Set robot arm name with 1 string parameter: name",
    handle_set_name,
    CMD_PARSE_FORMAT_STRING
},
```

## 回包和日志注意

固件中常见输出包括：

- `ACK_RECEIVED`
- `ACK_COMPLETED`
- `ACK_RESPONSE`
- `[STATUS]`
- `help` 命令表输出
- `ARM_LOG*` 日志

如果上位机代码依赖固定文本解析，修改输出格式前需要先搜索 `software/` 和 `docs/` 中的引用。

以 `CMD4` 为例，如果只是调整版本文案，风险较低；如果把 `Hardware Version`、`Software Version` 或 `Build Date` 的字段名改掉，就可能影响工具脚本、测试脚本或用户文档中对版本输出的判断。

## 具体修改例：调整 CMD4 版本输出

假设要给 `CMD4` 增加一个新的只读字段，建议按下面顺序做：

1. 在 `handle_get_version` 中增加字段来源和输出格式。
2. 保持 `[CMD][4]` 编号不变，不要修改 `CMD_ID_GET_VERSION` 的位置。
3. 如果 `help` 文案需要更准确，同步修改 `g_shell_cmd_list` 中的描述。
4. 搜索 `CMD4`、`version`、`Hardware Version`、`Software Version`，确认文档和脚本是否依赖旧输出。
5. 烧录后执行 `help` 和 `[CMD][4]`，保存串口输出作为验证记录。

如果只是新增字段，通常应尽量追加到输出末尾，减少旧脚本解析失败的概率。

## 修改后验证

建议至少执行：

```text
help
[CMD][4]
[CMD][6]
```

然后执行被修改的命令，并确认：

- 参数错误时有可理解的错误输出。
- 正常参数能返回预期 ACK 或响应。
- 真实动作符合文档描述。
- SDK、工具或诊断脚本没有解析失败。

## 待项目方补充

- 哪些已有命令属于稳定公开 API，哪些属于开发/诊断命令。
- 破坏性协议变更的版本号规则和迁移策略。
- ACK、日志和错误码的正式对外格式规范。
