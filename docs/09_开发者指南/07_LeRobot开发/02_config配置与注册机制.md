# config 配置与注册机制

`config.py` 负责把 ZYArm 插件注册到 LeRobot。开发者最先需要看这里，因为 LeRobot 命令中的 `--robot.type=zyarm_follower` 和 `--teleop.type=zyarm_leader` 都来自这里。

## 注册入口

关键代码位于 `software/lerobot_robot_zyarm/src/lerobot_robot_zyarm/config.py`：

```python
@RobotConfig.register_subclass("zyarm_follower")
@dataclass
class ZyArmFollowerRobotConfig(RobotConfig):
    ...

@TeleoperatorConfig.register_subclass("zyarm_leader")
@dataclass
class ZyArmLeaderTeleoperatorConfig(TeleoperatorConfig):
    ...
```

含义：

| 注册名 | 对应角色 | 对应实现 |
| --- | --- | --- |
| `zyarm_follower` | LeRobot robot，执行动作的 follower | `ZyArmFollowerRobot` |
| `zyarm_leader` | LeRobot teleoperator，提供动作输入的 leader | `ZyArmLeaderTeleoperator` |

## follower 配置字段

| 字段 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | `""` | follower 串口 |
| `baudrate` | `230400` | 串口波特率 |
| `timeout_s` | `0.02` | 串口读取超时 |
| `write_timeout_s` | `0.05` | 串口写入超时 |
| `ack_timeout_s` | `1.0` | 等待 ACK 的超时 |
| `action_timeout_s` | `10.0` | 动作相关超时 |
| `play_record_timeout_s` | `190.0` | 固件动作回放相关超时 |
| `state_max_age_ms` | `100.0` | 读取缓存状态时允许的最大年龄 |
| `initial_state_timeout_ms` | `1000.0` | 连接时首次读取状态的超时 |
| `query_state_on_missing_cache` | `True` | 缓存状态缺失时是否主动查询 |
| `slave_filter_lpf_alpha` | `0.15` | follower slave filter 低通滤波系数 |
| `cameras` | `{}` | LeRobot camera 配置 |
| `mapping` | `MappingConfig()` | SDK 映射配置 |
| `safety` | `SafetyConfig()` | SDK 安全配置 |

这些字段会影响 `robot.py` 中的 SDK 连接、状态读取、slave filter 和相机管理。

## leader 配置字段

| 字段 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | `""` | leader 串口 |
| `baudrate` | `230400` | 串口波特率 |
| `leader_hz` | `50.0` | leader 动作读取频率 |
| `action_max_age_ms` | `100.0` | leader action 最大年龄 |
| `wait_timeout_ms` | `50.0` | 等待 leader action 的超时 |
| `mapping` | `MappingConfig()` | SDK 映射配置 |
| `retarget` | `RetargetConfig()` | leader 到 follower 的重定向配置 |

`leader_hz`、`action_max_age_ms` 和 `wait_timeout_ms` 最终会进入 SDK 的 `TeleopConfig`。

## SDK config 构造

`robot.py` 和 `teleoperator.py` 都会把 LeRobot config 转成 SDK config：

```python
return ZyArmConfig(
    port=config.port,
    baudrate=config.baudrate,
    timeout_s=config.timeout_s,
    write_timeout_s=config.write_timeout_s,
    ack_timeout_s=config.ack_timeout_s,
)
```

如果你新增串口、映射或安全相关配置，需要同时检查：

- config dataclass 是否新增字段。
- `_make_sdk_config()` 是否传入该字段。
- 相关测试是否覆盖默认值和传参。
- [科研与数据采集](../../07_科研与数据采集/README.md) 是否需要更新使用说明。

## 修改建议

- 只改默认值时，要确认旧数据采集命令是否仍然合理。
- 新增字段时，要给出默认值，避免破坏现有命令。
- 修改注册名会影响 `--robot.type` 和 `--teleop.type`，通常不建议轻易修改。
- 修改 `slave_filter_lpf_alpha` 会影响 follower 手感和延迟，需要做短时间 teleoperate / record 验证。

## 待项目方补充

> 待项目方补充：请提供 LeRobot 版本升级策略、官方推荐默认参数、不同任务下的滤波参数建议和配置兼容性矩阵。
