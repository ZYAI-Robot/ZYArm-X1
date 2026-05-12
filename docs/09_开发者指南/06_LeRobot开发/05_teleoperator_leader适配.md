# teleoperator leader 适配

`teleoperator.py` 实现 LeRobot 的 leader teleoperator。它代表人工输入来源，负责连接 leader、启动 SDK 遥操读取、把 leader action 转成 follower 可执行的 LeRobot action。

源码入口：`software/lerobot_robot_zyarm/src/lerobot_robot_zyarm/teleoperator.py`

## 类入口

```python
class ZyArmLeaderTeleoperator(Teleoperator):
    config_class = ZyArmLeaderTeleoperatorConfig
    name = "zyarm_leader"
```

`name` 与 `--teleop.type=zyarm_leader` 对应。

## 生命周期

```text
__init__()
  -> connect()
  -> get_action()
  -> send_feedback()
  -> disconnect()
```

## 初始化

`__init__()` 支持两种方式：

- 传入 `leader` 假对象，用于测试。
- 根据 config 创建 `ZyArm` 和 `ZyArmLeader`，用于真实运行。

```python
self.arm = arm or ZyArm(self._make_sdk_config(config))
self.leader = ZyArmLeader(self.arm, self._make_teleop_config(config))
self.retargeter = Retargeter(config.retarget)
```

`Retargeter` 负责把 leader 读取到的位置映射成 follower 可执行的位置。它的配置来自 `RetargetConfig`。

## connect()

```python
if not getattr(self.leader.arm, "is_connected", False):
    self.leader.arm.connect()
self.leader.start()
self.configure()
```

含义：

- 先连接 leader 串口。
- 再启动 SDK 的 leader 读取循环。
- `configure()` 当前为空实现，保留给后续扩展。

## get_action()

```python
action = self.leader.get_action(wait=True, timeout_ms=self.config.wait_timeout_ms)
if action is None:
    raise RuntimeError("No zyarm leader action available")
positions = self.retargeter.apply(action)
return positions_to_action(positions)
```

数据流：

```text
ZyArmLeader
  -> get_action()
  -> Retargeter.apply()
  -> positions_to_action()
  -> LeRobot action
```

如果 leader 暂时没有 action，会抛出异常。`wait_timeout_ms` 决定等待时间。

## feedback

```python
def send_feedback(self, feedback):
    del feedback
    return None
```

当前 ZYArm leader 不使用 LeRobot feedback。如果后续需要力反馈、状态灯或其他反向提示，可以从这里扩展，但要先明确 LeRobot feedback 字段和 SDK 支持能力。

## disconnect()

```python
if getattr(self.leader.arm, "is_connected", False):
    self.leader.stop()
    self.leader.arm.close()
```

断开时先停止 leader 读取，再关闭串口。

## 修改检查

- 改 `leader_hz`：检查 `TeleopConfig` 和实际动作新鲜度。
- 改 `wait_timeout_ms`：检查 `get_action()` 超时和 LeRobot record 是否稳定。
- 改 retarget：检查 leader/follower 角度映射、夹爪范围和 replay。
- 增加 feedback：检查 LeRobot feedback schema 和 SDK 是否支持。
- 改 action 输出字段：同步检查 [features 与 conversion 数据字段](03_features与conversion数据字段.md)。

## 待项目方补充

> 待项目方补充：请提供官方 retarget 配置策略、leader/follower 几何差异处理方式、feedback 支持规划和推荐 leader 读取频率。
