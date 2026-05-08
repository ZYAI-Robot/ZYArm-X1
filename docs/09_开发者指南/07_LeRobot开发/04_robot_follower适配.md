# robot follower 适配

`robot.py` 实现 LeRobot 的 follower robot。它代表真实执行动作的机械臂，负责连接 follower、读取 observation、读取相机、接收 action 并下发到 SDK。

源码入口：`software/lerobot_robot_zyarm/src/lerobot_robot_zyarm/robot.py`

## 类入口

```python
class ZyArmFollowerRobot(Robot):
    config_class = ZyArmFollowerRobotConfig
    name = "zyarm_follower"
```

`config_class` 连接到 [config 配置与注册机制](02_config配置与注册机制.md)，`name` 与 `--robot.type=zyarm_follower` 对应。

## 生命周期

```text
__init__()
  -> connect()
  -> get_observation()
  -> send_action()
  -> disconnect()
```

## 初始化

`__init__()` 会创建三类对象：

| 对象 | 来源 | 作用 |
| --- | --- | --- |
| `self.arm` | `ZyArm(self._make_sdk_config(config))` | SDK 机械臂实例 |
| `self.safety` | `SafetyController(config.safety)` | action 安全处理 |
| `self.cameras` | `make_cameras_from_configs(config.cameras)` | LeRobot camera 实例 |

测试可以通过 `arm=` 或 `cameras=` 注入假对象，避免依赖真实设备。

## connect()

核心流程：

```text
connect()
  -> arm.connect()
  -> arm.query_state()
  -> _start_slave_filter()
  -> camera.connect()
  -> configure()
```

关键点：

- 连接后会先读取一次状态，确认 follower 可用。
- follower 会进入固件 slave filter 相关控制语义。
- 相机连接失败时会调用 `disconnect()` 清理。

## slave filter

```python
result = self.arm.set_master_slave_lpf(self.config.slave_filter_lpf_alpha)
result = self.arm.enter_slave_mode()
```

`slave_filter_lpf_alpha` 来自 follower config。这个值影响 follower 平滑和响应。修改它时，既要看手感，也要看数据采集延迟。

退出时：

```python
result = self.arm.stop_master_mode()
```

如果停止失败，当前实现会发出 warning，而不是直接抛出异常。

## get_observation()

```python
state = self.arm.get_latest_state(self.config.state_max_age_ms)
if state is None and self.config.query_state_on_missing_cache:
    state = self.arm.query_state(timeout_ms=self.config.initial_state_timeout_ms)

observation = positions_to_action(state.positions)
for name, camera in self.cameras.items():
    observation[name] = camera.read_latest()
```

含义：

- 优先读取 SDK 缓存状态。
- 缓存缺失时，可按配置主动查询状态。
- 关节和夹爪状态通过 `positions_to_action()` 转成 LeRobot 字段。
- 相机图像通过 `camera.read_latest()` 加入 observation。

## send_action()

```python
positions = action_to_positions(action)
safe_positions = self.safety.sanitize_positions(positions)
self.arm.fast_io(safe_positions)
return positions_to_action(safe_positions)
```

这条路径是 follower 真正执行 action 的位置：

```text
LeRobot action
  -> action_to_positions()
  -> SafetyController.sanitize_positions()
  -> zyarm_sdk.fast_io()
  -> follower 固件
```

如果你修改 action 表达、单位、安全边界或控制方式，这里是必须检查的核心位置。

## 修改检查

- 改 `get_observation()`：检查 dataset observation 字段和相机读取。
- 改 `send_action()`：检查 action 维度、单位、安全过滤和 replay。
- 改 slave filter：检查手感、延迟、`100Hz` 数据质量说明。
- 改相机管理：检查 LeRobot camera config 和 `test_robot.py`。
- 改 SDK config：同步检查 [SDK 开发](../04_SDK开发/README.md)。

## 待项目方补充

> 待项目方补充：请提供推荐 slave filter 参数范围、不同任务下的安全配置建议、follower 异常停止策略和长时间采集稳定性验证标准。
