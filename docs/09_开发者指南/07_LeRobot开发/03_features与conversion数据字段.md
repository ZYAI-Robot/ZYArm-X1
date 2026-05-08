# features 与 conversion 数据字段

`features.py` 定义 LeRobot 能看到哪些 action 和 observation 字段，`conversion.py` 负责在 LeRobot 字典和 ZYArm SDK public positions 之间转换。

这两个文件决定 dataset、replay、训练和 policy eval 的字段兼容性。修改这里要格外谨慎。

## 字段定义

关键代码位于 `features.py`：

```python
JOINT_NAMES = tuple(f"joint{index}" for index in range(7))
ACTION_KEYS = tuple(f"{name}.pos" for name in JOINT_NAMES)
```

当前稳定字段为：

```text
joint0.pos
joint1.pos
joint2.pos
joint3.pos
joint4.pos
joint5.pos
joint6.pos
```

含义：

| 字段 | 含义 |
| --- | --- |
| `joint0.pos` 到 `joint5.pos` | 6 个关节的 SDK/ROS 公共角度表达，单位为弧度 |
| `joint6.pos` | 夹爪归一化位置，范围为 `0.0..1.0` |

## action 和 observation features

```python
def joint_features() -> Dict[str, type]:
    return {key: float for key in ACTION_KEYS}
```

`action_features` 和没有相机时的基础 `observation_features` 都使用这 7 个字段。带相机时，`observation_features()` 会额外把 camera 加进去：

```python
features[str(name)] = (int(height), int(width), 3)
```

这里的 `(height, width, 3)` 表示 RGB 图像 shape。

## LeRobot action 转 SDK positions

关键代码位于 `conversion.py`：

```python
def action_to_positions(action):
    missing = [key for key in ACTION_KEYS if key not in action]
    if missing:
        raise ValueError(f"Missing zyarm action keys: {missing}")
    return normalize_public_positions(_to_float(action[key]) for key in ACTION_KEYS)
```

含义：

- LeRobot action 必须包含全部 `ACTION_KEYS`。
- 顺序固定为 `joint0.pos` 到 `joint6.pos`。
- 输入会被转成 float。
- 最后统一经过 `normalize_public_positions()`。

## SDK positions 转 LeRobot action

```python
def positions_to_action(positions):
    values = normalize_public_positions(positions)
    return {key: float(value) for key, value in zip(ACTION_KEYS, values)}
```

这个函数用于：

- follower observation 中的关节状态。
- leader action 输出。
- follower `send_action()` 返回实际发送的 action。

## 夹爪归一化

```python
GRIPPER_INDEX = 6
values[GRIPPER_INDEX] = min(1.0, max(0.0, values[GRIPPER_INDEX]))
```

`joint6.pos` 会被限制到 `0.0..1.0`。如果你改变夹爪表达，例如改成角度、毫米或固件原始值，需要同步检查：

- `conversion.py`
- `features.py`
- `test_conversion.py`
- replay 和 policy 输出维度
- [科研与数据采集](../../07_科研与数据采集/README.md) 中的数据字段说明

## 修改字段的风险

| 修改 | 风险 |
| --- | --- |
| 改 `ACTION_KEYS` 名称 | 旧 dataset、replay、policy 可能失效 |
| 改字段数量 | 策略输出维度和训练脚本需要同步 |
| 改关节单位 | SDK、ROS、LeRobot 数据解释会不一致 |
| 改夹爪范围 | replay 和 policy eval 的夹爪动作可能异常 |
| 增加 observation 字段 | dataset schema、训练读取逻辑和测试需要同步 |

## 测试入口

相关测试：

- `software/lerobot_robot_zyarm/tests/test_conversion.py`
- `software/lerobot_robot_zyarm/tests/test_robot.py`
- `software/lerobot_robot_zyarm/tests/test_teleoperator.py`

## 待项目方补充

> 待项目方补充：请确认 LeRobot dataset 字段长期兼容策略、字段命名是否需要跟随后续产品命名调整，以及夹爪字段是否保持 `0.0..1.0` 归一化表达。
