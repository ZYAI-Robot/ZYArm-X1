# 数据集检查与 replay 回放

采集完成后，不要马上进入训练或策略评估。先检查 dataset 是否能被读取、相机是否正确、动作是否能安全回放。

`lerobot-replay` 会读取 dataset 中保存的 action，并发送给 follower。它会真实驱动机械臂，首次回放必须空载、低风险，并确认可以快速断电。

## 数据集检查清单

采集后先确认：

- episode 数量符合预期。
- 每个 episode 都有明确任务描述。
- 如果使用相机，`front` 和 `wrist` 画面没有反。
- 画面能看到任务关键区域。
- 采集过程中没有长时间卡顿或明显丢帧。
- leader/follower 没有接反。
- follower 没有碰撞、卡住或接近限位。

如果这些检查没有通过，建议重新采集，不要把问题数据继续用于训练。

## replay 命令

```bash
lerobot-replay \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --dataset.repo_id=<user>/zyarm_demo \
  --dataset.episode=0
```

参数含义：

| 参数 | 含义 |
| --- | --- |
| `--robot.type=zyarm_follower` | 使用 ZYArm follower 执行动作 |
| `--robot.port` | follower 串口 |
| `--dataset.repo_id` | 要回放的数据集 |
| `--dataset.episode` | 要回放的 episode 编号 |

## 回放前检查

- follower 端口确认无误。
- follower 周围没有障碍物。
- follower 保持空载。
- 不要把手放在机械臂或夹爪附近。
- 可以快速切断 follower 电源。
- 不回放来源不明或任务内容不清楚的数据集。

## 成功现象

- replay 命令能读取指定 dataset 和 episode。
- follower 能按 dataset 中的 action 运动。
- 动作大致复现采集时的轨迹。
- 回放过程中没有明显卡顿、跳变或碰撞风险。

## 常见问题

| 现象 | 可能原因 | 处理方向 |
| --- | --- | --- |
| 找不到 dataset | `repo_id` 写错，或数据集没有生成成功 | 回到 record 输出确认数据集名称 |
| replay 到错误机械臂 | follower 端口写错 | 立即停止并重新确认端口 |
| 动作和采集时明显不一致 | 采集时端口接反、数据质量差或初始状态差异大 | 重新做短采集和短回放 |
| 回放动作危险 | 数据本身有危险动作或现场摆放不同 | 直接断电，停止使用该数据 |

## 待项目方补充

> 待项目方补充：请提供标准 dataset 目录示例、episode 检查截图、replay 正常演示视频、失败数据样例和数据质量评分标准。
