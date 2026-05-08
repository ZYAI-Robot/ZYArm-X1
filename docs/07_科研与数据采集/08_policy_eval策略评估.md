# policy eval 策略评估

`policy eval` 是让训练好的策略模型控制 follower，并记录评估过程。它比人工遥操和 replay 风险更高，因为动作由模型输出，可能出现不可预测运动。

本页只说明如何使用已有策略进行评估，不讲如何训练策略模型。训练流程、模型选择和算法细节需要根据具体 LeRobot 实验方案补充。

## 运行前提

- 已经完成 [record 数据采集](06_record数据采集.md) 和 [replay 回放](07_数据集检查与replay回放.md)。
- follower 可以稳定执行动作。
- 相机画面和训练/评估期望一致。
- 已有可用的 `policy.path`。
- 首次评估保持空载、低风险。
- 可以快速切断 follower 电源。

## 评估命令

LeRobot 使用 `lerobot-record` 运行策略评估。和人工采集相比，区别是把 teleop 输入换成 policy：

```bash
lerobot-record \
  --robot.type=zyarm_follower \
  --robot.port=/dev/ttyUSB1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=<user>/zyarm_eval \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=60 \
  --dataset.single_task="Evaluate the trained policy" \
  --policy.path=<user>/zyarm_policy \
  --display_data=true
```

参数含义：

| 参数 | 含义 |
| --- | --- |
| `--policy.path` | 策略模型路径或标识 |
| `--dataset.repo_id` | 保存评估数据的新数据集名称 |
| `--dataset.single_task` | 本次评估任务描述 |
| `--display_data=true` | 显示评估过程，便于观察异常 |

## 首次评估建议

- 先使用短 episode，例如 5 到 10 秒。
- 只放置低风险道具。
- 保持手远离机械臂和夹爪。
- 观察 follower 是否出现突然大幅运动。
- 出现异常立即停止程序或直接断电。

## 成功现象

- 策略能被 LeRobot 加载。
- follower 能按策略输出动作。
- 相机画面和 observation 正常进入评估数据集。
- 每个评估 episode 能正常结束。
- 评估过程没有碰撞、卡住或接近限位。

## 不建议继续的情况

- policy 来源不明确。
- 训练数据视角和当前相机视角不一致。
- replay 都无法稳定通过。
- 评估开始后出现高频抖动、突然大幅动作或持续逼近限位。
- 无法快速断电。

## 待项目方补充

> 待项目方补充：请提供官方策略评估样例、推荐 policy 路径格式、评估任务模板、评估安全边界图、成功/失败评估视频和结果记录模板。
