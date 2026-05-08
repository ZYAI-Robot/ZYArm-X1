### JoyCon 六轴虚拟光标：调参指导手册（`JoyConSixDofController`）

本手册面向 `joycon_sixdof_controller.py` 的 `JoyConSixDofController`，目标是：**更稳、更跟手、更不累**，并能按你的习惯（速度/细腻度/抑抖）快速找到最佳参数。

---

### 你在调什么（核心思想）
- **输入**来自 JoyCon：原始 IMU（陀螺仪 gyro + 加速度 accel）、摇杆、按键。
- **输出**是“虚拟光标”：`pose.position`（位置）+ `pose.euler_rpy`（虚拟姿态）+ `pose.direction`（前向向量）。
- 这套控制器强调“手感”：不要求虚拟姿态/位置与真实物理一一对应，允许加入滤波、曲线、死区等优化。

> 重要更新：当前版本姿态解算 **固定使用 Mahony AHRS**（gyro+accel 融合），不再使用 `GyroTrackingJoyCon.rotation` 的姿态输出。

---

### 快速上手：推荐调参顺序（非常重要）
建议按从“最影响体验、最不容易踩坑”到“更细的优化”这个顺序调：

- **1) 摇杆死区**：先把“漂移”和“误触”解决（`stick_deadzone`）
- **2) 摇杆曲线**：决定“中心细腻/边缘快不快”（`stick_expo`）
- **3) 平面速度**：决定 XY 移动快慢（`xy_speed`）
- **4) 竖直速度**：决定 Z 升降快慢（`z_speed`）
- **5) 姿态灵敏度**：决定手腕转动映射到光标朝向的幅度（`orientation_gain`）
- **6) 姿态平滑**：决定稳不稳、跟不跟（`orientation_smoothing`）
- **7) 横滚权重**：减少“横滚带来的晃”（`roll_weight`）
- **8) 更新频率**：一般不用动（`update_hz`）

---

### 参数说明（每个参数到底影响什么）

#### 位置相关（摇杆 + R/ZR）
- **`xy_speed`（默认 0.40）**：XY 平面速度（单位/秒，单位由你业务定义）。
  - 增大：移动更快；缺点是更难精准停住。
  - 减小：更稳更细；缺点是大范围移动费劲。

- **`xy_follows_virtual_rpy`（默认 True）**：是否让摇杆位移跟随虚拟点完整姿态（roll/pitch/yaw）。
  - True：摇杆位移向量会按虚拟点姿态旋转到世界坐标；**俯仰/横滚会让前推/侧推带来 Z 分量**（更像“机体坐标移动/飞行”）。
  - False：不跟随姿态，推摇杆永远按“世界固定 X/Y（水平方向）”移动（旧行为）。

- **`z_speed`（默认 0.30）**：Z 轴速度（单位/秒），由 `R` 上升、`ZR` 下降控制。
  - 建议：让 `z_speed` 略小于或等于 `xy_speed`，手感更“像在平面上画”。

- **`stick_deadzone`（默认 0.12）**：摇杆死区（0~1）。
  - 解决：静止时慢慢漂移、手不动也在动。
  - 经验：0.08~0.18 常用；漂得厉害就加大。

- **`stick_expo`（默认 1.7）**：摇杆曲线指数（>=1）。
  - 越大：中心更细腻、边缘更快（更“游戏手柄”）。
  - 越小：更线性、响应更直接。
  - 经验：1.3~2.2 常用。

#### 姿态相关（IMU → 虚拟姿态）
- **`orientation_gain`（默认 1.0）**：物理姿态增量 → 虚拟姿态增量的缩放（越大越灵敏）。
  - 增大：转动更灵敏（小手腕动作产生更大朝向变化）。
  - 减小：更稳更慢（适合精细操作或容易抖的用户）。
  - 经验：0.6~1.6 常用。

- **`orientation_smoothing`（默认 0.18）**：姿态一阶低通“跟随系数”（0~1）。
  - **越大**：越跟手（但更容易抖、也更“黏着噪声”）。
  - **越小**：越稳（但会有延迟/拖影）。
  - 经验：0.12~0.35 常用。

- **`roll_weight`（默认 0.35）**：roll 对虚拟姿态的权重。
  - roll 很容易因为握持倾斜而产生“晃”，所以默认降权。
  - 如果你真的需要 roll 强参与（比如工具绕前向轴控制），再调大（0.6~1.0）。

- **`orientation_deadband_rad`（默认 0.0）**：姿态增量死区（单位：rad）。
  - 作用：过滤非常小的抖动/噪声，让“手不动”时更稳。
  - 建议：先从 `0.002~0.01` 试起；如果感觉“起步不灵敏”就减小。

- **`stationary_smoothing_boost`（默认 0.10）**：静止时额外增加的 smoothing（0~1）。
  - 作用：当 JoyCon 被判定“静止”时，让虚拟姿态更稳（动态时不额外加延迟）。
  - 建议：0.05~0.20 常用；觉得静止仍抖就增大，觉得停下来“黏”就减小。

#### Mahony 解算参数（影响“物理姿态源”的稳定性）
Mahony 会把 gyro 积分得到的姿态用 accel（重力方向）做反馈矫正；在 JoyCon “几乎不移动”的场景里通常比直接用库内 rotation 更稳。

- **`mahony_kp`（默认 1.6）**：比例反馈强度（越大越“拉回重力方向”，越稳但可能更“硬”）。
  - 经验：1.0~3.0 常用。
- **`mahony_ki`（默认 0.0）**：积分项（用于估计陀螺零偏）。建议先保持 0，追求更稳再小幅开启。
  - 经验：0.0~0.2（太大容易慢性偏置/发散）。
- **`mahony_accel_reject_g`（默认 0.25）**：当加速度模长偏离 1g 太多时，降低 accel 参与（抗“抖动/甩动”的误修正）。
  - 增大：更信任 accel（静止更稳，但运动时更容易被误修正）。
  - 减小：更保守（运动更稳，但纯静止时收敛慢一些）。
- **`mahony_stationary_gyro_rad_s`（默认 0.05）**：静止判定的 gyro 阈值（rad/s）。
- **`mahony_stationary_accel_tol_g`（默认 0.08）**：静止判定的 accel 模长容忍度（g）。
- **`mahony_yaw_hold_when_stationary`（默认 True）**：静止时压住 yaw 漂移（无磁力计时 yaw 不可观测，这是“手感策略”）。
  - 如果你希望静止时仍能“缓慢转 yaw”，可设为 False（但更容易漂移）。

#### 其他
- **`update_hz`（默认 100）**：控制器循环更新频率。
  - 一般 60~150 都可以。越高越顺，但 CPU/USB/蓝牙压力更大。

- **`home_calib_seconds`（默认 2.0）**：按 HOME 时校准时长。
  - 越长：更稳（但你按一下要等更久）。

- **`home_resets_position`（默认 False）**：按 HOME 是否同时把位置归零。
  - 你要“回到初始点”就开 True；只想“回正方向”就保持 False。

- **`use_right_stick`（默认随 device）**：用右摇杆还是左摇杆当 XY 输入。

---

### 一套“常用手感 preset”（直接复制）

#### 1) 更稳、更不抖（适合精准抓取/对准）
- `stick_deadzone=0.16`
- `stick_expo=2.0`
- `xy_speed=0.28`
- `z_speed=0.22`
- `orientation_gain=0.85`
- `orientation_smoothing=0.14`
- `roll_weight=0.25`
- `orientation_deadband_rad=0.006`
- `stationary_smoothing_boost=0.14`
- `mahony_kp=1.8`
- `mahony_ki=0.0`

#### 2) 更跟手、更快（适合大范围移动/熟练用户）
- `stick_deadzone=0.10`
- `stick_expo=1.6`
- `xy_speed=0.55`
- `z_speed=0.45`
- `orientation_gain=1.25`
- `orientation_smoothing=0.28`
- `roll_weight=0.40`
- `orientation_deadband_rad=0.002`
- `stationary_smoothing_boost=0.06`
- `mahony_kp=1.3`
- `mahony_ki=0.0`

#### 3) “游戏手柄”风格（中心很细，推到底很快）
- `stick_deadzone=0.12`
- `stick_expo=2.2`
- `xy_speed=0.45`
- `z_speed=0.35`
- `orientation_gain=1.05`
- `orientation_smoothing=0.20`
- `roll_weight=0.35`
- `orientation_deadband_rad=0.004`
- `stationary_smoothing_boost=0.10`
- `mahony_kp=1.6`
- `mahony_ki=0.0`

---

### 常见问题 → 该调哪个参数

- **手不动也在移动（漂移）**
  - 优先：增大 `stick_deadzone`
  - 次选：重新插拔/重连手柄，让 `_stick_center` 重新采样

- **一推摇杆就太快，难以微调**
  - 降低 `xy_speed`
  - 或增大 `stick_expo`（中心更慢）

- **中心很慢但推到底也不够快**
  - 增大 `xy_speed`
  - 保持较大的 `stick_expo`

- **方向抖、手腕小抖动被放大**
  - 降低 `orientation_gain`
  - 或降低 `orientation_smoothing`
  - 或降低 `roll_weight`（特别是你觉得“侧倾会乱飞”）
  - 或增大 `orientation_deadband_rad`（先从 0.004 起）
  - 或增大 `stationary_smoothing_boost`（如果主要是“静止时抖”）
  - 或略增大 `mahony_kp`（提升“回正”力度）

- **方向很稳但跟手太慢、像有延迟**
  - 增大 `orientation_smoothing`
  - 或增大 `orientation_gain`（但注意抖动）
  - 或减小 `orientation_deadband_rad`（死区过大也会像“迟钝”）
  - 或减小 `mahony_kp`（过大可能让动态感觉更“硬”）

- **不想 roll 参与（只要 yaw/pitch 指向）**
  - 把 `roll_weight` 调到 0~0.15

- **静止时 yaw 会慢慢漂（没有磁力计时的典型现象）**
  - 保持 `mahony_yaw_hold_when_stationary=True`（默认）
  - 并适当调大 `stationary_smoothing_boost` 或 `orientation_deadband_rad`

---

### 建议的“量化”调参方法（最省时间）
建议你用同一套动作测试每次改动：
- **摇杆**：轻推 10%、中推 50%、推到底 100%，观察速度和停稳难度
- **姿态**：小手腕转动 ±10°、±30°，观察光标朝向变化幅度与抖动
- **锁死/解锁**：锁死时乱晃不应改变；解锁时应从解锁瞬间开始“相对增量”变化
- **HOME**：按下后是否能明显“回正”且更稳

---

### 代码里怎么改（推荐方式）
运行中动态调参（不用重启）：

```python
ctl.set_sensitivity(
    stick_deadzone=0.14,
    stick_expo=2.0,
    xy_speed=0.35,
    z_speed=0.25,
    orientation_gain=0.9,
    orientation_smoothing=0.16,
    roll_weight=0.25,
    orientation_deadband_rad=0.004,
    stationary_smoothing_boost=0.12,
    mahony_kp=1.6,
    mahony_ki=0.0,
)
```


