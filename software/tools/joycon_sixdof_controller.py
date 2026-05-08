"""
六轴 JoyCon 控制器（虚拟空间光标）

设计目标：
- 维护“虚拟”的空间方向(姿态) + 位置，不必与真实物理姿态一一对应，便于做手感优化
- 姿态输入：JoyCon 原始 IMU（gyro+accel），使用 Mahony AHRS 解算出稳定姿态
- 位置输入：摇杆控制 XY 平面；R 上升；ZR 下降
- HOME：重新校准陀螺仪并复位姿态（可选是否复位位置）
- 摇杆按下：锁死/解锁。锁死期间不更新方向与位置；解锁时以“解锁那一刻”为基准做增量更新
- 其他按键：支持注册“按下”回调

依赖：
- pyjoycon
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from pyjoycon import GyroTrackingJoyCon, get_L_id, get_R_id


Vec3 = Tuple[float, float, float]
Callback = Callable[[], None]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _wrap_pi(a: float) -> float:
    """把角度 wrap 到 [-pi, pi]，避免 yaw 过零导致突跳。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _apply_deadzone_and_expo(x: float, deadzone: float, expo: float) -> float:
    """
    - deadzone: [0,1)
    - expo: >= 1（越大越“细腻”，中心更慢，边缘更快）
    """
    ax = abs(x)
    if ax <= deadzone:
        return 0.0
    t = (ax - deadzone) / (1.0 - deadzone)
    t = _clamp(t, 0.0, 1.0)
    t = t ** expo
    return math.copysign(t, x)


# ---------------------------
# Mahony AHRS（gyro + accel）
# ---------------------------
Quat = Tuple[float, float, float, float]  # (w, x, y, z)


def _quat_norm(q: Quat) -> Quat:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (w * inv, x * inv, y * inv, z * inv)


def _quat_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q: Quat) -> Quat:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_rotate_world_to_body(q_body_to_world: Quat, v_world: Vec3) -> Vec3:
    # v_body = q^{-1} ⊗ v ⊗ q = conj(q) ⊗ v ⊗ q（q 为单位四元数）
    qc = _quat_conj(q_body_to_world)
    vq: Quat = (0.0, float(v_world[0]), float(v_world[1]), float(v_world[2]))
    r = _quat_mul(_quat_mul(qc, vq), q_body_to_world)
    return (r[1], r[2], r[3])


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    # ZYX: q = qz(yaw) * qy(pitch) * qx(roll)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = cy * sp * cr + sy * cp * sr
    z = sy * cp * cr - cy * sp * sr
    return _quat_norm((w, x, y, z))


def _quat_to_rpy(q_body_to_world: Quat) -> Vec3:
    # ZYX
    w, x, y, z = q_body_to_world
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = _clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


class MahonyAHRS:
    """
    Mahony AHRS（互补滤波）：gyro 积分 + accel(重力方向) 反馈修正。

    约定：
    - 输入 gyro 单位 rad/s
    - 输入 accel 单位 g（归一化后只用方向）
    - 世界系 up = +Z（与本项目虚拟点 FLU：+X前 +Y左 +Z上 一致）
    - q 表示 body -> world 的旋转
    """

    def __init__(
        self,
        *,
        kp: float = 1.6,
        ki: float = 0.0,
        accel_reject_g: float = 0.25,
        stationary_gyro_rad_s: float = 0.05,
        stationary_accel_tol_g: float = 0.08,
        yaw_hold_when_stationary: bool = True,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.accel_reject_g = float(accel_reject_g)
        self.stationary_gyro_rad_s = float(stationary_gyro_rad_s)
        self.stationary_accel_tol_g = float(stationary_accel_tol_g)
        self.yaw_hold_when_stationary = bool(yaw_hold_when_stationary)

        self.q: Quat = (1.0, 0.0, 0.0, 0.0)
        self._ei: Vec3 = (0.0, 0.0, 0.0)  # integral error
        self.is_stationary: bool = False
        self._inited: bool = False

        # 运行时校准量（用于应对“不是严格物理单位/标定不准”的情况）
        # - gyro_bias: 静止时 gyro 的均值偏置（rad/s）
        # - accel_mag_ref: 静止时 |accel| 的基线（理想应接近 1g，但实际可能偏离）
        self.gyro_bias: Vec3 = (0.0, 0.0, 0.0)
        self.accel_mag_ref: float = 1.0

    def reset(self):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self._ei = (0.0, 0.0, 0.0)
        self.is_stationary = False
        self._inited = False

    def set_calibration(self, *, gyro_bias: Optional[Vec3] = None, accel_mag_ref: Optional[float] = None):
        if gyro_bias is not None:
            self.gyro_bias = (float(gyro_bias[0]), float(gyro_bias[1]), float(gyro_bias[2]))
        if accel_mag_ref is not None:
            am = float(accel_mag_ref)
            if am > 1e-6:
                self.accel_mag_ref = am

    def reset_from_accel(self, ax_g: float, ay_g: float, az_g: float):
        # 用重力方向初始化 roll/pitch（yaw 置 0）
        ax, ay, az = float(ax_g), float(ay_g), float(az_g)
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if n <= 1e-9:
            self.reset()
            self._inited = True
            return
        ax, ay, az = ax / n, ay / n, az / n
        # accel 指向 up（静止时约为 +Z），用它反推 roll/pitch
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        self.q = _rpy_to_quat(roll, pitch, 0.0)
        self._ei = (0.0, 0.0, 0.0)
        self.is_stationary = False
        self._inited = True

    def update(self, gx: float, gy: float, gz: float, ax_g: float, ay_g: float, az_g: float, dt: float):
        dt = _clamp(float(dt), 1e-4, 0.05)
        gx, gy, gz = float(gx), float(gy), float(gz)
        ax, ay, az = float(ax_g), float(ay_g), float(az_g)

        # 外部校准：去掉静止零偏
        bx, by, bz = self.gyro_bias
        gx -= bx
        gy -= by
        gz -= bz

        # 归一化 accel（只用方向）
        an = math.sqrt(ax * ax + ay * ay + az * az)
        use_accel = True
        if an <= 1e-9:
            use_accel = False
        else:
            # 非 1g（剧烈运动/抖动）时降低信任
            if abs(an - float(self.accel_mag_ref)) > self.accel_reject_g:
                use_accel = False
            ax, ay, az = ax / an, ay / an, az / an

        # 静止判定（用于抑制漂移/抖动）
        gnorm = math.sqrt(gx * gx + gy * gy + gz * gz)
        self.is_stationary = bool(
            (gnorm <= self.stationary_gyro_rad_s)
            and (an > 1e-9)
            and (abs(an - float(self.accel_mag_ref)) <= self.stationary_accel_tol_g)
        )

        if not self._inited:
            self.reset_from_accel(ax_g, ay_g, az_g)

        # 误差：让估计的 up 与测量的 accel(up) 对齐
        ex = ey = ez = 0.0
        if use_accel:
            ux, uy, uz = _quat_rotate_world_to_body(self.q, (0.0, 0.0, 1.0))
            # e = a × u_est
            ex = ay * uz - az * uy
            ey = az * ux - ax * uz
            ez = ax * uy - ay * ux

        # 积分项（bias 估计）：仅在“可信 & 静止”时积分更稳
        ix, iy, iz = self._ei
        if self.ki > 0.0 and use_accel and self.is_stationary:
            ix += self.ki * ex * dt
            iy += self.ki * ey * dt
            iz += self.ki * ez * dt
            self._ei = (ix, iy, iz)
        elif self.ki <= 0.0:
            self._ei = (0.0, 0.0, 0.0)

        # 反馈修正
        gx += self.kp * ex + ix
        gy += self.kp * ey + iy
        gz += self.kp * ez + iz

        # 静止时抑制 yaw 漂移（无磁力计时 yaw 不可观测，只能做“手感策略”）
        if self.is_stationary and self.yaw_hold_when_stationary:
            gz = 0.0

        # 四元数积分：q_dot = 0.5 * q ⊗ [0, w]
        q0, q1, q2, q3 = self.q
        dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        q0 += dq0 * dt
        q1 += dq1 * dt
        q2 += dq2 * dt
        q3 += dq3 * dt
        self.q = _quat_norm((q0, q1, q2, q3))


def _euler_to_direction(roll: float, pitch: float, yaw: float) -> Vec3:
    """
    由欧拉角得到“前向方向”单位向量。
    约定：
    - 虚拟点坐标系采用 FLU：+X 朝前、+Y 朝左、+Z 朝上
    - yaw/pitch/roll 的正方向遵循右手系，按 R = Rz(yaw) * Ry(pitch) * Rx(roll) 的直观组合
    """
    # 先 yaw(绕Z) 再 pitch(绕Y) 再 roll(绕X) 的一个简单近似组合（不追求严格航姿学一致）
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    # 基于“前向 +X”推一个直观的方向向量（主要由 yaw/pitch 决定；roll 轻量影响）
    x = cp * cy
    y = cp * sy
    z = sp

    # roll 对方向本不影响（只影响“右/上”），但为了手感可让它微弱影响 z（可调）。
    z = _clamp(z + 0.10 * sr, -1.0, 1.0)

    n = math.sqrt(x * x + y * y + z * z)
    if n <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (x / n, y / n, z / n)


@dataclass
class CursorPose:
    position: Vec3
    euler_rpy: Vec3  # (roll, pitch, yaw) in rad, 虚拟姿态
    direction: Vec3  # unit vec
    locked: bool


class JoyConSixDofController:
    """
    六轴控制器：
    - start()/stop()：后台线程更新
    - get_pose()：获取光标的虚拟方向与位置
    - on(button, fn)：注册按键“按下(press)”回调
    - on_short(button, fn)：注册按键“短按(release < long_press_seconds)”回调
    - on_long(button, fn)：注册按键“长按(hold >= long_press_seconds)”回调（达到阈值即触发一次）
    """

    # 右 JoyCon 推荐使用右摇杆；左 JoyCon 用左摇杆
    _BUTTON_MAP_RIGHT = {
        "a": ("buttons", "right", "a"),
        "b": ("buttons", "right", "b"),
        "x": ("buttons", "right", "x"),
        "y": ("buttons", "right", "y"),
        "plus": ("buttons", "shared", "plus"),
        "r": ("buttons", "right", "r"),
        "zr": ("buttons", "right", "zr"),
        "home": ("buttons", "shared", "home"),
        "stick_btn": ("buttons", "shared", "r-stick"),
        "sr": ("buttons", "right", "sr"),
        "sl": ("buttons", "right", "sl"),
        "minus": ("buttons", "shared", "minus"),
        "capture": ("buttons", "shared", "capture"),
    }

    _BUTTON_MAP_LEFT = {
        "up": ("buttons", "left", "up"),
        "down": ("buttons", "left", "down"),
        "left": ("buttons", "left", "left"),
        "right": ("buttons", "left", "right"),
        "minus": ("buttons", "shared", "minus"),
        "l": ("buttons", "left", "l"),
        "zl": ("buttons", "left", "zl"),
        "capture": ("buttons", "shared", "capture"),
        "home": ("buttons", "shared", "home"),
        "stick_btn": ("buttons", "shared", "l-stick"),
        "sr": ("buttons", "left", "sr"),
        "sl": ("buttons", "left", "sl"),
        "plus": ("buttons", "shared", "plus"),
        "y": ("buttons", "left", "left"),
        "a": ("buttons", "left", "right"),
        "b": ("buttons", "left", "down"),
    }

    def __init__(
        self,
        device: str = "right",
        *,
        update_hz: float = 100.0,
        # 启动/校准流程
        require_home_calibration: bool = True,
        verbose: bool = True,
        # 姿态（虚拟）手感参数
        orientation_gain: float = 1,          # 物理增量 -> 虚拟增量的缩放
        orientation_smoothing: float = 0.3,    # [0,1] 越大越"跟手"，越小越"稳"
        roll_weight: float = 1,              # roll 对虚拟姿态的权重（可用于减少"横滚晃动"）
        orientation_deadband_rad: float = 0.0,  # 小抖动死区（rad），0 表示关闭
        stationary_smoothing_boost: float = 0.10,  # 静止时额外增加的 smoothing（提升稳态手感）
        # Mahony 参数
        mahony_kp: float = 1.6,
        mahony_ki: float = 0.0,
        mahony_accel_reject_g: float = 0.25,
        mahony_stationary_gyro_rad_s: float = 0.05,
        mahony_stationary_accel_tol_g: float = 0.08,
        mahony_yaw_hold_when_stationary: bool = True,
        # 位置（虚拟）手感参数
        xy_speed: float = 120,                 # 单位/秒（你的业务单位）
        z_speed: float = 60.0,                  # 单位/秒
        stick_deadzone: float = 0.12,           # [0,1)
        stick_expo: float = 1.7,                # >=1
        # XY 位移坐标系：把摇杆的 (前/左) 位移按虚拟点完整姿态(roll/pitch/yaw) 旋转到世界坐标
        # - False: 不跟随姿态（更像"在地面/世界坐标里走"）
        # - True : 跟随 r/p/y（更像"在机体坐标里飞"-—前推在俯仰时会带来 Z 分量）
        xy_follows_virtual_rpy: bool = False,
        # HOME 行为
        home_calib_seconds: float = 2.0,
        home_resets_position: bool = False,
        # 右摇杆/左摇杆选择：默认跟随 device
        use_right_stick: Optional[bool] = None,
        # 长按判定阈值（秒）
        long_press_seconds: float = 1.0,
        # 按键"松开"去抖（秒）：
        # JoyCon 蓝牙状态在按住时偶发抖成 0（短暂掉包/报告空洞）会导致长按计时被误判为"已松开并重置"，
        # 体感就会变成"要按更久才触发"。这里仅对 release 做去抖，press 仍立即生效。
        button_release_debounce_seconds: float = 0.04,
        # 位置范围限制
        position_limits: Optional[Tuple[float, float, float, float, float, float]] = (0, 350, -9999, 9999, -9999, 9999),  # (min_x, max_x, min_y, max_y, min_z, max_z)
        # XY平面旋转中心点及距离限制
        xy_pivot: Tuple[float, float] = (-113.2, 0.0),  # 旋转中心点 (x, y)
        xy_radius_limits: Tuple[float, float] = (50, 350.0),  # 距离范围 (min_r, max_r)
    ):
        if device not in ("right", "left"):
            raise ValueError('device 只支持 "right" 或 "left"')
        if update_hz <= 0:
            raise ValueError("update_hz 必须 > 0")
        if not (0.0 <= orientation_smoothing <= 1.0):
            raise ValueError("orientation_smoothing 必须在 [0,1]")
        if orientation_deadband_rad < 0.0:
            raise ValueError("orientation_deadband_rad 必须 >= 0")
        if not (0.0 <= stationary_smoothing_boost <= 1.0):
            raise ValueError("stationary_smoothing_boost 必须在 [0,1]")
        if not (0.0 <= stick_deadzone < 1.0):
            raise ValueError("stick_deadzone 必须在 [0,1)")
        if stick_expo < 1.0:
            raise ValueError("stick_expo 必须 >= 1")

        self.device = device
        self.update_hz = float(update_hz)

        self.require_home_calibration = bool(require_home_calibration)
        self.verbose = bool(verbose)

        self.orientation_gain = float(orientation_gain)
        self.orientation_smoothing = float(orientation_smoothing)
        self.roll_weight = float(roll_weight)
        self.orientation_deadband_rad = float(orientation_deadband_rad)
        self.stationary_smoothing_boost = float(stationary_smoothing_boost)

        self._mahony = MahonyAHRS(
            kp=float(mahony_kp),
            ki=float(mahony_ki),
            accel_reject_g=float(mahony_accel_reject_g),
            stationary_gyro_rad_s=float(mahony_stationary_gyro_rad_s),
            stationary_accel_tol_g=float(mahony_stationary_accel_tol_g),
            yaw_hold_when_stationary=bool(mahony_yaw_hold_when_stationary),
        )

        self.xy_speed = float(xy_speed)
        self.z_speed = float(z_speed)
        self.stick_deadzone = float(stick_deadzone)
        self.stick_expo = float(stick_expo)
        self.xy_follows_virtual_rpy = bool(xy_follows_virtual_rpy)

        # 位置范围限制
        if position_limits is not None:
            min_x, max_x, min_y, max_y, min_z, max_z = position_limits
            if min_x > max_x or min_y > max_y or min_z > max_z:
                raise ValueError("位置范围参数错误：最小值不能大于最大值")
            self.position_limits = position_limits
        else:
            self.position_limits = None

        # XY平面旋转中心点及距离限制
        self.xy_pivot = (float(xy_pivot[0]), float(xy_pivot[1]))
        min_r, max_r = xy_radius_limits
        if min_r < 0 or max_r < 0 or min_r > max_r:
            raise ValueError("xy_radius_limits 参数错误：min_r 和 max_r 必须 >= 0 且 min_r <= max_r")
        self.xy_radius_limits = (float(min_r), float(max_r))

        self.home_calib_seconds = float(home_calib_seconds)
        self.home_resets_position = bool(home_resets_position)

        if use_right_stick is None:
            use_right_stick = (device == "right")
        self.use_right_stick = bool(use_right_stick)

        lps = float(long_press_seconds)
        if lps <= 0.0:
            raise ValueError("long_press_seconds 必须 > 0")
        self.long_press_seconds = lps

        brds = float(button_release_debounce_seconds)
        if brds < 0.0:
            raise ValueError("button_release_debounce_seconds 必须 >= 0")
        self.button_release_debounce_seconds = brds

        joycon_id = get_R_id() if device == "right" else get_L_id()
        # get_*_id() 在未连接/未配对时可能返回 (None, None, None)
        if joycon_id is None or any(v is None for v in joycon_id):
            raise RuntimeError(
                f"未检测到 {device} JoyCon（get_{'R' if device=='right' else 'L'}_id() 返回空）。"
                "请确认：已蓝牙配对/驱动可用/未被其他程序占用。"
            )
        self._joycon = GyroTrackingJoyCon(*joycon_id)

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # 虚拟状态
        self._pos: List[float] = [0.0, 0.0, 0.0]
        self._rpy: List[float] = [0.0, 0.0, 0.0]
        # 启动默认锁住：避免未校准时姿态漂移影响虚拟点
        self._locked = True if self.require_home_calibration else False

        # 校准状态机：HOME 单击后自动采样一段时间 -> 估计 gyro_bias / accel_mag_ref
        self._calibrated: bool = not self.require_home_calibration
        self._calib_active: bool = False
        self._calib_t0: float = 0.0
        self._calib_t_end: float = 0.0
        self._calib_last_hint_t: float = 0.0
        self._calib_gyro_sum: List[float] = [0.0, 0.0, 0.0]
        self._calib_acc_mag_sum: float = 0.0
        self._calib_n: int = 0

        # 断链保护：一旦检测到 JoyCon 断链/读取失败，就锁住虚拟点（位置+姿态都不再更新）
        self._link_ok: bool = True
        self._last_link_err_t: float = 0.0

        # 最近一次物理姿态（Mahony 解算的欧拉角），用于按键解锁瞬间取基准
        self._last_phys_rpy: Vec3 = (0.0, 0.0, 0.0)
        self._have_last_phys: bool = False

        # 解锁基准：用于“增量控制”
        self._unlock_phys_rpy_ref: Optional[Vec3] = None
        self._unlock_virtual_rpy_ref: Optional[Vec3] = None

        # 摇杆中心校准（简单：首次读取时记录）
        self._stick_center: Optional[Tuple[float, float]] = None

        # 按键回调（按下触发）
        self._callbacks: Dict[str, List[Callback]] = {}
        # 按键回调（短按/长按）
        self._short_callbacks: Dict[str, List[Callback]] = {}
        self._long_callbacks: Dict[str, List[Callback]] = {}
        # 稳定态按键（经过 release 去抖后的状态）
        self._stable_buttons: Dict[str, int] = {}
        # release 去抖：raw 变 0 时先进入 pending，持续足够久才确认“松开”
        self._release_pending_t0: Dict[str, float] = {}
        # 长按判定：记录按下时刻、长按是否已触发
        self._press_t0: Dict[str, float] = {}
        self._long_fired: Dict[str, bool] = {}

        # 默认体验：B 键
        # - 长按：复位方向+位置
        #   并自动锁定（需要再次按下摇杆解锁）
        # 若当前 device 不含 b（例如 left），则不注册
        if "b" in self._get_button_map():
            def _b_long():
                self.reset_virtual_pose(reset_position=True, reset_orientation=True)
                self._force_lock()
                if self.verbose:
                    print("[JoyCon] B 长按：已复位虚拟点方向+位置，并自动锁定。请按摇杆键解锁继续。")

            self.on_long("b", _b_long)

    # ---------------------------
    # 对外接口
    # ---------------------------
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        if self.verbose and self.require_home_calibration and not self._calibrated:
            print(
                "[JoyCon] 已连接：默认锁定。请将手柄静止放置，单击 HOME 开始校准（约 "
                f"{self.home_calib_seconds:.1f}s）；完成后按摇杆解锁运行。"
            )
        self._stop_evt.clear()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 1.0):
        self._running = False
        self._stop_evt.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=join_timeout_s)

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def is_locked(self) -> bool:
        with self._lock:
            return bool(self._locked)

    def available_buttons(self) -> List[str]:
        """返回当前 device 支持的按钮名（可用于 on(button, fn)）。"""
        return sorted(list(self._get_button_map().keys()))

    def lock(self):
        """手动锁死（等价于按下摇杆）。"""
        with self._lock:
            if self._locked:
                return
        self._toggle_lock()

    def unlock(self):
        """手动解锁（等价于再次按下摇杆）。"""
        with self._lock:
            if not self._locked:
                return
        self._toggle_lock()

    def set_sensitivity(
        self,
        *,
        orientation_gain: Optional[float] = None,
        xy_speed: Optional[float] = None,
        z_speed: Optional[float] = None,
        stick_deadzone: Optional[float] = None,
        stick_expo: Optional[float] = None,
        xy_follows_virtual_rpy: Optional[bool] = None,
        orientation_smoothing: Optional[float] = None,
        roll_weight: Optional[float] = None,
        orientation_deadband_rad: Optional[float] = None,
        stationary_smoothing_boost: Optional[float] = None,
        mahony_kp: Optional[float] = None,
        mahony_ki: Optional[float] = None,
        mahony_accel_reject_g: Optional[float] = None,
        mahony_stationary_gyro_rad_s: Optional[float] = None,
        mahony_stationary_accel_tol_g: Optional[float] = None,
        mahony_yaw_hold_when_stationary: Optional[bool] = None,
        position_limits: Optional[Tuple[float, float, float, float, float, float]] = None,  # (min_x, max_x, min_y, max_y, min_z, max_z)
        xy_pivot: Optional[Tuple[float, float]] = None,  # 旋转中心点 (x, y)
        xy_radius_limits: Optional[Tuple[float, float]] = None,  # 距离范围 (min_r, max_r)
    ):
        with self._lock:
            if orientation_gain is not None:
                self.orientation_gain = float(orientation_gain)
            if xy_speed is not None:
                self.xy_speed = float(xy_speed)
            if z_speed is not None:
                self.z_speed = float(z_speed)
            if stick_deadzone is not None:
                dz = float(stick_deadzone)
                if not (0.0 <= dz < 1.0):
                    raise ValueError("stick_deadzone 必须在 [0,1)")
                self.stick_deadzone = dz
            if stick_expo is not None:
                ex = float(stick_expo)
                if ex < 1.0:
                    raise ValueError("stick_expo 必须 >= 1")
                self.stick_expo = ex
            if xy_follows_virtual_rpy is not None:
                self.xy_follows_virtual_rpy = bool(xy_follows_virtual_rpy)
            if orientation_smoothing is not None:
                sm = float(orientation_smoothing)
                if not (0.0 <= sm <= 1.0):
                    raise ValueError("orientation_smoothing 必须在 [0,1]")
                self.orientation_smoothing = sm
            if roll_weight is not None:
                self.roll_weight = float(roll_weight)
            if orientation_deadband_rad is not None:
                db = float(orientation_deadband_rad)
                if db < 0.0:
                    raise ValueError("orientation_deadband_rad 必须 >= 0")
                self.orientation_deadband_rad = db
            if stationary_smoothing_boost is not None:
                sb = float(stationary_smoothing_boost)
                if not (0.0 <= sb <= 1.0):
                    raise ValueError("stationary_smoothing_boost 必须在 [0,1]")
                self.stationary_smoothing_boost = sb
            if position_limits is not None:
                min_x, max_x, min_y, max_y, min_z, max_z = position_limits
                if min_x > max_x or min_y > max_y or min_z > max_z:
                    raise ValueError("位置范围参数错误：最小值不能大于最大值")
                self.position_limits = position_limits

            if xy_pivot is not None:
                self.xy_pivot = (float(xy_pivot[0]), float(xy_pivot[1]))

            if xy_radius_limits is not None:
                min_r, max_r = xy_radius_limits
                if min_r < 0 or max_r < 0 or min_r > max_r:
                    raise ValueError("xy_radius_limits 参数错误：min_r 和 max_r 必须 >= 0 且 min_r <= max_r")
                self.xy_radius_limits = (float(min_r), float(max_r))

            # Mahony 参数：允许运行中调（不重置状态；需要立即生效可手动按 HOME 或调用 reset_virtual_pose/reset）
            if mahony_kp is not None:
                self._mahony.kp = float(mahony_kp)
            if mahony_ki is not None:
                self._mahony.ki = float(mahony_ki)
            if mahony_accel_reject_g is not None:
                self._mahony.accel_reject_g = float(mahony_accel_reject_g)
            if mahony_stationary_gyro_rad_s is not None:
                self._mahony.stationary_gyro_rad_s = float(mahony_stationary_gyro_rad_s)
            if mahony_stationary_accel_tol_g is not None:
                self._mahony.stationary_accel_tol_g = float(mahony_stationary_accel_tol_g)
            if mahony_yaw_hold_when_stationary is not None:
                self._mahony.yaw_hold_when_stationary = bool(mahony_yaw_hold_when_stationary)

    def reset_virtual_pose(self, *, reset_position: bool = True, reset_orientation: bool = True):
        with self._lock:
            if reset_position:
                self._pos = [0.0, 0.0, 0.0]
            if reset_orientation:
                self._rpy = [0.0, 0.0, 0.0]
            self._unlock_phys_rpy_ref = None
            self._unlock_virtual_rpy_ref = None

    def get_pose(self) -> CursorPose:
        with self._lock:
            r, p, y = self._rpy
            pos = (float(self._pos[0]), float(self._pos[1]), float(self._pos[2]))
            direction = _euler_to_direction(r, p, y)
            return CursorPose(
                position=pos,
                euler_rpy=(float(r), float(p), float(y)),
                direction=direction,
                locked=bool(self._locked),
            )

    def on(self, button: str, fn: Callback):
        """
        注册按键“按下(press)”回调。
        button 建议使用：
        - right: a/b/x/y/plus/r/zr/home/stick_btn/sr/sl
        - left : up/down/left/right/minus/l/zl/capture/home/stick_btn/sr/sl
        """
        if not callable(fn):
            raise TypeError("fn 必须是可调用对象")
        with self._lock:
            self._callbacks.setdefault(button, []).append(fn)

    def on_short(self, button: str, fn: Callback):
        """注册按键“短按”回调：按下后在阈值内松开即触发。"""
        if not callable(fn):
            raise TypeError("fn 必须是可调用对象")
        with self._lock:
            self._short_callbacks.setdefault(button, []).append(fn)

    def on_long(self, button: str, fn: Callback):
        """注册按键“长按”回调：按住达到 long_press_seconds 即触发一次。"""
        if not callable(fn):
            raise TypeError("fn 必须是可调用对象")
        with self._lock:
            self._long_callbacks.setdefault(button, []).append(fn)

    def get_button_down(self, button: str) -> bool:
        """
        查询“当前是否按住某个按键”。

        用途：实现“按住某键持续生效”的控制（例如夹爪角度连续增减）。

        说明：
        - button 名称同 on(button, fn)（可用 available_buttons() 查看）
        - 该函数会即时读取 joycon.get_status()，可能有轻微开销；建议在固定频率循环中使用
        """
        btn_map = self._get_button_map()
        path = btn_map.get(button)
        if not path:
            return False
        st = self._safe_get_status()
        if not st:
            return False
        return bool(self._read_button(st, path))

    # ---------------------------
    # 内部：输入采集/更新
    # ---------------------------
    def _force_lock(self):
        """
        强制进入锁定状态（不做 toggle）。
        用途：断链保护/某些“复位后需要重新确认”的场景。
        """
        with self._lock:
            self._locked = True
            # 锁死时清掉基准，避免后续解锁时拿到旧基准
            self._unlock_phys_rpy_ref = None
            self._unlock_virtual_rpy_ref = None

    def _on_link_lost(self, err: Optional[Exception] = None):
        self._force_lock()
        self._link_ok = False
        now = time.perf_counter()
        # 限频打印，避免刷屏
        if self.verbose and (now - self._last_link_err_t) > 1.0:
            self._last_link_err_t = now
            msg = "[JoyCon] 连接断开/读取失败：已自动锁住虚拟点（位置+姿态保持不变）。"
            if err is not None:
                msg += f" err={err}"
            print(msg)

    def _on_link_ok(self):
        if not self._link_ok and self.verbose:
            print("[JoyCon] 连接恢复：仍保持锁定，请确认姿态稳定后按摇杆解锁。")
        self._link_ok = True

    def _safe_get_status(self) -> Optional[dict]:
        try:
            st = self._joycon.get_status()
        except Exception as e:
            self._on_link_lost(e)
            return None
        self._on_link_ok()
        return st if isinstance(st, dict) else None

    def _get_phys_rpy_mahony(self, dt: float) -> Vec3:
        """
        用 Mahony AHRS 从 JoyCon 原始 IMU（gyro+accel）解算姿态。

        数据来源：
        - self._joycon.gyro_in_rad : 每次报告包含 3 组 gyro(rad/s)
        - self._joycon.accel_in_g  : 每次报告包含 3 组 accel(g)

        坐标系适配：
        - JoyCon 体感(统一到右手柄)可视为 FRD：+X前 +Y右 +Z下
        - 本项目虚拟点为 FLU：+X前 +Y左 +Z上
        所以对 gyro/accel 都做 (y,z) 取反。
        """
        try:
            gyros = list(getattr(self._joycon, "gyro_in_rad"))
            accs = list(getattr(self._joycon, "accel_in_g"))
        except Exception:
            return self._last_phys_rpy

        n = min(len(gyros), len(accs))
        if n <= 0:
            return self._last_phys_rpy

        dt_s = _clamp(float(dt) / float(n), 1e-4, 0.05)
        for i in range(n):
            gx, gy, gz = gyros[i]
            ax, ay, az = accs[i]
            # FRD -> FLU
            gx, gy, gz = float(gx), -float(gy), -float(gz)
            ax, ay, az = float(ax), -float(ay), -float(az)
            self._mahony.update(gx, gy, gz, ax, ay, az, dt_s)

        return _quat_to_rpy(self._mahony.q)

    def _get_phys_rpy_estimate(self) -> Vec3:
        """不读传感器，直接返回当前 Mahony 内部姿态估计（用于解锁瞬间兜底）。"""
        return _quat_to_rpy(self._mahony.q)

    def _read_stick_raw(self) -> Tuple[float, float]:
        st = self._safe_get_status()
        if not st:
            return (0.0, 0.0)
        sticks = st.get("analog-sticks", {}) if isinstance(st, dict) else {}
        side = "right" if self.use_right_stick else "left"
        s = sticks.get(side, {}) if isinstance(sticks, dict) else {}
        hx = float(s.get("horizontal", 0.0))
        vy = float(s.get("vertical", 0.0))
        return (hx, vy)

    def _stick_to_unit(self, raw: Tuple[float, float]) -> Tuple[float, float]:
        """
        把 12bit 摇杆值映射到 [-1,1]，并做中心校准。
        """
        hx, vy = raw
        if self._stick_center is None:
            # 首次读到的值作为中心，能显著减少不同手柄漂移
            self._stick_center = (hx, vy)
        cx, cy = self._stick_center
        # 经验：12bit，中心约 2048，范围约 0~4095；我们用 2048 做尺度
        sx = 2048.0
        sy = 2048.0
        x = (hx - cx) / sx
        y = (vy - cy) / sy
        x = _clamp(x, -1.0, 1.0)
        y = _clamp(y, -1.0, 1.0)
        return (x, y)

    def _read_button(self, st: dict, path: Tuple[str, str, str]) -> int:
        a, b, c = path
        try:
            v = st.get(a, {}).get(b, {}).get(c, 0)
            return int(v)
        except Exception:
            return 0

    def _get_button_map(self) -> Dict[str, Tuple[str, str, str]]:
        return self._BUTTON_MAP_RIGHT if self.device == "right" else self._BUTTON_MAP_LEFT

    def _emit_press_callbacks(self, button: str):
        # HOME 和 stick_btn 我们自己会处理，但也允许用户注册回调
        cbs = self._callbacks.get(button, [])
        for fn in cbs:
            try:
                fn()
            except Exception:
                # 不让用户回调炸掉控制线程
                pass

    def _emit_short_callbacks(self, button: str):
        cbs = self._short_callbacks.get(button, [])
        for fn in cbs:
            try:
                fn()
            except Exception:
                pass

    def _emit_long_callbacks(self, button: str):
        cbs = self._long_callbacks.get(button, [])
        for fn in cbs:
            try:
                fn()
            except Exception:
                pass

    def _handle_button_edges(self, st: dict):
        now = time.perf_counter()
        long_s = float(self.long_press_seconds)
        debounce_s = float(getattr(self, "button_release_debounce_seconds", 0.0))
        # 注意：JoyCon 状态读取在蓝牙环境下可能出现“按住时偶发读到 0”的短暂抖动。
        # 若把这类抖动当作真实松开，会导致长按计时频繁重置，体感变成“要按更久才触发”。
        # 这里采用“仅对 release 去抖”的策略：press 立即确认，release 需持续为 0 达到 debounce_s 才确认。
        btn_map = self._get_button_map()
        for name, path in btn_map.items():
            raw = self._read_button(st, path)
            stable = int(self._stable_buttons.get(name, 0))

            # -----------------------------
            # 1) press: 0 -> 1 立即确认
            # -----------------------------
            if stable == 0 and raw == 1:
                self._stable_buttons[name] = 1
                self._release_pending_t0.pop(name, None)

                self._press_t0[name] = now
                self._long_fired[name] = False

                if name == "home" or name == "capture":
                    self._start_home_calibration()
                if name == "stick_btn":
                    self._toggle_lock()
                self._emit_press_callbacks(name)
                continue

            # -----------------------------------------
            # 2) hold: 1 & raw=1（或 release pending 中）
            # -----------------------------------------
            if stable == 1 and raw == 1:
                # raw 恢复为 1：取消 release pending
                self._release_pending_t0.pop(name, None)

                t0 = self._press_t0.get(name)
                if t0 is not None and (not self._long_fired.get(name, False)):
                    if (now - float(t0)) >= long_s:
                        self._long_fired[name] = True
                        self._emit_long_callbacks(name)
                continue

            # -----------------------------------------
            # 3) release: 1 -> 0 需要持续 debounce_s 才确认
            # -----------------------------------------
            if stable == 1 and raw == 0:
                if debounce_s <= 0.0:
                    # 不去抖：立即确认松开
                    self._stable_buttons[name] = 0
                    self._release_pending_t0.pop(name, None)
                else:
                    t_pending = self._release_pending_t0.get(name)
                    if t_pending is None:
                        self._release_pending_t0[name] = now
                        continue
                    if (now - float(t_pending)) < debounce_s:
                        continue
                    # 持续为 0 足够久：确认松开
                    self._stable_buttons[name] = 0
                    self._release_pending_t0.pop(name, None)

                # 确认松开后的短按判定
                t0 = self._press_t0.pop(name, None)
                long_fired = bool(self._long_fired.pop(name, False))
                if t0 is not None and (not long_fired):
                    if (now - float(t0)) < long_s:
                        self._emit_short_callbacks(name)
                continue

            # stable==0 & raw==0：无事发生
            # stable==0 & raw==1：上面已处理 press
            # stable==1 & raw==?：上面已处理 hold/release
            # 其他极端情况：忽略

    def _start_home_calibration(self):
        """
        HOME 单击触发一次校准（无需长按，避免 JoyCon 长按 HOME 关机/休眠）。
        校准期间保持锁定。
        """
        now = time.perf_counter()
        self._calib_active = True
        self._calib_t0 = now
        self._calib_t_end = now + float(self.home_calib_seconds)
        self._calib_last_hint_t = 0.0
        self._calib_gyro_sum = [0.0, 0.0, 0.0]
        self._calib_acc_mag_sum = 0.0
        self._calib_n = 0
        self._calibrated = False
        with self._lock:
            self._locked = True
        # 同时让 pyjoycon 内部也做一次校准（不依赖它的 rotation，但它会修正陀螺标定）
        try:
            self._joycon.calibrate(seconds=self.home_calib_seconds)
        except Exception:
            pass
        if self.verbose:
            print(f"[JoyCon] 开始校准：请保持静止约 {self.home_calib_seconds:.1f}s ...")

    def _toggle_lock(self):
        with self._lock:
            if (not self._calibrated) and self.require_home_calibration:
                # 未校准前禁止解锁（避免漂移影响虚拟点/机械臂）
                self._locked = True
                return
            self._locked = not self._locked
            if self._locked:
                # 锁死时清掉基准，避免解锁时拿到旧基准
                self._unlock_phys_rpy_ref = None
                self._unlock_virtual_rpy_ref = None
            else:
                # 解锁：以"当前物理姿态"为基准做增量更新
                # 获取当前物理姿态作为参考
                current_phys_rpy = self._get_phys_rpy_estimate()
                self._unlock_phys_rpy_ref = (
                    current_phys_rpy[0], 
                    current_phys_rpy[1], 
                    current_phys_rpy[2]
                )
                # 记录当前虚拟姿态作为参考，但不解锁瞬间修改虚拟姿态
                self._unlock_virtual_rpy_ref = (
                    self._rpy[0], 
                    self._rpy[1], 
                    self._rpy[2]
                )

    def _update_home_calibration(self):
        """
        HOME 单击后自动校准：
        - 采样 gyro_in_rad / accel_in_g
        - 估计 gyro_bias（静止零偏）和 accel_mag_ref（静止模长基线）
        - 写入 Mahony，以提升静止判定与抗漂移
        """
        if not self._calib_active:
            return
        now = time.perf_counter()

        # 采样：使用当前报告的 3 组样本求均值（降低噪声）
        try:
            gyros = list(getattr(self._joycon, "gyro_in_rad"))
            accs = list(getattr(self._joycon, "accel_in_g"))
        except Exception:
            gyros, accs = [], []
        n = min(len(gyros), len(accs))
        if n > 0:
            sgx = sgy = sgz = 0.0
            sam = 0.0
            for i in range(n):
                gx, gy, gz = gyros[i]
                ax, ay, az = accs[i]
                # FRD -> FLU（y,z 取反）
                gx, gy, gz = float(gx), -float(gy), -float(gz)
                ax, ay, az = float(ax), -float(ay), -float(az)
                sgx += gx
                sgy += gy
                sgz += gz
                sam += math.sqrt(ax * ax + ay * ay + az * az)
            inv = 1.0 / float(n)
            self._calib_gyro_sum[0] += sgx * inv
            self._calib_gyro_sum[1] += sgy * inv
            self._calib_gyro_sum[2] += sgz * inv
            self._calib_acc_mag_sum += sam * inv
            self._calib_n += 1

        elapsed = now - float(self._calib_t0)
        if self.verbose and (now - self._calib_last_hint_t) > 0.5:
            self._calib_last_hint_t = now
            pct = _clamp(elapsed / float(self.home_calib_seconds), 0.0, 1.0) * 100.0
            print(f"[JoyCon] 校准中... {pct:5.1f}%  samples={self._calib_n}")

        if now < float(self._calib_t_end):
            return
        if self._calib_n < 10:
            # 时间到了但样本太少（蓝牙/循环频率低），允许继续采样一会儿
            return

        # 完成校准
        bgx = self._calib_gyro_sum[0] / float(self._calib_n)
        bgy = self._calib_gyro_sum[1] / float(self._calib_n)
        bgz = self._calib_gyro_sum[2] / float(self._calib_n)
        amref = self._calib_acc_mag_sum / float(self._calib_n)
        self._mahony.set_calibration(gyro_bias=(bgx, bgy, bgz), accel_mag_ref=amref)
        self._mahony.reset()

        with self._lock:
            self._rpy = [0.0, 0.0, 0.0]
            if self.home_resets_position:
                self._pos = [0.0, 0.0, 0.0]
            self._unlock_phys_rpy_ref = None
            self._unlock_virtual_rpy_ref = None

        self._calibrated = True
        self._locked = True  # 仍保持锁住，等待用户解锁
        self._calib_active = False
        if self.verbose:
            print(
                "[JoyCon] 校准完成："
                f" gyro_bias=({bgx:+.4f},{bgy:+.4f},{bgz:+.4f}) rad/s,"
                f" accel_mag_ref={amref:.3f}g。请按摇杆解锁运行。"
            )

    def _update_virtual_orientation(self, dt: float):
        # 锁死时不更新
        with self._lock:
            if self._locked:
                # 在锁住状态下，仍需要更新物理姿态以便在解锁时获得最新的物理姿态
                phys = self._get_phys_rpy_mahony(dt)
                self._last_phys_rpy = phys
                self._have_last_phys = True
                return
            gain = self.orientation_gain
            smooth = self.orientation_smoothing
            roll_w = self.roll_weight
            deadband = self.orientation_deadband_rad
            smooth_boost = self.stationary_smoothing_boost
            ref_phys = self._unlock_phys_rpy_ref
            ref_virt = self._unlock_virtual_rpy_ref

        phys = self._get_phys_rpy_mahony(dt)
        self._last_phys_rpy = phys
        self._have_last_phys = True

        # 静止时更稳一点（不改变动态时的延迟手感）
        if getattr(self._mahony, "is_stationary", False):
            smooth = _clamp(smooth + smooth_boost, 0.0, 1.0)

        if ref_phys is None or ref_virt is None:
            # 如果没有参考点，说明还没设置过，则使用当前物理姿态作为参考
            # 这种情况理论上不应该发生，因为我们会在解锁时设置参考点
            with self._lock:
                self._unlock_phys_rpy_ref = phys
                self._unlock_virtual_rpy_ref = (self._rpy[0], self._rpy[1], self._rpy[2])
            return

        # 计算当前物理姿态与参考物理姿态的差值
        dr = _wrap_pi(phys[0] - ref_phys[0]) * gain * roll_w
        dp = _wrap_pi(phys[1] - ref_phys[1]) * gain
        dy = _wrap_pi(phys[2] - ref_phys[2]) * gain

        if deadband > 0.0:
            if abs(dr) < deadband:
                dr = 0.0
            if abs(dp) < deadband:
                dp = 0.0
            if abs(dy) < deadband:
                dy = 0.0

        # 目标姿态：基于解锁时的虚拟姿态加上物理姿态的变化量
        target_r = ref_virt[0] + dr
        target_p = ref_virt[1] + dp
        target_y = ref_virt[2] + dy

        with self._lock:
            # 一阶低通：手感更"粘"，抑制抖动
            self._rpy[0] = (1.0 - smooth) * self._rpy[0] + smooth * target_r
            self._rpy[1] = (1.0 - smooth) * self._rpy[1] + smooth * target_p
            self._rpy[2] = (1.0 - smooth) * self._rpy[2] + smooth * target_y

    def _update_virtual_position(self, dt: float):
        with self._lock:
            if self._locked:
                return
            xy_speed = self.xy_speed
            z_speed = self.z_speed
            dz = self.stick_deadzone
            expo = self.stick_expo
            roll = float(self._rpy[0])
            pitch = float(self._rpy[1])
            yaw = float(self._rpy[2])

        raw = self._read_stick_raw()
        ux, uy = self._stick_to_unit(raw)
        ux = _apply_deadzone_and_expo(ux, dz, expo)
        uy = _apply_deadzone_and_expo(uy, dz, expo)

        with self._lock:
            cur_x = self._pos[0]
            cur_y = self._pos[1]
            cur_z = self._pos[2]
            pivot_x, pivot_y = self.xy_pivot

        dx = cur_x - pivot_x
        dy = cur_y - pivot_y
        r_xy = math.sqrt(dx * dx + dy * dy)
        if r_xy > 1e-6:
            dir_x = dx / r_xy
            dir_y = dy / r_xy
        else:
            dir_x = 1.0
            dir_y = 0.0

        dr = uy * xy_speed * dt
        if r_xy > 1e-6:
            dtheta = (-ux) * xy_speed * dt / r_xy
        else:
            dtheta = 0.0

        if abs(dr) > 1e-9:
            new_r = r_xy + dr
            new_r = max(0.0, new_r)
            new_x = pivot_x + dir_x * new_r
            new_y = pivot_y + dir_y * new_r
        else:
            new_x = cur_x
            new_y = cur_y

        if abs(dtheta) > 1e-9:
            cos_dt = math.cos(dtheta)
            sin_dt = math.sin(dtheta)
            tx = new_x - pivot_x
            ty = new_y - pivot_y
            rot_x = tx * cos_dt - ty * sin_dt
            rot_y = tx * sin_dt + ty * cos_dt
            new_x = rot_x + pivot_x
            new_y = rot_y + pivot_y

        st = self._safe_get_status()
        if not st:
            return
        btn_map = self._get_button_map()
        up_btn = "r" if self.device == "right" else "l"
        down_btn = "zr" if self.device == "right" else "zl"
        up = self._read_button(st, btn_map.get(up_btn, ("buttons", "right", "r")))
        down = self._read_button(st, btn_map.get(down_btn, ("buttons", "right", "zr")))

        vz = 0.0
        if up:
            vz += z_speed
        if down:
            vz -= z_speed

        new_z = cur_z + vz * dt

        with self._lock:
            if self.position_limits is not None:
                min_x, max_x, min_y, max_y, min_z, max_z = self.position_limits
                new_z = _clamp(new_z, min_z, max_z)

            pivot_x, pivot_y = self.xy_pivot
            min_r, max_r = self.xy_radius_limits
            dx = new_x - pivot_x
            dy = new_y - pivot_y
            r_xy = math.sqrt(dx * dx + dy * dy)
            if r_xy < min_r and r_xy > 1e-6:
                scale = min_r / r_xy
                new_x = pivot_x + dx * scale
                new_y = pivot_y + dy * scale
            elif r_xy > max_r:
                scale = max_r / r_xy
                new_x = pivot_x + dx * scale
                new_y = pivot_y + dy * scale

            self._pos[0] = new_x
            self._pos[1] = new_y
            self._pos[2] = new_z

    def _loop(self):
        dt_target = 1.0 / self.update_hz
        t_prev = time.perf_counter()
        while self._running and not self._stop_evt.is_set():
            now = time.perf_counter()
            dt = now - t_prev
            if dt < dt_target:
                time.sleep(max(0.0, dt_target - dt))
                continue
            t_prev = now
            dt = _clamp(dt, 1e-4, 0.2)

            # 1) 按键/状态读取（断链时会自动锁住并跳过更新）
            st = self._safe_get_status()
            if not st:
                time.sleep(0.05)
                continue
            # 0) HOME 单击校准（默认上电锁住，建议先校准）
            if self.require_home_calibration and not self._calibrated:
                self._update_home_calibration()
            self._handle_button_edges(st)

            # 2) 虚拟姿态（增量+平滑）
            self._update_virtual_orientation(dt)

            # 3) 虚拟位置（摇杆+R/ZR）
            self._update_virtual_position(dt)


if __name__ == "__main__":
    # 最小 demo：打印姿态与位置（按 stick_btn 锁死/解锁，HOME 复位姿态）
    ctl = JoyConSixDofController(device="right", update_hz=100.0)

    def on_a():
        print("\n[A] pressed")

    ctl.on("a", on_a)
    ctl.start()
    try:
        while True:
            pose = ctl.get_pose()
            x, y, z = pose.position
            dx, dy, dz = pose.direction
            r, p, yw = pose.euler_rpy
            print(
                f"\rlock={int(pose.locked)}  "
                f"pos=({x:+.3f},{y:+.3f},{z:+.3f})  "
                f"dir=({dx:+.3f},{dy:+.3f},{dz:+.3f})  "
                f"rpy=({r:+.2f},{p:+.2f},{yw:+.2f})",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        ctl.stop()
    


