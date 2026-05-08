"""
机械臂串口控制器

作用：
- 从 JoyConSixDofController 读取虚拟位置/欧拉角（rpy）
- 连接串口后自动进入“遥控模式”：
  - 发送 "[CMD][24][1]"（enable）并等待 ACK SUCCESS
  - 发送 "[CMD][25]"（遥控复位/初始化姿态）并等待 ACK SUCCESS
- JoyCon 解锁后，以固定周期发送遥控位姿：
  - 发送 "[CMD][26][x y z rx ry rz claw_angle]\\n"（无返回值，不等待 ACK）
- JoyCon 锁住后停止发送遥控位姿
- B 长按：额外发送 "[CMD][25]" 并等待 ACK SUCCESS（JoyCon 内部默认也会复位虚拟位姿并锁住）
- 程序退出：发送 "[CMD][24][0]" 关闭遥控模式（可等待 ACK），最后发送 "[CMD][1]" 整体复位（可等待 ACK）

依赖：
- pyserial（pip install pyserial）
"""

from __future__ import annotations

import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Union
import datetime
import os

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None  # pyserial 未安装时延迟报错

from joycon_sixdof_controller import JoyConSixDofController


@dataclass
class ArmSerialConfig:
    port: str
    baudrate: int = 115200
    # JoyCon 解锁后，遥控位姿发送频率（需求：每 10ms -> 100Hz）
    send_hz: float = 100.0
    timeout_s: float = 0.1
    write_timeout_s: float = 0.1

    # 协议命令 ID（按用户定义）
    remote_enable_cmd_id: int = 24  # enable/disable 遥控模式
    remote_reset_cmd_id: int = 25   # 遥控复位/初始化姿态
    teleop_cmd_id: int = 26         # 遥控位姿（无返回）
    global_reset_cmd_id: int = 1    # 整体复位

    # ACK 等待参数
    ack_timeout_s: float = 2.0
    ack_retry: int = 1
    # 串口输出打印：运行过程中把机械臂的串口输出实时打印出来
    print_rx: bool = True
    rx_print_prefix: str = "[ARM][RX]"
    # 有些机械臂固件会把我们发送的 "[CMD]..." 原样回显到串口输出；默认过滤掉避免干扰阅读/ACK 等待
    ignore_cmd_echo: bool = True
    # 协议要求：rx/ry/rz（以及 claw_angle）单位为“度”。JoyConSixDofController 输出为 rad，因此默认做 rad->deg
    angles_in_degrees: bool = True
    # 协议要求：x/y/z 单位为“mm”。JoyCon 输出单位由你业务定义：
    # - 若你把虚拟 position 当成“m”，这里可设 1000.0 转为 mm
    # - 若你已直接以“mm”维护 position，这里保持 1.0
    pos_scale: float = 1.0
    # 锁死时是否仍持续发送（默认发送最后状态；你也可以改成 False 让机械臂停留在上一次）
    send_when_locked: bool = False
    # 浮点格式（避免过长字符串/带宽浪费；同时保持可读）
    float_fmt: str = "{:+.3f}"

    # 终端状态行：用 \r 在同一行持续显示“当前遥控位置与角度”
    # 注意：若 print_rx=True，会和 RX 日志混在一起；本项目会自动在打印 RX/ACK 前清理状态行并重绘。
    print_status_line: bool = True
    status_hz: float = 20.0

    # 夹爪角度（0~100）
    claw_init: float = 0.0
    claw_min: float = 0.0
    claw_max: float = 100.0
    # 按住 Y 增大、按住 A 减小的变化速度（单位：角度值/秒）
    claw_rate_per_s: float = 60.0


class ArmSerialController:
    """
    机械臂串口控制器：
    - start()/stop()：后台线程按 send_hz 发送
    - connect()/close()：串口管理（start 时会自动 connect）
    """

    def __init__(self, joycon: JoyConSixDofController, config: ArmSerialConfig):
        self.joycon = joycon
        self.cfg = config

        self._ser: Optional["serial.Serial"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        # 串口读写互斥锁：send_command(wait_ack=True) 需要"写+读"原子化；重连路径也可能重入，因此用 RLock
        self._io_lock = threading.RLock()
        self._last_cmd: Optional[str] = None
        self._pause_send = threading.Event()

        # 串口接收线程：持续读取机械臂输出，打印并分发 ACK
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop_evt = threading.Event()
        self._ack_cv = threading.Condition()
        # cmd_id -> (ok, raw_line, t)
        self._ack_state: dict[int, tuple[bool, str, float]] = {}
        self._rx_buf = bytearray()

        # 终端状态行（\r 覆盖刷新）
        self._status_lock = threading.Lock()
        self._status_line: str = ""
        self._status_line_len: int = 0
        self._status_last_print_t: float = 0.0

        self._claw_angle: float = float(self.cfg.claw_init)
        self._claw_angle = self._clamp(self._claw_angle, self.cfg.claw_min, self.cfg.claw_max)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"arm_serial_log_{timestamp}.txt"
        self.log_file = None
        self._log_queue: List[str] = []
        self._log_lock = threading.Lock()
        self._log_flush_evt = threading.Event()

        if self.cfg.send_hz <= 0:
            raise ValueError("send_hz 必须 > 0")

    def connect(self):
        if serial is None:  # pragma: no cover
            raise RuntimeError("未安装 pyserial：请先执行 `pip install pyserial`")
        with self._lock:
            if self._ser and self._ser.is_open:
                return
            self._ser = serial.Serial(
                port=self.cfg.port,
                baudrate=int(self.cfg.baudrate),
                timeout=float(self.cfg.timeout_s),
                write_timeout=float(self.cfg.write_timeout_s),
            )
        # 打开串口后先清一下输入缓存，避免读到历史残留 ACK
        try:
            with self._io_lock:
                with self._lock:
                    ser = self._ser
                if ser and getattr(ser, "is_open", False):
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
        except Exception:
            pass
        # 启动接收线程（确保 ACK 不会丢）
        self._ensure_rx_thread()
        self.log_file = open(self.log_filename, 'w', encoding='utf-8')
        self._log_flush_evt.clear()
        t = threading.Thread(target=self._log_flush_loop, daemon=True)
        t.start()
        self._print_log_line(f"[INFO] Started logging to {self.log_filename} at {datetime.datetime.now()}")

    def close(self):
        try:
            self._stop_rx_thread(join_timeout_s=0.5)
        except Exception:
            pass
        self._log_flush_evt.set()
        with self._lock:
            ser = self._ser
            self._ser = None
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass
        with self._log_lock:
            if self._log_queue:
                remaining = list(self._log_queue)
                self._log_queue = []
                if self.log_file and not self.log_file.closed:
                    try:
                        self.log_file.writelines(remaining)
                        self.log_file.flush()
                    except Exception:
                        pass
        if self.log_file and not self.log_file.closed:
            try:
                self.log_file.write(f"[INFO] Closing log at {datetime.datetime.now()}\n")
                self.log_file.close()
            except Exception:
                pass

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self.connect()
        # 连接后按照协议：进入遥控模式 -> 遥控复位（均需 ACK）
        if not self.enable_remote(True, wait_ack=True):
            raise RuntimeError("进入遥控模式失败：CMD24 enable 未收到 SUCCESS ACK")
        if not self.remote_reset(wait_ack=True):
            raise RuntimeError("遥控复位失败：CMD25 未收到 SUCCESS ACK")

        # B 长按：额外触发一次机械臂遥控复位（JoyCon 默认长按 B 也会复位虚拟点并锁住）
        try:
            # 注意：JoyCon 的回调是在 JoyCon 更新线程里同步执行的。
            # 若这里直接 wait_ack，会阻塞 JoyCon 采样与按键处理，导致体验变差。
            self.joycon.on_long("b", lambda: self._run_async(lambda: self.remote_reset(wait_ack=True)))
        except Exception:
            pass

        self._stop_evt.clear()
        self._pause_send.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 1.0, run_shutdown_sequence: bool = True):
        self._stop_evt.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=join_timeout_s)

        if run_shutdown_sequence:
            # 退出顺序：关闭遥控模式 -> 整体复位
            try:
                self.enable_remote(False, wait_ack=True)
            except Exception:
                pass
            try:
                self.global_reset(wait_ack=True)
            except Exception:
                pass
        self.close()

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        return lo if x < lo else (hi if x > hi else x)

    def get_claw_angle(self) -> float:
        return float(self._claw_angle)

    def set_claw_angle(self, angle: float):
        self._claw_angle = self._clamp(float(angle), float(self.cfg.claw_min), float(self.cfg.claw_max))

    # ---------------------------
    # 协议：命令组帧 / ACK 解析
    # ---------------------------
    _ACK_RE = re.compile(r"ACK_COMPLETED:\s*CMD_ID=(\d+),\s*(SUCCESS|ERROR)\s*$")

    def _format_cmd(self, cmd_id: int, params: Optional[Sequence[Union[int, float]]] = None) -> str:
        """
        协议：
        - 无参数: "[CMD][25]\\n"
        - 有参数: "[CMD][24][1]\\n" 或 "[CMD][26][x y z ...]\\n"（最多 10 个数）
        """
        cid = int(cmd_id)
        if not params:
            return f"[CMD][{cid}]\n"
        if len(params) > 10:
            raise ValueError("params 最多 10 个")
        f = self.cfg.float_fmt.format
        parts: List[str] = []
        for p in params:
            if isinstance(p, int):
                parts.append(str(int(p)))
            else:
                parts.append(f(float(p)))
        return f"[CMD][{cid}][{' '.join(parts)}]\n"

    def _read_line(self) -> Optional[str]:
        with self._lock:
            ser = self._ser
        if not ser or not getattr(ser, "is_open", False):
            return None
        try:
            b = ser.readline()
        except Exception:
            return None
        if not b:
            return None
        try:
            return b.decode("utf-8", errors="ignore").strip("\r\n")
        except Exception:
            return None

    def _ensure_rx_thread(self):
        """确保串口接收线程在运行。"""
        t = self._rx_thread
        if t and t.is_alive():
            return
        self._rx_stop_evt.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _stop_rx_thread(self, *, join_timeout_s: float = 0.5):
        self._rx_stop_evt.set()
        t = self._rx_thread
        if t and t.is_alive():
            t.join(timeout=join_timeout_s)
        self._rx_thread = None

    def _log_flush_loop(self):
        while not self._log_flush_evt.is_set():
            self._log_flush_evt.wait(timeout=0.05)
            with self._log_lock:
                if not self._log_queue:
                    continue
                lines = self._log_queue
                self._log_queue = []
            if self.log_file and not self.log_file.closed:
                try:
                    self.log_file.writelines(lines)
                    self.log_file.flush()
                except Exception:
                    pass

    def _rx_loop(self):
        """
        串口接收线程：
        - 持续读取机械臂输出
        - 打印输出
        - 解析 ACK_COMPLETED 并唤醒等待者
        """
        while (not self._rx_stop_evt.is_set()) and (not self._stop_evt.is_set()):
            # 用字节缓冲按 '\n' 切行：
            # - 避免 readline() 在超时返回“半行”导致打印/解析出现拼接（例如你看到的 "[1[CMD][26]..."）
            with self._lock:
                ser = self._ser
            if not ser or not getattr(ser, "is_open", False):
                time.sleep(0.05)
                continue
            try:
                chunk = ser.read(getattr(ser, "in_waiting", 0) or 1)
            except Exception:
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._rx_buf.extend(chunk)
            while True:
                nl = self._rx_buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(self._rx_buf[:nl])
                del self._rx_buf[: nl + 1]
                try:
                    line = raw.decode("utf-8", errors="ignore").strip("\r\n")
                except Exception:
                    line = ""
                if not line:
                    continue

                # 先判断 ACK 行（用于等待）
                m = self._ACK_RE.match(line.strip())
                if m:
                    cmd_id = int(m.group(1))
                    status = str(m.group(2)).upper()
                    ok = (status == "SUCCESS")
                    with self._ack_cv:
                        self._ack_state[cmd_id] = (ok, line, time.perf_counter())
                        self._ack_cv.notify_all()
                    # ACK 也打印出来，方便排查
                    if self.cfg.print_rx:
                        self._print_log_line(f"[ARM][ACK] {line}")
                    continue

                # 回显过滤：固件若回显我们发出的 [CMD]...，默认不打印
                if self.cfg.ignore_cmd_echo and ("[CMD]" in line):
                    continue

                # 普通输出：直接打印
                if self.cfg.print_rx:
                    self._print_log_line(f"{self.cfg.rx_print_prefix} {line}")

    def _wait_ack(self, cmd_id: int, *, timeout_s: float) -> Tuple[bool, Optional[str]]:
        """
        等待形如：
        - SUCCESS: "ACK_COMPLETED: CMD_ID=%d, SUCCESS"
        - ERROR:   "ACK_COMPLETED: CMD_ID=%d, ERROR"
        """
        deadline = time.perf_counter() + float(timeout_s)
        target = int(cmd_id)
        with self._ack_cv:
            while True:
                v = self._ack_state.get(target)
                if v is not None:
                    ok, raw, _t = v
                    return (bool(ok), raw)
                now = time.perf_counter()
                remain = deadline - now
                if remain <= 0.0:
                    return (False, None)
                self._ack_cv.wait(timeout=remain)

    def _write_line(self, s: str, ser=None):
        if s and (not s.endswith("\n")):
            s = s + "\n"
        if ser is None:
            with self._lock:
                ser = self._ser
        if not ser or not getattr(ser, "is_open", False):
            return
        try:
            ser.write(s.encode("utf-8"))
        except Exception:
            try:
                self.close()
                self.connect()
                with self._lock:
                    ser2 = self._ser
                if ser2 and ser2.is_open:
                    ser2.write(s.encode("utf-8"))
            except Exception:
                pass

    def _run_async(self, fn):
        """用后台线程执行可能阻塞的操作（如 wait_ack），避免阻塞 JoyCon 采样线程。"""
        try:
            t = threading.Thread(target=fn, daemon=True)
            t.start()
        except Exception:
            # 忽略：不让异步调度失败影响主流程
            pass

    def send_command(
        self,
        cmd_id: int,
        params: Optional[Sequence[Union[int, float]]] = None,
        *,
        wait_ack: bool = False,
        ack_timeout_s: Optional[float] = None,
    ) -> bool:
        """
        发送命令；如 wait_ack=True 则等待对应 CMD_ID 的 ACK_COMPLETED。

        teleop（CMD26）无返回值，请 wait_ack=False。
        """
        # 复位期间机械臂不接收任何指令：这里做“硬拦截”，避免其他线程/回调误发。
        # 允许发送的只有复位相关指令本身（CMD25 / CMD1）。
        cid = int(cmd_id)
        if self._pause_send.is_set():
            if cid not in (int(self.cfg.remote_reset_cmd_id), int(self.cfg.global_reset_cmd_id)):
                return False

        cmd = self._format_cmd(cid, params)
        self._last_cmd = cmd
        if not wait_ack:
            with self._lock:
                ser = self._ser
            self._write_line(cmd, ser)
            return True

        timeout = float(self.cfg.ack_timeout_s if ack_timeout_s is None else ack_timeout_s)
        # 发送动作需要与 teleop 写入互斥，避免交叉输出
        with self._io_lock:
            # 等待 ACK 的命令：发送前先清串口输入缓存，避免历史残留 ACK 误匹配
            try:
                with self._lock:
                    ser = self._ser
                if ser and getattr(ser, "is_open", False):
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
            except Exception:
                pass
            # 清掉同 cmd_id 的历史 ACK（必须在 write 之前；否则可能把“刚到的 ACK”误删）
            try:
                with self._ack_cv:
                    self._ack_state.pop(cid, None)
            except Exception:
                pass
            self._write_line(cmd)
        # 等待 ACK 不需要持有 io_lock，避免阻塞 teleop 写入；ACK 由接收线程分发
        ok, _raw = self._wait_ack(cid, timeout_s=timeout)
        return bool(ok)

    def enable_remote(self, enable: bool, *, wait_ack: bool = True) -> bool:
        v = 1 if bool(enable) else 0
        for _ in range(max(1, int(self.cfg.ack_retry) + 1)):
            ok = self.send_command(int(self.cfg.remote_enable_cmd_id), [v], wait_ack=wait_ack)
            if ok or not wait_ack:
                return bool(ok)
        return False

    def remote_reset(self, *, wait_ack: bool = True) -> bool:
        self._pause_send.set()
        try:
            for _ in range(max(1, int(self.cfg.ack_retry) + 1)):
                ok = self.send_command(int(self.cfg.remote_reset_cmd_id), None, wait_ack=wait_ack)
                if ok or not wait_ack:
                    self._claw_angle: float = float(self.cfg.claw_init)
                    return bool(ok)
            return False
        finally:
            self._pause_send.clear()

    def global_reset(self, *, wait_ack: bool = True) -> bool:
        self._pause_send.set()
        try:
            return bool(self.send_command(int(self.cfg.global_reset_cmd_id), None, wait_ack=wait_ack))
        finally:
            self._pause_send.clear()

    def _loop(self):
        dt_target = 1.0 / float(self.cfg.send_hz)
        spin_threshold = dt_target * 0.5
        t_prev = time.perf_counter()
        while not self._stop_evt.is_set():
            if self._pause_send.is_set():
                time.sleep(0.005)
                t_prev = time.perf_counter()
                continue

            now = time.perf_counter()
            dt = now - t_prev
            if dt < dt_target:
                remaining = dt_target - dt
                if remaining >= spin_threshold:
                    time.sleep(remaining * 0.8)
                continue
            t_prev += dt_target
            dt = self._clamp(float(dt), 1e-4, 0.2)

            pose = self.joycon.get_pose()
            if pose.locked and not self.cfg.send_when_locked:
                # 即使锁住不发，也刷新一次状态行，让终端能看到“锁住”状态
                self._maybe_print_status_line(
                    pose_locked=True,
                    x=float(pose.position[0]) * float(self.cfg.pos_scale),
                    y=float(pose.position[1]) * float(self.cfg.pos_scale),
                    z=float(pose.position[2]) * float(self.cfg.pos_scale),
                    r=float(pose.euler_rpy[0]),
                    p=float(pose.euler_rpy[1]),
                    yw=float(pose.euler_rpy[2]),
                    claw=float(self._claw_angle),
                )
                continue

            x, y, z = pose.position
            r, p, yw = pose.euler_rpy  # rad

            # 夹爪：按住 Y 增大，按住 A 减小（范围 0~100）
            y_down = self.joycon.get_button_down("y")
            a_down = self.joycon.get_button_down("a")
            if y_down or a_down:
                delta = 0.0
                if y_down:
                    delta += float(self.cfg.claw_rate_per_s) * dt
                if a_down:
                    delta -= float(self.cfg.claw_rate_per_s) * dt
                self._claw_angle = self._clamp(
                    self._claw_angle + delta,
                    float(self.cfg.claw_min),
                    float(self.cfg.claw_max),
                )

            x *= float(self.cfg.pos_scale)
            y *= float(self.cfg.pos_scale)
            z *= float(self.cfg.pos_scale)

            if self.cfg.angles_in_degrees:
                r = math.degrees(r)
                p = math.degrees(p)
                yw = math.degrees(yw)

            self._maybe_print_status_line(
                pose_locked=bool(pose.locked),
                x=float(x),
                y=float(y),
                z=float(z),
                r=float(r),
                p=float(p),
                yw=float(yw),
                claw=float(self._claw_angle),
            )

            # CMD26：无返回值，不等待 ACK
            # 参数：x y z rx ry rz claw_angle（最多 10 个参数的子集）
            self.send_command(
                int(self.cfg.teleop_cmd_id),
                [x, y, z, r, p, yw, float(self._claw_angle)],
                wait_ack=False,
            )

    # ---------------------------
    # 终端状态行（\r 单行刷新）
    # ---------------------------
    def _clear_status_line(self):
        if not self.cfg.print_status_line:
            return
        with self._status_lock:
            n = int(self._status_line_len)
            if n <= 0:
                return
            try:
                sys.stdout.write("\r" + (" " * n) + "\r")
                sys.stdout.flush()
            except Exception:
                pass

    def _print_log_line(self, s: str):
        if self.cfg.print_status_line:
            self._clear_status_line()
        timestamp = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S.%f] ")
        line = timestamp + s + "\n"
        with self._log_lock:
            self._log_queue.append(line)
        if self.cfg.print_status_line:
            self._redraw_status_line()

    def _redraw_status_line(self):
        if not self.cfg.print_status_line:
            return
        with self._status_lock:
            s = self._status_line
            n = int(self._status_line_len)
        if not s:
            return
        try:
            # 用 \r 覆盖同一行；末尾加空格用于覆盖“变短”的情况
            sys.stdout.write("\r" + s + (" " * max(0, n - len(s))))
            sys.stdout.flush()
        except Exception:
            pass

    def _maybe_print_status_line(
        self,
        *,
        pose_locked: bool,
        x: float,
        y: float,
        z: float,
        r: float,
        p: float,
        yw: float,
        claw: float,
    ):
        if not self.cfg.print_status_line:
            return
        hz = float(self.cfg.status_hz)
        if hz <= 0:
            interval = 0.0
        else:
            interval = 1.0 / hz

        now = time.perf_counter()
        with self._status_lock:
            if interval > 0.0 and (now - float(self._status_last_print_t)) < interval:
                return
            self._status_last_print_t = now

        f = self.cfg.float_fmt.format
        lock_str = "LOCK" if bool(pose_locked) else "UNLOCK"
        ang_unit = "deg" if bool(self.cfg.angles_in_degrees) else "rad"
        s = (
            f"[POSE][{lock_str}] "
            f"xyz(mm)=({f(x)},{f(y)},{f(z)}) "
            f"rpy({ang_unit})=({f(r)},{f(p)},{f(yw)}) "
            f"claw={f(claw)}"
        )

        with self._status_lock:
            self._status_line = s
            self._status_line_len = max(int(self._status_line_len), len(s))

        self._redraw_status_line()


if __name__ == "__main__":
    # 示例：
    # python arm_serial_controller.py --port COM3 --baudrate 230400 --send-hz 100
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            "JoyCon -> 机械臂串口遥控：连接后自动 [CMD][24][1] + [CMD][25] 等 ACK；"
            "解锁后按 send_hz 发送 [CMD][26][x y z rx ry rz claw_angle]（无 ACK；单位：xyz=mm, rxyz=deg 默认）。"
        )
    )
    parser.add_argument("--port", required=True, help="串口端口，例如 COM3")
    parser.add_argument("--baudrate", type=int, default=230400)
    parser.add_argument("--send-hz", type=float, default=60.0)
    parser.add_argument("--angles-rad", action="store_true", help="改为以 rad 发送（默认发送 deg）")
    parser.add_argument("--pos-scale", type=float, default=1, help="位置缩放系数（把 JoyCon position 换算到 mm）")
    parser.add_argument("--claw-init", type=float, default=0.0, help="夹爪初始角度（0~100）")
    parser.add_argument("--claw-rate", type=float, default=60.0, help="按住Y/A时夹爪变化速度（角度值/秒）")
    parser.add_argument("--no-status-line", action="store_true", help="禁用终端单行状态显示（\\r 刷新）")
    parser.add_argument("--status-hz", type=float, default=20.0, help="终端状态行刷新频率（Hz），默认 20")
    parser.add_argument("--device", choices=["right", "left"], default="right")
    args = parser.parse_args()

    joy = JoyConSixDofController(device=args.device, update_hz=100.0)
    joy.start()

    arm = ArmSerialController(
        joy,
        ArmSerialConfig(
            port=args.port,
            baudrate=args.baudrate,
            send_hz=args.send_hz,
            angles_in_degrees=not bool(args.angles_rad),
            pos_scale=float(args.pos_scale),
            claw_init=float(args.claw_init),
            claw_rate_per_s=float(args.claw_rate),
            print_status_line=not bool(args.no_status_line),
            status_hz=float(args.status_hz),
        ),
    )
    arm.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        arm.stop(run_shutdown_sequence=True)
        joy.stop()

