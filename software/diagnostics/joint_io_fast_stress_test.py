#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CMD_ID_JOINT_IO_FAST 压力测试

作用：
- 以指定频率向机械臂发送 `[CMD][36][j0 j1 j2 j3 j4 j5 claw]`
- 统计实际发送频率与固件返回 `[STATUS]` 的接收频率
- 每个 4 秒运动周期发送一次 `[CMD][35]` 查询串口状态
- 用于评估当前固件在不同频率下能稳定支撑多高的 CMD_ID_JOINT_IO_FAST 频率

默认运动轨迹：
- 2 秒内线性插值到 J0:6.90 J1:-103.80 J2:39.80 J3:2.10 J4:44.70 J5:-2.60 CLAW:19.30
- 再 2 秒内线性插值恢复到 J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:0
"""

from __future__ import annotations

import argparse
import re
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Deque, List, Optional, Sequence

import serial


CMD_ID_JOINT_IO_FAST = 36
CMD_ID_SERIAL_STATUS = 35
DEFAULT_BAUDRATE = 230400
DEFAULT_FREQ_HZ = 50.0
DEFAULT_DURATION_S = 30.0
DEFAULT_MOVE_PHASE_S = 2.0
DEFAULT_CYCLE_S = DEFAULT_MOVE_PHASE_S * 2.0
DEFAULT_STATS_INTERVAL_S = DEFAULT_CYCLE_S
DEFAULT_WINDOW_SIZE = 400
INITIAL_ANGLES = [0.0, -180.0, 90.0, 0.0, 0.0, 0.0, 0.0]
TARGET_ANGLES = [6.90, -103.80, 39.80, 2.10, 44.70, -2.60, 19.30]

STATUS_RE = re.compile(
    r"\[STATUS\]\s*"
    r"J0:([-\d.]+)\s*J1:([-\d.]+)\s*J2:([-\d.]+)\s*J3:([-\d.]+)\s*"
    r"J4:([-\d.]+)\s*J5:([-\d.]+)\s*CLAW:([-\d.]+)"
)
CMD35_RESPONSE_RE = re.compile(r"\bCMD_ID=35\b")


def parse_status_frame(line: str) -> Optional[List[float]]:
    match = STATUS_RE.search(line)
    if match is None:
        return None
    return [float(match.group(i)) for i in range(1, 8)]


def build_joint_io_fast_command(angles: Sequence[float]) -> str:
    return "[CMD][36][{}]\n".format(" ".join(f"{angle:.2f}" for angle in angles))


def build_simple_command(cmd_id: int) -> str:
    return f"[CMD][{cmd_id}]\n"


def interpolate_angles(start: Sequence[float], end: Sequence[float], alpha: float) -> List[float]:
    clamped = min(max(alpha, 0.0), 1.0)
    return [start_angle + (end_angle - start_angle) * clamped for start_angle, end_angle in zip(start, end)]


@dataclass
class RollingFrequency:
    timestamps: Deque[float] = field(default_factory=lambda: deque(maxlen=DEFAULT_WINDOW_SIZE))

    def observe(self, timestamp: float) -> None:
        self.timestamps.append(timestamp)

    @property
    def hz(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        span = self.timestamps[-1] - self.timestamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self.timestamps) - 1) / span


class JointIoFastStressTest:
    def __init__(
        self,
        port: str,
        baudrate: int,
        target_freq_hz: float,
        duration_s: float,
        target_angles: Optional[Sequence[float]],
        stats_interval_s: float,
        read_timeout_s: float,
        print_rx: bool,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.target_freq_hz = target_freq_hz
        self.duration_s = duration_s
        self.fixed_angles = list(target_angles) if target_angles is not None else None
        self.stats_interval_s = stats_interval_s
        self.read_timeout_s = read_timeout_s
        self.print_rx = print_rx

        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.io_lock = threading.RLock()

        self.send_freq = RollingFrequency()
        self.status_freq = RollingFrequency()

        self.send_count = 0
        self.send_fail_count = 0
        self.status_count = 0
        self.other_line_count = 0
        self.error_line_count = 0
        self.status_miss_count = 0
        self.start_time = 0.0
        self.start_perf_time = 0.0
        self.motion_started_at = 0.0
        self.last_stats_time = 0.0
        self.last_stats_send_count = 0
        self.last_stats_status_count = 0
        self.cmd35_count = 0
        self.cmd35_fail_count = 0
        self.cmd35_response_count = 0
        self.last_cmd35_response: Optional[str] = None
        self.last_status: Optional[List[float]] = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"joint_io_fast_stress_{timestamp}.txt"
        self.log_file = open(self.log_filename, "w", encoding="utf-8")

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame) -> None:
        self._log(f"[INFO] 接收到退出信号: {signum}")
        self.running = False

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] {message}"
        print(formatted)
        self._write_log_line(formatted)

    def _write_log_line(self, line: str) -> None:
        try:
            self.log_file.write(line + "\n")
            self.log_file.flush()
        except Exception:
            pass

    def _open_serial(self) -> Optional[serial.Serial]:
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout_s,
                write_timeout=self.read_timeout_s,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self._log(f"[INFO] 串口连接成功: {self.port} @ {self.baudrate}")
            return ser
        except Exception as exc:
            self._log(f"[ERROR] 打开串口失败: {exc}")
            return None

    def _is_ignored_line(self, line: str) -> bool:
        if not line:
            return True
        if line.startswith("[CMD]"):
            return True
        return False

    def _handle_rx_line(self, line: str) -> None:
        status = parse_status_frame(line)
        if status is not None:
            now = time.perf_counter()
            self.status_count += 1
            self.status_freq.observe(now)
            self.last_status = status
            self._write_log_line(f"[RX][STATUS] {line}")
            if self.print_rx:
                self._log(f"[STATUS] {line}")
            return

        if CMD35_RESPONSE_RE.search(line):
            self.cmd35_response_count += 1
            self.last_cmd35_response = line
            self._log(f"[CMD35] {line}")
            return

        self.other_line_count += 1
        if "[ERROR]" in line or "[WARN]" in line:
            self.error_line_count += 1
        self._write_log_line(f"[RX] {line}")
        if self.print_rx:
            self._log(f"[RX] {line}")

    def _rx_loop(self) -> None:
        assert self.serial_port is not None

        while self.running:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip("\r\n")
                if self._is_ignored_line(line):
                    continue
                self._handle_rx_line(line)
            except Exception as exc:
                self._log(f"[ERROR] 接收线程异常: {exc}")
                time.sleep(0.01)

    def _current_motion_angles(self, timestamp: float) -> List[float]:
        if self.fixed_angles is not None:
            return list(self.fixed_angles)

        phase = (timestamp - self.motion_started_at) % DEFAULT_CYCLE_S
        if phase < DEFAULT_MOVE_PHASE_S:
            return interpolate_angles(INITIAL_ANGLES, TARGET_ANGLES, phase / DEFAULT_MOVE_PHASE_S)
        return interpolate_angles(TARGET_ANGLES, INITIAL_ANGLES, (phase - DEFAULT_MOVE_PHASE_S) / DEFAULT_MOVE_PHASE_S)

    def _send_serial_status_query(self) -> None:
        assert self.serial_port is not None

        command = build_simple_command(CMD_ID_SERIAL_STATUS)
        try:
            with self.io_lock:
                self.serial_port.write(command.encode("utf-8"))
                self.serial_port.flush()
            self.cmd35_count += 1
            self._write_log_line(f"[TX][CMD35] {command.strip()}")
        except Exception as exc:
            self.cmd35_fail_count += 1
            self._log(f"[ERROR] Failed to send CMD35 serial status query: {exc}")

    def _send_loop(self) -> None:
        assert self.serial_port is not None

        period = 1.0 / self.target_freq_hz
        next_deadline = time.perf_counter()

        while self.running:
            started_at = time.perf_counter()
            angles = self._current_motion_angles(started_at)
            command = build_joint_io_fast_command(angles)
            try:
                with self.io_lock:
                    self.serial_port.write(command.encode("utf-8"))
                    self.serial_port.flush()
                self.send_count += 1
                self.send_freq.observe(started_at)
            except Exception as exc:
                self.send_fail_count += 1
                self._log(f"[ERROR] 发送命令失败: {exc}")

            next_deadline += period
            sleep_time = next_deadline - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_deadline = time.perf_counter()

    def _status_ratio(self) -> float:
        if self.send_count <= 0:
            return 0.0
        return self.status_count / self.send_count * 100.0

    def _print_statistics(self) -> None:
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0.0
        now = time.perf_counter()
        if self.last_stats_time > 0.0:
            interval_s = now - self.last_stats_time
            interval_send_count = self.send_count - self.last_stats_send_count
            interval_status_count = self.status_count - self.last_stats_status_count
        else:
            interval_s = now - self.start_perf_time if self.start_perf_time > 0.0 else elapsed
            interval_send_count = self.send_count
            interval_status_count = self.status_count

        interval_send_hz = interval_send_count / interval_s if interval_s > 0.0 else 0.0
        interval_status_hz = interval_status_count / interval_s if interval_s > 0.0 else 0.0
        send_hz = self.send_freq.hz
        recv_hz = self.status_freq.hz
        ratio = self._status_ratio()

        print("\n" + "=" * 72)
        print(f"运行时间: {elapsed:.2f}s")
        print(f"运动周期: {DEFAULT_CYCLE_S:.2f}s ({DEFAULT_MOVE_PHASE_S:.2f}s 到目标 + {DEFAULT_MOVE_PHASE_S:.2f}s 回初始)")
        print(f"目标发送频率: {self.target_freq_hz:.2f} Hz")
        print(f"本周期CMD36发送帧率: {interval_send_hz:.2f} Hz ({interval_send_count} 帧 / {interval_s:.2f}s)")
        print(f"本周期STATUS接收帧率: {interval_status_hz:.2f} Hz ({interval_status_count} 帧 / {interval_s:.2f}s)")
        print(f"滚动CMD36发送帧率: {send_hz:.2f} Hz")
        print(f"滚动STATUS接收帧率: {recv_hz:.2f} Hz")
        print(f"发送总数: {self.send_count}")
        print(f"发送失败: {self.send_fail_count}")
        print(f"状态帧数: {self.status_count}")
        print(f"状态/发送比: {ratio:.2f}%")
        print(f"CMD35串口状态查询: 已发送 {self.cmd35_count}, 失败 {self.cmd35_fail_count}, 返回行 {self.cmd35_response_count}")
        print(f"其他串口行数: {self.other_line_count}")
        print(f"告警/错误行数: {self.error_line_count}")
        if self.last_status is not None:
            print(
                "最近状态: "
                + " ".join(f"J{i}:{angle:.2f}" for i, angle in enumerate(self.last_status[:6]))
                + f" CLAW:{self.last_status[6]:.2f}"
            )
        if self.last_cmd35_response is not None:
            print(f"最近CMD35返回: {self.last_cmd35_response}")
        print("=" * 72 + "\n")
        self.last_stats_time = now
        self.last_stats_send_count = self.send_count
        self.last_stats_status_count = self.status_count

    def _stats_loop(self) -> None:
        while self.running:
            time.sleep(self.stats_interval_s)
            if self.running:
                self._send_serial_status_query()
                self._print_statistics()

    def run(self) -> int:
        self.serial_port = self._open_serial()
        if self.serial_port is None:
            return 1

        self._log(f"[INFO] 日志文件: {self.log_filename}")
        self._log(f"[INFO] CMD_ID_JOINT_IO_FAST = {CMD_ID_JOINT_IO_FAST}")
        self._log(f"[INFO] 目标频率: {self.target_freq_hz:.2f} Hz")
        self._log(f"[INFO] 测试时长: {'直到 Ctrl+C' if self.duration_s <= 0 else f'{self.duration_s:.1f}s'}")
        if self.fixed_angles is None:
            self._log(
                "[INFO] 运动模式: CMD36线性插值 "
                f"{' '.join(f'{angle:.2f}' for angle in INITIAL_ANGLES)} -> "
                f"{' '.join(f'{angle:.2f}' for angle in TARGET_ANGLES)} -> "
                f"{' '.join(f'{angle:.2f}' for angle in INITIAL_ANGLES)}"
            )
            self._log(f"[INFO] 运动周期: {DEFAULT_CYCLE_S:.2f}s，每 {self.stats_interval_s:.2f}s 统计并发送 CMD35")
        else:
            self._log(f"[INFO] 固定角度模式: {' '.join(f'{angle:.2f}' for angle in self.fixed_angles)}")

        self.running = True
        self.start_time = time.time()
        self.start_perf_time = time.perf_counter()
        self.motion_started_at = self.start_perf_time

        rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="joint-io-fast-rx")
        send_thread = threading.Thread(target=self._send_loop, daemon=True, name="joint-io-fast-send")
        stats_thread = threading.Thread(target=self._stats_loop, daemon=True, name="joint-io-fast-stats")

        rx_thread.start()
        send_thread.start()
        stats_thread.start()

        try:
            if self.duration_s <= 0:
                while self.running:
                    time.sleep(0.2)
            else:
                deadline = time.time() + self.duration_s
                while self.running and time.time() < deadline:
                    time.sleep(0.2)
        finally:
            self.running = False
            send_thread.join(timeout=1.0)
            rx_thread.join(timeout=1.0)
            stats_thread.join(timeout=1.0)
            self._print_statistics()
            self._cleanup()

        return 0

    def _cleanup(self) -> None:
        self._log("[INFO] 正在清理资源...")
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                with self.io_lock:
                    self.serial_port.close()
                self._log("[INFO] 串口已关闭")
            except Exception as exc:
                self._log(f"[WARN] 关闭串口失败: {exc}")

        if self.log_file and not self.log_file.closed:
            self.log_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CMD_ID_JOINT_IO_FAST 压力测试")
    parser.add_argument("--port", required=True, help="机械臂串口，例如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help=f"串口波特率，默认 {DEFAULT_BAUDRATE}")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ_HZ, help=f"目标发送频率(Hz)，默认 {DEFAULT_FREQ_HZ}")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help=f"测试时长(秒)，<=0 表示持续运行直到 Ctrl+C，默认 {DEFAULT_DURATION_S}")
    parser.add_argument(
        "--angles",
        type=float,
        nargs=7,
        metavar=("J0", "J1", "J2", "J3", "J4", "J5", "CLAW"),
        default=None,
        help="可选：发送固定的 7 个关节目标角度；不指定时按默认 4 秒往返轨迹插值压力测试",
    )
    parser.add_argument("--stats-interval", type=float, default=DEFAULT_STATS_INTERVAL_S, help=f"统计打印周期(秒)，默认 {DEFAULT_STATS_INTERVAL_S}，同时发送 CMD35")
    parser.add_argument("--read-timeout", type=float, default=0.1, help="串口读超时(秒)，默认 0.1")
    parser.add_argument("--print-rx", action="store_true", help="打印接收到的 STATUS/日志行")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.freq <= 0:
        print("[ERROR] 发送频率必须大于 0Hz")
        return 1
    if args.baudrate <= 0:
        print("[ERROR] 波特率必须大于 0")
        return 1
    if args.stats_interval <= 0:
        print("[ERROR] 统计周期必须大于 0")
        return 1

    test = JointIoFastStressTest(
        port=args.port,
        baudrate=args.baudrate,
        target_freq_hz=args.freq,
        duration_s=args.duration,
        target_angles=args.angles,
        stats_interval_s=args.stats_interval,
        read_timeout_s=args.read_timeout,
        print_rx=args.print_rx,
    )
    return test.run()


if __name__ == "__main__":
    sys.exit(main())
