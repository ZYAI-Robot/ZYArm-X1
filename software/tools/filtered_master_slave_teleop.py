#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import serial


MASTER_SAMPLE_PERIOD_MS = 10
DEFAULT_MASTER_SLAVE_FREQ_HZ = 50
MASTER_SLAVE_ROLE_MASTER = 1
MASTER_SLAVE_ROLE_SLAVE = 2
MD_FRAME_RE = re.compile(
    r"\[MD\]\[(\d)\]\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]"
)


def parse_md_frame(line: str) -> Optional[Tuple[int, List[float]]]:
    match = MD_FRAME_RE.search(line)
    if match is None:
        return None
    frame_id = int(match.group(1))
    angles = [float(match.group(i)) for i in range(2, 9)]
    return frame_id, angles


def build_master_slave_start_command(role: int, freq_hz: int = DEFAULT_MASTER_SLAVE_FREQ_HZ) -> str:
    return f"[CMD][32][{role} {freq_hz}]\n"


@dataclass
class FrameLossTracker:
    last_frame_id: Optional[int] = None
    received_frames: int = 0
    lost_frames: int = 0

    def observe(self, frame_id: int) -> int:
        self.received_frames += 1
        if self.last_frame_id is None:
            self.last_frame_id = frame_id
            return 0

        step = (frame_id - self.last_frame_id) % 10
        lost = 0 if step in (0, 1) else step - 1
        self.lost_frames += lost
        self.last_frame_id = frame_id
        return lost

    @property
    def total_expected_frames(self) -> int:
        return self.received_frames + self.lost_frames

    @property
    def loss_percent(self) -> float:
        total = self.total_expected_frames
        if total <= 0:
            return 0.0
        return self.lost_frames / total * 100.0


class MasterSlaveRemote:
    def __init__(self, mode, master_port=None, slave_port=None, freq_hz: int = DEFAULT_MASTER_SLAVE_FREQ_HZ):
        self.mode = mode
        self.master_port = master_port
        self.slave_port = slave_port
        self.freq_hz = freq_hz
        self.master_serial = None
        self.slave_serial = None
        self.running = False

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"master_slave_remote_log_{timestamp}.txt"
        self.log_file = open(self.log_filename, "w", encoding="utf-8")
        print(f"[INFO] 日志文件: {self.log_filename}")

        self.frame_tracker = FrameLossTracker()
        self.stats = {
            "total_packets": 0,
            "success_packets": 0,
            "failed_packets": 0,
            "start_time": None,
            "latency_samples": deque(maxlen=100),
            "master_received": 0,
            "slave_sent": 0,
            "frame_id_duplicates": 0,
        }

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n[INFO] 接收到退出信号，正在关闭...")
        self.running = False
        self._cleanup()
        sys.exit(0)

    def _log(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] {message}"
        print(formatted)
        try:
            self.log_file.write(formatted + "\n")
            self.log_file.flush()
        except Exception:
            pass

    def _open_serial(self, port):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=230400,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self._log(f"[INFO] 串口 {port} 连接成功")
            return ser
        except Exception as exc:
            self._log(f"[ERROR] 打开串口 {port} 失败: {exc}")
            return None

    def _send_command(self, ser, command):
        try:
            ser.write(command.encode("utf-8"))
            ser.flush()
            # self._log(f"[SEND] {command.strip()}")
            return True
        except Exception as exc:
            self._log(f"[ERROR] 发送命令失败: {exc}")
            return False

    def _parse_master_angles(self, master_data_line):
        parsed = parse_md_frame(master_data_line)
        if parsed is None:
            return None, None
        frame_id, angles = parsed
        return frame_id, angles

    def _record_frame(self, frame_id: int) -> None:
        previous = self.frame_tracker.last_frame_id
        lost = self.frame_tracker.observe(frame_id)
        if previous == frame_id:
            self.stats["frame_id_duplicates"] += 1
            self._log(f"[WARN] 检测到重复帧号: {frame_id}")
            return
        if lost > 0:
            self._log(
                f"[WARN] 检测到丢帧: 上一帧 {previous}, 当前帧 {frame_id}, 本次丢失 {lost} 帧, 累计丢失 {self.frame_tracker.lost_frames} 帧"
            )

    def _angles_to_cmd(self, angles):
        angles_str = " ".join(f"{angle:.2f}" for angle in angles)
        return f"[CMD][3][{angles_str}]\n"

    def _is_ignored_line(self, line: str) -> bool:
        if not line:
            return True
        if "[CMD" in line and "]" in line and ":" in line:
            return True
        if "Available commands" in line:
            return True
        return False

    def _handle_md_line(self, line: str, forward_to_slave: bool) -> None:
        frame_id, angles = self._parse_master_angles(line)
        if frame_id is None or angles is None:
            self._log(f"[WARN] 无法解析主臂数据: {line}")
            return

        self.stats["total_packets"] += 1
        self.stats["master_received"] += 1
        self._record_frame(frame_id)

        if not forward_to_slave:
            self.stats["success_packets"] += 1
            return

        started_at = time.perf_counter()
        slave_cmd = self._angles_to_cmd(angles)
        if self._send_command(self.slave_serial, slave_cmd):
            self.stats["slave_sent"] += 1
            self.stats["success_packets"] += 1
            self.stats["latency_samples"].append(time.perf_counter() - started_at)
        else:
            self.stats["failed_packets"] += 1
            self._log("[ERROR] Failed to send to slave")

    def _master_slave_loop(self):
        self.master_serial.reset_input_buffer()
        self.slave_serial.reset_input_buffer()
        self._log("[INFO] Master-slave loop started, buffers cleared")

        while self.running:
            try:
                line = self.master_serial.readline().decode("utf-8", errors="ignore").strip()
                if self._is_ignored_line(line):
                    continue
                # self._log(f"[MASTER] {line}")
                if line.startswith("[MD]"):
                    self._handle_md_line(line, forward_to_slave=True)
            except Exception as exc:
                self._log(f"[ERROR] Master-slave loop error: {exc}")
                time.sleep(0.01)

    def _read_master_data(self):
        try:
            if self.master_serial.in_waiting > 0:
                line = self.master_serial.readline().decode("utf-8", errors="ignore").strip()
                if self._is_ignored_line(line):
                    return None
                return line
            return None
        except Exception as exc:
            self._log(f"[ERROR] 读取主臂数据失败: {exc}")
            return None

    def _print_statistics(self):
        if not self.stats["start_time"]:
            return

        elapsed = time.time() - self.stats["start_time"]
        master_received = self.stats["master_received"]
        slave_sent = self.stats["slave_sent"]
        failed = self.stats["failed_packets"]
        success_rate = (slave_sent / master_received * 100.0) if master_received > 0 else 0.0

        if self.stats["latency_samples"]:
            avg_latency = sum(self.stats["latency_samples"]) / len(self.stats["latency_samples"])
            max_latency = max(self.stats["latency_samples"])
            min_latency = min(self.stats["latency_samples"])
        else:
            avg_latency = max_latency = min_latency = 0.0

        print("\n" + "=" * 60)
        print(f"运行时间: {elapsed:.2f}s")
        print(f"主臂收到: {master_received} 包")
        if self.mode == 1:
            print(f"从臂发送: {slave_sent} 包 ({success_rate:.2f}%)")
            print(f"发送失败: {failed} 包")
        print(f"接收帧数: {self.frame_tracker.received_frames} 帧")
        print(f"丢失帧数: {self.frame_tracker.lost_frames} 帧")
        print(f"预计总帧数: {self.frame_tracker.total_expected_frames} 帧")
        print(f"丢帧率: {self.frame_tracker.loss_percent:.2f}%")
        print(f"重复帧号: {self.stats['frame_id_duplicates']} 帧")
        if self.stats["latency_samples"]:
            print(
                f"延迟统计: 平均 {avg_latency * 1000:.2f}ms, 最大 {max_latency * 1000:.2f}ms, 最小 {min_latency * 1000:.2f}ms"
            )
        print("=" * 60 + "\n")

    def start_test_mode(self):
        self._log("[INFO] 启动测试模式")
        self._log(f"[INFO] 遥操频率: {self.freq_hz}Hz")
        self.master_serial = self._open_serial(self.master_port)
        if not self.master_serial:
            self._log("[ERROR] 无法打开主臂串口")
            return

        time.sleep(2)
        self.master_serial.reset_input_buffer()

        if not self._send_command(
            self.master_serial,
            build_master_slave_start_command(MASTER_SLAVE_ROLE_SLAVE, self.freq_hz),
        ):
            self._log("[ERROR] 发送测试模式命令失败")
            return

        self._log("[INFO] 等待固件进入卸力模式...")
        time.sleep(3)

        self._log("[INFO] 测试模式已启动，开始采集主臂数据...")
        self.running = True
        self.stats["start_time"] = time.time()

        stats_thread = threading.Thread(target=self._stats_thread, daemon=True)
        stats_thread.start()

        try:
            while self.running:
                line = self._read_master_data()
                if line:
                    self._log(f"[MASTER] {line}")
                    if line.startswith("[MD]"):
                        self._handle_md_line(line, forward_to_slave=False)
                time.sleep(0.01)
        except KeyboardInterrupt:
            self._log("[INFO] 用户中断")
        finally:
            self._cleanup()

    def start_master_slave_mode(self):
        self._log("[INFO] 启动主从跟随模式（单线程架构）")
        self._log(f"[INFO] 主臂采样周期: {MASTER_SAMPLE_PERIOD_MS}ms")
        self._log(f"[INFO] 遥操频率: {self.freq_hz}Hz")

        self.master_serial = self._open_serial(self.master_port)
        if not self.master_serial:
            self._log("[ERROR] 无法打开主臂串口")
            return

        self.slave_serial = self._open_serial(self.slave_port)
        if not self.slave_serial:
            self._log("[ERROR] 无法打开从臂串口")
            return

        time.sleep(2)
        self.master_serial.reset_input_buffer()
        self.slave_serial.reset_input_buffer()

        if not self._send_command(
            self.master_serial,
            build_master_slave_start_command(MASTER_SLAVE_ROLE_MASTER, self.freq_hz),
        ):
            self._log("[ERROR] 发送主臂启动命令失败")
            return

        time.sleep(1)

        if not self._send_command(
            self.slave_serial,
            build_master_slave_start_command(MASTER_SLAVE_ROLE_SLAVE, self.freq_hz),
        ):
            self._log("[ERROR] 发送从臂启动命令失败")
            return

        self._log("[INFO] 等待固件进入工作状态...")
        time.sleep(3)

        self._log("[INFO] 主从跟随模式已启动，开始单线程循环...")
        self.running = True
        self.stats["start_time"] = time.time()

        stats_thread = threading.Thread(target=self._stats_thread, daemon=True, name="Stats")
        stats_thread.start()

        try:
            self._master_slave_loop()
        except KeyboardInterrupt:
            self._log("[INFO] 用户中断")
        finally:
            self._cleanup()

    def _stats_thread(self):
        while self.running:
            time.sleep(5)
            if self.running:
                self._print_statistics()

    def _stop_firmware_master_slave(self):
        stop_cmd = "[CMD][33][]\n"
        ack_keyword = "ACK_COMPLETED: CMD_ID=33"
        ack_timeout = 3.0

        targets = []
        if self.master_serial and self.master_serial.is_open:
            targets.append(("主臂", self.master_serial))
        if self.slave_serial and self.slave_serial.is_open:
            targets.append(("从臂", self.slave_serial))

        for name, ser in targets:
            try:
                self._log(f"[INFO] 向{name}发送停止命令 CMD_ID=33...")
                ser.write(stop_cmd.encode("utf-8"))
                ser.flush()

                deadline = time.time() + ack_timeout
                while time.time() < deadline:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self._log(f"[{name}] {line}")
                        if ack_keyword in line:
                            self._log(f"[INFO] {name}已确认停止")
                            break
                    time.sleep(0.05)
                else:
                    self._log(f"[WARN] {name}停止命令超时（{ack_timeout}s），强制关闭串口")
            except Exception as exc:
                self._log(f"[ERROR] 向{name}发送停止命令失败: {exc}")

    def _cleanup(self):
        self._log("[INFO] 正在清理资源...")
        self._print_statistics()
        self._stop_firmware_master_slave()

        if self.master_serial and self.master_serial.is_open:
            self.master_serial.close()
            self._log("[INFO] 主臂串口已关闭")

        if self.slave_serial and self.slave_serial.is_open:
            self.slave_serial.close()
            self._log("[INFO] 从臂串口已关闭")

        if self.log_file and not self.log_file.closed:
            self.log_file.close()


def main():
    parser = argparse.ArgumentParser(description="主从跟随远程控制脚本")
    parser.add_argument("--mode", type=int, required=True, choices=[1, 2], help="工作模式：1=主从模式，2=测试模式")
    parser.add_argument("--master", type=str, required=True, help="主臂串口号，例如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--slave", type=str, help="从臂串口号，例如 COM5 或 /dev/ttyUSB1，仅主从模式需要")
    parser.add_argument("--freq", type=int, default=DEFAULT_MASTER_SLAVE_FREQ_HZ, help="遥操频率，单位 Hz，默认 50")

    args = parser.parse_args()

    if args.mode == 1 and not args.slave:
        print("[ERROR] 主从模式需要指定从臂串口号 (--slave)")
        sys.exit(1)
    if args.freq <= 0:
        print("[ERROR] 遥操频率必须大于 0Hz (--freq)")
        sys.exit(1)

    remote = MasterSlaveRemote(args.mode, args.master, args.slave, freq_hz=args.freq)
    if args.mode == 1:
        remote.start_master_slave_mode()
    else:
        remote.start_test_mode()


if __name__ == "__main__":
    main()
