#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

SDK_SRC = Path(__file__).resolve().parents[1] / "zyarm_sdk" / "python" / "src"
if str(SDK_SRC) not in sys.path:
    sys.path.insert(0, str(SDK_SRC))

from zyarm_sdk import ZyArm, ZyArmConfig  # noqa: E402
from zyarm_sdk.config import TeleopConfig  # noqa: E402
from zyarm_sdk.teleop import ZyArmTeleopPair  # noqa: E402


DEFAULT_FOLLOW_HZ = 50.0
DEFAULT_WAIT_TIMEOUT_MS = 50.0
STATS_PRINT_INTERVAL_S = 5.0

running = True


def _signal_handler(_signum, _frame) -> None:
    global running
    print("\n[INFO] 接收到退出信号，正在关闭...")
    running = False


def _print_teleop_stats(leader: ZyArm, follower: ZyArm) -> None:
    leader_stats = leader.get_frame_stats()
    follower_stats = follower.get_frame_stats()
    print(
        "[STATS] "
        f"md_received={leader_stats.master_data_received} "
        f"md_gap={leader_stats.master_data_gap_count} "
        f"md_hz={leader_stats.master_data_rate_hz:.1f} "
        f"follower_status={follower_stats.status_received} "
        f"follower_status_hz={follower_stats.status_rate_hz:.1f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="基于 ZyArmTeleopPair 的 fast_io 主从遥操工具")
    parser.add_argument("--leader", required=True, help="leader 串口，例如 /dev/ttyUSB0 或 COM3")
    parser.add_argument("--follower", required=True, help="follower 串口，例如 /dev/ttyUSB1 或 COM5")
    parser.add_argument("--baudrate", type=int, default=230400, help="串口波特率，默认 230400")
    parser.add_argument("--freq", type=float, default=DEFAULT_FOLLOW_HZ, help="遥操频率，单位 Hz，默认 50")
    parser.add_argument("--duration", type=float, default=0.0, help="运行时长，单位秒；0 表示直到 Ctrl-C")
    parser.add_argument(
        "--wait-timeout-ms",
        type=float,
        default=DEFAULT_WAIT_TIMEOUT_MS,
        help="等待主臂动作的超时时间，默认 50ms",
    )
    parser.add_argument("--action-max-age-ms", type=float, default=100.0, help="允许的主臂动作最大年龄，默认 100ms")
    args = parser.parse_args()

    if args.baudrate <= 0:
        print("[ERROR] 波特率必须大于 0 (--baudrate)")
        sys.exit(1)
    if args.freq <= 0.0:
        print("[ERROR] 遥操频率必须大于 0Hz (--freq)")
        sys.exit(1)
    if args.duration < 0.0:
        print("[ERROR] 运行时长不能小于 0 (--duration)")
        sys.exit(1)
    if args.wait_timeout_ms <= 0.0:
        print("[ERROR] 等待超时时间必须大于 0 (--wait-timeout-ms)")
        sys.exit(1)
    if args.action_max_age_ms <= 0.0:
        print("[ERROR] 动作最大年龄必须大于 0 (--action-max-age-ms)")
        sys.exit(1)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    leader = ZyArm(ZyArmConfig(port=args.leader, baudrate=args.baudrate))
    follower = ZyArm(ZyArmConfig(port=args.follower, baudrate=args.baudrate))
    config = TeleopConfig(
        leader_hz=args.freq,
        action_max_age_ms=args.action_max_age_ms,
        wait_timeout_ms=args.wait_timeout_ms,
    )

    pair = ZyArmTeleopPair(leader, follower, config=config)
    print("[INFO] fast_io 遥操会驱动真实从臂运动，请确认急停/断电手段可用。")
    print(f"[INFO] leader={args.leader}, follower={args.follower}, freq={args.freq:.1f}Hz")
    print("[INFO] ZyArmTeleopPair 会自动让 follower 进入固件 slave 模式，CMD36 fast_io 会经过 slave 滤波。")

    deadline = None if args.duration == 0.0 else time.time() + args.duration
    auto_follow_started = False
    try:
        pair.connect()
        leader.reset_frame_stats()
        follower.reset_frame_stats()
        pair.start_auto_follow()
        auto_follow_started = True
        next_stats_at = time.monotonic() + STATS_PRINT_INTERVAL_S
        while running:
            if deadline is not None and time.time() >= deadline:
                break
            now = time.monotonic()
            if now >= next_stats_at:
                _print_teleop_stats(leader, follower)
                next_stats_at = now + STATS_PRINT_INTERVAL_S
            time.sleep(0.2)
    finally:
        if auto_follow_started:
            pair.stop()
        leader.close()
        follower.close()
        print("[INFO] fast_io 遥操已关闭")


if __name__ == "__main__":
    main()
