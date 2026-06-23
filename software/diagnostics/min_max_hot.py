#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂最大最远压力测试脚本。

流程：
    1. 连接机械臂并等待初始化完成；
    2. 先复位一次，确保从安全初始姿态开始；
    3. 重复指定次数：移动到 [400, 0, 0, 0, 0, 0]，然后复位。

使用方法:
    python min_max_hot.py <串口号> <重复次数>

示例:
    python min_max_hot.py COM3 10
    python min_max_hot.py COM5 50
"""

import argparse
import time

from sdk_helpers import create_arm, ok


TARGET_POSE = [400, 0, 0, 0, 0, 0]
STARTUP_WAIT_S = 5
DEFAULT_BAUDRATE = 230400


def parse_arguments():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="机械臂最大最远压力测试脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s COM3 10          # 使用 COM3 串口，重复测试 10 次
  %(prog)s COM5 50          # 使用 COM5 串口，重复测试 50 次
        """,
    )

    parser.add_argument(
        "serial_port",
        type=str,
        help="串口号（例如: COM3, COM4, /dev/ttyUSB0）",
    )
    parser.add_argument(
        "repeat_count",
        type=int,
        help="重复测试次数（正整数）",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"波特率（默认: {DEFAULT_BAUDRATE}）",
    )

    args = parser.parse_args()

    if args.repeat_count <= 0:
        parser.error("重复次数必须是正整数")

    return args


def reset_robot(robot, label):
    print(f"{label}...")
    if ok(robot.reset()):
        print("  -> 复位成功")
        return True

    print("  -> 复位失败")
    return False


def move_to_target(robot):
    x, y, z, rx, ry, rz = TARGET_POSE
    print(f"移动到目标位置: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")

    if ok(robot.move_ik(x, y, z, rx, ry, rz)):
        print("  -> 移动成功")
        return True

    print("  -> 移动失败")
    return False


def main():
    args = parse_arguments()

    print("=" * 60)
    print("机械臂最大最远压力测试")
    print("=" * 60)
    print(f"串口: {args.serial_port}")
    print(f"波特率: {args.baudrate}")
    print(f"重复次数: {args.repeat_count}")
    print(f"目标位置: {TARGET_POSE}")
    print("=" * 60)

    print("\n正在连接机器人...")
    robot = create_arm(args.serial_port, baudrate=args.baudrate)

    if not robot.is_connected:
        print("错误: 无法连接到机器人！")
        return -1

    success_count = 0
    fail_count = 0

    try:
        print("等待机器人初始化...")
        time.sleep(STARTUP_WAIT_S)

        if not reset_robot(robot, "\n安全初始复位"):
            return -1

        print(f"\n开始执行 {args.repeat_count} 次最大最远压测...")
        print("-" * 60)

        for index in range(1, args.repeat_count + 1):
            print(f"\n>>> 第 {index}/{args.repeat_count} 次测试")

            if not move_to_target(robot):
                fail_count += 1
                print("本轮移动失败，停止后续测试。")
                break

            if not reset_robot(robot, "复位机器人"):
                fail_count += 1
                print("本轮复位失败，停止后续测试。")
                break

            success_count += 1
            print(f"✓ 第 {index} 次测试完成")

        total_count = success_count + fail_count
        success_rate = (success_count / total_count * 100) if total_count else 0.0

        print("\n" + "=" * 60)
        print("测试完成！")
        print("=" * 60)
        print(f"计划测试次数: {args.repeat_count}")
        print(f"实际执行次数: {total_count}")
        print(f"成功次数: {success_count}")
        print(f"失败次数: {fail_count}")
        print(f"成功率: {success_rate:.2f}%")
        print("=" * 60)

        return 0 if fail_count == 0 else -1

    except KeyboardInterrupt:
        print("\n\n用户中断测试！")
        return -1
    except Exception as exc:
        print(f"\n错误: {exc}")
        import traceback

        traceback.print_exc()
        return -1
    finally:
        print("\n正在清理资源...")
        robot.close()
        print("完成！")


if __name__ == "__main__":
    raise SystemExit(main())
