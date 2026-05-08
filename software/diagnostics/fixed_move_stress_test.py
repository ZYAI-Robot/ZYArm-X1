#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂随机移动压力测试脚本
随机移动到指定位置列表中的一个，然后控制夹爪开合，重复指定次数

使用方法:
    python random_move_stress_test.py <串口号> <重复次数>
    
示例:
    python random_move_stress_test.py COM3 10
    python random_move_stress_test.py COM5 50
"""

import sys
import os
import time
import random
import argparse

from sdk_helpers import create_arm, ok


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='机械臂随机移动压力测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  %(prog)s COM3 10          # 使用COM3串口，重复测试10次
  %(prog)s COM5 50          # 使用COM5串口，重复测试50次
        ''')
    
    parser.add_argument('serial_port', 
                        type=str,
                        help='串口号（例如: COM3, COM4, /dev/ttyUSB0）')
    
    parser.add_argument('repeat_count', 
                        type=int,
                        help='重复测试次数（正整数）')
    
    parser.add_argument('--baudrate', 
                        type=int, 
                        default=230400,
                        help='波特率（默认: 230400）')
    
    parser.add_argument('--no-claw', 
                        action='store_true',
                        help='跳过夹爪控制（默认: 执行夹爪控制）')
    
    args = parser.parse_args()
    
    # 验证参数
    if args.repeat_count <= 0:
        parser.error("重复次数必须是正整数")
    
    return args


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_arguments()
    
    # 配置参数
    SERIAL_PORT = args.serial_port
    BAUDRATE = args.baudrate
    REPEAT_COUNT = args.repeat_count
    NO_CLAW = args.no_claw
    
    # 目标位置列表 [x, y, z, rx, ry, rz]
    TARGET_POSITIONS = [
        [200, 0, 100, 0, 0, 0],
        [200, 0, 100, 0, -90, 0],
        [200, -100, 100, 0, -90, 0],
        [200, 100, 200, 0, -90, 0],
        [400, 0, 100, 0, 0, 0],
        [500, 0, 100, 0, -90, 0],
        [400, -100, 100, 0, -90, 0],
        [400, 100, 200, 0, -90, 0],
        [400, 100, 100, 0, 0, 0],
        [300, 100, 300, 0, -90, 0],
        [300, 300, 300, 0, -90, 0],
        [300, -100, 0, 0, 0, 0],
    ]
    
    # 夹爪位置（度）
    CLAW_OPEN = 1.0      # 夹爪张开，SDK 默认使用归一化单位
    CLAW_CLOSE = 0.0     # 夹爪闭合
    
    print("=" * 60)
    print("机械臂随机移动压力测试")
    print("=" * 60)
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"重复次数: {REPEAT_COUNT}")
    print(f"目标位置数量: {len(TARGET_POSITIONS)}")
    print(f"夹爪控制: {'跳过' if NO_CLAW else '启用'}")
    print("=" * 60)
    
    # 创建机器人实例
    print("\n正在连接机器人...")
    robot = create_arm(SERIAL_PORT, baudrate=BAUDRATE)
    
    if not robot.is_connected:
        print("错误: 无法连接到机器人！")
        return -1
    
    try:
        # 等待机器人初始化完成
        print("等待机器人初始化...")
        time.sleep(5)
        
        # 复位机器人
        print("\n复位机器人...")
        if not ok(robot.reset()):
            print("错误: 复位失败！")
            return -1
        
        # 开始循环测试
        success_count = 0
        fail_count = 0
        last_target_pos = None  # 记录上一次选择的位置

        if NO_CLAW:
            robot.set_gripper(CLAW_OPEN, sync=False)
            time.sleep(5)
            robot.set_gripper(CLAW_CLOSE, sync=False)
            time.sleep(5)
        
        print(f"\n开始执行 {REPEAT_COUNT} 次随机移动和夹爪控制...")
        print("-" * 60)
        
        for i in range(REPEAT_COUNT):
            print(f"\n>>> 第 {i + 1}/{REPEAT_COUNT} 次测试")
            
            # 随机选择一个目标位置，确保不与上一次相同
            if len(TARGET_POSITIONS) > 1 and last_target_pos is not None:
                # 创建候选列表，排除上一次的位置
                available_positions = [pos for pos in TARGET_POSITIONS if pos != last_target_pos]
                target_pos = random.choice(available_positions)
            else:
                # 第一次选择或只有一个位置时，直接随机选择
                target_pos = random.choice(TARGET_POSITIONS)
            
            last_target_pos = target_pos  # 记录本次选择的位置
            x, y, z, rx, ry, rz = target_pos
            
            print(f"目标位置: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
            
            # 移动到目标位置
            print("执行移动...")
            if not ok(robot.move_ik(x, y, z, rx, ry, rz)):
                print("错误: 移动失败！")
                fail_count += 1
                continue

            # no claw control, skip this test
            if NO_CLAW:
                print("跳过夹爪控制...")
            continue
            
            # 控制夹爪打开到100度
            print(f"夹爪打开到 {CLAW_OPEN} 度...")
            if not ok(robot.set_gripper(CLAW_OPEN, sync=True)):
                print("错误: 夹爪打开失败！")
                fail_count += 1
                continue
            
            # 控制夹爪闭合到0度
            print(f"夹爪闭合到 {CLAW_CLOSE} 度...")
            if not ok(robot.set_gripper(CLAW_CLOSE, sync=True)):
                print("错误: 夹爪闭合失败！")
                fail_count += 1
                continue
            
            success_count += 1
            print(f"✓ 第 {i + 1} 次测试完成")
        
        # 打印测试结果统计
        print("\n" + "=" * 60)
        print("测试完成！")
        print("=" * 60)
        print(f"总测试次数: {REPEAT_COUNT}")
        print(f"成功次数: {success_count}")
        print(f"失败次数: {fail_count}")
        print(f"成功率: {success_count / REPEAT_COUNT * 100:.2f}%")
        print("=" * 60)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n用户中断测试！")
        return -1
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return -1
    finally:
        # 清理资源
        print("\n正在清理资源...")
        robot.close()
        print("完成！")


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
