#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Move Record Test Script

这是一个用于测试机械臂运动和记录动作播放功能的压力测试脚本。主要功能包括：

1. 连接到指定串口上的机械臂设备
2. 循环执行以下操作（次数由参数指定）：
   - 将机械臂移动到预设的一系列目标位置
   - 在每个位置控制夹爪进行开合操作
   - 播放预先记录的动作序列
3. 统计测试结果（成功/失败次数及成功率）

该脚本可用于验证机械臂在长时间运行和重复操作下的稳定性和可靠性，检测潜在的问题或故障点。

使用方法:
  python move_record_test.py [串口号] [重复次数]

示例:
  python move_record_test.py COM3 10    # 使用COM3串口，重复测试10次
  python move_record_test.py /dev/ttyUSB0 50  # 使用Linux串口，重复测试50次
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
    
    # 目标位置列表 [x, y, z, rx, ry, rz]
    TARGET_POSITIONS = [
        [200, 0, 100, 0, 0, 0],
        [200, 0, 100, 0, -90, 0],
        [200, -100, 100, 0, -90, 0],
        [200, 100, 200, 0, -90, 0]
    ]

    # 需播放的动作ID
    RECORD_ID = [1, 2]
    
    # 夹爪位置（度）
    CLAW_OPEN = 1.0      # 夹爪张开，SDK 默认使用归一化单位
    CLAW_CLOSE = 0.0     # 夹爪闭合
    
    print("=" * 60)
    print("机械臂移动与动作播放测试")
    print("=" * 60)
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"重复次数: {REPEAT_COUNT}")
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
        
        print(f"\n开始执行 {REPEAT_COUNT} 次随机移动和夹爪控制...")
        print("-" * 60)
        
        for i in range(REPEAT_COUNT):
            print(f"\n>>> 第 {i + 1}/{REPEAT_COUNT} 次测试")
            
            for j in range(len(TARGET_POSITIONS)):
                target_pos = TARGET_POSITIONS[j]
                x, y, z, rx, ry, rz = target_pos
                
                print(f"目标位置: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
                
                # 移动到目标位置
                print("执行移动...")
                if not ok(robot.move_ik(x, y, z, rx, ry, rz)):
                    print("错误: 移动失败！")
                    fail_count += 1
                    continue
                
                # 控制夹爪打开到100度
                print(f"夹爪打开到 {CLAW_OPEN:.1f} ...")
                if not ok(robot.set_gripper(CLAW_OPEN, sync=True)):
                    print("错误: 夹爪打开失败！")
                    fail_count += 1
                    continue
                
                # 控制夹爪闭合到0度
                print(f"夹爪闭合到 {CLAW_CLOSE:.1f} ...")
                if not ok(robot.set_gripper(CLAW_CLOSE, sync=True)):
                    print("错误: 夹爪闭合失败！")
                    fail_count += 1
                    continue
            
            for j in range(len(RECORD_ID)):
                record_id = RECORD_ID[j]
                print(f"播放动作 {record_id}...")
                if not ok(robot.play_record(record_id)):
                    print("错误: 播放动作失败！")
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
