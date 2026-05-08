#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
循环通过串口发送help指令给下位机，实时打印下位机固件的help反馈信息和ACK信息

使用方法:
    python help_ack_print_test.py <串口号> <重复次数>

示例:
    python help_ack_print_test.py COM3 10
    python help_ack_print_test.py COM5 50
"""

import sys
import time
import argparse
import serial


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='机械臂发送help指令并实时打印ACK反馈测试脚本',
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
                        default=115200,
                        help='波特率（默认: 115200）')
    
    parser.add_argument('--wait-time',
                        type=float,
                        default=3.0,
                        help='每次发送后等待响应的时间（秒，默认: 3.0）')
    
    args = parser.parse_args()
    
    # 验证参数
    if args.repeat_count <= 0:
        parser.error("重复次数必须是正整数")
    
    if args.wait_time <= 0:
        parser.error("等待时间必须是正数")
    
    return args


def print_serial_response(robot, wait_time=3.0):
    """
    实时打印串口响应数据
    
    Args:
        robot: pyserial 串口对象
        wait_time: 等待响应的时间（秒）
    """
    # 等待响应
    start_time = time.time()
    buffer = ""
    
    print("下位机反馈:")
    print("-" * 60)
    
    while time.time() - start_time < wait_time:
        try:
            waiting = getattr(robot, "in_waiting", 0)
            new_data = robot.read(waiting or 1).decode("utf-8", errors="ignore")
            
            if new_data:
                buffer += new_data
                
                # 检查是否有完整的行
                lines = buffer.split('\n')
                for line in lines[:-1]:  # 最后一个可能是不完整的
                    line = line.strip()
                    if line:  # 只打印非空行
                        print(line)
                
                # 保留最后一个不完整的行
                buffer = lines[-1]
            
            # 短暂延时
            time.sleep(0.05)
            
        except Exception as e:
            print(f"读取响应时出错: {e}")
            break
    
    # 打印剩余的缓冲区内容
    if buffer.strip():
        print(buffer.strip())
    
    print("-" * 60)


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_arguments()
    
    # 配置参数
    SERIAL_PORT = args.serial_port
    BAUDRATE = args.baudrate
    REPEAT_COUNT = args.repeat_count
    WAIT_TIME = args.wait_time
    
    print("=" * 60)
    print("机械臂help指令ACK实时打印测试")
    print("=" * 60)
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"重复次数: {REPEAT_COUNT}")
    print(f"等待响应时间: {WAIT_TIME}秒")
    print("=" * 60)
    
    # 创建机器人实例
    print("\n正在连接机器人...")
    robot = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=0.05,
        write_timeout=0.5,
    )

    if not robot.is_open:
        print("错误: 无法连接到机器人！")
        return -1

    try:
        # 等待机器人初始化完成
        print("等待机器人初始化...")
        time.sleep(5)
        
        print(f"\n开始执行 {REPEAT_COUNT} 次help指令测试...")
        print("=" * 60)
        
        success_count = 0
        fail_count = 0
        
        # 循环发送help指令并实时打印反馈
        for i in range(REPEAT_COUNT):
            print(f"\n>>> 第 {i + 1}/{REPEAT_COUNT} 次测试: 发送help指令")
            
            try:
                # 发送help指令
                command = "help\n"
                robot.write(command.encode('utf-8'))
                robot.flush()
                print(f"已发送指令: {command.strip()}")
                print(f"发送时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                
                # 实时打印串口响应
                print_serial_response(robot, WAIT_TIME)
                
                success_count += 1
                print(f"✓ 第 {i + 1} 次测试完成")
                
                # 间隔时间，避免对下位机的频繁请求
                if i < REPEAT_COUNT - 1:
                    print(f"\n等待 1 秒后继续下一次测试...")
                    time.sleep(1)
                    
            except Exception as e:
                print(f"错误: 发送help指令失败 - {e}")
                fail_count += 1
                continue
        
        # 打印测试结果统计
        print("\n" + "=" * 60)
        print("测试完成！")
        print("=" * 60)
        print(f"总测试次数: {REPEAT_COUNT}")
        print(f"成功次数: {success_count}")
        print(f"失败次数: {fail_count}")
        if REPEAT_COUNT > 0:
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
