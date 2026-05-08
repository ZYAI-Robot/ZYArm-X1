#!/usr/bin/env python3

import serial
import time
import argparse


def hardware_reset(serial_port: str, baudrate: int = 115200, timeout: float = 1.0):
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            timeout=timeout
        )
        
        ser.dtr = False
        ser.rts = True
        print(f"已设置 DTR=FALSE, RTS=TRUE")
        
        time.sleep(1)
        
        ser.rts = False
        print(f"已设置 RTS=FALSE (复位完成)")
        
        ser.close()
        print(f"串口 {serial_port} 已关闭")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        raise
    except Exception as e:
        print(f"错误: {e}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="机械臂硬件复位脚本")
    parser.add_argument(
        "-p", "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="串口地址 (默认: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baudrate",
        type=int,
        default=512000,
        help="波特率 (默认: 512000)"
    )
    
    args = parser.parse_args()
    
    hardware_reset(args.port, args.baudrate)
    time.sleep(2)
    print("复位完成，退出程序")
