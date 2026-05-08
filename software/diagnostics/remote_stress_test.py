#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
遥操压力测试
按照指定频率遥操机械臂，同时实时显示发送频率和接收关节信息的频率
"""

import sys
import os
import time
import threading
from collections import deque

from sdk_helpers import CommandId, create_arm, ok


class RemoteStressTest:
    def __init__(self, serial_device, baudrate=230400, target_freq=50):
        """
        初始化遥操压力测试
        
        Args:
            serial_device: 串口设备
            baudrate: 波特率
            target_freq: 目标遥操频率 (Hz)
        """
        self.serial_device = serial_device
        self.baudrate = baudrate
        self.target_freq = target_freq
        self.robot = None
        
        self.running = False
        self.send_thread = None
        self.stats_thread = None
        
        self.send_count = 0
        self.recv_joint_count = 0
        self.send_timestamps = deque(maxlen=200)
        self.recv_joint_timestamps = deque(maxlen=200)
        
        self.actual_send_freq = 0.0
        self.actual_recv_freq = 0.0
        self.last_joints = None
        
        self.lock = threading.Lock()
        
    def connect(self):
        """连接机械臂"""
        print(f"Connecting to robot via {self.serial_device} at {self.baudrate} baud...")
        self.robot = create_arm(self.serial_device, baudrate=self.baudrate, timeout_s=0.02)
        if not self.robot.is_connected:
            print("Failed to connect to robot!")
            return False
        print("Robot connected successfully!")
        time.sleep(3)
        return True
    
    def _send_loop(self):
        """遥操发送循环"""
        period = 1.0 / self.target_freq
        next_time = time.perf_counter()
        
        x = 0.0
        y = 0.0
        z = 0.0
        direction = 1
        step = 1
        
        print(f"[SEND] Starting send loop at target frequency: {self.target_freq} Hz")
        
        while self.running:
            start_time = time.perf_counter()
            
            x += direction * step
            if x > 200:
                direction = -1
            elif x < 0:
                direction = 1
            
            ret = self.robot.remote_control(x, y, z, 0, 0, 0, 0)
            
            if ok(ret):
                with self.lock:
                    self.send_count += 1
                    self.send_timestamps.append(time.perf_counter())
            
            elapsed = time.perf_counter() - start_time
            sleep_time = period - elapsed
            
            if sleep_time > 0:
                next_time += period
                current_time = time.perf_counter()
                if next_time > current_time:
                    time.sleep(next_time - current_time)
                else:
                    next_time = current_time + period
            else:
                next_time = time.perf_counter() + period
    
    def _stats_loop(self):
        """统计显示循环"""
        last_send_count = 0
        last_recv_count = 0
        last_time = time.perf_counter()
        
        print("[STATS] Starting statistics display loop")
        print("-" * 80)
        print(f"{'Time(s)':<10} {'SendCnt':<10} {'SendFreq':<12} {'RecvCnt':<10} {'RecvFreq':<12} {'Joints Info'}")
        print("-" * 80)
        
        while self.running:
            time.sleep(0.5)
            
            current_time = time.perf_counter()
            
            with self.lock:
                current_send_count = self.send_count
                current_recv_count = self.recv_joint_count
                
                if len(self.send_timestamps) >= 2:
                    time_span = self.send_timestamps[-1] - self.send_timestamps[0]
                    count = len(self.send_timestamps)
                    if time_span > 0.1:
                        self.actual_send_freq = (count - 1) / time_span
                
                if len(self.recv_joint_timestamps) >= 2:
                    time_span = self.recv_joint_timestamps[-1] - self.recv_joint_timestamps[0]
                    count = len(self.recv_joint_timestamps)
                    if time_span > 0.1:
                        self.actual_recv_freq = (count - 1) / time_span
            
            total_time = current_time - self.start_time if hasattr(self, 'start_time') else 0
            
            joints_str = ""
            with self.lock:
                if self.last_joints:
                    joints_str = f"J0:{self.last_joints['J0']:.1f} J1:{self.last_joints['J1']:.1f} J2:{self.last_joints['J2']:.1f} J3:{self.last_joints['J3']:.1f} J4:{self.last_joints['J4']:.1f} J5:{self.last_joints['J5']:.1f}"
            
            print(f"{total_time:<10.1f} {current_send_count:<10} {self.actual_send_freq:<12.2f} {current_recv_count:<10} {self.actual_recv_freq:<12.2f} {joints_str}")
            
            last_send_count = current_send_count
            last_recv_count = current_recv_count
            last_time = current_time
    
    def _monitor_serial(self):
        """监控串口返回数据，解析关节信息"""
        print("[MONITOR] Starting serial monitor for joint info")
        
        if not self.robot:
            print("[MONITOR] Error: No robot available")
            return

        last_sequence = -1
        while self.running:
            try:
                state = self.robot.get_latest_state()
                if state is not None and state.sequence != last_sequence:
                    last_sequence = state.sequence
                    with self.lock:
                        self.recv_joint_count += 1
                        self.recv_joint_timestamps.append(time.perf_counter())
                        self.last_joints = {
                            'J0': state.positions[0],
                            'J1': state.positions[1],
                            'J2': state.positions[2],
                            'J3': state.positions[3],
                            'J4': state.positions[4],
                            'J5': state.positions[5],
                            'CLAW': state.positions[6],
                        }
                time.sleep(0.001)
            except Exception as e:
                print(f"[MONITOR] Error: {e}")
                time.sleep(0.01)
    
    def run(self, duration=30):
        """
        运行遥操压力测试
        
        Args:
            duration: 测试持续时间（秒）
        """
        if not self.connect():
            return
        
        print(f"\n{'='*60}")
        print(f"Remote Stress Test Configuration:")
        print(f"  Target Frequency: {self.target_freq} Hz")
        print(f"  Test Duration: {duration} seconds")
        print(f"  Baudrate: {self.baudrate}")
        print(f"{'='*60}\n")
        
        print("Enabling remote mode...")
        if not ok(self.robot.set_remote_mode(True)):
            print("Failed to enable remote mode!")
            return
        print("Remote mode enabled.\n")
        
        print("Resetting robot in remote mode...")
        if not ok(self.robot.remote_reset()):
            print("Failed to reset robot!")
            return
        print("Robot reset completed.\n")
        self.robot.send_command(CommandId.STATUS_REPORT, [1.0, float(self.target_freq)], wait_ack=True)
        
        time.sleep(1)
        
        self.running = True
        self.start_time = time.perf_counter()
        
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.stats_thread = threading.Thread(target=self._stats_loop, daemon=True)
        self.monitor_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        
        self.send_thread.start()
        self.stats_thread.start()
        self.monitor_thread.start()
        
        print(f"Test running for {duration} seconds...\n")
        
        time.sleep(duration)
        
        print("\nStopping test...")
        self.running = False
        
        self.send_thread.join(timeout=2)
        self.stats_thread.join(timeout=2)
        self.monitor_thread.join(timeout=2)
        
        print("\nDisabling remote mode...")
        self.robot.set_remote_mode(False)
        self.robot.close()
        
        print(f"\n{'='*60}")
        print("Test Summary:")
        print(f"  Total Commands Sent: {self.send_count}")
        print(f"  Total Joint Info Received: {self.recv_joint_count}")
        print(f"  Average Send Frequency: {self.actual_send_freq:.2f} Hz")
        print(f"  Average Receive Frequency: {self.actual_recv_freq:.2f} Hz")
        print(f"{'='*60}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Remote Control Stress Test for Robot Arm')
    parser.add_argument('--port', '-p', type=str, default='COM3', 
                        help='Serial port (default: COM3)')
    parser.add_argument('--baudrate', '-b', type=int, default=230400,
                        help='Baudrate (default: 230400)')
    parser.add_argument('--freq', '-f', type=int, default=50,
                        help='Target frequency in Hz (default: 50)')
    parser.add_argument('--duration', '-d', type=int, default=30,
                        help='Test duration in seconds (default: 30)')
    
    args = parser.parse_args()
    
    test = RemoteStressTest(
        serial_device=args.port,
        baudrate=args.baudrate,
        target_freq=args.freq
    )

    test.run(duration=args.duration)


if __name__ == "__main__":
    main()
