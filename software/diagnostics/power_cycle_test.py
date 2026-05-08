#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    上下电测试脚本
    python power_cycle_test.py [机械臂串口] [继电器串口] [循环次数] --inner_loop [内循环次数]
    示例：python power_cycle_test.py COM3 COM4 50 --inner_loop 5
    注：--inner_loop参数为0，是直接的上下电循环测试，上电间隔4.6秒，不执行动作；内循环次数默认3次.
"""

"""
测试逻辑:
    （1）上电（初始状态断电）
    （2）启动等待4s（机械臂冷启动2s，bootloader和固件软启动2s，否则无法复位）
    （3）复位
    （4）执行动作，随机移动（完成后清理串口资源，否则再次上电串口报错）
    （5）复位
    （6）下电1s
    （7）返回（1）重复
"""

import sys
import os
import time
import random
import argparse
from pathlib import Path

from sdk_helpers import create_arm, ok

LEGACY_RELAY_SRC = Path(__file__).resolve().parents[1] / "zyarm_sdk"
if str(LEGACY_RELAY_SRC) not in sys.path:
    sys.path.insert(0, str(LEGACY_RELAY_SRC))

try:
    from URPT8B0 import URPT8B0
except ImportError:
    print("错误: 无法找到 'URPT8B0' 模块，请检查文件路径。")
    sys.exit(1)

# ==========================================
# 资源清理函数
# ==========================================
def safe_cleanup(robot):
    if not robot: return
    try:
        robot.close()
    except: pass

# ==========================================
# 串口解析
# ==========================================
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('robot_port', type=str, help="机械臂串口")
    parser.add_argument('relay_port', type=str, help="继电器串口")
    parser.add_argument('repeat_count', type=int, help="循环次数")
    
    # 添加 inner_loop 参数，默认值为 3
    parser.add_argument('--inner_loop', type=int, default=3, help="每次上电后的内循环动作次数 (默认: 3)")
    parser.add_argument('--baudrate', type=int, default=230400)
    parser.add_argument('--relay_baud', type=int, default=115200)
    return parser.parse_args()

# ==========================================
# 主程序
# ==========================================
def main():

    args = parse_args()
    
    # 随机动作的目标点列表
    TARGET_POSITIONS = [
        [200, 0, 100, 0, 0, 0],
        [200, 0, 100, 0, -90, 0],
        [200, -100, 100, 0, -90, 0],
        [200, 100, 200, 0, -90, 0],
        [400, 0, 100, 0, -90, 0],   # [400, 0, 100, 0, 0, 0]失败
        [500, 0, 100, 0, -90, 0],
        [400, -100, 100, 0, -90, 0],
        [400, 100, 200, 0, -90, 0],
        [400, 100, 100, 0, -90, 0], # [400, 100, 100, 0, 0, 0]失败
        [300, 100, 300, 0, -90, 0],
        [300, 300, 300, 0, -90, 0],
        [300, -100, 0, 0, 0, 0],
    ]
    
    # 夹爪参数
    CLAW_OPEN = 1.0
    CLAW_CLOSE = 0.0

    # 获取参数中的内循环次数
    INNER_LOOP_COUNT = args.inner_loop

    print("="*60)
    print(" 机械臂压力测试")
    print(" 逻辑: 上电 -> 等待 -> 复位 -> 随机动作 -> 复位 -> 下电 -> 下电1s")
    print(f" 机械臂: {args.robot_port} | 继电器: {args.relay_port}")
    print(f" 总周期: {args.repeat_count} | 内循环: {INNER_LOOP_COUNT} 次/周期")
    print("="*60)

    try:
        # 继电器控制类 URPT8B0
        relay = URPT8B0(args.relay_port, args.relay_baud)
    except Exception as e:
        print(f"无法连接继电器: {e}")
        return -1

    # 初始化变量
    last_target_pos = None
    success_cycles = 0
    fail_cycles = 0

    try:
        for i in range(args.repeat_count):
            print(f"\n======== [ 周期 {i+1} / {args.repeat_count} ] ========")
            robot = None
            cycle_failed = False # 标记本周期是否失败

            try:
                # ---------------------------------------------------
                # 1. 上电
                # ---------------------------------------------------
                print(">> [1] 上电...")
                relay.power_on()
                
                # ---------------------------------------------------
                # 2. 启动等待
                # ---------------------------------------------------
                print(">> [2] 冷启动等待2秒...")
                time.sleep(2)
                print(">>     冷启动等待结束...")

                # 连接串口
                print(">> [Connect] 连接机械臂...")
                try:
                    robot = create_arm(args.robot_port, args.baudrate)
                except Exception as e:
                    print(f"   !! 连接失败: {e}")
                    fail_cycles += 1
                    continue

                if not robot.is_connected:
                    print("   !! 连接对象创建失败")
                    fail_cycles += 1
                    continue

                # 软启动等待
                print(">> 等待 Bootloader 结束和固件初始化完成...")
                time.sleep(2)
                print(">> 等待结束...")

                # ---------------------------------------------------
                # 3. 复位
                # ---------------------------------------------------
                print(">> [3] 复位...")
                if ok(robot.reset()):
                    print("   -> 复位成功")
                else:
                    print("   -> 复位失败/超时")
                    cycle_failed = True # 复位失败则不进入内循环
                
                # ---------------------------------------------------
                # 4. 执行动作序列
                # ---------------------------------------------------

                if not cycle_failed:
                    print(f">> [4] 随机动作开始 (循环 {INNER_LOOP_COUNT} 次)...")
                    
                    # 加入内循环次数，每次上电执行几次随机动作
                    for j in range(INNER_LOOP_COUNT):
                        if cycle_failed: break # 如果动作失败，跳出内循环
                        
                        print(f"\n   --- [Inner Loop {j+1}/{INNER_LOOP_COUNT}] ---")

                        # === [ 随机移动开始 ] ===

                        # A. 随机选择位置 (不重复)
                        if len(TARGET_POSITIONS) > 1 and last_target_pos is not None:
                            available_positions = [pos for pos in TARGET_POSITIONS if pos != last_target_pos]
                            target_pos = random.choice(available_positions)
                        else:
                            target_pos = random.choice(TARGET_POSITIONS)
                        
                        last_target_pos = target_pos
                        x, y, z, rx, ry, rz = target_pos
                        
                        time.sleep(1) # 可选，稍微减少一点等待让内循环快一点
                        print(f"   -> 目标位置: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")

                        # B. 执行移动
                        if not ok(robot.move_ik(x, y, z, rx, ry, rz)):
                            print("   !! 错误: 移动失败！")
                            cycle_failed = True
                        else:
                            print("   -> 移动成功")
                            
                            # C. 只有移动成功才执行夹爪
                            if not cycle_failed:
                                print(f"   -> 夹爪打开到 {CLAW_OPEN} ...")
                                if not ok(robot.set_gripper(CLAW_OPEN, sync=True)):
                                    print("   !! 错误: 夹爪打开失败！")
                                    cycle_failed = True
                                
                                if not cycle_failed:
                                    print(f"   -> 夹爪闭合到 {CLAW_CLOSE} ...")
                                    if not ok(robot.set_gripper(CLAW_CLOSE, sync=True)):
                                        print("   !! 错误: 夹爪闭合失败！")
                                        cycle_failed = True
                        
                        # 内循环单次动作结束后的等待
                        time.sleep(0.5)

                    print(">>     随机动作结束...")

                # ---------------------------------------------------
                # 5. 复位
                # ---------------------------------------------------
                print(">> [5] 复位...")
                if ok(robot.reset()):
                    print("   -> 复位成功")
                else:
                    print("   -> 复位失败/超时")

                # === [ 随机移动结束 ] ===

                time.sleep(0.5)

                # 统计结果
                if not cycle_failed:
                    print(f"✓ 周期 {i + 1} 成功")
                    success_cycles += 1
                else:
                    print(f"✗ 周期 {i + 1} 失败")
                    fail_cycles += 1

            except Exception as e:
                print(f"!! 运行异常: {e}")
                fail_cycles += 1
            
            finally:
                # 资源清理
                if robot:
                    safe_cleanup(robot)
                    del robot
                
                time.sleep(0.1) 

                # ---------------------------------------------------
                # 6. 下电
                # ---------------------------------------------------
                print(">> [6] 下电...")
                try: relay.power_off()
                except: pass

                # ---------------------------------------------------
                # 7. 等待 1S
                # ---------------------------------------------------
                print(">> [7] 等待 1秒...")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断测试")
    finally:
        try: relay.close()
        except: pass
        print(f"测试结束: {success_cycles} 成功 / {fail_cycles} 失败")
        if (success_cycles + fail_cycles) > 0:
            print(f"成功率：{success_cycles / (success_cycles + fail_cycles) * 100:.2f}%")

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
