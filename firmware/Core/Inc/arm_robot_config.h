#ifndef __ARM_ROBOT_CONFIG_H
#define __ARM_ROBOT_CONFIG_H

#include "stm32f407xx.h"

#define ARM_ROBOT_SERVO_USE_FASHION_START       1
#define ARM_SERVO_NUM                           9       // 舵机总数，包括夹爪
#define ARM_JOINT_SPEED_DEFAULT                 30.0f   // 默认关节速度 degree/s
#define ARM_NAME_MAX_LEN                        32      // 全局名称最大长度

#define ARM_SERVO_UART                          (&huart6)
#define ARM_ANGLE_ERROR		                    (3.0f)  // 角度允许误差
#define ARM_ACCEL_TIME_FACTOR                   (0.1f)  // 加速度时间因子
#define ARM_MONITOR_FAST_PERIOD_MS              100U    // 双舵机快速监控基础周期
#define ARM_MONITOR_HEALTH_PERIOD_MS            500U    // 普通健康监控周期
#define ARM_DUAL_SERVO_HOLD_POWER               0       // power=0 表示使用舵机功率保护上限
#define ARM_DUAL_SERVO_HOLD_LOG_ENABLE          0       // 是否打印双舵机锁力观测日志
#define ARM_DUAL_SERVO_STAGNANT_WINDOW_MS       1000U   // 执行中角度停滞观察窗口
#define ARM_DUAL_SERVO_STAGNANT_ANGLE_DEG       0.5f    // 执行中角度停滞阈值
#define ARM_DUAL_SERVO_STREAM_IDLE_MS           500U    // 快速/流式停发后进入静止观察的空闲时间
#define ARM_DUAL_SERVO_STREAM_STILL_WINDOW_MS   1000U   // 快速/流式停发后的静止观察窗口
#define ARM_DUAL_SERVO_STREAM_STILL_ANGLE_DEG   0.5f    // 快速/流式停发后的静止角度阈值
#define ARM_MONITOR_WAKE_FLAG                   0x00000001U

#endif
