#ifndef __ARM_ROBOT_CONFIG_H
#define __ARM_ROBOT_CONFIG_H

#include "stm32f407xx.h"

#define ARM_ROBOT_SERVO_USE_FASHION_START       1
#define ARM_SERVO_NUM                           9       // 舵机总数，包括夹爪
#define ARM_JOINT_SPEED_DEFAULT                 50.0f   // 默认关节速度 degree/s
#define ARM_NAME_MAX_LEN                        32      // 全局名称最大长度

#define ARM_SERVO_UART                          (&huart6)
#define ARM_ANGLE_ERROR		                    (3.0f)  // 角度允许误差
#define ARM_ACCEL_TIME_FACTOR                   (0.1f)  // 加速度时间因子

#endif
