#ifndef __ARM_MASTER_SLAVE_H__
#define __ARM_MASTER_SLAVE_H__

#include "arm_robot.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_MASTER_SLAVE_DEFAULT_PERIOD_MS 10
#define ARM_MASTER_SLAVE_DEFAULT_FREQ_HZ 50
#define ARM_MASTER_SLAVE_DEFAULT_LPF_COEFFICIENT 0.15f
#define ARM_MASTER_SLAVE_MAX_ANGLE_CHANGE 5.0f

/**
 * @brief 主从跟随状态
 */
typedef enum {
    ARM_MASTER_SLAVE_STATUS_IDLE = 0, 
    ARM_MASTER_SLAVE_STATUS_RUNNING = 1,
    ARM_MASTER_SLAVE_STATUS_STOP = 2, 
} ArmMasterSlaveStatus;

/**
 * @brief 机械臂角色
 */
typedef enum {
    ARM_ROLE_MASTER = 1, 
    ARM_ROLE_SLAVE = 2, 
} ArmRole;

/**
 * @brief 主从跟随管理器
 */
typedef struct {
    ArmMasterSlaveStatus status;
    osThreadId_t task_handle;
    uint32_t sample_period_ms;
    ArmRole role;
    float lpf_coefficient;
} ArmMasterSlaveManager;

/**
 * @brief 启动主从跟随功能
 * @param role 机械臂角色，ARM_ROLE_MASTER或ARM_ROLE_SLAVE
 * @return 0表示成功，非0表示失败
 */
int arm_master_slave_start(int role);

/**
 * @brief 设置主从模式采样频率
 * @param freq_hz 采样频率，单位 Hz
 * @return 0 表示成功，非 0 表示失败
 */
int arm_master_slave_set_frequency_hz(uint32_t freq_hz);

/**
 * @brief 停止主从跟随功能
 * @return 0表示成功，非0表示失败
 */
int arm_master_slave_stop(void);

/**
 * @brief 设置从臂接收的关节角数据
 * @param angles 关节角数组，长度为ARM_JOINTS_NUM
 */
void arm_slave_set_angles(float angles[ARM_JOINTS_NUM]);

/**
 * @brief 读取主臂关节角度
 * @param angles 关节角数组，长度为ARM_JOINTS_NUM
 * @return 0表示成功，非0表示失败
 */
int arm_master_read_joint_angles(float angles[ARM_JOINTS_NUM]);

/**
 * @brief 获取主从跟随状态
 * @return 主从跟随状态
 */
ArmMasterSlaveStatus arm_master_slave_get_status(void);

/**
 * @brief 获取当前主从模式角色
 * @return 当前主从角色
 */
ArmRole arm_master_slave_get_role(void);

/**
 * @brief 设置主从模式低通滤波系数
 * @param alpha 滤波系数，范围[0.0, 1.0]
 */
void arm_master_slave_set_lpf_coefficient(float alpha);

/**
 * @brief 获取主从模式低通滤波系数
 * @return 当前滤波系数
 */
float arm_master_slave_get_lpf_coefficient(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_MASTER_SLAVE_H__ */
