#ifndef __ARM_ROBOT_H__
#define __ARM_ROBOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "inverse_kinematics.h"
#include <stdbool.h>
#include <stdint.h>
#include "usart.h"
#include "cmsis_os.h"
#include "arm_robot_config.h"
#include "fs_uart_servo.h"

#define ARM_ROBOT_VERSION_TOC               1U  // 机械臂版本：ToC
#define ARM_JOINTS_NUM                      7U  // 机械臂关节数，包含夹爪
#define ARM_JOINTS_NO_CLAW_NUM              (ARM_JOINTS_NUM - 1U)   // 机械臂关节数，不包含夹爪
#define ARM_CLAW_JOINT_ID                   (ARM_JOINTS_NUM - 1U)
#define ARM_FLOAT_ERROR                     -100000.0f

#define ARM_ALL_JOINTS_SYNC_MASK            (0xFFFFU)  // 1111 1111
#define ARM_CLAW_SYNC_MASK                  (1U << (ARM_JOINTS_NUM - 1U)) // 0100 0000
#define ARM_ALL_JOINTS_NO_CLAW_SYNC_MASK    (ARM_CLAW_SYNC_MASK - 1U) // 0011 1111
#define ARM_MAX_WAIT_TIME                    10000   // 最大等待时间，单位毫秒
#define ARM_SYNC_NO_CHANGE                  (-999.9f) // 关节同步控制时不改变该关节位置标志
#define ARM_FLOAT_TOLERANCE                 (1e-6f) // 浮点数比较精度
#define ARM_MAX_ERROR_COUNT                 10
#define ARM_DEFAULT_ACCEL_TIME              200 // 默认加速度时间，单位毫秒

// #define ARM_CALC_ACCEL_TIME(interval)       ((int)roundf(ARM_ACCEL_TIME_FACTOR * (interval)))  // 计算加速度时间
#define ARM_CALC_ACCEL_TIME(interval)       (ARM_DEFAULT_ACCEL_TIME)

enum ServoDirection {
    CW_DIRECTION = 0,
    CCW_DIRECTION = 1,
};

typedef struct {
    int id;
    enum ServoDirection direction;
} Servo;

typedef struct {
    int servo_id;
    float angle;
} ServoSyncData;


typedef struct {
    void (*init)(UART_HandleTypeDef *uart);
    int (*set_angle_interval)(int servo_id, float angle, int interval_ms);
    int (*set_angle_interval_acc)(int servo_id, float angle, int interval_ms, int acc_ms, int dec_ms);
    int (*set_angle_interval_velocity)(int servo_id, float angle, float velocity, int acc_ms, int dec_ms);
    int (*get_angle)(int servo_id, float* angle);
    int (*stop)(int servo_id, int mode, int power);
    void (*irq_handler)(void);
    int (*sync_move)(ServoSyncData *data, int data_num);
    int (*reset_angle)(int servo_id);
    int (*get_status)(int servo_id);
    int (*set_zero)(int servo_id);
    int (*monitor)(int servo_id, ServoData *servodata);
    int (*monitor_batch)(const uint8_t *servo_ids, int servo_count, ServoData *servodata);
} ArmServoOpt;

typedef struct {
    float init_angle;
    float angle;
    float min_angle;
    float max_angle;
    int servo_nums;     // 使用的舵机数量
    Servo *servos[2];   // 指向舵机的指针, 最多支持2个舵机驱动一个关节
    bool sync;          // 是否同步运动
} ArmJoint;

enum ArmSate {
    ARM_STATE_IDLE = 0,
    ARM_STATE_RECORDING,
    ARM_STATE_PLAYING,
    ARM_STATE_REMOTE,
    ARM_STATE_MASTER_SLAVE,
    ARM_STATE_NUM,
};

enum ArmServoMode {
    ARM_SERVO_MODE_INTERVAL = 0,
    ARM_SERVO_MODE_INTERVAL_ACC,
    ARM_SERVO_MODE_VELOCITY_ACC, 
};

enum ArmJointStopMode {
    ARM_JOINT_STOP_UNLOAD_MODE, // 卸载模式
    ARM_JOINT_STOP_LOCK_MODE,   // 锁定模式
    ARM_JOINT_STOP_DAMPING_MODE, // 阻尼模式
};

/* 保持4字节对齐 */
typedef struct {
    uint32_t checksum;
    uint32_t magic;
    /* 若需新增存放到FLASH的配置, 往后添加即可 */
    char name[ARM_NAME_MAX_LEN];
    float speed; // 关节速度 (degree/s)
    int set_zero_flag; // 是否设置零角度标志  
} ArmConfig;
#define STATIC_ASSERT(condition, name) typedef char name[(condition) ? 1 : -1]
STATIC_ASSERT((sizeof(ArmConfig) % 4 == 0), arm_config_must_be_4_byte_aligned);

typedef struct {
    ArmJoint joint[ARM_JOINTS_NUM]; // 关节数组，包含夹爪共7个关节
    ArmConfig cfg;
    osThreadId_t tasks_handle;      // 任务句柄
    enum ArmSate state;
    ArmServoOpt *servo_ops;
    int servo_error_flag;           // 舵机错误标志, 1表示舵机通讯错误
} ArmRobot;

typedef struct {
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
} ArmPose;

extern ArmRobot g_arm_robot;
extern Servo g_servos[ARM_SERVO_NUM + 1];

void arm_robot_init(void);
int arm_robot_reset(void);
bool arm_joint_check_angle_valid(int joint_id, float angle);
int arm_set_joint_angle_interval(int joint_id, float angle, int interval_ms);
int arm_set_joint_angle_interval_acc(int joint_id, float angle, int interval_ms, int acc_ms, int dec_ms);
int arm_set_joint_angle_velocity_acc(int joint_id, float angle, float velocity, int acc_ms, int dec_ms);

/**
     * 获取机械臂关节角度
     * @param joint_id 关节ID
     * @param angle 指向存储关节角度的指针
     * @return 0表示成功，非0表示失败
     */
int arm_get_joint_angle(int joint_id, float *angle);
int arm_joint_snapshot_read(bool claw_update, float joint_angles[ARM_JOINTS_NUM]);
void arm_print_status_frame(const float joint_angles[ARM_JOINTS_NUM]);

/**
     * 更新机械臂各关节角度
     * @param claw_update 是否更新夹爪角度, true: 更新, false: 不更新
     * @return 0表示成功，非0表示失败
     */
int arm_joint_angle_update(bool claw_update);
int arm_set_joint_stop(int joint_id, enum ArmJointStopMode mode, int param);
int arm_set_all_joint_stop(enum ArmJointStopMode mode, int param);
int arm_set_zero(void);
void arm_robot_request_stop(void);

/**
     * 设置机械臂关节同步标志
     * @param sync_mask 同步掩码，每一位对应一个关节，1表示同步，0表示不同步
     * @param sync 是否同步
     */
void arm_robot_set_sync(uint32_t sync_mask, bool sync);

/**
     * 等待同步完成
     * @param timeout_ms 超时时间，单位毫秒，<0表示等待10000ms
     * @return true表示同步完成，false表示超时未完成
     */
bool arm_wait_sync_finished(int timeout_ms);
int arm_joint_sync_move(float joint_angles[ARM_JOINTS_NUM]);
void arm_monitor(void);

#ifdef __cplusplus
}
#endif
#endif /* __ARM_ROBOT_H__ */
