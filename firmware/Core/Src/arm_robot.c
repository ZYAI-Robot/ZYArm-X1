#include "arm_robot.h"
#include "arm_shell.h"
#include "arm_flash.h"
#include "arm_robot_kinematics.h"
#include "w25q128.h"
#include "w2812.h"
#include <stdio.h>
#include <string.h>

#define ARM_ROBOT_LOG_TAG "ARM_ROBOT"
#define ARM_JOINT_SNAPSHOT_MAX_ATTEMPTS 2U
#define ARM_MONITOR_RECOVERY_DELAY_MS 20U
#define ARM_MONITOR_TEMPERATURE_WARN_C 40
#define ARM_MONITOR_TEMPERATURE_FILTER_CYCLES (120U * 2U)

/**
 * @brief 配置区域
 * @details 根据实际硬件配置以下参数
 */
// DH表
const dh_param_t g_robot_dh_params[6] = { 
    {0, 0, 0, 0},
    {37, -M_PI_2, 0, -M_PI_2},
    {270, 0, 0, M_PI_2},
#if defined(ARM_ROBOT_VERSION_TOC) && (ARM_ROBOT_VERSION_TOC == 1U)
    {39.91, M_PI_2, -190, 0},
#else
    {39.91, M_PI_2, -230, 0},
#endif
    {0, -M_PI_2, 0, 0},
    {0, M_PI_2, 0, 0} 
};

const float g_robot_claw_length = 186.8f; // 夹爪长度

// 舵机实例
Servo g_servos[ARM_SERVO_NUM + 1] = {
    {0}, // 保留，index与舵机ID对应
    {.id = 1, .direction = CW_DIRECTION},   // J1
    {.id = 2, .direction = CW_DIRECTION},   // J2-right
    {.id = 3, .direction = CCW_DIRECTION},  // J2-left
    {.id = 4, .direction = CCW_DIRECTION},  // J3-right
    {.id = 5, .direction = CW_DIRECTION},   // J3-left
    {.id = 6, .direction = CCW_DIRECTION},  // J4
    {.id = 7, .direction = CW_DIRECTION},   // J5
    {.id = 8, .direction = CCW_DIRECTION},  // J6
    {.id = 9, .direction = CCW_DIRECTION},  // Claw
};

static const uint8_t g_arm_primary_servo_ids[ARM_JOINTS_NUM] = {1U, 2U, 4U, 6U, 7U, 8U, 9U};
static const uint8_t g_arm_all_servo_ids[ARM_SERVO_NUM] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U};

// 机械臂实例
ArmRobot g_arm_robot = {   
    // 关节初始化配置
    .joint = {
        { // J0
            .init_angle = 0, 
            .min_angle = -170, 
            .max_angle = 170,
            .servo_nums = 1,
            .servos = {&g_servos[1]},
            .sync = true
        }, 
        { // J1
            .init_angle = -180, 
            .min_angle = -180, 
            .max_angle = -10,
            .servo_nums = 2,
            .servos = {&g_servos[2], &g_servos[3]},
            .sync = true
        },
        { // J2
            .init_angle = 90, 
            .min_angle = -80, 
            .max_angle = 90,
            .servo_nums = 2, 
            .servos = {&g_servos[4], &g_servos[5]},
            .sync = true
        },
        { // J3
            .init_angle = 0, 
            .min_angle = -170, 
            .max_angle = 170,
            .servo_nums = 1,
            .servos = {&g_servos[6]},
            .sync = true
        },  
        { // J4 
            .init_angle = 0, 
            .min_angle = -100, 
            .max_angle = 100, 
            .servo_nums = 1,
            .servos = {&g_servos[7]},
            .sync = true
        },    
        { // J5
            .init_angle = 0, 
            .min_angle = -170, 
            .max_angle = 170,
            .servo_nums = 1,
            .servos = {&g_servos[8]}, 
            .sync = true
        },  
        { // Claw/J6
            .init_angle = 0, 
            .min_angle = 0, 
            .max_angle = 100,
            .servo_nums = 1,
            .servos = {&g_servos[9]}, 
            .sync = true
        }
    },

    .cfg = {
        .name = "ZYArmRobot",
        .speed = ARM_JOINT_SPEED_DEFAULT,
    },
};

/* ============================ 配置结束 ============================ */

static void arm_servo_init(void)
{
#if defined(ARM_ROBOT_SERVO_USE_FASHION_START) && (ARM_ROBOT_SERVO_USE_FASHION_START == 1)
    extern ArmServoOpt g_fashion_start_servo_ops;
    g_arm_robot.servo_ops = &g_fashion_start_servo_ops;
#else
    #error "No servo driver defined, please define one in arm_robot_config.h"
#endif

    // 校验前面的配置是否正确
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        ArmJoint *joint = &g_arm_robot.joint[i];
        for (int j = 0; j < joint->servo_nums; j++) {
            if (joint->servos[j] == NULL) {
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Joint %d servo[%d] is NULL, please configure the servo information correctly in arm_servo_init.\n",
                    i,
                    j
                );
                return;
            }
        }
    }

    g_arm_robot.servo_ops->init(ARM_SERVO_UART);
}

static void arm_first_user_config(void)
{
    uint8_t input_char[10] = {0};
    
    // 提示用户手动调整机械臂到零位
    ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to load robot config from Flash\n");
    safe_printf("Please manually adjust the robotic arm to zero position and enter Y to continue.\n");
    arm_set_all_joint_stop(ARM_JOINT_STOP_DAMPING_MODE, 800);
    while (1) {
        if (HAL_UART_Receive(&huart1, &(input_char[0]), 1, HAL_MAX_DELAY) == HAL_OK) {
            if (!((input_char[0] == 'Y') || (input_char[0] == 'y'))) {
                ARM_LOGW_TAG(ARM_ROBOT_LOG_TAG, "Invalid input, please enter Y to continue\n");
                continue;
            }

            safe_printf("Setting zero position...\n");
            if (arm_set_zero() == 0) {
                safe_printf("Zero position setting completed successfully.\n");
                break;
            } else {
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Failed to set zero position, please try again after powering off and on\n"
                );
            }
        }
    }
    HAL_UART_Receive(&huart1, &(input_char[0]), 10, 10); // 刷新串口缓冲区
}

void arm_robot_init(void)
{
    int ret = 0; 
    W2812_Init(4);
    vTaskDelay(1000); // 等待电压稳定

    arm_servo_init();

    ret = W25Q128_Init(100);
    if (ret != HAL_OK) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to initialize W25Q128, ret=%d\n", ret);
        return;
    }

    ret = arm_flash_config_load();
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to load robot config from flash, ret=%d\n", ret);
        arm_first_user_config();
    }

    ret = arm_shell_init();
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to initialize arm shell module, ret=%d\n", ret);
        return;
    }

    vTaskDelay(100);
    arm_robot_reset();
}

int arm_set_all_joint_stop(enum ArmJointStopMode mode, int param)
{
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        int ret = arm_set_joint_stop(i, mode, param);
        if (ret != 0) {
            ARM_LOGE_TAG(
                ARM_ROBOT_LOG_TAG,
                "Failed to set joint %d stop, mode=%d, param=%d, ret=%d\n",
                i,
                mode,
                param,
                ret
            );
            return ret;
        }
    }
    return 0;
}

int arm_robot_reset(void)
{
    float interval_ms;
    float target_angles[ARM_JOINTS_NUM];
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        target_angles[i] = g_arm_robot.joint[i].init_angle;
    }

    int ret = arm_joint_angle_update(true);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Update joint angles failed\n");
        return -1;
    }

    arm_cal_interval_with_angle_diff(target_angles, &interval_ms);

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        ret = arm_set_joint_angle_interval_acc(i, g_arm_robot.joint[i].init_angle, 
                        (int)roundf(interval_ms), ARM_DEFAULT_ACCEL_TIME, ARM_DEFAULT_ACCEL_TIME);
        if (ret != 0) {
            ARM_LOGE_TAG(
                ARM_ROBOT_LOG_TAG,
                "Failed to reset joint %d to angle %.2f, ret=%d\n",
                i,
                g_arm_robot.joint[i].init_angle,
                ret
            );
            return ret;
        }
    }
    return 0;
}

bool arm_joint_check_angle_valid(int joint_id, float angle)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid joint_id %d\n", joint_id);
        return false;
    }

    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    if ((angle < joint->min_angle) || (angle > joint->max_angle)) {
        ARM_LOGE_TAG(
            ARM_ROBOT_LOG_TAG,
            "Joint %d angle %.2f out of range [%.2f, %.2f]\n",
            joint_id,
            angle,
            joint->min_angle,
            joint->max_angle
        );
        return false;
    }
    return true;
}

static int arm_set_joint_angle_common(int joint_id, float angle, enum ArmServoMode mode, 
                                        int interval_ms, float velocity, int acc_ms, int dec_ms)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid joint_id %d\n", joint_id);
        return -1;
    }

    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    // 与初始角度的差值
    float angle_diff = angle - joint->init_angle;

    int ret = 0;
    for (int i = 0; i < joint->servo_nums; i++) {
        Servo *servo = joint->servos[i];
        float servo_angle = angle_diff;

        if (servo->direction == CCW_DIRECTION) {
            servo_angle = -servo_angle;
        }

        int servo_id = servo->id;

        switch (mode)
        {
        case ARM_SERVO_MODE_INTERVAL:
            ret = g_arm_robot.servo_ops->set_angle_interval(servo_id, servo_angle, interval_ms);
            break;
        case ARM_SERVO_MODE_INTERVAL_ACC:
            ret = g_arm_robot.servo_ops->set_angle_interval_acc(servo_id, servo_angle, interval_ms, acc_ms, dec_ms);
            break;
        case ARM_SERVO_MODE_VELOCITY_ACC:
            ret = g_arm_robot.servo_ops->set_angle_interval_velocity(servo_id, servo_angle, velocity, acc_ms, dec_ms);
            break;
        default:
            ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid servo mode %d\n", mode);
            return -1;
        }
        
        if (ret != 0) {
            ARM_LOGE_TAG(
                ARM_ROBOT_LOG_TAG,
                "Failed to set angle for joint %d servo %d, ret=%d\n",
                joint_id,
                servo_id,
                ret
            );
            return ret;
        }
    }
    joint->angle = angle;
    return 0;
}

int arm_set_joint_angle_interval(int joint_id, float angle, int interval_ms) 
{
    return arm_set_joint_angle_common(joint_id, angle, ARM_SERVO_MODE_INTERVAL, interval_ms, 0, 0, 0);
}

int arm_set_joint_angle_interval_acc(int joint_id, float angle, int interval_ms, int acc_ms, int dec_ms)
{
    return arm_set_joint_angle_common(joint_id, angle, ARM_SERVO_MODE_INTERVAL_ACC, interval_ms, 0, acc_ms, dec_ms);
}

int arm_set_joint_angle_velocity_acc(int joint_id, float angle, float velocity, int acc_ms, int dec_ms)
{
    return arm_set_joint_angle_common(joint_id, angle, ARM_SERVO_MODE_VELOCITY_ACC, 0, velocity, acc_ms, dec_ms);
}

static int arm_joint_snapshot_servo_count(bool claw_update)
{
    return claw_update ? ARM_JOINTS_NUM : ARM_JOINTS_NO_CLAW_NUM;
}

static float arm_joint_angle_from_primary_servo(int joint_id, float servo_angle)
{
    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    Servo *servo = joint->servos[0];
    float angle_diff = servo_angle;

    if (servo->direction == CCW_DIRECTION) {
        angle_diff = -angle_diff;
    }

    return angle_diff + joint->init_angle;
}

static void arm_joint_snapshot_seed(float snapshot[ARM_JOINTS_NUM])
{
    for (int i = 0; i < ARM_JOINTS_NUM; ++i) {
        snapshot[i] = g_arm_robot.joint[i].angle;
    }
}

static int arm_joint_snapshot_read_once(bool claw_update, float snapshot[ARM_JOINTS_NUM])
{
    int joint_count;
    ServoData monitor_data[ARM_JOINTS_NUM] = {0};
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;

    if ((snapshot == NULL) || (servo_ops == NULL) || (servo_ops->monitor_batch == NULL)) {
        return -1;
    }

    joint_count = arm_joint_snapshot_servo_count(claw_update);
    arm_joint_snapshot_seed(snapshot);
    if (servo_ops->monitor_batch(g_arm_primary_servo_ids, joint_count, monitor_data) != 0) {
        return -1;
    }

    for (int i = 0; i < joint_count; ++i) {
        snapshot[i] = arm_joint_angle_from_primary_servo(i, monitor_data[i].angle);
    }

    return 0;
}

int arm_get_joint_angle(int joint_id, float *angle)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid joint_id %d\n", joint_id);
        return -1;
    }

    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    Servo *servo = joint->servos[0];
    int servo_id = servo->id;
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;
    float servo_angle = 0.0f;
    
    int ret = servo_ops->get_angle(servo_id, &servo_angle);
    if (ret != 0) {
        ARM_LOGW_TAG(
            ARM_ROBOT_LOG_TAG,
            "Failed to get angle for joint %d servo %d, ret=%d\n",
            joint_id,
            servo_id,
            ret
        );
        return -1;
    }

    float angle_diff = servo_angle;
    if (servo->direction == CCW_DIRECTION) {
        angle_diff = -angle_diff;
    }

    *angle = angle_diff + joint->init_angle;
    return 0;
}

int arm_joint_snapshot_read(bool claw_update, float joint_angles[ARM_JOINTS_NUM])
{
    int joint_count;
    float snapshot[ARM_JOINTS_NUM] = {0};

    if (joint_angles == NULL) {
        return -1;
    }

    joint_count = arm_joint_snapshot_servo_count(claw_update);
    for (uint32_t attempt = 0U; attempt < ARM_JOINT_SNAPSHOT_MAX_ATTEMPTS; ++attempt) {
        if (arm_joint_snapshot_read_once(claw_update, snapshot) == 0) {
            for (int i = 0; i < joint_count; ++i) {
                g_arm_robot.joint[i].angle = snapshot[i];
            }
            memcpy(joint_angles, snapshot, sizeof(snapshot));
            return 0;
        }
    }

    return -1;
}

void arm_print_status_frame(const float joint_angles[ARM_JOINTS_NUM])
{
    if (joint_angles == NULL) {
        return;
    }

    safe_printf(
        "[STATUS] J0:%.2f J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f CLAW:%.2f\n",
        joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3],
        joint_angles[4], joint_angles[5], joint_angles[6]
    );
}

int arm_joint_angle_update(bool claw_update)
{
    float joint_angles[ARM_JOINTS_NUM] = {0};

    return arm_joint_snapshot_read(claw_update, joint_angles);
}

void arm_robot_set_sync(uint32_t sync_mask, bool sync)
{
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        if (sync_mask & (1 << i)) {
            g_arm_robot.joint[i].sync = sync;
        }
    }
}

bool arm_wait_sync_finished(int timeout_ms)
{
    uint8_t joint_finished[ARM_JOINTS_NUM] = {0};
    uint8_t finished_count = 0;

    if ((timeout_ms < 0) || (timeout_ms > ARM_MAX_WAIT_TIME)) {
        timeout_ms = ARM_MAX_WAIT_TIME;
    }

    uint32_t start_time = osKernelGetTickCount();
    int err_count = 0;
    while (osKernelGetTickCount() - start_time < timeout_ms) { 
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            if (finished_count >= ARM_JOINTS_NUM) {
                return true;
            }

            if (joint_finished[i] == 1) {
                continue;
            }

            if (g_arm_robot.joint[i].sync == false) {
                joint_finished[i] = 1;
                finished_count++;
                continue;
            }

            float angle;
            int ret = arm_get_joint_angle(i, &angle);
            if (ret != 0) {
                err_count++;
                if (err_count > ARM_MAX_ERROR_COUNT) {
                    ARM_LOGE_TAG(
                        ARM_ROBOT_LOG_TAG,
                        "Failed to get angle for joint %d during wait sync\n",
                        i
                    );
                    return false;
                }
                osDelay(10);
                continue;
            }

            if (fabsf(angle - g_arm_robot.joint[i].angle) < ARM_ANGLE_ERROR) {
                joint_finished[i] = 1;
                finished_count++;
            }
            osDelay(10);
        }
    }
    
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        if (joint_finished[i] == 0) {
            float current_angle = -999.9;
            arm_get_joint_angle(i, &current_angle);
            ARM_LOGE_TAG(
                ARM_ROBOT_LOG_TAG,
                "Joint %d sync finished timeout, target angle: %.2f, current angle: %.2f\n",
                i,
                g_arm_robot.joint[i].angle,
                current_angle
            );
            ArmJoint *joint = &g_arm_robot.joint[i];
            for (int j = 0; j < joint->servo_nums; j++) {
                Servo *servo = joint->servos[j];
                g_arm_robot.servo_ops->get_status(servo->id);
            }
        }
    }
    return false;
}

int arm_set_joint_stop(int joint_id, enum ArmJointStopMode mode, int param)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid joint_id %d\n", joint_id);
        return -1;
    }

    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    for (int i = 0; i < joint->servo_nums; i++) {
        Servo *servo = joint->servos[i];
        int ret = g_arm_robot.servo_ops->stop(servo->id, mode, param);
        if (ret != 0) {
            ARM_LOGE_TAG(
                ARM_ROBOT_LOG_TAG,
                "Failed to stop joint %d servo %d, ret=%d\n",
                joint_id,
                servo->id,
                ret
            );
            return ret;
        }
    }

    return 0;
}

// 建议仅内部使用，没有加速度
int arm_joint_sync_move(float joint_angles[ARM_JOINTS_NUM])
{
    ServoSyncData sync_data[ARM_SERVO_NUM] = {0};
    int data_num = 0;

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        float angle = joint_angles[i];
        if (fabsf(angle - ARM_SYNC_NO_CHANGE) < ARM_FLOAT_TOLERANCE) {
            continue;
        }

        ArmJoint *joint = &g_arm_robot.joint[i];
        float angle_diff = angle - g_arm_robot.joint[i].init_angle;
        
        for (int n = 0; n < joint->servo_nums; n++) {
            Servo *servo = joint->servos[n];
            float servo_angle = angle_diff;
            if (servo->direction == CCW_DIRECTION) {
                servo_angle = -servo_angle;
            }

            int servo_id = servo->id;
            
            sync_data[data_num].servo_id = servo_id;
            sync_data[data_num].angle = servo_angle;
            data_num++;
        }
    }

    int ret = g_arm_robot.servo_ops->sync_move(sync_data, data_num);
    if  (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to sync move, ret=%d\n", ret);
        return -1;
    }

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        float angle = joint_angles[i];
        if (fabsf(angle - ARM_SYNC_NO_CHANGE) < ARM_FLOAT_TOLERANCE) {
            continue;
        }
        g_arm_robot.joint[i].angle = angle;
    }
    return 0;
}

int arm_set_zero(void)
{
    ArmConfig *cfg = &g_arm_robot.cfg;
    int ret = 0;
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        ArmJoint *joint = &g_arm_robot.joint[i];
        for (int n = 0; n < joint->servo_nums; n++) {
            Servo *servo = joint->servos[n];
            if (g_arm_robot.servo_ops->set_zero(servo->id) != 0) {
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Failed to set zero for joint %d servo %d\n",
                    i,
                    servo->id
                );
                ret = -1;
            }
        }
        joint->angle = joint->init_angle;
    }

    if (ret != 0) {
        return ret;
    }

    safe_printf("Zero position setting completed successfully.\n");

    if (cfg->set_zero_flag == 1) {
        return 0;
    }
    cfg->set_zero_flag = 1;
    ret = arm_flash_config_save();
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to save config\n");
        return -1;
    }

    return 0;
}

static void arm_robot_update_light(void)
{
    static float brightness = 0.8f;
    static int init = 0;
    static int last_servo_error_flag = 0;

    // 默认初始化为绿色状态
    if (init == 0) {
        init = 1;
        W2812_SetAllColors(0, 255 * brightness, 0);
        W2812_SendData(false);
        return;
    }

    if (g_arm_robot.servo_error_flag == last_servo_error_flag) {
        return;
    }

    last_servo_error_flag = g_arm_robot.servo_error_flag;

    if (g_arm_robot.servo_error_flag == 1) {
        // 错误状态的灯光，红色
        W2812_SetAllColors(255 * brightness, 0, 0);
    } else if (g_arm_robot.servo_error_flag == 0) {
        // 正常状态的灯光，绿色
        W2812_SetAllColors(0, 255 * brightness, 0);
    }

    W2812_SendData(false);
    return;
}

static int arm_monitor_batch_read(ServoData servodata[ARM_JOINTS_NUM])
{
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;

    if ((servo_ops == NULL) || (servodata == NULL)) {
        return -1;
    }

    if (servo_ops->monitor_batch != NULL) {
        return servo_ops->monitor_batch(g_arm_primary_servo_ids, ARM_JOINTS_NUM, servodata);
    }

    if (servo_ops->monitor == NULL) {
        return -1;
    }

    for (int i = 0; i < ARM_JOINTS_NUM; ++i) {
        if (servo_ops->monitor((int)g_arm_primary_servo_ids[i], &servodata[i]) != 0) {
            return -1;
        }
    }

    return 0;
}

static bool arm_monitor_status_requires_recovery(const ServoData *status)
{
    if (status == NULL) {
        return false;
    }

    return ((fabsf(status->angle) - 180.0f) > (1e-6f)) ||
           ((status->circle_count != 0) && (status->circle_count != -1));
}

static void arm_monitor_handle_temperature_warning(uint8_t servo_id, const ServoData *status)
{
    static uint32_t filter_count[ARM_SERVO_NUM] = {0};
    uint32_t *counter;

    if ((status == NULL) || (servo_id < 1U) || (servo_id > ARM_SERVO_NUM)) {
        return;
    }

    counter = &filter_count[servo_id - 1U];
    if (status->temperature > ARM_MONITOR_TEMPERATURE_WARN_C) {
        (*counter)++;
        if ((*counter % ARM_MONITOR_TEMPERATURE_FILTER_CYCLES) == 0U) {
            ARM_LOGW_TAG(
                ARM_ROBOT_LOG_TAG,
                "Servo %u temperature is %d\n",
                (unsigned int)servo_id,
                status->temperature
            );
        }
    } else {
        *counter = 0U;
    }
}

static void arm_monitor_handle_servo_status(uint8_t servo_id, const ServoData *status)
{
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;
    int ret;

    if ((status == NULL) || (servo_ops == NULL)) {
        return;
    }

    arm_monitor_handle_temperature_warning(servo_id, status);
    if (!arm_monitor_status_requires_recovery(status)) {
        return;
    }

    ARM_LOGW_TAG(
        ARM_ROBOT_LOG_TAG,
        "Servo %u monitor abnormal: angle=%.2f, circle_count=%d, attempting recovery\n",
        (unsigned int)servo_id,
        status->angle,
        status->circle_count
    );

    if (servo_ops->stop != NULL) {
        (void)servo_ops->stop((int)servo_id, 0, 0);
    }

    if (servo_ops->reset_angle == NULL) {
        return;
    }

    osDelay(ARM_MONITOR_RECOVERY_DELAY_MS);
    ret = servo_ops->reset_angle((int)servo_id);
    if (ret != 0) {
        ARM_LOGE_TAG(
            ARM_ROBOT_LOG_TAG,
            "Failed to reset abnormal servo %u, ret=%d\n",
            (unsigned int)servo_id,
            ret
        );
    }
}

static uint16_t arm_monitor_collect_failed_servo_mask(void)
{
    uint16_t failed_mask = 0U;
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;
    ServoData status = {0};

    if ((servo_ops == NULL) || (servo_ops->monitor == NULL)) {
        return 0xFFFFU;
    }

    for (int i = 0; i < ARM_SERVO_NUM; ++i) {
        if ((servo_ops->monitor(g_arm_all_servo_ids[i], &status) != 0) ||
            arm_monitor_status_requires_recovery(&status)) {
            failed_mask |= (uint16_t)(1U << i);
        }
    }

    return failed_mask;
}

static void arm_monitor_format_failed_ids(uint16_t failed_mask, char *buffer, size_t buffer_size)
{
    size_t offset = 0U;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    buffer[0] = '\0';
    for (int i = 0; i < ARM_SERVO_NUM; ++i) {
        if ((failed_mask & (1U << i)) == 0U) {
            continue;
        }

        offset += (size_t)snprintf(
            buffer + offset,
            (offset < buffer_size) ? (buffer_size - offset) : 0U,
            "%s%u",
            (offset == 0U) ? "" : ",",
            (unsigned int)g_arm_all_servo_ids[i]
        );

        if (offset >= (buffer_size - 1U)) {
            break;
        }
    }
}

void arm_monitor(void)
{
    static uint8_t consecutive_batch_failures = 0U;
    static uint16_t last_failed_servo_mask = 0xFFFFU;
    ServoData monitor_data[ARM_JOINTS_NUM] = {0};

    if (arm_monitor_batch_read(monitor_data) == 0) {
        for (int i = 0; i < ARM_JOINTS_NUM; ++i) {
            arm_monitor_handle_servo_status(g_arm_primary_servo_ids[i], &monitor_data[i]);
        }
        if (g_arm_robot.servo_error_flag != 0) {
            ARM_LOGI_TAG(ARM_ROBOT_LOG_TAG, "Servo monitor recovered\n");
        }
        consecutive_batch_failures = 0U;
        last_failed_servo_mask = 0xFFFFU;
        g_arm_robot.servo_error_flag = 0;
        arm_robot_update_light();
        return;
    }

    if (consecutive_batch_failures < 0xFFU) {
        consecutive_batch_failures++;
    }

    if (consecutive_batch_failures >= 3U) {
        uint16_t failed_servo_mask = arm_monitor_collect_failed_servo_mask();

        g_arm_robot.servo_error_flag = 1;
        if (failed_servo_mask != last_failed_servo_mask) {
            if (failed_servo_mask == 0U) {
                ARM_LOGW_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Batch monitor failed %u times, but single-servo diagnosis found no failing ID\n",
                    (unsigned int)consecutive_batch_failures
                );
            } else {
                char failed_ids[48];

                arm_monitor_format_failed_ids(failed_servo_mask, failed_ids, sizeof(failed_ids));
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Batch monitor failed %u times, abnormal servo IDs: %s\n",
                    (unsigned int)consecutive_batch_failures,
                    failed_ids
                );
            }
            last_failed_servo_mask = failed_servo_mask;
        }
    }

    arm_robot_update_light();
}
