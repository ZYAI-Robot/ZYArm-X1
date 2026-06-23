#include "arm_robot.h"
#include "arm_first_use_runtime.h"
#include "arm_shell.h"
#include "arm_flash.h"
#include "arm_monitor.h"
#include "arm_recorder.h"
#include "arm_robot_kinematics.h"
#include "arm_w25q128_partition.h"
#include "w25q128.h"
#include "w2812.h"
#include <stdio.h>
#include <string.h>

#define ARM_ROBOT_LOG_TAG "ARM_ROBOT"
#define ARM_JOINT_SNAPSHOT_MAX_ATTEMPTS 3U

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
static volatile uint32_t g_arm_robot_stop_generation = 0U;

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

static int arm_mark_zero_configured(void)
{
    ArmConfig *cfg = &g_arm_robot.cfg;
    ArmConfig old_cfg = *cfg;
    int ret;

    if (cfg->set_zero_flag == 1) {
        return 0;
    }

    cfg->set_zero_flag = 1;
    ret = arm_flash_config_save();
    if (ret != 0) {
        *cfg = old_cfg;
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to save config\n");
        return -1;
    }

    return 0;
}

static void arm_first_user_config(void)
{
    uint8_t input_char[10] = {0};
    
    // 提示用户手动调整机械臂到零位
    ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to load robot config from Flash\n");
    safe_printf(
        "Enter Y to set current position as zero, or S to skip and use the zero stored in servos.\n"
    );
    arm_set_all_joint_stop(ARM_JOINT_STOP_DAMPING_MODE, 800);
    while (1) {
        if (HAL_UART_Receive(&huart1, &(input_char[0]), 1, HAL_MAX_DELAY) == HAL_OK) {
            if ((input_char[0] == 'Y') || (input_char[0] == 'y')) {
                safe_printf("Setting zero position...\n");
                if (arm_set_zero() == 0) {
                    break;
                }
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Failed to set zero position, please try again after powering off and on\n"
                );
                continue;
            }

            if ((input_char[0] == 'S') || (input_char[0] == 's')) {
                safe_printf("Skipping zero position setting, saving config...\n");
                if (arm_mark_zero_configured() == 0) {
                    safe_printf("Zero position setting skipped, using the zero stored in servos.\n");
                    break;
                }
                ARM_LOGE_TAG(
                    ARM_ROBOT_LOG_TAG,
                    "Failed to save zero configuration, please try again after powering off and on\n"
                );
                continue;
            }

            ARM_LOGW_TAG(ARM_ROBOT_LOG_TAG, "Invalid input, please enter Y or S\n");
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

    ret = arm_w25q128_partition_init();
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to validate W25Q128 partitions, ret=%d\n", ret);
        return;
    }

    ret = arm_record_flash_validate_capacity();
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to validate recorder flash capacity, ret=%d\n", ret);
        return;
    }

    ret = arm_first_use_runtime_init();
    if (ret != 0) {
        ARM_LOGW_TAG(ARM_ROBOT_LOG_TAG, "First-use runtime meter is unavailable, ret=%d\n", ret);
    }

    arm_monitor_init();

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

static float arm_joint_target_to_servo_angle(
    const ArmJoint *joint,
    const Servo *servo,
    float joint_angle
) {
    float angle_diff;

    if ((joint == NULL) || (servo == NULL)) {
        return 0.0f;
    }

    angle_diff = joint_angle - joint->init_angle;
    return (servo->direction == CCW_DIRECTION) ? -angle_diff : angle_diff;
}

static void arm_joint_get_monitor_servo_targets(
    const ArmJoint *joint,
    float joint_angle,
    float *primary_target_servo_angle,
    float *secondary_target_servo_angle
) {
    if (primary_target_servo_angle != NULL) {
        *primary_target_servo_angle = 0.0f;
    }
    if (secondary_target_servo_angle != NULL) {
        *secondary_target_servo_angle = 0.0f;
    }
    if (joint == NULL) {
        return;
    }

    if ((primary_target_servo_angle != NULL) &&
        (joint->servo_nums > 0) &&
        (joint->servos[0] != NULL)) {
        *primary_target_servo_angle =
            arm_joint_target_to_servo_angle(joint, joint->servos[0], joint_angle);
    }
    if ((secondary_target_servo_angle != NULL) &&
        (joint->servo_nums > 1) &&
        (joint->servos[1] != NULL)) {
        *secondary_target_servo_angle =
            arm_joint_target_to_servo_angle(joint, joint->servos[1], joint_angle);
    }
}

void arm_joint_arm_monitor(
    int joint_id,
    float angle,
    enum ArmMotionMonitorPolicy monitor_policy
) {
    ArmJoint *joint;
    float primary_target_servo_angle = 0.0f;
    float secondary_target_servo_angle = 0.0f;

    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        return;
    }

    joint = &g_arm_robot.joint[joint_id];
    arm_joint_get_monitor_servo_targets(
        joint,
        angle,
        &primary_target_servo_angle,
        &secondary_target_servo_angle
    );
    arm_monitor_on_joint_motion(
        joint_id,
        angle,
        primary_target_servo_angle,
        secondary_target_servo_angle,
        monitor_policy
    );
}

static int arm_set_joint_angle_common(
    int joint_id,
    float angle,
    enum ArmServoMode mode,
    int interval_ms,
    float velocity,
    int acc_ms,
    int dec_ms,
    enum ArmMotionMonitorPolicy monitor_policy
)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Invalid joint_id %d\n", joint_id);
        return -1;
    }

    ArmJoint *joint = &g_arm_robot.joint[joint_id];
    arm_monitor_cancel_joint_motion(joint_id);

    int ret = 0;
    for (int i = 0; i < joint->servo_nums; i++) {
        Servo *servo = joint->servos[i];
        float servo_angle = arm_joint_target_to_servo_angle(joint, servo, angle);

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
    arm_joint_arm_monitor(joint_id, angle, monitor_policy);
    return 0;
}

int arm_set_joint_angle_interval(int joint_id, float angle, int interval_ms) 
{
    return arm_set_joint_angle_common(
        joint_id,
        angle,
        ARM_SERVO_MODE_INTERVAL,
        interval_ms,
        0,
        0,
        0,
        ARM_MOTION_MONITOR_AUTO_HOLD
    );
}

int arm_set_joint_angle_interval_acc(int joint_id, float angle, int interval_ms, int acc_ms, int dec_ms)
{
    return arm_set_joint_angle_interval_acc_with_monitor_policy(
        joint_id,
        angle,
        interval_ms,
        acc_ms,
        dec_ms,
        ARM_MOTION_MONITOR_AUTO_HOLD
    );
}

int arm_set_joint_angle_interval_acc_with_monitor_policy(
    int joint_id,
    float angle,
    int interval_ms,
    int acc_ms,
    int dec_ms,
    enum ArmMotionMonitorPolicy monitor_policy
) {
    return arm_set_joint_angle_common(
        joint_id,
        angle,
        ARM_SERVO_MODE_INTERVAL_ACC,
        interval_ms,
        0,
        acc_ms,
        dec_ms,
        monitor_policy
    );
}

int arm_set_joint_angle_velocity_acc(int joint_id, float angle, float velocity, int acc_ms, int dec_ms)
{
    return arm_set_joint_angle_common(
        joint_id,
        angle,
        ARM_SERVO_MODE_VELOCITY_ACC,
        0,
        velocity,
        acc_ms,
        dec_ms,
        ARM_MOTION_MONITOR_AUTO_HOLD
    );
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

void arm_robot_request_stop(void)
{
    g_arm_robot_stop_generation++;
    arm_robot_set_sync(ARM_ALL_JOINTS_SYNC_MASK, false);
}

bool arm_wait_sync_finished(int timeout_ms)
{
    uint8_t joint_finished[ARM_JOINTS_NUM] = {0};
    uint8_t finished_count = 0;
    uint32_t stop_generation = g_arm_robot_stop_generation;

    if ((timeout_ms < 0) || (timeout_ms > ARM_MAX_WAIT_TIME)) {
        timeout_ms = ARM_MAX_WAIT_TIME;
    }

    uint32_t start_time = osKernelGetTickCount();
    int err_count = 0;
    while (osKernelGetTickCount() - start_time < timeout_ms) { 
        if (stop_generation != g_arm_robot_stop_generation) {
            arm_robot_set_sync(ARM_ALL_JOINTS_SYNC_MASK, false);
            return false;
        }

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
    arm_monitor_cancel_joint_motion(joint_id);
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

int arm_joint_sync_move_with_monitor_policy(
    float joint_angles[ARM_JOINTS_NUM],
    enum ArmMotionMonitorPolicy monitor_policy
)
{
    ServoSyncData sync_data[ARM_SERVO_NUM] = {0};
    int data_num = 0;

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        float angle = joint_angles[i];
        if (fabsf(angle - ARM_SYNC_NO_CHANGE) < ARM_FLOAT_TOLERANCE) {
            continue;
        }

        ArmJoint *joint = &g_arm_robot.joint[i];
        for (int n = 0; n < joint->servo_nums; n++) {
            Servo *servo = joint->servos[n];
            float servo_angle = arm_joint_target_to_servo_angle(joint, servo, angle);

            int servo_id = servo->id;
            
            sync_data[data_num].servo_id = servo_id;
            sync_data[data_num].angle = servo_angle;
            data_num++;
        }
    }

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        if (fabsf(joint_angles[i] - ARM_SYNC_NO_CHANGE) < ARM_FLOAT_TOLERANCE) {
            continue;
        }
        arm_monitor_cancel_joint_motion(i);
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
        arm_joint_arm_monitor(i, angle, monitor_policy);
    }
    return 0;
}

// 建议仅内部使用，没有加速度
int arm_joint_sync_move(float joint_angles[ARM_JOINTS_NUM])
{
    return arm_joint_sync_move_with_monitor_policy(joint_angles, ARM_MOTION_MONITOR_AUTO_HOLD);
}

int arm_set_zero(void)
{
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

    return arm_mark_zero_configured();
}
