#include "arm_recorder.h"
#include "arm_robot.h"
#include "arm_shell.h"
#include "arm_flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os2.h"
#include "math.h"
#include "arm_robot_kinematics.h"

#define ARM_RECORD_LOG_TAG "RECORDER"

static int g_error = 0;

static ArmRecordManager g_record_manager = {0}; // 作为一个全局共用的动作录制管理器
ArmRecorderParam g_arm_recorder_param = {
    .lpf_coefficient = 0.8,
    .player_interval_ms = 10,
};

static bool arm_record_check_name(const char *name)
{
    if (name == NULL || strlen(name) == 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "record_check_name need 1 params: record_name\n");
        return false;
    }

    for (int i = 0; i < ARM_RECORD_MAX_NUMS; i++) {
        ArmRecordHead head = {0};
        HAL_StatusTypeDef ret = W25Q128_Read(i * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&head, sizeof(ArmRecordHead), 1000);
        if (ret != HAL_OK) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action head information\n");
            return false;    
        }

        if (strcmp(head.name, name) == 0) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action name %s already exists\n", name);
            return false;
        }
    }
    return true;
}

static int arm_record_manger_get(const char *name)
{
    int index = 0;
    ArmRecordHead head = {0};
    for (index = 0; index < ARM_RECORD_MAX_NUMS; index++) {
        HAL_StatusTypeDef ret = W25Q128_Read(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&head, sizeof(ArmRecordHead), 1000);
        if (ret != HAL_OK) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action head information\n");
            return -1;    
        }

        if ((head.state == ARM_RECORD_STATUS_VALID) && (strlen(head.name) > 0) && (strcmp(head.name, name) == 0)) {
            ret = W25Q128_Read(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&g_record_manager, sizeof(ArmRecordManager), 1000);
            if (ret != HAL_OK) {
                ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action recording information\n");
                return -1;    
            }
            return 0;
        } 
    }

    return -1;
}

void arm_player_task(void *argument)
{
    int ret = 0;
    int record_interval_ms = ARM_RECORD_INTERVAL_DEFAULT; // 播放间隔100ms
    int interpolation_step = record_interval_ms / g_arm_recorder_param.player_interval_ms; // 插值步数
    
    const float *current_joint_angles;
    const float *target_joint_angles;
    float joint_angle_sync[ARM_JOINTS_NUM] = {0};
    int player_interval_ms = g_arm_recorder_param.player_interval_ms;
    float target_joints[6];
    float interval_ms;
    TickType_t xLastWakeTime;

    // 开始播放动作
    safe_printf("Waiting for the robotic arm to position...\n");
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        target_joints[i] = g_record_manager.info[0].joint_angle[i];
    }

    ret = arm_update_cal_interval_with_angle_diff(target_joints, &interval_ms);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to update the calibration interval\n");
        g_error = -1;
        goto EXIT;
    }
    
    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        arm_set_joint_angle_interval_acc(i, g_record_manager.info[0].joint_angle[i], (int)roundf(interval_ms), ARM_DEFAULT_ACCEL_TIME, ARM_DEFAULT_ACCEL_TIME);
    }
    
    vTaskDelay(roundf(interval_ms + 1000));
    safe_printf("The robotic arm starts to play the motion trajectory...\n");
    // 初始化当前位置
    current_joint_angles = g_record_manager.info[0].joint_angle;
    xLastWakeTime = xTaskGetTickCount();
    
    for (int info_num = 1; info_num < g_record_manager.head.info_num; info_num++) {
        // 设置目标位置
        target_joint_angles = g_record_manager.info[info_num].joint_angle;
        
        // 使用更短的时间间隔进行插值控制，减缓机械臂抖动
        for (int step = 1; step <= interpolation_step; step++) {
            float ratio = (float)step / interpolation_step;
            
            // 更新每个关节的角度
            for (int joint_id = 0; joint_id < ARM_JOINTS_NUM; joint_id++) {
                float interpolated_angle = current_joint_angles[joint_id] + 
                    (target_joint_angles[joint_id] - current_joint_angles[joint_id]) * ratio;
                joint_angle_sync[joint_id] = interpolated_angle;
            }

            ret = arm_joint_sync_move(joint_angle_sync);
            if (ret != 0) {
                ARM_LOGW_TAG(ARM_RECORD_LOG_TAG, "Robot arm failed to move to the target point\n");
            }

            // 等待下一个控制周期
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(player_interval_ms));
        }
        
        // 更新当前位置为目标位置
        current_joint_angles = target_joint_angles;
        
        if (info_num % 10 == 0) {
            safe_printf("\rPlaying time: %d/%ds    ", (info_num/10), (g_record_manager.head.info_num/10));
        }
        if (g_record_manager.head.state == ARM_RECORD_STATUS_STOP) {
            break;
        }
    }
    safe_printf("\n");
    safe_printf("Robot arm action playing finished.\n");

EXIT:
    g_record_manager.head.state = 0;
    g_arm_robot.state = ARM_STATE_IDLE;
    send_ack_completed(CMD_ID_RECORD_PLAYER, g_error);
    vTaskDelete(NULL);
}

void arm_record_manger_list(uint32_t cmd_id)
{ 
    int index = 0;
    ArmRecordHead head = {0};
    int count = 0;
    send_string_response(cmd_id, "arm record list:\n");
    for (index = 0; index < ARM_RECORD_MAX_NUMS; index++) {
        HAL_StatusTypeDef ret = W25Q128_Read(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&head, sizeof(ArmRecordHead), 1000);
        if (ret != HAL_OK) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action head information\n");
            continue;    
        }

        if ((head.state == ARM_RECORD_STATUS_VALID) && (head.info_num > 0) && (strlen(head.name) > 0)) {
            send_string_response(cmd_id, "index: %d, name: %s, duration: %.2fs\n", index, head.name, ((float)head.info_num) * ARM_RECORD_INTERVAL_DEFAULT / 1000);
            count++;
        }
    }

    if (count == 0) {
        send_string_response(cmd_id, "No action recording information.\n");
    }
}

int arm_record_info_show(const char *name)
{
    safe_printf("Action recording information (%d)\n", sizeof(ArmRecordManager));

    int ret = arm_record_manger_get(name);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action recording information is not found\n");
        return -1;
    }

    if ((g_record_manager.head.state != ARM_RECORD_STATUS_VALID) || (g_record_manager.head.info_num == 0)) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action recording information is invalid\n");
        return -1;
    }

    safe_printf("Name: %s\n", g_record_manager.head.name);
    safe_printf("Info number: %d\n", g_record_manager.head.info_num);

    for (int i = 0; i < g_record_manager.head.info_num; i++) {
        safe_printf("[%d] [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n", i, g_record_manager.info[i].joint_angle[0], g_record_manager.info[i].joint_angle[1], g_record_manager.info[i].joint_angle[2],
            g_record_manager.info[i].joint_angle[3], g_record_manager.info[i].joint_angle[4], g_record_manager.info[i].joint_angle[5], g_record_manager.info[i].joint_angle[6]);
    }

    return 0;
}

int arm_record_flash_erase(const char *name)
{
    if (strcmp(name, "all") == 0) {
        // 擦除所有动作录制相关扇区
        for (int index = 0; index < ARM_RECORD_MAX_NUMS; index++) {
            HAL_StatusTypeDef ret = W25Q128_EraseSectors(index * ARM_RECORD_MANAGER_FLASH_SIZE, ARM_RECORD_MANAGER_FLASH_SIZE, 1000);
            if (ret != HAL_OK) {
                ARM_LOGE_TAG(
                    ARM_RECORD_LOG_TAG,
                    "Failed to erase the action recording information from FLASH%d\n",
                    index * ARM_RECORD_MANAGER_FLASH_SIZE
                );
                return -1;
            }
        }
        return 0;
    }

    // 查找指定名称的录制
    int index = 0;
    ArmRecordHead head = {0};
    for (index = 0; index < ARM_RECORD_MAX_NUMS; index++) {
        HAL_StatusTypeDef ret = W25Q128_Read(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&head, sizeof(ArmRecordHead), 1000);
        if (ret != HAL_OK) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action head information\n");
            return -1;
        }

        if ((head.state == ARM_RECORD_STATUS_VALID) && (strlen(head.name) > 0) && (strcmp(head.name, name) == 0)) {
            // 找到对应录制并擦除
            HAL_StatusTypeDef ret = W25Q128_EraseSectors(index * ARM_RECORD_MANAGER_FLASH_SIZE, ARM_RECORD_MANAGER_FLASH_SIZE, 1000);
            if (ret != HAL_OK) {
                ARM_LOGE_TAG(
                    ARM_RECORD_LOG_TAG,
                    "Failed to erase the action recording information from FLASH%d\n",
                    index * ARM_RECORD_MANAGER_FLASH_SIZE
                );
                return -1;
            }
            return 0;
        }
    }

    ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action recording named '%s' is not found\n", name);
    return -1;
}

int arm_record_flash_malloc(void)
{
    int index = 0;
    ArmRecordHead head = {0};
    for (index = 0; index < ARM_RECORD_MAX_NUMS; index++) {
        HAL_StatusTypeDef ret = W25Q128_Read(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&head, sizeof(ArmRecordHead), 1000);
        if (ret != HAL_OK) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to read the action head information\n");
            return -1;    
        }

        if (head.state != ARM_RECORD_STATUS_VALID) {
            return index;
        }
    }

    return -1;
}

static int arm_record_save(void)
{
    int index = 0;

    // 查询空闲位置
    index = arm_record_flash_malloc();
    if (index < 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "No free space to save the action recording information\n");
        return -1;
    }
    
    HAL_StatusTypeDef ret = W25Q128_Write(index * ARM_RECORD_MANAGER_FLASH_SIZE, (uint8_t *)&g_record_manager, sizeof(ArmRecordManager), 1000);
    if (ret != HAL_OK) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to write the action recording information to FLASH\n");
        return -1;
    }

    return index;
}
void arm_record_task(void *argument)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(ARM_RECORD_INTERVAL_DEFAULT);
    TickType_t xLastWakeTime;
    int info_num = 0;
    float alpha = g_arm_recorder_param.lpf_coefficient;
    int ret;

    safe_printf("Robot arm record task running.\n");
    ret = arm_set_all_joint_stop(ARM_JOINT_STOP_LOCK_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to stop the robotic arm\n");
        g_error = -1;
        goto EXIT;
    }

    safe_printf("The robotic arm will start recording the action in 5 seconds. Please get ready!\n");
    for (int time = 5; time  > 0; time--) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        safe_printf("%ds    \n", time);
    }
    safe_printf("\n");
    
    // 电机下电，减少扭矩
    ret = arm_set_all_joint_stop(ARM_JOINT_STOP_UNLOAD_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Unloading the torque of the robotic arm failed\n");
        g_error = -1;
        goto EXIT;
    }

    // 开始记录
    safe_printf("The robotic arm is recording the action ...\n");

    // 使用vTaskDelayUntil进行定时控制
    xLastWakeTime = xTaskGetTickCount();

    for (info_num = 0; info_num < ARM_RECORD_INFO_NUM; info_num++) {
        // 使用vTaskDelayUntil进行定时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        for (int joint_id = 0; joint_id < ARM_JOINTS_NUM; joint_id++) {
            float angle = 0;
            ret = arm_get_joint_angle(joint_id, &angle);
            if (ret != 0) {
                ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to get the joint angle of joint %d\n", joint_id);
                g_error = -1;
                goto EXIT;
            }

            g_record_manager.info[info_num].joint_angle[joint_id] = angle;
            if (g_record_manager.info[info_num].joint_angle[joint_id] == ARM_FLOAT_ERROR) {
                ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to get the joint angle of joint %d\n", joint_id);
                g_error = -1;
                goto EXIT;
            }
        }

        g_record_manager.head.info_num = info_num + 1;
        if (info_num % 10 == 0) {
            safe_printf("Recording time: %d/300s    \n", (info_num/10));
        }
        if (g_record_manager.head.state == ARM_RECORD_STATUS_STOP) {
            safe_printf("Robot arm action recording stopped.\n");
            break;
        }
    }
    safe_printf("\n");
    safe_printf("Robot arm action recording completed.\n");
    safe_printf("Recording duration: %d s\n", info_num/10);
    
    g_record_manager.head.state = ARM_RECORD_STATUS_VALID;

    if (g_record_manager.head.info_num == 0) {
        goto EXIT;
    }
    
    // 数据后处理
    safe_printf("Please wait for data processing ...\n");
    
    // 一阶低通滤波
    for (int i = 1; i < g_record_manager.head.info_num; i++) {
        for (int joint_id = 0; joint_id < ARM_JOINTS_NUM; joint_id++) {
            g_record_manager.info[i].joint_angle[joint_id] = 
                alpha * g_record_manager.info[i].joint_angle[joint_id] + 
                (1.0f - alpha) * g_record_manager.info[i-1].joint_angle[joint_id];
        }
    }

    safe_printf("Complete data processing and save data.\n");
    // 将滤波后的数据保存到flash中
    ret = arm_record_save();
    if (ret >= 0 ) {
        safe_printf("The action recording information is saved, the FLASH index is %d.\n", ret);
    } else {
        g_error = -1;
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to save the action recording information\n");
    }
EXIT:
    // 电机上电
    ret = arm_set_all_joint_stop(ARM_JOINT_STOP_LOCK_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Loading the torque of the robotic arm failed\n");
        g_error = -1;
    }

    // 更新当前位置
    for (int joint_id = 0; joint_id < ARM_JOINTS_NUM; joint_id++) {
        float angle = 0;
        ret = arm_get_joint_angle(joint_id, &angle);
        if (ret != 0) {
            ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "Failed to get the joint angle\n");
            g_error = -1;
            goto EXIT2;
        }
        g_arm_robot.joint[joint_id].angle = angle;
    }
EXIT2:
    send_ack_completed(CMD_ID_RECORD, g_error);
    g_record_manager.head.state = 0;
    g_arm_robot.state = ARM_STATE_IDLE;
    vTaskDelete(NULL);
}

int arm_record_start(const char *name)
{
    safe_printf("Start recording action.\n");
    if (g_record_manager.head.state == ARM_RECORD_STATUS_RUNNING) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "arm record task is running\n");
        return -1;
    }

    if (!arm_record_check_name(name)) {
        return -1;
    }

    if (arm_record_flash_malloc() < 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action recording space is full. Please clear the FLASH\n");
        return -1;
    }

    osThreadAttr_t record_task_attributes = {
        .name = "arm_record_task",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityBelowNormal,
    };

    g_error = 0;
    g_record_manager.head.state = ARM_RECORD_STATUS_RUNNING;
    strncpy(g_record_manager.head.name, name, ARM_RECORD_NAME_MAX_LEN - 1);
    g_record_manager.head.name[ARM_RECORD_NAME_MAX_LEN - 1] = '\0';
    g_record_manager.head.record_thread_handle = osThreadNew(arm_record_task, NULL, &record_task_attributes);
    if (g_record_manager.head.record_thread_handle == NULL) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "arm record create thread failed\n");
        g_record_manager.head.state = 0;
        return -1;
    }

    g_arm_robot.tasks_handle = g_record_manager.head.record_thread_handle;
    g_arm_robot.state = ARM_STATE_RECORDING;

    return 0;
}

int arm_record_player_start(const char *name)
{
    if (g_record_manager.head.state == ARM_RECORD_STATUS_RUNNING) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "arm record task is running\n");
        return -1;
    }

    int ret = arm_record_manger_get(name); // 从flash中读取数据, 并赋值给g_record_manager
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "The action recording information is invalid\n");
        return -1;
    }
    
    osThreadAttr_t record_task_attributes = {
        .name = "arm_record_player_task",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityBelowNormal,
    };

    g_error = 0;
    g_record_manager.head.state = ARM_RECORD_STATUS_RUNNING;
    strncpy(g_record_manager.head.name, name, ARM_RECORD_NAME_MAX_LEN - 1);
    g_record_manager.head.name[ARM_RECORD_NAME_MAX_LEN - 1] = '\0';
    g_record_manager.head.record_thread_handle = osThreadNew(arm_player_task, NULL, &record_task_attributes);
    if (g_record_manager.head.record_thread_handle == NULL) {
        ARM_LOGE_TAG(ARM_RECORD_LOG_TAG, "arm record player create thread failed\n");
        g_record_manager.head.state = 0;
        return -1;
    }

    g_arm_robot.tasks_handle = g_record_manager.head.record_thread_handle;
    g_arm_robot.state = ARM_STATE_PLAYING;
    return 0;
}
void arm_record_stop(void)
{
    if (g_record_manager.head.state == ARM_RECORD_STATUS_RUNNING) {
        g_record_manager.head.state = ARM_RECORD_STATUS_STOP;
        safe_printf("Robot arm action recording stopped.\n");
    }
    return;
}
