#include "arm_master_slave.h"
#include "arm_robot.h"
#include "arm_shell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

#define ARM_MASTER_SLAVE_LOG_TAG "MASTER_SLAVE"

// 全局主从跟随管理器
static ArmMasterSlaveManager g_master_slave_manager = {
    .status = ARM_MASTER_SLAVE_STATUS_IDLE,
    .sample_period_ms = ARM_MASTER_SLAVE_DEFAULT_PERIOD_MS,
    .task_handle = NULL,
    .role = ARM_ROLE_MASTER,
    .lpf_coefficient = ARM_MASTER_SLAVE_DEFAULT_LPF_COEFFICIENT,
};

// 错误码
static int g_error = 0;

// 从臂滤波状态
static float g_prev_filtered_angles[ARM_JOINTS_NUM] = {0};
static bool g_filter_initialized = false;

// 从臂前一次执行的角度（用于角度变化限制）
static float g_prev_executed_angles[ARM_JOINTS_NUM] = {0};
static bool g_executed_angles_initialized = false;

// 任务函数前置声明
static void arm_master_task(void *argument);

// 主臂进入卸力模式
static int arm_master_enter_unload_mode(void)
{
    safe_printf("[MASTER] Master arm enters unload mode...\n");
    int ret = arm_set_all_joint_stop(ARM_JOINT_STOP_UNLOAD_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master arm unload mode failed\n");
        return -1;
    }
    safe_printf("[MASTER] Master arm unload mode success, you can manually drag it\n");
    return 0;
}

// 主臂读取所有关节角度(同步接口)
static int arm_master_read_joint_angles(float angles[ARM_JOINTS_NUM])
{
    return arm_joint_snapshot_read(true, angles);
}

// 从臂同步设置关节角度
static int arm_slave_set_joint_angles_real(float angles[ARM_JOINTS_NUM])
{
    int ret = arm_joint_sync_move(angles);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Slave arm set joint angles failed\n");
        return -1;
    }
    return 0;
}

// 启动主从跟随功能
int arm_master_slave_start(int role)
{
    // 检查是否已经在运行
    if (g_master_slave_manager.status == ARM_MASTER_SLAVE_STATUS_RUNNING) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master slave follow task is already running\n");
        return -1;
    }

    // 设置运行状态
    g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_RUNNING;
    g_error = 0;

    osThreadAttr_t task_attributes = {
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };

    // 根据角色创建不同任务
    if (role == ARM_ROLE_MASTER) {
        // 主臂任务
        g_master_slave_manager.role = ARM_ROLE_MASTER;
        task_attributes.name = "arm_master_task";
        g_master_slave_manager.task_handle = osThreadNew(arm_master_task, NULL, &task_attributes);
        if (g_master_slave_manager.task_handle == NULL) {
            ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Failed to create master arm task\n");
            g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_IDLE;
            return -1;
        }
        safe_printf("[MASTER] Master arm task created successfully\n");
    }
    else if (role == ARM_ROLE_SLAVE) {
        // 从臂模式下，控制逻辑在 arm_slave_set_angles() 中直接处理
        // 无需创建任务，仅设置状态
        g_master_slave_manager.role = ARM_ROLE_SLAVE;
        g_master_slave_manager.task_handle = NULL;
        safe_printf("[SLAVE] Slave arm mode active.\n");
    }
    else {
        ARM_LOGE_TAG(
            ARM_MASTER_SLAVE_LOG_TAG,
            "Invalid role: %d (1=Master arm, 2=Slave arm)\n",
            role
        );
        g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_IDLE;
        return -1;
    }

    // 更新机械臂状态
    g_arm_robot.tasks_handle = g_master_slave_manager.task_handle;
    g_arm_robot.state = ARM_STATE_MASTER_SLAVE;

    return 0;
}

// 停止主从跟随功能
int arm_master_slave_set_frequency_hz(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master slave frequency must be greater than 0 Hz\n");
        return -1;
    }

    uint32_t sample_period_ms = 1000U / freq_hz;
    if (sample_period_ms == 0) {
        sample_period_ms = 1;
    }

    g_master_slave_manager.sample_period_ms = sample_period_ms;
    safe_printf(
        "[MASTER_SLAVE] Frequency set to %lu Hz (sample period %lu ms)\n",
        (unsigned long)freq_hz,
        (unsigned long)sample_period_ms
    );
    return 0;
}

int arm_master_slave_stop(void)
{
    if (g_master_slave_manager.status != ARM_MASTER_SLAVE_STATUS_RUNNING) {
        ARM_LOGW_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master slave follow task is not running\n");
        return 0;
    }

    safe_printf("[MASTER_SLAVE] Stopping master slave follow task...\n");
    g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_STOP;

    if (g_master_slave_manager.role == ARM_ROLE_SLAVE) {
        g_filter_initialized = false;
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            g_prev_filtered_angles[i] = 0;
        }
        g_executed_angles_initialized = false;
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            g_prev_executed_angles[i] = 0;
        }
        g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_IDLE;
        g_arm_robot.state = ARM_STATE_IDLE;
        g_arm_robot.tasks_handle = NULL;
        send_ack_completed(CMD_ID_MASTER_SLAVE, g_error);
    }

    return 0;
}

// 获取主从跟随当前状态（内部使用）
ArmMasterSlaveStatus arm_master_slave_get_status(void)
{
    return g_master_slave_manager.status;
}

ArmRole arm_master_slave_get_role(void)
{
    return g_master_slave_manager.role;
}

void arm_master_slave_set_lpf_coefficient(float alpha)
{
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    g_master_slave_manager.lpf_coefficient = alpha;
    safe_printf("[MASTER_SLAVE] LPF coefficient set to %.2f\n", alpha);
}

float arm_master_slave_get_lpf_coefficient(void)
{
    return g_master_slave_manager.lpf_coefficient;
}

// 从臂接收上位机发送的关节角数据，完成增量滤波后直接控制关节角度
void arm_slave_set_angles(float angles[ARM_JOINTS_NUM])
{
    if (g_master_slave_manager.role != ARM_ROLE_SLAVE ||
        g_master_slave_manager.status != ARM_MASTER_SLAVE_STATUS_RUNNING) {
        return;
    }

    float filtered_angles[ARM_JOINTS_NUM];
    float alpha = g_master_slave_manager.lpf_coefficient;

    if (!g_filter_initialized) {
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            g_prev_filtered_angles[i] = angles[i];
            filtered_angles[i] = angles[i];
        }
        g_filter_initialized = true;
    } else {
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            filtered_angles[i] = alpha * angles[i] + (1.0f - alpha) * g_prev_filtered_angles[i];
            g_prev_filtered_angles[i] = filtered_angles[i];
        }
    }

    float limited_angles[ARM_JOINTS_NUM];
    if (!g_executed_angles_initialized) {
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            g_prev_executed_angles[i] = filtered_angles[i];
            limited_angles[i] = filtered_angles[i];
        }
        g_executed_angles_initialized = true;
    } else {
        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            float angle_change = filtered_angles[i] - g_prev_executed_angles[i];
            
            if (angle_change > ARM_MASTER_SLAVE_MAX_ANGLE_CHANGE) {
                limited_angles[i] = g_prev_executed_angles[i] + ARM_MASTER_SLAVE_MAX_ANGLE_CHANGE;
            } else if (angle_change < -ARM_MASTER_SLAVE_MAX_ANGLE_CHANGE) {
                limited_angles[i] = g_prev_executed_angles[i] - ARM_MASTER_SLAVE_MAX_ANGLE_CHANGE;
            } else {
                limited_angles[i] = filtered_angles[i];
            }
            
            g_prev_executed_angles[i] = limited_angles[i];
        }
    }

    arm_joint_sync_move(limited_angles);
}

// 主臂任务函数：主臂进入卸力模式，循环读取关节角并打印供上位机采集
static void arm_master_task(void *argument)
{
    int ret = 0;
    float master_angles[ARM_JOINTS_NUM] = {0};
    uint32_t consecutive_read_errors = 0U;
    TickType_t xLastWakeTime;

    safe_printf("[MASTER] Master arm task started\n");

    // 主臂进入卸力模式
    safe_printf("[MASTER] Master arm enters unload mode...\n");
    ret = arm_set_all_joint_stop(ARM_JOINT_STOP_UNLOAD_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master arm unload mode failed\n");
        g_error = -1;
        goto EXIT;
    }
    safe_printf("[MASTER] Master arm unload mode success, can be manually dragged\n");

    safe_printf("[MASTER] Ready to start collecting joint angles...\n");
    
    // 初始化定时器
    xLastWakeTime = xTaskGetTickCount();

    int frame_id = 0;

    // 主循环
    while (g_master_slave_manager.status == ARM_MASTER_SLAVE_STATUS_RUNNING) {
        // 同步读取所有关节角度
        ret = arm_master_read_joint_angles(master_angles);
        if (ret != 0) {
            consecutive_read_errors++;
            if ((consecutive_read_errors == 1U) || ((consecutive_read_errors % 10U) == 0U)) {
                ARM_LOGE_TAG(
                    ARM_MASTER_SLAVE_LOG_TAG,
                    "Master arm skipped [MD] frame after snapshot failure, consecutive_errors=%lu\n",
                    (unsigned long)consecutive_read_errors
                );
            }
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(g_master_slave_manager.sample_period_ms));
            continue;
        }
        consecutive_read_errors = 0U;

        // 打印关节角度数据（上位机会采集这些数据）
        safe_printf("[MD][%d][%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n",
                    frame_id,
                    master_angles[0], master_angles[1], master_angles[2], master_angles[3],
                    master_angles[4], master_angles[5], master_angles[6]);
        frame_id = (frame_id + 1) % 10;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(g_master_slave_manager.sample_period_ms));
    }

EXIT:
    // 恢复主臂锁定模式
    safe_printf("[MASTER] Master arm returns to lock mode...\n");
    ret = arm_set_all_joint_stop(ARM_JOINT_STOP_LOCK_MODE, 0);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master arm lock mode failed\n");
        g_error = -1;
    }

    // 更新主臂当前位置
    {
        float snapshot[ARM_JOINTS_NUM] = {0};
        if (arm_joint_snapshot_read(true, snapshot) != 0) {
            ARM_LOGW_TAG(ARM_MASTER_SLAVE_LOG_TAG, "Master arm final snapshot update failed\n");
        }
    }

    safe_printf("[MASTER] Master arm task ended\n");
    
    // 清理状态
    g_master_slave_manager.status = ARM_MASTER_SLAVE_STATUS_IDLE;
    g_master_slave_manager.task_handle = NULL;
    g_arm_robot.state = ARM_STATE_IDLE;
    g_arm_robot.tasks_handle = NULL;

    // 发送完成响应
    send_ack_completed(CMD_ID_MASTER_SLAVE, g_error);

    // 删除任务
    vTaskDelete(NULL);
}
