#include "arm_status_report.h"

#include "arm_robot.h"
#include "arm_shell.h"
#include "cmsis_os2.h"
#include "task.h"

#define ARM_STATUS_REPORT_TASK_STACK_SIZE (1024U * 4U)
#define ARM_STATUS_REPORT_STOP_TIMEOUT_MS 1000U
#define ARM_STATUS_REPORT_LOG_TAG "STATUS_REPORT"

typedef struct {
    osThreadId_t task_handle;
    uint32_t sample_period_ms;
    uint32_t frequency_hz;
    volatile uint8_t running;
    volatile uint8_t stop_requested;
} ArmStatusReportManager;

static ArmStatusReportManager g_status_report_manager = {
    .task_handle = NULL,
    .sample_period_ms = 1000U / ARM_STATUS_REPORT_DEFAULT_FREQ_HZ,
    .frequency_hz = ARM_STATUS_REPORT_DEFAULT_FREQ_HZ,
    .running = 0U,
    .stop_requested = 0U,
};

static uint32_t arm_status_report_calc_period_ms(uint32_t freq_hz)
{
    uint32_t sample_period_ms = 1000U / freq_hz;

    if (sample_period_ms == 0U) {
        sample_period_ms = 1U;
    }

    return sample_period_ms;
}

static void arm_status_report_cleanup(void)
{
    taskENTER_CRITICAL();
    g_status_report_manager.running = 0U;
    g_status_report_manager.stop_requested = 0U;
    g_status_report_manager.task_handle = NULL;
    taskEXIT_CRITICAL();
}

static void arm_status_report_task(void *argument)
{
    uint32_t consecutive_errors = 0U;
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)argument;

    safe_printf(
        "[STATUS_REPORT] started at %lu Hz\n",
        (unsigned long)g_status_report_manager.frequency_hz
    );

    while (g_status_report_manager.stop_requested == 0U) {
        float joint_snapshot[ARM_JOINTS_NUM] = {0};
        int ret = arm_joint_snapshot_read(true, joint_snapshot);
        if (ret == 0) {
            consecutive_errors = 0U;
            arm_print_status_frame(joint_snapshot);
        } else {
            consecutive_errors++;
            if ((consecutive_errors == 1U) || ((consecutive_errors % 10U) == 0U)) {
                ARM_LOGE_TAG(
                    ARM_STATUS_REPORT_LOG_TAG,
                    "status_report skipped cycle after read failure, consecutive_errors=%lu\n",
                    (unsigned long)consecutive_errors
                );
            }
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(g_status_report_manager.sample_period_ms)
        );
    }

    safe_printf("[STATUS_REPORT] stopped\n");
    arm_status_report_cleanup();
    vTaskDelete(NULL);
}

int arm_status_report_start(uint32_t freq_hz)
{
    osThreadAttr_t task_attributes = {
        .name = "arm_status_report",
        .stack_size = ARM_STATUS_REPORT_TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityLow,
    };

    if ((freq_hz < ARM_STATUS_REPORT_MIN_FREQ_HZ) ||
        (freq_hz > ARM_STATUS_REPORT_MAX_FREQ_HZ)) {
        ARM_LOGE_TAG(
            ARM_STATUS_REPORT_LOG_TAG,
            "status_report frequency must be within [%lu, %lu] Hz\n",
            (unsigned long)ARM_STATUS_REPORT_MIN_FREQ_HZ,
            (unsigned long)ARM_STATUS_REPORT_MAX_FREQ_HZ
        );
        return -1;
    }

    taskENTER_CRITICAL();
    g_status_report_manager.frequency_hz = freq_hz;
    g_status_report_manager.sample_period_ms = arm_status_report_calc_period_ms(freq_hz);
    g_status_report_manager.stop_requested = 0U;

    if (g_status_report_manager.running != 0U) {
        taskEXIT_CRITICAL();
        safe_printf(
            "[STATUS_REPORT] updated frequency to %lu Hz\n",
            (unsigned long)freq_hz
        );
        return 0;
    }

    g_status_report_manager.running = 1U;
    taskEXIT_CRITICAL();

    g_status_report_manager.task_handle = osThreadNew(
        arm_status_report_task,
        NULL,
        &task_attributes
    );
    if (g_status_report_manager.task_handle == NULL) {
        arm_status_report_cleanup();
        ARM_LOGE_TAG(ARM_STATUS_REPORT_LOG_TAG, "Failed to create status_report task\n");
        return -1;
    }

    return 0;
}

int arm_status_report_stop(void)
{
    uint32_t start_tick;

    if (g_status_report_manager.running == 0U) {
        return 0;
    }

    g_status_report_manager.stop_requested = 1U;
    start_tick = osKernelGetTickCount();

    while (g_status_report_manager.running != 0U) {
        if ((osKernelGetTickCount() - start_tick) >= ARM_STATUS_REPORT_STOP_TIMEOUT_MS) {
            ARM_LOGE_TAG(ARM_STATUS_REPORT_LOG_TAG, "status_report stop timeout\n");
            return -1;
        }
        osDelay(10U);
    }

    return 0;
}

bool arm_status_report_is_running(void)
{
    return g_status_report_manager.running != 0U;
}

uint32_t arm_status_report_get_frequency_hz(void)
{
    return g_status_report_manager.frequency_hz;
}
