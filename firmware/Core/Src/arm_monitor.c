#include "arm_monitor.h"
#include "arm_first_use_runtime.h"
#include "arm_shell.h"
#include "w2812.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ARM_ROBOT_LOG_TAG "ARM_ROBOT"
#define ARM_MONITOR_RECOVERY_DELAY_MS 20U
#define ARM_MONITOR_TEMPERATURE_WARN_C 40
#define ARM_MONITOR_TEMPERATURE_FILTER_CYCLES (120U * 2U)
#define ARM_DUAL_SERVO_JOINT_NUM 2U
#define ARM_SERVO_STATUS_EXECUTING_MASK 0x01U
#define ARM_SERVO_STATUS_STALL_MASK 0x04U

typedef struct {
    uint8_t joint_id;
    uint8_t primary_servo_id;
    uint8_t secondary_servo_id;
} ArmDualServoJointConfig;

typedef struct {
    bool valid;
    float start_angle;
    uint32_t start_tick;
} ArmDualServoStagnantWindow;

typedef struct {
    bool falling_edge;
    bool angle_stagnant;
    uint32_t stagnant_window_ms;
    float stagnant_delta_deg;
} ArmDualServoMonitorUpdate;

typedef struct {
    bool active;
    bool observed_executing;
    bool primary_status_valid;
    bool secondary_status_valid;
    uint8_t joint_id;
    uint8_t primary_servo_id;
    uint8_t secondary_servo_id;
    float target_angle;
    float primary_target_servo_angle;
    float secondary_target_servo_angle;
    uint32_t command_generation;
    uint32_t start_tick;
    ServoData primary_last_status;
    ServoData secondary_last_status;
    ArmDualServoStagnantWindow primary_stagnant_window;
    ArmDualServoStagnantWindow secondary_stagnant_window;
} ArmDualServoMonitorSession;

typedef struct {
    bool active;
    bool observing;
    bool primary_status_valid;
    bool secondary_status_valid;
    uint8_t joint_id;
    uint8_t primary_servo_id;
    uint8_t secondary_servo_id;
    float target_angle;
    float primary_target_servo_angle;
    float secondary_target_servo_angle;
    uint32_t command_generation;
    uint32_t last_frame_tick;
    ServoData primary_last_status;
    ServoData secondary_last_status;
    ArmDualServoStagnantWindow primary_still_window;
    ArmDualServoStagnantWindow secondary_still_window;
} ArmDualServoStreamIdleSession;

static const uint8_t g_arm_all_servo_ids[ARM_SERVO_NUM] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U};
static const ArmDualServoJointConfig g_dual_servo_joint_configs[ARM_DUAL_SERVO_JOINT_NUM] = {
    {.joint_id = 1U, .primary_servo_id = 2U, .secondary_servo_id = 3U},
    {.joint_id = 2U, .primary_servo_id = 4U, .secondary_servo_id = 5U},
};
static ArmDualServoMonitorSession g_dual_servo_monitor_sessions[ARM_DUAL_SERVO_JOINT_NUM];
static ArmDualServoStreamIdleSession g_dual_servo_stream_sessions[ARM_DUAL_SERVO_JOINT_NUM];
static volatile uint32_t g_dual_servo_command_generation = 0U;
static bool g_arm_monitor_health_started = false;
static uint32_t g_arm_monitor_last_health_tick = 0U;
static osThreadId_t g_arm_monitor_task_handle = NULL;
static osMutexId_t g_dual_servo_monitor_mutex = NULL;

static void arm_dual_servo_monitor_mutex_init_once(void)
{
    static const osMutexAttr_t mutex_attr = {
        .name = "dualServoMonitor"
    };

    if (g_dual_servo_monitor_mutex == NULL) {
        g_dual_servo_monitor_mutex = osMutexNew(&mutex_attr);
        if (g_dual_servo_monitor_mutex == NULL) {
            ARM_LOGE_TAG(ARM_ROBOT_LOG_TAG, "Failed to create dual servo monitor mutex\n");
        }
    }
}

static void arm_dual_servo_monitor_lock(void)
{
    arm_dual_servo_monitor_mutex_init_once();
    if (g_dual_servo_monitor_mutex != NULL) {
        (void)osMutexAcquire(g_dual_servo_monitor_mutex, osWaitForever);
    }
}

static void arm_dual_servo_monitor_unlock(void)
{
    if (g_dual_servo_monitor_mutex != NULL) {
        (void)osMutexRelease(g_dual_servo_monitor_mutex);
    }
}

void arm_monitor_init(void)
{
    arm_dual_servo_monitor_mutex_init_once();
}

void arm_monitor_set_task_handle(osThreadId_t task_handle)
{
    arm_monitor_init();
    g_arm_monitor_task_handle = task_handle;
}

static int arm_dual_servo_config_index_from_joint(int joint_id)
{
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        if ((int)g_dual_servo_joint_configs[i].joint_id == joint_id) {
            return (int)i;
        }
    }

    return -1;
}

static void arm_dual_servo_notify_monitor_thread(void)
{
    if (g_arm_monitor_task_handle != NULL) {
        (void)osThreadFlagsSet(g_arm_monitor_task_handle, ARM_MONITOR_WAKE_FLAG);
    }
}

void arm_monitor_cancel_joint_motion(int joint_id)
{
    int index = arm_dual_servo_config_index_from_joint(joint_id);
    ArmDualServoMonitorSession *session;
    ArmDualServoStreamIdleSession *stream_session;
    uint32_t generation;

    if (index < 0) {
        return;
    }

    arm_dual_servo_monitor_lock();
    session = &g_dual_servo_monitor_sessions[index];
    stream_session = &g_dual_servo_stream_sessions[index];
    generation = ++g_dual_servo_command_generation;

    session->active = false;
    session->command_generation = generation;
    stream_session->active = false;
    stream_session->observing = false;
    stream_session->command_generation = generation;
    arm_dual_servo_monitor_unlock();
}

void arm_monitor_cancel_all_joint_motion(void)
{
    uint32_t generation;

    arm_dual_servo_monitor_lock();
    generation = ++g_dual_servo_command_generation;
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        g_dual_servo_monitor_sessions[i].active = false;
        g_dual_servo_monitor_sessions[i].command_generation = generation;
        g_dual_servo_stream_sessions[i].active = false;
        g_dual_servo_stream_sessions[i].observing = false;
        g_dual_servo_stream_sessions[i].command_generation = generation;
    }
    arm_dual_servo_monitor_unlock();
}

void arm_monitor_on_joint_motion(
    int joint_id,
    float target_angle,
    float primary_target_servo_angle,
    float secondary_target_servo_angle,
    enum ArmMotionMonitorPolicy monitor_policy
)
{
    int index = arm_dual_servo_config_index_from_joint(joint_id);
    ArmDualServoMonitorSession *session;
    ArmDualServoStreamIdleSession *stream_session;
    uint32_t generation;
    uint32_t now;
    bool should_notify = false;

    if (index < 0) {
        return;
    }

    arm_dual_servo_monitor_lock();
    session = &g_dual_servo_monitor_sessions[index];
    stream_session = &g_dual_servo_stream_sessions[index];
    generation = ++g_dual_servo_command_generation;
    now = osKernelGetTickCount();

    if (monitor_policy == ARM_MOTION_MONITOR_DISABLED) {
        session->active = false;
        session->command_generation = generation;
        stream_session->active = false;
        stream_session->observing = false;
        stream_session->command_generation = generation;
        arm_dual_servo_monitor_unlock();
        return;
    }

    if (monitor_policy == ARM_MOTION_MONITOR_STREAM_IDLE_HOLD) {
        session->active = false;
        session->command_generation = generation;
        memset(stream_session, 0, sizeof(*stream_session));
        stream_session->joint_id = g_dual_servo_joint_configs[index].joint_id;
        stream_session->primary_servo_id = g_dual_servo_joint_configs[index].primary_servo_id;
        stream_session->secondary_servo_id = g_dual_servo_joint_configs[index].secondary_servo_id;
        stream_session->target_angle = target_angle;
        stream_session->primary_target_servo_angle = primary_target_servo_angle;
        stream_session->secondary_target_servo_angle = secondary_target_servo_angle;
        stream_session->command_generation = generation;
        stream_session->last_frame_tick = now;
        stream_session->active = true;
        arm_dual_servo_monitor_unlock();
        return;
    }

    if (monitor_policy != ARM_MOTION_MONITOR_AUTO_HOLD) {
        arm_dual_servo_monitor_unlock();
        return;
    }

    stream_session->active = false;
    stream_session->observing = false;
    stream_session->command_generation = generation;
    memset(session, 0, sizeof(*session));
    session->joint_id = g_dual_servo_joint_configs[index].joint_id;
    session->primary_servo_id = g_dual_servo_joint_configs[index].primary_servo_id;
    session->secondary_servo_id = g_dual_servo_joint_configs[index].secondary_servo_id;
    session->target_angle = target_angle;
    session->primary_target_servo_angle = primary_target_servo_angle;
    session->secondary_target_servo_angle = secondary_target_servo_angle;
    session->command_generation = generation;
    session->start_tick = now;
    session->active = true;
    should_notify = true;
    arm_dual_servo_monitor_unlock();

    if (should_notify) {
        arm_dual_servo_notify_monitor_thread();
    }
}

static bool arm_dual_servo_has_active_monitor(void)
{
    bool has_active = false;

    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        if (g_dual_servo_monitor_sessions[i].active) {
            has_active = true;
            break;
        }
    }
    arm_dual_servo_monitor_unlock();

    return has_active;
}

static uint32_t arm_dual_servo_elapsed_ms(const ArmDualServoMonitorSession *session)
{
    if (session == NULL) {
        return 0U;
    }

    return osKernelGetTickCount() - session->start_tick;
}

static bool arm_servo_status_bit_is_set(uint8_t status, uint8_t mask)
{
    return ((status & mask) != 0U);
}

static const char *arm_dual_servo_role_text(bool primary)
{
    return primary ? "primary" : "secondary";
}

static const ServoData *arm_find_servo_monitor_data(
    const ServoData *monitor_data,
    int data_count,
    uint8_t servo_id
) {
    if (monitor_data == NULL) {
        return NULL;
    }

    for (int i = 0; i < data_count; ++i) {
        if (monitor_data[i].id == servo_id) {
            return &monitor_data[i];
        }
    }

    return NULL;
}

static int arm_monitor_read_servo_ids(
    const uint8_t *servo_ids,
    int servo_count,
    ServoData *servodata
) {
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;

    if ((servo_ops == NULL) || (servo_ids == NULL) || (servodata == NULL) || (servo_count <= 0)) {
        return -1;
    }

    if (servo_ops->monitor_batch != NULL) {
        return servo_ops->monitor_batch(servo_ids, servo_count, servodata);
    }

    if (servo_ops->monitor == NULL) {
        return -1;
    }

    for (int i = 0; i < servo_count; ++i) {
        if (servo_ops->monitor((int)servo_ids[i], &servodata[i]) != 0) {
            return -1;
        }
    }

    return 0;
}

static void arm_dual_servo_log_edge(
    const ArmDualServoMonitorSession *session,
    const ServoData *previous,
    const ServoData *current,
    bool primary,
    const char *bit_name,
    bool old_value,
    bool new_value
) {
#if ARM_DUAL_SERVO_HOLD_LOG_ENABLE
    if ((session == NULL) || (previous == NULL) || (current == NULL) || (bit_name == NULL)) {
        return;
    }

    ARM_LOGI_TAG(
        ARM_ROBOT_LOG_TAG,
        "DUAL_SERVO_EDGE joint=%u servo=%u role=%s bit=%s %u->%u status=%u->%u angle=%.2f current=%d power=%d temp=%d elapsed=%lu target=%.2f\n",
        (unsigned int)session->joint_id,
        (unsigned int)current->id,
        arm_dual_servo_role_text(primary),
        bit_name,
        old_value ? 1U : 0U,
        new_value ? 1U : 0U,
        (unsigned int)previous->status,
        (unsigned int)current->status,
        current->angle,
        current->current,
        current->power,
        current->temperature,
        (unsigned long)arm_dual_servo_elapsed_ms(session),
        session->target_angle
    );
#else
    (void)session;
    (void)previous;
    (void)current;
    (void)primary;
    (void)bit_name;
    (void)old_value;
    (void)new_value;
#endif
}

static bool arm_dual_servo_update_angle_window(
    ArmDualServoStagnantWindow *window,
    float current_angle,
    uint32_t now,
    uint32_t window_limit_ms,
    float angle_limit_deg,
    uint32_t *window_ms,
    float *delta_deg
) {
    float delta;
    uint32_t elapsed;

    if ((window == NULL) || (window_ms == NULL) || (delta_deg == NULL)) {
        return false;
    }

    *window_ms = 0U;
    *delta_deg = 0.0f;

    if (!window->valid) {
        window->valid = true;
        window->start_angle = current_angle;
        window->start_tick = now;
        return false;
    }

    delta = fabsf(current_angle - window->start_angle);
    if (delta > angle_limit_deg) {
        window->start_angle = current_angle;
        window->start_tick = now;
        return false;
    }

    elapsed = now - window->start_tick;
    if (elapsed < window_limit_ms) {
        return false;
    }

    *window_ms = elapsed;
    *delta_deg = delta;
    return true;
}

static bool arm_dual_servo_update_stagnant_window(
    ArmDualServoStagnantWindow *window,
    const ServoData *current,
    bool executing,
    uint32_t now,
    uint32_t *window_ms,
    float *delta_deg
) {
    if ((window == NULL) || (current == NULL) || (window_ms == NULL) || (delta_deg == NULL)) {
        return false;
    }

    *window_ms = 0U;
    *delta_deg = 0.0f;

    if (!executing) {
        memset(window, 0, sizeof(*window));
        return false;
    }

    return arm_dual_servo_update_angle_window(
        window,
        current->angle,
        now,
        ARM_DUAL_SERVO_STAGNANT_WINDOW_MS,
        ARM_DUAL_SERVO_STAGNANT_ANGLE_DEG,
        window_ms,
        delta_deg
    );
}

static bool arm_dual_servo_is_near_target(
    const ServoData *current,
    float target_servo_angle
) {
    if (current == NULL) {
        return false;
    }

    return fabsf(current->angle - target_servo_angle) <= ARM_ANGLE_ERROR;
}

static bool arm_dual_servo_pair_is_near_target(
    const ArmDualServoMonitorSession *session,
    const ServoData *primary,
    const ServoData *secondary
) {
    if ((session == NULL) || (primary == NULL) || (secondary == NULL)) {
        return false;
    }

    return arm_dual_servo_is_near_target(primary, session->primary_target_servo_angle) &&
           arm_dual_servo_is_near_target(secondary, session->secondary_target_servo_angle);
}

static ArmDualServoMonitorUpdate arm_dual_servo_update_status(
    ArmDualServoMonitorSession *session,
    const ServoData *current,
    bool primary
) {
    ArmDualServoMonitorUpdate result = {0};
    ServoData *previous;
    bool *valid;
    ArmDualServoStagnantWindow *stagnant_window;
    bool executing;
    uint32_t now;

    if ((session == NULL) || (current == NULL)) {
        return result;
    }

    previous = primary ? &session->primary_last_status : &session->secondary_last_status;
    valid = primary ? &session->primary_status_valid : &session->secondary_status_valid;
    stagnant_window = primary ? &session->primary_stagnant_window : &session->secondary_stagnant_window;
    executing = arm_servo_status_bit_is_set(current->status, ARM_SERVO_STATUS_EXECUTING_MASK);
    now = osKernelGetTickCount();
    if (executing) {
        session->observed_executing = true;
    }

    if (*valid) {
        bool old_executing = arm_servo_status_bit_is_set(previous->status, ARM_SERVO_STATUS_EXECUTING_MASK);
        bool old_stall = arm_servo_status_bit_is_set(previous->status, ARM_SERVO_STATUS_STALL_MASK);
        bool new_stall = arm_servo_status_bit_is_set(current->status, ARM_SERVO_STATUS_STALL_MASK);

        if (old_executing != executing) {
            arm_dual_servo_log_edge(session, previous, current, primary, "executing", old_executing, executing);
        }

        if (old_stall != new_stall) {
            arm_dual_servo_log_edge(session, previous, current, primary, "stall", old_stall, new_stall);
        }

        result.falling_edge = session->observed_executing && old_executing && !executing;
    }

    result.angle_stagnant = arm_dual_servo_update_stagnant_window(
        stagnant_window,
        current,
        executing,
        now,
        &result.stagnant_window_ms,
        &result.stagnant_delta_deg
    );
    *previous = *current;
    *valid = true;
    return result;
}

static void arm_dual_servo_log_hold_summary(
    const ArmDualServoMonitorSession *session,
    const ServoData *primary,
    const ServoData *secondary,
    uint8_t trigger_servo_id,
    const char *trigger_role,
    const char *reason,
    uint32_t stagnant_window_ms,
    float stagnant_delta_deg
) {
#if ARM_DUAL_SERVO_HOLD_LOG_ENABLE
    if ((session == NULL) || (primary == NULL) || (secondary == NULL) ||
        (trigger_role == NULL) || (reason == NULL)) {
        return;
    }

    if (stagnant_window_ms > 0U) {
        ARM_LOGI_TAG(
            ARM_ROBOT_LOG_TAG,
            "DUAL_SERVO_HOLD j=%u trig=%u/%s reason=%s mode=lock pwr=%d t=%lu win=%lu da=%.2f "
            "P{id=%u,st=%u,a=%.2f,c=%d,p=%d,T=%d} S{id=%u,st=%u,a=%.2f,c=%d,p=%d,T=%d}\n",
            (unsigned int)session->joint_id,
            (unsigned int)trigger_servo_id,
            trigger_role,
            reason,
            ARM_DUAL_SERVO_HOLD_POWER,
            (unsigned long)arm_dual_servo_elapsed_ms(session),
            (unsigned long)stagnant_window_ms,
            stagnant_delta_deg,
            (unsigned int)primary->id,
            (unsigned int)primary->status,
            primary->angle,
            primary->current,
            primary->power,
            primary->temperature,
            (unsigned int)secondary->id,
            (unsigned int)secondary->status,
            secondary->angle,
            secondary->current,
            secondary->power,
            secondary->temperature
        );
        return;
    }

    ARM_LOGI_TAG(
        ARM_ROBOT_LOG_TAG,
        "DUAL_SERVO_HOLD j=%u trig=%u/%s reason=%s mode=lock pwr=%d t=%lu "
        "P{id=%u,st=%u,a=%.2f,c=%d,p=%d,T=%d} S{id=%u,st=%u,a=%.2f,c=%d,p=%d,T=%d}\n",
        (unsigned int)session->joint_id,
        (unsigned int)trigger_servo_id,
        trigger_role,
        reason,
        ARM_DUAL_SERVO_HOLD_POWER,
        (unsigned long)arm_dual_servo_elapsed_ms(session),
        (unsigned int)primary->id,
        (unsigned int)primary->status,
        primary->angle,
        primary->current,
        primary->power,
        primary->temperature,
        (unsigned int)secondary->id,
        (unsigned int)secondary->status,
        secondary->angle,
        secondary->current,
        secondary->power,
        secondary->temperature
    );
#else
    (void)session;
    (void)primary;
    (void)secondary;
    (void)trigger_servo_id;
    (void)trigger_role;
    (void)reason;
    (void)stagnant_window_ms;
    (void)stagnant_delta_deg;
#endif
}

static int arm_dual_servo_enter_hold(
    const ArmDualServoMonitorSession *session,
    const ServoData *primary,
    const ServoData *secondary,
    uint8_t trigger_servo_id,
    const char *trigger_role,
    const char *reason,
    uint32_t stagnant_window_ms,
    float stagnant_delta_deg
) {
    ArmServoOpt *servo_ops = g_arm_robot.servo_ops;
    int primary_ret;
    int secondary_ret;

    if ((session == NULL) || (servo_ops == NULL)) {
        return -1;
    }

    if (servo_ops->stop == NULL) {
        return -1;
    }

    primary_ret = servo_ops->stop((int)session->primary_servo_id, ARM_JOINT_STOP_LOCK_MODE, ARM_DUAL_SERVO_HOLD_POWER);
    secondary_ret = servo_ops->stop((int)session->secondary_servo_id, ARM_JOINT_STOP_LOCK_MODE, ARM_DUAL_SERVO_HOLD_POWER);

    if ((primary_ret != 0) || (secondary_ret != 0)) {
        ARM_LOGE_TAG(
            ARM_ROBOT_LOG_TAG,
            "Failed to enter dual servo hold: joint=%u primary_ret=%d secondary_ret=%d\n",
            (unsigned int)session->joint_id,
            primary_ret,
            secondary_ret
        );
        return (primary_ret != 0) ? primary_ret : secondary_ret;
    }

    arm_dual_servo_log_hold_summary(
        session,
        primary,
        secondary,
        trigger_servo_id,
        trigger_role,
        reason,
        stagnant_window_ms,
        stagnant_delta_deg
    );
    return 0;
}

static void arm_dual_servo_add_monitor_id(uint8_t *servo_ids, int *servo_count, uint8_t servo_id)
{
    if ((servo_ids == NULL) || (servo_count == NULL)) {
        return;
    }

    for (int i = 0; i < *servo_count; ++i) {
        if (servo_ids[i] == servo_id) {
            return;
        }
    }

    if (*servo_count < (int)(ARM_DUAL_SERVO_JOINT_NUM * 2U)) {
        servo_ids[*servo_count] = servo_id;
        (*servo_count)++;
    }
}

static void arm_dual_servo_fast_monitor(void)
{
    uint8_t servo_ids[ARM_DUAL_SERVO_JOINT_NUM * 2U] = {0};
    uint32_t captured_generations[ARM_DUAL_SERVO_JOINT_NUM] = {0};
    ServoData monitor_data[ARM_DUAL_SERVO_JOINT_NUM * 2U] = {0};
    int servo_count = 0;

    if (!arm_dual_servo_has_active_monitor()) {
        return;
    }

    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        const ArmDualServoMonitorSession *session = &g_dual_servo_monitor_sessions[i];
        if (!session->active) {
            continue;
        }

        captured_generations[i] = session->command_generation;
        arm_dual_servo_add_monitor_id(servo_ids, &servo_count, session->primary_servo_id);
        arm_dual_servo_add_monitor_id(servo_ids, &servo_count, session->secondary_servo_id);
    }
    arm_dual_servo_monitor_unlock();

    if ((servo_count <= 0) || (arm_monitor_read_servo_ids(servo_ids, servo_count, monitor_data) != 0)) {
        return;
    }

    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        ArmDualServoMonitorSession *session = &g_dual_servo_monitor_sessions[i];
        const ServoData *primary;
        const ServoData *secondary;
        ArmDualServoMonitorUpdate primary_update;
        ArmDualServoMonitorUpdate secondary_update;
        uint8_t trigger_servo_id = 0U;
        const char *trigger_role = "";
        const char *reason = "";
        uint32_t stagnant_window_ms = 0U;
        float stagnant_delta_deg = 0.0f;
        bool near_target;
        bool primary_executing;
        bool secondary_executing;

        arm_dual_servo_monitor_lock();
        if (!session->active || (session->command_generation != captured_generations[i])) {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        primary = arm_find_servo_monitor_data(monitor_data, servo_count, session->primary_servo_id);
        secondary = arm_find_servo_monitor_data(monitor_data, servo_count, session->secondary_servo_id);
        if ((primary == NULL) || (secondary == NULL)) {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        primary_update = arm_dual_servo_update_status(session, primary, true);
        secondary_update = arm_dual_servo_update_status(session, secondary, false);
        near_target = arm_dual_servo_pair_is_near_target(session, primary, secondary);
        primary_executing = arm_servo_status_bit_is_set(primary->status, ARM_SERVO_STATUS_EXECUTING_MASK);
        secondary_executing = arm_servo_status_bit_is_set(secondary->status, ARM_SERVO_STATUS_EXECUTING_MASK);

        if (!near_target) {
            if (!primary_executing && !secondary_executing) {
                session->active = false;
            }
            arm_dual_servo_monitor_unlock();
            continue;
        }

        if (primary_update.falling_edge) {
            trigger_servo_id = session->primary_servo_id;
            trigger_role = arm_dual_servo_role_text(true);
            reason = "target-reached";
        } else if (secondary_update.falling_edge) {
            trigger_servo_id = session->secondary_servo_id;
            trigger_role = arm_dual_servo_role_text(false);
            reason = "target-reached";
        } else if (primary_update.angle_stagnant) {
            trigger_servo_id = session->primary_servo_id;
            trigger_role = arm_dual_servo_role_text(true);
            reason = "near-target-stable";
            stagnant_window_ms = primary_update.stagnant_window_ms;
            stagnant_delta_deg = primary_update.stagnant_delta_deg;
        } else if (secondary_update.angle_stagnant) {
            trigger_servo_id = session->secondary_servo_id;
            trigger_role = arm_dual_servo_role_text(false);
            reason = "near-target-stable";
            stagnant_window_ms = secondary_update.stagnant_window_ms;
            stagnant_delta_deg = secondary_update.stagnant_delta_deg;
        } else if (!primary_executing && !secondary_executing) {
            trigger_servo_id = session->primary_servo_id;
            trigger_role = arm_dual_servo_role_text(true);
            reason = "target-reached";
        } else {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        if (arm_dual_servo_enter_hold(
                session,
                primary,
                secondary,
                trigger_servo_id,
                trigger_role,
                reason,
                stagnant_window_ms,
                stagnant_delta_deg
            ) == 0) {
            session->active = false;
        }
        arm_dual_servo_monitor_unlock();
    }
}

static bool arm_dual_servo_has_stream_observation(void)
{
    bool has_observation = false;

    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        if (g_dual_servo_stream_sessions[i].active &&
            g_dual_servo_stream_sessions[i].observing) {
            has_observation = true;
            break;
        }
    }
    arm_dual_servo_monitor_unlock();

    return has_observation;
}

static void arm_dual_servo_make_stream_monitor_session(
    const ArmDualServoStreamIdleSession *stream_session,
    ArmDualServoMonitorSession *monitor_session
) {
    if ((stream_session == NULL) || (monitor_session == NULL)) {
        return;
    }

    memset(monitor_session, 0, sizeof(*monitor_session));
    monitor_session->active = stream_session->active;
    monitor_session->joint_id = stream_session->joint_id;
    monitor_session->primary_servo_id = stream_session->primary_servo_id;
    monitor_session->secondary_servo_id = stream_session->secondary_servo_id;
    monitor_session->target_angle = stream_session->target_angle;
    monitor_session->primary_target_servo_angle = stream_session->primary_target_servo_angle;
    monitor_session->secondary_target_servo_angle = stream_session->secondary_target_servo_angle;
    monitor_session->command_generation = stream_session->command_generation;
    monitor_session->start_tick = stream_session->last_frame_tick;
}

static void arm_dual_servo_stream_start_due_observations(uint32_t now)
{
    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        ArmDualServoStreamIdleSession *stream_session = &g_dual_servo_stream_sessions[i];

        if (!stream_session->active || stream_session->observing) {
            continue;
        }

        if ((now - stream_session->last_frame_tick) < ARM_DUAL_SERVO_STREAM_IDLE_MS) {
            continue;
        }

        stream_session->observing = true;
        stream_session->primary_status_valid = false;
        stream_session->secondary_status_valid = false;
        memset(&stream_session->primary_still_window, 0, sizeof(stream_session->primary_still_window));
        memset(&stream_session->secondary_still_window, 0, sizeof(stream_session->secondary_still_window));
    }
    arm_dual_servo_monitor_unlock();
}

static void arm_dual_servo_stream_log_status_edges(
    ArmDualServoStreamIdleSession *stream_session,
    const ArmDualServoMonitorSession *log_session,
    const ServoData *current,
    bool primary
) {
    ServoData *previous;
    bool *valid;
    bool executing;

    if ((stream_session == NULL) || (log_session == NULL) || (current == NULL)) {
        return;
    }

    previous = primary ? &stream_session->primary_last_status : &stream_session->secondary_last_status;
    valid = primary ? &stream_session->primary_status_valid : &stream_session->secondary_status_valid;
    executing = arm_servo_status_bit_is_set(current->status, ARM_SERVO_STATUS_EXECUTING_MASK);

    if (*valid) {
        bool old_executing = arm_servo_status_bit_is_set(previous->status, ARM_SERVO_STATUS_EXECUTING_MASK);
        bool old_stall = arm_servo_status_bit_is_set(previous->status, ARM_SERVO_STATUS_STALL_MASK);
        bool new_stall = arm_servo_status_bit_is_set(current->status, ARM_SERVO_STATUS_STALL_MASK);

        if (old_executing != executing) {
            arm_dual_servo_log_edge(log_session, previous, current, primary, "executing", old_executing, executing);
        }

        if (old_stall != new_stall) {
            arm_dual_servo_log_edge(log_session, previous, current, primary, "stall", old_stall, new_stall);
        }
    }

    *previous = *current;
    *valid = true;
}

static bool arm_dual_servo_stream_update_still_window(
    ArmDualServoStagnantWindow *window,
    const ServoData *current,
    uint32_t now,
    uint32_t *window_ms,
    float *delta_deg
) {
    if ((window == NULL) || (current == NULL) || (window_ms == NULL) || (delta_deg == NULL)) {
        return false;
    }

    return arm_dual_servo_update_angle_window(
        window,
        current->angle,
        now,
        ARM_DUAL_SERVO_STREAM_STILL_WINDOW_MS,
        ARM_DUAL_SERVO_STREAM_STILL_ANGLE_DEG,
        window_ms,
        delta_deg
    );
}

static void arm_dual_servo_stream_idle_monitor(uint32_t now)
{
    uint8_t servo_ids[ARM_DUAL_SERVO_JOINT_NUM * 2U] = {0};
    uint32_t captured_generations[ARM_DUAL_SERVO_JOINT_NUM] = {0};
    ServoData monitor_data[ARM_DUAL_SERVO_JOINT_NUM * 2U] = {0};
    int servo_count = 0;

    arm_dual_servo_stream_start_due_observations(now);
    if (!arm_dual_servo_has_stream_observation()) {
        return;
    }

    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        const ArmDualServoStreamIdleSession *stream_session = &g_dual_servo_stream_sessions[i];

        if (!stream_session->active || !stream_session->observing) {
            continue;
        }

        captured_generations[i] = stream_session->command_generation;
        arm_dual_servo_add_monitor_id(servo_ids, &servo_count, stream_session->primary_servo_id);
        arm_dual_servo_add_monitor_id(servo_ids, &servo_count, stream_session->secondary_servo_id);
    }
    arm_dual_servo_monitor_unlock();

    if ((servo_count <= 0) || (arm_monitor_read_servo_ids(servo_ids, servo_count, monitor_data) != 0)) {
        return;
    }

    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        ArmDualServoStreamIdleSession *stream_session = &g_dual_servo_stream_sessions[i];
        ArmDualServoMonitorSession log_session;
        const ServoData *primary;
        const ServoData *secondary;
        bool primary_still;
        bool secondary_still;
        uint32_t primary_window_ms = 0U;
        uint32_t secondary_window_ms = 0U;
        float primary_delta_deg = 0.0f;
        float secondary_delta_deg = 0.0f;
        uint32_t hold_window_ms;
        float hold_delta_deg;

        arm_dual_servo_monitor_lock();
        if (!stream_session->active ||
            !stream_session->observing ||
            (stream_session->command_generation != captured_generations[i])) {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        primary = arm_find_servo_monitor_data(monitor_data, servo_count, stream_session->primary_servo_id);
        secondary = arm_find_servo_monitor_data(monitor_data, servo_count, stream_session->secondary_servo_id);
        if ((primary == NULL) || (secondary == NULL)) {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        arm_dual_servo_make_stream_monitor_session(stream_session, &log_session);
        arm_dual_servo_stream_log_status_edges(stream_session, &log_session, primary, true);
        arm_dual_servo_stream_log_status_edges(stream_session, &log_session, secondary, false);
        primary_still = arm_dual_servo_stream_update_still_window(
            &stream_session->primary_still_window,
            primary,
            now,
            &primary_window_ms,
            &primary_delta_deg
        );
        secondary_still = arm_dual_servo_stream_update_still_window(
            &stream_session->secondary_still_window,
            secondary,
            now,
            &secondary_window_ms,
            &secondary_delta_deg
        );

        if (!primary_still || !secondary_still) {
            arm_dual_servo_monitor_unlock();
            continue;
        }

        if (!arm_dual_servo_pair_is_near_target(&log_session, primary, secondary)) {
            stream_session->active = false;
            stream_session->observing = false;
            arm_dual_servo_monitor_unlock();
            continue;
        }

        hold_window_ms = (primary_window_ms < secondary_window_ms) ? primary_window_ms : secondary_window_ms;
        hold_delta_deg = (primary_delta_deg > secondary_delta_deg) ? primary_delta_deg : secondary_delta_deg;
        if (arm_dual_servo_enter_hold(
                &log_session,
                primary,
                secondary,
                stream_session->primary_servo_id,
                arm_dual_servo_role_text(true),
                "stream-idle-still",
                hold_window_ms,
                hold_delta_deg
            ) == 0) {
            stream_session->active = false;
            stream_session->observing = false;
        }
        arm_dual_servo_monitor_unlock();
    }
}

static uint32_t arm_dual_servo_stream_next_idle_delay_ms(uint32_t now, bool *has_stream_delay)
{
    uint32_t best_delay = 0U;

    if (has_stream_delay == NULL) {
        return 1U;
    }

    *has_stream_delay = false;
    arm_dual_servo_monitor_lock();
    for (uint32_t i = 0U; i < ARM_DUAL_SERVO_JOINT_NUM; ++i) {
        const ArmDualServoStreamIdleSession *stream_session = &g_dual_servo_stream_sessions[i];
        uint32_t elapsed;
        uint32_t delay;

        if (!stream_session->active || stream_session->observing) {
            continue;
        }

        elapsed = now - stream_session->last_frame_tick;
        delay = (elapsed >= ARM_DUAL_SERVO_STREAM_IDLE_MS) ?
            1U :
            (ARM_DUAL_SERVO_STREAM_IDLE_MS - elapsed);
        if (!*has_stream_delay || (delay < best_delay)) {
            best_delay = delay;
            *has_stream_delay = true;
        }
    }
    arm_dual_servo_monitor_unlock();

    return *has_stream_delay ? best_delay : 1U;
}

static void arm_robot_update_light(void)
{
    static float brightness = 0.8f;
    static int init = 0;
    static int last_servo_error_flag = 0;

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
        W2812_SetAllColors(255 * brightness, 0, 0);
    } else if (g_arm_robot.servo_error_flag == 0) {
        W2812_SetAllColors(0, 255 * brightness, 0);
    }

    W2812_SendData(false);
}

static int arm_monitor_batch_read(ServoData servodata[ARM_SERVO_NUM])
{
    return arm_monitor_read_servo_ids(g_arm_all_servo_ids, ARM_SERVO_NUM, servodata);
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

static void arm_monitor_health_once(void)
{
    static uint8_t consecutive_batch_failures = 0U;
    static uint16_t last_failed_servo_mask = 0xFFFFU;
    ServoData monitor_data[ARM_SERVO_NUM] = {0};

    if (arm_monitor_batch_read(monitor_data) == 0) {
        for (int i = 0; i < ARM_SERVO_NUM; ++i) {
            arm_monitor_handle_servo_status(g_arm_all_servo_ids[i], &monitor_data[i]);
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

void arm_monitor(void)
{
    uint32_t now = osKernelGetTickCount();

    arm_first_use_runtime_tick(now);

    if (arm_dual_servo_has_active_monitor()) {
        arm_dual_servo_fast_monitor();
    }
    arm_dual_servo_stream_idle_monitor(now);

    if (!g_arm_monitor_health_started ||
        ((now - g_arm_monitor_last_health_tick) >= ARM_MONITOR_HEALTH_PERIOD_MS)) {
        g_arm_monitor_health_started = true;
        g_arm_monitor_last_health_tick = now;
        arm_monitor_health_once();
    }
}

uint32_t arm_monitor_get_next_delay_ms(void)
{
    uint32_t now;
    uint32_t elapsed;
    uint32_t health_delay;
    uint32_t stream_delay;
    bool has_stream_delay = false;

    if (arm_dual_servo_has_active_monitor() || arm_dual_servo_has_stream_observation()) {
        return ARM_MONITOR_FAST_PERIOD_MS;
    }

    if (!g_arm_monitor_health_started) {
        return 1U;
    }

    now = osKernelGetTickCount();
    elapsed = now - g_arm_monitor_last_health_tick;
    if (elapsed >= ARM_MONITOR_HEALTH_PERIOD_MS) {
        return 1U;
    }

    health_delay = ARM_MONITOR_HEALTH_PERIOD_MS - elapsed;
    stream_delay = arm_dual_servo_stream_next_idle_delay_ms(now, &has_stream_delay);
    if (has_stream_delay && (stream_delay < health_delay)) {
        return stream_delay;
    }

    return health_delay;
}
