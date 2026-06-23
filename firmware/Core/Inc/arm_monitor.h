#ifndef __ARM_MONITOR_H__
#define __ARM_MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"
#include "arm_robot.h"

void arm_monitor_init(void);
void arm_monitor_set_task_handle(osThreadId_t task_handle);
void arm_monitor_cancel_joint_motion(int joint_id);
void arm_monitor_cancel_all_joint_motion(void);
void arm_monitor_on_joint_motion(
    int joint_id,
    float target_angle,
    float primary_target_servo_angle,
    float secondary_target_servo_angle,
    enum ArmMotionMonitorPolicy monitor_policy
);
void arm_monitor(void);
uint32_t arm_monitor_get_next_delay_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_MONITOR_H__ */
