#ifndef __ARM_REMOTE_H
#define __ARM_REMOTE_H

#include "stdbool.h"
#include "arm_robot.h"

extern ArmPose g_arm_remote_init_pose;
extern float g_remote_angle_threshold;

int arm_remote_ik(float x, float y, float z, float rx, float ry, float rz, float claw_angle);
int arm_remote_reset(void);
void arm_remote_set_reset_flag(void);
bool arm_remote_is_reset(void);

#endif