#ifndef __ARM_ROBOT_KINEMATICS_H__
#define __ARM_ROBOT_KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_robot.h"
#include "inverse_kinematics.h"

extern int g_joint_weights[ARM_JOINTS_NO_CLAW_NUM];

/**
     * 机械臂逆运动学计算并执行
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param z 目标位置z坐标
     * @param rx 目标旋转rx角度
     * @param ry 目标旋转ry角度
     * @param rz 目标旋转rz角度
     * @return 0表示成功，非0表示失败
     */
int arm_robot_ik(float x, float y, float z, float rx, float ry, float rz);
float cal_joint_angle_distance(float joints_arr1[6], float joints_arr2[6]);
int arm_cal_interval_with_angle_diff(float target_joints[6], float *interval_ms);
int arm_update_cal_interval_with_angle_diff(float target_joints[6], float *interval_ms);
int arm_robot_ik_inner(ArmPose *pose, float solution[6]);
int arm_robot_fk(float joint_angles[6], ArmPose *pose);
int arm_get_optimal_solution(ik_solution_t solutions[IK_MAX_SOLUTIONS], int solution_count);

#ifdef __cplusplus
}
#endif
#endif /* __ARM_ROBOT_IK_H__ */
