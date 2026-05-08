#include "arm_robot_kinematics.h"
#include "inverse_kinematics.h"
#include "arm_shell.h"
#include "string.h"

#define ARM_KINEMATICS_LOG_TAG "KINEMATICS"

// IK 仅计算 6 个非夹爪关节(0~5)，避免按 ARM_JOINTS_NUM(含夹爪)遍历导致越界
int g_joint_weights[ARM_JOINTS_NO_CLAW_NUM] = {5, 5, 5, 5, 1, 1};

void print_transform(const transform_matrix_t *T)
{
    safe_printf("Transform Matrix:\n");
    safe_printf("  [%.2f %.2f %.2f %.2f]\n", T->data[0][0], T->data[0][1], T->data[0][2], T->data[0][3]);
    safe_printf("  [%.2f %.2f %.2f %.2f]\n", T->data[1][0], T->data[1][1], T->data[1][2], T->data[1][3]);
    safe_printf("  [%.2f %.2f %.2f %.2f]\n", T->data[2][0], T->data[2][1], T->data[2][2], T->data[2][3]);
    safe_printf("  [0.0f 0.0f 0.0f 1.0f]\n");
}
float cal_joint_angle_distance(float joints_arr1[6], float joints_arr2[6])
{
    float sum = 0;
    for (int i = 0; i < 6; i++) {
        float d = joints_arr1[i] - joints_arr2[i];
        sum += d * d;
    }
    return sqrtf(sum);
}

int arm_cal_interval_with_angle_diff(float target_joints[6], float *interval_ms)
{
    float current_joints[6];
    for (int i = 0; i < 6; i++) {
        current_joints[i] = g_arm_robot.joint[i].angle;
    }

    float distance = cal_joint_angle_distance(target_joints, current_joints);
    *interval_ms = fabsf(distance * 1000 / g_arm_robot.cfg.speed);
    return 0;
}

int arm_update_cal_interval_with_angle_diff(float target_joints[6], float *interval_ms)
{
    int ret = arm_joint_angle_update(false);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Update joint angles failed\n");
        return -1;
    }

    ret = arm_cal_interval_with_angle_diff(target_joints, interval_ms);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Calculate interval failed\n");
        return -1;
    }

    return 0;
}

static bool arm_joint_angle_map(int joint_id, float angle, float *angle_map)
{
    // 检查参数有效性
    if ((joint_id < 0) || (joint_id >= ARM_JOINTS_NUM)) {
        return false;
    }
    
    if (angle_map == NULL) {
        return false;
    }
    
    // 定义浮点数比较的精度
    const float epsilon = ARM_FLOAT_TOLERANCE;
    
    // 获取关节限制
    float min_angle = g_arm_robot.joint[joint_id].min_angle;
    float max_angle = g_arm_robot.joint[joint_id].max_angle;
    
    // 如果角度已经在有效范围内，直接返回
    if ((angle > (min_angle - epsilon)) && (angle < (max_angle + epsilon))) {
        *angle_map = angle;
        return true;
    }
    
    // 尝试通过加减360度来转换角度到有效范围内
    float mapped_angle = angle;
    
    // 如果角度小于最小值，尝试加上360度
    while (mapped_angle < (min_angle - epsilon)) {
        mapped_angle += 360.0f;
    }
    
    // 如果角度大于最大值，尝试减去360度
    while (mapped_angle > (max_angle + epsilon)) {
        mapped_angle -= 360.0f;
    }
    
    // 检查转换后的角度是否在有效范围内
    if ((mapped_angle > (min_angle - epsilon)) && (mapped_angle < (max_angle + epsilon))) {
        *angle_map = mapped_angle;
        return true;
    }
    
    // 如果仍然无效，则返回false
    return false;
}

int arm_get_optimal_solution(ik_solution_t solutions[IK_MAX_SOLUTIONS], int solution_count)
{
    float min_diff = 1e9f;
    int optimal_index = -1;
    float diff = 0;
    bool invail = true;
    for (int i = 0; i < solution_count; i++) {
        diff = 0;
        invail = true;
        // solutions[i].joint_angles 只有 6 个元素(不含夹爪)
        for (int j = 0; j < 6; j++) {
            float angle_map = 0;
            if (arm_joint_angle_map(j, solutions[i].joint_angles[j], &angle_map) == false) {
                invail = false;
                break;
            }
            solutions[i].joint_angles[j] = angle_map;
            // 与上一个角度的差值
            float d = solutions[i].joint_angles[j] - g_arm_robot.joint[j].angle;

            // 若关节0、3、4、5，需要加上关节角度与0度的差值，以倾向往0度方向
            if ((j == 0) || (j == 3) || (j == 4) || (j == 5)) {
                d += fabsf(solutions[i].joint_angles[j]);
            }

            diff += g_joint_weights[j] * d * d;
        }

        if (invail == false) {
            continue;
        }

        if (diff < min_diff) {
            min_diff = diff;
            optimal_index = i;
        }
    }
    return optimal_index;
}

static void arm_pose_to_transform(ArmPose *pose, transform_matrix_t *T)
{
    float x = pose->x;
    float y = pose->y;
    float z = pose->z;
    float rx = pose->rx;
    float ry = pose->ry;
    float rz = pose->rz;
    float rx_radians = rx * (M_PI / 180.0f);
    float ry_radians = ry * (M_PI / 180.0f);
    float rz_radians = rz * (M_PI / 180.0f);

    transform_matrix_t T_pos = { .data = {{1, 0, 0, x}, 
                                 {0, 1, 0, y},
                                 {0, 0, 1, z},
                                 {0, 0, 0, 1}} };

    transform_matrix_t T_rx = { .data = {{1, 0, 0, 0}, 
                                {0, cosf(rx_radians), -sinf(rx_radians), 0},
                                {0, sinf(rx_radians), cosf(rx_radians), 0},
                                {0, 0, 0, 1}} };

    transform_matrix_t T_ry = { .data = {{cosf(ry_radians), 0, sinf(ry_radians), 0},
                                {0, 1, 0, 0},
                                {-sinf(ry_radians), 0, cosf(ry_radians), 0},
                                {0, 0, 0, 1}} };
    
    transform_matrix_t T_rz = { .data = {{cosf(rz_radians), -sinf(rz_radians), 0, 0},
                                {sinf(rz_radians), cosf(rz_radians), 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}} };

    // 夹爪坐标系到末端执行器坐标系的变换
    transform_matrix_t T_clamp_to_ee = {.data = {{1, 0, 0, 0},
                                                 {0, 1, 0, 0}, 
                                                 {0, 0, 1, g_robot_claw_length},
                                                 {0, 0, 0, 1}}};

    transform_matrix_t T_tmp_1 = {0};
    transform_matrix_t T_tmp_2 = {0};
    multiply_transform(&T_pos, &T_rx, &T_tmp_1);
    multiply_transform(&T_tmp_1, &T_ry, &T_tmp_2);
    multiply_transform(&T_tmp_2, &T_rz, &T_tmp_1);
    multiply_transform(&T_tmp_1, &T_clamp_to_ee, T);
}

/**
 * @brief 从齐次变换矩阵提取欧拉角
 * @param T 齐次变换矩阵
 * @param rx 输出X轴旋转角（度）
 * @param ry 输出Y轴旋转角（度）
 * @param rz 输出Z轴旋转角（度）
 */
static void transform_to_euler(const transform_matrix_t *T, float *rx, float *ry, float *rz)
{
    float r11 = T->data[0][0], r12 = T->data[0][1], r13 = T->data[0][2];
    float r21 = T->data[1][0], r22 = T->data[1][1], r23 = T->data[1][2];
    float r31 = T->data[2][0], r32 = T->data[2][1], r33 = T->data[2][2];
    
    // 计算欧拉角（X-Y-Z顺序，与逆运动学一致）
    float cos_ry = sqrtf(r11 * r11 + r12 * r12);
    
    if (cos_ry > 0.0001f) {
        // 非奇异情况
        *ry = atan2f(r13, cos_ry);
        *rx = atan2f(-r23 / cos_ry, r33 / cos_ry);
        *rz = atan2f(-r12 / cos_ry, r11 / cos_ry);
    } else {
        // 奇异情况（ry = ±90度）
        *ry = r13 > 0.0f ? M_PI / 2.0f : -M_PI / 2.0f;
        *rx = atan2f(r32, r22);
        *rz = 0.0f;
    }
    
    // 转换为度数
    *rx = *rx * 180.0f / M_PI;
    *ry = *ry * 180.0f / M_PI;
    *rz = *rz * 180.0f / M_PI;
}

/**
 * @brief 机械臂正运动学计算
 * @param joint_angles 6个关节的角度（度）
 * @param pose 输出的末端执行器位姿
 * @return 成功返回0
 */
int arm_robot_fk(float joint_angles[6], ArmPose *pose)
{
    transform_matrix_t T_ee = {0};
    transform_matrix_t T_temp = {0};
    
    // 初始化单位矩阵
    for (int i = 0; i < 4; i++) {
        T_ee.data[i][i] = 1.0f;
    }
    
    // 计算6个关节的变换矩阵并累乘
    for (int i = 0; i < 6; i++) {
        transform_matrix_t T_i;
        float theta_rad = joint_angles[i] * M_PI / 180.0f;
        
        // 使用DH参数计算关节变换矩阵
        dh_transform(
            g_robot_dh_params[i].a,
            g_robot_dh_params[i].alpha,
            g_robot_dh_params[i].d,
            theta_rad,
            &T_i
        );
        
        // 累乘变换矩阵
        multiply_transform(&T_ee, &T_i, &T_temp);
        memcpy(&T_ee, &T_temp, sizeof(transform_matrix_t));
    }
    
    // 处理夹爪长度
    transform_matrix_t T_ee_to_claw = {.data = {{1, 0, 0, 0},
                                             {0, 1, 0, 0}, 
                                             {0, 0, 1, -g_robot_claw_length},
                                             {0, 0, 0, 1}}};
    multiply_transform(&T_ee, &T_ee_to_claw, &T_temp);
    
    // 提取位置信息
    pose->x = T_temp.data[0][3];
    pose->y = T_temp.data[1][3];
    pose->z = T_temp.data[2][3];
    
    // 提取姿态信息（欧拉角）
    transform_to_euler(&T_temp, &pose->rx, &pose->ry, &pose->rz);
    
    return 0;
}

int arm_robot_ik_inner(ArmPose *pose, float solution[6])
{
    transform_matrix_t T = {0};
    arm_pose_to_transform(pose, &T);
    ik_solution_t solutions[IK_MAX_SOLUTIONS] = {0};

    int ret = inverse_kinematics_solve(&T, solutions, false);
    if (ret < 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Inverse kinematics failed, location unreachable\n");
        return -1;
    }

    int optimal_index = arm_get_optimal_solution(solutions, IK_MAX_SOLUTIONS);
    if (optimal_index < 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Inverse kinematics failed, no optimal solution found\n");
        return -1;
    }

    for (int i = 0; i < 6; i++) {
        solution[i] = solutions[optimal_index].joint_angles[i];
    }
    return 0;
}

int arm_robot_ik(float x, float y, float z, float rx, float ry, float rz)
{
    ArmPose target_pose = {0};
    target_pose.x = x;
    target_pose.y = y;
    target_pose.z = z;
    target_pose.rx = rx;
    target_pose.ry = ry;
    target_pose.rz = rz;

    int ret = arm_joint_angle_update(false);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Update joint angles failed\n");
        return -1;
    }

    float solution[6];
    ret = arm_robot_ik_inner(&target_pose, solution);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "IK calculation failed\n");
        return -1;
    }

    float interval_ms;
    ret = arm_cal_interval_with_angle_diff(solution, &interval_ms);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_KINEMATICS_LOG_TAG, "Calculate interval failed\n");
        return -1;
    }

    for (int i = 0; i < 6; i++) {
        arm_set_joint_angle_interval_acc(i, solution[i], (int)roundf(interval_ms), ARM_DEFAULT_ACCEL_TIME, ARM_DEFAULT_ACCEL_TIME);
    }
    return 0;
}
