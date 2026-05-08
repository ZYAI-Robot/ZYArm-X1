#include "arm_robot.h"
#include "arm_robot_kinematics.h"
#include "arm_shell.h"

#define ARM_REMOTE_LOG_TAG "REMOTE"

#define IK_SINGULARITY_THRESHOLD 10.0f

static int g_remote_reset_flag = 0;
static float g_rmote_joint4_init_angle = 0.0f;

ArmPose g_arm_remote_init_pose = {
    // 起始位置可配
    .x = 300.0f,
    .y = 0.0f,
    .z = 150.0f,

    // 起始角度不可配
    .rx = 0.0f,
    .ry = -90.0f,
    .rz = 0.0f,
};
int arm_remote_reset(void)
{
    int ret = arm_robot_ik(g_arm_remote_init_pose.x, g_arm_remote_init_pose.y, g_arm_remote_init_pose.z, 
                            g_arm_remote_init_pose.rx, g_arm_remote_init_pose.ry, g_arm_remote_init_pose.rz);
    if (ret != 0) {
        return -1;
    }

    g_rmote_joint4_init_angle = g_arm_robot.joint[4].angle;

    float interval_ms = 2000;
    ret = arm_set_joint_angle_interval_acc(ARM_CLAW_JOINT_ID, 0, (int)roundf(interval_ms), 
                        ARM_DEFAULT_ACCEL_TIME, ARM_DEFAULT_ACCEL_TIME);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

void arm_remote_set_reset_flag(void)
{
    g_remote_reset_flag = 1;
}

bool arm_remote_is_reset(void)
{
    return g_remote_reset_flag;
}

static void arm_remote_fk(ArmPose *pose, transform_matrix_t *T)
{
    float x = pose->x + g_arm_remote_init_pose.x - g_robot_claw_length;
    float y = pose->y + g_arm_remote_init_pose.y;
    float z = pose->z + g_arm_remote_init_pose.z;
    
    // 为将末端坐标系与手柄坐标系对齐，需先旋转y轴-90度
    float rx = pose->rz;
    float ry = pose->ry;
    float rz = -pose->rx;
    float rx_radians = rx * (M_PI / 180.0f);
    float ry_radians = ry * (M_PI / 180.0f);
    float rz_radians = rz * (M_PI / 180.0f);

    transform_matrix_t T_ry_prv = { .data = {{0, 0, -1, 0},
                                {0, 1, 0, 0},
                                {1, 0, 0, 0},
                                {0, 0, 0, 1}} };

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
    
    transform_matrix_t T_tmp_1 = {0};
    transform_matrix_t T_tmp_2 = {0};
    multiply_transform(&T_pos, &T_ry_prv, &T_tmp_1);
    multiply_transform(&T_tmp_1, &T_rx, &T_tmp_2);
    multiply_transform(&T_tmp_2, &T_ry, &T_tmp_1);
    multiply_transform(&T_tmp_1, &T_rz, T);
}
float g_remote_angle_threshold = 2.0f;
static int arm_remote_ik_inner(ArmPose *pose, float solution[6])
{
    transform_matrix_t T = {0};
    arm_remote_fk(pose, &T);
    ik_solution_t solutions[IK_MAX_SOLUTIONS] = {0};

    int ret = inverse_kinematics_solve(&T, solutions, true);
    if (ret < 0) {
        ARM_LOGE_TAG(ARM_REMOTE_LOG_TAG, "Inverse kinematics failed, location unreachable\n");
        return -1;
    }

    int optimal_index = arm_get_optimal_solution(solutions, 2);
    if (optimal_index < 0) {
        ARM_LOGE_TAG(ARM_REMOTE_LOG_TAG, "Inverse kinematics failed, no optimal solution found\n");
        return -1;
    }

    for (int i = 0; i < 6; i++) {
        solution[i] = solutions[optimal_index].joint_angles[i];
    }

    solution[3] = 0.0f;
    solution[4] = pose->ry + g_rmote_joint4_init_angle;
    solution[5] = -pose->rx;
    
    return 0;
}

int arm_remote_ik(float x, float y, float z, float rx, float ry, float rz, float claw_angle)
{
    static int print_filter = 0;
    static float real_claw_angle = 0;
    ArmPose target_pose = {0};
    target_pose.x = x;
    target_pose.y = y;
    target_pose.z = z;
    target_pose.rx = rx;
    target_pose.ry = ry;
    target_pose.rz = 0;
    
    float joint_angles[7] = {0};
    int ret = arm_remote_ik_inner(&target_pose, joint_angles);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_REMOTE_LOG_TAG, "IK calculation failed\n");
        return -1;
    }

    joint_angles[6] = claw_angle;
    print_filter++;
    if (print_filter == 2) {
        print_filter = 0;
        // 获取当前关节角度
        float current_joint_angles[ARM_JOINTS_NUM] = {0};
        ret = arm_joint_snapshot_read(true, current_joint_angles);
        if (ret != 0) {
            return -1;
        }
        safe_printf("[current](%.2f %.2f %.2f %.2f %.2f %.2f %.2f) [target](%.2f %.2f %.2f %.2f %.2f %.2f %.2f)\n", current_joint_angles[0], current_joint_angles[1], 
            current_joint_angles[2], current_joint_angles[3], current_joint_angles[4], current_joint_angles[5], current_joint_angles[6],
            joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6]);
    }

    ret = arm_joint_sync_move(joint_angles);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_REMOTE_LOG_TAG, "Joint sync move failed\n");
        return -1;
    }
    
    return 0;
}
