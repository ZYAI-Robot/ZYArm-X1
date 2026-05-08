#include "inverse_kinematics.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float ik_compute_theta2(float beta, float psi, float theta3);
static void solve_euler_angles(const transform_matrix_t* T, 
                       float* theta4, float* theta5, float* theta6);
static void ik_radians_to_degrees(ik_solution_t solutions[IK_MAX_SOLUTIONS]);

/**
 * 正向运动学求解函数
 * 
 * 该函数使用DH参数法计算给定关节角度的机械臂末端位姿
 * 通过连乘各个关节的变换矩阵得到末端执行器的齐次变换矩阵
 * 
 * @param joint_angles 包含6个关节角度值的数组(单位:度)
 * @param T_ee 输出的末端执行器齐次变换矩阵
 * 
 * @note 需要预先正确设置全局DH参数g_robot_dh_params
 */
void forward_kinematics(const float joint_angles[6], transform_matrix_t* T_claw)
{
    // 初始化为单位矩阵
    transform_matrix_t T_temp;
    transform_matrix_t T_temp2;
    transform_matrix_t *T_ee = &T_temp2;
    memset(T_ee, 0, sizeof(transform_matrix_t));
    T_ee->data[0][0] = 1.0f;
    T_ee->data[1][1] = 1.0f;
    T_ee->data[2][2] = 1.0f;
    T_ee->data[3][3] = 1.0f;
    
    // 依次计算每个关节的变换矩阵并累乘
    for (int i = 0; i < 6; i++) {
        transform_matrix_t T_i;
        // 将角度从度转换为弧度
        float theta_rad = joint_angles[i] * M_PI / 180.0f;
        
        // 根据DH参数计算变换矩阵
        dh_transform(
            g_robot_dh_params[i].a,
            g_robot_dh_params[i].alpha,
            g_robot_dh_params[i].d,
            theta_rad,
            &T_i
        );
        
        // 累乘到总的变换矩阵
        multiply_transform(T_ee, &T_i, &T_temp);
        memcpy(T_ee, &T_temp, sizeof(transform_matrix_t));
    }

    static transform_matrix_t T_ee_to_claw = {.data = {{1, 0, 0, 0},
                                                 {0, 1, 0, 0}, 
                                                 {0, 0, 1, -159.4},
                                                 {0, 0, 0, 1}}};
    multiply_transform(T_ee, &T_ee_to_claw, T_claw);
}

/**
 * 机械臂逆运动学求解函数
 * 
 * 该函数使用几何法和Pieper解法相结合的方法计算6自由度机械臂的逆运动学解。
 * 通过给定的目标位姿矩阵，计算出最多4组可能的关节角度解。
 * 
 * 解算过程分为两步：
 * 1. 前三个关节(theta1, theta2, theta3)使用几何法求解
 * 2. 后三个关节(theta4, theta5, theta6)使用Pieper方法求解
 * 
 * @param T_clamp 夹爪位姿齐次变换矩阵
 * @param solutions 存储逆解的数组，最多可存储IK_MAX_SOLUTIONS(4)组解
 *                  每组解包含6个关节角度值
 * @return 成功返回0，失败返回-1（如目标位置不可达）
 * 
 * @note 需要预先正确设置全局DH参数g_robot_dh_params
 * @note 机械臂需满足Pieper准则（相邻三个关节轴线相交于一点）
 */
int inverse_kinematics_solve(const transform_matrix_t* T_clamp,
                            ik_solution_t solutions[IK_MAX_SOLUTIONS], bool just_geometry)
{
    const transform_matrix_t *T_target = T_clamp;
      
    // 几何法求解前三关节角度theta1、theta2、theta3
    // 提取DH参数
    float a1 = g_robot_dh_params[1].a;
    float a2 = g_robot_dh_params[2].a;
    float a3 = g_robot_dh_params[3].a;
    float d4 = g_robot_dh_params[3].d;
    
    // 腕点位置
    float px = T_target->data[0][3];
    float py = T_target->data[1][3];
    float pz = T_target->data[2][3];

    float l1 = a2; // 大臂长度
    float l2 = sqrtf(a3*a3 + d4*d4); // 小臂长度

    float theta3_init = atan2f(a3, fabsf(d4)) + M_PI;   // 由于结构关系，theta3的初始偏移量
    
    // 将目标位置投影到平面，利用三角函数求解theta1。由于结构关系，theata1只有一个解。
    float theta1 = 0;
    float r_sq = px*px + py*py;
    if (r_sq > IK_SOLVER_TOLERANCE) {
        theta1 = atan2f(py, px);
    } else {
        theta1 = 0; // 奇异位置
    }

    int i = 0;
    // 为方便计算，将坐标系切换到frame1中 (关节为1-6, 对应坐标系frame0-frame5)
    float x1 = px * cosf(theta1) + py * sinf(theta1) - a1; // 等同于 sqrt(r_sq) - a1
    float z1 = pz;
        
    // 检查是否可达
    float dist_sq = x1*x1 + z1*z1;
    float dist = sqrtf(dist_sq);
    if (dist > (l1 + l2)) {
        return -1; // 无解
    }

    // 计算theta3
    float theta3 = acosf((dist_sq - l1*l1 - l2*l2) / (2*l1*l2));
    float theta3_result[2] = {theta3, -theta3}; // theta3有两种可能的解

    float beta = atan2f(z1, x1);
    float psi = acosf((dist_sq + l1*l1 - l2*l2) / (2*l1*dist));
    
    for (int theta3_index = 0; theta3_index < 2; theta3_index++) { 
        theta3 = theta3_result[theta3_index];
        float theta2 = ik_compute_theta2(beta, psi, theta3);

        // 将几何法求得的theta2、theta3角度转为关节坐标系角度, theta1一致
        theta3 = theta3_init - theta3 + M_PI/2;
        theta2 = -theta2;

        solutions[i].joint_angles[0] = theta1;
        solutions[i].joint_angles[1] = theta2;
        solutions[i].joint_angles[2] = theta3;            
        i++;
    }

    if (just_geometry) {
        ik_radians_to_degrees(solutions);
        return 0;
    }

    // pieper解法计算theta4, theta5, theta6
    for (i = 0; i < 2; i++) {
        // 计算前三关节齐次变换 T03
        transform_matrix_t T01;
        dh_transform(g_robot_dh_params[0].a, g_robot_dh_params[0].alpha, g_robot_dh_params[0].d, solutions[i].joint_angles[0], &T01);
        transform_matrix_t T12;
        dh_transform(g_robot_dh_params[1].a, g_robot_dh_params[1].alpha, g_robot_dh_params[1].d, solutions[i].joint_angles[1], &T12);
        transform_matrix_t T23;
        dh_transform(g_robot_dh_params[2].a, g_robot_dh_params[2].alpha, g_robot_dh_params[2].d, solutions[i].joint_angles[2], &T23);
        transform_matrix_t T34_0;
        dh_transform(g_robot_dh_params[3].a, g_robot_dh_params[3].alpha, g_robot_dh_params[3].d, 0, &T34_0); // T34_0的theta4为0

        // T04_0 = T01 * T12 * T23 * T34_0;
        transform_matrix_t T02;
        multiply_transform(&T01, &T12, &T02);
        transform_matrix_t T03;
        multiply_transform(&T02, &T23, &T03);
        transform_matrix_t T04_0;
        multiply_transform(&T03, &T34_0, &T04_0);

        // T46_0 = inv(T04_0) * T_target
        transform_matrix_t T04_0_inv;
        inverse_transform(&T04_0, &T04_0_inv);
        transform_matrix_t T46_0;
        multiply_transform(&T04_0_inv, T_target, &T46_0);

        // 从R46提取欧拉角zyz, 计算theta4, theta5, theta6 (当前坐标设置，欧拉角与DH坐标系一致，无需转化)
        float theta4 = 0;
        float theta5 = 0;
        float theta6 = 0;
        
        // 调用函数逆解欧拉角
        solve_euler_angles(&T46_0, &theta4, &theta5, &theta6);
        solutions[i].joint_angles[3] = theta4;
        solutions[i].joint_angles[4] = theta5;
        solutions[i].joint_angles[5] = theta6;
    }

    // 翻转腕部，扩展到4组解
    memcpy(&solutions[2], &solutions[0], sizeof(ik_solution_t) * 2);
    for (i = 2; i < 4; i++) {
        solutions[i].joint_angles[3] += M_PI;
        solutions[i].joint_angles[4] = -solutions[i].joint_angles[4];
        solutions[i].joint_angles[5] += M_PI;
    }

    ik_radians_to_degrees(solutions);

    return 0;
}
static void ik_radians_to_degrees(ik_solution_t solutions[IK_MAX_SOLUTIONS])
{
    for (int i = 0; i < IK_MAX_SOLUTIONS; i++) {
        for (int j = 0; j < 6; j++) {
            solutions[i].joint_angles[j] = solutions[i].joint_angles[j] * 180 / M_PI;
        }
    }
}

/**
 * 计算第二个关节角度theta2
 * 
 * 根据几何关系计算第二个关节的角度值，考虑theta3的符号影响
 * 
 * @param beta 从基座到目标点的角度
 * @param psi  在三角形中由余弦定理计算的角度
 * @param theta3 第三个关节的角度
 * @return 计算得到的theta2角度值
 */
static float ik_compute_theta2(float beta, float psi, float theta3)
{
    float theta2 = 0;
    if (theta3 < 0) {
        theta2 = beta + psi;
    } else {
        theta2 = beta - psi;
    }
    return theta2;
}

/**
 * 构造DH参数对应的齐次变换矩阵
 * 
 * 根据标准DH参数法构造从关节i-1到关节i的齐次变换矩阵
 * 变换顺序为：绕Z轴旋转theta -> 沿Z轴平移d -> 沿X轴平移a -> 绕X轴旋转alpha
 * 
 * @param a      连杆长度 (link length)
 * @param alpha  连杆扭转角 (link twist)
 * @param d      关节偏距 (link offset)  
 * @param theta  关节角 (joint angle)
 * @param result 输出的齐次变换矩阵
 */
void dh_transform(float a, float alpha, float d, float theta, transform_matrix_t* result)
{
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(alpha);
    float sa = sinf(alpha);
    
    result->data[0][0] = ct;      result->data[0][1] = -st;     result->data[0][2] = 0;    result->data[0][3] = a;
    result->data[1][0] = st*ca;   result->data[1][1] = ct*ca;   result->data[1][2] = -sa;  result->data[1][3] = -d*sa;
    result->data[2][0] = st*sa;   result->data[2][1] = ct*sa;   result->data[2][2] = ca;   result->data[2][3] = d*ca;
    result->data[3][0] = 0;       result->data[3][1] = 0;       result->data[3][2] = 0;    result->data[3][3] = 1;
}

/**
 * 计算齐次变换矩阵的逆矩阵
 * 
 * 利用齐次变换矩阵的特殊结构（旋转部分为正交矩阵），
 * 通过转置旋转部分并重新计算平移部分来高效计算逆矩阵
 * 
 * @param T      输入的齐次变换矩阵
 * @param result 输出的逆变换矩阵
 */
void inverse_transform(const transform_matrix_t* T, transform_matrix_t* result)
{
    // 转置旋转部分（假设是正交矩阵）
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result->data[i][j] = T->data[j][i];
        }
    }
    
    // 计算平移部分
    for (int i = 0; i < 3; i++) {
        result->data[i][3] = 0;
        for (int j = 0; j < 3; j++) {
            result->data[i][3] -= result->data[i][j] * T->data[j][3];
        }
    }
    
    // 最后一行保持不变
    result->data[3][0] = 0;
    result->data[3][1] = 0;
    result->data[3][2] = 0;
    result->data[3][3] = 1;
}

/**
 * 矩阵相乘函数，计算结果存储在result中
 * @param A 第一个变换矩阵
 * @param B 第二个变换矩阵
 * @param result 存储结果的变换矩阵
 */
void multiply_transform(const transform_matrix_t* A, const transform_matrix_t* B, transform_matrix_t* result)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result->data[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result->data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }
}

/**
 * 从齐次变换矩阵中逆解欧拉角(ZYZ顺序)
 * @param T 输入的齐次变换矩阵
 * @param theta4 输出的第4关节角度
 * @param theta5 输出的第5关节角度
 * @param theta6 输出的第6关节角度
 */
static void solve_euler_angles(const transform_matrix_t* T, 
                       float* theta4, float* theta5, float* theta6)
{
    float val = sqrtf(T->data[2][0]*T->data[2][0] + T->data[2][1]*T->data[2][1]);
    *theta5 = atan2f(val, T->data[2][2]);
    
    if (fabsf(*theta5) > IK_SOLVER_TOLERANCE) { 
        *theta4 = atan2f(T->data[1][2], T->data[0][2]);
        *theta6 = atan2f(T->data[2][1], -T->data[2][0]);
    } else { // 万向锁（theta5=0）
        *theta4 = 0; // 任意值，取0简化
        *theta6 = atan2f(-T->data[0][1], T->data[0][0]);
    }
}
