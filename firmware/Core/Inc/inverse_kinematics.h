#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义常量
#define IK_MAX_SOLUTIONS    4
#define IK_SOLVER_TOLERANCE 1e-6f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

// 数据结构定义
typedef struct {
    float a;
    float alpha;
    float d;
    float theta;
} dh_param_t;

typedef struct {
    float data[4][4];
} transform_matrix_t;

typedef struct {
    float joint_angles[6];
} ik_solution_t;

// 全局DH参数变量声明，需要根据机械臂自行定义
// 例如：const dh_param_t g_robot_dh_params[6] = { {a0, alpha0, d0, theta0}, ... };
extern const dh_param_t g_robot_dh_params[6];
extern const float g_robot_claw_length; // 夹爪长度

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
void forward_kinematics(const float joint_angles[6], transform_matrix_t* T_ee);

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
 * @param T_clamp   夹爪位姿齐次变换矩阵
 * @param solutions 存储逆解的数组，最多可存储IK_MAX_SOLUTIONS(4)组解
 *                  每组解包含6个关节角度值
 * @return 成功返回0，失败返回-1（如目标位置不可达）
 * 
 * @note 需要预先正确设置全局DH参数g_robot_dh_params
 * @note 机械臂需满足Pieper准则（相邻三个关节轴线相交于一点）
 */
int inverse_kinematics_solve(const transform_matrix_t* T_clamp,
                            ik_solution_t solutions[IK_MAX_SOLUTIONS], bool just_geometry);

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
void dh_transform(float a, float alpha, float d, float theta, transform_matrix_t* result);

/**
 * 计算两个齐次变换矩阵的乘积
 * 
 * 执行标准的4x4矩阵乘法，计算A×B的结果
 * 
 * @param A 第一个变换矩阵
 * @param B 第二个变换矩阵
 * @param result 存储结果的变换矩阵
 */
void multiply_transform(const transform_matrix_t* A, const transform_matrix_t* B, transform_matrix_t* result);

/**
 * 计算齐次变换矩阵的逆矩阵
 * 
 * 利用齐次变换矩阵的特殊结构（旋转部分为正交矩阵），
 * 通过转置旋转部分并重新计算平移部分来高效计算逆矩阵
 * 
 * @param T      输入的齐次变换矩阵
 * @param result 输出的逆变换矩阵
 */
void inverse_transform(const transform_matrix_t* T, transform_matrix_t* result);

#ifdef __cplusplus
}
#endif

#endif // INVERSE_KINEMATICS_H
