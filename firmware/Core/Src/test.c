#include "arm_controller.h"
#include <stdio.h>

extern ArmController arm_controller;
// 测试机械臂基本功能
void test_arm_basic_functions(void)
{
    printf("=== 机械臂基本功能测试 ===\n");
    
    // 1. 显示当前状态
    arm_display_status();
    
    // 2. 检查肩部同步
    printf("检查肩部同步状态...\n");
    arm_check_shoulder_sync();
    
    // 3. 测试单个关节运动
    printf("测试单个关节运动...\n");
    
    // 测试底座旋转 (ID1)
    printf("测试底座旋转(ID1)...\n");
    arm_move_joint(JOINT_BASE_ROTATION, 200, 1000);
    osDelay(1200);
    arm_move_joint(JOINT_BASE_ROTATION, 800, 1000);
    osDelay(1200);
    arm_move_joint(JOINT_BASE_ROTATION, 500, 1000);
    osDelay(1200);
    
    // 测试肩部同步运动 (ID2和ID3)
    printf("测试肩部同步运动(ID2和ID3)...\n");
    arm_move_shoulder_sync(200, 1000);
    osDelay(1200);
    arm_move_shoulder_sync(600, 1000);
    osDelay(1200);
    
    // 测试肘部运动 (ID4, ID5)
    printf("测试肘部运动(ID4, ID5)...\n");
    arm_move_joint(JOINT_ELBOW_PITCH, 300, 1000);
    arm_move_joint(JOINT_ELBOW_ROLL, 300, 1000);
    osDelay(1200);
    arm_move_joint(JOINT_ELBOW_PITCH, 700, 1000);
    arm_move_joint(JOINT_ELBOW_ROLL, 700, 1000);
    osDelay(1200);
    
    // 4. 测试协同运动
    printf("测试协同运动...\n");
    ArmHomePosition test_pos = {
        .base_rotation = 300, .shoulder = 500, 
        .elbow_pitch = 700, .elbow_roll = 300,
        .wrist_pitch = 400, .wrist_roll = 600, 
        .gripper = 500
    };
    arm_move_smooth(&test_pos, 2000);
    
    // 5. 返回复位位置
    printf("返回直立状态...\n");
    arm_reset_to_home(&arm_controller);
    
    printf("基本功能测试完成!\n");
}

// 预设动作演示
void arm_preset_demo(void)
{
    printf("=== 预设动作演示 ===\n");
    
    // 姿势1: 伸展姿势
    printf("姿势1: 完全伸展\n");
    ArmHomePosition pose_stretch = {
        .base_rotation = 500, .shoulder = 800, 
        .elbow_pitch = 800, .elbow_roll = 500,
        .wrist_pitch = 500, .wrist_roll = 500, 
        .gripper = 0
    };
    arm_move_smooth(&pose_stretch, 2500);
    osDelay(3000);
    
    // 姿势2: 收缩姿势
    printf("姿势2: 完全收缩\n");
    ArmHomePosition pose_contract = {
        .base_rotation = 500, .shoulder = 200, 
        .elbow_pitch = 200, .elbow_roll = 500,
        .wrist_pitch = 500, .wrist_roll = 500, 
        .gripper = 0
    };
    arm_move_smooth(&pose_contract, 2500);
    osDelay(3000);
    
    // 返回直立状态
    printf("返回直立状态...\n");
    arm_reset_to_home(&arm_controller);
    
    printf("预设动作演示完成!\n");
}