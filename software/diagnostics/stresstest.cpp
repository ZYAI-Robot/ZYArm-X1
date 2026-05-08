#include "zyarm_sdk/arm.hpp"
#include <iostream>
#include <unistd.h>
#include <chrono>

int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 2) {
        std::cerr << "用法: " << argv[0] << " <串口设备>" << std::endl;
        std::cerr << "示例: " << argv[0] << " /dev/ttyUSB0" << std::endl;
        return 1;
    }
    
    const char* serial_port = argv[1];
    std::cout << "使用串口设备: " << serial_port << std::endl;
    
    // 初始化机器人执行器
    zyarm_sdk::ZyArmConfig config;
    config.port = serial_port;
    zyarm_sdk::ZyArm robot(config);
    robot.connect();
    
    // 检查连接状态
    if (!robot.is_connected()) {
        std::cerr << "错误: 无法连接到机器人!" << std::endl;
        return 1;
    }
    
    std::cout << "机器人连接成功，开始压力测试..." << std::endl;
    sleep(1);
    
    int iteration = 0;
    
    if (!robot.reset().accepted) {
        std::cerr << "复位失败!" << std::endl;
    }

    if (!robot.set_gripper(1.0, true).accepted) {
        std::cerr << "夹爪打开失败" << std::endl;
    }

    std::cerr << "夹爪打开" << std::endl;

    if (!robot.set_gripper(0.1, false).accepted) {
        std::cerr << "夹爪关闭失败" << std::endl;
    }

    sleep(5);//保证收到下位机的 success
    std::cerr << "夹爪关闭" << std::endl;



    while (true) {
        iteration++;
        std::cout << "\n=== 迭代次数: " << iteration << " ===" << std::endl;
        
        // 位姿1: [CMD][0][200 0 100 0 0 0]
        std::cout << "\n测试位姿1: [200, 0, 100, 0, 0, 0]\n" << std::endl;

        
        if (!robot.move_ik(200, 0, 100, 0, 0, 0).accepted) {
            std::cerr << "移动到位姿1失败!" << std::endl;
            continue;
        }
        
        // 位姿2: [CMD][0][200 0 100 0 -90 0]
        std::cout << "\n测试位姿2: [200, 0, 100, 0, -90, 0]\n" << std::endl;
        // if (robot.reset() != 0) {
        //     std::cerr << "复位失败!" << std::endl;
        //     continue;
        // }
        
        if (!robot.move_ik(200, 0, 100, 0, -90, 0).accepted) {
            std::cerr << "移动到位姿2失败!" << std::endl;
            continue;
        }
        
        // 位姿3: [CMD][0][200 -100 100 0 -90 0]
        std::cout << "\n测试位姿3: [200, -100, 100]\n" << std::endl;
        // if (robot.reset() != 0) {
        //     std::cerr << "复位失败!" << std::endl;
        //     continue;
        // }
        
        if (!robot.move_ik(200, -100, 100, 0, -90, 0).accepted) {
            std::cerr << "移动到位姿3失败!" << std::endl;
            continue;
        }
        
        // 位姿4: [CMD][0][200 100 200 0 -90 0]
        std::cout << "\n测试位姿4: [200, 100, 200, 0, -90, 0]\n" << std::endl;
        // if (robot.reset() != 0) {
        //     std::cerr << "复位失败!" << std::endl;
        //     continue;
        // }
        
        if (!robot.move_ik(200, 100, 200, 0, -90, 0).accepted) {
            std::cerr << "移动到位姿4失败!" << std::endl;
            continue;
        }
        
        // 短暂延迟后继续下一轮
        sleep(1);
    }
    
    return 0;
}
