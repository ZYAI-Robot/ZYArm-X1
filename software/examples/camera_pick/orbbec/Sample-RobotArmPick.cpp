#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/Utils.hpp"
#include "window.hpp"
#include "zyarm_sdk/arm.hpp"
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>

float ry_camera = 1.2042f;    // 相机旋转角度，单位弧度, 绕Y轴旋转
float h_camera_base = 330.0f;     // 相机到基座的垂直高度，单位mm, Z轴方向
float l_base_robot = -300.0f;      // 基座到机械臂底座的水平距离，单位mm, Y轴方向
float d_base_robot = 50.0f;
// 鼠标点击点结构
struct ClickPoint {
    int x;
    int y;
    bool valid;
    
    ClickPoint() : x(0), y(0), valid(false) {}
};

// 全局变量存储鼠标点击位置
static ClickPoint g_clickPoint;

// 机器人执行器实例
static zyarm_sdk::ZyArm* g_robotActor = nullptr;

// 鼠标回调函数
void onMouse(int event, int x, int y, int flags, void* param) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_clickPoint.x = x;
        g_clickPoint.y = y;
        g_clickPoint.valid = true;
        std::cout << "Mouse clicked at: (" << x << ", " << y << ")" << std::endl;
    }
}

void transformPointCloudToRobotBase(OBPoint3f *pointCloud, OBPoint3f *robot)
{
    // 点云坐标系转换到场合相机坐标系
    OBPoint3f camera;
    camera.x = pointCloud->z;
    camera.y = pointCloud->x;
    camera.z = pointCloud->y;
    
    // 计算相机坐标系到机械臂基座坐标系的变换
    float cos_ry = cosf(ry_camera);
    float sin_ry = sinf(ry_camera);
    
    // 旋转变换
    float x_rotated = cos_ry * camera.x + sin_ry * camera.z;
    float y_rotated = camera.y;
    float z_rotated = -sin_ry * camera.x + cos_ry * camera.z;
    
    // 平移变换
    robot->x = x_rotated - d_base_robot;
    robot->y = y_rotated + l_base_robot;
    robot->z = z_rotated + h_camera_base;
    robot->z -= 103.8f; // 机械臂基座到第一个关节的高度偏移
}

void robotArmPick(zyarm_sdk::ZyArm *robot, OBPoint3f *pickPoint)
{
    std::cout << "Starting robot arm pick sequence..." << std::endl;
    bool result = true;
    
    // Close claw to grab object
    std::cout << "Closing claw to grab object" << std::endl;
    result = robot->set_gripper(0.5, true).accepted;
    if (!result) {
        std::cerr << "Failed to close claw!" << std::endl;
        return;
    }
    
    // Move to pick position (hovering)
    std::cout << "Moving to pick position: (" << pickPoint->x << ", " << pickPoint->y << ", 0)" << std::endl;
    result = robot->move_ik(pickPoint->x, pickPoint->y, -50).accepted;
    if (!result) {
        std::cerr << "Failed to move to pick position!" << std::endl;
        return;
    }

  
    // Move down to pick up object
    std::cout << "Moving down to pick up object" << std::endl;
    // robot->set_speed(20);
    // sleep(1);
    result = robot->move_ik(pickPoint->x, pickPoint->y, -80).accepted;
    if (!result) {
        std::cerr << "Failed to move down!" << std::endl;
        return;
    }
    // robot->set_speed(30);
    // sleep(1);

    // close claw
    std::cout << "Fully closing claw" << std::endl;
    result = robot->set_gripper(0.2, true).accepted;
    if (!result) {
        std::cerr << "Failed to fully close claw!" << std::endl;
        return;
    }
    // sleep(1);

    // Move back up with object
    std::cout << "Moving back up with object" << std::endl;
    result = robot->move_ik(pickPoint->x, pickPoint->y, 0).accepted;
    if (!result) {
        std::cerr << "Failed to move back up!" << std::endl;
        return;
    }

    std::cout << "Playing action" << std::endl;
    result = robot->play_record(2222).accepted;
    if (!result) {
        std::cerr << "Failed to play action!" << std::endl;
        return;
    }

    // std::cout << "Moving (200 0 -80)" << std::endl;
    // result = robot->move_ik(200, 0, -80).accepted;
    // if (!result) {
    //     std::cerr << "Failed to move back up!" << std::endl;
    //     return;
    // }

    // result = robot->set_gripper(0.45, true).accepted;
    // if (!result) {
    //     std::cerr << "Failed to fully close claw!" << std::endl;
    //     return;
    // }

    // result = robot->move_ik(200, 0, 0).accepted;
    // if (!result) {
    //     std::cerr << "Failed to move back up!" << std::endl;
    //     return;
    // }
    robot->reset();
    
    // // Play action sequence
    // std::cout << "Playing action sequence" << std::endl;
    // result = robot->play_record(0).accepted;
    // if (!result) {
    //     std::cerr << "Failed to play action sequence!" << std::endl;
    //     return;
    // }
    // sleep(1);
    
    // // Reset robot
    // std::cout << "Resetting robot" << std::endl;
    // result = robot->reset().accepted;
    // if (!result) {
    //     std::cerr << "Failed to reset robot!" << std::endl;
    //     return;
    // }
    // sleep(1);
    
    // std::cout << "Robot arm pick sequence completed successfully!" << std::endl;
}

int main(int argc, char **argv) try {
    // 解析命令行参数，必须提供4个参数
    if (argc != 6) {
        std::cout << "Error: This program requires exactly 4 arguments." << std::endl;
        std::cout << "Usage: " << argv[0] << " ry_camera h_camera_base l_base_robot serial_port" << std::endl;
        std::cout << "Example: " << argv[0] << " 1.2042 330.0 -300.0 50 /dev/ttyUSB0" << std::endl;
        std::cout << "Where:" << std::endl;
        std::cout << "  ry_camera: camera rotation angle in radians around Y-axis" << std::endl;
        std::cout << "  h_camera_base: vertical height from camera to base in mm (Z direction)" << std::endl;
        std::cout << "  l_base_robot: horizontal distance from base to robot arm base in mm (Y axis)" << std::endl;
        std::cout << "  d_base_robot: deepth distance from base to robot arm base in mm (X axis)" << std::endl;
        std::cout << "  serial_port: serial port device file for robot arm connection" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    ry_camera = atof(argv[1]);
    h_camera_base = atof(argv[2]);
    l_base_robot = atof(argv[3]);
    d_base_robot = atof(argv[4]);
    const char* serial_port = argv[5];
    
    std::cout << "Using parameters:" << std::endl;
    std::cout << "ry_camera = " << ry_camera << " rad" << std::endl;
    std::cout << "h_camera_base = " << h_camera_base << " mm" << std::endl;
    std::cout << "l_base_robot = " << l_base_robot << " mm" << std::endl;
    std::cout << "d_base_robot = " << d_base_robot << " mm" << std::endl;
    std::cout << "serial_port = " << serial_port << std::endl;
    
    // 初始化机器人执行器，指定串口设备
    zyarm_sdk::ZyArmConfig robot_config;
    robot_config.port = serial_port;
    g_robotActor = new zyarm_sdk::ZyArm(robot_config);
    g_robotActor->connect();
    sleep(1);
    g_robotActor->reset();
    g_robotActor->set_speed(30); // 30mm/s
    
    // 创建Pipeline用于获取数据流
    ob::Pipeline pipe;

    // 创建配置对象
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    
    // 启用彩色流和深度流
    config->enableVideoStream(OB_STREAM_COLOR);
    config->enableVideoStream(OB_STREAM_DEPTH);
    
    // 设置对齐模式为软件对齐到彩色相机
    config->setAlignMode(ALIGN_D2C_SW_MODE);

    // 启动Pipeline
    pipe.start(config);
    
    // 获取彩色流配置信息
    auto colorProfile = pipe.getEnabledStreamProfileList()->getVideoStreamProfile(0);
    
    // 创建窗口用于显示彩色图像
    Window app("Robot Arm Pick Demo", colorProfile->width(), colorProfile->height());
    
    // 设置鼠标回调
    cv::setMouseCallback("Robot Arm Pick Demo", onMouse, nullptr);
    
    std::cout << "Robot Arm Pick Demo" << std::endl;
    std::cout << "Click on the color image to get the 3D coordinates at that point." << std::endl;
    std::cout << "Press ESC to exit." << std::endl;

    while(app) {
        // 等待数据帧
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        // 获取彩色帧和深度帧
        auto colorFrame = frameSet->colorFrame();
        auto depthFrame = frameSet->depthFrame();
        
        if(colorFrame == nullptr || depthFrame == nullptr) {
            continue;
        }

        // 渲染彩色帧到窗口
        app.addToRender(colorFrame);
        
        // 如果有鼠标点击事件
        if(g_clickPoint.valid) {
            // 检查点击坐标是否在图像范围内
            if(g_clickPoint.x >= 0 && g_clickPoint.x < (int)depthFrame->width() && 
               g_clickPoint.y >= 0 && g_clickPoint.y < (int)depthFrame->height()) {
                
                // 获取深度值
                uint16_t* depthData = (uint16_t*)depthFrame->data();
                uint16_t depthValue = depthData[g_clickPoint.y * depthFrame->width() + g_clickPoint.x];
                
                if(depthValue > 0) {
                    // 获取深度值比例
                    float depthScale = depthFrame->getValueScale();
                    float scaledDepth = depthValue * depthScale;
                    
                    // 计算点云坐标
                    OBPoint3f point3f;
                    OBPoint2f point2f = { static_cast<float>(g_clickPoint.x), static_cast<float>(g_clickPoint.y) };
                    
                    // 获取相机校准参数
                    auto calibrationParam = pipe.getCalibrationParam(config);
                    
                    // 将2D像素坐标转换为3D空间坐标
                    bool result = ob::CoordinateTransformHelper::calibration2dTo3d(
                        calibrationParam, 
                        point2f, 
                        scaledDepth, 
                        OB_SENSOR_DEPTH, 
                        OB_SENSOR_DEPTH, 
                        &point3f
                    );
                    
                    if(result) {
                        std::cout << "Clicked pixel (" << g_clickPoint.x << ", " << g_clickPoint.y 
                                  << ") -> Point cloud coordinate: (" 
                                  << point3f.x << ", " << point3f.y << ", " << point3f.z << ") mm" << std::endl;
                        
                        // 调用机器人移动函数
                        // if (g_robotActor && g_robotActor->is_connected()) {
                            OBPoint3f robotPoint;
                            transformPointCloudToRobotBase(&point3f, &robotPoint);
                            robotArmPick(g_robotActor, &robotPoint);
                        // }
                    } else {
                        std::cout << "Failed to compute 3D coordinates for pixel (" 
                                  << g_clickPoint.x << ", " << g_clickPoint.y << ")" << std::endl;
                    }
                } else {
                    std::cout << "Clicked pixel has no valid depth data." << std::endl;
                }
            } else {
                std::cout << "Clicked point is outside the image bounds." << std::endl;
            }
            
            // 重置点击状态
            g_clickPoint.valid = false;
        }
        
        // 检查按键事件
        int key = app.waitKey(1);
        if(key == ESC_KEY) {
            break;
        }
    }

    // 停止Pipeline
    pipe.stop();
    
    // 释放机器人执行器资源
    if (g_robotActor) {
        delete g_robotActor;
        g_robotActor = nullptr;
    }

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
