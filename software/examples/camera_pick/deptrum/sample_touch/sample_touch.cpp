// 基于深度相机的机械臂抓取系统 
// 功能：点击RGB窗口获取3D坐标并执行机械臂抓取

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"
#include "sample_helper.h"
#include "zyarm_sdk/arm.hpp"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// ==================== 配置参数 ====================
#define FRAME_MODE_INDEX 2      // 分辨率模式索引
#define IR_FPS 15               // IR相机帧率
#define ALIGN_MODE true         // 对齐模式
#define DEPTH_CORRECTION true   // 深度校正
#define ENABLE_RGB true         // 开启RGB流
#define ENABLE_DEPTH true       // 开启Depth流

// 手眼标定参数
#define RY_CAMERA 1.2042f        // 相机旋转角度（弧度）
#define H_CAMERA_BASE 330.0f     // 相机到基座高度（mm）
#define L_BASE_ROBOT -300.0f     // 基座到机械臂Y轴距离（mm）
#define D_BASE_ROBOT 50.0f       // 基座到机械臂X轴距离（mm）
#define ROBOT_BASE_OFFSET 103.8f // 机械臂基座高度偏移（mm）

// 机械臂配置
#define SERIAL_PORT "/dev/ttyUSB0" // 默认串口设备

//检查机械臂命令是否成功
#define CHECK_ROBOT(cmd, msg) if (!(cmd).accepted) std::cout << msg << std::endl;

// ==================== 配置参数结束 ====================

// 3D点结构体
struct Point3f {
    float x, y, z;
    Point3f() : x(0), y(0), z(0) {}
    Point3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// 全局变量
CameraParam g_camera_param;                       // 相机内参（fx, fy, cx, cy）
std::shared_ptr<uint16_t> g_depth_data = nullptr; // 深度数据缓冲区
int g_depth_rows = 0, g_depth_cols = 0;           // 深度图尺寸
cv::Mat g_rgb_image;                              // RGB图像缓冲区（主线程显示用）
std::mutex g_data_mutex;                          // 数据访问互斥锁
zyarm_sdk::ZyArm* g_robotActor = nullptr;         // 机械臂控制器
std::atomic<bool> g_robot_busy{false};            // 机械臂忙碌标志
std::atomic<bool> is_running{true};               // 主循环运行标志

/**
 * @brief 坐标转换：相机坐标系 -> 机械臂基座坐标系
 */
void transformCameraToRobotBase(const Point3f* cameraPoint, Point3f* robotPoint) {
    Point3f camera;
    camera.x = cameraPoint->z;
    camera.y = cameraPoint->x;
    camera.z = -cameraPoint->y;

    float cos_ry = cosf(RY_CAMERA);
    float sin_ry = sinf(RY_CAMERA);

    float x_rotated = cos_ry * camera.x + sin_ry * camera.z;
    float y_rotated = camera.y;
    float z_rotated = -sin_ry * camera.x + cos_ry * camera.z;

    robotPoint->x = x_rotated - D_BASE_ROBOT;
    robotPoint->y = y_rotated + L_BASE_ROBOT;
    robotPoint->z = z_rotated + H_CAMERA_BASE - ROBOT_BASE_OFFSET; 
}

/**
 * @brief 机械臂抓取动作序列
 */
void robotArmPick(zyarm_sdk::ZyArm* robot, Point3f* pickPoint) {
    std::cout << "========================================\n"
              << "机械臂坐标: X=" << pickPoint->x
              << " Y=" << pickPoint->y
              << " Z=" << pickPoint->z << " mm\n"
              << "========================================" << std::endl;

    if (!robot || !robot->is_connected()) {
        std::cout << "机械臂未连接" << std::endl;
        return;
    }

    std::cout << "开始抓取序列..." << std::endl;

    CHECK_ROBOT(robot->set_gripper(0.5, true), "机械臂爪爪打开失败");
    CHECK_ROBOT(robot->move_ik(pickPoint->x, pickPoint->y, -50), "机械臂移动失败");
    CHECK_ROBOT(robot->move_ik(pickPoint->x, pickPoint->y, -80), "机械臂移动失败");
    CHECK_ROBOT(robot->set_gripper(0.2, true), "机械臂爪爪关闭失败");
    CHECK_ROBOT(robot->move_ik(pickPoint->x, pickPoint->y, 0), "机械臂移动失败");
    CHECK_ROBOT(robot->play_record(2222), "机械臂播放动作失败");
    CHECK_ROBOT(robot->reset(), "机械臂重置失败");

    std::cout << "抓取完成" << std::endl;
}

/**
 * @brief 鼠标回调：点击获取3D坐标并触发抓取
 */
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;

    Point3f robotPoint;
    bool valid_point = false;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);

        if (g_depth_data && x >= 0 && x < g_depth_cols && y >= 0 && y < g_depth_rows) {
            uint16_t depth_mm = g_depth_data.get()[y * g_depth_cols + x];

            if (depth_mm > 0) {
                float z = depth_mm;
                float x_3d = (x - g_camera_param.cx) * z / g_camera_param.fx;
                float y_3d = (y - g_camera_param.cy) * z / g_camera_param.fy;

                std::cout << "点击位置: (" << x << ", " << y << ") 深度: " << depth_mm << " mm\n"
                          << "相机坐标: X=" << x_3d << " Y=" << y_3d << " Z=" << z << " mm" << std::endl;

                Point3f cameraPoint(x_3d, y_3d, z);
                transformCameraToRobotBase(&cameraPoint, &robotPoint);
                valid_point = true;
            } else {
                std::cout << "点击位置无有效深度值" << std::endl;
            }
        }
    }

    if (valid_point) {
        if (g_robot_busy.load()) {
            std::cout << "机械臂忙碌中，请稍后" << std::endl;
            return;
        }

        g_robot_busy.store(true);

        std::thread([robotPoint]() {
            Point3f pickPoint = robotPoint;
            robotArmPick(g_robotActor, &pickPoint);
            g_robot_busy.store(false);
        }).detach();
    }
}

/**
 * @brief 配置流类型
 * 根据宏定义自动选择RGB和Depth流类型
 */
void configureStreamTypes(std::shared_ptr<Device> device, std::vector<StreamType>& stream_types) {
    std::vector<StreamType> supported_types;
    device->GetSupportedStreamType(supported_types);

    std::cout << "配置流类型: ";

    // 同时开启RGB和Depth时，优先使用Rgbd组合流
    for (auto type : supported_types) {
        if (type == StreamType::kRgbd) {
            stream_types.push_back(StreamType::kRgbd);
            std::cout << "Rgbd ";
            std::cout << std::endl;
            return;
        }
    }

    std::cout << std::endl;
}

/**
 * @brief 数据流处理线程（仅负责获取数据）
 */
int streamProcess(std::shared_ptr<Device> device, const std::vector<StreamType>& stream_type) {
    Stream* stream = nullptr;
    if (device->CreateStream(stream, stream_type) != 0) {
        std::cerr << "创建流失败" << std::endl;
        return -1;
    }
    if (stream->Start() != 0) {
        std::cerr << "启动流失败" << std::endl;
        return -1;
    }

    StreamFrames frames;

    while (is_running) {
        int ret = stream->GetFrames(frames, 2000);
        if (ret != 0) continue;

        cv::Mat rgb_image_local;

        for (int i = 0; i < frames.count; i++) {
            auto frame = frames.frame_ptr[i];

            // 处理RGB彩色图像
            if (frame->frame_type == kRgbFrame) {
                if (frame->rows * frame->cols * 1.5f == frame->size) {
                    cv::Mat yuv_mat(frame->rows * 1.5f, frame->cols, CV_8UC1, frame->data.get());
                    cv::Mat temp_rgb;
                    cv::cvtColor(yuv_mat, temp_rgb, cv::COLOR_YUV2BGR_NV12);
                    rgb_image_local = temp_rgb.clone();  // 添加 clone() 确保深拷贝
                } else if (frame->data && frame->size > 0) {
                    cv::Mat rgb_mat(frame->rows, frame->cols, CV_8UC3, frame->data.get());
                    rgb_image_local = rgb_mat.clone();
                }
            // 处理Depth深度图像
            } else if (frame->frame_type == kDepthFrame) {
                std::lock_guard<std::mutex> lock(g_data_mutex);
                g_depth_rows = frame->rows;
                g_depth_cols = frame->cols;
                int depth_size = g_depth_rows * g_depth_cols;
                g_depth_data.reset(new uint16_t[depth_size], [](uint16_t* p) { delete[] p; });
                memcpy(g_depth_data.get(), frame->data.get(), depth_size * sizeof(uint16_t));
            }
        }

        // 将RGB图像传递给主线程
        if (!rgb_image_local.empty()) {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_rgb_image = rgb_image_local.clone();
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    return 0;
}

/**
 * @brief 初始化并启动设备
 */
int prepareDevice() {
    std::vector<DeviceInformation> device_list;
    if (DeviceManager::GetInstance()->GetDeviceList(device_list) != 0) return -1;
    if (device_list.empty()) {
        std::cerr << "未找到设备" << std::endl;
        return -1;
    }

    std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    if (!device || device->Open() != 0) return -1;

    std::string device_name = device->GetDeviceName();
    std::cout << "设备: " << device_name << " SDK版本: " << device->GetSdkVersion() << std::endl;

    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> resolution_vec;
    device->GetSupportedFrameMode(resolution_vec);

    int index = FRAME_MODE_INDEX < resolution_vec.size() ? FRAME_MODE_INDEX : 0;
    device->SetMode(std::get<0>(resolution_vec[index]),
                    std::get<1>(resolution_vec[index]),
                    std::get<2>(resolution_vec[index]));

    if (device_name.find("Aurora9") == 0) {
        auto device_900 = std::dynamic_pointer_cast<Aurora900>(device);
        device_900->SetIrFps(IR_FPS);
        device_900->SetLaserCurrent(1450);
        device_900->SwitchAlignedMode(ALIGN_MODE);
        if (ALIGN_MODE) device_900->DepthCorrection(DEPTH_CORRECTION);
        device_900->EnableUndistortRgb(true);

        Intrinsic ir_intri, rgb_intri;
        Extrinsic extrinsic;
        device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
        g_camera_param.cx = ir_intri.principal_point[0];
        g_camera_param.cy = ir_intri.principal_point[1];
        g_camera_param.fx = ir_intri.focal_length[0];
        g_camera_param.fy = ir_intri.focal_length[1];

        std::cout << "相机内参: fx=" << g_camera_param.fx << " fy=" << g_camera_param.fy
                  << " cx=" << g_camera_param.cx << " cy=" << g_camera_param.cy << std::endl;
    }

    std::vector<StreamType> stream_types;
    configureStreamTypes(device, stream_types);
    if (stream_types.empty()) return 0;

    // 启动数据采集线程
    std::vector<std::thread> threads;
    for (auto& type : stream_types) {
        threads.emplace_back(streamProcess, device, std::vector<StreamType>{type});
    }

    // 在主线程创建OpenCV窗口并处理GUI事件
    cv::namedWindow("RGB", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::setMouseCallback("RGB", onMouse, nullptr);
    std::cout << "RGB窗口已创建，点击图像可触发抓取，按 'q' 退出" << std::endl;

    // 主线程GUI循环
    int display_count = 0;
    while (is_running) {
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            if (!g_rgb_image.empty()) {
                cv::imshow("RGB", g_rgb_image);
            }
        }

        // 处理GUI事件，检测按键
        int key = cv::waitKey(30);
        if (key == 'q' || key == 'Q') {
            is_running = false;
        }
    }

    // 等待所有线程结束
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }

    cv::destroyWindow("RGB");
    device->Close();
    return 0;
}

int main(int argc, char** argv) {
    std::string serial_port = (argc > 1) ? argv[1] : SERIAL_PORT;

    std::cout << "初始化机械臂，串口: " << serial_port << std::endl;
    zyarm_sdk::ZyArmConfig robot_config;
    robot_config.port = serial_port;
    g_robotActor = new zyarm_sdk::ZyArm(robot_config);
    g_robotActor->connect();

    if (g_robotActor && g_robotActor->is_connected()) {
        std::cout << "机械臂连接成功" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        g_robotActor->reset();
        g_robotActor->set_speed(30);
    } else {
        std::cerr << "机械臂连接失败，仅打印坐标" << std::endl;
    }

    prepareDevice();

    while (g_robot_busy.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (g_robotActor) {
        delete g_robotActor;
        g_robotActor = nullptr;
    }

    return 0;
}
