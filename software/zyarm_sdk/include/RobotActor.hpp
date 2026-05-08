#pragma once

#include <string>
#include <iostream>

class RobotActor {
private:
    std::string serialDevice_;
    bool isConnected_;
    int serial_fd_;  // 保存串口文件描述符

    /**
     * 等待并处理来自机器人的同步响应
     * @param cmdId 命令ID
     * @return true表示命令执行成功，false表示命令执行失败或出错
     */
    bool waitForResponse(int cmdId);

public:
    /**
     * 构造函数，需要指定串口设备
     * @param serialDevice 串口设备路径，例如 "/dev/ttyUSB0"
     */
    RobotActor(const std::string& serialDevice);
    
    /**
     * 移动到指定位置, 强制同步（等待执行完成）
     * @param x X坐标 (机器人坐标系)
     * @param y Y坐标 (机器人坐标系)
     * @param z Z坐标 (机器人坐标系)
     * @return 0表示成功，非0表示失败
     */
    int move(float x, float y, float z);
    
    /**
     * 复位机器人, 强制同步（等待执行完成）
     * @return 0表示成功，非0表示失败
     */
    int reset();
    
    /**
     * 控制夹爪开合
     * @param position 夹爪位置，范围0.0（闭合）到1.0（张开）
     * @param sync 是否同步执行（等待执行完成）
     * @return 0表示成功，非0表示失败
     */
    int claw(float position, bool sync = false);
    
    /**
     * 播放动作, 强制同步（等待执行完成）
     * @param actionId 动作ID
     * @param sync 是否同步执行（等待执行完成）
     * @return 0表示成功，非0表示失败
     */
    int actorPlayer(int actionId);
    
    /**
     * 设置关节速度
     * @param speed 关节速度
     * @return 0表示成功，非0表示失败
     */
    int setSpeed(float speed);
    
    /**
     * 检查机器人是否连接
     */
    bool isConnected() const { return isConnected_; }
    
    /**
     * 析构函数，关闭串口连接
     */
    ~RobotActor();
};