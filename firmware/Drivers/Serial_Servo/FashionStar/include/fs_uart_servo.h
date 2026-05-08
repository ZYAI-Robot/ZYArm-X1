/*
 * Fashion Star 总线伺服舵机驱动库
 * Version: v0.0.2
 * UpdateTime: 2024/07/17
 */

#ifndef __FASHION_STAR_USART_SERVO_H
#define __FASHION_STAR_USART_SERVO_H

#include "math.h"
#include "usart.h"
#include "ring_buffer.h"
#include "fs_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#define COUNTDOWN_MS 100

// FSUS通信协议
// 注: FSUS是Fashion Star Uart Servo的缩写

// 串口通讯超时设置
#define FSUS_TIMEOUT_MS FSUS_TRANSACTION_TIMEOUT_MS
// 舵机用户自定义数据块的大小 单位Byte
#define FSUS_SERVO_BATCH_DATA_SIZE 32
// 返回的响应数据包最长的长度
#define FSUS_PACK_RESPONSE_MAX_SIZE 350
// 在总线伺服舵机的通信系统设计里, 使用的字节序为Little Endian(低字节序/小端格式)
// STM32系统的数值存储模式就是Little Endian
// 所以0x4c12 这个数值, 在实际发送的时候低位会在前面 0x12, 0x4c
#define FSUS_PACK_REQUEST_HEADER 0x4c12
#define FSUS_PACK_RESPONSE_HEADER 0x1c05

// FSUS控制指令数据
// 注: 一下所有的指令都是针对单个舵机的
#define FSUS_CMD_NUM 30
#define FSUS_CMD_PING 1                               // 舵机通讯检测
#define FSUS_CMD_RESET_USER_DATA 2                    // 重置用户数据
#define FSUS_CMD_READ_DATA 3                          // 单个舵机 读取数据库
#define FSUS_CMD_WRITE_DATA 4                         // 单个舵机 写入数据块
#define FSUS_CMD_READ_BATCH_DATA 5                    // 单个舵机 批次读取(读取一个舵机所有的数据)
#define FSUS_CMD_WRITE_BATCH_DATA 6                   // 单个舵机 批次写入(写入一个舵机所有的数据)
#define FSUS_CMD_SPIN 7                               // 单个舵机 设置轮式模式
#define FSUS_CMD_ROTATE 8                             // 角度控制模式(设置舵机的角度))
#define FSUS_CMD_DAMPING 9                            // 阻尼模式
#define FSUS_CMD_READ_ANGLE 10                        // 舵机角度读取
#define FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL 11       // 角度设置(指定周期)
#define FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY 12       // 角度设置(指定转速)
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN 13             // 多圈角度设置
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL 14 // 多圈角度设置(指定周期)
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY 15 // 多圈角度设置(指定转速)
#define FSUS_CMD_QUERY_SERVO_ANGLE_MTURN 16           // 查询舵机角度(多圈)
#define FSUS_CMD_RESERT_SERVO_ANGLE_MTURN 17          // 重置舵机多圈角度
#define FSUS_CMD_BEGIN_ASYNC 18                       // 开始异步命令
#define FSUS_CMD_END_ASYNC 19                      		// 结束异步命令
#define FSUS_CMD_SET_SERVO_ReadData 22                // 舵机数据监控
#define FSUS_CMD_SET_ORIGIN_POINT 23                  // 设置零点
#define FSUS_CMD_CONTROL_MODE_STOP 24                 // 控制模式停止指令
#define FSUS_CMD_SET_SERVO_SyncCommand 25             // 同步命令

// FSUS状态码
#define FSUS_STATUS uint8_t
#define FSUS_STATUS_SUCCESS 0               // 设置/读取成功
#define FSUS_STATUS_FAIL 1                  // 设置/读取失败
#define FSUS_STATUS_TIMEOUT 2               // 等待超时
#define FSUS_STATUS_WRONG_RESPONSE_HEADER 3 // 响应头不对
#define FSUS_STATUS_UNKOWN_CMD_ID 4         // 未知的控制指令
#define FSUS_STATUS_SIZE_TOO_BIG 5          // 参数的size大于FSUS_PACK_RESPONSE_MAX_SIZE里面的限制
#define FSUS_STATUS_CHECKSUM_ERROR 6        // 校验和错误
#define FSUS_STATUS_ID_NOT_MATCH 7          // 请求的舵机ID跟反馈回来的舵机ID不匹配
#define FSUS_STATUS_ERRO 8                  // 设置同步模式错误
#define FSUS_STATUS_SEND_FAIL 9             // 请求发送失败
#define FSUS_STATUS_TX_ERROR_EVENT 10       // 等待过程中收到发送错误事件
#define FSUS_STATUS_REPLY_CMD_MISMATCH 11   // 回复命令字与预期不匹配
#define FSUS_STATUS_REPLY_SIZE_MISMATCH 12  // 回复长度与预期不匹配
#define FSUS_STATUS_REPLY_INCOMPLETE 13     // 批量同步回复未收齐

// 静止状态判断条件
#define FSUS_ANGLE_DEADAREA 2.0f  // 电机角度死区
#define FSUS_WAIT_COUNT_MAX 10000 // 等待重复查询角度的最大次数

/* 舵机只读数据ID及使用说明 (只读)*/
#define FSUS_PARAM_VOLTAGE 1    // 电压 (单位mV)
#define FSUS_PARAM_CURRENT 2    // 电流 (单位mA)
#define FSUS_PARAM_POWER 3      // 功率 (单位mw)
#define FSUS_PARAM_TEMPRATURE 4 // 温度 (单位ADC)

/*
舵机工作状态
BIT[0] - 执行指令置1，执行完成后清零。
BIT[1] - 执行指令错误置1，在下次正确执行后清零。
BIT[2] - 堵转错误置1，解除堵转后清零。
BIT[3] - 电压过高置1，电压恢复正常后清零。
BIT[4] - 电压过低置1，电压恢复正常后清零。
BIT[5] - 电流错误置1，电流恢复正常后清零。
BIT[6] - 功率错误置1，功率恢复正常后清零。
BIT[7] - 温度错误置1，温度恢复正常后清零。
*/
#define FSUS_PARAM_SERVO_STATUS 5 // 舵机工作状态 (字节长度 1)

/* 舵机用户自定义参数的数据ID及使用说明 (可度也可写)*/

/* 此项设置同时具备两个功能
 * 在轮式模式与角度控制模式下
 * 1. 舵机指令是否可以中断 interruptable?
 * 2. 是否产生反馈数据?
 * 0x00(默认)
 *      舵机控制指令执行可以被中断, 新的指令覆盖旧的指令
 *      无反馈数据
 * 0x01
 *      舵机控制指令不可以被中断, 新的指令添加到等候队列里面
 *      等候队列的长度是1, 需要自己在程序里面维护一个队列
 *      当新的控制指令超出了缓冲区的大小之后, 新添加的指令被忽略
 *      指令执行结束之后发送反馈数据
 */
#define FSUS_PARAM_RESPONSE_SWITCH 33
/*
 * 舵机的ID号, (字节长度 1)
 * 取值范围是 0-254
 * 255号为广播地址，不能赋值给舵机 广播地址在PING指令中使用。
 */
#define FSUS_PARAM_SERVO_ID 34
/*
 * 串口通讯的波特率ID  (字节长度 1)
 * 取值范围 [0x01,0x07] , 默认值0x05
 * 0x01-9600,
 * 0x02-19200,
 * 0x03-38400,
 * 0x04-57600,
 * 0x05-115200 (默认波特率),
 * 0x06-250000,
 * 0x07-500000,
 * 波特率设置时即生效
 */
#define FSUS_PARAM_BAUDRATE 36

/* 舵机保护值相关设置, 超过阈值舵机就进入保护模式 */
/*
 * 舵机堵转保护模式  (字节长度 1)
 * 0x00-模式1 降功率到堵轉功率上限
 * 0x01-模式2 释放舵机锁力
 */
#define FSUS_PARAM_STALL_PROTECT 37
/* 舵机堵转功率上限, (单位mW) (字节长度 2) */
#define FSUS_PARAM_STALL_POWER_LIMIT 38
/* 舵机电压下限 (单位mV) (字节长度 2) */
#define FSUS_PARAM_OVER_VOLT_LOW 39
/* 舵机电压上限 (单位mV) (字节长度 2) */
#define FSUS_PARAM_OVER_VOLT_HIGH 40
/* 舵机温度上限 (单位 摄氏度) (字节长度 2) */
#define FSUS_PARAM_OVER_TEMPERATURE 41
/* 舵机功率上限 (单位mW) (字节长度 2) */
#define FSUS_PARAM_OVER_POWER 42
/* 舵机电流上限 (单位mA) (字节长度 2) */
#define FSUS_PARAM_OVER_CURRENT 43
/*
 * 舵机启动加速度处理开关 (字节长度 1)
 * 0x00 不启动加速度处理 (无效设置)
 * 0x01 启用加速度处理(默认值)
 *      舵机梯形速度控制,根据时间t推算加速度a
 *      行程前1/4 加速
 *      行程中间1/2保持匀速
 *      行程后1/4
 */
#define FSUS_PARAM_ACCEL_SWITCH 44
/*
 * 舵机上电锁力开关 (字节长度 1)
 * 0x00 上电舵机释放锁力(默认值)
 * 0x11 上电时刹车
 */
#define FSUS_PARAM_POWER_ON_LOCK_SWITCH 46
/*
 * [轮式模式] 轮式模式刹车开关 (字节长度 1)
 * 0x00 停止时舵机释放锁力(默认)
 * 0x01 停止时刹车
 */
#define FSUS_PARAM_WHEEL_MODE_BRAKE_SWITCH 47
/*
 * [角度模式] 角度限制开关 (字节长度 1)
 * 0x00 关闭角度限制
 * 0x01 开启角度限制
 * 注: 只有角度限制模式开启之后, 角度上限下限才有效
 */
#define FSUS_PARAM_ANGLE_LIMIT_SWITCH 48
/*
 * [角度模式] 舵机上电首次角度设置缓慢执行 (字节长度 1)
 * 0x00 关闭
 * 0x01 开启
 * 开启后更安全
 * 缓慢旋转的时间周期即为下方的”舵机上电启动时间“
 */
#define FSUS_PARAM_SOFT_START_SWITCH 49
/*
 * [角度模式] 舵机上电启动时间 (单位ms)(字节长度 2)
 * 默认值: 0x0bb8
 */
#define FSUS_PARAM_SOFT_START_TIME 50
/*
 * [角度模式] 舵机角度上限 (单位0.1度)(字节长度 2)
 */
#define FSUS_PARAM_ANGLE_LIMIT_HIGH 51
/*
 * [角度模式] 舵机角度下限 (单位0.1度)(字节长度 2)
 */
#define FSUS_PARAM_ANGLE_LIMIT_LOW 52
/*
 * [角度模式] 舵机中位角度偏移 (单位0.1度)(字节长度 2)
 */
#define FSUS_PARAM_ANGLE_MID_OFFSET 53

// 帧头接收完成的标志位
#define FSUS_RECV_FLAG_HEADER 0x01
// 控制指令接收完成的标志位
#define FSUS_RECV_FLAG_CMD_ID 0x02
// 内容长度接收完成的标志位
#define FSUS_RECV_FLAG_SIZE 0x04
// 内容接收完成的标志位
#define FSUS_RECV_FLAG_CONTENT 0x08
// 校验和接收的标志位
#define FSUS_RECV_FLAG_CHECKSUM 0x10

// 总线伺服舵机用户自定义设置
// 是否开启响应模式
#define FSUS_IS_RESPONSE_ON 0

// 数据帧结构体（统一结构）
typedef struct {
    uint16_t header;
    uint8_t cmdId;
    uint16_t size;  // 统一使用16位长度
    uint8_t content[FSUS_PACK_RESPONSE_MAX_SIZE];
    uint8_t checksum;
    uint8_t status;
    uint8_t isSync;  // 新增同步标志
} PackageTypeDef;

// 函数声明
void FSUS_Package2RingBuffer(PackageTypeDef *pkg, RingBufferTypeDef *ringBuf);
// 计算校验和
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg);
// 判断是否为有效的请求头的
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg);
// 字节数组转换为数据帧
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg);
// 构造发送数据帧
FSUS_STATUS FSUS_SendPackage_Common(Usart_DataTypeDef *usart, uint8_t cmdId, uint16_t size, uint8_t *content, uint8_t isSync);
// 接收数据帧 (在接收的时候动态的申请内存)
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg);
FSUS_STATUS FSUS_sync_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg);

// 舵机通讯检测
// 注: 如果没有舵机响应这个Ping指令的话, 就会超时
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id);

// 重置舵机的用户资料
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id);

// 读取数据
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t *size);

// 写入数据
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size);

// 设置舵机的角度
// @angle 单位度
// @interval 单位ms
// @power 舵机执行功率 单位mW
//        若power=0或者大于保护值
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power);

/* 设置舵机的角度(指定周期) */
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id,
                                         float angle, uint16_t interval, uint16_t t_acc,
                                         uint16_t t_dec, uint16_t power);

/* 设置舵机的角度(指定转速) */
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id,
                                         float angle, float velocity, uint16_t t_acc,
                                         uint16_t t_dec, uint16_t power);

/* 查询单个舵机的角度信息 angle 单位度 */
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle);

/* 设置舵机的角度(多圈模式) */
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
                                    uint32_t interval, uint16_t power);

/* 设置舵机的角度(多圈模式, 指定周期) */
FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
                                              uint32_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power);

/* 设置舵机的角度(多圈模式, 指定转速) */
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
                                              float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power);

/* 查询舵机的角度(多圈模式) */
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle);

/* 舵机阻尼模式 */
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power);

/*重置多圈舵机角度*/
FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id);

/*零点设置 仅适用于无刷磁编码舵机*/
FSUS_STATUS FSUS_SetOriginPoint(Usart_DataTypeDef *usart, uint8_t servo_id);

/* 舵机开始异步命令*/
FSUS_STATUS FSUS_BeginAsync(Usart_DataTypeDef *usart);

/* 舵机结束异步命令*/
FSUS_STATUS FSUS_EndAsync(Usart_DataTypeDef *usart,uint8_t mode);

// 舵机的数据结构体
typedef struct {
    uint8_t id;
    int16_t voltage;
    int16_t current;
    int16_t power;
    int16_t temperature;
    uint8_t status;
    float angle;
    int16_t circle_count;
} ServoData;
/* 舵机单个数据监控*/
FSUS_STATUS FSUS_ServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_id, ServoData servodata[]);
FSUS_STATUS FSUS_ServoMonitorSyncGroup(
    Usart_DataTypeDef *usart,
    const uint8_t servo_ids[],
    uint8_t servo_count,
    ServoData servodata[]
);
/* 舵机控制模式停止指令*/
FSUS_STATUS FSUS_StopOnControlMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t mode, uint16_t power);

/*同步命令的模式选择
* 1：设置舵机的角度
* 2：设置舵机的角度(指定周期)
* 3：设置舵机的角度(指定转速)
* 4：设置舵机的角度(多圈模式)
* 5：设置舵机的角度(多圈模式, 指定周期) 
* 6：设置舵机的角度(多圈模式, 指定转速)*/
typedef enum {
    MODE_SET_SERVO_ANGLE = 1,
    MODE_SET_SERVO_ANGLE_BY_INTERVAL = 2,
		MODE_SET_SERVO_ANGLE_BY_VELOCITY = 3,
    MODE_SET_SERVO_ANGLE_MTURN = 4,
    MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL = 5,
    MODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY = 6
} ServoMode;

/*同步命令的舵机设置参数结构体*/
typedef struct
{
	uint8_t id;
	float angle;
	float velocity;
	uint16_t interval_single;
	uint32_t interval_multi;
	uint16_t t_acc; 
	uint16_t t_dec;
	uint16_t power;
}FSUS_sync_servo;

extern FSUS_sync_servo SyncArray[20]; // 假设您要控制20个伺服同步

/* 同步命令选择模式控制函数*/
FSUS_STATUS FSUS_SyncCommand(Usart_DataTypeDef *usart, uint8_t servo_count, uint8_t ServoMode, FSUS_sync_servo servoSync[]);

#endif
