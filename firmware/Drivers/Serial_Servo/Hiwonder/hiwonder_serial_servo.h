#ifndef __HIWONDER_SERIAL_SERVO_H
#define __HIWONDER_SERIAL_SERVO_H

#if (defined(ARM_ROBOT_SERVO_USE_HIWONDER) && (ARM_ROBOT_SERVO_USE_HIWONDER))

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SERIAL_SERVO_FRAME_HEADER         0x55
#define SERIAL_SERVO_MOVE_TIME_WRITE      1
#define SERIAL_SERVO_MOVE_TIME_READ       2
#define SERIAL_SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERIAL_SERVO_MOVE_TIME_WAIT_READ  8
#define SERIAL_SERVO_MOVE_START           11
#define SERIAL_SERVO_MOVE_STOP            12

#define SERIAL_SERVO_ID_WRITE             13
#define SERIAL_SERVO_ID_READ              14
#define SERIAL_SERVO_ANGLE_OFFSET_ADJUST  17
#define SERIAL_SERVO_ANGLE_OFFSET_WRITE   18
#define SERIAL_SERVO_ANGLE_OFFSET_READ    19
#define SERIAL_SERVO_ANGLE_LIMIT_WRITE    20
#define SERIAL_SERVO_ANGLE_LIMIT_READ     21
#define SERIAL_SERVO_VIN_LIMIT_WRITE      22
#define SERIAL_SERVO_VIN_LIMIT_READ       23
#define SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERIAL_SERVO_TEMP_MAX_LIMIT_READ  25
#define SERIAL_SERVO_TEMP_READ            26
#define SERIAL_SERVO_VIN_READ             27
#define SERIAL_SERVO_POS_READ             28
#define SERIAL_SERVO_OR_MOTOR_MODE_WRITE  29
#define SERIAL_SERVO_OR_MOTOR_MODE_READ   30
#define SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERIAL_SERVO_LOAD_OR_UNLOAD_READ  32
#define SERIAL_SERVO_LED_CTRL_WRITE       33
#define SERIAL_SERVO_LED_CTRL_READ        34
#define SERIAL_SERVO_LED_ERROR_WRITE      35
#define SERIAL_SERVO_LED_ERROR_READ       36

#define SERIAL_ANGLE_STEP                 (0.24f)
#define SERIAL_SERVO_CMD_INTERVAL         (1)  // 串口指令间隔ms

#define CMD_SERVO_MOVE 0x03

#pragma pack(1)
#pragma anon_unions  // 添加这行来启用匿名联合支持
typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    union {
        struct {
            uint8_t servo_id;
            uint8_t length;
            uint8_t command;
            uint8_t args[8];
        } elements;
        uint8_t data_raw[11];
    };
}SerialServoCmdTypeDef;
#pragma pack()

typedef enum {
    SERIAL_SERVO_RECV_STARTBYTE_1,
    SERIAL_SERVO_RECV_STARTBYTE_2,
    SERIAL_SERVO_RECV_SERVO_ID,
    SERIAL_SERVO_RECV_LENGTH,
    SERIAL_SERVO_RECV_COMMAND,
    SERIAL_SERVO_RECV_ARGUMENTS,
    SERIAL_SERVO_RECV_CHECKSUM,
} SerialServoRecvState;

typedef enum {
    SERIAL_SERVO_WRITE_DATA_READY,
		SERIAL_SERVO_WRITE_DATA,
    SERIAL_SERVO_WRITE_DATA_FINISH,
    SERIAL_SERVO_READ_DATA,
		SERIAL_SERVO_READ_DATA_FINISH,	
		SERIAL_SERVO_READ_DATA_ERROR
} SerialServoITState;

typedef struct SerialServoControllerTypeDef SerialServoControllerTypeDef;
struct SerialServoControllerTypeDef {
	SerialServoITState it_state; 
    SerialServoRecvState rx_state;
    SerialServoCmdTypeDef rx_frame;
    uint32_t rx_args_index;

    SerialServoCmdTypeDef tx_frame;
    uint32_t tx_byte_index;
    bool tx_only;

	void (*write_pin)(uint8_t new_state);
    int8_t (*serial_write_and_read)(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only);
};

extern SerialServoControllerTypeDef serial_servo_controller;


/**
 * @brief 总线舵机初始化
 * 
 * @param  NULL
 * 
 * @return None
 */
void serial_servo_init(void);

/**
 * @brief 舵机ID设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  old_id           需要设置的舵机ID
 * @param  new_id           写入的舵机新ID
 * @return None
 */
void serial_servo_set_id(SerialServoControllerTypeDef *self, uint8_t old_id, uint8_t new_id);

/**
 * @brief 舵机ID读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取的舵机ID
 * @param  ret_servo_id     读取到的舵机ID
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_id(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *ret_servo_id);

/**
 * @brief 舵机位置设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置的舵机ID
 * @param  position         指定的舵机预设位置，可设定的范围为0~1000
 * @param  duration         舵机运行时间，可设定的范围为0~30000ms
 * @return None
 */
void serial_servo_set_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int position, uint16_t duration);

/**
 * @brief 舵机位置读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取的舵机ID
 * @param  position         读取到的舵机位置
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int *position);

/**
 * @brief 舵机偏差设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置的舵机ID
 * @param  new_deviation    偏差值，可设定的范围为-100~100
 * @return None
 */
void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int new_deviation);

/**
 * @brief 舵机偏差保存函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要保存偏差的舵机ID
 * @return None
 */
void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id);

/**
 * @brief 舵机偏差读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取偏差的舵机ID
 * @param  deviation        读到的舵机偏差值
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int8_t *deviation);

/**
 * @brief 舵机角度范围设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置角度范围的舵机ID
 * @param  limit_l          舵机角度设置最小值，可设定的角度范围为0~1000
 * @param  limit_h          舵机角度设置最大值，可设定的角度范围为0~1000
 * @return None
 */
void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h);

/**
 * @brief 舵机角度范围读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取舵机角度范围的舵机ID
 * @param  limit            读取到的舵机角度范围
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2]);

/**
 * @brief 舵机内部最大温度设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置温度范围的舵机ID
 * @param  limit            舵机最大温度设置，可设定的温度范围为0~100
 * @return None
 */
void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit);

/**
 * @brief 舵机内部最大温度读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取温度范围的舵机ID
 * @param  limit            读取到的舵机最大温度值
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *limit);

/**
 * @brief 舵机内部当前温度读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取当前内部温度的舵机ID
 * @param  temp             读取到的舵机当前内部温度
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_temp(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *temp);

/**
 * @brief 舵机电压范围设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置电压范围的舵机ID
 * @param  limit_l          需要设置的最小电压值，可设定的范围为4500~14000（单位：mv）
 * @param  limit_h          需要设置的最大电压值，可设定的范围为4500~14000（单位：mv）
 * @return None
 */
void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h);

/**
 * @brief 舵机电压范围读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取电压范围的舵机ID
 * @param  limit            读取到的电压最大最小值（单位：mv）
 * 
 * @return true     读取成功
 * @return false    读取失败
 */
bool serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2]);

/**
 * @brief 舵机当前电压读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取当前电压的舵机ID
 * @param  vin              读取到的当前舵机电压（单位：mv）
 * @return true 
 * @return false 
 */
bool serial_servo_read_vin(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t *vin);

/**
 * @brief 停止当前舵机运动
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要停止当前运动的舵机ID
 * @return None
 */
void serial_servo_stop(SerialServoControllerTypeDef *self, uint8_t servo_id);

/**
 * @brief 舵机上电/掉电状态设置函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要设置当前状态的舵机ID
 * @param  load
 *   @arg  0 掉电
 *   @arg  1 上电
 * @return None
 */
void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t load);

/**
 * @brief 舵机上电/掉电状态读取函数
 * 
 * @param  self             指向SerialServoControllerTypeDef类型的指针
 * @param  servo_id         需要读取当前状态的舵机ID
 * @param  load_unload      读取到的状态值
 *   @arg  0 掉电
 *   @arg  1 上电
 * @return true     读取成功
 * @return false    读取失败 
 */
bool serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t* load_unload);

/**
 * @brief 校验和计算内联函数
 * 
 * @param  buf      需要计算校验和的数组
 * @return uint8_t  校验和计算结构
 */
static inline uint8_t serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

/**
 * @brief   总线舵机串口接收处理内联函数
 * 
 * @param   self             指向SerialServoControllerTypeDef类型的指针
 * @param   rx_byte          接收到的数据（单个字节）
 * @return  int              当前校验结果
 *    @arg   -1   舵机帧头1校验成功OR失败
 *    @arg   -2   舵机帧头2校验成功OR失败
 *    @arg    1   舵机ID校验完毕
 *    @arg   -3   数据包长度校验失败
 *    @arg    2   数据包长度校验成功
 *    @arg    3   舵机命令校验成功
 *    @arg    4   舵机数据接收完成
 *    @arg  -99   校验和匹配失败
 *    @arg    0   校验和匹配成功
 *    @arg -100   未知错误
 */
static inline int serial_servo_rx_handler(SerialServoControllerTypeDef *self, uint8_t rx_byte)
{
    switch (self->rx_state) {
        case SERIAL_SERVO_RECV_STARTBYTE_1: {
            self->rx_state = SERIAL_SERVO_FRAME_HEADER == rx_byte ? SERIAL_SERVO_RECV_STARTBYTE_2 : SERIAL_SERVO_RECV_STARTBYTE_1;
			if(self->rx_state == SERIAL_SERVO_RECV_STARTBYTE_1)
			{
				self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
			}
            self->rx_frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
            return -1;
        }
        case SERIAL_SERVO_RECV_STARTBYTE_2: {
            self->rx_state = SERIAL_SERVO_FRAME_HEADER == rx_byte ? SERIAL_SERVO_RECV_SERVO_ID : SERIAL_SERVO_RECV_STARTBYTE_1;
			if(self->rx_state == SERIAL_SERVO_RECV_STARTBYTE_1)
			{
				self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
			}
            self->rx_frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
            return -2;
        }
        case SERIAL_SERVO_RECV_SERVO_ID: {
            self->rx_frame.elements.servo_id = rx_byte;
			self->rx_state = SERIAL_SERVO_RECV_LENGTH;
            return 1;
        }
        case SERIAL_SERVO_RECV_LENGTH: {
			/* 包长度超过允许长度 */
            if(rx_byte > 7) {
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1; 
				self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
                return -3;
            }
            self->rx_frame.elements.length = rx_byte;
            self->rx_state = SERIAL_SERVO_RECV_COMMAND;
            return 2;
        }
        case SERIAL_SERVO_RECV_COMMAND: {
            self->rx_frame.elements.command = rx_byte;
            self->rx_args_index = 0;
            /* 若没有参数则直接进入校验字段 */
            self->rx_state = self->rx_frame.elements.length == 6 ? SERIAL_SERVO_RECV_CHECKSUM : SERIAL_SERVO_RECV_ARGUMENTS; 
            return 3;
        }
        case SERIAL_SERVO_RECV_ARGUMENTS: {
            self->rx_frame.elements.args[self->rx_args_index++] = rx_byte;
            if (self->rx_args_index + 3 == self->rx_frame.elements.length) {
                self->rx_state = SERIAL_SERVO_RECV_CHECKSUM;
            }
            return 4;
        }
        case SERIAL_SERVO_RECV_CHECKSUM: {
            if(serial_servo_checksum((uint8_t*)&self->rx_frame) != rx_byte) {
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
				self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
                return -99;
            } else {
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
                return 0;
            }
        }

        default: {
            self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
			self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
            return -100;
        }
    }
}
#endif
#endif
