#include "arm_robot_config.h"

#if (defined(ARM_ROBOT_SERVO_USE_HIWONDER) && (ARM_ROBOT_SERVO_USE_HIWONDER))
#include <stdio.h>
#include "serial_servo.h"
#include "global.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

SerialServoControllerTypeDef serial_servo_controller;

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 将高地八位合成为十六位



/**
 * @brief 自动填充数据帧的帧头、ID、命令字段
 * 
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  servo_id   舵机id号
 * @param  cmd        控制命令
 *   @arg  SERIAL_SERVO_MOVE_TIME_WRITE         设置舵机位置与运行时间
 *   @arg  SERIAL_SERVO_POS_READ                读取舵机位置
 *   @arg  SERIAL_SERVO_ID_WRITE                设置舵机ID
 *   @arg  SERIAL_SERVO_ID_READ                 读取舵机ID
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_ADJUST     设置舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_WRITE      保存舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_READ       读取舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_LIMIT_WRITE       设置舵机角度范围
 *   @arg  SERIAL_SERVO_ANGLE_LIMIT_READ        读取舵机角度范围
 *   @arg  SERIAL_SERVO_VIN_LIMIT_WRITE         设置舵机电压范围
 *   @arg  SERIAL_SERVO_VIN_LIMIT_READ          读取舵机电压范围
 *   @arg  SERIAL_SERVO_VIN_READ                读取当前舵机电压
 *   @arg  SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE    设置舵机温度范围
 *   @arg  SERIAL_SERVO_TEMP_MAX_LIMIT_READ     读取舵机温度范围
 *   @arg  SERIAL_SERVO_TEMP_READ               读取当前舵机温度
 *   @arg  SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE    设置舵机状态(上电/掉电)
 *   @arg  SERIAL_SERVO_LOAD_OR_UNLOAD_READ     读取舵机状态(上电/掉电)
 *   @arg  SERIAL_SERVO_MOVE_STOP               停止舵机运行
 */
static void cmd_frame_init(SerialServoCmdTypeDef *frame, int servo_id, uint8_t cmd)
{
    frame->header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame->header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame->elements.servo_id = servo_id;
    frame->elements.command = cmd;
}

/**
 * @brief 自动填充数据帧的数据长度、校验值字段
 * 
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  args_num   发送的数据个数
 */

static void cmd_frame_complete(SerialServoCmdTypeDef *frame, uint8_t args_num)
{
    frame->elements.length = args_num + 3;
    frame->elements.args[args_num] = serial_servo_checksum((uint8_t*)frame);
//	frame->elements.args[args_num + 1] = 0x00;
}

static void write_pin(uint8_t new_state)
{
	
	if(new_state==0)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_SET);
	}
	else if(new_state==1)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_SET);
	}
}

/**
 * @brief 读写总线舵机数据
 * 
 * @param  self       指向SerialServoCmdTypeDef类型的指针
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  tx_only    读写标志符
 * @return int8_t     1-已完成数据帧发送 0-已完成数据帧读取
 */

static int8_t serial_write_and_read(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only)
{

    int8_t ret = -1;	
	switch(self->it_state)
	{
		case SERIAL_SERVO_WRITE_DATA_READY:
			/* 进入写模式 */
			self->write_pin(0);
			memcpy(&self->tx_frame, frame, sizeof(SerialServoCmdTypeDef));
			self->tx_byte_index = 0;
			self->tx_only = tx_only;
			self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
			__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
			__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
			__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TXE);
			
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_TXE);
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
			
			ret = 1;
			break;
		
		case SERIAL_SERVO_READ_DATA_FINISH:
			self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
			ret = 0;
			break;
		
		case SERIAL_SERVO_READ_DATA_ERROR:
			self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
			break;
			
		default:
			break;
	}
	return ret;
}

void serial_servo_set_id(SerialServoControllerTypeDef *self, uint8_t old_id, uint8_t new_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, old_id, SERIAL_SERVO_ID_WRITE);
    frame.elements.args[0] = new_id;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_id(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *ret_servo_id)
{
	SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ID_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
        *ret_servo_id = self->rx_frame.elements.args[0];
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int position, uint16_t duration)
{
    SerialServoCmdTypeDef frame;
    position = position > 1000 ? 1000 : position;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(position);
    frame.elements.args[1] = GET_HIGH_BYTE(position);
    frame.elements.args[2] = GET_LOW_BYTE(duration);
    frame.elements.args[3] = GET_HIGH_BYTE(duration);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int *position)
{
	SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_POS_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false)) 
	{
        *position = (int)(*((int16_t*)self->rx_frame.elements.args));
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int new_deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_ADJUST);
    frame.elements.args[0] = (uint8_t) ((int8_t) new_deviation);
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_WRITE);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}


bool serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int8_t *deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
        *deviation = (int8_t)(self->rx_frame.elements.args[0]);
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
	SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_WRITE);
    limit_l = limit_l > 1000 ? 1000 : limit_l;
	limit_h = limit_h > 1000 ? 1000 : limit_h;
	uint32_t real_limit_l = limit_l > limit_h ? limit_h : limit_l;
	uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
		limit[0] = *((uint16_t*)(&self->rx_frame.elements.args[0]));
		limit[1] = *((uint16_t*)(&self->rx_frame.elements.args[2]));
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;			
        return true;
    }
    return false;
}

void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE);
    frame.elements.args[0] = limit > 100 ? 100 : (uint8_t)limit;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false)) 
	{
        *limit = (uint8_t)(self->rx_frame.elements.args[0]);
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;	
        return true;
    }
    return false;
}

bool serial_servo_read_temp(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *temp)
{
    SerialServoCmdTypeDef frame;	
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
        *temp = (uint8_t)(self->rx_frame.elements.args[0]);
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;	
        return true;
    }
    return false;
}

void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_WRITE);
    limit_l = limit_l < 4500 ? 4500 : limit_l;
    limit_h = limit_h > 14000 ? 14000 : limit_h;
	uint32_t real_limit_l  = limit_l > limit_h ? limit_h : limit_l;
	uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;	
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
        limit[0] = *((uint16_t*)(&self->rx_frame.elements.args[0]));
		limit[1] = *((uint16_t*)(&self->rx_frame.elements.args[2]));
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;	
        return true;
    }
    return false;
}

bool serial_servo_read_vin(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t *vin)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_READ);
    cmd_frame_complete(&frame, 0);
    if(0 == self->serial_write_and_read(self, &frame, false))
	{
        *vin = ((uint32_t) * ((uint16_t*)self->rx_frame.elements.args));
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;	
        return true;
    }
    return false;
}

void serial_servo_stop(SerialServoControllerTypeDef *self, uint8_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_STOP);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t load)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE);
    frame.elements.args[0] = load;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t* load_unload)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
	{
        *load_unload = (uint8_t)(self->rx_frame.elements.args[0]);
		memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
		memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
		self->it_state = SERIAL_SERVO_WRITE_DATA_READY;	
        return true;
    }
    return false;
}

static void serial_servo_controller_object_init(SerialServoControllerTypeDef *self)
{
    self->rx_args_index = 0;
	self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
    memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));

    self->tx_only = true;
    self->tx_byte_index = 0;
    memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
	
	self->write_pin = NULL;
    self->serial_write_and_read = NULL;
}

void serial_servo_init()
{
	serial_servo_controller_object_init(&serial_servo_controller);
	serial_servo_controller.serial_write_and_read = serial_write_and_read;
	serial_servo_controller.write_pin = write_pin;
	
	/* 写入模式, 只有在带接收过程的指令才会打开接收 */
    serial_servo_controller.write_pin(0);
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TXE);
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
}

void serial_servo_irq_handler(void)
{
	extern SerialServoControllerTypeDef serial_servo_controller;
	
	/* 处理发送数据寄存器空中断 (TXE) */
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) != RESET) {
		__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TXE);
		
		/* 如果还有数据需要发送 */
		if (serial_servo_controller.tx_byte_index < serial_servo_controller.tx_frame.elements.length + 3) {  
		    serial_servo_controller.it_state = SERIAL_SERVO_WRITE_DATA;
			/* 发送下一个字节 */
			huart6.Instance->DR = ((uint8_t*)(&serial_servo_controller.tx_frame))[serial_servo_controller.tx_byte_index++];
		} else {
			/* 数据发送完毕，关闭发送中断 */
			__HAL_UART_DISABLE_IT(&huart6, UART_IT_TXE);
		}
	}
	
	/* 处理发送完成中断 (TC) */
	if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != RESET) {
		__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
		
		if (serial_servo_controller.tx_only) {
			/* 只发送模式，设置状态为发送完成 */
			serial_servo_controller.it_state = SERIAL_SERVO_WRITE_DATA_READY;
		} else {
			/* 收发模式，准备接收数据 */
			serial_servo_controller.write_pin(1);
			serial_servo_controller.it_state = SERIAL_SERVO_READ_DATA;
			/* 开启接收中断 */
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
		}
		/* 关闭传输完成中断 */
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_TC);
	}

	/* 处理接收数据寄存器非空中断 (RXNE) */
	if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
		__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
		
		/* 处理接收到的数据 */
		if (0 == serial_servo_rx_handler(&serial_servo_controller, (uint8_t)(huart6.Instance->DR & (uint8_t)0x00FF))) {
			/* 接收完成 */
			serial_servo_controller.it_state = SERIAL_SERVO_READ_DATA_FINISH;
		}
	}
}
#endif
