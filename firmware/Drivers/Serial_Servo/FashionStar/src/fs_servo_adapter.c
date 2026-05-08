#include "arm_robot_config.h"

#if defined(ARM_ROBOT_SERVO_USE_FASHION_START) && (ARM_ROBOT_SERVO_USE_FASHION_START)
#include "arm_robot.h"
#include "fs_uart_servo.h"
#include "arm_shell.h"
#include <string.h>

#define SERVO_ACC_TIME_DEFAULT (100)
#define SERVO_DEC_TIME_DEFAULT (100)
#define SERVO_ANGLE_ERROR_THRESHOLD (10.0f)
float g_servo_acc_ratio = 0.1;	// 加速时间占比
float g_servo_dec_ratio = 0.2;	// 减速时间占比

static Usart_DataTypeDef *servo_usart = &FSUS_Usart;
static FSUS_sync_servo g_sync_servo[ARM_SERVO_NUM];
static int servo_stop(int id, int mode, int power);
static int servo_monitor(int id, ServoData *status);

static int servo_reset_angle(int servo_id)
{
	return (int)FSUS_ServoAngleReset(servo_usart, (uint8_t)servo_id);
}

static void servo_init(UART_HandleTypeDef *uart)
{
	// 关闭串口输入输出
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_SET);
	User_Uart_Init(uart);

	for (int i = 1; i <= ARM_SERVO_NUM; i++) {
		servo_reset_angle(i);
	}
}

static int servo_set_angle_interval(int servo_id, float angle, int interval_ms)
{
	FSUS_STATUS ret = FSUS_SetServoAngle(servo_usart, servo_id, angle, interval_ms, 0);
	return (int) ret;
}

static int servo_set_angle_interval_acc(int servo_id, float angle, int interval_ms, int acc_ms, int dec_ms)
{
	if (acc_ms < 20) acc_ms = 20;
	if (dec_ms < 20) dec_ms = 20;
	if (interval_ms < acc_ms + dec_ms) {
		interval_ms = acc_ms + dec_ms + 10;
	}
	FSUS_STATUS ret = FSUS_SetServoAngleByInterval(servo_usart, servo_id, angle, interval_ms, 
				(uint16_t)(acc_ms), 
				(uint16_t)(dec_ms), 0);
	return (int) ret;
}

static int servo_set_angle_interval_velocity(int servo_id, float angle, float velocity, int acc_ms, int dec_ms)
{
	if (acc_ms < 20) acc_ms = 20;
	if (dec_ms < 20) dec_ms = 20;
	FSUS_STATUS ret = FSUS_SetServoAngleByVelocity(servo_usart, servo_id, \
				angle, velocity, (uint16_t)acc_ms, \
				(uint16_t)dec_ms, 0);
	return (int) ret;
}
static int servo_get_angle(int id, float *angle)
{	
	FSUS_STATUS ret = FSUS_QueryServoAngle(servo_usart, id, angle);

	if (ret != FSUS_STATUS_SUCCESS) {
		safe_printf("ERROR, query servo id:%d angle failed, ret=%d.\n", id, ret);
		return ret;
	}
	return 0;
}

static int servo_stop(int id, int mode, int power)
{
	FSUS_STATUS ret = FSUS_StopOnControlMode(servo_usart, id, (uint8_t)mode, (uint16_t)power);
	return (int) ret;
}

static int servo_sync_move(ServoSyncData *data, int data_num)
{
	for (int i = 0; i < data_num; i++) {
		g_sync_servo[i].id = data[i].servo_id;
		g_sync_servo[i].angle = data[i].angle;
	}

	FSUS_STATUS ret = FSUS_SyncCommand(servo_usart, data_num, MODE_SET_SERVO_ANGLE, g_sync_servo);
	return (int) ret;
}

static int servo_get_status(int id)
{
	ServoData status = {0};
	int ret = servo_monitor(id, &status);
	if (ret != 0) {
		safe_printf("ERROR, query servo id:%d status failed, ret=%d.\n", id, ret);
		return ret;
	}
	safe_printf("servo id:%d status: angle=%f, circle_count=%d, current=%d, power=%d, status=%d, temperature=%d, voltage=%d.\n", id, status.angle, status.circle_count, status.current, status.power, status.status, status.temperature, status.voltage);
	return 0;
}

static int servo_set_zero(int servo_id)
{
	float angle = 0;
	FSUS_STATUS set_ret = FSUS_SetOriginPoint(servo_usart, (uint8_t)servo_id);
	if (set_ret != FSUS_STATUS_SUCCESS) {
		safe_printf("[ERROR] Failed to set origin for servo %d, ret=%d\n", servo_id, set_ret);
		return (int)set_ret;
	}

	int ret = servo_get_angle(servo_id, &angle);
	if (ret != 0) {
		safe_printf("[ERROR] Failed to get angle for servo %d, ret=%d\n", servo_id, ret);
		return ret;
	}

	safe_printf("[INFO] Set zero for servo %d, angle=%f\n", servo_id, angle);

	if (angle > SERVO_ANGLE_ERROR_THRESHOLD) {
		safe_printf("[ERROR] Failed to set zero for servo %d, angle is %f\n", servo_id, angle);
		return -1;
	}

	return 0;
}

static int servo_monitor(int id, ServoData *status)
{
	FSUS_STATUS ret;

	if (status == NULL) {
		return -1;
	}

	memset(status, 0, sizeof(*status));
	ret = FSUS_ServoMonitor(servo_usart, (uint8_t)id, status);
	return (ret == FSUS_STATUS_SUCCESS) ? 0 : (int)ret;
}

static int servo_monitor_batch(const uint8_t *servo_ids, int servo_count, ServoData *servodata)
{
	FSUS_STATUS ret;

	if ((servo_ids == NULL) || (servodata == NULL) || (servo_count <= 0) ||
	    (servo_count > ARM_JOINTS_NUM)) {
		return -1;
	}

	ret = FSUS_ServoMonitorSyncGroup(servo_usart, servo_ids, (uint8_t)servo_count, servodata);
	return (ret == FSUS_STATUS_SUCCESS) ? 0 : (int)ret;
}

ArmServoOpt g_fashion_start_servo_ops = {
	.init = servo_init,
	.set_angle_interval = servo_set_angle_interval,
	.set_angle_interval_acc = servo_set_angle_interval_acc,
	.set_angle_interval_velocity = servo_set_angle_interval_velocity,
	.get_angle = servo_get_angle,
	.stop = servo_stop,
	.irq_handler = User_RxCpltCallback,
	.sync_move = servo_sync_move,
	.reset_angle = servo_reset_angle,
	.get_status = servo_get_status,
	.set_zero = servo_set_zero,
	.monitor = servo_monitor,
	.monitor_batch = servo_monitor_batch,
};
#endif
