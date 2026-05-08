/*
 * Fashion Star 总线伺服舵机驱动库
 * Version: v0.0.2
 * UpdateTime: 2024/07/17
 */
#include "fs_uart_servo.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

//同步命令舵机结构体数组
FSUS_sync_servo SyncArray[20]; // 假设您要控制20个伺服同步

#define FSUS_STATUS_NEED_MORE_BYTES 0xFEU
#define FSUS_BEGIN_TRANSACTION_OR_RETURN(usart_) \
    do { \
        if (Usart_BeginTransaction((usart_), portMAX_DELAY) != pdTRUE) { \
            return FSUS_STATUS_TIMEOUT; \
        } \
    } while (0)

static TickType_t fsus_timeout_ticks(void)
{
    TickType_t timeout_ticks = pdMS_TO_TICKS(FSUS_TIMEOUT_MS);

    if (timeout_ticks == 0U) {
        timeout_ticks = 1U;
    }

    return timeout_ticks;
}

static void fsus_parse_servo_data(const PackageTypeDef *pkg, ServoData *servo_data)
{
    double temp;

    servo_data->id = pkg->content[0];
    servo_data->voltage = (int16_t)((pkg->content[2] << 8) | pkg->content[1]);
    servo_data->current = (int16_t)((pkg->content[4] << 8) | pkg->content[3]);
    servo_data->power = (int16_t)((pkg->content[6] << 8) | pkg->content[5]);
    servo_data->temperature = (int16_t)((pkg->content[8] << 8) | pkg->content[7]);
    temp = (double)servo_data->temperature;
    servo_data->temperature =
        1 / (log(temp / (4096.0f - temp)) / 3435.0f + 1 / (273.15 + 25)) - 273.15;
    servo_data->status = pkg->content[9];
    servo_data->angle = (int32_t)(
        (pkg->content[13] << 24) |
        (pkg->content[12] << 16) |
        (pkg->content[11] << 8) |
        pkg->content[10]
    );
    servo_data->angle = (float)(servo_data->angle / 10.0f);
    servo_data->circle_count = (int16_t)((pkg->content[15] << 8) | pkg->content[14]);
}

static FSUS_STATUS fsus_try_parse_package_from_ring(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg)
{
    uint16_t available_bytes;
    uint16_t total_len;
    uint16_t size;
    FSUS_STATUS status = FSUS_STATUS_NEED_MORE_BYTES;

    taskENTER_CRITICAL();

    while (RingBuffer_GetByteUsed(ringBuf) >= 2U) {
        if (RingBuffer_PeekUShort(ringBuf, 0U) == FSUS_PACK_RESPONSE_HEADER) {
            break;
        }
        (void)RingBuffer_ReadByte(ringBuf);
    }

    available_bytes = RingBuffer_GetByteUsed(ringBuf);
    if (available_bytes < 4U) {
        goto exit;
    }

    pkg->header = FSUS_PACK_RESPONSE_HEADER;
    pkg->cmdId = RingBuffer_PeekByte(ringBuf, 2U);

    if (RingBuffer_PeekByte(ringBuf, 3U) == 0xFFU) {
        if (available_bytes < 6U) {
            goto exit;
        }
        pkg->isSync = 1U;
        size = RingBuffer_PeekUShort(ringBuf, 4U);
        total_len = (uint16_t)(7U + size);
    } else {
        pkg->isSync = 0U;
        size = RingBuffer_PeekByte(ringBuf, 3U);
        total_len = (uint16_t)(5U + size);
    }

    if (size > FSUS_PACK_RESPONSE_MAX_SIZE) {
        (void)RingBuffer_ReadByte(ringBuf);
        status = FSUS_STATUS_SIZE_TOO_BIG;
        goto exit;
    }

    if (available_bytes < total_len) {
        goto exit;
    }

    pkg->header = RingBuffer_ReadUShort(ringBuf);
    pkg->cmdId = RingBuffer_ReadByte(ringBuf);
    if (pkg->isSync != 0U) {
        (void)RingBuffer_ReadByte(ringBuf);
        pkg->size = RingBuffer_ReadUShort(ringBuf);
    } else {
        pkg->size = RingBuffer_ReadByte(ringBuf);
    }
    RingBuffer_ReadByteArray(ringBuf, pkg->content, pkg->size);
    pkg->checksum = RingBuffer_ReadByte(ringBuf);

    status = FSUS_STATUS_SUCCESS;

exit:
    taskEXIT_CRITICAL();

    if (status != FSUS_STATUS_SUCCESS) {
        return status;
    }

    return FSUS_IsValidResponsePackage(pkg);
}

static FSUS_STATUS fsus_wait_for_package(Usart_DataTypeDef *usart, PackageTypeDef *pkg)
{
    TickType_t start_tick;
    TickType_t timeout_ticks;
    uint32_t events = 0U;

    if ((usart == NULL) || (pkg == NULL)) {
        return FSUS_STATUS_FAIL;
    }

    start_tick = xTaskGetTickCount();
    timeout_ticks = fsus_timeout_ticks();

    while (1) {
        TickType_t elapsed_ticks;
        FSUS_STATUS status = fsus_try_parse_package_from_ring(usart->recvBuf, pkg);

        if (status != FSUS_STATUS_NEED_MORE_BYTES) {
            return status;
        }

        elapsed_ticks = xTaskGetTickCount() - start_tick;
        if (elapsed_ticks >= timeout_ticks) {
            usart->transaction_timeout_count++;
            Usart_NoteRequestTimeout(usart);
            return FSUS_STATUS_TIMEOUT;
        }

        if (Usart_WaitForNotification(timeout_ticks - elapsed_ticks, &events) != pdTRUE) {
            usart->transaction_timeout_count++;
            Usart_NoteRequestTimeout(usart);
            return FSUS_STATUS_TIMEOUT;
        }

        if ((events & FSUS_USART_NOTIFY_TX_ERROR) != 0U) {
            return FSUS_STATUS_TX_ERROR_EVENT;
        }
    }
}

static UsartRequestReplyFailReason fsus_fail_reason_from_status(FSUS_STATUS status)
{
    switch (status) {
        case FSUS_STATUS_SUCCESS:
            return USART_REQ_REPLY_FAIL_NONE;
        case FSUS_STATUS_TIMEOUT:
            return USART_REQ_REPLY_FAIL_TIMEOUT;
        case FSUS_STATUS_SEND_FAIL:
            return USART_REQ_REPLY_FAIL_SEND_FAIL;
        case FSUS_STATUS_TX_ERROR_EVENT:
            return USART_REQ_REPLY_FAIL_TX_ERROR_EVENT;
        case FSUS_STATUS_WRONG_RESPONSE_HEADER:
        case FSUS_STATUS_UNKOWN_CMD_ID:
        case FSUS_STATUS_SIZE_TOO_BIG:
        case FSUS_STATUS_CHECKSUM_ERROR:
            return USART_REQ_REPLY_FAIL_BAD_PACKET;
        case FSUS_STATUS_REPLY_INCOMPLETE:
            return USART_REQ_REPLY_FAIL_REPLY_INCOMPLETE;
        case FSUS_STATUS_ID_NOT_MATCH:
        case FSUS_STATUS_REPLY_CMD_MISMATCH:
        case FSUS_STATUS_REPLY_SIZE_MISMATCH:
        case FSUS_STATUS_FAIL:
        default:
            return USART_REQ_REPLY_FAIL_REPLY_MISMATCH;
    }
}

static void fsus_finish_request(Usart_DataTypeDef *usart, FSUS_STATUS status)
{
    if ((usart == NULL) || (usart->request_reply_pending == 0U)) {
        return;
    }

    Usart_NoteRequestReplyResult(
        usart,
        (status == FSUS_STATUS_SUCCESS) ? pdTRUE : pdFALSE,
        fsus_fail_reason_from_status(status)
    );
}

static FSUS_STATUS fsus_expect_reply(
    const PackageTypeDef *pkg,
    uint8_t expected_cmd,
    uint16_t expected_size
)
{
    if (pkg == NULL) {
        return FSUS_STATUS_FAIL;
    }

    if (pkg->cmdId != expected_cmd) {
        return FSUS_STATUS_REPLY_CMD_MISMATCH;
    }

    if ((expected_size > 0U) && (pkg->size != expected_size)) {
        return FSUS_STATUS_REPLY_SIZE_MISMATCH;
    }

    return FSUS_STATUS_SUCCESS;
}

static int fsus_find_servo_index(
    const uint8_t servo_ids[],
    uint8_t servo_count,
    uint8_t servo_id
)
{
    for (uint8_t i = 0U; i < servo_count; ++i) {
        if (servo_ids[i] == servo_id) {
            return (int)i;
        }
    }

    return -1;
}


// 统一数据包处理
void FSUS_Package2RingBuffer(PackageTypeDef *pkg, RingBufferTypeDef *ringBuf) {
    uint8_t checksum;
    RingBuffer_WriteUShort(ringBuf, pkg->header);// 写入帧头
    RingBuffer_WriteByte(ringBuf, pkg->cmdId);// 写入指令ID
    
    if (pkg->isSync || pkg->size > 255) {
        RingBuffer_WriteByte(ringBuf, 0xFF);
        RingBuffer_WriteUShort(ringBuf, pkg->size);// 写入包的长度
    } else {
        RingBuffer_WriteByte(ringBuf, (uint8_t)pkg->size);// 写入包的长度
    }
    
    RingBuffer_WriteByteArray(ringBuf, pkg->content, pkg->size);// 写入内容主题
    checksum = RingBuffer_GetChecksum(ringBuf);// 计算校验和
    RingBuffer_WriteByte(ringBuf, checksum);// 写入校验和
}

// 计算Package的校验和
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg) {
    RingBufferTypeDef ringBuf;// 初始化环形队列
    uint8_t pkgBuf[FSUS_PACK_RESPONSE_MAX_SIZE + 5];
    RingBuffer_Init(&ringBuf, sizeof(pkgBuf), pkgBuf);
	// 将Package转换为ringbuffer
	// 在转换的时候,会自动的计算checksum
    FSUS_Package2RingBuffer(pkg, &ringBuf);
	// 获取环形队列队尾的元素(即校验和的位置)
    return RingBuffer_GetValueByIndex(&ringBuf, RingBuffer_GetByteUsed(&ringBuf)-1);
}

// 判断是否为有效的请求头的（数据包验证）
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg) {
    if (pkg->header != FSUS_PACK_RESPONSE_HEADER)
        return FSUS_STATUS_WRONG_RESPONSE_HEADER;// 帧头不对
    if (pkg->cmdId > FSUS_CMD_NUM)// 判断控制指令是否有效 指令范围超出
        return FSUS_STATUS_UNKOWN_CMD_ID;
    if (pkg->size > (FSUS_PACK_RESPONSE_MAX_SIZE - 6))  // 调整校验长度
        return FSUS_STATUS_SIZE_TOO_BIG;
    if (FSUS_CalcChecksum(pkg) != pkg->checksum)// 校验和不匹配
        return FSUS_STATUS_CHECKSUM_ERROR;
    return FSUS_STATUS_SUCCESS;// 数据有效
}

// 字节数组转换为数据帧
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg){
    // 申请内存
    pkg = (PackageTypeDef *)malloc(sizeof(PackageTypeDef));
    // 读取帧头
    pkg->header = RingBuffer_ReadUShort(ringBuf);
    // 读取指令ID
    pkg->cmdId = RingBuffer_ReadByte(ringBuf);
    // 读取包的长度
    pkg->size = RingBuffer_ReadByte(ringBuf);
    // 申请参数的内存空间
    // pkg->content = (uint8_t *)malloc(pkg->size);
    // 写入content
    RingBuffer_ReadByteArray(ringBuf, pkg->content, pkg->size);
    // 写入校验和
    pkg->checksum = RingBuffer_ReadByte(ringBuf);
    // 返回当前的数据帧是否为有效反馈数据帧
    return FSUS_IsValidResponsePackage(pkg);
}

// 发送数据包
FSUS_STATUS FSUS_SendPackage_Common(Usart_DataTypeDef *usart, uint8_t cmdId, uint16_t size, uint8_t *content, uint8_t isSync) {
    PackageTypeDef pkg = {
        .header = FSUS_PACK_REQUEST_HEADER,
        .cmdId = cmdId,
        .size = size,
        .isSync = isSync
    };
    uint8_t servo_id = 0U;

    memcpy(pkg.content, content, size);
    if ((content != NULL) && (size > 0U) &&
        ((cmdId == FSUS_CMD_READ_ANGLE) || (cmdId == FSUS_CMD_SET_SERVO_ReadData))) {
        servo_id = content[0];
    }
    FSUS_Package2RingBuffer(&pkg, usart->sendBuf);// 将数据放入发送缓冲区
    Usart_NoteRequestStart(usart, cmdId, servo_id);
    if (Usart_SendAll(usart) != HAL_OK) {
        return FSUS_STATUS_SEND_FAIL;
    }
    return FSUS_STATUS_SUCCESS;
}

// 接收数据包（统一处理）
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg) {
    return fsus_wait_for_package(usart, pkg);
}

FSUS_STATUS FSUS_sync_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg)
{
    FSUS_STATUS status = FSUS_RecvPackage(usart, pkg);

    if (status != FSUS_STATUS_SUCCESS) {
        return status;
    }

    return fsus_expect_reply(pkg, FSUS_CMD_SET_SERVO_ReadData, 16U);
}

// 舵机通讯检测
// 注: 如果没有舵机响应这个Ping指令的话, 就会超时
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码
	uint8_t ehcoServoId; // PING得到的舵机ID

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_PING, 1, &servo_id, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if(statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_PING, 1U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id){
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
		}
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}

// 重置舵机的用户资料
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id){
	const uint8_t size = 1;
	FSUS_STATUS statusCode;

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_RESET_USER_DATA, size, &servo_id, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_RESET_USER_DATA, 0U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		if ((pkg.size < 2U) || (pkg.content[0] != servo_id)) {
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		uint8_t result = (uint8_t)pkg.content[1];
		if (result == 1){
			statusCode = FSUS_STATUS_SUCCESS;
		}else{
			statusCode = FSUS_STATUS_FAIL;
		}
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}

// 读取数据
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id,  uint8_t address, uint8_t *value, uint8_t *size){
	FSUS_STATUS statusCode;
	uint8_t buffer[2] = {servo_id, address};

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_READ_DATA, 2, buffer, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_READ_DATA, 0U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		if ((pkg.size < 2U) || (pkg.content[0] != servo_id) || (pkg.content[1] != address)) {
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		*size = pkg.size - 2; // content的长度减去servo_id跟address的长度
		for (int i=0; i<*size; i++){
			value[i] = pkg.content[i+2];
		}
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}

// 写入数据
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size){
	FSUS_STATUS statusCode;
	uint8_t buffer[size+2]; // 舵机ID + 地址位Address + 数据byte数
	buffer[0] = servo_id;
	buffer[1] = address;
	for (int i=0; i<size; i++){
		buffer[i+2] = value[i];
	}

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_WRITE_DATA, size + 2, buffer, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_WRITE_DATA, 0U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		if ((pkg.size < 3U) || (pkg.content[0] != servo_id) || (pkg.content[1] != address)) {
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		uint8_t result = pkg.content[2];
		if(result == 1){
			statusCode = FSUS_STATUS_SUCCESS;
		}else{
			statusCode = FSUS_STATUS_FAIL;
		}
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}


// 设置舵机的角度
// @angle 单位度
// @interval 单位ms
// @power 舵机执行功率 单位mW
//        若power=0或者大于保护值
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power){
	FSUS_STATUS statusCode;
	const uint8_t size = 7;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_ROTATE, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 设置舵机的角度(指定周期) */
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, 
				float angle, uint16_t interval, uint16_t t_acc, 
				uint16_t t_dec, uint16_t  power){
	FSUS_STATUS statusCode;
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if (t_acc < 20){
		t_acc = 20;
	}
	if (t_dec < 20){
		t_dec = 20;
	}

	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 设置舵机的角度(指定转速) */
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float angle, float velocity, uint16_t t_acc, \
				uint16_t t_dec, uint16_t  power){
	FSUS_STATUS statusCode;
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}

	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 查询单个舵机的角度信息 angle 单位度 */
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	const uint8_t size = 1; // 请求包content的长度
	FSUS_STATUS statusCode;
	uint8_t ehcoServoId;
	int16_t echoAngle;

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_READ_ANGLE, size, &servo_id, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_READ_ANGLE, 3U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id){
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		echoAngle = (int16_t)(pkg.content[1] | (pkg.content[2] << 8));
		*angle = (float)(echoAngle / 10.0);
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
  return statusCode;
}	

/* 设置舵机的角度(多圈模式) */
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, 
	uint32_t interval, uint16_t power){
	FSUS_STATUS statusCode;
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		angle = 4096000;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 设置舵机的角度(多圈模式, 指定周期) */
FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			uint32_t interval,  uint16_t t_acc,  uint16_t t_dec, uint16_t power){
	FSUS_STATUS statusCode;
	const uint8_t size = 15;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);

	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		interval = 4096000;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 设置舵机的角度(多圈模式, 指定转速) */
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			float velocity, uint16_t t_acc,  uint16_t t_dec, uint16_t power){
	FSUS_STATUS statusCode;
	const uint8_t size = 13;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 查询舵机的角度(多圈模式) */
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	const uint8_t size = 1; // 请求包content的长度
	FSUS_STATUS statusCode;
	uint8_t ehcoServoId;
	int32_t echoAngle;

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	PackageTypeDef pkg;
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, size, &servo_id, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, 5U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id){
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) |  (pkg.content[3] << 16) | (pkg.content[4] << 24));
		*angle = (float)(echoAngle / 10.0);
	}

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
  return statusCode;
}


/* 舵机阻尼模式 */
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power){
	FSUS_STATUS statusCode;
	const uint8_t size = 3; // 请求包content的长度
	uint8_t buffer[size+1]; // content缓冲区
	RingBufferTypeDef ringBuf; // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteUShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_DAMPING, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}


// 舵机重置多圈角度圈数
FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id){
	FSUS_STATUS statusCode;

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_RESERT_SERVO_ANGLE_MTURN, 1, &servo_id, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/*零点设置 仅适用于无刷磁编码舵机*/
FSUS_STATUS FSUS_SetOriginPoint(Usart_DataTypeDef *usart, uint8_t servo_id)
{
	FSUS_STATUS statusCode;

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_ORIGIN_POINT, 2, &servo_id, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 舵机开始异步命令*/
FSUS_STATUS FSUS_BeginAsync(Usart_DataTypeDef *usart)
{
	FSUS_STATUS statusCode;
	const uint8_t size = 0; // 请求包content的长度
	uint8_t buffer[size+1]; // content缓冲区
	RingBufferTypeDef ringBuf; // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	RingBuffer_WriteByte(&ringBuf, 0);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_BEGIN_ASYNC, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 舵机结束异步命令*/
FSUS_STATUS FSUS_EndAsync(Usart_DataTypeDef *usart,uint8_t mode)
{
/*mode
	0:执行存储的命令
	1:取消存储的命令
*/
	FSUS_STATUS statusCode;
	const uint8_t size = 1; // 请求包content的长度
	uint8_t buffer[size+1]; // content缓冲区
	RingBufferTypeDef ringBuf; // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	RingBuffer_WriteByte(&ringBuf, mode);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_END_ASYNC, size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 舵机单个数据监控*/
FSUS_STATUS FSUS_ServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_id, ServoData servodata[]) {
	FSUS_STATUS statusCode;
	const uint8_t size = 1U;
	PackageTypeDef pkg;

	if ((usart == NULL) || (servodata == NULL)) {
		return FSUS_STATUS_FAIL;
	}

	memset(servodata, 0, sizeof(*servodata));

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ReadData, size, &servo_id, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	statusCode = fsus_expect_reply(&pkg, FSUS_CMD_SET_SERVO_ReadData, 16U);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	if (pkg.content[0] != servo_id) {
		statusCode = FSUS_STATUS_ID_NOT_MATCH;
		goto cleanup;
	}

	fsus_parse_servo_data(&pkg, servodata);

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}

FSUS_STATUS FSUS_ServoMonitorSyncGroup(
    Usart_DataTypeDef *usart,
    const uint8_t servo_ids[],
    uint8_t servo_count,
    ServoData servodata[]
) {
	FSUS_STATUS statusCode = FSUS_STATUS_SUCCESS;
	uint8_t buffer[3U + 20U];
	ServoData data_buffer[20] = {0};
	uint8_t received_flags[20] = {0};
	uint8_t received_count = 0U;
	uint16_t size = 0U;
	RingBufferTypeDef ringBuf;
	PackageTypeDef pkg;

	if ((usart == NULL) || (servo_ids == NULL) || (servodata == NULL) ||
	    (servo_count == 0U) || (servo_count > 20U)) {
		return FSUS_STATUS_FAIL;
	}

	size = (uint16_t)(3U + servo_count);
	RingBuffer_Init(&ringBuf, size, buffer);
	RingBuffer_WriteByte(&ringBuf, FSUS_CMD_SET_SERVO_ReadData);
	RingBuffer_WriteByte(&ringBuf, 1U);
	RingBuffer_WriteByte(&ringBuf, servo_count);
	for (uint8_t i = 0U; i < servo_count; ++i) {
		RingBuffer_WriteByte(&ringBuf, servo_ids[i]);
	}

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_SyncCommand, size, buffer + 1, 0);
	if (statusCode != FSUS_STATUS_SUCCESS) {
		goto cleanup;
	}

	for (uint8_t i = 0U; i < servo_count; ++i) {
		int matched_index = -1;

		statusCode = fsus_wait_for_package(usart, &pkg);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		statusCode = fsus_expect_reply(&pkg, FSUS_CMD_SET_SERVO_ReadData, 16U);
		if (statusCode != FSUS_STATUS_SUCCESS) {
			goto cleanup;
		}

		matched_index = fsus_find_servo_index(servo_ids, servo_count, pkg.content[0]);
		if ((matched_index < 0) || (received_flags[matched_index] != 0U)) {
			statusCode = FSUS_STATUS_ID_NOT_MATCH;
			goto cleanup;
		}

		fsus_parse_servo_data(&pkg, &data_buffer[matched_index]);
		received_flags[matched_index] = 1U;
		received_count++;
	}

	if (received_count != servo_count) {
		statusCode = FSUS_STATUS_REPLY_INCOMPLETE;
		goto cleanup;
	}

	memcpy(servodata, data_buffer, sizeof(ServoData) * servo_count);

cleanup:
	fsus_finish_request(usart, statusCode);
	Usart_EndTransaction(usart);
	return statusCode;
}


/* 舵机控制模式停止指令*/
//mode 指令停止形式
//0-停止后卸力(失锁)
//1-停止后保持锁力
//2-停止后进入阻尼状态
FSUS_STATUS FSUS_StopOnControlMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t mode, uint16_t power) {
	FSUS_STATUS statusCode;
	const uint8_t size = 4;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	RingBuffer_WriteByte(&ringBuf, servo_id);
	mode = mode | 0x10;
	RingBuffer_WriteByte(&ringBuf, mode);
	RingBuffer_WriteShort(&ringBuf, power);

	FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);
	statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_CONTROL_MODE_STOP, (uint8_t)size, buffer + 1, 0);
	Usart_EndTransaction(usart);
	return statusCode;
}

/* 同步命令选择模式控制函数*/
// @servo_count 舵机同步数量
// @servomode 同步命令模式选择
// @FSUS_servo_sync servoSync[] 舵机参数结构体数组
FSUS_STATUS FSUS_SyncCommand(Usart_DataTypeDef *usart, uint8_t servo_count, uint8_t ServoMode, FSUS_sync_servo servoSync[]) {
		FSUS_STATUS statusCode = FSUS_STATUS_SUCCESS;
		uint8_t buffer[3 + servo_count * 15];
		uint16_t size = 0U;
		RingBufferTypeDef ringBuf;

    if ((usart == NULL) || (servoSync == NULL) || (servo_count == 0U)) {
        return FSUS_STATUS_FAIL;
    }

    switch (ServoMode) {
        case MODE_SET_SERVO_ANGLE:
						size = 3 + servo_count * 7;
            RingBuffer_Init(&ringBuf, size, buffer);
						RingBuffer_WriteByte(&ringBuf, 8);
            RingBuffer_WriteByte(&ringBuf, 7);
            RingBuffer_WriteByte(&ringBuf, servo_count);
            for (int i = 0; i < servo_count; i++) {
                RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
                if (servoSync[i].angle > 180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * 180.0f));
                } else if (servoSync[i].angle < -180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * -180.0f));
                } else {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
                }
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].interval_single);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
            }
            break;
				case MODE_SET_SERVO_ANGLE_BY_INTERVAL:
            /* 同步命令设置舵机的角度(单圈模式，指定周期) */
						size = 3 + servo_count * 11;
            RingBuffer_Init(&ringBuf, size, buffer);
            RingBuffer_WriteByte(&ringBuf, 11);
            RingBuffer_WriteByte(&ringBuf, 11);
            RingBuffer_WriteByte(&ringBuf, servo_count);
            for (int i = 0; i < servo_count; i++) {
                RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
                if (servoSync[i].angle > 180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * 180.0f));
                } else if (servoSync[i].angle < -180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * -180.0f));
                } else {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
                }
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].interval_single);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
            }
            break;
				case MODE_SET_SERVO_ANGLE_BY_VELOCITY:
            /* 同步命令设置舵机的角度(单圈模式，指定转速) */
						size = 3 + servo_count * 11;
            RingBuffer_Init(&ringBuf, size, buffer);
            RingBuffer_WriteByte(&ringBuf,12);
						RingBuffer_WriteByte(&ringBuf,11);
						RingBuffer_WriteByte(&ringBuf,servo_count);
	
						for(int i=0; i<servo_count; i++){
						RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
						if (servoSync[i].angle > 180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * 180.0f));
                } else if (servoSync[i].angle < -180.0f) {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * -180.0f));
                } else {
                    RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
                }
						RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*servoSync[i].velocity));
						RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
            RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
            RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
            }
            break;
        case MODE_SET_SERVO_ANGLE_MTURN:
            /* 同步命令设置舵机的角度(多圈模式) */
						size = 3 + servo_count * 11;
            RingBuffer_Init(&ringBuf, size, buffer);
            RingBuffer_WriteByte(&ringBuf, 13);
            RingBuffer_WriteByte(&ringBuf, 11);
            RingBuffer_WriteByte(&ringBuf, servo_count);
            for (int i = 0; i < servo_count; i++) {
                RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
                if (servoSync[i].angle > 368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * 368640.0f));
                } else if (servoSync[i].angle < -368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * -368640.0f));
                } else {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
                }
                RingBuffer_WriteULong(&ringBuf, servoSync[i].interval_multi);
                RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
            }
            break;
        case MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL:
            /* 同步命令设置舵机的角度(多圈模式, 指定周期) */
						size = 3 + servo_count * 15;
            RingBuffer_Init(&ringBuf, size, buffer);
            RingBuffer_WriteByte(&ringBuf, 14);
            RingBuffer_WriteByte(&ringBuf, 15);
            RingBuffer_WriteByte(&ringBuf, servo_count);
            for (int i = 0; i < servo_count; i++) {
                RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
                if (servoSync[i].angle > 368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * 368640.0f));
                } else if (servoSync[i].angle < -368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * -368640.0f));
                } else {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
                }
                RingBuffer_WriteULong(&ringBuf, servoSync[i].interval_multi);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
                RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
            }
            break;
        case MODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY:
						size = 3 + servo_count * 13;
            RingBuffer_Init(&ringBuf, size, buffer);
            RingBuffer_WriteByte(&ringBuf, 15);
            RingBuffer_WriteByte(&ringBuf, 13);
            RingBuffer_WriteByte(&ringBuf, servo_count);
            for (int i = 0; i < servo_count; i++) {
                RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
                if (servoSync[i].angle > 368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * 368640.0f));
                } else if (servoSync[i].angle < -368640.0f) {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * -368640.0f));
                } else {
                    RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
                }
                RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f * servoSync[i].velocity));
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
                RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
                RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
            }
            break;
						
        default:
            return FSUS_STATUS_ERRO; // 无效的模式
    }

    FSUS_BEGIN_TRANSACTION_OR_RETURN(usart);

		if(size<=255U) {
			statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_SyncCommand, (uint8_t)size, buffer + 1, 0);
		} else {
			statusCode = FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_SyncCommand, size, buffer + 1, 1);
		}

		Usart_EndTransaction(usart);
    return statusCode;
}	
