/*
 * 环形队列
 * ---------------------------------------------------
 * ---------------------------------------------------
 */
 
#include "ring_buffer.h"
#include <stdint.h>

// 队列重置
void RingBuffer_Reset(RingBufferTypeDef *ringBuf){
	// 队首指针跟队尾指针都指向buffer的首个地址
	ringBuf->head = 0;
	ringBuf->tail = 0;
}

// 创建队列
void RingBuffer_Init(RingBufferTypeDef* ringBuf, uint16_t capacity, uint8_t *buffer){
    // RingBufferTypeDef *ringBuf = (RingBufferTypeDef *)malloc(sizeof(RingBufferTypeDef));
	// Buffer内存申请成功
	ringBuf->buffer = buffer;// (uint8_t *)malloc(ringBuf->size);
	ringBuf->head = 0;
	ringBuf->tail = 0;
	// 尺寸size需要比容量capacity大一
	ringBuf->size = capacity + 1;
	// 数值初始化置为0
	for (uint16_t i=0; i<ringBuf->size; i++){
		ringBuf->buffer[i] = 0;
	}
	// printf("[RingBuffer_Init] ring buffer init pbuffer=%p\n", ringBuf->buffer);
}

// 销毁队列
//void RingBuffer_Destory(RingBufferTypeDef *ringBuf){
//	free(ringBuf->buffer);
//	free(ringBuf);
//}

// 获得环形队列的容量
uint16_t RingBuffer_GetCapacity(RingBufferTypeDef *ringBuf){
	return ringBuf->size - 1;
}

// 获取环形队列已经存好的字节数
uint16_t RingBuffer_GetByteUsed(RingBufferTypeDef *ringBuf) {
    if (ringBuf->head <= ringBuf->tail) {
        return ringBuf->tail - ringBuf->head;
    } else {
        return ringBuf->size - (ringBuf->head - ringBuf->tail);
    }
}

// 获取队列的剩余容量
uint16_t RingBuffer_GetByteFree(RingBufferTypeDef *ringBuf){
	return RingBuffer_GetCapacity(ringBuf) - RingBuffer_GetByteUsed(ringBuf);
}

// 判断环形队列是否为空
uint8_t RingBuffer_IsEmpty(RingBufferTypeDef *ringBuf){
	return RingBuffer_GetByteFree(ringBuf) == RingBuffer_GetCapacity(ringBuf);
}

// 判断环形队列是否已满
uint8_t RingBuffer_IsFull(RingBufferTypeDef *ringBuf){
	return RingBuffer_GetByteFree(ringBuf) == 0;
}

// 根据索引号读取第i个元素
uint8_t RingBuffer_GetValueByIndex(RingBufferTypeDef *ringBuf, uint16_t index){
	if (index >= RingBuffer_GetByteUsed(ringBuf)){
		// 索引号超出了范围
		return 0;
	}
	uint16_t rbIdx = (ringBuf->head + index + 1) % ringBuf->size;
	return ringBuf->buffer[rbIdx];
}

// 弹出队首元素
uint8_t RingBuffer_Pop(RingBufferTypeDef *ringBuf){
	uint8_t temp;
	if(RingBuffer_IsEmpty(ringBuf)){
		return 0;
	}
	else{
		ringBuf->head =(ringBuf->head + 1)%ringBuf->size;
		temp = ringBuf->buffer[ringBuf->head];
		ringBuf->buffer[ringBuf->head] = 0;
		return temp;
	}
	
}


// 写入队尾元素
// ring_buffer.c
void RingBuffer_Push(RingBufferTypeDef *ringBuf, uint8_t value) {
    if (RingBuffer_IsFull(ringBuf)) {
        // 可选：增加溢出标志或日志输出
        return; // 直接返回避免覆盖
    }
    ringBuf->tail = (ringBuf->tail + 1) % ringBuf->size;
    ringBuf->buffer[ringBuf->tail] = value;
}

// 读取单个字节
uint8_t RingBuffer_ReadByte(RingBufferTypeDef *ringBuf){
	return RingBuffer_Pop(ringBuf);
}

// 读取字节数组
void RingBuffer_ReadByteArray(RingBufferTypeDef *ringBuf, uint8_t* dest, uint16_t size){
	for(uint16_t idx=0; idx<size; idx++){
		dest[idx] = RingBuffer_Pop(ringBuf);
	}
}

// 读取有符号Short整数(两个字节)
int16_t RingBuffer_ReadShort(RingBufferTypeDef *ringBuf){
	int16_t value;
	uint8_t *p = (uint8_t *)&value;
	RingBuffer_ReadByteArray(ringBuf, p, 2);
	return value;
}

// 读取无符号Short整数(两个字节)
uint16_t RingBuffer_ReadUShort(RingBufferTypeDef *ringBuf){
	uint16_t value;
	uint8_t *p = (uint8_t *)&value;
	RingBuffer_ReadByteArray(ringBuf, p, 2);
	return value;
}

// 读取有符号Long类型的整数(四个字节)
int32_t RingBuffer_ReadLong(RingBufferTypeDef *ringBuf){
	int32_t value;
	uint8_t *p = (uint8_t *)&value;
	RingBuffer_ReadByteArray(ringBuf, p, 4);
	return value;
}

// 读取无符号Long类型的整数(四个字节)
uint32_t RingBuffer_ReadULong(RingBufferTypeDef *ringBuf){
	uint32_t value;
	uint8_t *p = (uint8_t *)&value;
	RingBuffer_ReadByteArray(ringBuf, p, 4);
	return value;
}

// 读取浮点数(四个字节)
float RingBuffer_ReadFloat(RingBufferTypeDef *ringBuf){
	float value;
	uint8_t *p = (uint8_t *)&value;
	RingBuffer_ReadByteArray(ringBuf, p, 4);
	return value;
}

// 写入单个字节
void RingBuffer_WriteByte(RingBufferTypeDef *ringBuf, uint8_t value){
	RingBuffer_Push(ringBuf, value);
}

// 写入字节数组
void RingBuffer_WriteByteArray(RingBufferTypeDef *ringBuf, uint8_t* src, uint16_t size){
	for(uint16_t idx=0; idx<size; idx++){
		RingBuffer_Push(ringBuf, src[idx]);
	}
}
// 写入有符号Short整数(两个字节)
void RingBuffer_WriteShort(RingBufferTypeDef *ringBuf, int16_t value){
	uint8_t *p= (uint8_t *)&value;
	RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// 写入无符号Short整数(两个字节)
void RingBuffer_WriteUShort(RingBufferTypeDef *ringBuf, uint16_t value){
	uint8_t *p= (uint8_t *)&value;
	RingBuffer_WriteByteArray(ringBuf, p, 2);
}

// 写入有符号Long类型的整数(四个字节)
void RingBuffer_WriteLong(RingBufferTypeDef *ringBuf, int32_t value){
	uint8_t *p= (uint8_t *)&value;
	RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// 写入无符号Long类型的整数(四个字节)
void RingBuffer_WriteULong(RingBufferTypeDef *ringBuf, uint32_t value){
	uint8_t *p= (uint8_t *)&value;
	RingBuffer_WriteByteArray(ringBuf, p, 4);
}

// 写入浮点数(四个字节)
void RingBuffer_WriteFloat(RingBufferTypeDef *ringBuf, float value){
	uint8_t *p= (uint8_t *)&value;
	RingBuffer_WriteByteArray(ringBuf, p, 4);
}


// 计算校验和
uint8_t RingBuffer_GetChecksum(RingBufferTypeDef *ringBuf) {
    uint16_t nByte = RingBuffer_GetByteUsed(ringBuf);
    uint32_t bSum = 0;
    for (uint16_t i = 0; i < nByte; i++) {
        uint16_t rbIdx = (ringBuf->head + i + 1) % ringBuf->size;
        bSum = (bSum + ringBuf->buffer[rbIdx]) % 256;
    }
    return (uint8_t)bSum;
}

// 
uint8_t RingBuffer_PeekByte(RingBufferTypeDef *ringBuf, uint16_t index) {
    if (index >= RingBuffer_GetByteUsed(ringBuf)) return 0;
    uint16_t rbIdx = (ringBuf->head + index + 1) % ringBuf->size;
    return ringBuf->buffer[rbIdx];
}

uint16_t RingBuffer_PeekUShort(RingBufferTypeDef *ringBuf, uint16_t index) {
    if (index + 1 >= RingBuffer_GetByteUsed(ringBuf)) return 0;
    uint16_t rbIdx = (ringBuf->head + index + 1) % ringBuf->size;
    return (ringBuf->buffer[rbIdx] | (ringBuf->buffer[(rbIdx + 1) % ringBuf->size] << 8));
}

// 同步命令长度写入支持
void RingBuffer_WriteUShortWithFlag(RingBufferTypeDef *ringBuf, uint16_t value) {
    RingBuffer_WriteByte(ringBuf, 0xFF);       // 写入同步标志
    RingBuffer_WriteUShort(ringBuf, value);    // 写入16位长度
}
// 增强读取逻辑以解析同步长度
uint16_t RingBuffer_ReadUShortWithFlag(RingBufferTypeDef *ringBuf, uint8_t *isSync) {
    uint8_t flag = RingBuffer_ReadByte(ringBuf);
    *isSync = (flag == 0xFF) ? 1 : 0;
    if (*isSync) {
        return RingBuffer_ReadUShort(ringBuf); // 读取16位长度
    } else {
        return (uint16_t)flag;                // 直接返回8位长度
    }
}
