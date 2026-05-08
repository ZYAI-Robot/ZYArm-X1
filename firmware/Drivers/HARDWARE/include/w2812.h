/* Includes ------------------------------------------------------------------*/
#ifndef __W2812_H__
#define __W2812_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"

/* Exported constants --------------------------------------------------------*/
#define W2812_RESET_TIME_US           50      // 复位时间，单位微秒
#define W2812_COLOR_BITS              24      // 每个LED的颜色位数
#define W2812_TIMING_0_HIGH           0.4     // 0码高电平时间，单位微秒
#define W2812_TIMING_0_LOW            0.85    // 0码低电平时间，单位微秒
#define W2812_TIMING_1_HIGH           0.8     // 1码高电平时间，单位微秒
#define W2812_TIMING_1_LOW            0.45    // 1码低电平时间，单位微秒

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t r;    // 红色通道
    uint8_t g;    // 绿色通道
    uint8_t b;    // 蓝色通道
} W2812_ColorTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef W2812_Init(uint16_t ledCount);
HAL_StatusTypeDef W2812_DeInit(void);
HAL_StatusTypeDef W2812_SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef W2812_SetAllColors(uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef W2812_SetColors(uint16_t startIndex, uint16_t count, uint8_t* colors);
HAL_StatusTypeDef W2812_SendData(bool wait);
HAL_StatusTypeDef W2812_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __W2812_H__ */
