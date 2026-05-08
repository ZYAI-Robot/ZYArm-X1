#ifndef __ARM_FLASH_H
#define __ARM_FLASH_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include "arm_robot.h"
#include <stdint.h>

// Flash扇区定义 (STM32F407VE)
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)  // 16KB
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)  // 16KB
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)  // 16KB
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)  // 16KB
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)  // 64KB
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)  // 128KB
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)  // 128KB
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)  // 128KB

#define AMR_FLASH_MAGIC             0xAA55AA55U
#define ARM_CONFIG_FLASH_SECTOR     FLASH_SECTOR_7
#define ARM_CONFIG_FLASH_ADDR       ADDR_FLASH_SECTOR_7
#define ARM_CONFIG_FLASH_SIZE       (128U * 1024U)  // 128KB

int arm_flash_config_save(void);
int arm_flash_config_load(void);
int arm_flash_write(uint32_t addr, uint32_t offset, void* data, uint32_t size);
int arm_flash_erase_sector(uint32_t sector);

#endif
