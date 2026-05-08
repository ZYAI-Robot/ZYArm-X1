/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __W25Q128_H__
#define __W25Q128_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
// W25Q128 Commands
#define W25Q128_CMD_WRITE_ENABLE           0x06
#define W25Q128_CMD_WRITE_DISABLE          0x04
#define W25Q128_CMD_READ_STATUS_REG1       0x05
#define W25Q128_CMD_READ_STATUS_REG2       0x35
#define W25Q128_CMD_WRITE_STATUS_REG       0x01
#define W25Q128_CMD_PAGE_PROGRAM           0x02
#define W25Q128_CMD_QUAD_PAGE_PROGRAM      0x32
#define W25Q128_CMD_BLOCK_ERASE_64KB       0xD8
#define W25Q128_CMD_BLOCK_ERASE_32KB       0x52
#define W25Q128_CMD_SECTOR_ERASE_4KB       0x20
#define W25Q128_CMD_CHIP_ERASE             0xC7
#define W25Q128_CMD_ERASE_SUSPEND          0x75
#define W25Q128_CMD_ERASE_RESUME           0x7A
#define W25Q128_CMD_POWER_DOWN             0xB9
#define W25Q128_CMD_HIGH_PERFORMANCE_MODE  0xA3
#define W25Q128_CMD_CONTINUOUS_READ_MODE   0xFF
#define W25Q128_CMD_RELEASE_POWER_DOWN     0xAB
#define W25Q128_CMD_MANUFACTURER_DEVICE_ID 0x90
#define W25Q128_CMD_READ_UNIQUE_ID         0x4B
#define W25Q128_CMD_JEDEC_ID               0x9F
#define W25Q128_CMD_READ_DATA              0x03
#define W25Q128_CMD_FAST_READ              0x0B
#define W25Q128_CMD_FAST_READ_DUAL         0x3B
#define W25Q128_CMD_FAST_READ_QUAD         0x6B
#define W25Q128_CMD_WORD_READ_QUAD         0xEB
#define W25Q128_CMD_OCTAL_WORD_READ        0xE3
#define W25Q128_CMD_DUAL_IO_READ           0xBB
#define W25Q128_CMD_QUAD_IO_READ           0xEB

// W25Q128 Parameters
#define W25Q128_PAGE_SIZE                  0x100
#define W25Q128_SECTOR_SIZE                0x1000
#define W25Q128_BLOCK_SIZE_32KB            0x8000
#define W25Q128_BLOCK_SIZE_64KB            0x10000
#define W25Q128_TOTAL_SIZE                 0x1000000
#define W25Q128_SECTORS_COUNT              0x1000
#define W25Q128_BLOCKS_32KB_COUNT          0x200
#define W25Q128_BLOCKS_64KB_COUNT          0x100

// Status Register Bits
#define W25Q128_STATUS_REG1_BUSY           0x01
#define W25Q128_STATUS_REG1_WEL            0x02
#define W25Q128_STATUS_REG1_BP0            0x04
#define W25Q128_STATUS_REG1_BP1            0x08
#define W25Q128_STATUS_REG1_BP2            0x10
#define W25Q128_STATUS_REG1_TB             0x20
#define W25Q128_STATUS_REG1_SEC            0x40
#define W25Q128_STATUS_REG1_SRP0           0x80

#define W25Q128_STATUS_REG2_SRP1           0x01
#define W25Q128_STATUS_REG2_QE             0x02
#define W25Q128_STATUS_REG2_LB             0x0C
#define W25Q128_STATUS_REG2_CMP            0x10
#define W25Q128_STATUS_REG2_SUS            0x80

/* Exported functions prototypes ---------------------------------------------*/
// W25Q128 Operations
HAL_StatusTypeDef W25Q128_Init(uint32_t timeout);
HAL_StatusTypeDef W25Q128_Read(uint32_t addr, uint8_t* data, uint32_t len, uint32_t timeout);
HAL_StatusTypeDef W25Q128_Write(uint32_t addr, uint8_t* data, uint32_t len, uint32_t timeout);
HAL_StatusTypeDef W25Q128_EraseSectors(uint32_t startAddr, uint32_t len, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H__ */
