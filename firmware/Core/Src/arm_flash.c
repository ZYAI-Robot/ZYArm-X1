
#include "arm_flash.h"
#include "arm_shell.h"

#define ARM_FLASH_LOG_TAG "FLASH"

int arm_flash_erase_sector(uint32_t sector)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    
    safe_printf("Starting Flash sector 0x%x erase...\n", sector);
    
    // 解锁Flash
    HAL_FLASH_Unlock();

    // 使用RTOS安全方式禁用中断，避免全局中断关闭导致异常
    UBaseType_t primask = taskENTER_CRITICAL_FROM_ISR();
    
    // 清除所有错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    
    // 设置擦除参数
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    // 执行擦除
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    // 恢复中断状态
    taskEXIT_CRITICAL_FROM_ISR(primask);
    
    // 锁定Flash
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        ARM_LOGE_TAG(ARM_FLASH_LOG_TAG, "Flash erase failed, error code: %lu\n", sector_error);
        return -1;
    }
    
    safe_printf("Flash sector erase completed successfully\n");
    return 0;
}

static uint32_t calculate_checksum(const uint8_t* data, uint32_t size)
{
    uint32_t sum = 0;
    uint32_t i;
    
    for (i = 0; i < size; i++) {
        sum += data[i];
    }
    
    return sum;
}

int arm_flash_write(uint32_t addr, uint32_t offset, void* data, uint32_t size)
{   
    // 偏移与尺寸必须是按4字节对齐
    if ((offset % 4) || (size % 4)) {
        ARM_LOGE_TAG(ARM_FLASH_LOG_TAG, "Offset and size must be algin of 4\n");
        return -1;
    }

    // 解锁Flash
    HAL_FLASH_Unlock();

    uint32_t address = addr + offset;
    uint32_t *data_ptr = data;
    HAL_StatusTypeDef status;

    // 按字(32位)写入数据
    for (int i = 0; i < size/4; i++) {
        uint32_t data_word = data_ptr[i];
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, data_word);
        
        if (status != HAL_OK) {
            ARM_LOGE_TAG(ARM_FLASH_LOG_TAG, "Flash write failed at 0x%08lX\n", address + i * 4);
            HAL_FLASH_Lock();
            return false;
        }
        
        // 等待操作完成
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
            while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        }
    }
    
    // 锁定Flash
    HAL_FLASH_Lock();
    safe_printf("Flash write successful\n");
    return 0;
}

int arm_flash_config_save(void)
{
    safe_printf("Starting to write robot config data to Flash...\n");
    
    // 更新校验和
    g_arm_robot.cfg.checksum = calculate_checksum(
        (const uint8_t*)g_arm_robot.cfg.name, // 跳过前面的magic和check_sum
        sizeof(ArmConfig) - sizeof(uint32_t) * 2);
    
    // 确保魔术字已设置
    g_arm_robot.cfg.magic = AMR_FLASH_MAGIC;

    // 先擦除扇区
    if (arm_flash_erase_sector(ARM_CONFIG_FLASH_SECTOR) != 0) {
        return -1;
    }

    return arm_flash_write(ARM_CONFIG_FLASH_ADDR, 0, &g_arm_robot.cfg, sizeof(ArmConfig));
}

int arm_flash_config_load(void)
{
    ArmConfig *config = (ArmConfig *)ARM_CONFIG_FLASH_ADDR;
    if (config->magic != AMR_FLASH_MAGIC) {
        ARM_LOGE_TAG(ARM_FLASH_LOG_TAG, "Flash data magic word mismatch\n");
        return -1;
    }

    // 计算校验和并进行验证
    uint32_t calculated_checksum = calculate_checksum(
        (uint8_t*)(config->name), sizeof(ArmConfig) - sizeof(uint32_t) * 2);
    if (calculated_checksum != config->checksum) {
        ARM_LOGE_TAG(ARM_FLASH_LOG_TAG, "Checksum verification failed\n");
        return -1;
    }

    memcpy(&g_arm_robot.cfg, config, sizeof(ArmConfig));
    return 0;
}
