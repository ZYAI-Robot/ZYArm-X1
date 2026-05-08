/* Includes ------------------------------------------------------------------*/
#include "w25q128.h"
#include "spi.h"
#include "stm32f4xx_hal_gpio.h"
#include "arm_shell.h"

#define SPI_SELECT_W25Q128()	HAL_GPIO_WritePin(W25Q128_NSS_GPIO_Port, W25Q128_NSS_Pin, GPIO_PIN_RESET)	//CS=0
#define SPI_DESELECT_W25Q128()	HAL_GPIO_WritePin(W25Q128_NSS_GPIO_Port, W25Q128_NSS_Pin, GPIO_PIN_SET)	    //CS=1
#define W25Q128_ID 0xEF4018
/**
  * @brief  Waits until the W25Q128 is ready
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
static HAL_StatusTypeDef W25Q128_WaitForReady(uint32_t timeout)
{
    uint8_t status = 0;
    uint32_t startTick = HAL_GetTick();
    HAL_StatusTypeDef ret = HAL_OK;
    do
    {
        uint8_t cmd = W25Q128_CMD_READ_STATUS_REG1;
        SPI_SELECT_W25Q128();
        ret = HAL_SPI_Transmit(&hspi1, &cmd, 1, timeout);
        if (ret != HAL_OK)
        {
            SPI_DESELECT_W25Q128();
            return ret;
        }
        
        ret = HAL_SPI_Receive(&hspi1, &status, 1, timeout);
        SPI_DESELECT_W25Q128();
        if (ret != HAL_OK)
        {
            return ret;
        }
        
        if ((HAL_GetTick() - startTick) > timeout)
        {
            ret = HAL_TIMEOUT;
            return ret;
        }
    } while (status & W25Q128_STATUS_REG1_BUSY);

    return ret;
}

/**
  * @brief  Enables the write operation
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
static HAL_StatusTypeDef W25Q128_WriteEnable(uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;
    SPI_SELECT_W25Q128();
    
    ret = HAL_SPI_Transmit(&hspi1, &cmd, 1, timeout);
    
    SPI_DESELECT_W25Q128();
    return ret;
}

/**
  * @brief  Reads the ID of the W25Q128
  * @param  id: Pointer to store the ID
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
static HAL_StatusTypeDef W25Q128_ReadID(uint32_t* id, uint32_t timeout)
{
    uint8_t idBytes[3] = {0};
    uint8_t cmd = W25Q128_CMD_JEDEC_ID;

    HAL_StatusTypeDef ret = HAL_OK;
    SPI_SELECT_W25Q128();
    
    /* Send the command */
    ret = HAL_SPI_Transmit(&hspi1, &cmd, 1, timeout);
    if (ret != HAL_OK)
    {
        SPI_DESELECT_W25Q128();
        return ret;
    }

    /* Read the ID */
    ret = HAL_SPI_Receive(&hspi1, idBytes, 3, timeout);
    SPI_DESELECT_W25Q128();
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    *id = ((uint32_t)idBytes[0] << 16) | ((uint32_t)idBytes[1] << 8) | (uint32_t)idBytes[2];

    return ret;
}

/**
  * @brief  Initializes the W25Q128
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W25Q128_Init(uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint32_t id = 0;
    SPI_DESELECT_W25Q128(); // Ensure the device is in the correct state
    
    /* SPI1 is already initialized by MX_SPI1_Init() */
    /* Wait for the device to be ready */
    ret = W25Q128_WaitForReady(timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    /* Read and verify the ID */
    ret = W25Q128_ReadID(&id, timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    /* Check if the ID is valid for W25Q128 */
    safe_printf("W25Q128 ID: 0x%X\n", id);
    if ((id & 0xFFFFFF) != W25Q128_ID)
    {
        ret = HAL_ERROR;
    }
   
    return ret;
}

/**
  * @brief  Reads bytes from the W25Q128
  * @param  addr: Address to read from
  * @param  data: Buffer to store the data
  * @param  len: Length of data to read
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W25Q128_Read(uint32_t addr, uint8_t* data, uint32_t len, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    /* Check if parameters are valid */
    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }
    
    if (addr >= W25Q128_TOTAL_SIZE || (addr + len) > W25Q128_TOTAL_SIZE)
    {
        return HAL_ERROR;
    }

    /* Wait for the device to be ready */
    ret = W25Q128_WaitForReady(timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    SPI_SELECT_W25Q128();
    
    /* Send the command and address */
    uint8_t cmd[4] = {W25Q128_CMD_READ_DATA, 
                     (uint8_t)(addr >> 16), 
                     (uint8_t)(addr >> 8), 
                     (uint8_t)addr};
    ret = HAL_SPI_Transmit(&hspi1, cmd, 4, timeout);
    if (ret != HAL_OK)
    {
        SPI_DESELECT_W25Q128();
        return ret;
    }
    
    /* Read the data */
    ret = HAL_SPI_Receive(&hspi1, data, len, timeout);    
    SPI_DESELECT_W25Q128(); 
    
    return ret;
}

/**
  * @brief  Writes a page to the W25Q128
  * @param  addr: Address to write to
  * @param  data: Data to write
  * @param  len: Length of data to write
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
static HAL_StatusTypeDef W25Q128_WritePage(uint32_t addr, uint8_t* data, uint16_t len, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    if (len > W25Q128_PAGE_SIZE)
    {
        len = W25Q128_PAGE_SIZE;
    }
    
    /* Enable write operation */
    ret = W25Q128_WriteEnable(timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    /* Wait for the device to be ready */
    ret = W25Q128_WaitForReady(timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    SPI_SELECT_W25Q128();
    
    /* Send the command and address */
    uint8_t cmd[4] = {W25Q128_CMD_PAGE_PROGRAM, 
                     (uint8_t)(addr >> 16), 
                     (uint8_t)(addr >> 8), 
                     (uint8_t)addr};
    ret = HAL_SPI_Transmit(&hspi1, cmd, 4, timeout);
    if (ret != HAL_OK)
    {
        SPI_DESELECT_W25Q128();
        return ret;
    }
    
    /* Send the data */
    ret = HAL_SPI_Transmit(&hspi1, data, len, timeout);
    SPI_DESELECT_W25Q128();
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    /* Wait for the device to be ready */
    ret = W25Q128_WaitForReady(timeout);
    if (ret != HAL_OK)
    {
        return ret;
    }
    
    return ret;
}

/**
  * @brief  Writes bytes to the W25Q128
  * @param  addr: Address to write to
  * @param  data: Data to write
  * @param  len: Length of data to write
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W25Q128_Write(uint32_t addr, uint8_t* data, uint32_t len, uint32_t timeout)
{
    uint16_t pageRemaining = W25Q128_PAGE_SIZE - (addr % W25Q128_PAGE_SIZE);
    HAL_StatusTypeDef ret = HAL_OK;
    
    /* Check if parameters are valid */
    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }
    
    if (addr >= W25Q128_TOTAL_SIZE || (addr + len) > W25Q128_TOTAL_SIZE)
    {
        return HAL_ERROR;
    }
    
    if (len <= pageRemaining)
    {
        ret = W25Q128_WritePage(addr, data, len, timeout);
        if (ret != HAL_OK)
        {
            return ret;
        }
    }
    else
    {
        /* Write the first part */
        ret = W25Q128_WritePage(addr, data, pageRemaining, timeout);
        if (ret != HAL_OK)
        {
            return ret;
        }
        len -= pageRemaining;
        data += pageRemaining;
        addr += pageRemaining;
        
        /* Write the middle parts */
        while (len >= W25Q128_PAGE_SIZE)
        {
            ret = W25Q128_WritePage(addr, data, W25Q128_PAGE_SIZE, timeout);
            if (ret != HAL_OK)
            {
                return ret;
            }
            len -= W25Q128_PAGE_SIZE;
            data += W25Q128_PAGE_SIZE;
            addr += W25Q128_PAGE_SIZE;
        }
        
        /* Write the last part */
        if (len > 0)
        {
            ret = W25Q128_WritePage(addr, data, len, timeout);
            if (ret != HAL_OK)
            {
                return ret;
            }
        }
    }
    
    return ret;
}

/**
  * @brief  Erases sectors of the W25Q128
  * @param  startAddr: Start address to erase (must be sector aligned)
  * @param  len: Length to erase
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W25Q128_EraseSectors(uint32_t startAddr, uint32_t len, uint32_t timeout)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint32_t sectorAddr;
    uint32_t sectorCount;
    
    /* Check if parameters are valid */
    if (len == 0)
    {
        return HAL_ERROR;
    }
    
    if (startAddr >= W25Q128_TOTAL_SIZE || (startAddr + len) > W25Q128_TOTAL_SIZE)
    {
        return HAL_ERROR;
    }
    
    /* Check if start address is sector aligned */
    if (startAddr % W25Q128_SECTOR_SIZE != 0)
    {
        safe_printf("Error: Start address is not sector aligned\n");
        return HAL_ERROR;
    }
    
    /* Check if length is sector aligned */
    if (len % W25Q128_SECTOR_SIZE != 0)
    {
        safe_printf("Error: Length is not sector aligned\n");
        return HAL_ERROR;
    }
    
    /* Calculate the number of sectors to erase */
    sectorCount = (len + W25Q128_SECTOR_SIZE - 1) / W25Q128_SECTOR_SIZE;
    
    /* Erase each sector */
    for (uint32_t i = 0; i < sectorCount; i++)
    {
        sectorAddr = startAddr + (i * W25Q128_SECTOR_SIZE);
        
        /* Enable write operation */
        ret = W25Q128_WriteEnable(timeout);
        if (ret != HAL_OK)
        {
            return ret;
        }
        
        /* Wait for the device to be ready */
        ret = W25Q128_WaitForReady(timeout);
        if (ret != HAL_OK)
        {
            return ret;
        }
        
        /* Select the W25Q128 */
        SPI_SELECT_W25Q128();
        
        /* Send the command and address */
        uint8_t cmd[4] = {W25Q128_CMD_SECTOR_ERASE_4KB, 
                         (uint8_t)(sectorAddr >> 16), 
                         (uint8_t)(sectorAddr >> 8), 
                         (uint8_t)sectorAddr};
        ret = HAL_SPI_Transmit(&hspi1, cmd, 4, timeout);
        SPI_DESELECT_W25Q128();
        if (ret != HAL_OK)
        {
            return ret;
        }
        
        /* Wait for the device to be ready */
        ret = W25Q128_WaitForReady(timeout);
        if (ret != HAL_OK)
        {
            return ret;
        }
    }
        
    return ret;
}
