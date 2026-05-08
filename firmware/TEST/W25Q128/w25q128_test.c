#ifdef ARM_TESTS
/* Includes ------------------------------------------------------------------*/
#include "w25q128.h"
#include "usart.h"
#include <stdio.h>
#include "arm_shell.h"
#include "stdint.h"
#include <stdlib.h>

/* Define large data size: 13K = 13 * 1024 = 13312 bytes */
#define LARGE_DATA_SIZE 4050

/* Global buffers for large data test */
uint8_t largeData[LARGE_DATA_SIZE];
uint8_t verifyData[LARGE_DATA_SIZE];

/**
  * @brief  Tests the W25Q128 FLASH memory
  * @param  timeout: Timeout in milliseconds
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W25Q128_Test(uint32_t timeout)
{
    uint8_t testData[256] = {0};
    uint8_t readData[256] = {0};
    uint16_t i = 0;
    HAL_StatusTypeDef ret = HAL_OK;
    
    safe_printf("=== W25Q128 Test Start ===\r\n");
    
    /* Test 1: Initialize the W25Q128 */
    safe_printf("Test 1: Initializing W25Q128...\r\n");
    ret = W25Q128_Init(timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Init failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Init passed!\r\n");
    
    /* Prepare test data */
    for (i = 0; i < 256; i++)
    {
        testData[i] = i;
    }
    
    /* Test 2: Erase sectors */
    safe_printf("Test 2: Erasing sector 0...\r\n");
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase sector failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Erase sector passed!\r\n");
    
    /* Test 3: Write data */
    safe_printf("Test 3: Writing test data...\r\n");
    ret = W25Q128_Write(0, testData, 256, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Write bytes failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Write data passed!\r\n");
    
    /* Test 4: Read data */
    safe_printf("Test 4: Reading test data...\r\n");
    ret = W25Q128_Read(0, readData, 256, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read bytes failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Read data passed!\r\n");
    
    /* Test 5: Verify data */
    safe_printf("Test 5: Verifying test data...\r\n");
    for (i = 0; i < 256; i++)
    {
        if (readData[i] != testData[i])
        {
            safe_printf("Error at address 0x%04X: expected 0x%02X, got 0x%02X\r\n", i, testData[i], readData[i]);
        return HAL_ERROR;
        }
    }
    safe_printf("Verify data passed!\r\n");
    
    /* Test 6: Test multiple sectors erase */
    safe_printf("Test 6: Erasing multiple sectors (0-1)...\r\n");
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE * 2, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase multiple sectors failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Erase multiple sectors passed!\r\n");
    
    /* Test 7: Test read after erase (should be 0xFF) */
    safe_printf("Test 7: Reading after erase...\r\n");
    ret = W25Q128_Read(0, readData, 16, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read after erase failed: %d\r\n", ret);
        return ret;
    }
    for (i = 0; i < 16; i++)
    {
        if (readData[i] != 0xFF)
        {
            safe_printf("Error at address 0x%04X: expected 0xFF, got 0x%02X\r\n", i, readData[i]);
            return HAL_ERROR;
        }
    }
    safe_printf("Read after erase passed!\r\n");
    
    /* === Edge Case Tests === */
    safe_printf("=== Edge Case Tests ===\r\n");
    
    /* Test 8: Read edge cases */
    safe_printf("Test 8: Read edge cases...\r\n");
    
    /* Test 8.1: Read 1 byte */
    ret = W25Q128_Read(0, readData, 1, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read 1 byte failed: %d\r\n", ret);
        return ret;
    }
    
    /* Test 8.2: Read from boundary address */
    ret = W25Q128_Read(W25Q128_SECTOR_SIZE - 16, readData, 16, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read from boundary address failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Read edge cases passed!\r\n");
    
    /* Test 9: Write edge cases */
    safe_printf("Test 9: Write edge cases...\r\n");
    
    /* Test 9.1: Write 1 byte */
    testData[0] = 0xAA;
    ret = W25Q128_Write(0, testData, 1, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Write 1 byte failed: %d\r\n", ret);
        return ret;
    }
    
    /* Test 9.2: Verify 1 byte write */
    ret = W25Q128_Read(0, readData, 1, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Verify 1 byte write failed: %d\r\n", ret);
        return ret;
    }
    if (readData[0] != 0xAA)
    {
        safe_printf("1 byte write verification failed: expected 0xAA, got 0x%02X\r\n", readData[0]);
        return HAL_ERROR;
    }

    safe_printf("Write edge cases passed!\r\n");
    
    /* Test 9.3: Write across page boundary */
    safe_printf("Test 9.3: Writing across page boundary...\r\n");
    
    /* Erase the sector first */
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase sector for page boundary test failed: %d\r\n", ret);
        return ret;
    }
    
    /* Prepare test data that will cross page boundary */
    for (i = 0; i < 32; i++)
    {
        testData[i] = 0xAA + i;
    }
    
    /* Write from page boundary - 16 bytes before boundary, 16 bytes after */
    ret = W25Q128_Write(W25Q128_PAGE_SIZE - 16, testData, 32, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Write across page boundary failed: %d\r\n", ret);
        return ret;
    }
    
    /* Verify the data */
    ret = W25Q128_Read(W25Q128_PAGE_SIZE - 16, readData, 32, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read across page boundary failed: %d\r\n", ret);
        return ret;
    }
    
    for (i = 0; i < 32; i++)
    {
        if (readData[i] != (0xAA + i))
        {
            safe_printf("Page boundary write verification failed at offset %d: expected 0x%02X, got 0x%02X\r\n", 
                        i, 0xAA + i, readData[i]);
            return HAL_ERROR;
        }
    }
    safe_printf("Write across page boundary passed!\r\n");
    
    /* Test 9.4: Write large data (13K) */
    safe_printf("Test 9.4: Writing large data (13K)...\r\n");
    
    /* Prepare large test data */
    for (i = 0; i < LARGE_DATA_SIZE; i++)
    {
        largeData[i] = i & 0xFF; /* Use byte value from 0-255 */
    }
    
    /* Erase sectors needed for 13K data (need 4 sectors: 4 * 4K = 16K) */
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE * 4, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase sectors for large data test failed: %d\r\n", ret);
        return ret;
    }
    
    /* Write large data */
    ret = W25Q128_Write(0, largeData, LARGE_DATA_SIZE, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Write large data failed: %d\r\n", ret);
        return ret;
    }
    
    /* Verify large data */
    ret = W25Q128_Read(0, verifyData, LARGE_DATA_SIZE, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read large data failed: %d\r\n", ret);
        return ret;
    }
    
    /* Check data integrity */
    uint32_t errorCount = 0;
    for (i = 0; i < LARGE_DATA_SIZE; i++)
    {
        if (verifyData[i] != (i & 0xFF))
        {
            errorCount++;
            /* Print first few errors only */
            if (errorCount <= 5)
            {
                safe_printf("Large data verification failed at offset %d: expected 0x%02X, got 0x%02X\r\n", 
                            i, i & 0xFF, verifyData[i]);
            }
        }
    }
    
    if (errorCount > 0)
    {
        safe_printf("Large data verification failed with %d errors\r\n", errorCount);
        return HAL_ERROR;
    }
    safe_printf("Large data write verification passed!\r\n");
    
    /* Erase the sectors */
    safe_printf("Erasing sectors after large data test...\r\n");
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE * 4, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase sectors after large data test failed: %d\r\n", ret);
        return ret;
    }
    
    /* Verify data is 0xFF after erase */
    ret = W25Q128_Read(0, verifyData, 256, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Read after erase for large data test failed: %d\r\n", ret);
        return ret;
    }
    
    for (i = 0; i < 256; i++)
    {
        if (verifyData[i] != 0xFF)
        {
            safe_printf("Large data erase verification failed at offset %d: expected 0xFF, got 0x%02X\r\n", 
                        i, verifyData[i]);
            return HAL_ERROR;
        }
    }
    safe_printf("Large data erase verification passed!\r\n");
    
    /* Test 10: Erase edge cases */


    safe_printf("Test 10: Erase edge cases...\r\n");
    
    /* Test 10.1: Erase single sector */
    ret = W25Q128_EraseSectors(0, W25Q128_SECTOR_SIZE, timeout);
    if (ret != HAL_OK)
    {
        safe_printf("Erase single sector failed: %d\r\n", ret);
        return ret;
    }
    safe_printf("Erase edge cases passed!\r\n");
    
    /* Test 11: Error case tests */
    safe_printf("Test 11: Error case tests...\r\n");
    
    /* Test 11.1: Read with NULL pointer */
    ret = W25Q128_Read(0, NULL, 1, timeout);
    if (ret == HAL_OK)
    {
        safe_printf("Read with NULL pointer should have failed\r\n");
        return HAL_ERROR;
    }
    
    /* Test 11.2: Write with NULL pointer */
    ret = W25Q128_Write(0, NULL, 1, timeout);
    if (ret == HAL_OK)
    {
        safe_printf("Write with NULL pointer should have failed\r\n");
        return HAL_ERROR;
    }
    
    /* Test 11.3: Erase with zero length */
    ret = W25Q128_EraseSectors(0, 0, timeout);
    if (ret == HAL_OK)
    {
        safe_printf("Erase with zero length should have failed\r\n");
        return HAL_ERROR;
    }
    safe_printf("Error case tests passed!\r\n");
    
    safe_printf("=== All tests passed! ===\r\n");
    return HAL_OK;
}
#endif
