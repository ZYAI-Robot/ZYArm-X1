/* Includes ------------------------------------------------------------------*/
#include "w2812.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "string.h"
#include "arm_shell.h"

/* Private constants ---------------------------------------------------------*/
#define W2812_SPI_HANDLE              &hspi3
#define W2812_SPI_TIMEOUT             100

/* Private variables ---------------------------------------------------------*/
static uint16_t g_ledCount = 0;
static uint8_t* g_colorBuffer = NULL;
static uint8_t* g_spiBuffer = NULL;
static uint32_t g_spiBufferSize = 0;

/* Private function prototypes -----------------------------------------------*/
static void W2812_ConvertColorToSpiData(void);
static void W2812_DelayUs(uint32_t us);

/**
  * @brief  Initializes the W2812 LED strip
  * @param  ledCount: Number of LEDs in the strip
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_Init(uint16_t ledCount)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    /* Check if ledCount is valid */
    if (ledCount == 0)
    {
        return HAL_ERROR;
    }
    
    /* If already initialized, deinitialize first */
    if (g_colorBuffer != NULL || g_spiBuffer != NULL)
    {
        W2812_DeInit();
    }
    
    /* Store the LED count */
    g_ledCount = ledCount;
    
    /* Calculate buffer sizes */
    uint32_t colorBufferSize = ledCount * 3;  // 3 bytes per LED (RGB)
    g_spiBufferSize = ledCount * W2812_COLOR_BITS;  // 24 bits per LED, each bit gets a 16-bit SPI word
    
    /* Allocate memory for buffers */
    g_colorBuffer = (uint8_t*)malloc(colorBufferSize);
    g_spiBuffer = (uint8_t*)malloc(g_spiBufferSize * 2);  // 16-bit per SPI word (2 bytes per word)
    
    if (g_colorBuffer == NULL || g_spiBuffer == NULL)
    {
        /* Free allocated memory if any */
        if (g_colorBuffer != NULL)
        {
            free(g_colorBuffer);
            g_colorBuffer = NULL;
        }
        if (g_spiBuffer != NULL)
        {
            free(g_spiBuffer);
            g_spiBuffer = NULL;
        }
        g_ledCount = 0;
        g_spiBufferSize = 0;
        return HAL_ERROR;
    }
    
    /* Initialize color buffer to all black */
    memset(g_colorBuffer, 0, colorBufferSize);
    memset(g_spiBuffer, 0, g_spiBufferSize * 2);    
    /* Send reset signal */
    ret = W2812_Reset();
    
    return ret;
}

/**
  * @brief  Deinitializes the W2812 LED strip
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_DeInit(void)
{
    /* Free allocated memory */
    if (g_colorBuffer != NULL)
    {
        free(g_colorBuffer);
        g_colorBuffer = NULL;
    }
    if (g_spiBuffer != NULL)
    {
        free(g_spiBuffer);
        g_spiBuffer = NULL;
    }
    
    /* Reset LED count */
    g_ledCount = 0;
    g_spiBufferSize = 0;
    
    return HAL_OK;
}

/**
  * @brief  Sets the color of a single LED
  * @param  index: Index of the LED (0-based)
  * @param  r: Red component (0-255)
  * @param  g: Green component (0-255)
  * @param  b: Blue component (0-255)
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= g_ledCount || g_colorBuffer == NULL)
    {
        return HAL_ERROR;
    }
    
    /* W2812 uses GRB format */
    g_colorBuffer[index * 3 + 0] = g;
    g_colorBuffer[index * 3 + 1] = r;
    g_colorBuffer[index * 3 + 2] = b;
    
    return HAL_OK;
}

/**
  * @brief  Sets the color of all LEDs
  * @param  r: Red component (0-255)
  * @param  g: Green component (0-255)
  * @param  b: Blue component (0-255)
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_SetAllColors(uint8_t r, uint8_t g, uint8_t b)
{
    if (g_colorBuffer == NULL || g_ledCount == 0)
    {
        return HAL_ERROR;
    }
    
    /* W2812 uses GRB format */
    for (uint16_t i = 0; i < g_ledCount; i++)
    {
        g_colorBuffer[i * 3 + 0] = g;
        g_colorBuffer[i * 3 + 1] = r;
        g_colorBuffer[i * 3 + 2] = b;
    }
    
    return HAL_OK;
}

/**
  * @brief  Sets the color of multiple LEDs
  * @param  startIndex: Start index of the LEDs (0-based)
  * @param  count: Number of LEDs to set
  * @param  colors: Array of RGB colors (each color is 3 bytes: R, G, B)
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_SetColors(uint16_t startIndex, uint16_t count, uint8_t* colors)
{
    if (startIndex >= g_ledCount || (startIndex + count) > g_ledCount || 
        g_colorBuffer == NULL || colors == NULL)
    {
        return HAL_ERROR;
    }
    
    /* W2812 uses GRB format */
    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t r = colors[i * 3 + 0];
        uint8_t g = colors[i * 3 + 1];
        uint8_t b = colors[i * 3 + 2];
        
        g_colorBuffer[(startIndex + i) * 3 + 0] = g;
        g_colorBuffer[(startIndex + i) * 3 + 1] = r;
        g_colorBuffer[(startIndex + i) * 3 + 2] = b;
    }
    
    return HAL_OK;
}

/**
  * @brief  Sends data to the W2812 LED strip
  * @retval HAL_StatusTypeDef: HAL status
  * @param  wait: Whether to wait for DMA transfer to complete
  */
HAL_StatusTypeDef W2812_SendData(bool wait)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    if (g_ledCount == 0 || g_colorBuffer == NULL || g_spiBuffer == NULL)
    {
        return HAL_ERROR;
    }
    
    /* Convert color data to SPI timing data */
    W2812_ConvertColorToSpiData();
    
    /* Send data via SPI DMA */
    /* For 16-bit SPI, the size should be in half-words */
    ret = HAL_SPI_Transmit_DMA(W2812_SPI_HANDLE, g_spiBuffer, g_spiBufferSize);

    /* Wait for DMA transfer to complete */
    if ((ret == HAL_OK) && wait)
    {
        /* Wait until SPI is not busy with timeout */
        uint32_t timeout = HAL_GetTick() + W2812_SPI_TIMEOUT;
        while (HAL_SPI_GetState(W2812_SPI_HANDLE) != HAL_SPI_STATE_READY)
        {
            if (HAL_GetTick() > timeout)
            {
                safe_printf("SPI timeout\n");
                ret = HAL_TIMEOUT;
                break;
            }
        }
        
        /* Send reset signal */
        W2812_Reset();
    }
    return ret;
}

/**
  * @brief  Sends reset signal to the W2812 LED strip
  * @retval HAL_StatusTypeDef: HAL status
  */
HAL_StatusTypeDef W2812_Reset(void)
{
    /* W2812 requires a low pulse of at least 50us to reset */
    /* For SPI-based implementation, we need to ensure the bus is idle
     * The delay alone should be sufficient for reset
     */
    W2812_DelayUs(W2812_RESET_TIME_US);
    return HAL_OK;
}

/**
  * @brief  Converts color data to SPI timing data
  * @retval None
  */
static void W2812_ConvertColorToSpiData(void)
{
    if (g_colorBuffer == NULL || g_spiBuffer == NULL)
    {
        return;
    }
    
    uint8_t* colorPtr = g_colorBuffer;
    uint16_t* spiPtr = (uint16_t*)g_spiBuffer;
    
    /* Pre-calculate 0 and 1 codes for faster access
     * SPI clock frequency: 10.5MHz (168MHz / 4), each bit ~0.0952μs
     * W2812 timing requirements:
     * 0 code: high ~0.4μs, low ~0.85μs
     * 1 code: high ~0.8μs, low ~0.45μs
     * 
     * Adjusted for 16-bit SPI words:
     * 0 code: 4 high bits + 12 low bits (4*0.0952=0.381μs high, 12*0.0952=1.142μs low)
     * 1 code: 8 high bits + 8 low bits (8*0.0952=0.762μs high, 8*0.0952=0.762μs low)
     * 
     * Note: STM32 SPI DMA uses little-endian for 16-bit transfers, so we need to swap bytes
     */
    const uint16_t CODE_0 = __REV16(0xF000);  // Byte-swapped for little-endian SPI DMA
    const uint16_t CODE_1 = __REV16(0xFF00);  // Byte-swapped for little-endian SPI DMA
    
    /* Process each LED */
    for (uint16_t ledIndex = 0; ledIndex < g_ledCount; ledIndex++)
    {
        /* Get GRB color data */
        uint8_t g = *colorPtr++;
        uint8_t r = *colorPtr++;
        uint8_t b = *colorPtr++;
                
        /* Process each bit in GRB order using faster bit manipulation */
        // Green channel
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            *spiPtr++ = (g & mask) ? CODE_1 : CODE_0;
        }
        
        // Red channel
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            *spiPtr++ = (r & mask) ? CODE_1 : CODE_0;
        }
        
        // Blue channel
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            *spiPtr++ = (b & mask) ? CODE_1 : CODE_0;
        }
    }
}

/**
  * @brief  Delays for a specified number of microseconds
  * @param  us: Number of microseconds to delay
  * @retval None
  */
static void W2812_DelayUs(uint32_t us)
{
    /* For W2812 reset signal, we only need a delay > 50us
     * HAL_Delay provides millisecond-level precision, which is sufficient
     */
    if (us >= 1000)
    {
        HAL_Delay(us / 1000);
    }
    else
    {
        /* For delays less than 1ms, use HAL_Delay(1) to ensure enough delay */
        HAL_Delay(1);
    }
}
