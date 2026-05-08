#include "fs_usart.h"

#include <string.h>

uint8_t UsartSendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t UsartRecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef UsartSendRingBuf;
RingBufferTypeDef UsartRecvRingBuf;
Usart_DataTypeDef FSUS_Usart;
uint8_t rc1;

#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk) && defined(CoreDebug_DEMCR_TRCENA_Msk)
#define USART_HAS_DWT_CYCLE_COUNTER 1
#else
#define USART_HAS_DWT_CYCLE_COUNTER 0
#endif

#define FSUS_REQUEST_CMD_READ_ANGLE 10U
#define FSUS_REQUEST_CMD_MONITOR 22U
#define FSUS_REQUEST_CMD_SYNC_COMMAND 25U

static uint8_t usart_cycle_counter_ready = 0U;

static void usart_cycle_counter_init(void)
{
#if USART_HAS_DWT_CYCLE_COUNTER
    if (usart_cycle_counter_ready == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        usart_cycle_counter_ready = 1U;
    }
#else
    usart_cycle_counter_ready = 1U;
#endif
}

static uint32_t usart_time_now(void)
{
    usart_cycle_counter_init();

#if USART_HAS_DWT_CYCLE_COUNTER
    return DWT->CYCCNT;
#else
    return HAL_GetTick();
#endif
}

static uint32_t usart_elapsed_us(uint32_t start, uint32_t end)
{
    uint32_t elapsed = end - start;

#if USART_HAS_DWT_CYCLE_COUNTER
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    if (cycles_per_us == 0U) {
        return 0U;
    }

    return elapsed / cycles_per_us;
#else
    return elapsed * 1000U;
#endif
}

static uint32_t usart_update_avg_us(uint32_t previous_avg_us, uint32_t sample_us, uint32_t sample_count)
{
    if (sample_count == 0U) {
        return 0U;
    }

    if (sample_count == 1U) {
        return sample_us;
    }

    return (uint32_t)((((uint64_t)previous_avg_us) * (sample_count - 1U) + sample_us) / sample_count);
}

static void usart_note_tx_dma_start_failure(Usart_DataTypeDef *usart, HAL_StatusTypeDef status)
{
    if (usart == NULL) {
        return;
    }

    if (status == HAL_BUSY) {
        usart->tx_dma_start_busy_count++;
    } else {
        usart->tx_dma_start_fail_count++;
    }
}

static void usart_note_request_reply_fail_reason(
    Usart_DataTypeDef *usart,
    UsartRequestReplyFailReason fail_reason
)
{
    if (usart == NULL) {
        return;
    }

    switch (fail_reason) {
        case USART_REQ_REPLY_FAIL_TIMEOUT:
            usart->request_reply_fail_timeout_count++;
            break;
        case USART_REQ_REPLY_FAIL_SEND_FAIL:
            usart->request_reply_fail_send_fail_count++;
            break;
        case USART_REQ_REPLY_FAIL_TX_ERROR_EVENT:
            usart->request_reply_fail_tx_error_event_count++;
            break;
        case USART_REQ_REPLY_FAIL_BAD_PACKET:
            usart->request_reply_fail_bad_packet_count++;
            break;
        case USART_REQ_REPLY_FAIL_REPLY_MISMATCH:
            usart->request_reply_fail_reply_mismatch_count++;
            break;
        case USART_REQ_REPLY_FAIL_REPLY_INCOMPLETE:
            usart->request_reply_fail_reply_incomplete_count++;
            break;
        case USART_REQ_REPLY_FAIL_NONE:
        default:
            break;
    }
}

static void usart_note_uart_error(Usart_DataTypeDef *usart, uint32_t error_code)
{
    uint32_t non_dma_error_bits;

    if (usart == NULL) {
        return;
    }

    non_dma_error_bits = error_code & ~(uint32_t)HAL_UART_ERROR_DMA;

    if ((error_code & HAL_UART_ERROR_DMA) != 0U) {
        usart->uart_dma_error_count++;
    }
    if (non_dma_error_bits != 0U) {
        usart->uart_line_error_count++;
    }
}

static void usart_note_timeout_for_cmd(Usart_DataTypeDef *usart, uint8_t cmd_id)
{
    if (usart == NULL) {
        return;
    }

    switch (cmd_id) {
        case FSUS_REQUEST_CMD_READ_ANGLE:
            usart->timeout_read_angle_count++;
            break;
        case FSUS_REQUEST_CMD_MONITOR:
            usart->timeout_monitor_count++;
            break;
        case FSUS_REQUEST_CMD_SYNC_COMMAND:
            usart->timeout_sync_command_count++;
            break;
        default:
            usart->timeout_other_count++;
            break;
    }
}

static void usart_set_tx_mode(Usart_DataTypeDef *usart)
{
    if ((usart == NULL) || (usart->huartX == NULL)) {
        return;
    }

    if (usart->huartX->Instance == USART6) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
    }
}

static void usart_set_rx_mode(Usart_DataTypeDef *usart)
{
    if ((usart == NULL) || (usart->huartX == NULL)) {
        return;
    }

    if (usart->huartX->Instance == USART6) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}

static void usart_notify_waiter_from_isr(Usart_DataTypeDef *usart, uint32_t events, BaseType_t *task_woken)
{
    TaskHandle_t waiting_task;

    if ((usart == NULL) || (task_woken == NULL)) {
        return;
    }

    waiting_task = usart->waiting_task;
    if (waiting_task != NULL) {
        xTaskNotifyFromISR(waiting_task, events, eSetBits, task_woken);
    }
}

static void usart_push_rx_byte_from_isr(Usart_DataTypeDef *usart, uint8_t value, BaseType_t *task_woken)
{
    if (usart == NULL) {
        return;
    }

    if (RingBuffer_IsFull(usart->recvBuf)) {
        usart->rx_ring_overflow_count++;
    } else {
        RingBuffer_Push(usart->recvBuf, value);
    }

    usart_notify_waiter_from_isr(usart, FSUS_USART_NOTIFY_RX, task_woken);
}

static uint16_t usart_drain_send_buffer(Usart_DataTypeDef *usart)
{
    uint16_t bytes_to_send;

    if (usart == NULL) {
        return 0U;
    }

    bytes_to_send = RingBuffer_GetByteUsed(usart->sendBuf);
    if (bytes_to_send > USART_SEND_BUF_SIZE) {
        bytes_to_send = USART_SEND_BUF_SIZE;
    }

    for (uint16_t i = 0; i < bytes_to_send; ++i) {
        usart->tx_buffer[i] = RingBuffer_Pop(usart->sendBuf);
    }

    return bytes_to_send;
}

static HAL_StatusTypeDef usart_wait_for_tc_flag(Usart_DataTypeDef *usart, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if ((usart == NULL) || (usart->huartX == NULL)) {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(usart->huartX, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
        taskYIELD();
    }

    return HAL_OK;
}

static TickType_t usart_gap_ticks_from_ms(uint32_t gap_ms)
{
    TickType_t gap_ticks = pdMS_TO_TICKS(gap_ms);

    if ((gap_ms > 0U) && (gap_ticks == 0U)) {
        gap_ticks = 1U;
    }

    return gap_ticks;
}

static void usart_wait_min_transaction_gap(Usart_DataTypeDef *usart)
{
    uint32_t last_complete_tick_ms;
    uint8_t last_complete_valid;
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    TickType_t wait_ticks;

    if ((usart == NULL) || (FSUS_TRANSACTION_GAP_MS == 0U)) {
        return;
    }

    taskENTER_CRITICAL();
    last_complete_tick_ms = usart->last_transaction_complete_tick_ms;
    last_complete_valid = usart->last_transaction_complete_valid;
    taskEXIT_CRITICAL();

    if (last_complete_valid == 0U) {
        return;
    }

    elapsed_ms = HAL_GetTick() - last_complete_tick_ms;
    if (elapsed_ms >= FSUS_TRANSACTION_GAP_MS) {
        return;
    }

    remaining_ms = FSUS_TRANSACTION_GAP_MS - elapsed_ms;
    wait_ticks = usart_gap_ticks_from_ms(remaining_ms);
    if (wait_ticks > 0U) {
        vTaskDelay(wait_ticks);
    }
}

void User_Uart_Init(UART_HandleTypeDef *huartx)
{
    RingBuffer_Init(&UsartSendRingBuf, USART_SEND_BUF_SIZE, UsartSendBuf);
    RingBuffer_Init(&UsartRecvRingBuf, USART_RECV_BUF_SIZE, UsartRecvBuf);
    usart_cycle_counter_init();

    memset(&FSUS_Usart, 0, sizeof(FSUS_Usart));
    FSUS_Usart.recvBuf = &UsartRecvRingBuf;
    FSUS_Usart.sendBuf = &UsartSendRingBuf;
    FSUS_Usart.huartX = huartx;
    FSUS_Usart.transaction_mutex = xSemaphoreCreateRecursiveMutex();

    usart_set_rx_mode(&FSUS_Usart);
    HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
}

BaseType_t Usart_BeginTransaction(Usart_DataTypeDef *usart, TickType_t timeout_ticks)
{
    TaskHandle_t current_task;
    uint32_t events;

    if ((usart == NULL) || (usart->transaction_mutex == NULL)) {
        return pdFALSE;
    }

    if (xSemaphoreTakeRecursive(usart->transaction_mutex, timeout_ticks) != pdTRUE) {
        return pdFALSE;
    }

    current_task = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL();
    if (usart->owner_task == current_task) {
        usart->transaction_depth++;
        taskEXIT_CRITICAL();
        return pdTRUE;
    }

    usart->owner_task = current_task;
    usart->waiting_task = current_task;
    usart->transaction_depth = 1U;
    taskEXIT_CRITICAL();

    usart_wait_min_transaction_gap(usart);
    Usart_ResetRxBuffer(usart, pdTRUE);
    (void)xTaskNotifyWait(0U, FSUS_USART_NOTIFY_ALL, &events, 0U);
    return pdTRUE;
}

void Usart_EndTransaction(Usart_DataTypeDef *usart)
{
    uint32_t events;

    if ((usart == NULL) || (usart->transaction_mutex == NULL)) {
        return;
    }

    taskENTER_CRITICAL();
    if (usart->transaction_depth > 1U) {
        usart->transaction_depth--;
        taskEXIT_CRITICAL();
        (void)xSemaphoreGiveRecursive(usart->transaction_mutex);
        return;
    }

    usart->transaction_depth = 0U;
    usart->owner_task = NULL;
    usart->waiting_task = NULL;
    usart->request_cmd_id = 0U;
    usart->request_servo_id = 0U;
    usart->request_reply_pending = 0U;
    usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_NONE;
    taskEXIT_CRITICAL();

    (void)xTaskNotifyWait(0U, FSUS_USART_NOTIFY_ALL, &events, 0U);
    taskENTER_CRITICAL();
    usart->last_transaction_complete_tick_ms = HAL_GetTick();
    usart->last_transaction_complete_valid = 1U;
    taskEXIT_CRITICAL();
    (void)xSemaphoreGiveRecursive(usart->transaction_mutex);
}

void Usart_ResetRxBuffer(Usart_DataTypeDef *usart, BaseType_t count_stale_bytes)
{
    uint16_t pending_bytes;

    if (usart == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    pending_bytes = RingBuffer_GetByteUsed(usart->recvBuf);
    if ((count_stale_bytes == pdTRUE) && (pending_bytes > 0U)) {
        usart->stale_rx_drop_count += pending_bytes;
    }
    RingBuffer_Reset(usart->recvBuf);
    taskEXIT_CRITICAL();
}

BaseType_t Usart_WaitForNotification(uint32_t timeout_ticks, uint32_t *events)
{
    return xTaskNotifyWait(0U, FSUS_USART_NOTIFY_ALL, events, timeout_ticks);
}

void Usart_NoteRequestStart(Usart_DataTypeDef *usart, uint8_t cmd_id, uint8_t servo_id)
{
    if (usart == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    usart->request_cmd_id = cmd_id;
    usart->request_servo_id = servo_id;
    usart->request_reply_start_cycles = usart_time_now();
    usart->request_reply_pending = 1U;
    usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_NONE;
    taskEXIT_CRITICAL();
}

void Usart_NoteRequestTimeout(Usart_DataTypeDef *usart)
{
    if (usart == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    if (usart->request_reply_pending != 0U) {
        usart_note_timeout_for_cmd(usart, usart->request_cmd_id);
    }
    taskEXIT_CRITICAL();
}

void Usart_NoteRequestReplyResult(
    Usart_DataTypeDef *usart,
    BaseType_t success,
    UsartRequestReplyFailReason fail_reason
)
{
    uint32_t end;
    uint32_t elapsed_us;
    uint32_t success_count;

    if (usart == NULL) {
        return;
    }

    end = usart_time_now();

    taskENTER_CRITICAL();
    if (usart->request_reply_pending == 0U) {
        taskEXIT_CRITICAL();
        return;
    }

    elapsed_us = usart_elapsed_us(usart->request_reply_start_cycles, end);
    usart->request_reply_pending = 0U;
    usart->request_reply_last_us = elapsed_us;
    if (elapsed_us > usart->request_reply_max_us) {
        usart->request_reply_max_us = elapsed_us;
    }

    if (success == pdTRUE) {
        success_count = usart->request_reply_success_count + 1U;
        usart->request_reply_success_count = success_count;
        usart->request_reply_avg_us =
            usart_update_avg_us(usart->request_reply_avg_us, elapsed_us, success_count);
    } else {
        usart->request_reply_fail_count++;
        if (fail_reason == USART_REQ_REPLY_FAIL_NONE) {
            fail_reason = USART_REQ_REPLY_FAIL_SEND_FAIL;
        }
        usart_note_request_reply_fail_reason(usart, fail_reason);
    }

    usart->request_cmd_id = 0U;
    usart->request_servo_id = 0U;
    usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_NONE;
    taskEXIT_CRITICAL();
}

void Usart_GetRuntimeStats(Usart_DataTypeDef *usart, UsartRuntimeStats *stats)
{
    if ((usart == NULL) || (stats == NULL)) {
        return;
    }

    taskENTER_CRITICAL();
    stats->rx_ring_overflow_count = usart->rx_ring_overflow_count;
    stats->stale_rx_drop_count = usart->stale_rx_drop_count;
    stats->transaction_timeout_count = usart->transaction_timeout_count;
    stats->tx_error_count = usart->tx_error_count;
    stats->tx_dma_start_busy_count = usart->tx_dma_start_busy_count;
    stats->tx_dma_start_fail_count = usart->tx_dma_start_fail_count;
    stats->tx_done_irq_count = usart->tx_done_irq_count;
    stats->tx_done_timeout_count = usart->tx_done_timeout_count;
    stats->tx_tc_timeout_count = usart->tx_tc_timeout_count;
    stats->uart_dma_error_count = usart->uart_dma_error_count;
    stats->uart_line_error_count = usart->uart_line_error_count;
    stats->timeout_read_angle_count = usart->timeout_read_angle_count;
    stats->timeout_monitor_count = usart->timeout_monitor_count;
    stats->timeout_sync_command_count = usart->timeout_sync_command_count;
    stats->timeout_other_count = usart->timeout_other_count;
    stats->request_reply_success_count = usart->request_reply_success_count;
    stats->request_reply_fail_count = usart->request_reply_fail_count;
    stats->request_reply_last_us = usart->request_reply_last_us;
    stats->request_reply_avg_us = usart->request_reply_avg_us;
    stats->request_reply_max_us = usart->request_reply_max_us;
    stats->request_reply_fail_timeout_count = usart->request_reply_fail_timeout_count;
    stats->request_reply_fail_send_fail_count = usart->request_reply_fail_send_fail_count;
    stats->request_reply_fail_tx_error_event_count = usart->request_reply_fail_tx_error_event_count;
    stats->request_reply_fail_bad_packet_count = usart->request_reply_fail_bad_packet_count;
    stats->request_reply_fail_reply_mismatch_count = usart->request_reply_fail_reply_mismatch_count;
    stats->request_reply_fail_reply_incomplete_count = usart->request_reply_fail_reply_incomplete_count;
    taskEXIT_CRITICAL();
}

HAL_StatusTypeDef Usart_SendAll(Usart_DataTypeDef *usart)
{
    HAL_StatusTypeDef status;
    uint16_t bytes_to_send;
    uint32_t events = 0U;
    TickType_t timeout_ticks = pdMS_TO_TICKS(FSUS_TRANSACTION_TIMEOUT_MS);

    if ((usart == NULL) || (usart->huartX == NULL) || (usart->sendBuf == NULL)) {
        return HAL_ERROR;
    }

    if (timeout_ticks == 0U) {
        timeout_ticks = 1U;
    }

    bytes_to_send = usart_drain_send_buffer(usart);
    if (bytes_to_send == 0U) {
        return HAL_OK;
    }

    usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_SEND_FAIL;

    usart_set_tx_mode(usart);

    if ((usart->huartX->Instance == USART6) && (usart->huartX->hdmatx != NULL)) {
        status = HAL_UART_Transmit_DMA(usart->huartX, usart->tx_buffer, bytes_to_send);
        if (status != HAL_OK) {
            usart_note_tx_dma_start_failure(usart, status);
            usart->tx_error_count++;
            usart_set_rx_mode(usart);
            return status;
        }

        if (Usart_WaitForNotification(timeout_ticks, &events) != pdTRUE) {
            usart->transaction_timeout_count++;
            usart->tx_done_timeout_count++;
            usart->tx_error_count++;
            Usart_NoteRequestTimeout(usart);
            usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_TIMEOUT;
            (void)HAL_UART_AbortTransmit(usart->huartX);
            usart_set_rx_mode(usart);
            return HAL_TIMEOUT;
        }

        if ((events & FSUS_USART_NOTIFY_TX_ERROR) != 0U) {
            usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_TX_ERROR_EVENT;
            usart_set_rx_mode(usart);
            return HAL_ERROR;
        }

        status = usart_wait_for_tc_flag(usart, FSUS_TRANSACTION_TIMEOUT_MS);
        if (status != HAL_OK) {
            usart->transaction_timeout_count++;
            usart->tx_tc_timeout_count++;
            usart->tx_error_count++;
            Usart_NoteRequestTimeout(usart);
            usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_TIMEOUT;
            (void)HAL_UART_AbortTransmit(usart->huartX);
            usart_set_rx_mode(usart);
            return status;
        }

        usart_set_rx_mode(usart);
        return HAL_OK;
    }

    status = HAL_UART_Transmit(usart->huartX, usart->tx_buffer, bytes_to_send, FSUS_TRANSACTION_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = usart_wait_for_tc_flag(usart, FSUS_TRANSACTION_TIMEOUT_MS);
    } else {
        usart->tx_error_count++;
        usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_SEND_FAIL;
    }

    if (status == HAL_TIMEOUT) {
        usart->transaction_timeout_count++;
        Usart_NoteRequestTimeout(usart);
        usart->last_send_fail_reason = USART_REQ_REPLY_FAIL_TIMEOUT;
    }

    usart_set_rx_mode(usart);
    return status;
}

void User_RxCpltCallback(void)
{
    BaseType_t task_woken = pdFALSE;

    usart_push_rx_byte_from_isr(&FSUS_Usart, rc1, &task_woken);
    HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
}

void User_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t task_woken = pdFALSE;

    if ((huart == NULL) || (FSUS_Usart.huartX == NULL)) {
        return;
    }

    if (huart->Instance != FSUS_Usart.huartX->Instance) {
        return;
    }

    FSUS_Usart.tx_done_irq_count++;
    usart_notify_waiter_from_isr(&FSUS_Usart, FSUS_USART_NOTIFY_TX_DONE, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void User_UartErrorCallback(UART_HandleTypeDef *huart)
{
    BaseType_t task_woken = pdFALSE;

    if ((huart == NULL) || (FSUS_Usart.huartX == NULL)) {
        return;
    }

    if (huart->Instance != FSUS_Usart.huartX->Instance) {
        return;
    }

    usart_note_uart_error(&FSUS_Usart, huart->ErrorCode);
    FSUS_Usart.tx_error_count++;
    usart_notify_waiter_from_isr(&FSUS_Usart, FSUS_USART_NOTIFY_TX_ERROR, &task_woken);
    (void)HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
    portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t task_woken = pdFALSE;

    if ((huart == NULL) || (FSUS_Usart.huartX == NULL)) {
        return;
    }

    if (huart->Instance != FSUS_Usart.huartX->Instance) {
        return;
    }

    usart_push_rx_byte_from_isr(&FSUS_Usart, rc1, &task_woken);
    HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
    portYIELD_FROM_ISR(task_woken);
}
