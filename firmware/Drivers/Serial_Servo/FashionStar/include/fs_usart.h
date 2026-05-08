#ifndef __FS_UART_H__
#define __FS_UART_H__

#include "FreeRTOS.h"
#include "ring_buffer.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include <stdint.h>

#define USART_RECV_BUF_SIZE 500
#define USART_SEND_BUF_SIZE 500
#define FSUS_TRANSACTION_TIMEOUT_MS 10U
#define FSUS_TRANSACTION_GAP_MS 5U
#define FSUS_TIMEOUT_SERVO_BUCKETS 10U

#define FSUS_USART_NOTIFY_RX       (1UL << 0)
#define FSUS_USART_NOTIFY_TX_DONE  (1UL << 1)
#define FSUS_USART_NOTIFY_TX_ERROR (1UL << 2)
#define FSUS_USART_NOTIFY_ALL      (FSUS_USART_NOTIFY_RX | FSUS_USART_NOTIFY_TX_DONE | FSUS_USART_NOTIFY_TX_ERROR)

typedef enum
{
    USART_REQ_REPLY_FAIL_NONE = 0,
    USART_REQ_REPLY_FAIL_TIMEOUT,
    USART_REQ_REPLY_FAIL_SEND_FAIL,
    USART_REQ_REPLY_FAIL_TX_ERROR_EVENT,
    USART_REQ_REPLY_FAIL_BAD_PACKET,
    USART_REQ_REPLY_FAIL_REPLY_MISMATCH,
    USART_REQ_REPLY_FAIL_REPLY_INCOMPLETE,
} UsartRequestReplyFailReason;

typedef struct
{
    uint32_t rx_ring_overflow_count;
    uint32_t stale_rx_drop_count;
    uint32_t transaction_timeout_count;
    uint32_t tx_error_count;
    uint32_t tx_dma_start_busy_count;
    uint32_t tx_dma_start_fail_count;
    uint32_t tx_done_irq_count;
    uint32_t tx_done_timeout_count;
    uint32_t tx_tc_timeout_count;
    uint32_t uart_dma_error_count;
    uint32_t uart_line_error_count;
    uint32_t timeout_read_angle_count;
    uint32_t timeout_monitor_count;
    uint32_t timeout_sync_command_count;
    uint32_t timeout_other_count;
    uint32_t request_reply_success_count;
    uint32_t request_reply_fail_count;
    uint32_t request_reply_last_us;
    uint32_t request_reply_avg_us;
    uint32_t request_reply_max_us;
    uint32_t request_reply_fail_timeout_count;
    uint32_t request_reply_fail_send_fail_count;
    uint32_t request_reply_fail_tx_error_event_count;
    uint32_t request_reply_fail_bad_packet_count;
    uint32_t request_reply_fail_reply_mismatch_count;
    uint32_t request_reply_fail_reply_incomplete_count;
} UsartRuntimeStats;

typedef struct
{
    UART_HandleTypeDef *huartX;
    RingBufferTypeDef *sendBuf;
    RingBufferTypeDef *recvBuf;
    SemaphoreHandle_t transaction_mutex;
    TaskHandle_t waiting_task;
    TaskHandle_t owner_task;
    uint16_t transaction_depth;
    volatile uint32_t rx_ring_overflow_count;
    volatile uint32_t stale_rx_drop_count;
    volatile uint32_t transaction_timeout_count;
    volatile uint32_t tx_error_count;
    volatile uint32_t tx_dma_start_busy_count;
    volatile uint32_t tx_dma_start_fail_count;
    volatile uint32_t tx_done_irq_count;
    volatile uint32_t tx_done_timeout_count;
    volatile uint32_t tx_tc_timeout_count;
    volatile uint32_t uart_dma_error_count;
    volatile uint32_t uart_line_error_count;
    volatile uint32_t timeout_read_angle_count;
    volatile uint32_t timeout_monitor_count;
    volatile uint32_t timeout_sync_command_count;
    volatile uint32_t timeout_other_count;
    uint32_t request_reply_start_cycles;
    volatile uint8_t request_cmd_id;
    volatile uint8_t request_servo_id;
    uint8_t request_reply_pending;
    volatile uint8_t last_send_fail_reason;
    volatile uint32_t request_reply_success_count;
    volatile uint32_t request_reply_fail_count;
    volatile uint32_t request_reply_last_us;
    volatile uint32_t request_reply_avg_us;
    volatile uint32_t request_reply_max_us;
    volatile uint32_t request_reply_fail_timeout_count;
    volatile uint32_t request_reply_fail_send_fail_count;
    volatile uint32_t request_reply_fail_tx_error_event_count;
    volatile uint32_t request_reply_fail_bad_packet_count;
    volatile uint32_t request_reply_fail_reply_mismatch_count;
    volatile uint32_t request_reply_fail_reply_incomplete_count;
    uint32_t last_transaction_complete_tick_ms;
    uint8_t last_transaction_complete_valid;
    uint8_t tx_buffer[USART_SEND_BUF_SIZE];
} Usart_DataTypeDef;

extern Usart_DataTypeDef FSUS_Usart;

void User_Uart_Init(UART_HandleTypeDef *huartx);
BaseType_t Usart_BeginTransaction(Usart_DataTypeDef *usart, TickType_t timeout_ticks);
void Usart_EndTransaction(Usart_DataTypeDef *usart);
void Usart_ResetRxBuffer(Usart_DataTypeDef *usart, BaseType_t count_stale_bytes);
BaseType_t Usart_WaitForNotification(uint32_t timeout_ticks, uint32_t *events);
HAL_StatusTypeDef Usart_SendAll(Usart_DataTypeDef *usart);
void Usart_NoteRequestStart(Usart_DataTypeDef *usart, uint8_t cmd_id, uint8_t servo_id);
void Usart_NoteRequestTimeout(Usart_DataTypeDef *usart);
void Usart_NoteRequestReplyResult(
    Usart_DataTypeDef *usart,
    BaseType_t success,
    UsartRequestReplyFailReason fail_reason
);
void Usart_GetRuntimeStats(Usart_DataTypeDef *usart, UsartRuntimeStats *stats);
void User_RxCpltCallback(void);
void User_TxCpltCallback(UART_HandleTypeDef *huart);
void User_UartErrorCallback(UART_HandleTypeDef *huart);
#endif
