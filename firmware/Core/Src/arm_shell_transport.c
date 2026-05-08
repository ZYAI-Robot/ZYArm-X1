#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "arm_shell.h"
#include "arm_shell_transport.h"
#include "fs_usart.h"
#include "task.h"
#include "usart.h"

#define ARM_SHELL_TX_MAX_MSG_LEN 256U
#define ARM_SHELL_TX_FORMAT_BUFFER_LEN (ARM_SHELL_TX_MAX_MSG_LEN + 1U)
#define ARM_SHELL_TX_DIAG_RATE_LIMIT_MS 1000U
#define ARM_SHELL_TX_POLL_TIMEOUT_MS 100U
#define ARM_SHELL_TX_NO_DESC 0xFFFFU
#define ARM_TRANSPORT_LOG_TAG "TRANSPORT"

#define ARM_SHELL_TX_CTRL_DESC_COUNT 8U
#define ARM_SHELL_TX_CTRL_DATA_BYTES 2048U
#define ARM_SHELL_TX_STREAM_DESC_COUNT 48U
#define ARM_SHELL_TX_STREAM_DATA_BYTES 6144U

typedef enum {
    ARM_SHELL_TX_ACTIVE_NONE = 0,
    ARM_SHELL_TX_ACTIVE_CTRL_DESC = 1,
    ARM_SHELL_TX_ACTIVE_STREAM_DESC = 2,
} ArmShellTxActiveKind;

typedef enum {
    ARM_SHELL_TX_DESC_FREE = 0,
    ARM_SHELL_TX_DESC_RESERVED = 1,
    ARM_SHELL_TX_DESC_COMMITTED = 2,
    ARM_SHELL_TX_DESC_INFLIGHT = 3,
    ARM_SHELL_TX_DESC_SENT = 4,
    ARM_SHELL_TX_DESC_CANCELED = 5,
} ArmShellTxDescState;

typedef struct {
    uint16_t offset;
    uint16_t capacity;
    uint16_t len;
    uint8_t state;
    uint32_t seq;
} ArmShellTxDesc;

typedef struct {
    uint16_t desc_index;
    uint16_t offset;
    uint16_t len;
} ArmShellTxReservation;

typedef struct {
    ArmShellTxPriority priority;
    ArmShellTxDesc *desc;
    char *data;
    uint16_t desc_count;
    uint16_t data_bytes;

    volatile uint16_t alloc_head;
    volatile uint16_t alloc_desc_head;
    volatile uint16_t commit_head;
    volatile uint16_t tx_head;
    volatile uint16_t reclaim_tail;
    volatile uint16_t used_bytes;
    volatile uint16_t used_desc_count;
    volatile uint8_t wrapped;
    volatile uint32_t next_seq;
} ArmShellTxLane;

typedef struct {
    volatile uint8_t initialized;
    volatile uint8_t dma_active;
    volatile uint8_t current_kind;
    volatile uint16_t current_desc;

    volatile uint32_t pending_ctrl_drop_count;
    volatile uint32_t pending_stream_drop_count;
    volatile uint32_t pending_dma_error_count;
    volatile uint32_t pending_rx_overflow_count;
    volatile uint32_t pending_rx_uart_error_count;
    volatile uint32_t pending_reserve_cancel_count;
    volatile uint32_t pending_truncate_count;

    TickType_t last_diag_tick;
    ArmShellTransportStats stats;
    ArmShellTxLane ctrl_lane;
    ArmShellTxLane stream_lane;
} ArmShellTxRuntime;

static ArmShellTxDesc g_arm_shell_tx_ctrl_desc[ARM_SHELL_TX_CTRL_DESC_COUNT] = {0};
static char g_arm_shell_tx_ctrl_data[ARM_SHELL_TX_CTRL_DATA_BYTES] = {0};
static ArmShellTxDesc g_arm_shell_tx_stream_desc[ARM_SHELL_TX_STREAM_DESC_COUNT] = {0};
static char g_arm_shell_tx_stream_data[ARM_SHELL_TX_STREAM_DATA_BYTES] = {0};

static ArmShellTxRuntime g_arm_shell_tx = {
    .initialized = 0U,
    .dma_active = 0U,
    .current_kind = ARM_SHELL_TX_ACTIVE_NONE,
    .current_desc = ARM_SHELL_TX_NO_DESC,
};

static void arm_shell_transport_kick(void);
static void arm_shell_transport_kick_from_isr(BaseType_t *task_woken);
static void arm_shell_transport_try_emit_diag(void);

static inline int arm_shell_uart_polling_write(const char *buffer, uint16_t len)
{
    if ((buffer == NULL) || (len == 0U)) {
        return 0;
    }

    return (HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, ARM_SHELL_TX_POLL_TIMEOUT_MS) == HAL_OK) ? (int)len : -1;
}

static inline int arm_shell_is_exception_context(void)
{
    return (__get_IPSR() != 0U) ? 1 : 0;
}

static inline int arm_shell_can_use_async_tx(void)
{
    if (g_arm_shell_tx.initialized == 0U) {
        return 0;
    }

    if (arm_shell_is_exception_context()) {
        return 0;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return 0;
    }

    return 1;
}

static inline ArmShellTxLane *arm_shell_transport_lane_from_priority(ArmShellTxPriority priority)
{
    if (priority == ARM_SHELL_TX_PRIORITY_CTRL) {
        return &g_arm_shell_tx.ctrl_lane;
    }

    return &g_arm_shell_tx.stream_lane;
}

static inline ArmShellTxLane *arm_shell_transport_lane_from_active_kind(uint8_t kind)
{
    if (kind == ARM_SHELL_TX_ACTIVE_CTRL_DESC) {
        return &g_arm_shell_tx.ctrl_lane;
    }

    if (kind == ARM_SHELL_TX_ACTIVE_STREAM_DESC) {
        return &g_arm_shell_tx.stream_lane;
    }

    return NULL;
}

static inline uint8_t arm_shell_transport_active_kind_from_priority(ArmShellTxPriority priority)
{
    return (priority == ARM_SHELL_TX_PRIORITY_CTRL) ? ARM_SHELL_TX_ACTIVE_CTRL_DESC : ARM_SHELL_TX_ACTIVE_STREAM_DESC;
}

static inline void arm_shell_transport_note_drop(ArmShellTxPriority priority)
{
    if (priority == ARM_SHELL_TX_PRIORITY_CTRL) {
        g_arm_shell_tx.stats.tx_ctrl_drop_count++;
        g_arm_shell_tx.pending_ctrl_drop_count++;
        return;
    }

    g_arm_shell_tx.stats.tx_stream_drop_count++;
    g_arm_shell_tx.pending_stream_drop_count++;
}

static inline void arm_shell_transport_note_dma_error(void)
{
    g_arm_shell_tx.stats.tx_dma_error_count++;
    g_arm_shell_tx.pending_dma_error_count++;
}

static inline void arm_shell_transport_note_reserve_cancel(void)
{
    g_arm_shell_tx.stats.tx_reserve_cancel_count++;
    g_arm_shell_tx.pending_reserve_cancel_count++;
}

static inline void arm_shell_transport_note_truncate(void)
{
    g_arm_shell_tx.stats.tx_truncate_count++;
    g_arm_shell_tx.pending_truncate_count++;
}

static inline uint16_t arm_shell_lane_next_desc_index(const ArmShellTxLane *lane, uint16_t index)
{
    return (uint16_t)((index + 1U) % lane->desc_count);
}

static inline int arm_shell_lane_state_is_reclaimable(uint8_t state)
{
    return (state == ARM_SHELL_TX_DESC_SENT) || (state == ARM_SHELL_TX_DESC_CANCELED);
}

static uint16_t arm_shell_lane_oldest_offset_locked(const ArmShellTxLane *lane)
{
    if (lane->used_desc_count == 0U) {
        return 0U;
    }

    return lane->desc[lane->reclaim_tail].offset;
}

static int arm_shell_lane_has_free_desc_locked(const ArmShellTxLane *lane)
{
    return (lane->used_desc_count < lane->desc_count) ? 1 : 0;
}

static void arm_shell_lane_refresh_wrap_locked(ArmShellTxLane *lane)
{
    if (lane->used_desc_count == 0U) {
        lane->wrapped = 0U;
        return;
    }

    /* After a head-wrap, the tail slack stays unavailable until all older tail messages drain. */
    if ((lane->wrapped != 0U) &&
        (arm_shell_lane_oldest_offset_locked(lane) < lane->alloc_head)) {
        lane->wrapped = 0U;
    }
}

static void arm_shell_lane_sync_commit_head_locked(ArmShellTxLane *lane)
{
    uint16_t remaining = lane->used_desc_count;

    while (remaining > 0U) {
        uint8_t state = lane->desc[lane->commit_head].state;

        if ((state == ARM_SHELL_TX_DESC_RESERVED) ||
            (state == ARM_SHELL_TX_DESC_FREE)) {
            break;
        }

        lane->commit_head = arm_shell_lane_next_desc_index(lane, lane->commit_head);
        remaining--;
    }
}

static void arm_shell_lane_advance_tx_head_locked(ArmShellTxLane *lane)
{
    uint16_t remaining = lane->used_desc_count;

    if (remaining == 0U) {
        lane->tx_head = lane->alloc_desc_head;
        return;
    }

    while (remaining > 0U) {
        uint8_t state = lane->desc[lane->tx_head].state;

        if ((state == ARM_SHELL_TX_DESC_RESERVED) ||
            (state == ARM_SHELL_TX_DESC_COMMITTED) ||
            (state == ARM_SHELL_TX_DESC_INFLIGHT)) {
            return;
        }

        if (state == ARM_SHELL_TX_DESC_FREE) {
            return;
        }

        lane->tx_head = arm_shell_lane_next_desc_index(lane, lane->tx_head);
        remaining--;
    }

    lane->tx_head = lane->alloc_desc_head;
}

static void arm_shell_lane_reset_if_empty_locked(ArmShellTxLane *lane)
{
    if ((lane->used_desc_count != 0U) || (lane->used_bytes != 0U)) {
        return;
    }

    lane->used_bytes = 0U;
    lane->alloc_head = 0U;
    lane->wrapped = 0U;
    lane->reclaim_tail = lane->alloc_desc_head;
    lane->tx_head = lane->alloc_desc_head;
    lane->commit_head = lane->alloc_desc_head;
}

static void arm_shell_lane_reclaim_locked(ArmShellTxLane *lane)
{
    while (lane->used_desc_count != 0U) {
        ArmShellTxDesc *desc = &lane->desc[lane->reclaim_tail];

        if (!arm_shell_lane_state_is_reclaimable(desc->state)) {
            break;
        }

        if (lane->used_bytes >= desc->capacity) {
            lane->used_bytes = (uint16_t)(lane->used_bytes - desc->capacity);
        } else {
            lane->used_bytes = 0U;
        }

        memset(desc, 0, sizeof(*desc));
        desc->state = ARM_SHELL_TX_DESC_FREE;
        lane->used_desc_count--;
        lane->reclaim_tail = arm_shell_lane_next_desc_index(lane, lane->reclaim_tail);
    }

    arm_shell_lane_reset_if_empty_locked(lane);
    arm_shell_lane_refresh_wrap_locked(lane);
    arm_shell_lane_advance_tx_head_locked(lane);
}

static int arm_shell_lane_reserve_locked(
    ArmShellTxLane *lane,
    uint16_t len,
    ArmShellTxReservation *reservation
)
{
    uint16_t offset = 0U;
    uint8_t was_empty;
    ArmShellTxDesc *desc;

    if ((lane == NULL) || (reservation == NULL) || (len == 0U) || (len > lane->data_bytes)) {
        return -1;
    }

    memset(reservation, 0, sizeof(*reservation));
    arm_shell_lane_reclaim_locked(lane);

    if (!arm_shell_lane_has_free_desc_locked(lane)) {
        arm_shell_transport_note_drop(lane->priority);
        return -1;
    }

    was_empty = (lane->used_desc_count == 0U) ? 1U : 0U;
    if (was_empty != 0U) {
        lane->alloc_head = 0U;
        lane->wrapped = 0U;
    }

    if (was_empty == 0U) {
        uint16_t oldest_offset = arm_shell_lane_oldest_offset_locked(lane);

        offset = lane->alloc_head;
        if (lane->wrapped != 0U) {
            if ((oldest_offset <= lane->alloc_head) ||
                ((uint16_t)(oldest_offset - lane->alloc_head) < len)) {
                arm_shell_transport_note_drop(lane->priority);
                return -1;
            }
        } else {
            uint16_t tail_free = (uint16_t)(lane->data_bytes - lane->alloc_head);

            if (tail_free < len) {
                if (oldest_offset < len) {
                    arm_shell_transport_note_drop(lane->priority);
                    return -1;
                }
                lane->alloc_head = 0U;
                lane->wrapped = 1U;
                offset = 0U;
            }
        }
    }

    desc = &lane->desc[lane->alloc_desc_head];
    if (desc->state != ARM_SHELL_TX_DESC_FREE) {
        arm_shell_transport_note_drop(lane->priority);
        return -1;
    }

    offset = lane->alloc_head;
    memset(desc, 0, sizeof(*desc));
    desc->offset = offset;
    desc->capacity = len;
    desc->len = len;
    desc->state = ARM_SHELL_TX_DESC_RESERVED;
    desc->seq = lane->next_seq++;

    if (was_empty != 0U) {
        lane->reclaim_tail = lane->alloc_desc_head;
        lane->tx_head = lane->alloc_desc_head;
        lane->commit_head = lane->alloc_desc_head;
    }

    lane->used_bytes = (uint16_t)(lane->used_bytes + len);
    lane->used_desc_count++;
    lane->alloc_head = (uint16_t)((offset + len) % lane->data_bytes);
    reservation->desc_index = lane->alloc_desc_head;
    reservation->offset = offset;
    reservation->len = len;
    lane->alloc_desc_head = arm_shell_lane_next_desc_index(lane, lane->alloc_desc_head);
    return 0;
}

static void arm_shell_lane_commit_locked(ArmShellTxLane *lane, uint16_t desc_index)
{
    ArmShellTxDesc *desc = &lane->desc[desc_index];

    if (desc->state != ARM_SHELL_TX_DESC_RESERVED) {
        return;
    }

    desc->state = ARM_SHELL_TX_DESC_COMMITTED;
    arm_shell_lane_sync_commit_head_locked(lane);
}

static void arm_shell_lane_cancel_locked(ArmShellTxLane *lane, uint16_t desc_index)
{
    ArmShellTxDesc *desc = &lane->desc[desc_index];

    if (desc->state != ARM_SHELL_TX_DESC_RESERVED) {
        return;
    }

    desc->state = ARM_SHELL_TX_DESC_CANCELED;
    arm_shell_transport_note_reserve_cancel();
    arm_shell_lane_sync_commit_head_locked(lane);
    arm_shell_lane_reclaim_locked(lane);
}

static void arm_shell_lane_abort_locked(ArmShellTxLane *lane, uint16_t desc_index)
{
    ArmShellTxDesc *desc;

    if ((lane == NULL) || (desc_index >= lane->desc_count)) {
        return;
    }

    desc = &lane->desc[desc_index];
    if ((desc->state != ARM_SHELL_TX_DESC_RESERVED) &&
        (desc->state != ARM_SHELL_TX_DESC_COMMITTED) &&
        (desc->state != ARM_SHELL_TX_DESC_INFLIGHT)) {
        return;
    }

    desc->state = ARM_SHELL_TX_DESC_CANCELED;
    arm_shell_lane_sync_commit_head_locked(lane);
    arm_shell_lane_reclaim_locked(lane);
}

static BaseType_t arm_shell_lane_prepare_next_desc_locked(ArmShellTxLane *lane, uint16_t *desc_index)
{
    ArmShellTxDesc *desc;

    if ((lane == NULL) || (desc_index == NULL)) {
        return pdFALSE;
    }

    arm_shell_lane_reclaim_locked(lane);
    arm_shell_lane_advance_tx_head_locked(lane);
    if (lane->used_desc_count == 0U) {
        return pdFALSE;
    }

    desc = &lane->desc[lane->tx_head];
    if (desc->state != ARM_SHELL_TX_DESC_COMMITTED) {
        return pdFALSE;
    }

    desc->state = ARM_SHELL_TX_DESC_INFLIGHT;
    *desc_index = lane->tx_head;
    return pdTRUE;
}

static void arm_shell_lane_mark_sent_locked(ArmShellTxLane *lane, uint16_t desc_index)
{
    if ((lane == NULL) || (desc_index >= lane->desc_count)) {
        return;
    }

    if (lane->desc[desc_index].state == ARM_SHELL_TX_DESC_INFLIGHT) {
        lane->desc[desc_index].state = ARM_SHELL_TX_DESC_SENT;
        if (lane->tx_head == desc_index) {
            lane->tx_head = arm_shell_lane_next_desc_index(lane, lane->tx_head);
        }
        arm_shell_lane_reclaim_locked(lane);
    }
}

static int arm_shell_transport_start_lane_desc(ArmShellTxPriority priority, uint16_t desc_index)
{
    ArmShellTxLane *lane = arm_shell_transport_lane_from_priority(priority);
    ArmShellTxDesc *desc;

    if ((lane == NULL) || (desc_index >= lane->desc_count)) {
        return -1;
    }

    desc = &lane->desc[desc_index];
    if ((desc->state != ARM_SHELL_TX_DESC_INFLIGHT) || (desc->len == 0U)) {
        return -1;
    }

    g_arm_shell_tx.current_kind = arm_shell_transport_active_kind_from_priority(priority);
    g_arm_shell_tx.current_desc = desc_index;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&lane->data[desc->offset], desc->len) != HAL_OK) {
        g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
        g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
        return -1;
    }

    return 0;
}

static uint16_t arm_shell_transport_finalize_formatted_len(char *buffer, size_t size, int raw_len)
{
    uint16_t len;

    if (raw_len <= 0) {
        return 0U;
    }

    len = (raw_len >= (int)size) ? (uint16_t)(size - 1U) : (uint16_t)raw_len;
    if ((raw_len >= (int)size) && (len > 0U)) {
        if (buffer[len - 1U] != '\n') {
            buffer[len - 1U] = '\n';
        }
        arm_shell_transport_note_truncate();
    }

    return len;
}

static uint16_t arm_shell_transport_bound_message(
    const char *buffer,
    uint16_t len,
    char scratch[ARM_SHELL_TX_FORMAT_BUFFER_LEN]
)
{
    if ((buffer == NULL) || (len == 0U)) {
        return 0U;
    }

    if (len <= ARM_SHELL_TX_MAX_MSG_LEN) {
        return len;
    }

    memcpy(scratch, buffer, ARM_SHELL_TX_MAX_MSG_LEN);
    if (scratch[ARM_SHELL_TX_MAX_MSG_LEN - 1U] != '\n') {
        scratch[ARM_SHELL_TX_MAX_MSG_LEN - 1U] = '\n';
    }
    scratch[ARM_SHELL_TX_MAX_MSG_LEN] = '\0';
    arm_shell_transport_note_truncate();
    return ARM_SHELL_TX_MAX_MSG_LEN;
}

static int arm_shell_transport_format_message_v(
    char buffer[ARM_SHELL_TX_FORMAT_BUFFER_LEN],
    const char *format,
    va_list args
)
{
    int raw_len;

    if (format == NULL) {
        return -1;
    }

    raw_len = vsnprintf(buffer, ARM_SHELL_TX_FORMAT_BUFFER_LEN, format, args);
    if (raw_len <= 0) {
        return raw_len;
    }

    return (int)arm_shell_transport_finalize_formatted_len(buffer, ARM_SHELL_TX_FORMAT_BUFFER_LEN, raw_len);
}

static int arm_shell_transport_format_prefixed_message_v(
    char buffer[ARM_SHELL_TX_FORMAT_BUFFER_LEN],
    const char *prefix,
    const char *format,
    va_list args
)
{
    int prefix_len;
    int content_len;
    size_t remaining;
    uint16_t total_len;

    if (format == NULL) {
        return -1;
    }

    if (prefix == NULL) {
        prefix = "";
    }

    prefix_len = snprintf(buffer, ARM_SHELL_TX_FORMAT_BUFFER_LEN, "%s", prefix);
    if (prefix_len < 0) {
        return prefix_len;
    }

    total_len = arm_shell_transport_finalize_formatted_len(buffer, ARM_SHELL_TX_FORMAT_BUFFER_LEN, prefix_len);
    if ((uint16_t)prefix_len >= ARM_SHELL_TX_FORMAT_BUFFER_LEN) {
        return (int)total_len;
    }

    remaining = ARM_SHELL_TX_FORMAT_BUFFER_LEN - (size_t)total_len;
    content_len = vsnprintf(buffer + total_len, remaining, format, args);
    if (content_len < 0) {
        return content_len;
    }

    if ((size_t)content_len >= remaining) {
        total_len = ARM_SHELL_TX_MAX_MSG_LEN;
        if (buffer[total_len - 1U] != '\n') {
            buffer[total_len - 1U] = '\n';
        }
        arm_shell_transport_note_truncate();
        return (int)total_len;
    }

    total_len = (uint16_t)(total_len + content_len);
    return (int)total_len;
}

static int arm_shell_transport_submit_buffer(
    ArmShellTxPriority priority,
    const char *buffer,
    uint16_t len,
    int allow_diag_flush
)
{
    ArmShellTxLane *lane;
    ArmShellTxReservation reservation = {0};
    char scratch[ARM_SHELL_TX_FORMAT_BUFFER_LEN];
    const char *submit_buffer = buffer;
    uint16_t submit_len = len;

    if ((buffer == NULL) || (len == 0U)) {
        return 0;
    }

    if (len > ARM_SHELL_TX_MAX_MSG_LEN) {
        submit_buffer = scratch;
        submit_len = arm_shell_transport_bound_message(buffer, len, scratch);
    }

    if (submit_len == 0U) {
        return 0;
    }

    if (!arm_shell_can_use_async_tx()) {
        return arm_shell_uart_polling_write(submit_buffer, submit_len);
    }

    lane = arm_shell_transport_lane_from_priority(priority);
    if (lane == NULL) {
        return -1;
    }

    taskENTER_CRITICAL();
    if (arm_shell_lane_reserve_locked(lane, submit_len, &reservation) != 0) {
        taskEXIT_CRITICAL();
        return -1;
    }
    taskEXIT_CRITICAL();

    memcpy(&lane->data[reservation.offset], submit_buffer, reservation.len);

    taskENTER_CRITICAL();
    arm_shell_lane_commit_locked(lane, reservation.desc_index);
    taskEXIT_CRITICAL();

    arm_shell_transport_kick();
    if (allow_diag_flush != 0) {
        arm_shell_transport_try_emit_diag();
    }
    return (int)reservation.len;
}

static void arm_shell_transport_try_emit_diag(void)
{
    char diag_buffer[ARM_SHELL_TX_FORMAT_BUFFER_LEN];
    char diag_prefix[32];
    TickType_t now;
    uint32_t ctrl_drop;
    uint32_t stream_drop;
    uint32_t dma_error;
    uint32_t rx_overflow;
    uint32_t rx_uart_error;
    uint32_t reserve_cancel;
    uint32_t truncate_count;
    int len;

    if (!arm_shell_can_use_async_tx()) {
        return;
    }

    ctrl_drop = g_arm_shell_tx.pending_ctrl_drop_count;
    stream_drop = g_arm_shell_tx.pending_stream_drop_count;
    dma_error = g_arm_shell_tx.pending_dma_error_count;
    rx_overflow = g_arm_shell_tx.pending_rx_overflow_count;
    rx_uart_error = g_arm_shell_tx.pending_rx_uart_error_count;
    reserve_cancel = g_arm_shell_tx.pending_reserve_cancel_count;
    truncate_count = g_arm_shell_tx.pending_truncate_count;

    if ((ctrl_drop == 0U) && (stream_drop == 0U) &&
        (dma_error == 0U) && (rx_overflow == 0U) && (rx_uart_error == 0U) &&
        (reserve_cancel == 0U) && (truncate_count == 0U)) {
        return;
    }

    now = xTaskGetTickCount();
    if ((g_arm_shell_tx.last_diag_tick != 0U) &&
        ((now - g_arm_shell_tx.last_diag_tick) < pdMS_TO_TICKS(ARM_SHELL_TX_DIAG_RATE_LIMIT_MS))) {
        return;
    }

    (void)arm_log_format_prefix(
        diag_prefix,
        sizeof(diag_prefix),
        ARM_LOG_LEVEL_WARN,
        ARM_TRANSPORT_LOG_TAG
    );
    len = snprintf(
        diag_buffer,
        sizeof(diag_buffer),
        "%stransport_stats CTRL_DROP=%lu STREAM_DROP=%lu TRUNCATE=%lu RESERVE_CANCEL=%lu DMA_ERROR=%lu RX_OVERFLOW=%lu RX_UART_ERROR=%lu\n",
        diag_prefix,
        (unsigned long)ctrl_drop,
        (unsigned long)stream_drop,
        (unsigned long)truncate_count,
        (unsigned long)reserve_cancel,
        (unsigned long)dma_error,
        (unsigned long)rx_overflow,
        (unsigned long)rx_uart_error
    );
    if (len <= 0) {
        return;
    }

    len = (int)arm_shell_transport_finalize_formatted_len(diag_buffer, sizeof(diag_buffer), len);
    if (len <= 0) {
        return;
    }

    if (arm_shell_transport_submit_buffer(ARM_SHELL_TX_PRIORITY_STREAM, diag_buffer, (uint16_t)len, 0) < 0) {
        return;
    }

    g_arm_shell_tx.pending_ctrl_drop_count = 0U;
    g_arm_shell_tx.pending_stream_drop_count = 0U;
    g_arm_shell_tx.pending_dma_error_count = 0U;
    g_arm_shell_tx.pending_rx_overflow_count = 0U;
    g_arm_shell_tx.pending_rx_uart_error_count = 0U;
    g_arm_shell_tx.pending_reserve_cancel_count = 0U;
    g_arm_shell_tx.pending_truncate_count = 0U;
    g_arm_shell_tx.last_diag_tick = now;
    g_arm_shell_tx.stats.tx_diag_emit_count++;
}

static void arm_shell_transport_kick(void)
{
    while (1) {
        uint16_t desc_index = ARM_SHELL_TX_NO_DESC;

        taskENTER_CRITICAL();
        if (g_arm_shell_tx.dma_active != 0U) {
            taskEXIT_CRITICAL();
            return;
        }
        g_arm_shell_tx.dma_active = 1U;
        g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
        g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;

        if (arm_shell_lane_prepare_next_desc_locked(&g_arm_shell_tx.ctrl_lane, &desc_index) == pdTRUE) {
            taskEXIT_CRITICAL();
            if (arm_shell_transport_start_lane_desc(ARM_SHELL_TX_PRIORITY_CTRL, desc_index) == 0) {
                return;
            }

            taskENTER_CRITICAL();
            arm_shell_lane_abort_locked(&g_arm_shell_tx.ctrl_lane, desc_index);
            g_arm_shell_tx.dma_active = 0U;
            g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
            g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
            taskEXIT_CRITICAL();
            arm_shell_transport_note_dma_error();
            continue;
        }

        if (arm_shell_lane_prepare_next_desc_locked(&g_arm_shell_tx.stream_lane, &desc_index) == pdTRUE) {
            taskEXIT_CRITICAL();
            if (arm_shell_transport_start_lane_desc(ARM_SHELL_TX_PRIORITY_STREAM, desc_index) == 0) {
                return;
            }

            taskENTER_CRITICAL();
            arm_shell_lane_abort_locked(&g_arm_shell_tx.stream_lane, desc_index);
            g_arm_shell_tx.dma_active = 0U;
            g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
            g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
            taskEXIT_CRITICAL();
            arm_shell_transport_note_dma_error();
            continue;
        }

        g_arm_shell_tx.dma_active = 0U;
        g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
        g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
        taskEXIT_CRITICAL();
        return;
    }
}

static void arm_shell_transport_kick_from_isr(BaseType_t *task_woken)
{
    (void)task_woken;

    while (1) {
        uint16_t desc_index = ARM_SHELL_TX_NO_DESC;

        if (g_arm_shell_tx.dma_active != 0U) {
            return;
        }

        g_arm_shell_tx.dma_active = 1U;
        g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
        g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;

        if (arm_shell_lane_prepare_next_desc_locked(&g_arm_shell_tx.ctrl_lane, &desc_index) == pdTRUE) {
            if (arm_shell_transport_start_lane_desc(ARM_SHELL_TX_PRIORITY_CTRL, desc_index) == 0) {
                return;
            }

            arm_shell_lane_abort_locked(&g_arm_shell_tx.ctrl_lane, desc_index);
            g_arm_shell_tx.dma_active = 0U;
            g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
            g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
            arm_shell_transport_note_dma_error();
            continue;
        }

        if (arm_shell_lane_prepare_next_desc_locked(&g_arm_shell_tx.stream_lane, &desc_index) == pdTRUE) {
            if (arm_shell_transport_start_lane_desc(ARM_SHELL_TX_PRIORITY_STREAM, desc_index) == 0) {
                return;
            }

            arm_shell_lane_abort_locked(&g_arm_shell_tx.stream_lane, desc_index);
            g_arm_shell_tx.dma_active = 0U;
            g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
            g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
            arm_shell_transport_note_dma_error();
            continue;
        }

        g_arm_shell_tx.dma_active = 0U;
        g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
        g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
        return;
    }
}

static void arm_shell_transport_init_lane(
    ArmShellTxLane *lane,
    ArmShellTxPriority priority,
    ArmShellTxDesc *desc,
    char *data,
    uint16_t desc_count,
    uint16_t data_bytes
)
{
    memset(lane, 0, sizeof(*lane));
    lane->priority = priority;
    lane->desc = desc;
    lane->data = data;
    lane->desc_count = desc_count;
    lane->data_bytes = data_bytes;
    lane->alloc_head = 0U;
    lane->alloc_desc_head = 0U;
    lane->commit_head = 0U;
    lane->tx_head = 0U;
    lane->reclaim_tail = 0U;
    lane->used_bytes = 0U;
    lane->used_desc_count = 0U;
    lane->wrapped = 0U;
    lane->next_seq = 0U;
}

int arm_shell_transport_init(void)
{
    if (g_arm_shell_tx.initialized != 0U) {
        return 0;
    }

    memset(&g_arm_shell_tx, 0, sizeof(g_arm_shell_tx));
    memset(g_arm_shell_tx_ctrl_desc, 0, sizeof(g_arm_shell_tx_ctrl_desc));
    memset(g_arm_shell_tx_ctrl_data, 0, sizeof(g_arm_shell_tx_ctrl_data));
    memset(g_arm_shell_tx_stream_desc, 0, sizeof(g_arm_shell_tx_stream_desc));
    memset(g_arm_shell_tx_stream_data, 0, sizeof(g_arm_shell_tx_stream_data));

    arm_shell_transport_init_lane(
        &g_arm_shell_tx.ctrl_lane,
        ARM_SHELL_TX_PRIORITY_CTRL,
        g_arm_shell_tx_ctrl_desc,
        g_arm_shell_tx_ctrl_data,
        ARM_SHELL_TX_CTRL_DESC_COUNT,
        ARM_SHELL_TX_CTRL_DATA_BYTES
    );
    arm_shell_transport_init_lane(
        &g_arm_shell_tx.stream_lane,
        ARM_SHELL_TX_PRIORITY_STREAM,
        g_arm_shell_tx_stream_desc,
        g_arm_shell_tx_stream_data,
        ARM_SHELL_TX_STREAM_DESC_COUNT,
        ARM_SHELL_TX_STREAM_DATA_BYTES
    );

    g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
    g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
    g_arm_shell_tx.initialized = 1U;
    return 0;
}

int arm_shell_transport_submit(ArmShellTxPriority priority, const char *format, ...)
{
    int ret;
    va_list args;

    va_start(args, format);
    ret = arm_shell_transport_submit_v(priority, format, args);
    va_end(args);

    return ret;
}

int arm_shell_transport_submit_v(ArmShellTxPriority priority, const char *format, va_list args)
{
    char buffer[ARM_SHELL_TX_FORMAT_BUFFER_LEN];
    int len = arm_shell_transport_format_message_v(buffer, format, args);

    if (len <= 0) {
        return len;
    }

    return arm_shell_transport_submit_buffer(priority, buffer, (uint16_t)len, 1);
}

int arm_shell_transport_submit_prefixed_v(
    ArmShellTxPriority priority,
    const char *prefix,
    const char *format,
    va_list args
)
{
    char buffer[ARM_SHELL_TX_FORMAT_BUFFER_LEN];
    int len = arm_shell_transport_format_prefixed_message_v(buffer, prefix, format, args);

    if (len <= 0) {
        return len;
    }

    return arm_shell_transport_submit_buffer(priority, buffer, (uint16_t)len, 1);
}

void arm_shell_transport_note_rx_overflow(void)
{
    g_arm_shell_tx.stats.rx_overflow_count++;
    g_arm_shell_tx.pending_rx_overflow_count++;
}

void arm_shell_transport_note_rx_uart_error(void)
{
    g_arm_shell_tx.stats.rx_uart_error_count++;
    g_arm_shell_tx.pending_rx_uart_error_count++;
}

void arm_shell_transport_note_rx_frame_ok(void)
{
    g_arm_shell_tx.stats.rx_frame_ok_count++;
}

void arm_shell_transport_get_stats(ArmShellTransportStats *stats)
{
    if (stats == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *stats = g_arm_shell_tx.stats;
    taskEXIT_CRITICAL();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t task_woken = pdFALSE;
    ArmShellTxLane *lane;

    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART6) {
        User_TxCpltCallback(huart);
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }

    lane = arm_shell_transport_lane_from_active_kind(g_arm_shell_tx.current_kind);
    if ((lane != NULL) && (g_arm_shell_tx.current_desc != ARM_SHELL_TX_NO_DESC)) {
        arm_shell_lane_mark_sent_locked(lane, g_arm_shell_tx.current_desc);
        g_arm_shell_tx.stats.tx_dma_complete_count++;
    }

    g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
    g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
    g_arm_shell_tx.dma_active = 0U;

    arm_shell_transport_kick_from_isr(&task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BaseType_t task_woken = pdFALSE;
    ArmShellTxLane *lane;

    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART6) {
        User_UartErrorCallback(huart);
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }

    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U) {
        arm_shell_transport_note_dma_error();
    } else if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        arm_shell_transport_note_rx_uart_error();
    }

    lane = arm_shell_transport_lane_from_active_kind(g_arm_shell_tx.current_kind);
    if ((lane != NULL) && (g_arm_shell_tx.current_desc != ARM_SHELL_TX_NO_DESC)) {
        arm_shell_lane_abort_locked(lane, g_arm_shell_tx.current_desc);
    }

    g_arm_shell_tx.current_kind = ARM_SHELL_TX_ACTIVE_NONE;
    g_arm_shell_tx.current_desc = ARM_SHELL_TX_NO_DESC;
    g_arm_shell_tx.dma_active = 0U;

    arm_shell_transport_kick_from_isr(&task_woken);
    portYIELD_FROM_ISR(task_woken);
}
