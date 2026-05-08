#ifndef __ARM_SHELL_TRANSPORT_H
#define __ARM_SHELL_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdint.h>

typedef enum {
    ARM_SHELL_TX_PRIORITY_STREAM = 0,
    ARM_SHELL_TX_PRIORITY_CTRL = 1,
} ArmShellTxPriority;

typedef struct {
    uint32_t rx_frame_ok_count;
    uint32_t rx_overflow_count;
    uint32_t rx_uart_error_count;
    uint32_t tx_dma_complete_count;
    uint32_t tx_ctrl_drop_count;
    uint32_t tx_stream_drop_count;
    uint32_t tx_truncate_count;
    uint32_t tx_dma_error_count;
    uint32_t tx_reserve_cancel_count;
    uint32_t tx_diag_emit_count;
} ArmShellTransportStats;

int arm_shell_transport_init(void);
int arm_shell_transport_submit(ArmShellTxPriority priority, const char *format, ...);
int arm_shell_transport_submit_v(ArmShellTxPriority priority, const char *format, va_list args);
int arm_shell_transport_submit_prefixed_v(
    ArmShellTxPriority priority,
    const char *prefix,
    const char *format,
    va_list args
);
void arm_shell_transport_note_rx_overflow(void);
void arm_shell_transport_note_rx_uart_error(void);
void arm_shell_transport_note_rx_frame_ok(void);
void arm_shell_transport_get_stats(ArmShellTransportStats *stats);

#ifdef __cplusplus
}
#endif

#endif
