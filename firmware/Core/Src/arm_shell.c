#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arm_robot.h"
#include "arm_shell.h"
#include "usart.h"

#define ARM_SHELL_RAW_CMD_QUEUE_LENGTH 5
#define ARM_SHELL_CMD_QUEUE_LENGTH 15
#define ARM_SHELL_RX_STALE_TIMEOUT_MS 1000U
#define ARM_SHELL_LOG_TAG "SHELL"

ArmShell g_arm_shell = {0};
static uint8_t g_loacl_raw_cmd_buf[ARM_SHELL_CMD_MAX_LEN] = {0};
static ArmShellParserStats g_arm_shell_parser_stats = {0};

static const char *arm_log_level_name(ArmLogLevel level)
{
    switch (level) {
        case ARM_LOG_LEVEL_WARN:
            return "WARN";
        case ARM_LOG_LEVEL_ERROR:
            return "ERROR";
        case ARM_LOG_LEVEL_INFO:
        default:
            return "INFO";
    }
}

size_t arm_log_format_prefix(char *buffer, size_t buffer_size, ArmLogLevel level, const char *tag)
{
    int len;
    const char *level_name = arm_log_level_name(level);

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    if ((tag != NULL) && (tag[0] != '\0')) {
        len = snprintf(buffer, buffer_size, "[%s][%s] ", level_name, tag);
    } else {
        len = snprintf(buffer, buffer_size, "[%s] ", level_name);
    }

    if (len < 0) {
        buffer[0] = '\0';
        return 0U;
    }

    if ((size_t)len >= buffer_size) {
        buffer[buffer_size - 1U] = '\0';
        return buffer_size - 1U;
    }

    return (size_t)len;
}

static int arm_log_vprintf(ArmLogLevel level, const char *tag, const char *format, va_list args)
{
    char prefix[64];

    (void)arm_log_format_prefix(prefix, sizeof(prefix), level, tag);
    return arm_shell_transport_submit_prefixed_v(ARM_SHELL_TX_PRIORITY_STREAM, prefix, format, args);
}

/**
 * 解析原始命令字符串到命令结构体
 * @param command 指向命令结构体的指针
 * @return 0表示解析成功，非0表示解析失败
 */
static int parse_raw_command(ArmShellCmdPackage *command)
{
    static char string_cmd_scanf_format[64] = {0};
    int result = 0;

    if (sscanf((char *)g_loacl_raw_cmd_buf, "[CMD][%d]", &(command->cmd_id)) < 1) {
        return -1;
    }

    if (command->cmd_id >= CMD_ID_MAX_NUM) {
        return -1;
    }

    switch (g_shell_cmd_list[command->cmd_id].format) {
        case CMD_PARSE_FORMAT_FLOAT: {
            char *cmd_pos = strstr((char *)g_loacl_raw_cmd_buf, "[CMD]");
            if (cmd_pos == NULL) {
                return -1;
            }

            result = sscanf(
                cmd_pos,
                "[CMD][%d][%f %f %f %f %f %f %f %f %f %f]",
                &(command->cmd_id),
                &(command->params[0]), &(command->params[1]), &(command->params[2]), &(command->params[3]), &(command->params[4]),
                &(command->params[5]), &(command->params[6]), &(command->params[7]), &(command->params[8]), &(command->params[9])
            );

            if (result < 1) {
                return -1;
            }
            command->param_count = result - 1;
            break;
        }

        case CMD_PARSE_FORMAT_STRING: {
            if (string_cmd_scanf_format[0] == '\0') {
                snprintf(
                    string_cmd_scanf_format,
                    sizeof(string_cmd_scanf_format),
                    "[CMD][%%d][%%%ld[^]]]",
                    ARM_SHELL_CMD_STRING_PARAM_MAX_LEN - 1
                );
            }

            command->str_param = (char *)malloc(ARM_SHELL_CMD_STRING_PARAM_MAX_LEN);
            if (command->str_param == NULL) {
                ARM_LOGE_TAG(ARM_SHELL_LOG_TAG, "Memory allocation failed for string parameter\n");
                return -1;
            }

            result = sscanf((char *)g_loacl_raw_cmd_buf, string_cmd_scanf_format, &(command->cmd_id), command->str_param);
            if (result < 2) {
                free(command->str_param);
                command->str_param = NULL;
                return -1;
            }
            command->param_count = 0;
            break;
        }

        default:
            ARM_LOGE_TAG(ARM_SHELL_LOG_TAG, "Unknown command format\n");
            return -1;
    }

    return 0;
}

static void arm_shell_note_parse_success(void)
{
    g_arm_shell_parser_stats.parse_success_count++;
}

static void arm_shell_note_parse_error(void)
{
    g_arm_shell_parser_stats.parse_error_count++;
}

static void arm_shell_note_overflow(void)
{
    g_arm_shell_parser_stats.overflow_count++;
}

static void arm_shell_queue_rx_frame_from_isr(
    uint8_t *rx_buffer,
    uint16_t rx_len,
    BaseType_t *task_woken)
{
    if ((rx_buffer == NULL) || (rx_len == 0U)) {
        return;
    }

    rx_buffer[rx_len] = '\0';

    if (g_arm_shell.raw_cmd_queue == NULL) {
        return;
    }

    if (xQueueSendFromISR(g_arm_shell.raw_cmd_queue, rx_buffer, task_woken) == pdTRUE) {
        arm_shell_transport_note_rx_frame_ok();
    } else {
        arm_shell_note_overflow();
        arm_shell_transport_note_rx_overflow();
    }
}

/**
 * 命令解析任务
 * @param argument 任务参数
 */
static void arm_shell_analyser(void *argument)
{
    (void)argument;

    safe_printf("arm_shell_analyser task start.\n");
    while (xQueueReceive(g_arm_shell.raw_cmd_queue, g_loacl_raw_cmd_buf, portMAX_DELAY) == pdTRUE) {
        if ((strcmp((char *)g_loacl_raw_cmd_buf, "help") == 0) ||
            (strcmp((char *)g_loacl_raw_cmd_buf, "help\n") == 0) ||
            (strcmp((char *)g_loacl_raw_cmd_buf, "help\r") == 0)) {
            shell_show_help();
            continue;
        }

        ArmShellCmdPackage command = {0};
        int ret = parse_raw_command(&command);
        if (ret != 0) {
            arm_shell_note_parse_error();
            ARM_LOGW_TAG(
                ARM_SHELL_LOG_TAG,
                "Parse command failed for raw input: %s\n",
                g_loacl_raw_cmd_buf
            );
            continue;
        }

        arm_shell_note_parse_success();

        if (command.cmd_id == CMD_ID_STOP) {
            send_ack_received(command.cmd_id);
            shell_handle_stop();
            send_ack_completed(command.cmd_id, 0);
            continue;
        }

        ret = xQueueSend(g_arm_shell.cmd_queue, &command, 0);
        if (ret != pdTRUE) {
            ARM_LOGW_TAG(ARM_SHELL_LOG_TAG, "Command queue is full\n");
        }
    }

    ARM_LOGE_TAG(ARM_SHELL_LOG_TAG, "arm_shell_analyser task exit\n");
}

/**
 * 执行命令
 * @param command 指向命令结构体的指针
 */
static void execute_command(ArmShellCmdPackage *command)
{
    ShellFunc func = g_shell_cmd_list[command->cmd_id].func;
    if (func == NULL) {
        ARM_LOGE_TAG(ARM_SHELL_LOG_TAG, "No handler for command ID: %d\n", command->cmd_id);
        send_ack_completed(command->cmd_id, -1);
        return;
    }

    func(command);

    if ((g_shell_cmd_list[command->cmd_id].format == CMD_PARSE_FORMAT_STRING) && (command->str_param != NULL)) {
        free(command->str_param);
        command->str_param = NULL;
    }
}

/**
 * 命令执行任务
 * @param argument 任务参数
 */
static void arm_shell_executor(void *argument)
{
    ArmShellCmdPackage command;

    (void)argument;

    safe_printf("arm_shell_executor task start.\n");
    while (xQueueReceive(g_arm_shell.cmd_queue, &command, portMAX_DELAY) == pdTRUE) {
        execute_command(&command);
    }

    ARM_LOGE_TAG(ARM_SHELL_LOG_TAG, "arm_shell_executor task exit\n");
}

void arm_shell_irq_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint16_t rx_index = 0U;
    static uint8_t rx_overflowed = 0U;
    static uint32_t last_rx_tick = 0U;
    uint8_t *rx_buffer = g_arm_shell.cmd_buffer;
    uint32_t sr = huart1.Instance->SR;

    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
        arm_shell_transport_note_rx_uart_error();

        if ((sr & USART_SR_ORE) != 0U) {
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        }
        if ((sr & USART_SR_NE) != 0U) {
            __HAL_UART_CLEAR_NEFLAG(&huart1);
        }
        if ((sr & USART_SR_FE) != 0U) {
            __HAL_UART_CLEAR_FEFLAG(&huart1);
        }
        if ((sr & USART_SR_PE) != 0U) {
            __HAL_UART_CLEAR_PEFLAG(&huart1);
        }
    }

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart1.Instance->DR & 0x00FFU);
        uint32_t now = HAL_GetTick();

        if (rx_overflowed != 0U) {
            if ((data == '\n') || (data == '\r')) {
                rx_index = 0U;
                rx_overflowed = 0U;
                last_rx_tick = 0U;
            }
            continue;
        }

        if ((rx_index > 0U) && (last_rx_tick != 0U) &&
            ((now - last_rx_tick) > ARM_SHELL_RX_STALE_TIMEOUT_MS)) {
            rx_index = 0U;
            last_rx_tick = 0U;
        }

        if ((data == '\n') || (data == '\r')) {
            arm_shell_queue_rx_frame_from_isr(rx_buffer, rx_index, &xHigherPriorityTaskWoken);
            rx_index = 0U;
            last_rx_tick = 0U;
            continue;
        }

        if (rx_index < (ARM_SHELL_CMD_MAX_LEN - 1U)) {
            rx_buffer[rx_index] = data;
            rx_index++;
            last_rx_tick = now;
        } else {
            rx_overflowed = 1U;
            last_rx_tick = now;
            arm_shell_note_overflow();
            arm_shell_transport_note_rx_overflow();
        }
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int arm_shell_init(void)
{
    osThreadAttr_t shell_analyser_task_attributes = {
        .name = "arm_shell_analyser",
        .stack_size = 4096,
        .priority = (osPriority_t)osPriorityHigh,
    };

    osThreadAttr_t shell_executor_task_attributes = {
        .name = "arm_shell_executor",
        .stack_size = 4096,
        .priority = (osPriority_t)osPriorityNormal7,
    };

    if (arm_shell_transport_init() != 0) {
        return -1;
    }

    g_arm_shell.raw_cmd_queue = xQueueCreate(ARM_SHELL_RAW_CMD_QUEUE_LENGTH, sizeof(uint8_t[ARM_SHELL_CMD_MAX_LEN]));
    if (g_arm_shell.raw_cmd_queue == NULL) {
        return -1;
    }

    g_arm_shell.cmd_queue = xQueueCreate(ARM_SHELL_CMD_QUEUE_LENGTH, sizeof(ArmShellCmdPackage));
    if (g_arm_shell.cmd_queue == NULL) {
        vQueueDelete(g_arm_shell.raw_cmd_queue);
        g_arm_shell.raw_cmd_queue = NULL;
        return -1;
    }

    g_arm_shell.arm_shell_analyser_task_id = osThreadNew(arm_shell_analyser, NULL, &shell_analyser_task_attributes);
    if (g_arm_shell.arm_shell_analyser_task_id == NULL) {
        goto ERROR_HANDLER;
    }

    g_arm_shell.arm_shell_executor_task_id = osThreadNew(arm_shell_executor, NULL, &shell_executor_task_attributes);
    if (g_arm_shell.arm_shell_executor_task_id == NULL) {
        goto ERROR_HANDLER;
    }

    __DSB();
    __ISB();

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    safe_printf("Command format: [CMD][Command ID][Param0 Param1 Param2 ...]\n");
    shell_show_help();
    return 0;

ERROR_HANDLER:
    if (g_arm_shell.raw_cmd_queue != NULL) {
        vQueueDelete(g_arm_shell.raw_cmd_queue);
        g_arm_shell.raw_cmd_queue = NULL;
    }

    if (g_arm_shell.cmd_queue != NULL) {
        vQueueDelete(g_arm_shell.cmd_queue);
        g_arm_shell.cmd_queue = NULL;
    }
    return -1;
}

int safe_printf(const char *format, ...)
{
    int ret;
    va_list args;

    va_start(args, format);
    ret = arm_shell_transport_submit_v(ARM_SHELL_TX_PRIORITY_STREAM, format, args);
    va_end(args);

    return ret;
}

int arm_log_printf(ArmLogLevel level, const char *tag, const char *format, ...)
{
    int ret;
    va_list args;

    va_start(args, format);
    ret = arm_log_vprintf(level, tag, format, args);
    va_end(args);

    return ret;
}

void send_ack_received(uint32_t cmd_id)
{
    (void)arm_shell_transport_submit(ARM_SHELL_TX_PRIORITY_CTRL, "ACK_RECEIVED: CMD_ID=%lu\n", (unsigned long)cmd_id);
}

void send_ack_completed(uint32_t cmd_id, int ret)
{
    if (ret == 0) {
        (void)arm_shell_transport_submit(ARM_SHELL_TX_PRIORITY_CTRL, "ACK_COMPLETED: CMD_ID=%lu, SUCCESS\n", (unsigned long)cmd_id);
    } else {
        (void)arm_shell_transport_submit(ARM_SHELL_TX_PRIORITY_CTRL, "ACK_COMPLETED: CMD_ID=%lu, ERROR\n", (unsigned long)cmd_id);
    }
}

void send_string_response(uint32_t cmd_id, const char *format, ...)
{
    char prefix[64];
    va_list args;

    (void)snprintf(prefix, sizeof(prefix), "ACK_RESPONSE: CMD_ID=%lu, ", (unsigned long)cmd_id);

    va_start(args, format);
    (void)arm_shell_transport_submit_prefixed_v(ARM_SHELL_TX_PRIORITY_CTRL, prefix, format, args);
    va_end(args);
}

void send_stream_response(uint32_t cmd_id, const char *format, ...)
{
    char prefix[64];
    va_list args;

    (void)snprintf(prefix, sizeof(prefix), "ACK_RESPONSE: CMD_ID=%lu, ", (unsigned long)cmd_id);

    va_start(args, format);
    (void)arm_shell_transport_submit_prefixed_v(ARM_SHELL_TX_PRIORITY_STREAM, prefix, format, args);
    va_end(args);
}

void arm_shell_get_transport_stats(ArmShellTransportStats *stats)
{
    arm_shell_transport_get_stats(stats);
}

void arm_shell_get_parser_stats(ArmShellParserStats *stats)
{
    if (stats == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *stats = g_arm_shell_parser_stats;
    taskEXIT_CRITICAL();
}
