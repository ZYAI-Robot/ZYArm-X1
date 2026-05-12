#ifndef __ARM_SHELL_H
#define __ARM_SHELL_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>
#include "queue.h"
#include "arm_robot_config.h"
#include "arm_shell_transport.h"

#define ARM_SHELL_CMD_MAX_LEN 128
#define ARM_SHELL_CMD_STRING_PARAM_MAX_LEN (ARM_NAME_MAX_LEN + 1)

// 指令编号定义
typedef enum {
    CMD_ID_IK_INVERSE = 0,
    CMD_ID_RESET,
    CMD_ID_STOP,
    CMD_ID_JOINT_SYNC,
    CMD_ID_GET_VERSION,
    CMD_ID_SET_JOINT,
    CMD_ID_STATUS,
    CMD_ID_ZERO,
    CMD_ID_SET_CLAW,
    CMD_ID_GET_CLAW,
    CMD_ID_GET_JOINT,
    CMD_ID_SET_JOINT_SPEED,
    CMD_ID_GET_JOINT_SPEED,
    CMD_ID_RECORD,
    CMD_ID_RECORD_PLAYER,
    CMD_ID_RECORD_STOP,
    CMD_ID_RECORD_LIST,
    CMD_ID_STATUS_REPORT,
    CMD_ID_GET_STATUS_REPORT,
    CMD_ID_RECORD_CLEAR,
    CMD_ID_RECORD_SHOW,
    CMD_ID_SET_NAME,
    CMD_ID_GET_NAME,
    CMD_ID_POWER_OFF,
    CMD_ID_REMOTE_MODE,
    CMD_ID_REMOTE_RESET,
    CMD_ID_REMOTE,
    CMD_ID_GET_JOINT_WEIGHTS,
    CMD_ID_SET_JOINT_WEIGHTS,
    CMD_ID_SET_ANGLE_THRESHOLD,
    CMD_ID_GET_ANGLE_THRESHOLD,
    CMD_ID_GET_ALL_JOINTS_SYNC,  
    CMD_ID_MASTER_SLAVE,
    CMD_ID_MASTER_SLAVE_STOP,
    CMD_ID_MASTER_SLAVE_SET_LPF,
    CMD_ID_GET_TRANSPORT_STATS,
    CMD_ID_JOINT_IO_FAST,
    CMD_ID_IK_SOLVE,
    CMD_ID_MAX_NUM,
} CMD_ID;

typedef enum {
    CMD_PARSE_FORMAT_FLOAT = 0,     // 浮点数参数格式 [%f %f %f ...]
    CMD_PARSE_FORMAT_STRING = 1,    // 字符串参数格式 [%s]
} CommandParseFormat;

typedef enum {
    ARM_LOG_LEVEL_INFO = 0,
    ARM_LOG_LEVEL_WARN = 1,
    ARM_LOG_LEVEL_ERROR = 2,
} ArmLogLevel;

/**
     * 命令包结构体
     * @param cmd_id 命令ID
     * @param param_count 参数数量
     * @param params 参数数组，最多10个浮点数参数
     * @param str_param 字符串参数，最多64个字符
     */
typedef struct {
    uint32_t cmd_id;
    uint32_t param_count;
    float params[10];
    char *str_param;
} ArmShellCmdPackage;

// 指令处理函数类型定义
typedef void (* ShellFunc)(const ArmShellCmdPackage *cmd);

// 指令结构体定义
typedef struct {
    char *name;                 // 指令名称
    char *help;                 // 指令帮助信息
    ShellFunc func;             // 指令处理函数
    CommandParseFormat format;  // 指令解析格式
} ArmShellCmd;

typedef struct {
    uint8_t cmd_buffer[ARM_SHELL_CMD_MAX_LEN];
    QueueHandle_t cmd_queue;                    // 已处理解析的命令队列
    QueueHandle_t raw_cmd_queue;                // 未处理的命令字符串队列
    osThreadId_t arm_shell_analyser_task_id;
    osThreadId_t arm_shell_executor_task_id;
} ArmShell;

typedef struct {
    uint32_t parse_success_count;
    uint32_t parse_error_count;
    uint32_t overflow_count;
} ArmShellParserStats;

extern ArmShell g_arm_shell;
extern const ArmShellCmd g_shell_cmd_list[];

int arm_shell_init(void);

/**
     * 发送命令接收响应
     */
void arm_shell_irq_handler(void);

/**
     * 发送命令接收响应
     * @param cmd_id 命令ID
     */
void send_ack_received(uint32_t cmd_id);

/**
     * 发送命令完成响应
     * @param cmd_id 命令ID
     * @param ret 命令执行结果，0表示成功，非0表示失败
     */
void send_ack_completed(uint32_t cmd_id, int ret);
void send_string_response(uint32_t cmd_id, const char *format, ...);
void send_stream_response(uint32_t cmd_id, const char *format, ...);
void handle_stop(const ArmShellCmdPackage *cmd);
void shell_show_help(void);

/**
     * 安全打印函数，运行时会通过 UART transport 统一串行化输出
     * @param format 格式化字符串
     * @param ... 可变参数
     * @return 打印的字符数
     */
int safe_printf(const char *format, ...);
int arm_log_printf(ArmLogLevel level, const char *tag, const char *format, ...);
size_t arm_log_format_prefix(char *buffer, size_t buffer_size, ArmLogLevel level, const char *tag);

#define ARM_LOGI(format, ...) arm_log_printf(ARM_LOG_LEVEL_INFO, NULL, format, ##__VA_ARGS__)
#define ARM_LOGW(format, ...) arm_log_printf(ARM_LOG_LEVEL_WARN, NULL, format, ##__VA_ARGS__)
#define ARM_LOGE(format, ...) arm_log_printf(ARM_LOG_LEVEL_ERROR, NULL, format, ##__VA_ARGS__)
#define ARM_LOGI_TAG(tag, format, ...) arm_log_printf(ARM_LOG_LEVEL_INFO, tag, format, ##__VA_ARGS__)
#define ARM_LOGW_TAG(tag, format, ...) arm_log_printf(ARM_LOG_LEVEL_WARN, tag, format, ##__VA_ARGS__)
#define ARM_LOGE_TAG(tag, format, ...) arm_log_printf(ARM_LOG_LEVEL_ERROR, tag, format, ##__VA_ARGS__)

void shell_handle_stop(void);
void arm_shell_get_transport_stats(ArmShellTransportStats *stats);
void arm_shell_get_parser_stats(ArmShellParserStats *stats);
#endif
