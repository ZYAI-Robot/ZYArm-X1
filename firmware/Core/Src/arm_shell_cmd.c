#include "arm_shell.h"
#include "arm_robot_kinematics.h"
#include "commit_id.h"
#include "arm_flash.h"
#include "arm_recorder.h"
#include <string.h>
#include "arm_remote.h"
#include "arm_master_slave.h"
#include "arm_status_report.h"
#include "fs_usart.h"

#define ARM_SHELL_CMD_LOG_TAG "SHELL_CMD"

// 版本信息
#if defined(ARM_ROBOT_VERSION_TOC) && (ARM_ROBOT_VERSION_TOC == 1U)
static const char* HW_VERSION = "FS_V2.0 ToC";
#else
static const char* HW_VERSION = "FS_V2.0 ToB";
#endif

static const char* SW_VERSION = GIT_COMMIT_ID;
static const char* BUILD_DATE = __DATE__ " " __TIME__;

static void arm_parse_joint_targets(
    const ArmShellCmdPackage *cmd,
    float target_angles[ARM_JOINTS_NUM],
    uint8_t no_change[ARM_JOINTS_NUM],
    uint32_t *sync_mask
)
{
    uint32_t local_sync_mask = 0U;

    if ((cmd == NULL) || (target_angles == NULL)) {
        return;
    }

    if (no_change != NULL) {
        memset(no_change, 0, ARM_JOINTS_NUM);
    }

    for (int i = 0; i < ARM_JOINTS_NUM; i++) {
        if (fabsf(cmd->params[i] - ARM_SYNC_NO_CHANGE) < ARM_FLOAT_TOLERANCE) {
            target_angles[i] = g_arm_robot.joint[i].angle;
            if (no_change != NULL) {
                no_change[i] = 1U;
            }
            continue;
        }

        target_angles[i] = cmd->params[i];
        local_sync_mask |= (1UL << i);
    }

    if (sync_mask != NULL) {
        *sync_mask = local_sync_mask;
    }
}
static void handle_ik_inverse(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 6) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "ik_inverse need 6 params, got %d\n", cmd->param_count);
        return;
    }
    
    // 提取参数
    float x = cmd->params[0];
    float y = cmd->params[1];
    float z = cmd->params[2];
    float rx = cmd->params[3];
    float ry = cmd->params[4];
    float rz = cmd->params[5];
    bool verbose = (cmd->param_count >= 7) ? (bool)cmd->params[6] : false;

    send_ack_received(cmd->cmd_id);
    int ret = arm_robot_ik(x, y, z, rx, ry, rz);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }
    
    arm_robot_set_sync(ARM_ALL_JOINTS_NO_CLAW_SYNC_MASK, true);

    if (verbose) {
        send_string_response(cmd->cmd_id, "[IK] Joints Result: [%.2f %.2f %.2f %.2f %.2f %.2f]\n",
                g_arm_robot.joint[0].angle, g_arm_robot.joint[1].angle,
                g_arm_robot.joint[2].angle, g_arm_robot.joint[3].angle,
                g_arm_robot.joint[4].angle, g_arm_robot.joint[5].angle);
    }
    
    if (!arm_wait_sync_finished(-1)) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_reset(const ArmShellCmdPackage *cmd)
{
    int ret;

    send_ack_received(cmd->cmd_id);
    
    ret = arm_robot_reset();
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }

    arm_robot_set_sync(ARM_ALL_JOINTS_NO_CLAW_SYNC_MASK, true);
    if (!arm_wait_sync_finished(-1)) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_joint_sync(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 7) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "joint_sync need 7 params, got %d\n", cmd->param_count);
        return;
    }
    
    // 提取参数
    float target_angles[ARM_JOINTS_NUM];
    bool sync = (cmd->param_count >= 8) ? (bool)cmd->params[7] : true;
    bool verbose = (cmd->param_count >= 9) ? (bool)cmd->params[8] : false;
    bool fast = (cmd->param_count >= 10) ? (bool)cmd->params[9] : false;

    uint32_t sync_mask = 0;
    uint8_t no_change[ ARM_JOINTS_NUM] = {0};

    arm_parse_joint_targets(cmd, target_angles, no_change, &sync_mask);

    // 如果是从臂模式，将数据传递给从臂任务
    if (g_arm_robot.state == ARM_STATE_MASTER_SLAVE && 
        arm_master_slave_get_status() == ARM_MASTER_SLAVE_STATUS_RUNNING) {
        arm_slave_set_angles(target_angles);
        return;
    }

    if (fast) {
        if (arm_joint_sync_move(target_angles) != 0) {
            ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "joint_sync fast path failed\n");
        }
        return;
    } else {
        send_ack_received(cmd->cmd_id);
        int ret = arm_joint_angle_update(true);
        if (ret != 0) {
            ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "Update joint angles failed\n");
            send_ack_completed(cmd->cmd_id, ret);
            return;
        }

        float interval_ms;
        ret = arm_cal_interval_with_angle_diff(target_angles, &interval_ms);
        if (ret != 0) {
            send_ack_completed(cmd->cmd_id, ret);
            return;
        }

        for (int i = 0; i < ARM_JOINTS_NUM; i++) {
            if ( no_change[i]) {
                continue;
            }

            if (!arm_joint_check_angle_valid(i, target_angles[i])) {
                send_ack_completed(cmd->cmd_id, -1);
                return;
            }

            int ret = arm_set_joint_angle_interval_acc(i, target_angles[i], (int)roundf(interval_ms), ARM_DEFAULT_ACCEL_TIME, ARM_DEFAULT_ACCEL_TIME);
            if (ret != 0) {
                send_ack_completed(cmd->cmd_id, ret);
                return;
            }
        }
    }

    if (verbose) {
        ArmPose pose;
        arm_robot_fk(target_angles, &pose);
        send_string_response(cmd->cmd_id, "[FK] Target Pose: [%.2f %.2f %.2f %.2f %.2f %.2f]\n",
                pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
    }

    arm_robot_set_sync(sync_mask, sync);
    
    if (sync) {
        if (!arm_wait_sync_finished(-1)) {
            send_ack_completed(cmd->cmd_id, -1);
            return;
        }
    }

    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_get_version(const ArmShellCmdPackage *cmd)
{
    send_string_response(cmd->cmd_id, "\nHardware Version: %s \nSoftware Version: %s \nBuild Date: %s\n",
                 HW_VERSION, SW_VERSION, BUILD_DATE);
    send_ack_completed(cmd->cmd_id, 0);
}

// 单关节相关指令处理
static void handle_set_joint(const ArmShellCmdPackage *cmd)
{
    // 检查参数个数
    if (cmd->param_count < 2) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "set_joint need 2 params, got %d\n", cmd->param_count);
        return;
    }
    
    // 提取参数
    int joint_id = cmd->params[0];
    int angle = cmd->params[1];
    bool sync = (cmd->param_count >= 3) ? (bool)cmd->params[2] : true; // 默认同步等待
    
    send_ack_received(cmd->cmd_id);

    float current_angle;
    int ret = arm_get_joint_angle(joint_id, &current_angle);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }

    float interval_ms = fabsf((angle - current_angle) * 1000 / g_arm_robot.cfg.speed);
    if (interval_ms < 1000.0f) {
        interval_ms = 1000.0f;
    }

    if (!arm_joint_check_angle_valid(joint_id, angle)) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    // 执行单关节控制
    ret = arm_set_joint_angle_interval_acc(joint_id, angle, (int)roundf(interval_ms), ARM_CALC_ACCEL_TIME(interval_ms), ARM_CALC_ACCEL_TIME(interval_ms));
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }

    arm_robot_set_sync(1 << joint_id, sync);

    if (sync && !arm_wait_sync_finished(-1)) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_status(const ArmShellCmdPackage *cmd)
{
    float joint_snapshot[ARM_JOINTS_NUM] = {0};
    int ret = 0;
    send_ack_received(cmd->cmd_id);

    ret = arm_joint_snapshot_read(true, joint_snapshot);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    int verbose = (cmd->param_count >= 1) ? (int)cmd->params[0] : 0;

    if (verbose != 0) {
        for (int i = 1; i <= ARM_SERVO_NUM; i++) {
            ret = g_arm_robot.servo_ops->get_status(i);
            if (ret != 0) {
                send_ack_completed(cmd->cmd_id, ret);
                return;
            }
        }
    }

    send_string_response(
        cmd->cmd_id,
        "[STATUS] J0:%.2f J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f CLAW:%.2f\n",
        joint_snapshot[0], joint_snapshot[1], joint_snapshot[2], joint_snapshot[3],
        joint_snapshot[4], joint_snapshot[5], joint_snapshot[6]
    );
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_zero(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    int ret = arm_set_zero();    
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_status_report(const ArmShellCmdPackage *cmd)
{
    int ret = 0;

    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(
            ARM_SHELL_CMD_LOG_TAG,
            "status_report need 1 param: enable(0/1) [freq_hz]\n"
        );
        return;
    }

    send_ack_received(cmd->cmd_id);

    if ((int)cmd->params[0] == 0) {
        ret = arm_status_report_stop();
    } else if ((int)cmd->params[0] == 1) {
        uint32_t freq_hz = ARM_STATUS_REPORT_DEFAULT_FREQ_HZ;
        if (cmd->param_count >= 2) {
            freq_hz = (uint32_t)cmd->params[1];
        }
        ret = arm_status_report_start(freq_hz);
    } else {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "status_report enable must be 0 or 1\n");
        ret = -1;
    }

    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_get_status_report(const ArmShellCmdPackage *cmd)
{
    (void)cmd;

    send_ack_received(cmd->cmd_id);
    send_string_response(
        cmd->cmd_id,
        "STATUS_REPORT:RUNNING=%d,FREQ=%lu\n",
        arm_status_report_is_running() ? 1 : 0,
        (unsigned long)arm_status_report_get_frequency_hz()
    );
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_get_transport_stats(const ArmShellCmdPackage *cmd)
{
    ArmShellTransportStats stats = {0};
    ArmShellParserStats parser_stats = {0};
    UsartRuntimeStats servo_stats = {0};

    (void)cmd;

    arm_shell_get_transport_stats(&stats);
    arm_shell_get_parser_stats(&parser_stats);
    Usart_GetRuntimeStats(&FSUS_Usart, &servo_stats);
    send_ack_received(cmd->cmd_id);
    send_stream_response(
        cmd->cmd_id,
        "TRANSPORT_UART1_STATS:RX_OK=%lu,RX_OVERFLOW=%lu,RX_UART_ERROR=%lu,TX_DMA_COMPLETE=%lu,TX_CTRL_DROP=%lu,TX_STREAM_DROP=%lu,TX_TRUNCATE=%lu,TX_DMA_ERROR=%lu,TX_RESERVE_CANCEL=%lu,TX_DIAG_EMIT=%lu\n",
        (unsigned long)stats.rx_frame_ok_count,
        (unsigned long)stats.rx_overflow_count,
        (unsigned long)stats.rx_uart_error_count,
        (unsigned long)stats.tx_dma_complete_count,
        (unsigned long)stats.tx_ctrl_drop_count,
        (unsigned long)stats.tx_stream_drop_count,
        (unsigned long)stats.tx_truncate_count,
        (unsigned long)stats.tx_dma_error_count,
        (unsigned long)stats.tx_reserve_cancel_count,
        (unsigned long)stats.tx_diag_emit_count
    );
    send_stream_response(
        cmd->cmd_id,
        "CMD_PARSER_STATS:OK=%lu,ERROR=%lu,OVERFLOW=%lu\n",
        (unsigned long)parser_stats.parse_success_count,
        (unsigned long)parser_stats.parse_error_count,
        (unsigned long)parser_stats.overflow_count
    );
    send_stream_response(
        cmd->cmd_id,
        "SERVO_UART6_STATS:RX_OVERFLOW=%lu,STALE_RX_DROP=%lu,TIMEOUT=%lu,TX_ERROR=%lu\n",
        (unsigned long)servo_stats.rx_ring_overflow_count,
        (unsigned long)servo_stats.stale_rx_drop_count,
        (unsigned long)servo_stats.transaction_timeout_count,
        (unsigned long)servo_stats.tx_error_count
    );
    send_stream_response(
        cmd->cmd_id,
        "SERVO_UART6_TX_BREAKDOWN:DMA_START_BUSY=%lu,DMA_START_FAIL=%lu,TX_DONE_IRQ=%lu,TX_DONE_TIMEOUT=%lu,TC_TIMEOUT=%lu,UART_DMA_ERR=%lu,UART_LINE_ERR=%lu\n",
        (unsigned long)servo_stats.tx_dma_start_busy_count,
        (unsigned long)servo_stats.tx_dma_start_fail_count,
        (unsigned long)servo_stats.tx_done_irq_count,
        (unsigned long)servo_stats.tx_done_timeout_count,
        (unsigned long)servo_stats.tx_tc_timeout_count,
        (unsigned long)servo_stats.uart_dma_error_count,
        (unsigned long)servo_stats.uart_line_error_count
    );
    send_stream_response(
        cmd->cmd_id,
        "SERVO_TIMEOUT_BY_OP:READ_ANGLE=%lu,MONITOR=%lu,SYNC_CMD=%lu,OTHER=%lu\n",
        (unsigned long)servo_stats.timeout_read_angle_count,
        (unsigned long)servo_stats.timeout_monitor_count,
        (unsigned long)servo_stats.timeout_sync_command_count,
        (unsigned long)servo_stats.timeout_other_count
    );
    send_stream_response(
        cmd->cmd_id,
        "SERVO_REQ_REPLY:OK=%lu,FAIL=%lu,AVG_US=%lu,LAST_US=%lu,MAX_US=%lu\n",
        (unsigned long)servo_stats.request_reply_success_count,
        (unsigned long)servo_stats.request_reply_fail_count,
        (unsigned long)servo_stats.request_reply_avg_us,
        (unsigned long)servo_stats.request_reply_last_us,
        (unsigned long)servo_stats.request_reply_max_us
    );
    send_stream_response(
        cmd->cmd_id,
        "SERVO_REQ_REPLY_FAIL_BY_REASON:TIMEOUT=%lu,SEND_FAIL=%lu,TX_ERROR_EVENT=%lu,BAD_PACKET=%lu,REPLY_MISMATCH=%lu,REPLY_INCOMPLETE=%lu\n",
        (unsigned long)servo_stats.request_reply_fail_timeout_count,
        (unsigned long)servo_stats.request_reply_fail_send_fail_count,
        (unsigned long)servo_stats.request_reply_fail_tx_error_event_count,
        (unsigned long)servo_stats.request_reply_fail_bad_packet_count,
        (unsigned long)servo_stats.request_reply_fail_reply_mismatch_count,
        (unsigned long)servo_stats.request_reply_fail_reply_incomplete_count
    );
    send_ack_completed(cmd->cmd_id, 0);
}

// 夹爪相关指令处理 [CMD][8][角度，是否同步(可选，默认同步)]
static void handle_set_claw(const ArmShellCmdPackage *cmd)
{
    ArmShellCmdPackage tmp_cmd = {0};
    tmp_cmd.cmd_id = cmd->cmd_id;
    tmp_cmd.param_count = cmd->param_count + 1;
    tmp_cmd.params[0] = ARM_CLAW_JOINT_ID;
    tmp_cmd.params[1] = cmd->params[0];
    tmp_cmd.params[2] = cmd->params[1];
    handle_set_joint(&tmp_cmd);
}

static void handle_get_claw(const ArmShellCmdPackage *cmd)
{
    float claw_angle;
    int ret = arm_get_joint_angle(ARM_CLAW_JOINT_ID, &claw_angle);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }
    send_string_response(cmd->cmd_id, "CLAW:%.2f\n", claw_angle);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_get_joint(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "get_joint need 1 param, got %d\n", cmd->param_count);
        return;
    }

    int joint_id = cmd->params[0];
    float joint_angle;
    int ret = arm_get_joint_angle(joint_id, &joint_angle);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }
    send_string_response(cmd->cmd_id, "J%d:%.2f\n", joint_id, joint_angle);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_set_speed(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "set_speed need 1 param, got %d\n", cmd->param_count);
        return;
    }

    float speed = cmd->params[0];
    send_ack_received(cmd->cmd_id);
    g_arm_robot.cfg.speed = speed;
    int ret = arm_flash_config_save();
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_get_speed(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    send_string_response(cmd->cmd_id, "SPEED:%.2f\n", g_arm_robot.cfg.speed);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_record(const ArmShellCmdPackage *cmd)
{
    if (cmd->str_param == NULL || strlen(cmd->str_param) == 0) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "record need 1 params: record_name\n");
        return;
    }
    send_ack_received(cmd->cmd_id);
    int ret =arm_record_start(cmd->str_param);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
    }
}

static void handle_record_player(const ArmShellCmdPackage *cmd)
{
    if (cmd->str_param == NULL || strlen(cmd->str_param) == 0) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "record_player need 1 params: record_name\n");
        return;
    }
    send_ack_received(cmd->cmd_id);
    arm_robot_set_sync(ARM_ALL_JOINTS_SYNC_MASK, true);
    int ret = arm_record_player_start(cmd->str_param);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
    }
}

static void handle_record_stop(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    arm_record_stop();
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_record_list(const ArmShellCmdPackage *cmd) 
{
    send_ack_received(cmd->cmd_id);
    arm_record_manger_list();
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_record_clear(const ArmShellCmdPackage *cmd) 
{
    send_ack_received(cmd->cmd_id);
    int ret = arm_record_flash_erase(cmd->str_param);
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_record_show(const ArmShellCmdPackage *cmd) 
{
    if (cmd->str_param == NULL || strlen(cmd->str_param) == 0) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "record_show need 1 params: name\n");
        return;
    }
    send_ack_received(cmd->cmd_id);
    int ret = arm_record_info_show(cmd->str_param);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_set_name(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    strncpy(g_arm_robot.cfg.name, cmd->str_param, ARM_NAME_MAX_LEN - 1);
    int ret = arm_flash_config_save();
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_get_name(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    send_string_response(cmd->cmd_id, "NAME:%s\n", g_arm_robot.cfg.name);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_power_off(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    int ret = arm_set_all_joint_stop(ARM_JOINT_STOP_UNLOAD_MODE, 0);
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_remote(const ArmShellCmdPackage *cmd)
{
    if (g_arm_robot.state != ARM_STATE_REMOTE) {
        return;
    }

    if (!arm_remote_is_reset()) {
        return;
    }
    
    if (cmd->param_count < 6) {
        return;
    }
    
    // 提取参数
    float x = cmd->params[0];
    float y = cmd->params[1];
    float z = cmd->params[2];
    float rx = cmd->params[3];
    float ry = cmd->params[4];
    float rz = cmd->params[5];
    float claw_angle = cmd->params[6];
    arm_remote_ik(x, y, z, rx, ry, rz, claw_angle);
}

static void handle_remote_mode(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "remote_mode need 1 param: mode\n");
        return;
    }
    send_ack_received(cmd->cmd_id);

    if (cmd->params[0] == 0) {
        g_arm_robot.state = ARM_STATE_IDLE;
    } else {
        g_arm_robot.state = ARM_STATE_REMOTE;
    }

    send_ack_completed(cmd->cmd_id, 0);
    return;
}

static void handle_remote_reset(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    if (g_arm_robot.state != ARM_STATE_REMOTE) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "remote_reset can only be used in remote mode\n");
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    int ret = arm_remote_reset();
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }

    arm_robot_set_sync(ARM_ALL_JOINTS_SYNC_MASK, true);
    if (!arm_wait_sync_finished(-1)) {
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    arm_remote_set_reset_flag();
    send_ack_completed(cmd->cmd_id, 0);
    return;
}

static void handle_get_joint_weights(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    
    safe_printf("Joint Weights: [");
    for (int i = 0; i < ARM_JOINTS_NO_CLAW_NUM; i++) {
        safe_printf("%d", g_joint_weights[i]);
        if (i < ARM_JOINTS_NO_CLAW_NUM - 1) {
            safe_printf(", ");
        }
    }
    safe_printf("]\n");
    
    send_string_response(cmd->cmd_id, "Joint Weights: [%d, %d, %d, %d, %d, %d]\n", 
                        g_joint_weights[0], g_joint_weights[1], g_joint_weights[2], 
                        g_joint_weights[3], g_joint_weights[4], g_joint_weights[5]);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_set_joint_weights(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < ARM_JOINTS_NO_CLAW_NUM) {
        ARM_LOGE_TAG(
            ARM_SHELL_CMD_LOG_TAG,
            "set_joint_weights need %d params, got %d\n",
            ARM_JOINTS_NO_CLAW_NUM,
            cmd->param_count
        );
        return;
    }

    send_ack_received(cmd->cmd_id);
    
    // 更新关节权重
    for (int i = 0; i < ARM_JOINTS_NO_CLAW_NUM; i++) {
        g_joint_weights[i] = (int)cmd->params[i];
    }
    
    // 输出确认信息
    safe_printf("Joint weights updated: [");
    for (int i = 0; i < ARM_JOINTS_NO_CLAW_NUM; i++) {
        safe_printf("%d", g_joint_weights[i]);
        if (i < ARM_JOINTS_NO_CLAW_NUM - 1) {
            safe_printf(", ");
        }
    }
    safe_printf("]\n");
    
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_set_angle_threshold(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(
            ARM_SHELL_CMD_LOG_TAG,
            "set_angle_threshold need 1 param, got %d\n",
            cmd->param_count
        );
        return;
    }

    float threshold = cmd->params[0];
    send_ack_received(cmd->cmd_id);
    g_remote_angle_threshold = threshold;
    safe_printf("Angle threshold updated to %.2f\n", threshold);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_get_angle_threshold(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);
    send_string_response(cmd->cmd_id, "ANGLE_THRESHOLD:%.2f\n", g_remote_angle_threshold);
    send_ack_completed(cmd->cmd_id, 0);
}

/**
 * @brief Handle the master-slave follow command.
 * @param cmd Command package.
 *
 * Command format:
 * - Master: [CMD][32][1 [freq_hz]]
 * - Slave: [CMD][32][2 [freq_hz]]
 */
static void handle_master_slave(const ArmShellCmdPackage *cmd)
{
    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(
            ARM_SHELL_CMD_LOG_TAG,
            "master_slave need at least 1 param: role (1=master, 2=slave) [freq_hz]\n"
        );
        return;
    }

    send_ack_received(cmd->cmd_id);

    int ret = 0;
    int role = (int)cmd->params[0];
    uint32_t freq_hz = ARM_MASTER_SLAVE_DEFAULT_FREQ_HZ;

    if (role != ARM_ROLE_MASTER && role != ARM_ROLE_SLAVE) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "Invalid role: %d (1=master, 2=slave)\n", role);
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    if (cmd->param_count >= 2) {
        freq_hz = (uint32_t)cmd->params[1];
    }

    ret = arm_master_slave_set_frequency_hz(freq_hz);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }

    ret = arm_master_slave_start(role);
    if (ret != 0) {
        send_ack_completed(cmd->cmd_id, ret);
        return;
    }
    send_ack_completed(cmd->cmd_id, 0);
}

/**
 * @brief 处理停止主从跟随命令
 * @param cmd 命令包
 *
 * 命令格式：[CMD][33]
 */
static void handle_master_slave_stop(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);

    int ret = arm_master_slave_stop();
    send_ack_completed(cmd->cmd_id, ret);
}

static void handle_master_slave_set_lpf(const ArmShellCmdPackage *cmd)
{
    send_ack_received(cmd->cmd_id);

    if (cmd->param_count < 1) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "master_slave_set_lpf need 1 param: alpha\n");
        send_ack_completed(cmd->cmd_id, -1);
        return;
    }

    float alpha = cmd->params[0];
    arm_master_slave_set_lpf_coefficient(alpha);
    send_ack_completed(cmd->cmd_id, 0);
}

static void handle_joint_io_fast(const ArmShellCmdPackage *cmd)
{
    float target_angles[ARM_JOINTS_NUM] = {0};
    float joint_snapshot[ARM_JOINTS_NUM] = {0};
    int snapshot_ok;
    int ret;

    if (cmd->param_count < ARM_JOINTS_NUM) {
        ARM_LOGE_TAG(
            ARM_SHELL_CMD_LOG_TAG,
            "joint_io_fast need %d params, got %d\n",
            ARM_JOINTS_NUM,
            cmd->param_count
        );
        return;
    }

    arm_parse_joint_targets(cmd, target_angles, NULL, NULL);
    snapshot_ok = (arm_joint_snapshot_read(true, joint_snapshot) == 0);

    if (g_arm_robot.state == ARM_STATE_MASTER_SLAVE &&
        arm_master_slave_get_status() == ARM_MASTER_SLAVE_STATUS_RUNNING &&
        arm_master_slave_get_role() == ARM_ROLE_SLAVE) {
        arm_slave_set_angles(target_angles);
        if (snapshot_ok != 0) {
            arm_print_status_frame(joint_snapshot);
        }
        return;
    }

    ret = arm_joint_sync_move(target_angles);
    if (ret != 0) {
        ARM_LOGE_TAG(ARM_SHELL_CMD_LOG_TAG, "joint_io_fast sync move failed, ret=%d\n", ret);
        return;
    }

    if (snapshot_ok != 0) {
        arm_print_status_frame(joint_snapshot);
    }
}

static void stub_handle(const ArmShellCmdPackage *cmd)
{
    return;
}

// 指令列表
const ArmShellCmd g_shell_cmd_list[] = {
    [CMD_ID_IK_INVERSE] = {"ik_inverse", "Perform IK inverse calculation with 6 parameters: x y z rx ry rz [verbose]", handle_ik_inverse, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_RESET] = {"reset", "Reset the robot arm to initial position", handle_reset, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_STOP] = {"stop", "Stop all robot arm movements", stub_handle, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_JOINT_SYNC] = {"joint_sync", "Set all joints synchronously with 7 parameters: j0 j1 j2 j3 j4 j5 claw [sync verbose fast]", handle_joint_sync, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_VERSION] = {"version", "Get firmware version information", handle_get_version, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_SET_JOINT] = {"set_joint", "Set specific joint angle with 2 parameters: joint_id angle [sync]", handle_set_joint, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_STATUS] = {"status", "Query current status of all joints [verbose]", handle_status, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_ZERO] = {"zero", "Set current position as zero point", handle_zero, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_SET_CLAW] = {"set_claw", "Set claw position with 1 parameter: position [sync]", handle_set_claw, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_CLAW] = {"get_claw", "Get current claw position", handle_get_claw, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_JOINT] = {"get_joint", "Get specific joint angle with 1 parameter: joint_id", handle_get_joint, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_SET_JOINT_SPEED] = {"set_speed", "Set joint speed with 1 parameter: speed(degree/s)", handle_set_speed, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_JOINT_SPEED] = {"get_speed", "Get current joint speed", handle_get_speed, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_RECORD] = {"record", "Record current movement with 1 parameter: record_name", handle_record, CMD_PARSE_FORMAT_STRING},
    [CMD_ID_RECORD_PLAYER] = {"record_player", "Playback recorded movement with 1 parameter: record_name", handle_record_player, CMD_PARSE_FORMAT_STRING},
    [CMD_ID_RECORD_STOP] = {"record_stop", "Stop recording movement", handle_record_stop, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_RECORD_LIST] = {"record_list", "List of recorded movements", handle_record_list, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_STATUS_REPORT] = {"status_report", "Enable or disable automatic status reporting with parameters: enable(0/1) [freq_hz, default=20]", handle_status_report, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_STATUS_REPORT] = {"get_status_report", "Get automatic status reporting state and frequency", handle_get_status_report, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_RECORD_CLEAR] = {"record_clear", "Clear specific recorded movement with 1 parameter: record_name", handle_record_clear, CMD_PARSE_FORMAT_STRING},
    [CMD_ID_RECORD_SHOW] = {"record_show", "Show specific recorded movement with 1 parameter: record_name", handle_record_show, CMD_PARSE_FORMAT_STRING},
    [CMD_ID_SET_NAME] = {"set_name", "Set robot arm name with 1 string parameter: name", handle_set_name, CMD_PARSE_FORMAT_STRING},
    [CMD_ID_GET_NAME] = {"get_name", "Get current robot arm name", handle_get_name, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_POWER_OFF] = {"poweroff", "Power off the robot arm", handle_power_off, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_REMOTE_MODE] = {"remote_mode", "Enable or disable remote control mode with 1 parameter: enable(0/1)", handle_remote_mode, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_REMOTE_RESET] = {"remote_reset", "Reset the robot arm to initial position in remote control mode", handle_remote_reset, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_REMOTE] =  {"remote", "Remote control robot arm with 7 parameters: x y z rx ry rz claw_angle", handle_remote, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_JOINT_WEIGHTS] = {"get_weights", "Get joint weights for IK calculation", handle_get_joint_weights, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_SET_JOINT_WEIGHTS] = {"set_weights", "Set joint weights for IK calculation with 6 parameters: w0 w1 w2 w3 w4 w5", handle_set_joint_weights, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_SET_ANGLE_THRESHOLD] = {"set_angle_threshold", "Set the angle threshold for remote control with 1 parameter: threshold", handle_set_angle_threshold, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_ANGLE_THRESHOLD] = {"get_angle_threshold", "Get current angle threshold for remote control", handle_get_angle_threshold, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_MASTER_SLAVE] = {"master_slave", "Start master-slave follow mode with parameters: mode(1=master, 2=slave) [freq_hz, default=50]", handle_master_slave, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_MASTER_SLAVE_STOP] = {"master_slave_stop", "Stop master-slave follow mode", handle_master_slave_stop, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_MASTER_SLAVE_SET_LPF] = {"master_slave_set_lpf", "Set LPF coefficient with 1 parameter: alpha [0.0-1.0]", handle_master_slave_set_lpf, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_GET_TRANSPORT_STATS] = {"get_transport_stats", "Get firmware UART1/UART6 communication health snapshot", handle_get_transport_stats, CMD_PARSE_FORMAT_FLOAT},
    [CMD_ID_JOINT_IO_FAST] = {"joint_io_fast", "Batch read joint snapshot, fast sync move, and output [STATUS] with 7 parameters: j0 j1 j2 j3 j4 j5 claw", handle_joint_io_fast, CMD_PARSE_FORMAT_FLOAT},
};

void shell_show_help()
{
    safe_printf("Available commands:\n");
    for (int i = 0; i < sizeof(g_shell_cmd_list) / sizeof(ArmShellCmd); i++) {
        if (g_shell_cmd_list[i].func == NULL) {
            continue;
        }
        safe_printf("[CMD %d] %s: %s\n", i, g_shell_cmd_list[i].name, g_shell_cmd_list[i].help);
    }
}

void shell_handle_stop()
{
    // 删除正在工作的任务
    if ((g_arm_robot.state != ARM_STATE_IDLE) 
            && (g_arm_robot.tasks_handle != NULL)) {
        vTaskDelete(g_arm_robot.tasks_handle);
        g_arm_robot.tasks_handle = NULL;
        g_arm_robot.state = ARM_STATE_IDLE;
    }
    arm_set_all_joint_stop(ARM_JOINT_STOP_LOCK_MODE, 0);
}
