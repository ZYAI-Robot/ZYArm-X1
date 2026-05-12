#ifndef __ARM_RECORDER_H__
#define __ARM_RECORDER_H__

#include "arm_robot.h"
#include "cmsis_os2.h"
#include "arm_flash.h"
#include "arm_robot_config.h"
#include "w25q128.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_RECORD_INFO_NUM             3000
#define ARM_RECORD_INTERVAL_DEFAULT     100 // MS
#define ARM_RECORD_STATUS_RUNNING       (0x10012345U)
#define ARM_RECORD_STATUS_STOP          (0x20023456U)
#define ARM_RECORD_STATUS_VALID         (0x30034567U)

#define ARM_RECORD_MAX_NUMS             (3U)                    // 最大录制动作数量
#define ARM_RECORD_NAME_MAX_LEN         ARM_NAME_MAX_LEN        // 动作名称最大长度

#define ARM_RECORD_MANAGER_FLASH_SIZE   ((sizeof(ArmRecordManager) / (W25Q128_SECTOR_SIZE) + 1) * W25Q128_SECTOR_SIZE) // 每个动作占用块大小

typedef struct {
    float joint_angle[ARM_JOINTS_NUM];
} ArmRecordInfo;

typedef struct {
    char name[ARM_RECORD_NAME_MAX_LEN];
    int info_num;
    int state;
    osThreadId_t record_thread_handle;
} ArmRecordHead;

typedef struct {
    ArmRecordHead head;
    ArmRecordInfo info[ARM_RECORD_INFO_NUM];
} ArmRecordManager;

typedef struct {
    float lpf_coefficient;        // 一阶低通滤波系数(0.0-1.0)，值越大滤波越弱，值越小滤波越强
    int player_interval_ms;       // 动作播放间隔时间
} ArmRecorderParam;

extern ArmRecorderParam g_arm_recorder_param;

int arm_record_start(const char *name);
void arm_record_stop(void);
int arm_record_player_start(const char *name);
int arm_record_info_show(const char *name);
int arm_record_flash_erase(const char *name);
void arm_record_manger_list(uint32_t cmd_id);
int arm_record_flash_malloc(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_RECORDER_H__ */
