#ifndef __ARM_FIRST_USE_RUNTIME_H__
#define __ARM_FIRST_USE_RUNTIME_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ARM_FIRST_USE_RUNTIME_STATUS_NORMAL = 0,
    ARM_FIRST_USE_RUNTIME_STATUS_SATURATED,
    ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL,
    ARM_FIRST_USE_RUNTIME_STATUS_DISABLED,
} ArmFirstUseRuntimeStatus;

typedef struct {
    ArmFirstUseRuntimeStatus status;
    uint32_t runtime_minutes;
    uint32_t record_format_version;
    const char *firmware_commit_id;
    const char *firmware_build_time;
    const char *error_reason;
} ArmFirstUseRuntimeInfo;

int arm_first_use_runtime_init(void);
void arm_first_use_runtime_tick(uint32_t now_ms);
void arm_first_use_runtime_get_info(ArmFirstUseRuntimeInfo *info);
const char *arm_first_use_runtime_status_string(ArmFirstUseRuntimeStatus status);
int arm_first_use_runtime_format(void);

#ifdef __cplusplus
}
#endif

#endif
