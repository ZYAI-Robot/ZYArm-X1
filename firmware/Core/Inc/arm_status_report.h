#ifndef __ARM_STATUS_REPORT_H__
#define __ARM_STATUS_REPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ARM_STATUS_REPORT_DEFAULT_FREQ_HZ 20U
#define ARM_STATUS_REPORT_MIN_FREQ_HZ 1U
#define ARM_STATUS_REPORT_MAX_FREQ_HZ 50U

int arm_status_report_start(uint32_t freq_hz);
int arm_status_report_stop(void);
bool arm_status_report_is_running(void);
uint32_t arm_status_report_get_frequency_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_STATUS_REPORT_H__ */
