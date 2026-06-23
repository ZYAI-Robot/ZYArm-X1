#ifndef __ARM_W25Q128_PARTITION_H__
#define __ARM_W25Q128_PARTITION_H__

#include <stdbool.h>
#include <stdint.h>

#define ARM_W25Q128_ACCESS_READ   (1U << 0)
#define ARM_W25Q128_ACCESS_WRITE  (1U << 1)
#define ARM_W25Q128_ACCESS_ERASE  (1U << 2)

#define ARM_W25Q128_OK                  0
#define ARM_W25Q128_ERR_NOT_READY      -1
#define ARM_W25Q128_ERR_BUSY           -2
#define ARM_W25Q128_ERR_INVALID_ARG    -3
#define ARM_W25Q128_ERR_OUT_OF_RANGE   -4
#define ARM_W25Q128_ERR_DENIED         -5
#define ARM_W25Q128_ERR_DRIVER         -6
#define ARM_W25Q128_ERR_OVERLAP        -7

typedef enum {
    ARM_W25Q128_MODULE_RECORDER = 0,
    ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
    ARM_W25Q128_MODULE_COUNT
} ArmW25Q128ModuleId;

typedef struct {
    ArmW25Q128ModuleId module;
    const char *name;
    uint32_t start_sector;
    uint32_t sector_count;
    uint32_t flags;
} ArmW25Q128Partition;

typedef struct {
    uint32_t start_sector;
    uint32_t sector_count;
} ArmW25Q128FreeRange;

int arm_w25q128_partition_init(void);
bool arm_w25q128_partition_is_ready(void);
const ArmW25Q128Partition *arm_w25q128_partition_get(ArmW25Q128ModuleId module);
uint32_t arm_w25q128_partition_size(ArmW25Q128ModuleId module);
int arm_w25q128_partition_get_free_ranges(ArmW25Q128FreeRange *ranges,
                                          uint32_t max_ranges,
                                          uint32_t *range_count);

int arm_w25q128_partition_read(ArmW25Q128ModuleId module,
                               uint32_t offset,
                               void *data,
                               uint32_t len,
                               uint32_t timeout_ms);
int arm_w25q128_partition_write(ArmW25Q128ModuleId module,
                                uint32_t offset,
                                const void *data,
                                uint32_t len,
                                uint32_t timeout_ms);
int arm_w25q128_partition_erase(ArmW25Q128ModuleId module,
                                uint32_t offset,
                                uint32_t len,
                                uint32_t timeout_ms);

#endif
