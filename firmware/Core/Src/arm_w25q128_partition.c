#include "arm_w25q128_partition.h"

#include "FreeRTOS.h"
#include "task.h"
#include "w25q128.h"

#include <stddef.h>

#define ARM_W25Q128_RECORDER_SECTOR_COUNT 63U

/* Static W25Q128 layout: recorder uses sectors 0..62, first-use runtime uses sector 4095. */
static const ArmW25Q128Partition g_w25q128_partitions[] = {
    {
        .module = ARM_W25Q128_MODULE_RECORDER,
        .name = "RECORDER",
        .start_sector = 0U,
        .sector_count = ARM_W25Q128_RECORDER_SECTOR_COUNT,
        .flags = ARM_W25Q128_ACCESS_READ | ARM_W25Q128_ACCESS_WRITE | ARM_W25Q128_ACCESS_ERASE,
    },
    {
        .module = ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
        .name = "FIRST_USE_RUNTIME",
        .start_sector = W25Q128_SECTORS_COUNT - 1U,
        .sector_count = 1U,
        .flags = ARM_W25Q128_ACCESS_READ | ARM_W25Q128_ACCESS_WRITE | ARM_W25Q128_ACCESS_ERASE,
    },
};

static bool g_partition_ready = false;
static volatile bool g_partition_busy = false;

static uint32_t arm_w25q128_partition_count(void)
{
    return (uint32_t)(sizeof(g_w25q128_partitions) / sizeof(g_w25q128_partitions[0]));
}

static bool arm_w25q128_partition_ranges_overlap(const ArmW25Q128Partition *a,
                                                 const ArmW25Q128Partition *b)
{
    uint32_t a_end = a->start_sector + a->sector_count;
    uint32_t b_end = b->start_sector + b->sector_count;

    return (a->start_sector < b_end) && (b->start_sector < a_end);
}

static int arm_w25q128_partition_validate_table(void)
{
    uint32_t count = arm_w25q128_partition_count();

    for (uint32_t i = 0U; i < count; i++) {
        const ArmW25Q128Partition *partition = &g_w25q128_partitions[i];

        if ((partition->module >= ARM_W25Q128_MODULE_COUNT) || (partition->sector_count == 0U)) {
            return ARM_W25Q128_ERR_INVALID_ARG;
        }

        if (partition->start_sector >= W25Q128_SECTORS_COUNT) {
            return ARM_W25Q128_ERR_OUT_OF_RANGE;
        }

        if (partition->sector_count > (W25Q128_SECTORS_COUNT - partition->start_sector)) {
            return ARM_W25Q128_ERR_OUT_OF_RANGE;
        }

        for (uint32_t j = i + 1U; j < count; j++) {
            if (arm_w25q128_partition_ranges_overlap(partition, &g_w25q128_partitions[j])) {
                return ARM_W25Q128_ERR_OVERLAP;
            }
        }
    }

    return ARM_W25Q128_OK;
}

static int arm_w25q128_partition_try_lock(void)
{
    int ret = ARM_W25Q128_OK;

    taskENTER_CRITICAL();
    if (g_partition_busy) {
        ret = ARM_W25Q128_ERR_BUSY;
    } else {
        g_partition_busy = true;
    }
    taskEXIT_CRITICAL();

    return ret;
}

static void arm_w25q128_partition_unlock(void)
{
    taskENTER_CRITICAL();
    g_partition_busy = false;
    taskEXIT_CRITICAL();
}

static bool arm_w25q128_partition_sector_allocated(uint32_t sector)
{
    uint32_t count = arm_w25q128_partition_count();

    for (uint32_t i = 0U; i < count; i++) {
        const ArmW25Q128Partition *partition = &g_w25q128_partitions[i];
        uint32_t end_sector = partition->start_sector + partition->sector_count;

        if ((sector >= partition->start_sector) && (sector < end_sector)) {
            return true;
        }
    }

    return false;
}

static int arm_w25q128_partition_validate_access(ArmW25Q128ModuleId module,
                                                 uint32_t offset,
                                                 uint32_t len,
                                                 uint32_t access,
                                                 const ArmW25Q128Partition **partition_out)
{
    const ArmW25Q128Partition *partition = arm_w25q128_partition_get(module);
    uint32_t partition_size;

    if (!g_partition_ready) {
        return ARM_W25Q128_ERR_NOT_READY;
    }

    if ((partition == NULL) || (partition_out == NULL)) {
        return ARM_W25Q128_ERR_INVALID_ARG;
    }

    if ((partition->flags & access) != access) {
        return ARM_W25Q128_ERR_DENIED;
    }

    partition_size = partition->sector_count * W25Q128_SECTOR_SIZE;
    if ((offset > partition_size) || (len > (partition_size - offset))) {
        return ARM_W25Q128_ERR_OUT_OF_RANGE;
    }

    *partition_out = partition;
    return ARM_W25Q128_OK;
}

int arm_w25q128_partition_init(void)
{
    int ret;

    g_partition_ready = false;
    g_partition_busy = false;

    ret = arm_w25q128_partition_validate_table();
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    g_partition_ready = true;
    return ARM_W25Q128_OK;
}

bool arm_w25q128_partition_is_ready(void)
{
    return g_partition_ready;
}

const ArmW25Q128Partition *arm_w25q128_partition_get(ArmW25Q128ModuleId module)
{
    uint32_t count = arm_w25q128_partition_count();

    for (uint32_t i = 0U; i < count; i++) {
        if (g_w25q128_partitions[i].module == module) {
            return &g_w25q128_partitions[i];
        }
    }

    return NULL;
}

uint32_t arm_w25q128_partition_size(ArmW25Q128ModuleId module)
{
    const ArmW25Q128Partition *partition = arm_w25q128_partition_get(module);

    if (partition == NULL) {
        return 0U;
    }

    return partition->sector_count * W25Q128_SECTOR_SIZE;
}

int arm_w25q128_partition_get_free_ranges(ArmW25Q128FreeRange *ranges,
                                          uint32_t max_ranges,
                                          uint32_t *range_count)
{
    uint32_t found = 0U;
    bool truncated = false;
    uint32_t sector = 0U;

    if ((ranges == NULL) && (max_ranges > 0U)) {
        return ARM_W25Q128_ERR_INVALID_ARG;
    }

    while (sector < W25Q128_SECTORS_COUNT) {
        if (arm_w25q128_partition_sector_allocated(sector)) {
            sector++;
            continue;
        }

        uint32_t start_sector = sector;
        while ((sector < W25Q128_SECTORS_COUNT) && !arm_w25q128_partition_sector_allocated(sector)) {
            sector++;
        }

        if (found < max_ranges) {
            ranges[found].start_sector = start_sector;
            ranges[found].sector_count = sector - start_sector;
        } else {
            truncated = true;
        }

        found++;
    }

    if (range_count != NULL) {
        *range_count = found;
    }

    return truncated ? ARM_W25Q128_ERR_OUT_OF_RANGE : ARM_W25Q128_OK;
}

int arm_w25q128_partition_read(ArmW25Q128ModuleId module,
                               uint32_t offset,
                               void *data,
                               uint32_t len,
                               uint32_t timeout_ms)
{
    const ArmW25Q128Partition *partition = NULL;
    uint32_t address;
    int ret;

    if ((data == NULL) && (len > 0U)) {
        return ARM_W25Q128_ERR_INVALID_ARG;
    }

    ret = arm_w25q128_partition_validate_access(module,
                                                offset,
                                                len,
                                                ARM_W25Q128_ACCESS_READ,
                                                &partition);
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    if (len == 0U) {
        return ARM_W25Q128_OK;
    }

    ret = arm_w25q128_partition_try_lock();
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    address = partition->start_sector * W25Q128_SECTOR_SIZE + offset;
    ret = (W25Q128_Read(address, (uint8_t *)data, len, timeout_ms) == HAL_OK)
              ? ARM_W25Q128_OK
              : ARM_W25Q128_ERR_DRIVER;
    arm_w25q128_partition_unlock();

    return ret;
}

int arm_w25q128_partition_write(ArmW25Q128ModuleId module,
                                uint32_t offset,
                                const void *data,
                                uint32_t len,
                                uint32_t timeout_ms)
{
    const ArmW25Q128Partition *partition = NULL;
    uint32_t address;
    int ret;

    if ((data == NULL) && (len > 0U)) {
        return ARM_W25Q128_ERR_INVALID_ARG;
    }

    ret = arm_w25q128_partition_validate_access(module,
                                                offset,
                                                len,
                                                ARM_W25Q128_ACCESS_WRITE,
                                                &partition);
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    if (len == 0U) {
        return ARM_W25Q128_OK;
    }

    ret = arm_w25q128_partition_try_lock();
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    address = partition->start_sector * W25Q128_SECTOR_SIZE + offset;
    ret = (W25Q128_Write(address, (uint8_t *)data, len, timeout_ms) == HAL_OK)
              ? ARM_W25Q128_OK
              : ARM_W25Q128_ERR_DRIVER;
    arm_w25q128_partition_unlock();

    return ret;
}

int arm_w25q128_partition_erase(ArmW25Q128ModuleId module,
                                uint32_t offset,
                                uint32_t len,
                                uint32_t timeout_ms)
{
    const ArmW25Q128Partition *partition = NULL;
    uint32_t address;
    int ret;

    if (((offset % W25Q128_SECTOR_SIZE) != 0U) || ((len % W25Q128_SECTOR_SIZE) != 0U)) {
        return ARM_W25Q128_ERR_INVALID_ARG;
    }

    ret = arm_w25q128_partition_validate_access(module,
                                                offset,
                                                len,
                                                ARM_W25Q128_ACCESS_ERASE,
                                                &partition);
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    if (len == 0U) {
        return ARM_W25Q128_OK;
    }

    ret = arm_w25q128_partition_try_lock();
    if (ret != ARM_W25Q128_OK) {
        return ret;
    }

    address = partition->start_sector * W25Q128_SECTOR_SIZE + offset;
    ret = (W25Q128_EraseSectors(address, len, timeout_ms) == HAL_OK)
              ? ARM_W25Q128_OK
              : ARM_W25Q128_ERR_DRIVER;
    arm_w25q128_partition_unlock();

    return ret;
}
