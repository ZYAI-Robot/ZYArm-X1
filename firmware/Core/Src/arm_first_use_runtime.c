#include "arm_first_use_runtime.h"

#include "arm_w25q128_partition.h"
#include "commit_id.h"
#include "w25q128.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define ARM_FIRST_USE_RUNTIME_MAGIC              0x52544D31U
#define ARM_FIRST_USE_RUNTIME_FORMAT_VERSION     1U
#define ARM_FIRST_USE_RUNTIME_GRANULARITY_SEC    60U
#define ARM_FIRST_USE_RUNTIME_COMMIT_MARKER      0xA5A55A5AU
#define ARM_FIRST_USE_RUNTIME_COMMIT_BUSY        1
#define ARM_FIRST_USE_RUNTIME_MINUTE_MS          60000U
#define ARM_FIRST_USE_RUNTIME_READ_CHUNK         64U

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t granularity_sec;
    uint32_t bitmap_offset;
    uint32_t bitmap_bits;
    uint32_t header_size;
    uint32_t header_crc;
    uint32_t commit_marker;
} ArmFirstUseRuntimeHeader;

static const char *g_firmware_build_time = __DATE__ " " __TIME__;
static ArmFirstUseRuntimeStatus g_runtime_status = ARM_FIRST_USE_RUNTIME_STATUS_DISABLED;
static const char *g_runtime_error_reason = "not_initialized";
static uint32_t g_runtime_minutes = 0U;
static uint32_t g_runtime_format_version = 0U;
static uint32_t g_runtime_bitmap_offset = sizeof(ArmFirstUseRuntimeHeader);
static uint32_t g_runtime_bitmap_bits = 0U;
static uint32_t g_runtime_next_bit = 0U;
static uint32_t g_runtime_last_tick = 0U;
static uint32_t g_runtime_pending_ms = 0U;
static bool g_runtime_tick_started = false;

static uint32_t arm_first_use_runtime_expected_bitmap_bits(void)
{
    return (W25Q128_SECTOR_SIZE - (uint32_t)sizeof(ArmFirstUseRuntimeHeader)) * 8U;
}

static uint32_t arm_first_use_runtime_header_crc(const ArmFirstUseRuntimeHeader *header)
{
    ArmFirstUseRuntimeHeader temp;
    const uint8_t *bytes;
    uint32_t hash = 2166136261U;

    temp = *header;
    temp.header_crc = 0x00000000U;
    temp.commit_marker = 0xFFFFFFFFU;
    bytes = (const uint8_t *)&temp;

    for (uint32_t i = 0U; i < (uint32_t)sizeof(temp); i++) {
        hash ^= bytes[i];
        hash *= 16777619U;
    }

    return hash;
}

static void arm_first_use_runtime_set_error(ArmFirstUseRuntimeStatus status, const char *reason)
{
    g_runtime_status = status;
    g_runtime_error_reason = (reason != NULL) ? reason : "unknown";
}

static bool arm_first_use_runtime_buffer_erased(const uint8_t *buffer, uint32_t len)
{
    if (buffer == NULL) {
        return false;
    }

    for (uint32_t i = 0U; i < len; i++) {
        if (buffer[i] != 0xFFU) {
            return false;
        }
    }

    return true;
}

static int arm_first_use_runtime_sector_is_erased(bool *is_erased)
{
    uint8_t buffer[ARM_FIRST_USE_RUNTIME_READ_CHUNK];
    uint32_t offset = 0U;

    if (is_erased == NULL) {
        return -1;
    }

    *is_erased = false;
    while (offset < W25Q128_SECTOR_SIZE) {
        uint32_t len = W25Q128_SECTOR_SIZE - offset;
        if (len > sizeof(buffer)) {
            len = sizeof(buffer);
        }

        if (arm_w25q128_partition_read(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                       offset,
                                       buffer,
                                       len,
                                       1000) != ARM_W25Q128_OK) {
            return -1;
        }

        if (!arm_first_use_runtime_buffer_erased(buffer, len)) {
            return 0;
        }

        offset += len;
    }

    *is_erased = true;
    return 0;
}

static void arm_first_use_runtime_make_header(ArmFirstUseRuntimeHeader *header)
{
    if (header == NULL) {
        return;
    }

    memset(header, 0xFF, sizeof(*header));
    header->magic = ARM_FIRST_USE_RUNTIME_MAGIC;
    header->version = ARM_FIRST_USE_RUNTIME_FORMAT_VERSION;
    header->granularity_sec = ARM_FIRST_USE_RUNTIME_GRANULARITY_SEC;
    header->bitmap_offset = (uint32_t)sizeof(ArmFirstUseRuntimeHeader);
    header->bitmap_bits = arm_first_use_runtime_expected_bitmap_bits();
    header->header_size = (uint32_t)sizeof(ArmFirstUseRuntimeHeader);
    header->header_crc = arm_first_use_runtime_header_crc(header);
}

static int arm_first_use_runtime_write_header(void)
{
    ArmFirstUseRuntimeHeader header;
    uint32_t marker = ARM_FIRST_USE_RUNTIME_COMMIT_MARKER;

    arm_first_use_runtime_make_header(&header);
    if (arm_w25q128_partition_write(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                    0U,
                                    &header,
                                    sizeof(header),
                                    1000) != ARM_W25Q128_OK) {
        return -1;
    }

    if (arm_w25q128_partition_write(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                    (uint32_t)offsetof(ArmFirstUseRuntimeHeader, commit_marker),
                                    &marker,
                                    sizeof(marker),
                                    1000) != ARM_W25Q128_OK) {
        return -1;
    }

    return 0;
}

static int arm_first_use_runtime_validate_header(const ArmFirstUseRuntimeHeader *header)
{
    uint32_t expected_bits = arm_first_use_runtime_expected_bitmap_bits();

    if (header == NULL) {
        return -1;
    }

    if ((header->magic != ARM_FIRST_USE_RUNTIME_MAGIC) ||
        (header->commit_marker != ARM_FIRST_USE_RUNTIME_COMMIT_MARKER)) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL, "invalid_header");
        return -1;
    }

    if ((header->version != ARM_FIRST_USE_RUNTIME_FORMAT_VERSION) ||
        (header->granularity_sec != ARM_FIRST_USE_RUNTIME_GRANULARITY_SEC) ||
        (header->bitmap_offset != (uint32_t)sizeof(ArmFirstUseRuntimeHeader)) ||
        (header->bitmap_bits != expected_bits) ||
        (header->header_size != (uint32_t)sizeof(ArmFirstUseRuntimeHeader))) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL, "invalid_format");
        return -1;
    }

    if (header->header_crc != arm_first_use_runtime_header_crc(header)) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL, "header_crc_failed");
        return -1;
    }

    return 0;
}

static int arm_first_use_runtime_scan_bitmap(void)
{
    uint8_t buffer[ARM_FIRST_USE_RUNTIME_READ_CHUNK];
    uint32_t byte_count;
    uint32_t byte_index = 0U;
    uint32_t cleared_bits = 0U;
    bool found_uncleared = false;

    if (g_runtime_bitmap_bits == 0U) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL, "invalid_bitmap");
        return -1;
    }

    byte_count = (g_runtime_bitmap_bits + 7U) / 8U;
    while ((byte_index < byte_count) && !found_uncleared) {
        uint32_t len = byte_count - byte_index;
        if (len > sizeof(buffer)) {
            len = sizeof(buffer);
        }

        if (arm_w25q128_partition_read(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                       g_runtime_bitmap_offset + byte_index,
                                       buffer,
                                       len,
                                       1000) != ARM_W25Q128_OK) {
            arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_read_failed");
            return -1;
        }

        for (uint32_t i = 0U; (i < len) && !found_uncleared; i++) {
            for (uint32_t bit = 0U; bit < 8U; bit++) {
                uint32_t absolute_bit = (byte_index + i) * 8U + bit;
                uint8_t mask = (uint8_t)(1U << bit);

                if (absolute_bit >= g_runtime_bitmap_bits) {
                    found_uncleared = true;
                    break;
                }

                if ((buffer[i] & mask) == 0U) {
                    cleared_bits++;
                } else {
                    found_uncleared = true;
                    break;
                }
            }
        }

        byte_index += len;
    }

    g_runtime_minutes = cleared_bits;
    g_runtime_next_bit = cleared_bits;
    if (g_runtime_minutes >= g_runtime_bitmap_bits) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_SATURATED, "none");
    } else {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_NORMAL, "none");
    }

    return 0;
}

static int arm_first_use_runtime_load_or_create(void)
{
    ArmFirstUseRuntimeHeader header;
    bool header_erased;
    bool sector_erased = false;

    if (!arm_w25q128_partition_is_ready()) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "partition_unavailable");
        return -1;
    }

    if (arm_w25q128_partition_read(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                   0U,
                                   &header,
                                   sizeof(header),
                                   1000) != ARM_W25Q128_OK) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_read_failed");
        return -1;
    }

    header_erased = arm_first_use_runtime_buffer_erased((const uint8_t *)&header, sizeof(header));
    if (header_erased) {
        if (arm_first_use_runtime_sector_is_erased(&sector_erased) != 0) {
            arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_read_failed");
            return -1;
        }

        if (!sector_erased) {
            arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL, "invalid_blank_header");
            return -1;
        }

        if (arm_first_use_runtime_write_header() != 0) {
            arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_write_failed");
            return -1;
        }

        if (arm_w25q128_partition_read(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                       0U,
                                       &header,
                                       sizeof(header),
                                       1000) != ARM_W25Q128_OK) {
            arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_read_failed");
            return -1;
        }
    }

    if (arm_first_use_runtime_validate_header(&header) != 0) {
        g_runtime_format_version = 0U;
        return -1;
    }

    g_runtime_format_version = header.version;
    g_runtime_bitmap_offset = header.bitmap_offset;
    g_runtime_bitmap_bits = header.bitmap_bits;

    return arm_first_use_runtime_scan_bitmap();
}

static int arm_first_use_runtime_commit_next_minute(void)
{
    uint32_t byte_offset;
    uint8_t current_byte;
    uint8_t mask;
    uint8_t new_byte;
    int ret;

    if (g_runtime_next_bit >= g_runtime_bitmap_bits) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_SATURATED, "none");
        return 0;
    }

    byte_offset = g_runtime_bitmap_offset + (g_runtime_next_bit / 8U);
    mask = (uint8_t)(1U << (g_runtime_next_bit % 8U));

    ret = arm_w25q128_partition_read(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                     byte_offset,
                                     &current_byte,
                                     sizeof(current_byte),
                                     1000);
    if (ret == ARM_W25Q128_ERR_BUSY) {
        return ARM_FIRST_USE_RUNTIME_COMMIT_BUSY;
    }
    if (ret != ARM_W25Q128_OK) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_read_failed");
        return -1;
    }

    if ((current_byte & mask) == 0U) {
        if (arm_first_use_runtime_scan_bitmap() != 0) {
            return -1;
        }
        return 0;
    }

    new_byte = (uint8_t)(current_byte & (uint8_t)(~mask));
    ret = arm_w25q128_partition_write(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                      byte_offset,
                                      &new_byte,
                                      sizeof(new_byte),
                                      1000);
    if (ret == ARM_W25Q128_ERR_BUSY) {
        return ARM_FIRST_USE_RUNTIME_COMMIT_BUSY;
    }
    if (ret != ARM_W25Q128_OK) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_write_failed");
        return -1;
    }

    g_runtime_minutes++;
    g_runtime_next_bit++;
    if (g_runtime_next_bit >= g_runtime_bitmap_bits) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_SATURATED, "none");
    }

    return 0;
}

int arm_first_use_runtime_init(void)
{
    g_runtime_status = ARM_FIRST_USE_RUNTIME_STATUS_DISABLED;
    g_runtime_error_reason = "not_initialized";
    g_runtime_minutes = 0U;
    g_runtime_format_version = 0U;
    g_runtime_bitmap_offset = (uint32_t)sizeof(ArmFirstUseRuntimeHeader);
    g_runtime_bitmap_bits = arm_first_use_runtime_expected_bitmap_bits();
    g_runtime_next_bit = 0U;
    g_runtime_last_tick = 0U;
    g_runtime_pending_ms = 0U;
    g_runtime_tick_started = false;

    return arm_first_use_runtime_load_or_create();
}

void arm_first_use_runtime_tick(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if ((g_runtime_status != ARM_FIRST_USE_RUNTIME_STATUS_NORMAL) &&
        (g_runtime_status != ARM_FIRST_USE_RUNTIME_STATUS_SATURATED)) {
        return;
    }

    if (g_runtime_status == ARM_FIRST_USE_RUNTIME_STATUS_SATURATED) {
        return;
    }

    if (!g_runtime_tick_started) {
        g_runtime_tick_started = true;
        g_runtime_last_tick = now_ms;
        return;
    }

    elapsed_ms = now_ms - g_runtime_last_tick;
    g_runtime_last_tick = now_ms;
    g_runtime_pending_ms += elapsed_ms;

    while ((g_runtime_pending_ms >= ARM_FIRST_USE_RUNTIME_MINUTE_MS) &&
           (g_runtime_status == ARM_FIRST_USE_RUNTIME_STATUS_NORMAL)) {
        int ret = arm_first_use_runtime_commit_next_minute();
        if (ret == ARM_FIRST_USE_RUNTIME_COMMIT_BUSY) {
            break;
        }
        if (ret != 0) {
            break;
        }
        g_runtime_pending_ms -= ARM_FIRST_USE_RUNTIME_MINUTE_MS;
    }
}

void arm_first_use_runtime_get_info(ArmFirstUseRuntimeInfo *info)
{
    if (info == NULL) {
        return;
    }

    info->status = g_runtime_status;
    info->runtime_minutes = g_runtime_minutes;
    info->record_format_version = g_runtime_format_version;
    info->firmware_commit_id = GIT_COMMIT_ID;
    info->firmware_build_time = g_firmware_build_time;
    info->error_reason = g_runtime_error_reason;
}

const char *arm_first_use_runtime_status_string(ArmFirstUseRuntimeStatus status)
{
    switch (status) {
    case ARM_FIRST_USE_RUNTIME_STATUS_NORMAL:
        return "normal";
    case ARM_FIRST_USE_RUNTIME_STATUS_SATURATED:
        return "saturated";
    case ARM_FIRST_USE_RUNTIME_STATUS_ABNORMAL:
        return "abnormal";
    case ARM_FIRST_USE_RUNTIME_STATUS_DISABLED:
        return "disabled";
    default:
        return "unknown";
    }
}

int arm_first_use_runtime_format(void)
{
    if (!arm_w25q128_partition_is_ready()) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "partition_unavailable");
        return -1;
    }

    arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "formatting");
    if (arm_w25q128_partition_erase(ARM_W25Q128_MODULE_FIRST_USE_RUNTIME,
                                    0U,
                                    W25Q128_SECTOR_SIZE,
                                    1000) != ARM_W25Q128_OK) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_erase_failed");
        return -1;
    }

    if (arm_first_use_runtime_write_header() != 0) {
        arm_first_use_runtime_set_error(ARM_FIRST_USE_RUNTIME_STATUS_DISABLED, "flash_write_failed");
        return -1;
    }

    return arm_first_use_runtime_init();
}
