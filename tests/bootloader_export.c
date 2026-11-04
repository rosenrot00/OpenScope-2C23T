#include "../src/hw.h"
#include "../src/bootloader.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t registers[1024];
static uint16_t registers16[1024];
static uint32_t *test_reg(uint32_t addr) {
    assert(addr < FLASH_R_BASE || addr >= FLASH_R_BASE + 0x100u);
    return &registers[(addr / 4u) % 1024u];
}
#undef REG32
#define REG32(addr) (*test_reg(addr))
#undef REG16
#define REG16(addr) (registers16[((addr) / 2u) % 1024u])
#include "../src/usb_msc.c"

static uint8_t disk[40u * 1024u * 1024u];
static uint8_t source_data[BOOTLOADER_SIZE];
static uint8_t readable;
static uint8_t valid_vectors;
static uint8_t update_busy;
static unsigned writes;
static unsigned fail_write;
static uint32_t corrupt_addr;
static uint8_t corrupt_pending;
static uint8_t last_progress;
static uint8_t saw_completion;

uint8_t bootloader_readable(void) { return readable; }
uint8_t bootloader_vectors_valid(const uint8_t *data) { assert(data == source_data); return valid_vectors; }
const uint8_t *bootloader_source(void) { return source_data; }
uint32_t bootloader_crc32(const uint8_t *data, uint32_t len) {
    assert(data == source_data && len == BOOTLOADER_SIZE); return 0x12345678u;
}
uint16_t bootloader_describe(char *out, uint16_t capacity, uint32_t crc, const char *version) {
    assert(capacity >= 64u && crc == 0x12345678u && !strcmp(version, "v2026.10.1"));
    strcpy(out, "bootloader metadata\r\nCRC32: 12345678\r\n");
    return (uint16_t)strlen(out);
}
void w25q_init(void) {}
uint32_t w25q_capacity_bytes(void) { return sizeof(disk); }
uint8_t w25q_read(uint32_t addr, uint8_t *dst, uint16_t len) {
    assert(addr <= sizeof(disk) && len <= sizeof(disk) - addr);
    memcpy(dst, disk + addr, len);
    if (corrupt_pending && addr == corrupt_addr) { dst[0] ^= 1u; corrupt_pending = 0; }
    return 1;
}
uint8_t w25q_write_sector(uint32_t addr, const uint8_t *src) {
    assert(!(addr % MSC_SECTOR_SIZE) && addr <= sizeof(disk) - MSC_SECTOR_SIZE);
    if (++writes == fail_write) return 0;
    memcpy(disk + addr, src, MSC_SECTOR_SIZE);
    return 1;
}
void delay_ms(uint32_t ms) { (void)ms; }
void fw_update_status(fw_update_status_t *status) {
    memset(status, 0, sizeof(*status));
    if (update_busy) status->state = FW_UPDATE_STATE_STAGING;
}
void fw_update_clear(void) {}
void fw_update_note_file(uint32_t base_lba, uint32_t size) { (void)base_lba; (void)size; }
void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len) {
    (void)lba; (void)sector_offset; (void)data; (void)len;
}
uint8_t screenshot_render_current(uint32_t offset, uint8_t *dst, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) dst[i] = (uint8_t)(offset + i);
    return 1;
}
static void progress(uint8_t percent) {
    assert(percent >= last_progress && percent <= 100);
    last_progress = percent;
    saw_completion |= percent == 100;
}

static raw_fat_volume_t setup(uint16_t sector_bytes, uint8_t type, uint8_t spc) {
    raw_fat_volume_t fat;
    memset(disk, 0, sizeof(disk));
    memset(registers, 0, sizeof(registers));
    memset(registers16, 0, sizeof(registers16));
    usb_ready = usb_configured = 0;
    msc_state = MSC_IDLE;
    raw_write_loaded = raw_write_dirty = raw_write_failed = 0;
    raw_update_scan_pending = raw_update_streaming = msc_update_candidate = 0;
    readable = valid_vectors = 1;
    update_busy = corrupt_pending = saw_completion = last_progress = 0;
    writes = fail_write = 0;
    for (unsigned i = 0; i < sizeof(source_data); ++i) source_data[i] = (uint8_t)(i * 31u);
    uint32_t clusters = type == FAT_TYPE_12 ? 600u : (type == FAT_TYPE_16 ? 4200u : 66000u);
    // FAT32 uses a small synthetic BPB but enough clusters to select the type.
    uint32_t fat_sectors = type == FAT_TYPE_32 ? 520u : 24u;
    uint32_t root_sectors = type == FAT_TYPE_32 ? 0 : 2;
    uint32_t total = 1u + 2u * fat_sectors + root_sectors + clusters * spc;
    disk[0] = 0xEB;
    put_le16(disk + 11, sector_bytes);
    disk[13] = spc;
    put_le16(disk + 14, 1);
    disk[16] = 2;
    put_le16(disk + 17, (uint16_t)(root_sectors * sector_bytes / 32u));
    put_le32(disk + 32, total);
    if (type == FAT_TYPE_32) {
        put_le32(disk + 36, fat_sectors);
        put_le32(disk + 44, 2);
    } else {
        put_le16(disk + 22, (uint16_t)fat_sectors);
    }
    disk[510] = 0x55; disk[511] = 0xAA;
    assert(raw_fat_mount(&fat) && fat.type == type);
    if (type == FAT_TYPE_32) {
        for (uint8_t copy = 0; copy < fat.fat_count; ++copy)
            assert(raw_fat_write_entry_copy(&fat, copy, 2, 0x0FFFFFFFu));
    }
    uint8_t *root = disk + fat.root_lba * sector_bytes;
    memcpy(root, "KEEP    CSV", 11);
    root[11] = 0x20;
    // Garbage after the end marker must not become visible after export.
    memset(root + 96, 0xAB, 32);
    writes = 0;
    return fat;
}

static void check_file(const raw_fat_volume_t *fat, const uint8_t *entry,
                       const uint8_t *expected, uint32_t size) {
    assert(le32_at(entry + 28) == size && entry[12] == 0x18);
    uint32_t cluster = (uint32_t)entry[26] | ((uint32_t)entry[27] << 8) |
        ((uint32_t)entry[20] << 16) | ((uint32_t)entry[21] << 24);
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t bytes = (uint32_t)fat->bytes_per_sector * fat->sectors_per_cluster;
        if (bytes > size - offset) bytes = size - offset;
        assert(!memcmp(disk + raw_fat_cluster_lba(fat, cluster) * fat->bytes_per_sector,
                       expected + offset, bytes));
        offset += bytes;
        assert(raw_fat_next_cluster(fat, cluster, &cluster));
    }
    assert(raw_fat_cluster_is_eoc(fat, cluster));
}

static void test_export(uint16_t sector_bytes, uint8_t type, uint8_t spc) {
    raw_fat_volume_t fat = setup(sector_bytes, type, spc);
    usb_ready = usb_configured = 1;
    assert(usb_msc_export_bootloader("v2026.10.1", progress) == BOOT_EXPORT_OK);
    assert(usb_ready && !usb_configured && saw_completion);
    uint8_t *root = disk + fat.root_lba * sector_bytes;
    assert(!memcmp(root, "KEEP    CSV", 11));
    assert(!memcmp(root + 32, BOOTLOADER_FAT_NAME, 11));
    assert(!memcmp(root + 64, BOOTLOADER_FAT_INFO_NAME, 11));
    assert(root[96] == 0);
    assert(!root_entry_name_has_f2c23t(root + 32));
    raw_fat_file_t update;
    assert(!raw_fat_find_update_file(&fat, &update));
    check_file(&fat, root + 32, source_data, BOOTLOADER_SIZE);
    const char *info = "bootloader metadata\r\nCRC32: 12345678\r\n";
    check_file(&fat, root + 64, (const uint8_t *)info, (uint32_t)strlen(info));
    for (uint32_t off = 0; off < fat.sectors_per_fat * sector_bytes; ++off)
        assert(disk[fat.fat_lba * sector_bytes + off] ==
               disk[(fat.fat_lba + fat.sectors_per_fat) * sector_bytes + off]);
    source_data[9] ^= 0x5Au;
    last_progress = saw_completion = 0;
    assert(usb_msc_export_bootloader("v2026.10.1", progress) == BOOT_EXPORT_OK);
    check_file(&fat, root + 32, source_data, BOOTLOADER_SIZE);
    assert(saw_completion && !memcmp(root, "KEEP    CSV", 11));
    assert(root[96] == 0);
    uint32_t allocated = 0;
    for (uint32_t cluster = 2; cluster <= raw_fat_cluster_count(&fat) + 1u; ++cluster) {
        uint32_t value;
        assert(raw_fat_next_cluster(&fat, cluster, &value));
        allocated += value != 0;
    }
    uint32_t cluster_bytes = (uint32_t)fat.bytes_per_sector * fat.sectors_per_cluster;
    assert(allocated == (BOOTLOADER_SIZE + cluster_bytes - 1u) / cluster_bytes + 1u +
           (fat.type == FAT_TYPE_32));
}

static void test_failures(void) {
    raw_fat_volume_t fat = setup(512, FAT_TYPE_12, 1);
    readable = 0;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_PROTECTED && !writes);
    readable = 1; valid_vectors = 0;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_INVALID && !writes);
    valid_vectors = 1; update_busy = 1;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_BUSY && !writes);
    update_busy = 0; msc_state = MSC_SEND_CSW;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_BUSY && !writes);
    msc_state = MSC_IDLE; disk[510] = 0;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_STORAGE_ERROR && !writes);
    fat = setup(512, FAT_TYPE_12, 1);
    fail_write = 2;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_STORAGE_ERROR);
    uint8_t *root = disk + fat.root_lba * fat.bytes_per_sector;
    assert(root[32] == 0);
    fat = setup(512, FAT_TYPE_12, 1);
    corrupt_addr = raw_fat_cluster_lba(&fat, 2) * fat.bytes_per_sector;
    corrupt_pending = 1;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_VERIFY_ERROR);
    assert(disk[fat.root_lba * fat.bytes_per_sector + 32] == 0);
    fat = setup(512, FAT_TYPE_12, 1);
    for (uint32_t cluster = 2; cluster <= raw_fat_cluster_count(&fat) + 1u; ++cluster)
        assert(raw_fat_write_entry_copy(&fat, 0, cluster, 0x0FFFu));
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_NO_SPACE);
    // Fail once during FAT publication; rollback must free all new clusters.
    fat = setup(512, FAT_TYPE_12, 1);
    fail_write = BOOTLOADER_SIZE / 512u + 3u;
    assert(usb_msc_export_bootloader("v2026.10.1", 0) == BOOT_EXPORT_STORAGE_ERROR);
    assert(disk[fat.root_lba * fat.bytes_per_sector + 32] == 0);
    for (uint32_t cluster = 2; cluster <= raw_fat_cluster_count(&fat) + 1u; ++cluster) {
        uint32_t value;
        assert(raw_fat_next_cluster(&fat, cluster, &value) && !value);
    }
}

static void test_screenshot(void) {
    raw_fat_volume_t fat = setup(512, FAT_TYPE_12, 1);
    assert(usb_msc_store_screenshot());
    uint8_t *entry = disk + fat.root_lba * fat.bytes_per_sector + 32;
    assert(!memcmp(entry, "IMG_01  BMP", 11));
    uint8_t *expected = malloc(SCREENSHOT_FILE_SIZE);
    assert(expected);
    for (uint32_t i = 0; i < SCREENSHOT_FILE_SIZE; ++i) expected[i] = (uint8_t)i;
    check_file(&fat, entry, expected, SCREENSHOT_FILE_SIZE);
    free(expected);
}

static void test_storage_busy(void) {
    raw_fat_volume_t fat = setup(512, FAT_TYPE_12, 1);
    usb_ready = 1;
    // A screenshot between packets of a host WRITE must not reuse its pending sector.
    msc_state = MSC_DISCARD_OUT_DATA;
    raw_write_lba = 7;
    raw_write_loaded = raw_write_dirty = 1;
    memset(raw_write_sector, 0x5A, sizeof(raw_write_sector));
    assert(!usb_msc_store_screenshot() && !writes);
    assert(raw_write_loaded && raw_write_dirty && raw_write_lba == 7);
    for (uint16_t i = 0; i < sizeof(raw_write_sector); ++i) assert(raw_write_sector[i] == 0x5A);
    assert(disk[fat.root_lba * fat.bytes_per_sector + 32] == 0);
    assert(REG32(0xE000E180u) == 1u << 20 && REG32(NVIC_ISER0) == 1u << 20);
    raw_update_scan_pending = 1;
    msc_update_idle_ms = MSC_UPDATE_IDLE_APPLY_MS;
    usb_msc_poll();
    assert(raw_update_scan_pending);
    msc_state = MSC_IDLE;
    raw_write_loaded = raw_write_dirty = 0;
    usb_msc_poll();
    assert(!raw_update_scan_pending && !msc_update_candidate);
    REG32(0xE000E180u) = REG32(NVIC_ISER0) = 0;
    assert(usb_msc_store_screenshot() && writes);
    assert(REG32(0xE000E180u) == 1u << 20 && REG32(NVIC_ISER0) == 1u << 20);
    assert(!memcmp(disk + fat.root_lba * fat.bytes_per_sector + 32, "IMG_01  BMP", 11));
    usb_ready = 0;
}

int main(void) {
    test_export(512, FAT_TYPE_12, 1);
    test_export(4096, FAT_TYPE_12, 2);
    test_export(512, FAT_TYPE_16, 1);
    test_export(512, FAT_TYPE_32, 1);
    test_failures();
    test_screenshot();
    test_storage_busy();
    puts("bootloader FAT export checks passed (" HW_TARGET_NAME ")");
}
