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
static unsigned reads;
static unsigned fail_write;
static uint32_t corrupt_addr;
static uint8_t corrupt_pending;
static uint8_t last_progress;
static uint8_t saw_completion;
static uint32_t remote_freq;
static unsigned remote_freq_calls;

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
    ++reads;
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
uint8_t fw_update_request_apply(void) { return 0; }
void fw_update_note_file(uint32_t base_lba, uint32_t size) { (void)base_lba; (void)size; }
void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len) {
    (void)lba; (void)sector_offset; (void)data; (void)len;
}
void ui_set_dmm_mode(uint8_t mode) { (void)mode; }
void ui_set_main_mode(uint8_t mode) { (void)mode; }
void ui_scope_set_running(uint8_t run) { (void)run; }
void ui_scope_trigger_auto(void) {}
void ui_scope_step_timebase(int8_t dir) { (void)dir; }
void ui_scope_step_vdiv(uint8_t ch, int8_t dir) { (void)ch; (void)dir; }
static unsigned screenshot_requests;
static uint32_t injected_keys;
void ui_request_screenshot(void) { ++screenshot_requests; }
void ui_inject_keys(uint32_t events) { injected_keys |= events; }
void ui_gen_set_output(uint8_t run) { (void)run; }
void ui_gen_set_frequency(uint32_t hz) { remote_freq = hz; ++remote_freq_calls; }
void ui_gen_set_wave(uint8_t wave) { (void)wave; }
void ui_gen_set_amplitude(uint8_t tenths_v) { (void)tenths_v; }
void ui_gen_set_duty(uint8_t duty_percent) { (void)duty_percent; }
uint8_t ui_gen_is_running(void) { return 0; }
uint32_t ui_gen_frequency(void) { return remote_freq; }
uint8_t ui_gen_wave(void) { return 0; }
uint8_t ui_gen_amplitude(void) { return 33; }
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

    // Numbering continues after the highest index, then fills gaps once IMG_99 exists.
    fat = setup(512, FAT_TYPE_12, 2);
    uint8_t *root = disk + fat.root_lba * fat.bytes_per_sector;
    static const char *const names[] = {"IMG_99  BMP", "IMG_01  BMP", "IMG_03  BMP"};
    for (unsigned i = 0; i < 3; ++i) {
        memcpy(root + 32u * (i + 1u), names[i], 11);
        root[32u * (i + 1u) + 11] = 0x20;
    }
    assert(usb_msc_store_screenshot());
    assert(!memcmp(root + 128, "IMG_02  BMP", 11));
    assert(usb_msc_store_screenshot());
    assert(!memcmp(root + 160, "IMG_04  BMP", 11));
    uint8_t used[13];
    memset(used, 0xFF, sizeof(used));
    assert(!raw_screenshot_next_index(used));
    used[50 / 8] &= (uint8_t)~(1u << (50 % 8));
    assert(raw_screenshot_next_index(used) == 50);
    memset(used, 0, sizeof(used));
    used[0] = 1;
    assert(raw_screenshot_next_index(used) == 1);
}

// A capture that cannot be written used to come back as a bare failure, so
// "the drive is full" -- the only one the user can act on -- looked exactly
// like a write error. Each cause now has to name itself.
static void test_screenshot_failure_reasons(void) {
    raw_fat_volume_t fat = setup(512, FAT_TYPE_12, 1);
    uint32_t free_kb = 0;
    uint32_t total_kb = 0;
    uint32_t cluster_bytes;

    reads = 0;
    assert(usb_msc_storage_free_kb(&free_kb, &total_kb));
    assert(reads < 8u);
    cluster_bytes = (uint32_t)fat.bytes_per_sector * fat.sectors_per_cluster;
    assert(total_kb == raw_fat_cluster_count(&fat) * cluster_bytes / 1024u);
    assert(total_kb > 0u);
    assert(free_kb == total_kb);   // nothing is allocated on a fresh volume

    // One capture takes its clusters out of the free count, and no more.
    assert(usb_msc_save_screenshot() == SCREENSHOT_SAVE_OK);
    assert(usb_msc_storage_free_kb(&free_kb, 0));
    assert(free_kb == (raw_fat_cluster_count(&fat) - SCREENSHOT_CLUSTER_COUNT) * cluster_bytes / 1024u);

    // Every cluster taken: full, and nothing written.
    fat = setup(512, FAT_TYPE_12, 1);
    for (uint32_t cluster = 2; cluster <= raw_fat_cluster_count(&fat) + 1u; ++cluster) {
        assert(raw_fat_write_entry_copy(&fat, 0, cluster, 0x0FFFu));
    }
    writes = 0;
    assert(usb_msc_save_screenshot() == SCREENSHOT_SAVE_FULL);
    assert(disk[fat.root_lba * fat.bytes_per_sector + 32] == 0);
    assert(usb_msc_storage_free_kb(&free_kb, 0) && free_kb == 0);

    // Every root slot taken, but space to spare: a different complaint.
    fat = setup(512, FAT_TYPE_12, 1);
    uint8_t *root = disk + fat.root_lba * fat.bytes_per_sector;
    uint32_t root_slots = (fat.root_dir_sectors * fat.bytes_per_sector) / 32u;
    for (uint32_t i = 0; i < root_slots; ++i) {
        memcpy(root + 32u * i, "OTHER      ", 11);
        root[32u * i + 11] = 0x20;
    }
    assert(usb_msc_save_screenshot() == SCREENSHOT_SAVE_NO_SLOT);

    (void)setup(512, FAT_TYPE_12, 1);
    put_le32(disk + 32, (uint32_t)(sizeof(disk) / 512u) + 1u);
    assert(!usb_msc_storage_free_kb(&free_kb, &total_kb));
}

static void test_storage_busy(void) {
    raw_fat_volume_t fat = setup(512, FAT_TYPE_12, 1);
    usb_ready = 1;
    // A screenshot between packets of a host WRITE must not reuse its pending sector.
    msc_state = MSC_DISCARD_OUT_DATA;
    raw_write_lba = 7;
    raw_write_loaded = raw_write_dirty = 1;
    memset(raw_write_sector, 0x5A, sizeof(raw_write_sector));
    assert(usb_msc_save_screenshot() == SCREENSHOT_SAVE_BUSY && !writes);
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

static void test_cdc_command_dispatch(void) {
    static const uint8_t command[] = "GEN:FREQ:2000\r\n";
    (void)setup(512, FAT_TYPE_12, 1);
    usb_ready = usb_configured = 1;
    remote_freq = remote_freq_calls = 0;
    cdc_rx_pending = 0;
    REG32(NVIC_ISER0) = USB_IRQ_MASK;
    pma_write(USB_EP3_RX, command, sizeof(command) - 1u);
    USB_PMA16(3u * 8u + 6u) = sizeof(command) - 1u;
    cdc_receive_command();
    assert(cdc_rx_pending && !remote_freq_calls);
    usb_msc_poll();
    assert(!cdc_rx_pending && remote_freq_calls == 1 && remote_freq == 2000u);
    usb_msc_poll();
    assert(remote_freq_calls == 1);
    assert(handle_cdc_gen_command("GEN:FREQ:4294967295"));
    assert(remote_freq == UINT32_MAX && remote_freq_calls == 2);
    assert(handle_cdc_gen_command("GEN:FREQ:4294967296"));
    assert(handle_cdc_gen_command("GEN:FREQ:10junk"));
    assert(remote_freq_calls == 2);

    cdc_tx_tail = 0;
    cdc_tx_head = CDC_TX_BUF_SIZE - 5u;
    cdc_tx_busy = 1;
    usb_cdc_send_bytes(command, sizeof(command) - 1u);
    assert(cdc_tx_head == CDC_TX_BUF_SIZE - 5u && cdc_tx_tail == 0);
    usb_configured = 0;
    usb_cdc_send_bytes(command, 1);
    assert(cdc_tx_head == CDC_TX_BUF_SIZE - 5u);
    // SHOT and KEY: are the two commands that act on the interface rather
    // than on the generator, so they are the ones that must reach the UI.
    screenshot_requests = 0;
    injected_keys = 0;
    handle_cdc_rx_command("SHOT");
    assert(screenshot_requests == 1u);
    handle_cdc_rx_command("SHOTGUN");
    assert(screenshot_requests == 1u);
    handle_cdc_rx_command("KEY:MENU");
    handle_cdc_rx_command("KEY:AUTO_LONG");
    assert(injected_keys != 0u);
    // An unknown key name must do nothing at all rather than guess.
    uint32_t before = injected_keys;
    handle_cdc_rx_command("KEY:NOSUCHKEY");
    handle_cdc_rx_command("KEY:OK\runexpected");
    assert(injected_keys == before);

    puts("USB CDC deferred dispatch, numeric bounds and whole-line overflow checks passed");
}

int main(void) {
    test_export(512, FAT_TYPE_12, 1);
    test_export(4096, FAT_TYPE_12, 2);
    test_export(512, FAT_TYPE_16, 1);
    test_export(512, FAT_TYPE_32, 1);
    test_failures();
    test_screenshot();
    test_screenshot_failure_reasons();
    test_storage_busy();
    test_cdc_command_dispatch();
    puts("bootloader FAT export checks passed (" HW_TARGET_NAME ")");
}
