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
static unsigned fail_read;
static uint32_t corrupt_addr;
static uint8_t corrupt_pending;
static uint8_t last_progress;
static uint8_t saw_completion;
static uint32_t remote_freq;
static unsigned remote_freq_calls;
static uint32_t clock_ticks;
static const uint8_t *async_source;
static uint32_t async_address;
static unsigned async_steps;
static unsigned async_starts;

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
    if (reads == fail_read) return 0;
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
uint8_t w25q_write_sector_start(uint32_t addr, const uint8_t *src) {
    if (async_steps) return 0;
    assert(!(addr % MSC_SECTOR_SIZE) && src);
    async_source = src;
    async_address = addr;
    async_steps = 4;
    ++async_starts;
    return 1;
}
w25q_write_status_t w25q_write_sector_poll(void) {
    assert(async_steps);
    if (--async_steps) return W25Q_WRITE_BUSY;
    return w25q_write_sector(async_address, async_source) ? W25Q_WRITE_DONE : W25Q_WRITE_ERROR;
}
uint8_t w25q_write_sector_busy(void) { return async_steps != 0; }
uint32_t board_cycle_counter_read(void) { return clock_ticks; }
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
    msc_out_pending = raw_flush_pending = 0;
    usb_suspended = 0;
    async_steps = async_starts = 0;
    clock_ticks = 0;
    raw_write_loaded = raw_write_dirty = raw_write_failed = 0;
    raw_update_scan_pending = raw_update_streaming = msc_update_candidate = 0;
    readable = valid_vectors = 1;
    update_busy = corrupt_pending = saw_completion = last_progress = 0;
    writes = fail_write = fail_read = 0;
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

static void msc_test_setup(void) {
    (void)setup(512, FAT_TYPE_12, 1);
    usb_ready = usb_configured = 1;
    usb_suspended = 0;
    usb_last_frame = 0;
    usb_last_frame_ticks = 0;
    msc_unit_attention = 0;
    msc_disk_sectors = sizeof(disk) / MSC_SECTOR_SIZE;
    msc_abort_transfer();
    REG32(NVIC_ISER0) = USB_IRQ_MASK;
}

static void msc_test_packet(const uint8_t *data, uint16_t len) {
    unsigned before_reads = reads;
    unsigned before_writes = writes;
    unsigned before_starts = async_starts;
    assert(!msc_out_pending);
    pma_write(USB_EP1_RX, data, len);
    USB_PMA16(1u * 8u + 6u) = len;
    ep1_out();
    assert(msc_out_pending);
    assert(reads == before_reads && writes == before_writes && async_starts == before_starts);
}

static void msc_test_cbw(uint8_t op, uint32_t lba, uint16_t blocks, uint32_t tag) {
    uint8_t cbw[31] = {0};
    put_le32(cbw, 0x43425355u);
    put_le32(cbw + 4, tag);
    put_le32(cbw + 8, op == 0x12 ? 36u : (uint32_t)blocks * MSC_SECTOR_SIZE);
    cbw[12] = op == 0x2A ? 0 : 0x80;
    cbw[14] = 10;
    cbw[15] = op;
    put_be32(cbw + 17, lba);
    cbw[22] = (uint8_t)(blocks >> 8);
    cbw[23] = (uint8_t)blocks;
    msc_test_packet(cbw, sizeof(cbw));
}

static void msc_test_write_data(uint8_t value) {
    uint8_t packet[64];
    memset(packet, value, sizeof(packet));
    for (unsigned i = 0; i < MSC_SECTOR_SIZE / sizeof(packet); ++i) {
        msc_test_packet(packet, sizeof(packet));
        usb_msc_poll();
        assert(!msc_out_pending);
    }
}

static void test_deferred_msc_write(void) {
    msc_test_setup();
    msc_test_cbw(0x2A, 20, 1, 0x12345678u);
    assert(msc_state == MSC_IDLE);
    usb_msc_poll();
    assert(msc_state == MSC_DISCARD_OUT_DATA);
    msc_test_write_data(0x5A);
    assert(raw_flush_pending && async_starts == 1 && !writes);
    assert(USB_PMA16(1u * 8u + 2u) == 0); // No successful CSW while flash is busy.
    assert(usb_msc_save_screenshot() == SCREENSHOT_SAVE_BUSY);
    for (unsigned i = 0; i < 3; ++i) {
        usb_msc_poll(); // Each returns after one mock flash step.
        assert(raw_flush_pending && !writes && !disk[20u * MSC_SECTOR_SIZE]);
    }
    usb_msc_poll();
    assert(!raw_flush_pending && writes == 1 && msc_state == MSC_IDLE);
    uint8_t csw[13];
    pma_read(USB_EP1_TX, csw, sizeof(csw));
    assert(le32(csw) == 0x53425355u && le32(csw + 4) == 0x12345678u && csw[12] == MSC_CSW_OK);
    for (unsigned i = 0; i < MSC_SECTOR_SIZE; ++i) assert(disk[20u * MSC_SECTOR_SIZE + i] == 0x5A);

    unsigned previous_writes = writes;
    unsigned previous_starts = async_starts;
    raw_update_scan_pending = 0;
    msc_test_cbw(0x2A, 20, 1, 2);
    usb_msc_poll();
    msc_test_write_data(0x5A);
    assert(!raw_flush_pending && writes == previous_writes && async_starts == previous_starts);
    assert(!raw_update_scan_pending); // Identical host metadata needs neither erase nor rescan.
}

static void test_msc_write_failure(void) {
    msc_test_setup();
    fail_write = 1;
    msc_test_cbw(0x2A, 20, 1, 3);
    usb_msc_poll();
    msc_test_write_data(0x3C);
    while (raw_flush_pending) usb_msc_poll();
    assert(msc_discard_status == MSC_CSW_FAIL && msc_sense_key == 3 && msc_sense_asc == 0x0C);
    assert(!raw_update_scan_pending && disk[20u * MSC_SECTOR_SIZE] == 0);
}

static void test_msc_reset_during_write(void) {
    msc_test_setup();
    msc_test_cbw(0x2A, 20, 1, 4);
    usb_msc_poll();
    msc_test_write_data(0x77);
    assert(raw_flush_pending);
    msc_abort_transfer();
    msc_test_cbw(0x12, 0, 0, 5);
    for (unsigned i = 0; i < 3; ++i) {
        usb_msc_poll();
        assert(msc_out_pending && raw_flush_pending && msc_tag == 4);
        assert(raw_write_sector[0] == 0x77 && !writes);
    }
    usb_msc_poll();
    assert(!raw_flush_pending && !msc_out_pending && msc_tag == 5);
    assert(writes == 1 && disk[20u * MSC_SECTOR_SIZE] == 0x77);
    assert(USB_PMA16(1u * 8u + 2u) == 36); // New INQUIRY data, not the aborted write's CSW.
}

static void test_msc_short_packets(void) {
    msc_test_setup();
    msc_test_cbw(0x2A, 20, 2, 6);
    usb_msc_poll();
    uint8_t packet[63];
    memset(packet, 0x55, sizeof(packet));
    uint32_t remaining = 2u * MSC_SECTOR_SIZE;
    while (remaining) {
        uint16_t n = remaining < sizeof(packet) ? (uint16_t)remaining : sizeof(packet);
        msc_test_packet(packet, n);
        usb_msc_poll();
        while (raw_flush_pending || msc_out_pending) usb_msc_poll();
        remaining -= n;
    }
    assert(writes == 2 && msc_state == MSC_IDLE);
    for (unsigned i = 0; i < 2u * MSC_SECTOR_SIZE; ++i) assert(disk[20u * MSC_SECTOR_SIZE + i] == 0x55);
}

static void test_msc_read_error(void) {
    msc_test_setup();
    fail_read = reads + 1u;
    msc_test_cbw(0x28, 20, 1, 7);
    usb_msc_poll();
    assert(msc_state == MSC_SEND_CSW && msc_sense_key == 3 && msc_sense_asc == 0x11);
    assert(USB_PMA16(1u * 8u + 2u) == 0); // ZLP terminates the failed data phase.
    ep1_in();
    uint8_t csw[13];
    pma_read(USB_EP1_TX, csw, sizeof(csw));
    assert(csw[12] == MSC_CSW_FAIL && le32(csw + 8) == MSC_SECTOR_SIZE);
}

static void test_usb_link_activity(void) {
    msc_test_setup();
    assert(usb_cdc_is_configured());
    USB_FNR = 1;
    clock_ticks = 100;
    usb_link_poll();
    clock_ticks += USB_LINK_QUIET_CYCLES;
    usb_link_poll();
    assert(usb_configured && !usb_cdc_is_configured());
    USB_FNR = 2;
    usb_link_poll();
    assert(usb_cdc_is_configured()); // Resume without losing SET_CONFIGURATION.
    usb_suspended = 1;
    assert(!usb_cdc_is_configured());
    USB_FNR = 3;
    usb_link_poll();
    assert(usb_cdc_is_configured());
    usb_last_frame_ticks = UINT32_MAX - 100u;
    clock_ticks = USB_LINK_QUIET_CYCLES;
    usb_link_poll();
    assert(!usb_cdc_is_configured()); // Cycle-counter wrap is harmless.
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
    test_deferred_msc_write();
    test_msc_write_failure();
    test_msc_reset_during_write();
    test_msc_short_packets();
    test_msc_read_error();
    test_usb_link_activity();
    puts("USB MSC: cooperative durable writes, reset ownership, short packets, read errors and link expiry passed");
    puts("bootloader FAT export checks passed (" HW_TARGET_NAME ")");
}
