#include "usb_msc.h"

#include "board.h"
#include "fw_update.h"
#include "hw.h"
#include "screenshot.h"
#include "w25q.h"

#include <stddef.h>
#include <stdint.h>

enum {
    USB_EP0_SIZE = 64,
    USB_EP1_SIZE = 64,
    USB_EP0_TX = 0x0040,
    USB_EP0_RX = 0x0080,
    USB_EP1_TX = 0x00C0,
    USB_EP1_RX = 0x0100,

    USB_ISTR_CTR = 0x8000,
    USB_ISTR_RESET = 0x0400,
    USB_ISTR_SUSP = 0x0800,
    USB_ISTR_WKUP = 0x1000,

    USB_CNTR_FRES = 0x0001,
    USB_CNTR_PDWN = 0x0002,
    USB_CNTR_CTRM = 0x8000,
    USB_CNTR_RESETM = 0x0400,
    USB_CNTR_SUSPM = 0x0800,
    USB_CNTR_WKUPM = 0x1000,

    USB_DADDR_EF = 0x80,

    USB_EP_CTR_RX = 0x8000,
    USB_EP_DTOG_RX = 0x4000,
    USB_EP_STAT_RX = 0x3000,
    USB_EP_SETUP = 0x0800,
    USB_EP_TYPE = 0x0600,
    USB_EP_KIND = 0x0100,
    USB_EP_CTR_TX = 0x0080,
    USB_EP_DTOG_TX = 0x0040,
    USB_EP_STAT_TX = 0x0030,
    USB_EP_ADDR = 0x000F,
    USB_EPREG_MASK = USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_TYPE |
                     USB_EP_KIND | USB_EP_CTR_TX | USB_EP_ADDR,

    USB_EP_TYPE_BULK = 0x0000,
    USB_EP_TYPE_CONTROL = 0x0200,
    USB_STAT_DISABLED = 0,
    USB_STAT_STALL = 1,
    USB_STAT_NAK = 2,
    USB_STAT_VALID = 3,

    MSC_SECTOR_SIZE = 4096,
    FAT_TYPE_12 = 12,
    FAT_TYPE_16 = 16,
    FAT_TYPE_32 = 32,
    RAW_UPDATE_STAGE_BASE_LBA = 1,

    MSC_CSW_OK = 0,
    MSC_CSW_FAIL = 1,
    MSC_UPDATE_IDLE_APPLY_MS = 1000,
};

enum UsbEp0State {
    EP0_IDLE,
    EP0_TX_DATA,
    EP0_TX_STATUS,
    EP0_WAIT_OUT_STATUS,
};

enum MscState {
    MSC_IDLE,
    MSC_SEND_SMALL_DATA,
    MSC_SEND_READ_DATA,
    MSC_SEND_CSW,
    MSC_DISCARD_OUT_DATA,
};

static const uint8_t usb_device_desc[] = {
    18, 1, 0x00, 0x02, 0x00, 0x00, 0x00, USB_EP0_SIZE,
    0x3C, 0x2E, 0x20, 0x57, 0x00, 0x02, 1, 2, 3, 1,
};

static const uint8_t usb_config_desc[] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,
    7, 5, 0x81, 0x02, USB_EP1_SIZE, 0, 0,
    7, 5, 0x01, 0x02, USB_EP1_SIZE, 0, 0,
};

static const uint8_t usb_lang_desc[] = {4, 3, 0x09, 0x04};
static const char usb_manufacturer[] = "F2C23T";
static const char usb_product[] = "Runtime Storage";
static const char usb_serial[] = "0001";

static volatile enum UsbEp0State ep0_state;
static const uint8_t *ep0_ptr;
static uint16_t ep0_left;
static uint8_t ep0_pending_address;
static uint8_t ep0_address_pending;
static uint8_t usb_configured;
static uint8_t usb_ready;

static enum MscState msc_state;
static uint8_t msc_small[64];
static uint8_t msc_sense_key;
static uint8_t msc_sense_asc;
static uint8_t msc_csw_status;
static uint32_t msc_tag;
static uint32_t msc_residue;
static uint32_t msc_data_len;
static uint32_t msc_data_pos;
static uint32_t msc_read_lba;
static uint32_t msc_read_remaining;
static uint16_t msc_sector_pos;
static uint32_t msc_write_lba;
static uint32_t msc_write_pos;
static uint32_t msc_discard_remaining;
static uint8_t msc_discard_status;
static uint8_t msc_root_shadow[MSC_SECTOR_SIZE];
static uint8_t msc_update_candidate;
static uint8_t msc_unit_attention;
static uint16_t msc_update_idle_ms;
static uint32_t msc_disk_sectors;
static uint8_t raw_write_sector[MSC_SECTOR_SIZE];
static uint32_t raw_write_lba;
static uint8_t raw_write_loaded;
static uint8_t raw_write_dirty;
static uint8_t raw_write_failed;
static uint8_t raw_update_scan_pending;
static uint8_t raw_update_streaming;
static uint32_t raw_update_dir_byte_addr;
static uint32_t raw_screenshot_clusters[SCREENSHOT_CLUSTER_COUNT];

typedef struct {
    uint8_t valid;
    uint8_t type;
    uint8_t sectors_per_cluster;
    uint8_t fat_count;
    uint16_t bytes_per_sector;
    uint32_t part_lba;
    uint32_t fat_lba;
    uint32_t sectors_per_fat;
    uint32_t root_lba;
    uint32_t root_dir_sectors;
    uint32_t root_cluster;
    uint32_t data_lba;
    uint32_t total_sectors;
} raw_fat_volume_t;

typedef struct {
    uint32_t first_cluster;
    uint32_t size;
    uint32_t dir_byte_addr;
} raw_fat_file_t;

static void buf_zero(uint8_t *dst, uint16_t len) {
    while (len--) {
        *dst++ = 0;
    }
}

static void buf_copy(uint8_t *dst, const uint8_t *src, uint16_t len) {
    while (len--) {
        *dst++ = *src++;
    }
}

static uint16_t min_u16(uint16_t a, uint16_t b) {
    return a < b ? a : b;
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void put_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static uint32_t le32(const uint8_t *p) {
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t pma_rx_count(uint16_t size) {
    if (size > 62u) {
        uint16_t blocks = (uint16_t)((size + 31u) / 32u);
        return (uint16_t)(0x8000u | ((blocks - 1u) << 10));
    }
    return (uint16_t)(((size + 1u) / 2u) << 10);
}

static void pma_write(uint16_t addr, const uint8_t *src, uint16_t len) {
    uint16_t i = 0;
    while (i < len) {
        uint16_t v = src[i++];
        if (i < len) {
            v |= (uint16_t)src[i++] << 8;
        }
        USB_PMA16(addr) = v;
        addr = (uint16_t)(addr + 2u);
    }
}

static void pma_read(uint16_t addr, uint8_t *dst, uint16_t len) {
    uint16_t i = 0;
    while (i < len) {
        uint16_t v = USB_PMA16(addr);
        dst[i++] = (uint8_t)v;
        if (i < len) {
            dst[i++] = (uint8_t)(v >> 8);
        }
        addr = (uint16_t)(addr + 2u);
    }
}

static void pma_set_tx(uint8_t ep, uint16_t addr, uint16_t len) {
    uint16_t base = (uint16_t)(ep * 8u);
    USB_PMA16(base + 0u) = addr;
    USB_PMA16(base + 2u) = len;
}

static void pma_set_rx(uint8_t ep, uint16_t addr, uint16_t len) {
    uint16_t base = (uint16_t)(ep * 8u);
    USB_PMA16(base + 4u) = addr;
    USB_PMA16(base + 6u) = pma_rx_count(len);
}

static uint16_t pma_get_rx_count(uint8_t ep) {
    return (uint16_t)(USB_PMA16((uint16_t)(ep * 8u + 6u)) & 0x03FFu);
}

static void ep_set_type(uint8_t ep, uint16_t type) {
    USB_EPR(ep) = (uint16_t)(USB_EP_CTR_RX | USB_EP_CTR_TX | type | ep);
}

static void ep_set_tx_stat(uint8_t ep, uint8_t stat) {
    uint16_t reg = USB_EPR(ep);
    uint16_t desired = (uint16_t)((uint16_t)stat << 4);
    USB_EPR(ep) = (uint16_t)((reg & USB_EPREG_MASK) |
                             ((reg ^ desired) & USB_EP_STAT_TX));
}

static void ep_set_rx_stat(uint8_t ep, uint8_t stat) {
    uint16_t reg = USB_EPR(ep);
    uint16_t desired = (uint16_t)((uint16_t)stat << 12);
    USB_EPR(ep) = (uint16_t)((reg & USB_EPREG_MASK) |
                             ((reg ^ desired) & USB_EP_STAT_RX));
}

static void ep_clear_tx_ctr(uint8_t ep) {
    uint16_t reg = USB_EPR(ep);
    USB_EPR(ep) = (uint16_t)((reg & USB_EPREG_MASK & ~USB_EP_CTR_TX) |
                             USB_EP_CTR_RX);
}

static void ep_clear_rx_ctr(uint8_t ep) {
    uint16_t reg = USB_EPR(ep);
    USB_EPR(ep) = (uint16_t)((reg & USB_EPREG_MASK & ~USB_EP_CTR_RX) |
                             USB_EP_CTR_TX);
}

static void ep0_send_next(void) {
    uint16_t n = min_u16(ep0_left, USB_EP0_SIZE);
    if (n) {
        pma_write(USB_EP0_TX, ep0_ptr, n);
        ep0_ptr += n;
        ep0_left = (uint16_t)(ep0_left - n);
        ep0_state = EP0_TX_DATA;
    } else {
        ep0_state = EP0_TX_STATUS;
    }
    pma_set_tx(0, USB_EP0_TX, n);
    ep_set_tx_stat(0, USB_STAT_VALID);
}

static void ep0_send(const uint8_t *data, uint16_t len, uint16_t requested) {
    ep0_ptr = data;
    ep0_left = min_u16(len, requested);
    ep_set_rx_stat(0, USB_STAT_VALID);
    ep0_send_next();
}

static void ep0_send_zlp(void) {
    ep0_ptr = 0;
    ep0_left = 0;
    ep0_state = EP0_TX_STATUS;
    pma_set_tx(0, USB_EP0_TX, 0);
    ep_set_tx_stat(0, USB_STAT_VALID);
}

static void ep0_stall(void) {
    ep0_state = EP0_IDLE;
    ep_set_tx_stat(0, USB_STAT_STALL);
    ep_set_rx_stat(0, USB_STAT_STALL);
}

static uint16_t make_string_desc(uint8_t index) {
    const char *s;
    uint16_t i = 0;
    if (index == 0) {
        buf_copy(msc_small, usb_lang_desc, sizeof(usb_lang_desc));
        return sizeof(usb_lang_desc);
    }
    if (index == 1) {
        s = usb_manufacturer;
    } else if (index == 2) {
        s = usb_product;
    } else if (index == 3) {
        s = usb_serial;
    } else {
        return 0;
    }
    msc_small[1] = 3;
    while (s[i] && i < 31u) {
        msc_small[2u + i * 2u] = (uint8_t)s[i];
        msc_small[3u + i * 2u] = 0;
        ++i;
    }
    msc_small[0] = (uint8_t)(2u + i * 2u);
    return msc_small[0];
}

static void msc_set_sense(uint8_t key, uint8_t asc) {
    msc_sense_key = key;
    msc_sense_asc = asc;
}

static void msc_prepare_csw(uint8_t status, uint32_t residue) {
    put_le32(&msc_small[0], 0x53425355u);
    put_le32(&msc_small[4], msc_tag);
    put_le32(&msc_small[8], residue);
    msc_small[12] = status;
    msc_data_pos = 0;
    msc_data_len = 13;
    msc_state = MSC_SEND_CSW;
}

static void ep1_send_raw(const uint8_t *data, uint16_t len) {
    pma_write(USB_EP1_TX, data, len);
    pma_set_tx(1, USB_EP1_TX, len);
    ep_set_tx_stat(1, USB_STAT_VALID);
}

static void msc_send_next_in(void);

static void msc_start_small_in(uint16_t len, uint8_t status, uint32_t residue) {
    msc_data_pos = 0;
    msc_data_len = len;
    msc_csw_status = status;
    msc_residue = residue;
    msc_state = len ? MSC_SEND_SMALL_DATA : MSC_SEND_CSW;
    if (!len) {
        msc_prepare_csw(status, residue);
    }
    msc_send_next_in();
}

static uint8_t ascii_upper(uint8_t c) {
    if (c >= 'a' && c <= 'z') {
        return (uint8_t)(c - ('a' - 'A'));
    }
    return c;
}

static uint32_t le32_at(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t root_entry_ext_is_bin(const uint8_t *entry) {
    return ascii_upper(entry[8]) == 'B' &&
           ascii_upper(entry[9]) == 'I' &&
           ascii_upper(entry[10]) == 'N';
}

static uint8_t root_entry_name_has_f2c23t(const uint8_t *entry) {
    static const uint8_t needle[] = {'F', '2', 'C', '2', '3', 'T'};
    uint8_t matched = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        uint8_t c = ascii_upper(entry[i]);
        if (c == ' ') {
            break;
        }
        if (c == needle[matched]) {
            ++matched;
            if (matched == sizeof(needle)) {
                return 1;
            }
        } else {
            matched = c == needle[0] ? 1u : 0u;
        }
    }
    return 0;
}

static uint8_t lfn_char_match(uint8_t c, uint8_t *matched) {
    static const uint8_t needle[] = {'F', '2', 'C', '2', '3', 'T'};
    c = ascii_upper(c);
    if (c == needle[*matched]) {
        ++*matched;
        if (*matched == sizeof(needle)) {
            return 1;
        }
    } else {
        *matched = c == needle[0] ? 1u : 0u;
    }
    return 0;
}

static uint8_t lfn_entry_has_f2c23t(const uint8_t *entry, uint8_t *matched) {
    static const uint8_t offsets[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
    for (uint8_t i = 0; i < sizeof(offsets); ++i) {
        uint8_t off = offsets[i];
        uint16_t ch = (uint16_t)entry[off] | ((uint16_t)entry[off + 1u] << 8);
        if (ch == 0x0000u || ch == 0xFFFFu) {
            continue;
        }
        if (ch > 0x7Fu) {
            *matched = 0;
            continue;
        }
        if (lfn_char_match((uint8_t)ch, matched)) {
            return 1;
        }
    }
    return 0;
}

static uint8_t raw_write_flush(void) {
    uint8_t ok = 1;
    if (raw_write_loaded && raw_write_dirty) {
        ok = w25q_write_sector(raw_write_lba * (uint32_t)MSC_SECTOR_SIZE, raw_write_sector);
        raw_write_dirty = 0;
    }
    return ok;
}

static uint8_t is_power_of_two_u32(uint32_t value) {
    return value && ((value & (value - 1u)) == 0);
}

static uint8_t raw_fat_read_sector(const raw_fat_volume_t *fat, uint32_t lba, uint8_t *dst) {
    if (!fat || !fat->valid || !dst || fat->bytes_per_sector > MSC_SECTOR_SIZE) {
        return 0;
    }
    buf_zero(dst, MSC_SECTOR_SIZE);
    return w25q_read(lba * (uint32_t)fat->bytes_per_sector, dst, fat->bytes_per_sector);
}

static uint8_t raw_fat_read_bytes(const raw_fat_volume_t *fat,
                                  uint32_t byte_addr,
                                  uint8_t *dst,
                                  uint16_t len) {
    if (!fat || !fat->valid || !dst || !len) {
        return 0;
    }
    return w25q_read(byte_addr, dst, len);
}

static uint8_t raw_fat_parse_bpb(raw_fat_volume_t *fat, uint32_t part_lba, const uint8_t *bpb) {
    uint16_t bytes_per_sector = (uint16_t)bpb[11] | ((uint16_t)bpb[12] << 8);
    uint8_t sectors_per_cluster = bpb[13];
    uint16_t reserved = (uint16_t)bpb[14] | ((uint16_t)bpb[15] << 8);
    uint8_t fat_count = bpb[16];
    uint16_t root_entries = (uint16_t)bpb[17] | ((uint16_t)bpb[18] << 8);
    uint16_t total16 = (uint16_t)bpb[19] | ((uint16_t)bpb[20] << 8);
    uint16_t fat16 = (uint16_t)bpb[22] | ((uint16_t)bpb[23] << 8);
    uint32_t total32 = le32_at(&bpb[32]);
    uint32_t fat32 = le32_at(&bpb[36]);
    uint32_t total_sectors = total16 ? total16 : total32;
    uint32_t sectors_per_fat = fat16 ? fat16 : fat32;
    uint32_t root_dir_sectors;
    uint32_t metadata_sectors;
    uint32_t data_sectors;
    uint32_t clusters;

    if (bpb[510] != 0x55u || bpb[511] != 0xAAu) {
        return 0;
    }
    if ((bpb[0] != 0xEBu && bpb[0] != 0xE9u && bpb[0] != 0xE8u) ||
        (bytes_per_sector != 512u && bytes_per_sector != 1024u &&
         bytes_per_sector != 2048u && bytes_per_sector != 4096u) ||
        !is_power_of_two_u32(sectors_per_cluster) ||
        !reserved || !fat_count || fat_count > 2u || !total_sectors || !sectors_per_fat) {
        return 0;
    }

    root_dir_sectors = (((uint32_t)root_entries * 32u) + bytes_per_sector - 1u) / bytes_per_sector;
    metadata_sectors = (uint32_t)reserved + (uint32_t)fat_count * sectors_per_fat + root_dir_sectors;
    if (metadata_sectors >= total_sectors) {
        return 0;
    }
    data_sectors = total_sectors - metadata_sectors;
    clusters = data_sectors / sectors_per_cluster;
    if (clusters < 2u) {
        return 0;
    }

    fat->valid = 1;
    fat->type = clusters < 4085u ? FAT_TYPE_12 : (clusters < 65525u ? FAT_TYPE_16 : FAT_TYPE_32);
    fat->bytes_per_sector = bytes_per_sector;
    fat->sectors_per_cluster = sectors_per_cluster;
    fat->fat_count = fat_count;
    fat->part_lba = part_lba;
    fat->fat_lba = part_lba + reserved;
    fat->sectors_per_fat = sectors_per_fat;
    fat->root_dir_sectors = root_dir_sectors;
    fat->total_sectors = total_sectors;
    fat->data_lba = part_lba + metadata_sectors;
    if (fat->type == FAT_TYPE_32) {
        fat->root_cluster = le32_at(&bpb[44]);
        if (fat->root_cluster < 2u) {
            fat->root_cluster = 2;
        }
        fat->root_lba = fat->data_lba + (fat->root_cluster - 2u) * sectors_per_cluster;
    } else {
        fat->root_cluster = 0;
        fat->root_lba = part_lba + reserved + (uint32_t)fat_count * sectors_per_fat;
    }
    return 1;
}

static uint8_t raw_fat_mount(raw_fat_volume_t *fat) {
    uint8_t *sector = msc_root_shadow;

    buf_zero((uint8_t *)fat, sizeof(*fat));
    if (!w25q_read(0, sector, MSC_SECTOR_SIZE)) {
        return 0;
    }
    if (raw_fat_parse_bpb(fat, 0, sector)) {
        return 1;
    }

    for (uint8_t i = 0; i < 4u; ++i) {
        const uint8_t *entry = &sector[446u + (uint16_t)i * 16u];
        uint32_t start_lba = le32_at(&entry[8]);
        if (!entry[4] || !start_lba || start_lba >= msc_disk_sectors) {
            continue;
        }
        if (!w25q_read(start_lba * 512u, sector, MSC_SECTOR_SIZE)) {
            continue;
        }
        if (raw_fat_parse_bpb(fat, start_lba, sector)) {
            return 1;
        }
    }
    return 0;
}

static uint32_t raw_fat_cluster_lba(const raw_fat_volume_t *fat, uint32_t cluster) {
    return fat->data_lba + (cluster - 2u) * fat->sectors_per_cluster;
}

static uint8_t raw_fat_next_cluster(const raw_fat_volume_t *fat, uint32_t cluster, uint32_t *next) {
    uint32_t fat_offset;
    uint32_t byte_addr;
    uint8_t entry[4];

    if (!fat || !fat->valid || !next || cluster < 2u) {
        return 0;
    }
    if (fat->type == FAT_TYPE_12) {
        fat_offset = cluster + (cluster / 2u);
    } else {
        fat_offset = fat->type == FAT_TYPE_32 ? cluster * 4u : cluster * 2u;
    }
    byte_addr = (fat->fat_lba * (uint32_t)fat->bytes_per_sector) + fat_offset;
    if (!raw_fat_read_bytes(fat, byte_addr, entry, fat->type == FAT_TYPE_32 ? 4u : 2u)) {
        return 0;
    }
    if (fat->type == FAT_TYPE_32) {
        *next = le32_at(entry) & 0x0FFFFFFFu;
    } else if (fat->type == FAT_TYPE_16) {
        *next = (uint32_t)entry[0] | ((uint32_t)entry[1] << 8);
    } else {
        uint16_t pair = (uint16_t)entry[0] | ((uint16_t)entry[1] << 8);
        *next = (cluster & 1u) ? (uint32_t)(pair >> 4) : (uint32_t)(pair & 0x0FFFu);
    }
    return 1;
}

static uint8_t raw_fat_cluster_is_eoc(const raw_fat_volume_t *fat, uint32_t cluster) {
    if (fat->type == FAT_TYPE_32) {
        return cluster >= 0x0FFFFFF8u;
    }
    if (fat->type == FAT_TYPE_16) {
        return cluster >= 0xFFF8u;
    }
    return cluster >= 0x0FF8u;
}

static uint8_t raw_fat_entry_has_f2c23t_lfn(const uint8_t *entry) {
    uint8_t matched = 0;
    return lfn_entry_has_f2c23t(entry, &matched);
}

static uint8_t raw_fat_scan_dir_sector(const raw_fat_volume_t *fat,
                                       uint32_t dir_lba,
                                       raw_fat_file_t *file,
                                       uint8_t *lfn_has_name) {
    uint8_t *sector = msc_root_shadow;

    if (!raw_fat_read_sector(fat, dir_lba, sector)) {
        return 0;
    }

    for (uint16_t off = 0; off + 32u <= fat->bytes_per_sector; off = (uint16_t)(off + 32u)) {
        uint8_t *entry = &sector[off];
        uint8_t attr = entry[11];
        uint32_t size;
        uint32_t cluster;

        if (entry[0] == 0x00u) {
            return 2;
        }
        if (entry[0] == 0xE5u) {
            *lfn_has_name = 0;
            continue;
        }
        if (attr == 0x0Fu) {
            if (raw_fat_entry_has_f2c23t_lfn(entry)) {
                *lfn_has_name = 1;
            }
            continue;
        }
        if ((attr & 0x18u) || !root_entry_ext_is_bin(entry)) {
            *lfn_has_name = 0;
            continue;
        }
        if (!root_entry_name_has_f2c23t(entry) && !*lfn_has_name) {
            *lfn_has_name = 0;
            continue;
        }
        size = le32_at(&entry[28]);
        if (size < 8192u) {
            *lfn_has_name = 0;
            continue;
        }
        cluster = (uint32_t)entry[26] | ((uint32_t)entry[27] << 8);
        if (fat->type == FAT_TYPE_32) {
            cluster |= ((uint32_t)entry[20] | ((uint32_t)entry[21] << 8)) << 16;
        }
        if (cluster < 2u) {
            *lfn_has_name = 0;
            continue;
        }
        file->first_cluster = cluster;
        file->size = size;
        file->dir_byte_addr = dir_lba * (uint32_t)fat->bytes_per_sector + off;
        return 1;
    }
    return 0;
}

static uint8_t raw_fat_find_update_file(const raw_fat_volume_t *fat, raw_fat_file_t *file) {
    uint8_t lfn_has_name = 0;

    if (fat->type == FAT_TYPE_32) {
        uint32_t cluster = fat->root_cluster;
        uint16_t guard = 0;

        while (cluster >= 2u && !raw_fat_cluster_is_eoc(fat, cluster) && guard++ < 4096u) {
            uint32_t lba = raw_fat_cluster_lba(fat, cluster);
            for (uint8_t i = 0; i < fat->sectors_per_cluster; ++i) {
                uint8_t res = raw_fat_scan_dir_sector(fat, lba + i, file, &lfn_has_name);
                if (res == 1u) {
                    return 1;
                }
                if (res == 2u) {
                    return 0;
                }
            }
            if (!raw_fat_next_cluster(fat, cluster, &cluster)) {
                return 0;
            }
        }
        return 0;
    }

    for (uint32_t i = 0; i < fat->root_dir_sectors; ++i) {
        uint8_t res = raw_fat_scan_dir_sector(fat, fat->root_lba + i, file, &lfn_has_name);
        if (res == 1u) {
            return 1;
        }
        if (res == 2u) {
            return 0;
        }
    }
    return 0;
}

static uint8_t raw_fat_stage_update_file(const raw_fat_volume_t *fat, const raw_fat_file_t *file) {
    uint8_t *sector = msc_root_shadow;
    uint32_t cluster = file->first_cluster;
    uint32_t remaining = file->size;
    uint32_t offset = 0;
    uint16_t guard = 0;
    fw_update_status_t status;

    fw_update_clear();
    fw_update_note_file(RAW_UPDATE_STAGE_BASE_LBA, file->size);
    while (remaining && cluster >= 2u && !raw_fat_cluster_is_eoc(fat, cluster) && guard++ < 4096u) {
        uint32_t lba = raw_fat_cluster_lba(fat, cluster);
        for (uint8_t i = 0; i < fat->sectors_per_cluster && remaining; ++i) {
            uint16_t chunk = remaining > fat->bytes_per_sector ? fat->bytes_per_sector : (uint16_t)remaining;
            if (!raw_fat_read_sector(fat, lba + i, sector)) {
                return 0;
            }
            fw_update_usb_data(RAW_UPDATE_STAGE_BASE_LBA + offset / 512u,
                               (uint16_t)(offset & 511u),
                               sector,
                               chunk);
            fw_update_status(&status);
            if (status.state == FW_UPDATE_STATE_ERROR) {
                return 0;
            }
            offset += chunk;
            remaining -= chunk;
        }
        if (remaining && !raw_fat_next_cluster(fat, cluster, &cluster)) {
            return 0;
        }
    }

    fw_update_status(&status);
    return status.state == FW_UPDATE_STATE_READY && !remaining;
}

static uint8_t raw_storage_write_bytes(uint32_t byte_addr, const uint8_t *src, uint16_t len) {
    while (len) {
        uint32_t sector_addr = byte_addr & ~(uint32_t)(MSC_SECTOR_SIZE - 1u);
        uint16_t off = (uint16_t)(byte_addr - sector_addr);
        uint16_t chunk = min_u16(len, (uint16_t)(MSC_SECTOR_SIZE - off));

        if (!raw_write_flush()) {
            return 0;
        }
        if (!off && chunk == MSC_SECTOR_SIZE) {
            if (!w25q_write_sector(sector_addr, src)) {
                return 0;
            }
        } else {
            if (!w25q_read(sector_addr, raw_write_sector, MSC_SECTOR_SIZE)) {
                return 0;
            }
            buf_copy(&raw_write_sector[off], src, chunk);
            if (!w25q_write_sector(sector_addr, raw_write_sector)) {
                return 0;
            }
            raw_write_loaded = 0;
            raw_write_dirty = 0;
        }
        byte_addr += chunk;
        src += chunk;
        len = (uint16_t)(len - chunk);
    }
    return 1;
}

static uint32_t raw_fat_cluster_count(const raw_fat_volume_t *fat) {
    uint32_t data_relative_lba;

    if (!fat || !fat->valid || fat->data_lba < fat->part_lba ||
        !fat->sectors_per_cluster) {
        return 0;
    }
    data_relative_lba = fat->data_lba - fat->part_lba;
    if (data_relative_lba >= fat->total_sectors) {
        return 0;
    }
    return (fat->total_sectors - data_relative_lba) / fat->sectors_per_cluster;
}

static uint8_t raw_fat_write_entry_copy(const raw_fat_volume_t *fat,
                                        uint8_t copy,
                                        uint32_t cluster,
                                        uint32_t value) {
    uint32_t fat_offset;
    uint32_t byte_addr;
    uint8_t entry[4];

    if (!fat || !fat->valid || copy >= fat->fat_count || cluster < 2u) {
        return 0;
    }

    if (fat->type == FAT_TYPE_12) {
        fat_offset = cluster + (cluster / 2u);
    } else {
        fat_offset = fat->type == FAT_TYPE_32 ? cluster * 4u : cluster * 2u;
    }
    byte_addr = ((fat->fat_lba + (uint32_t)copy * fat->sectors_per_fat) *
                 (uint32_t)fat->bytes_per_sector) + fat_offset;

    if (fat->type == FAT_TYPE_32) {
        if (!raw_fat_read_bytes(fat, byte_addr, entry, 4)) {
            return 0;
        }
        value = (le32_at(entry) & 0xF0000000u) | (value & 0x0FFFFFFFu);
        put_le32(entry, value);
        return raw_storage_write_bytes(byte_addr, entry, 4);
    }
    if (fat->type == FAT_TYPE_12) {
        uint16_t pair;

        if (!raw_fat_read_bytes(fat, byte_addr, entry, 2)) {
            return 0;
        }
        pair = (uint16_t)entry[0] | ((uint16_t)entry[1] << 8);
        if (cluster & 1u) {
            pair = (uint16_t)((pair & 0x000Fu) | ((value & 0x0FFFu) << 4));
        } else {
            pair = (uint16_t)((pair & 0xF000u) | (value & 0x0FFFu));
        }
        put_le16(entry, pair);
        return raw_storage_write_bytes(byte_addr, entry, 2);
    }

    put_le16(entry, (uint16_t)value);
    return raw_storage_write_bytes(byte_addr, entry, 2);
}

static uint8_t raw_fat_find_free_clusters(const raw_fat_volume_t *fat, uint16_t needed) {
    uint32_t last_cluster = raw_fat_cluster_count(fat) + 1u;
    uint16_t found = 0;

    if (!needed || needed > SCREENSHOT_CLUSTER_COUNT) {
        return 0;
    }
    for (uint32_t cluster = 2; cluster <= last_cluster && found < needed; ++cluster) {
        uint32_t value;
        if (!raw_fat_next_cluster(fat, cluster, &value)) {
            return 0;
        }
        if (!value) {
            raw_screenshot_clusters[found++] = cluster;
        }
    }
    return found == needed;
}

static uint32_t raw_fat_entry_offset(const raw_fat_volume_t *fat, uint32_t cluster) {
    return fat->type == FAT_TYPE_12 ? cluster + (cluster / 2u) :
        (fat->type == FAT_TYPE_32 ? cluster * 4u : cluster * 2u);
}

static uint8_t raw_fat_patch_entry_in_sector(const raw_fat_volume_t *fat,
                                             uint16_t off,
                                             uint32_t cluster,
                                             uint32_t value) {
    if (fat->type == FAT_TYPE_32) {
        uint32_t old_value = le32_at(&raw_write_sector[off]);
        put_le32(&raw_write_sector[off],
                 (old_value & 0xF0000000u) | (value & 0x0FFFFFFFu));
        return 1;
    }
    if (fat->type == FAT_TYPE_12) {
        uint16_t pair = (uint16_t)raw_write_sector[off] |
            ((uint16_t)raw_write_sector[off + 1u] << 8);
        if (cluster & 1u) {
            pair = (uint16_t)((pair & 0x000Fu) | ((value & 0x0FFFu) << 4));
        } else {
            pair = (uint16_t)((pair & 0xF000u) | (value & 0x0FFFu));
        }
        put_le16(&raw_write_sector[off], pair);
        return 1;
    }
    put_le16(&raw_write_sector[off], (uint16_t)value);
    return 1;
}

static uint8_t raw_fat_link_clusters(const raw_fat_volume_t *fat, uint16_t count) {
    uint32_t eoc = fat->type == FAT_TYPE_32 ? 0x0FFFFFFFu :
        (fat->type == FAT_TYPE_16 ? 0xFFFFu : 0x0FFFu);
    uint16_t entry_width = fat->type == FAT_TYPE_32 ? 4u : 2u;

    if (!raw_write_flush()) {
        return 0;
    }

    for (uint8_t copy = 0; copy < fat->fat_count; ++copy) {
        uint32_t loaded_sector = 0xFFFFFFFFu;
        uint8_t loaded = 0;
        uint8_t dirty = 0;

        for (uint16_t i = 0; i < count; ++i) {
            uint32_t cluster = raw_screenshot_clusters[i];
            uint32_t next = (i + 1u) < count ? raw_screenshot_clusters[i + 1u] : eoc;
            uint32_t byte_addr =
                ((fat->fat_lba + (uint32_t)copy * fat->sectors_per_fat) *
                 (uint32_t)fat->bytes_per_sector) + raw_fat_entry_offset(fat, cluster);
            uint32_t sector_addr = byte_addr & ~(uint32_t)(MSC_SECTOR_SIZE - 1u);
            uint16_t off = (uint16_t)(byte_addr - sector_addr);

            if (off + entry_width > MSC_SECTOR_SIZE) {
                if (dirty && !w25q_write_sector(loaded_sector, raw_write_sector)) {
                    return 0;
                }
                loaded = 0;
                dirty = 0;
                if (!raw_fat_write_entry_copy(fat, copy, cluster, next)) {
                    return 0;
                }
                continue;
            }

            if (!loaded || loaded_sector != sector_addr) {
                if (dirty && !w25q_write_sector(loaded_sector, raw_write_sector)) {
                    return 0;
                }
                if (!w25q_read(sector_addr, raw_write_sector, MSC_SECTOR_SIZE)) {
                    return 0;
                }
                loaded_sector = sector_addr;
                loaded = 1;
                dirty = 0;
            }

            if (!raw_fat_patch_entry_in_sector(fat, off, cluster, next)) {
                return 0;
            }
            dirty = 1;
        }

        if (dirty && !w25q_write_sector(loaded_sector, raw_write_sector)) {
            return 0;
        }
    }

    raw_write_loaded = 0;
    raw_write_dirty = 0;

    return 1;
}

static void raw_screenshot_name(uint8_t *name, uint16_t index) {
    name[0] = 'I';
    name[1] = 'M';
    name[2] = 'G';
    name[3] = '_';
    name[4] = (uint8_t)('0' + (index / 10u) % 10u);
    name[5] = (uint8_t)('0' + index % 10u);
    name[6] = ' ';
    name[7] = ' ';
    name[8] = 'B';
    name[9] = 'M';
    name[10] = 'P';
}

static uint8_t raw_screenshot_entry_index(const uint8_t *entry, uint16_t *index) {
    if (entry[0] == 0x00u || entry[0] == 0xE5u || entry[11] == 0x0Fu ||
        (entry[11] & 0x18u) ||
        entry[0] != 'I' || entry[1] != 'M' || entry[2] != 'G' || entry[3] != '_' ||
        entry[4] < '0' || entry[4] > '9' || entry[5] < '0' || entry[5] > '9' ||
        entry[6] != ' ' || entry[7] != ' ' ||
        entry[8] != 'B' || entry[9] != 'M' || entry[10] != 'P') {
        return 0;
    }
    *index = (uint16_t)((uint16_t)(entry[4] - '0') * 10u +
                        (uint16_t)(entry[5] - '0'));
    return 1;
}

static uint8_t raw_fat_scan_screenshot_dir_sector(const raw_fat_volume_t *fat,
                                                  uint32_t dir_lba,
                                                  uint32_t *free_byte_addr,
                                                  uint8_t *have_free,
                                                  uint16_t *max_index,
                                                  uint8_t *have_max) {
    uint8_t *sector = msc_root_shadow;

    if (!raw_fat_read_sector(fat, dir_lba, sector)) {
        return 0;
    }
    for (uint16_t off = 0; off + 32u <= fat->bytes_per_sector; off = (uint16_t)(off + 32u)) {
        uint8_t *entry = &sector[off];
        uint16_t index;

        if (entry[0] == 0x00u || entry[0] == 0xE5u) {
            if (!*have_free) {
                *have_free = 1;
                *free_byte_addr = dir_lba * (uint32_t)fat->bytes_per_sector + off;
            }
            if (entry[0] == 0x00u) {
                return 2;
            }
            continue;
        }
        if (raw_screenshot_entry_index(entry, &index) &&
            (!*have_max || index > *max_index)) {
            *have_max = 1;
            *max_index = index;
        }
    }
    return 1;
}

static uint8_t raw_fat_find_screenshot_dir_entry(const raw_fat_volume_t *fat,
                                                 uint32_t *dir_byte_addr,
                                                 uint16_t *next_index) {
    uint8_t have_free = 0;
    uint8_t have_max = 0;
    uint16_t max_index = 0;

    if (fat->type == FAT_TYPE_32) {
        uint32_t cluster = fat->root_cluster;
        uint16_t guard = 0;

        while (cluster >= 2u && !raw_fat_cluster_is_eoc(fat, cluster) && guard++ < 4096u) {
            uint32_t lba = raw_fat_cluster_lba(fat, cluster);
            for (uint8_t i = 0; i < fat->sectors_per_cluster; ++i) {
                uint8_t res = raw_fat_scan_screenshot_dir_sector(
                    fat, lba + i, dir_byte_addr, &have_free, &max_index, &have_max);
                if (!res) {
                    return 0;
                }
                if (res == 2u) {
                    *next_index = have_max && max_index < 99u ? (uint16_t)(max_index + 1u) : 1u;
                    return have_free;
                }
            }
            if (!raw_fat_next_cluster(fat, cluster, &cluster)) {
                return 0;
            }
        }
    } else {
        for (uint32_t i = 0; i < fat->root_dir_sectors; ++i) {
            uint8_t res = raw_fat_scan_screenshot_dir_sector(
                fat, fat->root_lba + i, dir_byte_addr, &have_free, &max_index, &have_max);
            if (!res) {
                return 0;
            }
            if (res == 2u) {
                break;
            }
        }
    }

    *next_index = have_max && max_index < 99u ? (uint16_t)(max_index + 1u) : 1u;
    return have_free;
}

static uint8_t raw_fat_write_screenshot_chain(const raw_fat_volume_t *fat,
                                              uint32_t first_cluster) {
    uint8_t *sector = msc_root_shadow;
    uint32_t remaining = SCREENSHOT_FILE_SIZE;
    uint32_t file_offset = 0;
    uint32_t cluster = first_cluster;
    uint16_t guard = 0;

    while (remaining && cluster >= 2u && !raw_fat_cluster_is_eoc(fat, cluster) && guard++ < 4096u) {
        uint32_t lba = raw_fat_cluster_lba(fat, cluster);
        for (uint8_t s = 0; s < fat->sectors_per_cluster; ++s) {
            uint16_t chunk = remaining > fat->bytes_per_sector ?
                fat->bytes_per_sector : (uint16_t)remaining;

            buf_zero(sector, fat->bytes_per_sector);
            if (chunk) {
                if (!screenshot_render_current(file_offset, sector, chunk)) {
                    return 0;
                }
                file_offset += chunk;
                remaining -= chunk;
            }
            if (!raw_storage_write_bytes((lba + s) * (uint32_t)fat->bytes_per_sector,
                                         sector,
                                         fat->bytes_per_sector)) {
                return 0;
            }
            if (!remaining) {
                break;
            }
        }
        if (remaining && !raw_fat_next_cluster(fat, cluster, &cluster)) {
            return 0;
        }
    }
    return !remaining;
}

static uint8_t raw_fat_write_screenshot_entry(const raw_fat_volume_t *fat,
                                              uint32_t dir_byte_addr,
                                              uint16_t index) {
    uint8_t entry[32];
    uint32_t first_cluster = raw_screenshot_clusters[0];

    (void)fat;
    buf_zero(entry, sizeof(entry));
    raw_screenshot_name(entry, index);
    entry[11] = 0x20;
    entry[12] = 0x18;
    put_le16(&entry[20], (uint16_t)(first_cluster >> 16));
    put_le16(&entry[26], (uint16_t)first_cluster);
    put_le32(&entry[28], SCREENSHOT_FILE_SIZE);
    return raw_storage_write_bytes(dir_byte_addr, entry, sizeof(entry));
}

static uint8_t raw_fat_save_latest_screenshot(void) {
    raw_fat_volume_t fat;
    uint32_t dir_byte_addr = 0;
    uint32_t cluster_bytes;
    uint16_t cluster_count;
    uint16_t file_index = 0;

    if (!raw_fat_mount(&fat) || fat.bytes_per_sector > MSC_SECTOR_SIZE) {
        return 0;
    }

    cluster_bytes = (uint32_t)fat.bytes_per_sector * fat.sectors_per_cluster;
    if (!cluster_bytes) {
        return 0;
    }
    cluster_count = (uint16_t)((SCREENSHOT_FILE_SIZE + cluster_bytes - 1u) / cluster_bytes);
    if (!cluster_count || cluster_count > SCREENSHOT_CLUSTER_COUNT ||
        !raw_fat_find_screenshot_dir_entry(&fat, &dir_byte_addr, &file_index) ||
        !raw_fat_find_free_clusters(&fat, cluster_count)) {
        return 0;
    }

    if (!raw_fat_link_clusters(&fat, cluster_count) ||
        !raw_fat_write_screenshot_chain(&fat, raw_screenshot_clusters[0]) ||
        !raw_fat_write_screenshot_entry(&fat, dir_byte_addr, file_index)) {
        return 0;
    }
    msc_unit_attention = 1;
    return 1;
}

uint8_t usb_msc_store_screenshot(void) {
    return raw_fat_save_latest_screenshot();
}

static void raw_fat_delete_update_file(uint32_t byte_addr) {
    uint32_t sector_addr = byte_addr & ~(uint32_t)(MSC_SECTOR_SIZE - 1u);
    uint16_t off = (uint16_t)(byte_addr - sector_addr);

    if (!byte_addr) {
        return;
    }
    (void)raw_write_flush();
    if (!w25q_read(sector_addr, raw_write_sector, MSC_SECTOR_SIZE)) {
        return;
    }
    raw_write_sector[off] = 0xE5u;
    (void)w25q_write_sector(sector_addr, raw_write_sector);
    raw_write_loaded = 0;
    raw_write_dirty = 0;
}

static void raw_fat_scan_and_stage_update(void) {
    raw_fat_volume_t fat;
    raw_fat_file_t file;

    if (!raw_fat_mount(&fat)) {
        return;
    }
    buf_zero((uint8_t *)&file, sizeof(file));
    if (!raw_fat_find_update_file(&fat, &file)) {
        return;
    }

    raw_update_dir_byte_addr = file.dir_byte_addr;
    msc_update_candidate = 1;
    raw_update_streaming = 1;
    if (!raw_fat_stage_update_file(&fat, &file)) {
        raw_fat_delete_update_file(raw_update_dir_byte_addr);
        fw_update_clear();
        msc_update_candidate = 0;
    }
    raw_update_streaming = 0;
}

static uint8_t raw_write_prepare(uint32_t lba) {
    if (raw_write_loaded && raw_write_lba == lba) {
        return 1;
    }
    if (!raw_write_flush()) {
        raw_write_failed = 1;
        return 0;
    }
    if (!w25q_read(lba * (uint32_t)MSC_SECTOR_SIZE, raw_write_sector, MSC_SECTOR_SIZE)) {
        raw_write_loaded = 0;
        raw_write_failed = 1;
        return 0;
    }
    raw_write_lba = lba;
    raw_write_loaded = 1;
    raw_write_dirty = 0;
    return 1;
}

static void raw_write_chunk(uint32_t lba, uint16_t offset, const uint8_t *data, uint16_t len) {
    if (raw_write_failed || lba >= msc_disk_sectors || offset >= MSC_SECTOR_SIZE) {
        raw_write_failed = 1;
        return;
    }
    if (len > (uint16_t)(MSC_SECTOR_SIZE - offset)) {
        len = (uint16_t)(MSC_SECTOR_SIZE - offset);
    }
    if (!raw_write_prepare(lba)) {
        return;
    }
    buf_copy(&raw_write_sector[offset], data, len);
    raw_write_dirty = 1;
    if ((uint16_t)(offset + len) >= MSC_SECTOR_SIZE && !raw_write_flush()) {
        raw_write_failed = 1;
    }
}

static void disk_fill_chunk(uint32_t lba, uint16_t offset, uint8_t *dst, uint16_t len) {
    buf_zero(dst, len);
    if (lba < msc_disk_sectors) {
        uint32_t addr = lba * (uint32_t)MSC_SECTOR_SIZE + offset;
        (void)w25q_read(addr, dst, len);
    }
}

static void msc_start_read10(uint32_t lba, uint16_t blocks, uint32_t host_len) {
    uint32_t available = 0;
    if (lba < msc_disk_sectors) {
        uint32_t max_blocks = msc_disk_sectors - lba;
        if (blocks > max_blocks) {
            blocks = (uint16_t)max_blocks;
        }
        available = (uint32_t)blocks * MSC_SECTOR_SIZE;
    }
    if (available > host_len) {
        available = host_len;
    }
    if (!available) {
        msc_set_sense(0x05, 0x21);
        msc_start_small_in(0, MSC_CSW_FAIL, host_len);
        return;
    }
    msc_read_lba = lba;
    msc_read_remaining = available;
    msc_sector_pos = MSC_SECTOR_SIZE;
    msc_residue = host_len - available;
    msc_csw_status = MSC_CSW_OK;
    msc_state = MSC_SEND_READ_DATA;
    msc_send_next_in();
}

static void msc_send_next_in(void) {
    uint16_t n;
    if (msc_state == MSC_SEND_SMALL_DATA) {
        n = min_u16((uint16_t)(msc_data_len - msc_data_pos), USB_EP1_SIZE);
        ep1_send_raw(&msc_small[msc_data_pos], n);
        msc_data_pos += n;
        if (msc_data_pos >= msc_data_len) {
            msc_state = MSC_SEND_CSW;
            msc_data_pos = 0;
            msc_prepare_csw(msc_csw_status, msc_residue);
        }
    } else if (msc_state == MSC_SEND_READ_DATA) {
        if (!msc_read_remaining) {
            msc_prepare_csw(msc_csw_status, msc_residue);
            msc_send_next_in();
            return;
        }
        if (msc_sector_pos >= MSC_SECTOR_SIZE) {
            msc_sector_pos = 0;
        }
        n = min_u16((uint16_t)(MSC_SECTOR_SIZE - msc_sector_pos), USB_EP1_SIZE);
        if (n > msc_read_remaining) {
            n = (uint16_t)msc_read_remaining;
        }
        disk_fill_chunk(msc_read_lba, msc_sector_pos, msc_small, n);
        ep1_send_raw(msc_small, n);
        msc_sector_pos = (uint16_t)(msc_sector_pos + n);
        if (msc_sector_pos >= MSC_SECTOR_SIZE) {
            ++msc_read_lba;
        }
        msc_read_remaining -= n;
    } else if (msc_state == MSC_SEND_CSW) {
        n = min_u16((uint16_t)(msc_data_len - msc_data_pos), USB_EP1_SIZE);
        ep1_send_raw(&msc_small[msc_data_pos], n);
        msc_data_pos += n;
        if (msc_data_pos >= msc_data_len) {
            msc_state = MSC_IDLE;
        }
    }
}

static void msc_inquiry(uint32_t host_len) {
    buf_zero(msc_small, 36);
    msc_small[0] = 0x00;
    msc_small[1] = 0x80;
    msc_small[2] = 0x00;
    msc_small[3] = 0x01;
    msc_small[4] = 31;
    buf_copy(&msc_small[8], (const uint8_t *)"F2C23T  ", 8);
    buf_copy(&msc_small[16], (const uint8_t *)"W25Q RAW FLASH  ", 16);
    buf_copy(&msc_small[32], (const uint8_t *)"0001", 4);
    msc_start_small_in(min_u16(36, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 36u ? host_len - 36u : 0u);
}

static void msc_request_sense(uint32_t host_len) {
    buf_zero(msc_small, 18);
    msc_small[0] = 0x70;
    msc_small[2] = msc_sense_key;
    msc_small[7] = 10;
    msc_small[12] = msc_sense_asc;
    msc_start_small_in(min_u16(18, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 18u ? host_len - 18u : 0u);
    msc_set_sense(0, 0);
}

static void msc_read_capacity(uint32_t host_len) {
    buf_zero(msc_small, 8);
    put_be32(&msc_small[0], msc_disk_sectors ? msc_disk_sectors - 1u : 0u);
    put_be32(&msc_small[4], MSC_SECTOR_SIZE);
    msc_start_small_in(min_u16(8, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 8u ? host_len - 8u : 0u);
}

static void msc_read_format_capacities(uint32_t host_len) {
    buf_zero(msc_small, 12);
    msc_small[3] = 8;
    put_be32(&msc_small[4], msc_disk_sectors);
    msc_small[8] = 0x02;
    msc_small[9] = (uint8_t)(MSC_SECTOR_SIZE >> 16);
    msc_small[10] = (uint8_t)(MSC_SECTOR_SIZE >> 8);
    msc_small[11] = (uint8_t)MSC_SECTOR_SIZE;
    msc_start_small_in(min_u16(12, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 12u ? host_len - 12u : 0u);
}

static void msc_mode_sense6(uint32_t host_len) {
    buf_zero(msc_small, 4);
    msc_small[0] = 3;
    msc_small[2] = 0x00;
    msc_start_small_in(min_u16(4, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 4u ? host_len - 4u : 0u);
}

static void msc_mode_sense10(uint32_t host_len) {
    buf_zero(msc_small, 8);
    msc_small[1] = 6;
    msc_small[3] = 0x00;
    msc_start_small_in(min_u16(8, (uint16_t)host_len), MSC_CSW_OK,
                       host_len > 8u ? host_len - 8u : 0u);
}

static uint8_t msc_report_unit_attention(uint8_t op, uint32_t data_len) {
    if (!msc_unit_attention || op == 0x03u || op == 0x12u) {
        return 0;
    }

    msc_unit_attention = 0;
    msc_set_sense(0x06, 0x28);
    msc_start_small_in(0, MSC_CSW_FAIL, data_len);
    return 1;
}

static void msc_handle_cbw(const uint8_t *cbw, uint16_t len) {
    uint32_t data_len;
    uint8_t op;
    if (len != 31u || le32(&cbw[0]) != 0x43425355u || cbw[14] > 16u) {
        msc_set_sense(0x05, 0x20);
        msc_state = MSC_IDLE;
        return;
    }
    msc_tag = le32(&cbw[4]);
    data_len = le32(&cbw[8]);
    op = cbw[15];
    if (msc_report_unit_attention(op, data_len)) {
        return;
    }
    switch (op) {
    case 0x00:
    case 0x1E:
    case 0x2F:
    case 0x35:
        msc_set_sense(0, 0);
        msc_start_small_in(0, MSC_CSW_OK, data_len);
        break;
    case 0x1B:
        msc_set_sense(0, 0);
        msc_start_small_in(0, MSC_CSW_OK, data_len);
        break;
    case 0x12:
        msc_inquiry(data_len);
        break;
    case 0x03:
        msc_request_sense(data_len);
        break;
    case 0x23:
        msc_read_format_capacities(data_len);
        break;
    case 0x25:
        msc_read_capacity(data_len);
        break;
    case 0x1A:
        msc_mode_sense6(data_len);
        break;
    case 0x5A:
        msc_mode_sense10(data_len);
        break;
    case 0x28:
        msc_start_read10(be32(&cbw[17]), be16(&cbw[22]), data_len);
        break;
    case 0x2A:
    {
        uint32_t lba = be32(&cbw[17]);
        uint16_t blocks = be16(&cbw[22]);
        uint32_t expected = (uint32_t)blocks * MSC_SECTOR_SIZE;
        uint8_t ok = 1;
        msc_set_sense(0, 0);
        if (data_len != expected || lba >= msc_disk_sectors ||
            blocks > (uint16_t)(msc_disk_sectors - lba)) {
            ok = 0;
            msc_set_sense(0x05, 0x21);
        }
        if (data_len) {
            msc_discard_remaining = data_len;
            msc_discard_status = ok ? MSC_CSW_OK : MSC_CSW_FAIL;
            msc_write_lba = lba;
            msc_write_pos = 0;
            raw_write_loaded = 0;
            raw_write_dirty = 0;
            raw_write_failed = 0;
            msc_state = MSC_DISCARD_OUT_DATA;
        } else {
            msc_start_small_in(0, ok ? MSC_CSW_OK : MSC_CSW_FAIL, 0);
        }
        break;
    }
    default:
        msc_set_sense(0x05, 0x20);
        msc_start_small_in(0, MSC_CSW_FAIL, data_len);
        break;
    }
}

static void ep1_out(void) {
    uint16_t len = pma_get_rx_count(1);
    if (len > sizeof(msc_small)) {
        len = sizeof(msc_small);
    }
    pma_read(USB_EP1_RX, msc_small, len);
    ep_clear_rx_ctr(1);

    if (msc_state == MSC_DISCARD_OUT_DATA) {
        uint32_t chunk_lba = msc_write_lba + (msc_write_pos / MSC_SECTOR_SIZE);
        uint16_t chunk_off = (uint16_t)(msc_write_pos % MSC_SECTOR_SIZE);
        if (msc_discard_status == MSC_CSW_OK) {
            raw_write_chunk(chunk_lba, chunk_off, msc_small, len);
        }
        msc_write_pos += len;
        msc_update_idle_ms = 0;
        if (msc_discard_remaining > len) {
            msc_discard_remaining -= len;
        } else {
            msc_discard_remaining = 0;
            if (!raw_write_flush()) {
                raw_write_failed = 1;
            }
            if (raw_write_failed) {
                msc_discard_status = MSC_CSW_FAIL;
                msc_set_sense(0x03, 0x0C);
            } else if (msc_discard_status == MSC_CSW_OK) {
                raw_update_scan_pending = 1;
            }
            msc_prepare_csw(msc_discard_status, 0);
            msc_send_next_in();
        }
        ep_set_rx_stat(1, USB_STAT_VALID);
        return;
    }
    if (msc_state == MSC_IDLE) {
        msc_handle_cbw(msc_small, len);
    }
    ep_set_rx_stat(1, USB_STAT_VALID);
}

static void ep1_in(void) {
    ep_clear_tx_ctr(1);
    if (msc_state != MSC_IDLE) {
        msc_send_next_in();
    }
}

static void usb_handle_setup(const uint8_t *setup) {
    uint8_t bm = setup[0];
    uint8_t req = setup[1];
    uint16_t value = (uint16_t)setup[2] | ((uint16_t)setup[3] << 8);
    uint16_t index = (uint16_t)setup[4] | ((uint16_t)setup[5] << 8);
    uint16_t len = (uint16_t)setup[6] | ((uint16_t)setup[7] << 8);
    uint16_t n;

    if ((bm & 0x60u) == 0x20u) {
        if (req == 0xFE && len == 1u) {
            msc_small[0] = 0;
            ep0_send(msc_small, 1, len);
        } else if (req == 0xFF) {
            msc_state = MSC_IDLE;
            ep0_send_zlp();
        } else {
            ep0_stall();
        }
        (void)index;
        return;
    }

    if ((bm & 0x60u) != 0x00u) {
        ep0_stall();
        return;
    }

    switch (req) {
    case 0x06:
        if ((value >> 8) == 1u) {
            ep0_send(usb_device_desc, sizeof(usb_device_desc), len);
        } else if ((value >> 8) == 2u) {
            ep0_send(usb_config_desc, sizeof(usb_config_desc), len);
        } else if ((value >> 8) == 3u) {
            n = make_string_desc((uint8_t)value);
            if (n) {
                ep0_send(msc_small, n, len);
            } else {
                ep0_stall();
            }
        } else {
            ep0_stall();
        }
        break;
    case 0x05:
        ep0_pending_address = (uint8_t)(value & 0x7Fu);
        ep0_address_pending = 1;
        ep0_send_zlp();
        break;
    case 0x09:
        usb_configured = (uint8_t)value;
        ep_set_type(1, USB_EP_TYPE_BULK);
        pma_set_tx(1, USB_EP1_TX, 0);
        pma_set_rx(1, USB_EP1_RX, USB_EP1_SIZE);
        ep_set_tx_stat(1, USB_STAT_NAK);
        ep_set_rx_stat(1, USB_STAT_VALID);
        msc_state = MSC_IDLE;
        ep0_send_zlp();
        break;
    case 0x08:
        msc_small[0] = usb_configured;
        ep0_send(msc_small, 1, len);
        break;
    case 0x00:
        msc_small[0] = 0;
        msc_small[1] = 0;
        ep0_send(msc_small, 2, len);
        break;
    case 0x01:
    case 0x03:
        ep0_send_zlp();
        break;
    default:
        ep0_stall();
        break;
    }
}

static void ep0_rx(void) {
    uint8_t setup[8];
    uint16_t reg = USB_EPR(0);
    if (reg & USB_EP_SETUP) {
        pma_read(USB_EP0_RX, setup, 8);
        ep_clear_rx_ctr(0);
        ep_set_tx_stat(0, USB_STAT_NAK);
        ep_set_rx_stat(0, USB_STAT_NAK);
        usb_handle_setup(setup);
    } else {
        ep_clear_rx_ctr(0);
        ep_set_rx_stat(0, USB_STAT_VALID);
    }
}

static void ep0_tx(void) {
    ep_clear_tx_ctr(0);
    if (ep0_state == EP0_TX_DATA && ep0_left) {
        ep0_send_next();
        return;
    }
    if (ep0_address_pending) {
        ep0_address_pending = 0;
        USB_DADDR = (uint16_t)(USB_DADDR_EF | ep0_pending_address);
    }
    ep0_state = EP0_IDLE;
    ep_set_rx_stat(0, USB_STAT_VALID);
}

static void usb_reset_core(void) {
    USB_BTABLE = 0;
    USB_DADDR = USB_DADDR_EF;

    ep_set_type(0, USB_EP_TYPE_CONTROL);
    pma_set_tx(0, USB_EP0_TX, 0);
    pma_set_rx(0, USB_EP0_RX, USB_EP0_SIZE);
    ep_set_tx_stat(0, USB_STAT_NAK);
    ep_set_rx_stat(0, USB_STAT_VALID);

    ep_set_type(1, USB_EP_TYPE_BULK);
    pma_set_tx(1, USB_EP1_TX, 0);
    pma_set_rx(1, USB_EP1_RX, USB_EP1_SIZE);
    ep_set_tx_stat(1, USB_STAT_NAK);
    ep_set_rx_stat(1, USB_STAT_NAK);

    ep0_state = EP0_IDLE;
    ep0_address_pending = 0;
    usb_configured = 0;
    msc_state = MSC_IDLE;
    msc_set_sense(0, 0);
}

void usb_msc_init(void) {
    uint32_t w25q_bytes;

    if (usb_ready) {
        return;
    }

    w25q_init();
    w25q_bytes = w25q_capacity_bytes();
    msc_disk_sectors = w25q_bytes >= MSC_SECTOR_SIZE ? w25q_bytes / MSC_SECTOR_SIZE : 0u;

    RCC_APB2ENR |= (1u << 2); // GPIOA
    RCC_APB1ENR |= (1u << 23); // USBFS
    RCC_APB2ENR |= (1u << 22); // USBFS APB2 gate on AT32
    RCC_CFGR2 |= 1u << 25; // USB clock source enable on AT32
    REG32(RCC_BASE + 0x54u) |= (1u << 8) | (1u << 9);
    GPIO_CRH(GPIOA_BASE) = (GPIO_CRH(GPIOA_BASE) & ~((0xFu << 12) | (0xFu << 16))) |
                           (0x4u << 12) | (0x4u << 16);

    REG32(USB_BASE + 0x40u) |= 2u;
    REG32(USB_BASE + 0x60u) |= 2u;
    delay_ms(20);

    USB_CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
    delay_ms(4);
    USB_CNTR = USB_CNTR_FRES;
    delay_ms(4);
    USB_CNTR = 0;
    USB_ISTR = 0;
    usb_reset_core();
    USB_CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;

    REG32(0x4001580Cu) = 7990u;
    REG32(0x40015810u) = 8000u;
    REG32(0x40015814u) = 8010u;
    REG32(0x40015804u) = (REG32(0x40015804u) & ~3u) | 3u;
    REG32(USB_BASE + 0x60u) &= ~2u;
    REG32(USB_BASE + 0x40u) &= ~2u;

    REG32(NVIC_ISER0) = 1u << 20;
    usb_ready = 1;
}

void usb_msc_set_enabled(uint8_t enabled) {
    if (enabled) {
        usb_msc_init();
        return;
    }
    if (!usb_ready) {
        return;
    }

    USB_CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
    USB_ISTR = 0;
    REG32(0xE000E180u) = 1u << 20; // USB_LP_CAN1_RX0 IRQ disable
    RCC_APB1ENR &= ~(1u << 23);
    RCC_APB2ENR &= ~(1u << 22);
    usb_ready = 0;
    usb_configured = 0;
    msc_state = MSC_IDLE;
}

void usb_msc_poll(void) {
    fw_update_status_t status;

    if (!usb_ready) {
        return;
    }

    if (raw_update_scan_pending && !raw_update_streaming && !msc_update_candidate) {
        if (msc_update_idle_ms < MSC_UPDATE_IDLE_APPLY_MS) {
            ++msc_update_idle_ms;
            return;
        }
        raw_update_scan_pending = 0;
        msc_update_idle_ms = 0;
        raw_fat_scan_and_stage_update();
    }

    if (!msc_update_candidate) {
        return;
    }
    fw_update_status(&status);
    if (status.state == FW_UPDATE_STATE_ERROR) {
        raw_fat_delete_update_file(raw_update_dir_byte_addr);
        fw_update_clear();
        msc_update_candidate = 0;
        return;
    }
    if (msc_update_idle_ms < MSC_UPDATE_IDLE_APPLY_MS) {
        ++msc_update_idle_ms;
        return;
    }
    if (status.state == FW_UPDATE_STATE_READY &&
        status.expected_size >= 8192u &&
        status.bytes >= status.expected_size) {
        if (fw_update_request_apply()) {
            raw_fat_delete_update_file(raw_update_dir_byte_addr);
        } else {
            raw_fat_delete_update_file(raw_update_dir_byte_addr);
            fw_update_clear();
        }
        msc_update_candidate = 0;
    }
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    uint16_t istr;
    if (!usb_ready) {
        return;
    }
    while ((istr = USB_ISTR) & (USB_ISTR_CTR | USB_ISTR_RESET | USB_ISTR_SUSP | USB_ISTR_WKUP)) {
        if (istr & USB_ISTR_RESET) {
            USB_ISTR = (uint16_t)~USB_ISTR_RESET;
            usb_reset_core();
            continue;
        }
        if (istr & USB_ISTR_SUSP) {
            USB_ISTR = (uint16_t)~USB_ISTR_SUSP;
        }
        if (istr & USB_ISTR_WKUP) {
            USB_ISTR = (uint16_t)~USB_ISTR_WKUP;
        }
        if (istr & USB_ISTR_CTR) {
            uint8_t ep = (uint8_t)(istr & 0x0Fu);
            uint16_t reg = USB_EPR(ep);
            if (ep == 0) {
                if (reg & USB_EP_CTR_RX) {
                    ep0_rx();
                }
                if (USB_EPR(0) & USB_EP_CTR_TX) {
                    ep0_tx();
                }
            } else if (ep == 1) {
                if (reg & USB_EP_CTR_RX) {
                    ep1_out();
                }
                if (USB_EPR(1) & USB_EP_CTR_TX) {
                    ep1_in();
                }
            } else {
                if (reg & USB_EP_CTR_RX) {
                    ep_clear_rx_ctr(ep);
                }
                if (reg & USB_EP_CTR_TX) {
                    ep_clear_tx_ctr(ep);
                }
            }
        }
    }
}
