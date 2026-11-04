#include "arb_csv.h"
#include "usb_msc.h"
#include "w25q.h"

#include <stdint.h>

#define SECTOR_SIZE 4096u

struct arb_file {
    char name[ARB_FILENAME_LEN];
    uint32_t start_cluster;
    uint32_t file_size;
    uint16_t sample_count;
};

static uint8_t sector_buf[SECTOR_SIZE];
static struct arb_file files[ARB_MAX_FILES];
static uint8_t file_count;
static uint8_t scanned;

static uint32_t partition_addr;
static uint16_t bytes_per_sec;
static uint8_t  sec_per_cluster;
static uint16_t reserved_sec;
static uint8_t  fat_count;
static uint32_t fat_size;
static uint32_t root_cluster;
static uint16_t root_entries;
static uint8_t  is_fat32;

static uint8_t read_sector(uint32_t addr) {
    return w25q_read(addr, sector_buf, SECTOR_SIZE);
}

static uint16_t get_le16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}

static uint32_t get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static uint8_t parse_bpb(void) {
    if (sector_buf[510] != 0x55 || sector_buf[511] != 0xAA) {
        return 0;
    }
    bytes_per_sec = get_le16(&sector_buf[11]);
    if (bytes_per_sec < 512 || bytes_per_sec > SECTOR_SIZE) {
        return 0;
    }
    sec_per_cluster = sector_buf[13];
    if (sec_per_cluster == 0) {
        return 0;
    }
    reserved_sec = get_le16(&sector_buf[14]);
    if (reserved_sec == 0) {
        return 0;
    }
    fat_count = sector_buf[16];
    if (fat_count == 0) {
        return 0;
    }
    root_entries = get_le16(&sector_buf[17]);

    uint16_t fat_size_16 = get_le16(&sector_buf[22]);
    uint32_t total_sec = get_le16(&sector_buf[19]);
    if (total_sec == 0) {
        total_sec = get_le32(&sector_buf[32]);
    }
    if (total_sec == 0) {
        return 0;
    }

    fat_size = fat_size_16;
    root_cluster = 0;
    is_fat32 = 0;

    if (fat_size_16 == 0) {
        is_fat32 = 1;
        fat_size = get_le32(&sector_buf[36]);
        root_cluster = get_le32(&sector_buf[44]);
        if (fat_size == 0 || root_cluster == 0) {
            return 0;
        }
    }
    return 1;
}

static uint8_t mount(void) {
    if (!read_sector(0)) {
        return 0;
    }

    partition_addr = 0;
    if (sector_buf[510] == 0x55 && sector_buf[511] == 0xAA) {
        uint8_t type = sector_buf[450];
        if (type == 0x01 || type == 0x04 || type == 0x06 ||
            type == 0x0B || type == 0x0C || type == 0x0E) {
            partition_addr = get_le32(&sector_buf[454]) * 512u;
        }
    }

    if (!read_sector(partition_addr)) {
        return 0;
    }
    return parse_bpb();
}

static uint32_t first_data_sector(void) {
    if (is_fat32) {
        return reserved_sec + (uint32_t)fat_count * fat_size;
    }
    return reserved_sec + (uint32_t)fat_count * fat_size +
           (uint32_t)root_entries * 32u / bytes_per_sec;
}

static uint32_t cluster_to_addr(uint32_t cluster) {
    if (cluster < 2) {
        return 0;
    }
    uint32_t fds = first_data_sector();
    uint32_t sec = fds + (cluster - 2u) * (uint32_t)sec_per_cluster;
    return partition_addr + sec * (uint32_t)bytes_per_sec;
}

static uint32_t read_fat_entry(uint32_t cluster) {
    uint32_t byte_off;
    uint32_t fat_sec;
    uint32_t off;

    if (is_fat32) {
        byte_off = cluster * 4u;
    } else {
        byte_off = cluster * 2u;
    }
    fat_sec = reserved_sec + byte_off / (uint32_t)bytes_per_sec;
    off = byte_off % (uint32_t)bytes_per_sec;

    if (!read_sector(partition_addr + fat_sec * (uint32_t)bytes_per_sec)) {
        return 0xFFFFFFFF;
    }

    if (is_fat32) {
        uint32_t entry = get_le32(&sector_buf[off]) & 0x0FFFFFFFu;
        if (entry >= 0x0FFFFFF8u) {
            return 0xFFFFFFFF;
        }
        return entry;
    }
    uint16_t entry = get_le16(&sector_buf[off]);
    if (entry >= 0xFFF8u) {
        return 0xFFFFFFFF;
    }
    return entry;
}

static char to_upper(char c) {
    if (c >= 'a' && c <= 'z') {
        return (char)(c - 32);
    }
    return c;
}

static uint8_t is_csv_ext(const uint8_t *entry) {
    char ext[4];
    ext[0] = (char)to_upper((char)entry[8]);
    ext[1] = (char)to_upper((char)entry[9]);
    ext[2] = (char)to_upper((char)entry[10]);
    ext[3] = 0;
    return ext[0] == 'C' && ext[1] == 'S' && ext[2] == 'V';
}

static void dir_name_short(const uint8_t *entry, char *out, uint8_t max_len) {
    uint8_t o = 0;
    uint8_t i;

    for (i = 0; i < 8 && entry[i] != ' '; ++i) {
        if (o + 1u < max_len) {
            out[o++] = (char)entry[i];
        }
    }
    if (o + 1u < max_len) {
        out[o] = 0;
    } else {
        out[max_len - 1u] = 0;
    }
}

static uint8_t scan_root(void) {
    uint8_t found = 0;

    if (is_fat32) {
        uint32_t cluster = root_cluster;

        while (cluster != 0xFFFFFFFF && found < ARB_MAX_FILES) {
            uint32_t dir_addr = cluster_to_addr(cluster);
            if (dir_addr == 0) break;

            if (!read_sector(dir_addr)) break;

            for (uint16_t i = 0; i < bytes_per_sec && found < ARB_MAX_FILES; i += 32u) {
                uint8_t first_byte = sector_buf[i];
                if (first_byte == 0x00) break;
                if (first_byte == 0xE5) continue;
                if (sector_buf[i + 11] & 0x08u) continue;

                if (is_csv_ext(&sector_buf[i])) {
                    dir_name_short(&sector_buf[i], files[found].name, ARB_FILENAME_LEN);
                    files[found].start_cluster = get_le16(&sector_buf[i + 26]);
                    if (is_fat32) {
                        files[found].start_cluster |= (uint32_t)get_le16(&sector_buf[i + 20]) << 16u;
                    }
                    files[found].file_size = get_le32(&sector_buf[i + 28]);
                    files[found].sample_count = 0;
                    ++found;
                }
            }

            cluster = read_fat_entry(cluster);
        }
        file_count = found;
        return found > 0 ? 1u : 0u;
    }

    uint32_t root_sec = reserved_sec + (uint32_t)fat_count * fat_size;
    uint32_t root_addr = partition_addr + root_sec * (uint32_t)bytes_per_sec;
    uint16_t total_root_size = (uint16_t)((uint32_t)root_entries * 32u);
    uint16_t root_sectors = (total_root_size + bytes_per_sec - 1u) / bytes_per_sec;

    for (uint16_t sec = 0; sec < root_sectors && found < ARB_MAX_FILES; ++sec) {
        if (!read_sector(root_addr + (uint32_t)sec * (uint32_t)bytes_per_sec)) break;

        for (uint16_t i = 0; i < bytes_per_sec && found < ARB_MAX_FILES; i += 32u) {
            uint8_t first_byte = sector_buf[i];
            if (first_byte == 0x00) break;
            if (first_byte == 0xE5) continue;
            if (sector_buf[i + 11] & 0x08u) continue;

            if (is_csv_ext(&sector_buf[i])) {
                dir_name_short(&sector_buf[i], files[found].name, ARB_FILENAME_LEN);
                files[found].start_cluster = get_le16(&sector_buf[i + 26]);
                files[found].file_size = get_le32(&sector_buf[i + 28]);
                files[found].sample_count = 0;
                ++found;
            }
        }
    }
    file_count = found;
    return found > 0 ? 1u : 0u;
}

static uint16_t stream_csv_values(uint32_t start_cluster, uint32_t file_size,
                                   uint8_t *buffer, uint16_t max) {
    uint32_t cluster = start_cluster;
    uint32_t bytes_left = file_size;
    uint16_t parsed = 0;
    uint16_t partial = 0;
    uint8_t in_val = 0;

    while (cluster != 0xFFFFFFFF && bytes_left > 0 && parsed < max) {
        uint32_t addr = cluster_to_addr(cluster);
        if (addr == 0) break;

        uint16_t cluster_size = (uint16_t)((uint32_t)sec_per_cluster * (uint32_t)bytes_per_sec);
        uint32_t off = 0;

        while (off < cluster_size && bytes_left > 0 && parsed < max) {
            uint16_t sector_bytes = bytes_per_sec;
            if ((uint32_t)sector_bytes > bytes_left) {
                sector_bytes = (uint16_t)bytes_left;
            }
            if (!read_sector((uint32_t)(addr + off))) {
                goto done;
            }
            for (uint16_t i = 0; i < sector_bytes && bytes_left > 0 && parsed < max; ++i) {
                char c = (char)sector_buf[i];
                --bytes_left;
                if (c >= '0' && c <= '9') {
                    partial = (uint16_t)(partial * 10u + (uint16_t)(c - '0'));
                    if (partial > 255u) {
                        partial = 255u;
                    }
                    in_val = 1;
                } else if (in_val) {
                    if (buffer) {
                        buffer[parsed] = (uint8_t)partial;
                    }
                    ++parsed;
                    partial = 0;
                    in_val = 0;
                }
            }
            off += bytes_per_sec;
        }
        cluster = read_fat_entry(cluster);
    }

    if (in_val && parsed < max) {
        if (buffer) {
            buffer[parsed] = (uint8_t)partial;
        }
        ++parsed;
    }

done:
    return parsed;
}

uint8_t arb_scan_files(void) {
    uint8_t i;
    uint8_t ok;

    for (i = 0; i < ARB_MAX_FILES; ++i) {
        files[i].name[0] = 0;
        files[i].start_cluster = 0;
        files[i].file_size = 0;
        files[i].sample_count = 0;
    }
    file_count = 0;
    scanned = 0;

    // Retry on the next call while the USB host is writing.
    if (!usb_msc_storage_begin()) {
        return 0;
    }
    ok = mount() && scan_root();
    usb_msc_storage_end();
    if (!ok) {
        return 0;
    }
    scanned = 1;
    return file_count > 0 ? 1u : 0u;
}

uint8_t arb_file_count(void) {
    if (!scanned) {
        arb_scan_files();
    }
    return file_count;
}

const char *arb_file_name(uint8_t index) {
    if (index >= file_count) {
        return "";
    }
    return files[index].name;
}

uint16_t arb_file_sample_count(uint8_t index) {
    if (index >= file_count) {
        return 0;
    }
    if (files[index].sample_count == 0 && files[index].start_cluster != 0 &&
        usb_msc_storage_begin()) {
        files[index].sample_count = stream_csv_values(files[index].start_cluster,
                                                       files[index].file_size, 0, 2048);
        usb_msc_storage_end();
    }
    return files[index].sample_count;
}

uint8_t arb_load_file(uint8_t index, uint8_t *buffer, uint16_t max) {
    uint16_t parsed;

    if (index >= file_count) {
        return 0;
    }
    if (files[index].start_cluster == 0 || !usb_msc_storage_begin()) {
        return 0;
    }

    parsed = stream_csv_values(files[index].start_cluster, files[index].file_size,
                                buffer, max);
    usb_msc_storage_end();
    files[index].sample_count = parsed;
    return parsed > 0 ? 1u : 0u;
}
