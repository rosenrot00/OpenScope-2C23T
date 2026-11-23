#include "fw_update.h"

#include "app_config.h"
#include "hw.h"

#include <stdint.h>

enum {
    FW_APP_BASE = APP_BASE_ADDR,
    FW_STAGE_BASE = FW_STAGE_BASE_ADDR,
    FW_RAM_BASE = 0x20000000u,
    FW_RAM_END = 0x20036078u,
    FW_PAGE_SIZE = 2048u,
    FW_SECTOR_SIZE = 512u,
    FW_SECTORS_PER_PAGE = FW_PAGE_SIZE / FW_SECTOR_SIZE,
    FW_MAX_SIZE = APP_FLASH_SIZE_BYTES,
    FW_APP_END = FW_APP_BASE + FW_MAX_SIZE,
    FW_MIN_SIZE = 8192u,

    // One bit per staging page says "already erased", one bit per 512-byte
    // sector says "staged and read back". Tracking halfwords instead cost
    // FW_MAX_SIZE/16 bytes of RAM, which a 484 KiB application slot cannot
    // afford: 31 KiB of bitmap against about 8 KiB of free RAM.
    FW_STAGE_PAGE_COUNT = (FW_MAX_SIZE + FW_PAGE_SIZE - 1u) / FW_PAGE_SIZE,
    FW_STAGE_PAGE_BYTES = (FW_STAGE_PAGE_COUNT + 7u) / 8u,
    FW_STAGE_SECTOR_COUNT = (FW_MAX_SIZE + FW_SECTOR_SIZE - 1u) / FW_SECTOR_SIZE,
    FW_STAGE_SECTOR_BYTES = (FW_STAGE_SECTOR_COUNT + 7u) / 8u,
    FW_SECTOR_COVER_BYTES = FW_SECTOR_SIZE / 8u,

    // The 1 MiB flash is two independently controlled banks. Everything at or
    // above this address is driven by a second register set at +0x40, so the
    // staging area must be programmed through it; settings.c does the same for
    // the settings page at 0x080FF800. Using the bank 1 registers on a bank 2
    // address silently does nothing at all.
    FLASH_BANK2_BASE = 0x08080000u,

    FLASH_STS_BSY = 1u << 0,
    FLASH_STS_PGERR = 1u << 2,
    FLASH_STS_WRPRTERR = 1u << 4,
    FLASH_STS_EOP = 1u << 5,
    FLASH_CTRL_PG = 1u << 0,
    FLASH_CTRL_PER = 1u << 1,
    FLASH_CTRL_STRT = 1u << 6,
    FLASH_CTRL_LOCK = 1u << 7,

    FW_APPLY_DELAY_LOOPS = 5,
};

// Every register goes through REG32 so the host test can model the flash.
#define FLASH_REG(addr, bank1_offset) \
    REG32(FLASH_R_BASE + (bank1_offset) + \
          (((uint32_t)(addr) >= FLASH_BANK2_BASE) ? 0x40u : 0x00u))
#define FLASH_KEYR_OF(addr) FLASH_REG(addr, 0x04u)
#define FLASH_STS_OF(addr)  FLASH_REG(addr, 0x0Cu)
#define FLASH_CTRL_OF(addr) FLASH_REG(addr, 0x10u)
#define FLASH_ADDR_OF(addr) FLASH_REG(addr, 0x14u)

static volatile fw_update_status_t fw_status = {
    .state = FW_UPDATE_STATE_IDLE,
};
static volatile uint8_t fw_apply_requested;
static uint8_t fw_apply_delay_loops;
static volatile uint32_t fw_expected_size;
static uint32_t fw_stage_base_lba;
static uint8_t fw_stage_started;
static uint8_t fw_stage_pages[FW_STAGE_PAGE_BYTES];
static uint8_t fw_stage_sectors[FW_STAGE_SECTOR_BYTES];
static uint8_t fw_page_buffer[FW_PAGE_SIZE];

// The sector being filled, kept in RAM until every one of its bytes has
// arrived. A sector reaches flash complete or not at all, so a short write
// can never be mistaken for staged data.
static uint8_t fw_sector_buffer[FW_SECTOR_SIZE];
static uint8_t fw_sector_cover[FW_SECTOR_COVER_BYTES];
static uint32_t fw_sector_index;
static uint8_t fw_sector_held;

static void flash_wait(uint32_t addr) {
    while (FLASH_STS_OF(addr) & FLASH_STS_BSY) {
    }
}

static void flash_unlock(uint32_t addr) {
    if (FLASH_CTRL_OF(addr) & FLASH_CTRL_LOCK) {
        FLASH_KEYR_OF(addr) = 0x45670123u;
        FLASH_KEYR_OF(addr) = 0xCDEF89ABu;
    }
}

static void flash_lock(uint32_t addr) {
    FLASH_CTRL_OF(addr) |= FLASH_CTRL_LOCK;
}

static void flash_clear_status(uint32_t addr) {
    FLASH_STS_OF(addr) = FLASH_STS_EOP | FLASH_STS_PGERR | FLASH_STS_WRPRTERR;
}

static uint8_t flash_erase_page(uint32_t addr) {
    uint8_t ok;

    flash_wait(addr);
    flash_clear_status(addr);
    FLASH_CTRL_OF(addr) |= FLASH_CTRL_PER;
    FLASH_ADDR_OF(addr) = addr;
    FLASH_CTRL_OF(addr) |= FLASH_CTRL_STRT;
    flash_wait(addr);
    FLASH_CTRL_OF(addr) &= ~FLASH_CTRL_PER;
    ok = (FLASH_STS_OF(addr) & (FLASH_STS_PGERR | FLASH_STS_WRPRTERR)) ? 0u : 1u;
    flash_clear_status(addr);
    return ok;
}

static uint8_t flash_program_halfword(uint32_t addr, uint16_t value) {
    uint8_t ok;

    flash_wait(addr);
    flash_clear_status(addr);
    FLASH_CTRL_OF(addr) |= FLASH_CTRL_PG;
    REG16(addr) = value;
    flash_wait(addr);
    FLASH_CTRL_OF(addr) &= ~FLASH_CTRL_PG;
    ok = (FLASH_STS_OF(addr) & (FLASH_STS_PGERR | FLASH_STS_WRPRTERR)) ? 0u : 1u;
    flash_clear_status(addr);
    return ok;
}

static uint8_t bit_get(const uint8_t *bits, uint32_t bit) {
    return (bits[bit >> 3] & (uint8_t)(1u << (bit & 7u))) ? 1u : 0u;
}

static void bit_set(uint8_t *bits, uint32_t bit) {
    bits[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
}

static void bits_clear(uint8_t *bits, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        bits[i] = 0;
    }
}

static uint32_t min_u32(uint32_t a, uint32_t b) {
    return a < b ? a : b;
}

static uint8_t vector_valid(const uint8_t *data) {
    uint32_t sp = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                  ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    uint32_t pc = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                  ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

    if (sp < FW_RAM_BASE || sp > FW_RAM_END) {
        return 0;
    }
    if ((pc & 1u) == 0) {
        return 0;
    }
    pc &= ~1u;
    return pc >= FW_APP_BASE && pc < FW_APP_END;
}

static void fw_set_error(uint8_t error) {
    fw_status.error = error;
    fw_status.state = FW_UPDATE_STATE_ERROR;
    ++fw_status.sequence;
}

static uint8_t fw_stage_expected_complete(void) {
    uint32_t sectors;

    if (fw_expected_size < FW_MIN_SIZE || fw_expected_size > FW_MAX_SIZE) {
        return 0;
    }
    sectors = (fw_expected_size + FW_SECTOR_SIZE - 1u) / FW_SECTOR_SIZE;
    for (uint32_t sector = 0; sector < sectors; ++sector) {
        if (!bit_get(fw_stage_sectors, sector)) {
            return 0;
        }
    }
    return 1;
}

static void fw_maybe_ready(void) {
    if (fw_status.state == FW_UPDATE_STATE_STAGING &&
        fw_expected_size >= FW_MIN_SIZE &&
        fw_expected_size <= FW_MAX_SIZE &&
        fw_status.bytes >= fw_expected_size &&
        fw_stage_expected_complete()) {
        fw_status.state = FW_UPDATE_STATE_READY;
        ++fw_status.sequence;
    }
}

static void fw_sector_release(void) {
    fw_sector_held = 0;
    fw_sector_index = 0;
    bits_clear(fw_sector_cover, sizeof(fw_sector_cover));
}

static void fw_stage_reset(uint32_t base_lba) {
    fw_stage_started = 1;
    fw_stage_base_lba = base_lba;
    fw_status.state = FW_UPDATE_STATE_STAGING;
    fw_status.error = FW_UPDATE_ERR_NONE;
    fw_status.bytes = 0;
    fw_status.expected_size = fw_expected_size;
    fw_status.base_lba = base_lba;
    fw_apply_requested = 0;
    fw_apply_delay_loops = 0;
    bits_clear(fw_stage_pages, sizeof(fw_stage_pages));
    bits_clear(fw_stage_sectors, sizeof(fw_stage_sectors));
    fw_sector_release();
    ++fw_status.sequence;
}

// How many bytes of this sector belong to the image. Only the last sector is
// shorter than 512 bytes, and only once the file size is known.
static uint32_t fw_sector_span(uint32_t sector) {
    uint32_t base = sector * FW_SECTOR_SIZE;
    uint32_t limit = fw_expected_size ? fw_expected_size : FW_MAX_SIZE;

    if (limit > FW_MAX_SIZE) {
        limit = FW_MAX_SIZE;
    }
    if (base >= limit) {
        return 0;
    }
    return min_u32(limit - base, FW_SECTOR_SIZE);
}

static uint8_t fw_sector_covered(uint32_t span) {
    for (uint32_t i = 0; i < span; ++i) {
        if (!bit_get(fw_sector_cover, i)) {
            return 0;
        }
    }
    return 1;
}

static uint8_t fw_stage_matches(uint32_t sector, const uint8_t *data) {
    uint32_t addr = FW_STAGE_BASE + sector * FW_SECTOR_SIZE;

    for (uint32_t i = 0; i < FW_SECTOR_SIZE; i += 2u) {
        uint16_t value = (uint16_t)data[i] | ((uint16_t)data[i + 1u] << 8);
        if (REG16(addr + i) != value) {
            return 0;
        }
    }
    return 1;
}

static uint8_t fw_stage_program_and_verify(uint32_t sector, const uint8_t *data) {
    uint32_t addr = FW_STAGE_BASE + sector * FW_SECTOR_SIZE;

    for (uint32_t i = 0; i < FW_SECTOR_SIZE; i += 2u) {
        uint16_t value = (uint16_t)data[i] | ((uint16_t)data[i + 1u] << 8);
        if (value != 0xFFFFu && !flash_program_halfword(addr + i, value)) {
            return 0;
        }
    }
    // Check the whole sector, including skipped 0xFFFF words: a silent
    // erase failure can leave old data even when programming reports success.
    return fw_stage_matches(sector, data);
}

static uint8_t fw_stage_write_new_sector(uint32_t sector, const uint8_t *data) {
    uint32_t page = sector / FW_SECTORS_PER_PAGE;
    if (!bit_get(fw_stage_pages, page)) {
        if (!flash_erase_page(FW_STAGE_BASE + page * FW_PAGE_SIZE)) {
            return 0;
        }
        bit_set(fw_stage_pages, page);
    }
    return fw_stage_program_and_verify(sector, data);
}

// Restaging a sector that already holds different data needs the whole 2 KiB
// page erased, so the page's other staged sectors are copied out first.
static uint8_t fw_stage_rewrite_page(uint32_t sector, const uint8_t *data) {
    uint32_t page = sector / FW_SECTORS_PER_PAGE;
    uint32_t page_addr = FW_STAGE_BASE + page * FW_PAGE_SIZE;
    uint32_t first = page * FW_SECTORS_PER_PAGE;
    uint8_t keep[FW_SECTORS_PER_PAGE];

    for (uint32_t s = 0; s < FW_SECTORS_PER_PAGE; ++s) {
        uint32_t index = first + s;
        uint32_t offset = s * FW_SECTOR_SIZE;

        keep[s] = (index == sector) ||
                  (index < FW_STAGE_SECTOR_COUNT && bit_get(fw_stage_sectors, index));
        if (index == sector) {
            for (uint32_t i = 0; i < FW_SECTOR_SIZE; ++i) {
                fw_page_buffer[offset + i] = data[i];
            }
        } else if (keep[s]) {
            for (uint32_t i = 0; i < FW_SECTOR_SIZE; i += 2u) {
                uint16_t value = REG16(page_addr + offset + i);
                fw_page_buffer[offset + i] = (uint8_t)value;
                fw_page_buffer[offset + i + 1u] = (uint8_t)(value >> 8);
            }
        }
    }

    // A failed erase leaves the page in an unknown state; the caller turns
    // that into FW_UPDATE_ERR_FLASH and the whole staging run is abandoned.
    if (!flash_erase_page(page_addr)) {
        return 0;
    }
    bit_set(fw_stage_pages, page);

    for (uint32_t s = 0; s < FW_SECTORS_PER_PAGE; ++s) {
        if (keep[s] && !fw_stage_program_and_verify(first + s, fw_page_buffer + s * FW_SECTOR_SIZE)) {
            return 0;
        }
    }
    return 1;
}

// Write the held sector out once every byte of it has arrived. force says the
// caller is about to move to another sector, so an incomplete one is an error
// rather than a sector still being filled.
static uint8_t fw_sector_commit(uint8_t force) {
    uint32_t sector = fw_sector_index;
    uint32_t span;
    uint32_t end;
    uint8_t ok;

    if (!fw_sector_held) {
        return 1;
    }
    span = fw_sector_span(sector);
    if (!span || !fw_sector_covered(span)) {
        if (!force) {
            return 1;
        }
        fw_sector_release();
        fw_set_error(FW_UPDATE_ERR_ORDER);
        return 0;
    }

    flash_unlock(FW_STAGE_BASE);
    if (bit_get(fw_stage_sectors, sector)) {
        ok = fw_stage_matches(sector, fw_sector_buffer) ? 1u :
             fw_stage_rewrite_page(sector, fw_sector_buffer);
    } else {
        ok = fw_stage_write_new_sector(sector, fw_sector_buffer);
    }
    flash_lock(FW_STAGE_BASE);
    fw_sector_release();

    if (!ok) {
        fw_set_error(FW_UPDATE_ERR_FLASH);
        return 0;
    }
    bit_set(fw_stage_sectors, sector);
    end = sector * FW_SECTOR_SIZE + span;
    if (fw_status.bytes < end) {
        fw_status.bytes = end;
        ++fw_status.sequence;
    }
    return 1;
}

static void fw_sector_begin(uint32_t sector) {
    fw_sector_index = sector;
    fw_sector_held = 1;
    bits_clear(fw_sector_cover, sizeof(fw_sector_cover));
    for (uint32_t i = 0; i < FW_SECTOR_SIZE; ++i) {
        fw_sector_buffer[i] = 0xFFu;
    }
}

static uint8_t fw_stage_write(uint32_t offset, const uint8_t *data, uint16_t len) {
    if (offset >= FW_MAX_SIZE || (uint32_t)len > FW_MAX_SIZE - offset) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return 0;
    }

    while (len) {
        uint32_t sector = offset / FW_SECTOR_SIZE;
        uint32_t inner = offset % FW_SECTOR_SIZE;
        uint32_t chunk = min_u32(len, FW_SECTOR_SIZE - inner);

        if (fw_sector_held && fw_sector_index != sector && !fw_sector_commit(1)) {
            return 0;
        }
        if (!fw_sector_held) {
            fw_sector_begin(sector);
        }
        for (uint32_t i = 0; i < chunk; ++i) {
            fw_sector_buffer[inner + i] = data[i];
            bit_set(fw_sector_cover, inner + i);
        }
        if (!fw_sector_commit(0)) {
            return 0;
        }
        offset += chunk;
        data += chunk;
        len = (uint16_t)(len - chunk);
    }

    fw_maybe_ready();
    return 1;
}

void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len) {
    uint32_t offset;

    if (!data || !len || fw_status.state == FW_UPDATE_STATE_ERROR) {
        return;
    }

    if (!fw_stage_started) {
        if (sector_offset != 0 || len < 8u || !vector_valid(data)) {
            return;
        }
        fw_stage_reset(lba);
    }

    if (lba < fw_stage_base_lba) {
        return;
    }
    offset = (lba - fw_stage_base_lba) * 512u + sector_offset;
    if (fw_expected_size) {
        if (offset >= fw_expected_size) {
            return;
        }
        if ((uint32_t)len > fw_expected_size - offset) {
            len = (uint16_t)(fw_expected_size - offset);
        }
    }
    if (!len) {
        return;
    }
    (void)fw_stage_write(offset, data, len);
}

uint8_t fw_update_request_apply(void) {
    if (fw_status.state == FW_UPDATE_STATE_READY &&
        fw_expected_size >= FW_MIN_SIZE &&
        fw_expected_size <= FW_MAX_SIZE &&
        fw_status.bytes >= fw_expected_size) {
        fw_apply_requested = 1;
        fw_apply_delay_loops = FW_APPLY_DELAY_LOOPS;
        fw_status.state = FW_UPDATE_STATE_APPLYING;
        ++fw_status.sequence;
        return 1;
    }
    return 0;
}

static void fw_update_note_file_size(uint32_t size) {
    if (size >= FW_MIN_SIZE && size <= FW_MAX_SIZE) {
        if (size > fw_expected_size) {
            fw_expected_size = size;
            fw_status.expected_size = size;
            if (fw_status.state == FW_UPDATE_STATE_READY && fw_status.bytes < fw_expected_size) {
                fw_status.state = FW_UPDATE_STATE_STAGING;
            }
            ++fw_status.sequence;
        }
        fw_maybe_ready();
    } else if (size) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
    }
}

void fw_update_note_file(uint32_t base_lba, uint32_t size) {
    if (size < FW_MIN_SIZE || size > FW_MAX_SIZE) {
        if (size) {
            fw_set_error(FW_UPDATE_ERR_RANGE);
        }
        return;
    }
    if (base_lba && (!fw_stage_started || fw_stage_base_lba != base_lba)) {
        if (fw_status.state != FW_UPDATE_STATE_APPLYING) {
            fw_expected_size = size;
            fw_stage_reset(base_lba);
        }
    }
    fw_update_note_file_size(size);
}

void fw_update_staging_failed(void) {
    if (fw_status.state != FW_UPDATE_STATE_ERROR && fw_status.state != FW_UPDATE_STATE_APPLYING) {
        fw_set_error(FW_UPDATE_ERR_ORDER);
    }
}

void fw_update_clear(void) {
    if (fw_status.state == FW_UPDATE_STATE_APPLYING) {
        return;
    }
    fw_apply_requested = 0;
    fw_apply_delay_loops = 0;
    fw_expected_size = 0;
    fw_stage_started = 0;
    fw_stage_base_lba = 0;
    fw_sector_release();
    fw_status.state = FW_UPDATE_STATE_IDLE;
    fw_status.error = FW_UPDATE_ERR_NONE;
    fw_status.bytes = 0;
    fw_status.expected_size = 0;
    fw_status.base_lba = 0;
    ++fw_status.sequence;
}

#if defined(__arm__)
__attribute__((section(".data.ramfunc"), noinline, used))
#endif
static void fw_ram_install(uint32_t src, uint32_t dst, uint32_t size) {
    volatile uint32_t *flash_sts = (volatile uint32_t *)0x4002200Cu;
    volatile uint32_t *flash_ctrl = (volatile uint32_t *)0x40022010u;
    volatile uint32_t *flash_addr = (volatile uint32_t *)0x40022014u;
    volatile uint32_t *flash_keyr = (volatile uint32_t *)0x40022004u;
    volatile uint8_t *page = fw_page_buffer;
    uint32_t end;
    uint32_t page_size = 2048u;
    uint32_t timeout;
    uint32_t complete = 0;

    __asm__ volatile("cpsid i" ::: "memory");

    // The application always lives in bank 1, so these are the bank 1
    // registers. The staging area is only ever read here.
    if (dst != FW_APP_BASE || size == 0 || size > FW_MAX_SIZE) {
        goto fail_now;
    }
    end = dst + ((size + 2047u) & ~2047u);
    if (end > FW_APP_END) {
        goto fail_now;
    }

    if (*flash_ctrl & (1u << 7)) {
        *flash_keyr = 0x45670123u;
        *flash_keyr = 0xCDEF89ABu;
    }
    if (*flash_ctrl & (1u << 7)) {
        goto fail_now;
    }

    for (uint32_t addr = dst; addr < end; addr += 2048u) {
        uint32_t page_off = addr - dst;
        uint32_t page_bytes = size - page_off;
        if (page_bytes > page_size) {
            page_bytes = page_size;
        }
        uint32_t write_bytes = (page_bytes + 1u) & ~1u;
        for (uint32_t i = 0; i < page_size; ++i) {
            page[i] = i < page_bytes ? *(volatile uint8_t *)(uintptr_t)(src + page_off + i) : 0xFFu;
        }

        timeout = 0x00FFFFFFu;
        while ((*flash_sts & 1u) && --timeout) {
        }
        if (!timeout) {
            goto fail_now;
        }
        *flash_sts = (1u << 5) | (1u << 2) | (1u << 4);
        *flash_ctrl |= 1u << 1;
        *flash_addr = addr;
        *flash_ctrl |= 1u << 6;
        timeout = 0x00FFFFFFu;
        while ((*flash_sts & 1u) && --timeout) {
        }
        if (!timeout || (*flash_sts & ((1u << 2) | (1u << 4)))) {
            goto fail_now;
        }
        *flash_ctrl &= ~(1u << 1);

        for (uint32_t off = 0; off < write_bytes; off += 2u) {
            uint16_t value = (uint16_t)page[off] | ((uint16_t)page[off + 1u] << 8);
            timeout = 0x00FFFFFFu;
            while ((*flash_sts & 1u) && --timeout) {
            }
            if (!timeout) {
                goto fail_now;
            }
            *flash_sts = (1u << 5) | (1u << 2) | (1u << 4);
            *flash_ctrl |= 1u;
            *(volatile uint16_t *)(uintptr_t)(addr + off) = value;
            timeout = 0x00FFFFFFu;
            while ((*flash_sts & 1u) && --timeout) {
            }
            if (!timeout || (*flash_sts & ((1u << 2) | (1u << 4)))) {
                goto fail_now;
            }
            *flash_ctrl &= ~1u;
        }

        for (uint32_t i = 0; i < page_bytes; ++i) {
            if (*(volatile uint8_t *)(uintptr_t)(addr + i) != page[i]) {
                goto fail_now;
            }
        }
    }

    complete = 1;

fail_now:
    *flash_ctrl &= ~((1u << 0) | (1u << 1));
    *flash_ctrl |= 1u << 7;
    if (complete) {
        *(volatile uint32_t *)0x40021018u |= (1u << 3); // GPIOB clock
        *(volatile uint32_t *)0x40010C00u =
            (*(volatile uint32_t *)0x40010C00u & ~(0xFu << 8)) | (0x1u << 8);
        *(volatile uint32_t *)0x40010C10u = 1u << 2; // keep PB2 power hold high
        *(volatile uint32_t *)0xE000E010u = 0; // SysTick off
        for (uint32_t i = 0; i < 8u; ++i) {
            *(volatile uint32_t *)(uintptr_t)(0xE000E180u + i * 4u) = 0xFFFFFFFFu;
            *(volatile uint32_t *)(uintptr_t)(0xE000E280u + i * 4u) = 0xFFFFFFFFu;
        }
        *(volatile uint32_t *)0xE000ED08u = dst;
        __asm__ volatile(
            "dsb\n"
            "isb\n"
            "ldr r0, [%0]\n"
            "ldr r1, [%0, #4]\n"
            "msr msp, r0\n"
            "bx r1\n"
            :
            : "r"(dst)
            : "r0", "r1", "memory");
    }
    while (1) {
    }
}

void fw_update_service(void) {
    uint32_t size;

    if (!fw_apply_requested || fw_status.state != FW_UPDATE_STATE_APPLYING) {
        return;
    }
    if (fw_apply_delay_loops) {
        --fw_apply_delay_loops;
        return;
    }
    fw_apply_requested = 0;
    size = fw_expected_size ? fw_expected_size : fw_status.bytes;
    size = (size + 1u) & ~1u;
    if (size < FW_MIN_SIZE || size > FW_MAX_SIZE) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return;
    }
    if (!vector_valid((const uint8_t *)FW_STAGE_BASE)) {
        fw_set_error(FW_UPDATE_ERR_VECTOR);
        return;
    }
    fw_status.state = FW_UPDATE_STATE_APPLYING;
    ++fw_status.sequence;
    fw_ram_install(FW_STAGE_BASE, FW_APP_BASE, size);
}

void fw_update_status(fw_update_status_t *status) {
    if (!status) {
        return;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    *status = fw_status;
    __asm__ volatile("cpsie i" ::: "memory");
}
