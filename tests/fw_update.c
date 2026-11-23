// Host tests for the in-app updater's staging path.
//
// The flash is modelled closely enough to reproduce the failure that broke
// in-app updates on HW4.0: the 1 MiB flash is two independently controlled
// banks, and a program or erase aimed at a bank 2 address through the bank 1
// registers reports success while doing nothing at all. The staging area used
// to start at 0x08050000, so every byte of an image past 0x08080000 was
// dropped on the floor. Nothing read the staged image back, so the updater
// declared itself READY and flashed a truncated application that could not
// boot.
//
// The model therefore enforces three things the real part enforces:
//   * a halfword is only programmed when the owning bank is unlocked and has
//     its PG bit set, so a write through the wrong register set vanishes;
//   * programming can only clear bits, and programming a cell that is not
//     blank raises PGERR;
//   * an erase only happens when FLASH_ADDR belongs to the bank whose STRT
//     bit was set.
//
// Writes through a macro cannot run code, so every accessor first reconciles
// the previous one: it looks at what the caller left in the cell or register
// it was handed and applies the hardware's semantics then. The updater
// checks status after programming and verifies the sector before counting it.

#include "../src/hw.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/app_config.h"

#define STAGE_BASE FW_STAGE_BASE_ADDR
#define STAGE_BYTES APP_FLASH_SIZE_BYTES
#define BANK2_BASE 0x08080000u
#define PAGE_SIZE 2048u

#define STS_PGERR (1u << 2)
#define STS_WRPRTERR (1u << 4)
#define STS_EOP (1u << 5)
#define CTRL_PG (1u << 0)
#define CTRL_PER (1u << 1)
#define CTRL_STRT (1u << 6)
#define CTRL_LOCK (1u << 7)

static uint16_t cells[STAGE_BYTES / 2u];
static uint16_t gate;

static struct {
    uint32_t sts;
    uint32_t ctrl;
    uint32_t addr;
    uint32_t keyr;
    uint32_t keyr_step;
} banks[2];

// Test hooks: addresses at or above drop_writes_from are never programmed,
// and fail_erase_page refuses to erase one page.
static uint32_t drop_writes_from = 0xFFFFFFFFu;
static uint32_t fail_erase_page = 0xFFFFFFFFu;
static unsigned erase_count;
static unsigned program_count;

#define STS_W1C (STS_EOP | STS_PGERR | STS_WRPRTERR)

enum { LAST_NONE, LAST_CELL, LAST_STS, LAST_CTRL, LAST_ADDR, LAST_KEYR };
static int last_kind = LAST_NONE;
static unsigned last_bank;
static uint32_t last_index;
static uint32_t last_prev;
// FLASH_STS is write-1-to-clear, so the caller never gets the field itself.
static uint32_t sts_scratch;

static unsigned bank_of(uint32_t addr) {
    return addr >= BANK2_BASE ? 1u : 0u;
}

static void do_erase(unsigned bank) {
    uint32_t addr = banks[bank].addr & ~(PAGE_SIZE - 1u);
    uint32_t page;

    banks[bank].ctrl &= ~CTRL_STRT;
    banks[bank].sts |= STS_EOP;
    if (banks[bank].ctrl & CTRL_LOCK) {
        banks[bank].sts |= STS_WRPRTERR;
        return;
    }
    // The decisive rule: the bank that owns the address must be the bank
    // whose registers were driven, or nothing happens.
    if (bank_of(addr) != bank) {
        return;
    }
    if (addr < STAGE_BASE || addr + PAGE_SIZE > STAGE_BASE + STAGE_BYTES) {
        return;
    }
    page = (addr - STAGE_BASE) / PAGE_SIZE;
    if (page == fail_erase_page) {
        return;
    }
    ++erase_count;
    for (uint32_t i = 0; i < PAGE_SIZE; i += 2u) {
        cells[(addr - STAGE_BASE + i) / 2u] = 0xFFFFu;
    }
}

static void reconcile(void) {
    int kind = last_kind;
    unsigned bank = last_bank;
    uint32_t index = last_index;
    uint32_t prev = last_prev;

    last_kind = LAST_NONE;
    switch (kind) {
    case LAST_CELL: {
        uint16_t want = cells[index];
        uint16_t old = (uint16_t)prev;
        if (want == old) {
            break;
        }
        ++program_count;
        if (old != 0xFFFFu && want != old) {
            banks[bank].sts |= STS_PGERR;
        }
        cells[index] = (uint16_t)(old & want); // programming only clears bits
        banks[bank].sts |= STS_EOP;
        break;
    }
    case LAST_STS:
        // Writing a 1 clears that flag; the status word is never set this way.
        if (sts_scratch != prev || sts_scratch == STS_W1C) {
            banks[bank].sts = prev & ~(sts_scratch & STS_W1C);
        }
        break;
    case LAST_CTRL:
        if ((banks[bank].ctrl & CTRL_PER) && (banks[bank].ctrl & CTRL_STRT)) {
            do_erase(bank);
        }
        break;
    case LAST_KEYR:
        if (banks[bank].keyr == 0x45670123u) {
            banks[bank].keyr_step = 1u;
        } else if (banks[bank].keyr == 0xCDEF89ABu && banks[bank].keyr_step == 1u) {
            banks[bank].ctrl &= ~CTRL_LOCK;
            banks[bank].keyr_step = 0u;
        } else {
            banks[bank].keyr_step = 0u;
        }
        break;
    default:
        break;
    }
}

static uint32_t *test_reg32(uint32_t addr) {
    uint32_t offset = addr - FLASH_R_BASE;
    unsigned bank = offset >= 0x40u ? 1u : 0u;

    reconcile();
    offset -= bank * 0x40u;
    last_bank = bank;
    switch (offset) {
    case 0x04u:
        last_kind = LAST_KEYR;
        return &banks[bank].keyr;
    case 0x0Cu:
        last_kind = LAST_STS;
        last_prev = banks[bank].sts;
        sts_scratch = banks[bank].sts;
        return &sts_scratch;
    case 0x10u:
        last_kind = LAST_CTRL;
        return &banks[bank].ctrl;
    case 0x14u:
        last_kind = LAST_ADDR;
        return &banks[bank].addr;
    default:
        break;
    }
    fprintf(stderr, "unexpected flash register %#x\n", addr);
    abort();
}

static uint16_t *test_reg16(uint32_t addr) {
    unsigned bank = bank_of(addr);
    uint32_t index;

    reconcile();
    assert(addr >= STAGE_BASE && addr + 2u <= STAGE_BASE + STAGE_BYTES);
    assert((addr & 1u) == 0u);
    index = (addr - STAGE_BASE) / 2u;

    // Programming needs the owning bank unlocked with PG set. Otherwise the
    // caller gets a scratch cell: reads still work, writes go nowhere.
    if (addr < drop_writes_from && !(banks[bank].ctrl & CTRL_LOCK) &&
        (banks[bank].ctrl & CTRL_PG)) {
        last_kind = LAST_CELL;
        last_bank = bank;
        last_index = index;
        last_prev = cells[index];
        return &cells[index];
    }
    gate = cells[index];
    return &gate;
}

#undef REG32
#undef REG16
#define REG32(addr) (*test_reg32((uint32_t)(addr)))
#define REG16(addr) (*test_reg16((uint32_t)(addr)))

// Build the real staging code without the Cortex-M interrupt instructions.
#define __asm__
#define volatile(...)
#include "../src/fw_update.c"
#undef volatile
#undef __asm__

#define STAGE_LBA 1u

static void flash_reset(void) {
    memset(cells, 0xFF, sizeof(cells));
    memset(banks, 0, sizeof(banks));
    banks[0].ctrl = CTRL_LOCK;
    banks[1].ctrl = CTRL_LOCK;
    last_kind = LAST_NONE;
    drop_writes_from = 0xFFFFFFFFu;
    fail_erase_page = 0xFFFFFFFFu;
    erase_count = 0;
    program_count = 0;
    // fw_update_clear() refuses to run while an apply is pending, which is
    // right on the device and in the way here.
    fw_status.state = FW_UPDATE_STATE_IDLE;
    fw_update_clear();
}

static uint8_t staged_byte(uint32_t offset) {
    uint16_t cell = cells[offset / 2u];
    return (offset & 1u) ? (uint8_t)(cell >> 8) : (uint8_t)cell;
}

// A plausible firmware image: a vector table the updater accepts, then
// repeatable pseudo-random payload.
static void make_image(uint8_t *image, uint32_t size) {
    uint32_t sp = 0x20036078u;
    uint32_t pc = APP_BASE_ADDR + 0x1C1u;
    uint32_t state = 0x12345678u ^ size;

    for (uint32_t i = 0; i < 4u; ++i) {
        image[i] = (uint8_t)(sp >> (8u * i));
        image[4u + i] = (uint8_t)(pc >> (8u * i));
    }
    for (uint32_t i = 8u; i < size; ++i) {
        state = state * 1103515245u + 12345u;
        image[i] = (uint8_t)(state >> 16);
    }
}

// Feed the image the way raw_fat_stage_update_file does: announce the file,
// then hand over one FAT sector at a time, in order.
static void feed_image(const uint8_t *image, uint32_t size, uint32_t fat_sector) {
    uint32_t offset = 0;

    fw_update_note_file(STAGE_LBA, size);
    while (offset < size) {
        uint32_t chunk = size - offset;
        if (chunk > fat_sector) {
            chunk = fat_sector;
        }
        fw_update_usb_data(STAGE_LBA + offset / 512u, (uint16_t)(offset & 511u),
                           &image[offset], (uint16_t)chunk);
        offset += chunk;
    }
}

static void check_staged(const uint8_t *image, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) {
        if (staged_byte(i) != image[i]) {
            fprintf(stderr, "staged byte %u: %#02x, expected %#02x\n", i,
                    staged_byte(i), image[i]);
            abort();
        }
    }
}

// An image that fills most of the application slot must stage byte for byte
// and reach READY, whatever FAT sector size the drive was formatted with.
static void test_full_image(void) {
    static uint8_t image[STAGE_BYTES];
    const uint32_t sizes[] = {
        8192u,                  // the smallest the updater accepts
        268214u > STAGE_BYTES ? STAGE_BYTES : 268214u, // v2026.11.6 HW4.0
        STAGE_BYTES - 512u,
        STAGE_BYTES,
    };
    const uint32_t fat_sectors[] = {512u, 1024u, 2048u, 4096u};
    fw_update_status_t status;

    for (unsigned s = 0; s < sizeof(sizes) / sizeof(sizes[0]); ++s) {
        for (unsigned f = 0; f < sizeof(fat_sectors) / sizeof(fat_sectors[0]); ++f) {
            uint32_t size = sizes[s];

            make_image(image, size);
            flash_reset();
            feed_image(image, size, fat_sectors[f]);
            fw_update_status(&status);
            assert(status.error == FW_UPDATE_ERR_NONE);
            assert(status.state == FW_UPDATE_STATE_READY);
            assert(status.bytes == size);
            assert(status.expected_size == size);
            check_staged(image, size);
            assert(fw_update_request_apply());
        }
    }
}

// A size that is not a multiple of 512 leaves a short last sector: it must
// still be staged, and the bytes past the end left erased.
static void test_odd_tail(void) {
    static uint8_t image[70000];
    const uint32_t size = 69001u; // odd, and not a sector multiple
    fw_update_status_t status;

    make_image(image, size);
    flash_reset();
    feed_image(image, size, 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_READY);
    assert(status.bytes == size);
    check_staged(image, size);
    for (uint32_t i = size; i < ((size + 511u) & ~511u); ++i) {
        assert(staged_byte(i) == 0xFFu);
    }
}

// The HW4.0 regression. Writes past a point are dropped, exactly as they were
// when the staging area crossed 0x08080000 and the code drove only the bank 1
// registers. The updater must report a flash failure instead of declaring
// itself ready with a truncated image.
static void test_lost_writes_are_detected(void) {
    static uint8_t image[STAGE_BYTES];
    const uint32_t size = STAGE_BYTES > 268214u ? 268214u : STAGE_BYTES;
    const uint32_t reach = 196608u; // 0x08080000 - 0x08050000
    fw_update_status_t status;

    if (size <= reach) {
        return; // the slot is too small for this to be reachable
    }
    make_image(image, size);
    flash_reset();
    drop_writes_from = STAGE_BASE + reach;
    feed_image(image, size, 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_ERROR);
    assert(status.error == FW_UPDATE_ERR_FLASH);
    assert(!fw_update_request_apply());

    assert(status.bytes == reach); // The failed sector is not counted.
}

// An erase that does not take must be caught as well.
static void test_failed_erase_is_detected(void) {
    static uint8_t image[40000];
    const uint32_t size = 40000u;
    const uint32_t page = 7u;
    const uint32_t stale_offsets[] = {0u, FW_SECTOR_SIZE + 2u};
    fw_update_status_t status;

    for (unsigned i = 0; i < sizeof(stale_offsets) / sizeof(stale_offsets[0]); ++i) {
        uint32_t offset = page * PAGE_SIZE + stale_offsets[i];

        make_image(image, size);
        image[offset] = image[offset + 1u] = 0xFFu;
        flash_reset();
        fail_erase_page = page;
        // The second case leaves the page's first word blank. The stale
        // word would be missed by a page probe or write-only verification.
        cells[offset / 2u] = 0x1234u;
        feed_image(image, size, 512u);
        fw_update_staging_failed();
        fw_update_status(&status);
        assert(status.state == FW_UPDATE_STATE_ERROR);
        assert(status.error == FW_UPDATE_ERR_FLASH);
        assert(status.bytes == offset - offset % FW_SECTOR_SIZE);
        assert(!fw_update_request_apply());
    }

    // No reboot is needed after the error: the next transfer can use the slot.
    fail_erase_page = 0xFFFFFFFFu;
    fw_update_clear();
    feed_image(image, size, 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_READY && status.error == FW_UPDATE_ERR_NONE);
    check_staged(image, size);
}

// A sector left half filled when the next one starts is an error, not a
// sector quietly staged short.
static void test_partial_sector_is_rejected(void) {
    static uint8_t image[40000];
    const uint32_t size = 40000u;
    fw_update_status_t status;

    make_image(image, size);
    flash_reset();
    fw_update_note_file(STAGE_LBA, size);
    fw_update_usb_data(STAGE_LBA, 0, &image[0], 512u);
    fw_update_usb_data(STAGE_LBA + 1u, 0, &image[512], 256u); // short
    fw_update_usb_data(STAGE_LBA + 2u, 0, &image[1024], 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_ERROR);
    assert(status.error == FW_UPDATE_ERR_ORDER);

    // The same bytes delivered in two halves of one sector are fine.
    flash_reset();
    fw_update_note_file(STAGE_LBA, size);
    fw_update_usb_data(STAGE_LBA, 0, &image[0], 512u);
    fw_update_usb_data(STAGE_LBA + 1u, 0, &image[512], 256u);
    fw_update_usb_data(STAGE_LBA + 1u, 256u, &image[768], 256u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_STAGING);
    assert(status.error == FW_UPDATE_ERR_NONE);
    assert(status.bytes == 1024u);
    check_staged(image, 1024u);
}

// Staging the same file twice in a row, and re-sending a sector with
// different contents, both have to end with the staged image correct.
static void test_restage(void) {
    static uint8_t image[40000];
    static uint8_t other[40000];
    const uint32_t size = 40000u;
    fw_update_status_t status;

    make_image(image, size);
    make_image(other, size - 1u);
    flash_reset();
    feed_image(image, size, 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_READY);

    // A second run over the same staging area, as the device does when the
    // file is copied again.
    fw_update_clear();
    feed_image(other, size, 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_READY);
    check_staged(other, size);

    // One sector resent with different bytes: the page is rewritten and its
    // three other sectors survive.
    memcpy(image, other, size);
    for (uint32_t i = 0; i < 512u; ++i) {
        image[2048u + i] = (uint8_t)(i ^ 0x5Au);
    }
    fw_update_usb_data(STAGE_LBA + 4u, 0, &image[2048], 512u);
    fw_update_status(&status);
    assert(status.error == FW_UPDATE_ERR_NONE);
    check_staged(image, size);

    // A page rewrite must also check the neighbouring sectors it restores.
    drop_writes_from = STAGE_BASE + 2048u + 512u;
    image[2048] ^= 1u;
    fw_update_usb_data(STAGE_LBA + 4u, 0, &image[2048], 512u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_ERROR && status.error == FW_UPDATE_ERR_FLASH);
    assert(!fw_update_request_apply());
}

// Sizes the updater must refuse outright.
static void test_size_limits(void) {
    static uint8_t image[STAGE_BYTES];
    fw_update_status_t status;

    make_image(image, 8192u);
    flash_reset();
    fw_update_note_file(STAGE_LBA, (uint32_t)STAGE_BYTES + 2048u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_ERROR);
    assert(status.error == FW_UPDATE_ERR_RANGE);

    flash_reset();
    fw_update_note_file(STAGE_LBA, 4096u);
    fw_update_status(&status);
    assert(status.state == FW_UPDATE_STATE_ERROR);
    assert(status.error == FW_UPDATE_ERR_RANGE);
}

// The staging bookkeeping has to fit the RAM that is actually free. Tracking
// halfwords cost APP_FLASH_SIZE_BYTES/16 bytes, which is 31 KiB for a 484 KiB
// slot against about 8 KiB free.
static void test_ram_budget(void) {
    const size_t bookkeeping = sizeof(fw_stage_pages) + sizeof(fw_stage_sectors) +
                               sizeof(fw_sector_cover) + sizeof(fw_sector_buffer);

    assert(sizeof(fw_stage_sectors) == (STAGE_BYTES / 512u + 7u) / 8u);
    assert(bookkeeping <= 1024u);
    printf("fw_update: staging bookkeeping %u B for a %u B slot "
           "(halfword tracking would need %u B)\n",
           (unsigned)bookkeeping, (unsigned)STAGE_BYTES,
           (unsigned)(STAGE_BYTES / 16u));
}

int main(void) {
    test_ram_budget();
    test_full_image();
    test_odd_tail();
    test_lost_writes_are_detected();
    test_failed_erase_is_detected();
    test_partial_sector_is_rejected();
    test_restage();
    test_size_limits();
    printf("fw_update: staging, bank routing, read-back and size limits passed "
           "(app %#x, stage %#x, %u B)\n",
           (unsigned)APP_BASE_ADDR, (unsigned)STAGE_BASE, (unsigned)STAGE_BYTES);
    return 0;
}
