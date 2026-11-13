#include "../src/hw.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t flash[512];
static uint32_t *test_reg(uint32_t addr) {
    assert(addr >= 0x080FF800u && addr < 0x080FF800u + sizeof(flash));
    return &flash[(addr - 0x080FF800u) / 4u];
}
#undef REG32
#define REG32(addr) (*test_reg(addr))
// Test the real record loader without Cortex-M interrupt instructions.
#define __asm__
#define volatile(...)
#include "../src/settings.c"
#undef volatile
#undef __asm__

static unsigned record_count;

static void put_record(uint8_t type, uint16_t payload) {
    flash[1u + record_count++] = settings_state_record(type, payload);
}

static void put_u32(uint8_t lo_type, uint32_t value) {
    put_record(lo_type, (uint16_t)value);
    put_record((uint8_t)(lo_type + 1u), (uint16_t)(value >> 16));
}

static void test_split_values(void) {
    settings_state_t settings;

    memset(flash, 0xFF, sizeof(flash));
    flash[0] = SETTINGS_PAGE_MAGIC;
    record_count = 0;
    put_u32(SETTINGS_STATE_SIGGEN_FREQ_LO, 65536u);
    put_u32(SETTINGS_STATE_BODE_START_LO, 131072u);
    put_u32(SETTINGS_STATE_BODE_STOP_LO, 196608u);
    put_u32(SETTINGS_STATE_SIGGEN_SWEEP_START_LO, 65536u);
    put_u32(SETTINGS_STATE_SIGGEN_SWEEP_STOP_LO, 262144u);
    assert(settings_load(&settings));
    assert(settings.siggen_freq_hz == 65536u);
    assert(settings.bode_start_hz == 131072u && settings.bode_stop_hz == 196608u);
    assert(settings.siggen_sweep_start_hz == 65536u && settings.siggen_sweep_stop_hz == 262144u);

    // Out of range values are still clamped once everything is loaded.
    memset(flash, 0xFF, sizeof(flash));
    flash[0] = SETTINGS_PAGE_MAGIC;
    record_count = 0;
    put_u32(SETTINGS_STATE_SIGGEN_FREQ_LO, 0u);
    put_u32(SETTINGS_STATE_SIGGEN_SWEEP_MS_LO, 0x00010000u);
    put_u32(SETTINGS_STATE_SIGGEN_FM_FREQ_LO, 0x00010000u);
    put_record(SETTINGS_STATE_SCOPE_TRIGGER, 0u);
    assert(settings_load(&settings));
    assert(settings.siggen_freq_hz == SETTINGS_SIGGEN_DEFAULT_FREQ_HZ);
    assert(settings.siggen_sweep_ms == 9999u && settings.siggen_fm_freq_hz == SETTINGS_SIGGEN_FM_MAX_HZ);
    assert(settings.scope_trigger_level == 12u);
}

// The settings page lives at 0x080FF800, the last 2 KiB of flash, and a
// firmware update never erases it: flashing OpenScope over the stock FNIRSI
// firmware leaves whatever that firmware had written there.
//
// A state record carries no magic number at all -- only an 8-bit type byte
// (0xC9..0xE0) and an 8-bit XOR checksum -- so foreign words can be mistaken
// for saved settings. This measures how often that happens.
static void test_foreign_flash_is_rejected(void) {
    settings_state_t loaded;
    settings_state_t clean;
    unsigned pages_adopting = 0;
    unsigned siggen_hits = 0;
    const unsigned pages = 400u;

    // Reference: an erased page must yield untouched defaults.
    memset(flash, 0xFF, sizeof(flash));
    memset(&clean, 0, sizeof(clean));
    (void)settings_load(&clean);

    for (unsigned p = 0; p < pages; ++p) {
        // Reproducible pseudo-random filling, standing in for foreign data.
        uint32_t x = (p + 1u) * 2654435761u;
        for (unsigned i = 0; i < sizeof(flash) / sizeof(flash[0]); ++i) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            flash[i] = x;
        }
        // Count words a foreign page would pass off as generator settings.
        for (unsigned i = 0; i < sizeof(flash) / sizeof(flash[0]); ++i) {
            uint8_t type = (uint8_t)(flash[i] >> 24);
            uint16_t payload = (uint16_t)(flash[i] >> 8);
            if ((uint8_t)flash[i] != settings_state_checksum(type, payload)) continue;
            if (type == SETTINGS_STATE_SIGGEN_CORE ||
                type == SETTINGS_STATE_SIGGEN_FREQ_LO ||
                type == SETTINGS_STATE_SIGGEN_FREQ_HI) {
                ++siggen_hits;
            }
        }
        memset(&loaded, 0, sizeof(loaded));
        (void)settings_load(&loaded);
        if (memcmp(&loaded, &clean, sizeof(loaded)) != 0) {
            ++pages_adopting;
        }
    }

    // Without the page signature, 77 of these 400 pages changed the restored
    // settings and 11 words were taken for generator records -- an 8-bit type
    // plus an 8-bit XOR simply does not reject foreign data.
    printf("settings: %u/%u foreign pages changed the restored settings "
           "(%u would-be generator records ignored)\n",
           pages_adopting, pages, siggen_hits);
    assert(siggen_hits > 0u);   // foreign data does look like records...
    assert(pages_adopting == 0u); // ...but the signature keeps it out
}

// A page written by this firmware must still be read back normally.
static void test_signed_page_is_kept(void) {
    settings_state_t settings;

    memset(flash, 0xFF, sizeof(flash));
    flash[0] = SETTINGS_PAGE_MAGIC;
    record_count = 0;
    put_u32(SETTINGS_STATE_SIGGEN_FREQ_LO, 4242u);
    assert(settings_load(&settings));
    assert(settings.siggen_freq_hz == 4242u);

    // And an unsigned page must fall back to defaults, not to those records.
    flash[0] = 0xFFFFFFFFu;
    assert(!settings_load(&settings));
    assert(settings.siggen_freq_hz != 4242u);
}

int main(void) {
    test_split_values();
    test_foreign_flash_is_rejected();
    test_signed_page_is_kept();
    printf("settings: split records, page signature and foreign-page rejection passed\n");
    return 0;
}
