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
    flash[record_count++] = settings_state_record(type, payload);
}

static void put_u32(uint8_t lo_type, uint32_t value) {
    put_record(lo_type, (uint16_t)value);
    put_record((uint8_t)(lo_type + 1u), (uint16_t)(value >> 16));
}

static void test_split_values(void) {
    settings_state_t settings;

    memset(flash, 0xFF, sizeof(flash));
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

int main(void) {
    test_split_values();
    printf("settings: split 32-bit records restore without partial clamping passed\n");
    return 0;
}
