#include "../src/hw.h"
#include "../src/scope_timer.h"

#include <assert.h>
#include <stdio.h>

static struct { uint32_t address; uint32_t value; } registers[32];
static unsigned register_count;
static uint8_t transmitted[32];
static unsigned transmitted_count;

static volatile uint32_t *mock_register(uint32_t address) {
    for (unsigned i = 0; i < register_count; ++i) {
        if (registers[i].address == address) return &registers[i].value;
    }
    assert(register_count < 32);
    registers[register_count].address = address;
    return &registers[register_count++].value;
}

#undef REG32
#define REG32(address) (*mock_register(address))

static void mock_gpio_set(uint32_t base, uint32_t mask) {
    GPIO_ODR(base) |= mask;
#if HW_TARGET_HW40
    if (base == GPIOB_BASE && mask == (1u << 3)) {
        assert(transmitted_count < sizeof(transmitted));
        transmitted[transmitted_count++] = (uint8_t)GPIO_ODR(GPIOC_BASE);
    }
#else
    if (base == GPIOA_BASE && mask == (1u << 15)) {
        assert(transmitted_count < sizeof(transmitted));
        transmitted[transmitted_count++] = (uint8_t)SPI_DT(SPI3_BASE);
    }
#endif
}
static void mock_gpio_clear(uint32_t base, uint32_t mask) { GPIO_ODR(base) &= ~mask; }
static uint32_t mock_gpio_read(uint32_t base, uint32_t mask) { return GPIO_IDR(base) & mask; }
#define gpio_set mock_gpio_set
#define gpio_clear mock_gpio_clear
#define gpio_read mock_gpio_read
#include "../src/fpga.c"

void gpio_config_mask(uint32_t base, uint16_t mask, uint8_t cfg) {
    (void)base; (void)mask; (void)cfg;
}

static uint32_t read_be32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | bytes[3];
}

static void assert_frame(uint32_t word, uint32_t span) {
    unsigned offset = HW_TARGET_HW40 ? 0u : 1u;
    assert(transmitted_count == (HW_TARGET_HW40 ? 16u : 9u));
    assert(read_be32(&transmitted[offset]) == word);
    assert(read_be32(&transmitted[offset + 4]) == span);
    assert(fpga_last_tuning_word == word && fpga_last_span == span);
#if HW_TARGET_HW40
    for (unsigned i = 0; i < 8; ++i) assert(transmitted[8 + i] == i);
#else
    assert(transmitted[0] == 0);
#endif
    transmitted_count = 0;
}

static void test_roll_clock(void) {
    uint32_t ext = (1u << 16) | 8u;
    uint32_t cfg192 = ext | (7u << 18) | (1u << 29);
    uint32_t cfg240 = ext | (13u << 18) | (1u << 29);
    assert(scope_tmr1_hz(ext | (7u << 18), 0, 0) == 72000000u);
    assert(scope_tmr1_hz(cfg240, 0, 0) == 240000000u);
    assert(scope_tmr1_hz(0xc43fa40au, 0, 0) == 192000000u); // Measured on the legacy board.
    assert(scope_tmr1_hz(cfg192 | (7u << 8), 0, 0) == 192000000u); // APB1 is unrelated.
    for (uint32_t apb = 0; apb < 8; ++apb) {
        uint32_t cfg = cfg192 | (8u << 4) | (apb << 11); // AHB /2
        assert(scope_tmr1_hz(cfg, 0, 0) == (96000000u >> (apb > 4u ? apb - 4u : 0u)));
    }
    assert(scope_tmr1_hz(cfg192 | (1u << 17), 0, 1u << 12) == 64000000u); // HEXT /3
    assert(scope_tmr1_hz(cfg192 & ~(1u << 16), 0, 0) == 96000000u); // HICK /2
    assert(scope_tmr1_hz(0, 1u << 25, 1u << 9) == 48000000u);
    assert(scope_tmr1_hz(0, 1u << 25, 0) == 8000000u);
    assert(scope_tmr1_hz(4u, 1u << 25, 1u << 9) == 8000000u); // Direct HEXT
    const uint32_t clocks[] = {8000000, 72000000, 96000000, 192000000, 240000000};
    const uint16_t intervals_ms[] = {20, 400}; // 12 divisions / 300 roll points.
    for (unsigned i = 0; i < sizeof(clocks)/sizeof(clocks[0]); ++i) {
        for (unsigned j = 0; j < sizeof(intervals_ms)/sizeof(intervals_ms[0]); ++j) {
            uint16_t ms = intervals_ms[j];
            scope_timer_period_t p = scope_roll_period(clocks[i], ms);
            assert((p.psc + 1u) * (p.reload + 1u) == clocks[i] / 1000u * ms);
        }
    }
}

int main(void) {
    test_roll_clock();
#if !HW_TARGET_HW40
    fpga_clock_select_legacy(8u | (3u << 16) | (11u << 18) | (3u << 29)); // 240 MHz
    assert(FPGA_ADC_TICK_NS == 20u && fpga_phase_increment(1000) == 42950u);
    fpga_clock_select_legacy(8u | (3u << 16) | (15u << 18) | (2u << 29)); // 192 MHz
    assert(FPGA_ADC_TICK_NS == 25u && fpga_phase_increment(1000) == 53687u);
#endif
    uint32_t ref = HW_TARGET_HW40 ? 100000000u : 80000000u;
    for (uint32_t hz = 0; hz <= 2000000u; hz += 25u) {
        uint32_t expected = (uint32_t)((((uint64_t)hz << 32) + ref / 2u) / ref);
        uint32_t actual = fpga_phase_increment(hz);
        assert(actual + 1u >= expected && actual <= expected + 1u);
    }
    SPI_STS(SPI3_BASE) = 2;
    fpga_write_generator_timing(42950);
    assert_frame(42950, FPGA_SAMPLE_COUNT);
    static const uint32_t spans[] = {0, 52, 52428, 0x80000, 0x100000};
    for (unsigned i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        fpga_write_scope_timing(spans[i]);
        assert_frame(42950, spans[i]);
        fpga_write_generator_timing(85900);
        assert_frame(85900, spans[i]);
        fpga_write_generator_timing(0);
        assert_frame(0, spans[i]);
        fpga_write_generator_timing(42950);
        assert_frame(42950, spans[i]);
    }
    printf("FPGA timing: independent DDS/capture fields and hardware packet framing passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
