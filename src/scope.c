#include "scope.h"

#include "board.h"
#include "fpga.h"
#include "hw.h"

#include <stdint.h>

#ifndef SCOPE_HW_CAPTURE
#define SCOPE_HW_CAPTURE 0
#endif
#ifndef SCOPE_ANALOG_CONFIG
#define SCOPE_ANALOG_CONFIG 0
#endif
#ifndef SCOPE_ATTENUATOR_CONFIG
#define SCOPE_ATTENUATOR_CONFIG 0
#endif

enum {
    SCOPE_TIMING_SETTLE_MS = 2,
    SCOPE_ANALOG_RANGE_SETTLE_MS = 2,
    SCOPE_FAST_ALIGN_TIMEBASE_MAX = 3,
    SCOPE_HW_SLOW_TIMEBASE_START = 18,
    SCOPE_SLOW_POINT_COUNT = 300,
    SCOPE_SLOW_TIMER_TICK_HZ = 10000u,
};

enum {
    SCOPE_STATUS_IDLE,
    SCOPE_STATUS_NO_FPGA,
    SCOPE_STATUS_WAIT,
    SCOPE_STATUS_READ_ERROR,
    SCOPE_STATUS_FRAME,
};

static uint16_t scope_frame_count;
static uint16_t scope_wait_count;
static uint16_t scope_error_count;
static uint8_t scope_last_status;
static volatile uint8_t scope_slow_irq_enabled;
static volatile uint8_t scope_slow_samples[SCOPE_SLOW_POINT_COUNT * 2u];
static volatile uint16_t scope_slow_write_index;
static volatile uint16_t scope_slow_count;
static volatile uint16_t scope_slow_seq;

#if SCOPE_HW_CAPTURE && SCOPE_ANALOG_CONFIG
static uint8_t scope_analog_ready;
static uint16_t last_scope_dac[2] = {0xFFFFu, 0xFFFFu};
#if SCOPE_ATTENUATOR_CONFIG
static uint8_t last_scope_vdiv[2] = {0xFFu, 0xFFu};
#endif

#if SCOPE_ATTENUATOR_CONFIG
static void gpio_write_pin(uint32_t base, uint32_t mask, uint8_t value) {
    if (value) {
        gpio_set(base, mask);
    } else {
        gpio_clear(base, mask);
    }
}
#endif

static uint16_t scope_clamp_dac(uint16_t value) {
    return value > 4095u ? 4095u : value;
}

static void scope_dac_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac) {
    enum {
        DAC_DHR12RD_ADDR = 0x40007420u,
    };

    ch1_dac = scope_clamp_dac(ch1_dac);
    ch2_dac = scope_clamp_dac(ch2_dac);
    if (ch1_dac == last_scope_dac[0] && ch2_dac == last_scope_dac[1]) {
        return;
    }
    REG32(DAC_DHR12RD_ADDR) = (uint32_t)ch1_dac | ((uint32_t)ch2_dac << 16);
    last_scope_dac[0] = ch1_dac;
    last_scope_dac[1] = ch2_dac;
}

static void scope_dac_init(uint16_t ch1_dac, uint16_t ch2_dac) {
    enum {
        DAC_CR_ADDR = 0x40007400u,
        DAC_EN1 = 1u << 0,
        DAC_BOFF1 = 1u << 1,
        DAC_EN2 = 1u << 16,
        DAC_BOFF2 = 1u << 17,
    };

    RCC_APB1ENR |= 1u << 29; // DAC
    gpio_config_mask(GPIOA_BASE, (1u << 4) | (1u << 5), 0x0);
    REG32(DAC_CR_ADDR) = (REG32(DAC_CR_ADDR) & ~(DAC_BOFF1 | DAC_BOFF2)) | DAC_EN1 | DAC_EN2;
    scope_dac_set_offsets(ch1_dac, ch2_dac);
}

#if SCOPE_ATTENUATOR_CONFIG
static void scope_set_channel_range(uint8_t channel, uint16_t range_code) {
    uint8_t a = 0;
    uint8_t b = 0;
    uint8_t c = 0;
    uint8_t d = 0;

    switch (range_code) {
    case 0:
        b = 1;
        c = 1;
        break;
    case 2000:
        a = 1;
        break;
    case 1000:
        a = 1;
        d = 1;
        break;
    case 500:
        a = 1;
        c = 1;
        break;
    case 400:
        a = 1;
        c = 1;
        d = 1;
        break;
    case 200:
        a = 1;
        b = 1;
        break;
    case 100:
        a = 1;
        b = 1;
        d = 1;
        break;
    case 20:
        d = 1;
        break;
    case 10:
        c = 1;
        break;
    case 8:
        c = 1;
        d = 1;
        break;
    case 4:
        b = 1;
        break;
    case 2:
        b = 1;
        d = 1;
        break;
    case 40:
    default:
        break;
    }

    if (channel == 1u) {
        gpio_write_pin(GPIOA_BASE, 1u << 6, a);
        gpio_write_pin(GPIOD_BASE, 1u << 2, b);
        gpio_write_pin(GPIOC_BASE, 1u << 12, c);
        gpio_write_pin(GPIOC_BASE, 1u << 11, d);
    } else {
        gpio_write_pin(GPIOA_BASE, 1u << 7, a);
        gpio_write_pin(GPIOD_BASE, 1u << 12, b);
        gpio_write_pin(GPIOD_BASE, 1u << 13, c);
        gpio_write_pin(GPIOB_BASE, 1u << 1, d);
    }
}
#endif

#if SCOPE_ATTENUATOR_CONFIG
static uint16_t scope_range_mv(uint8_t vdiv) {
    static const uint16_t ranges[] = {1000, 400, 200, 100, 40, 20, 10, 4, 2};

    if (vdiv >= (uint8_t)(sizeof(ranges) / sizeof(ranges[0]))) {
        vdiv = 0;
    }
    return ranges[vdiv];
}
#endif

static void scope_analog_begin(void) {
#if SCOPE_ATTENUATOR_CONFIG
    gpio_config_mask(GPIOA_BASE, (1u << 6) | (1u << 7) | (1u << 10), 0x1);
    gpio_config_mask(GPIOB_BASE, 1u << 1, 0x1);
    gpio_config_mask(GPIOC_BASE, (1u << 11) | (1u << 12), 0x1);
    gpio_config_mask(GPIOD_BASE, (1u << 2) | (1u << 12) | (1u << 13), 0x1);
    gpio_config_mask(GPIOE_BASE, 1u << 0, 0x1);
#else
    gpio_config_mask(GPIOA_BASE, 1u << 10, 0x1);
    gpio_config_mask(GPIOE_BASE, 1u << 0, 0x1);
#endif

    if (scope_analog_ready) {
        return;
    }

    gpio_set(GPIOA_BASE, 1u << 10); // CH1 DC coupling
    gpio_set(GPIOE_BASE, 1u << 0);  // CH2 DC coupling
    scope_dac_init(2048u, 2048u);
    scope_analog_ready = 1;
}
#endif

static uint32_t scope_span_for_timebase(uint8_t timebase) {
    static const uint32_t timebase_unit_ns[] = {
        5u, 10u, 20u, 50u, 100u, 200u, 500u,
        1000u, 2000u, 5000u, 10000u, 20000u, 50000u,
        100000u, 200000u, 500000u, 1000000u, 2000000u, 5000000u,
        10000000u, 20000000u, 50000000u, 100000000u, 200000000u,
        500000000u, 1000000000u,
    };
    uint32_t ns;
    uint32_t span;

    if (timebase >= (uint8_t)(sizeof(timebase_unit_ns) / sizeof(timebase_unit_ns[0]))) {
        timebase = 4;
    }
    if (timebase >= SCOPE_HW_SLOW_TIMEBASE_START) {
        return 0x80000u;
    }
    ns = timebase_unit_ns[timebase];
    span = 52428800u / ns;
    if (span > 0x100000u) {
        span = 0x100000u;
    }
    return span;
}

static uint16_t scope_slow_interval_ms_for_timebase(uint8_t timebase) {
    static const uint32_t timebase_unit_ns[] = {
        5u, 10u, 20u, 50u, 100u, 200u, 500u,
        1000u, 2000u, 5000u, 10000u, 20000u, 50000u,
        100000u, 200000u, 500000u, 1000000u, 2000000u, 5000000u,
        10000000u, 20000000u, 50000000u, 100000000u, 200000000u,
        500000000u, 1000000000u,
    };
    uint32_t div_ms;
    uint32_t interval;

    if (timebase >= (uint8_t)(sizeof(timebase_unit_ns) / sizeof(timebase_unit_ns[0]))) {
        timebase = 19u;
    }
    div_ms = (timebase_unit_ns[timebase] + 50000u) / 100000u;
    if (!div_ms) {
        div_ms = 1u;
    }
    interval = (div_ms * 12u + SCOPE_SLOW_POINT_COUNT / 2u) / SCOPE_SLOW_POINT_COUNT;
    if (!interval) {
        interval = 1u;
    }
    if (interval > 60000u) {
        interval = 60000u;
    }
    return (uint16_t)interval;
}

uint8_t scope_hw_enabled(void) {
    return SCOPE_HW_CAPTURE ? 1u : 0u;
}

uint8_t scope_hw_ready(void) {
#if SCOPE_HW_CAPTURE
    return fpga_ready();
#else
    return 1u;
#endif
}

void scope_hw_configure_channels(uint8_t timebase,
                                 uint8_t ch1_vdiv,
                                 uint8_t ch2_vdiv,
                                 uint8_t ch1_dc,
                                 uint8_t ch2_dc,
                                 uint16_t ch1_dac,
                                 uint16_t ch2_dac) {
#if SCOPE_HW_CAPTURE
    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return;
    }
#if SCOPE_ANALOG_CONFIG
    scope_analog_begin();
    if (ch1_dc) {
        gpio_set(GPIOA_BASE, 1u << 10);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 10);
    }
    if (ch2_dc) {
        gpio_set(GPIOE_BASE, 1u << 0);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 0);
    }

#if SCOPE_ANALOG_CONFIG && SCOPE_ATTENUATOR_CONFIG
    uint8_t range_changed = 0;
    if (ch1_vdiv != last_scope_vdiv[0]) {
        scope_set_channel_range(1, scope_range_mv(ch1_vdiv));
        last_scope_vdiv[0] = ch1_vdiv;
        range_changed = 1;
    }
    if (ch2_vdiv != last_scope_vdiv[1]) {
        scope_set_channel_range(2, scope_range_mv(ch2_vdiv));
        last_scope_vdiv[1] = ch2_vdiv;
        range_changed = 1;
    }
    if (range_changed) {
        delay_ms(SCOPE_ANALOG_RANGE_SETTLE_MS);
    }
#else
    (void)ch1_vdiv;
    (void)ch2_vdiv;
#endif
    scope_dac_set_offsets(ch1_dac, ch2_dac);
#else
    (void)ch1_dc;
    (void)ch2_dc;
    (void)ch1_dac;
    (void)ch2_dac;
    (void)ch1_vdiv;
    (void)ch2_vdiv;
#endif
    fpga_write_timing(0, scope_span_for_timebase(timebase));
    delay_ms(SCOPE_TIMING_SETTLE_MS);

    fpga_capture_latch();
#else
    (void)timebase;
    (void)ch1_vdiv;
    (void)ch2_vdiv;
    (void)ch1_dc;
    (void)ch2_dc;
#endif
}

void scope_hw_configure(uint8_t timebase, uint8_t vdiv) {
    scope_hw_configure_channels(timebase, vdiv, vdiv, 1, 1, 2048u, 2048u);
}

void scope_hw_slow_stop(void) {
#if SCOPE_HW_CAPTURE
    scope_slow_irq_enabled = 0;
    TMR_IDEN(TMR1_BASE) &= ~1u;
    TMR_CTRL1(TMR1_BASE) &= ~1u;
    TMR_STS(TMR1_BASE) = ~1u;
#endif
}

void scope_hw_slow_start(uint8_t timebase) {
#if SCOPE_HW_CAPTURE
    uint16_t interval_ms = scope_slow_interval_ms_for_timebase(timebase);
    uint32_t ticks = ((uint32_t)interval_ms * SCOPE_SLOW_TIMER_TICK_HZ) / 1000u;

    if (!ticks) {
        ticks = 1u;
    }
    if (ticks > 0xFFFFu) {
        ticks = 0xFFFFu;
    }

    scope_hw_slow_stop();
    __asm__ volatile("cpsid i" ::: "memory");
    scope_slow_write_index = 0;
    scope_slow_count = 0;
    ++scope_slow_seq;
    __asm__ volatile("cpsie i" ::: "memory");

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return;
    }

    fpga_write_timing(0, scope_span_for_timebase(timebase));
    delay_ms(SCOPE_TIMING_SETTLE_MS);
    fpga_capture_latch();

    RCC_APB2ENR |= 1u << 11; // TMR1
    TMR_CTRL1(TMR1_BASE) = 0;
    TMR_IDEN(TMR1_BASE) = 0;
    TMR_PSC(TMR1_BASE) = 7199u; // 72 MHz / 7200 = 10 kHz
    TMR_PR(TMR1_BASE) = ticks - 1u;
    TMR_EG(TMR1_BASE) = 1u;
    TMR_STS(TMR1_BASE) = ~1u;
    scope_slow_irq_enabled = 1;
    TMR_IDEN(TMR1_BASE) |= 1u;
    REG32(NVIC_ISER0) = 1u << 25; // TMR1 update IRQ
    TMR_CTRL1(TMR1_BASE) = 1u;
#else
    (void)timebase;
#endif
}

uint8_t scope_hw_slow_snapshot(uint8_t *ch1,
                               uint8_t *ch2,
                               uint16_t max_points,
                               uint16_t *count,
                               uint16_t *seq) {
#if SCOPE_HW_CAPTURE
    uint16_t local_count;
    uint16_t local_write;
    uint16_t local_seq;
    uint16_t start;

    if (!ch1 || !ch2 || !count || !seq || !max_points) {
        return 0;
    }

    __asm__ volatile("cpsid i" ::: "memory");
    local_count = scope_slow_count;
    local_write = scope_slow_write_index;
    local_seq = scope_slow_seq;
    if (local_count > max_points) {
        local_count = max_points;
    }
    start = scope_slow_count >= SCOPE_SLOW_POINT_COUNT ? local_write : 0u;
    for (uint16_t i = 0; i < local_count; ++i) {
        uint16_t src = (uint16_t)(start + i);
        if (src >= SCOPE_SLOW_POINT_COUNT) {
            src = (uint16_t)(src - SCOPE_SLOW_POINT_COUNT);
        }
        ch1[i] = scope_slow_samples[(uint16_t)(src * 2u)];
        ch2[i] = scope_slow_samples[(uint16_t)(src * 2u + 1u)];
    }
    __asm__ volatile("cpsie i" ::: "memory");

    *count = local_count;
    *seq = local_seq;
    return 1;
#else
    (void)ch1;
    (void)ch2;
    (void)max_points;
    (void)count;
    (void)seq;
    return 0;
#endif
}

void scope_hw_slow_irq_handler(void) {
#if SCOPE_HW_CAPTURE
    uint8_t sample[2];
    uint16_t dst;

    if (!(TMR_STS(TMR1_BASE) & 1u)) {
        return;
    }
    TMR_STS(TMR1_BASE) = ~1u;
    if (!scope_slow_irq_enabled) {
        return;
    }
    if (!fpga_ready() || !fpga_capture_read_slow_point(sample)) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return;
    }

    dst = scope_slow_write_index;
    scope_slow_samples[(uint16_t)(dst * 2u)] = sample[0];
    scope_slow_samples[(uint16_t)(dst * 2u + 1u)] = sample[1];
    ++dst;
    if (dst >= SCOPE_SLOW_POINT_COUNT) {
        dst = 0;
    }
    scope_slow_write_index = dst;
    if (scope_slow_count < SCOPE_SLOW_POINT_COUNT) {
        ++scope_slow_count;
    }
    ++scope_slow_seq;
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
#endif
}

void TMR1_UP_IRQHandler(void) {
    scope_hw_slow_irq_handler();
}

void scope_hw_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac) {
#if SCOPE_HW_CAPTURE && SCOPE_ANALOG_CONFIG
    scope_analog_begin();
    scope_dac_set_offsets(ch1_dac, ch2_dac);
#else
    (void)ch1_dac;
    (void)ch2_dac;
#endif
}

void scope_hw_arm(void) {
#if SCOPE_HW_CAPTURE
    fpga_init_once();
    if (fpga_ready()) {
        fpga_capture_latch();
    } else {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
    }
#endif
}

uint8_t scope_hw_capture(uint8_t *dst, uint16_t len, uint8_t timebase) {
#if SCOPE_HW_CAPTURE
    uint8_t *read_dst;
    uint16_t read_len;

    if (!dst || len < FPGA_SCOPE_BUFFER_BYTES) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return 0;
    }
    if (!fpga_capture_ready()) {
        scope_last_status = SCOPE_STATUS_WAIT;
        ++scope_wait_count;
        return 0;
    }

    read_dst = dst;
    read_len = FPGA_SCOPE_BUFFER_BYTES;
    if (timebase <= SCOPE_FAST_ALIGN_TIMEBASE_MAX) {
        // Original firmware reads the four fastest ranges into buffer+1.
        // That keeps the FPGA byte stream phase aligned for CH1/CH2 pairs.
        dst[0] = 128u;
        dst[1] = 128u;
        read_dst = dst + 1u;
        read_len = FPGA_SCOPE_BUFFER_BYTES - 1u;
    }

    if (!fpga_capture_read(read_dst, read_len)) {
        fpga_capture_latch();
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    if (timebase <= SCOPE_FAST_ALIGN_TIMEBASE_MAX) {
        dst[0] = dst[2];
        dst[1] = dst[3];
    }
    fpga_capture_latch();
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
    return 1;
#else
    (void)dst;
    (void)len;
    (void)timebase;
    return 0;
#endif
}

uint8_t scope_hw_capture_point(uint8_t sample[2], uint8_t timebase) {
#if SCOPE_HW_CAPTURE
    if (!sample) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    (void)timebase;

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return 0;
    }
    if (!fpga_capture_read_slow_point(sample)) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
    return 1;
#else
    (void)sample;
    (void)timebase;
    return 0;
#endif
}

uint16_t scope_hw_frame_count(void) {
    return scope_frame_count;
}

uint16_t scope_hw_wait_count(void) {
    return scope_wait_count;
}

uint16_t scope_hw_error_count(void) {
    return scope_error_count;
}

uint8_t scope_hw_last_status(void) {
    return scope_last_status;
}
