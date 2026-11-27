#pragma once

#include "clock_tree.h"

static inline uint32_t scope_tmr1_hz(uint32_t cfg, uint32_t misc1, uint32_t misc3) {
    uint32_t hz = clock_ahb_hz(cfg, misc1, misc3);
    uint32_t apb2 = (cfg >> 11) & 7u;
    // With an APB divider other than one the timer receives twice PCLK2.
    return apb2 >= 4u ? hz >> (apb2 - 4u) : hz;
}

typedef struct { uint16_t psc, reload; } scope_timer_period_t;

static inline scope_timer_period_t scope_roll_period(uint32_t hz, uint16_t interval_ms) {
    uint32_t div = (hz + 5000u) / 10000u; // Keep the counter near 10 kHz.
    if (!div) div = 1u;
    // Roll intervals are 1..400 ms. Round using the actual divisor, even
    // when the source is not an exact multiple of 10 kHz.
    uint32_t cycles = (hz / 1000u) * interval_ms + ((hz % 1000u) * interval_ms + 500u) / 1000u;
    uint32_t ticks = (cycles + div / 2u) / div;
    return (scope_timer_period_t){(uint16_t)(div - 1u), (uint16_t)(ticks ? ticks - 1u : 0u)};
}
