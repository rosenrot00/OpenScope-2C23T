#pragma once

#include <stdint.h>

// AT32F403A CRM: both boards have an 8 MHz crystal. Decode the active
// system clock (switch status, not the requested source) and the AHB divider.
static inline uint32_t clock_ahb_hz(uint32_t cfg, uint32_t misc1, uint32_t misc3) {
    static const uint8_t ahb_shift[8] = {1, 2, 3, 4, 6, 7, 8, 9};
    uint32_t source = (cfg >> 2) & 3u;
    uint32_t hz = 8000000u;
    if (source == 2u) {
        uint32_t code = ((cfg >> 18) & 15u) | (((cfg >> 29) & 3u) << 4);
        uint32_t multiplier = code + (code >= 15u ? 1u : 2u);
        uint32_t divisor = 2u; // HICK/2 feeds the PLL independently of direct HICK mode.
        if (cfg & (1u << 16))
            divisor = (cfg & (1u << 17)) ? 2u + ((misc3 >> 12) & 3u) : 1u;
        hz = hz * multiplier / divisor;
    } else if (source == 0u && (misc1 & (1u << 25)) && (misc3 & (1u << 9))) {
        hz = 48000000u;
    }
    uint32_t ahb = (cfg >> 4) & 15u;
    return ahb >= 8u ? hz >> ahb_shift[ahb - 8u] : hz;
}

static inline uint32_t clock_apb1_hz(uint32_t cfg, uint32_t misc1, uint32_t misc3) {
    uint32_t hz = clock_ahb_hz(cfg, misc1, misc3);
    uint32_t apb1 = (cfg >> 8) & 7u;
    return apb1 >= 4u ? hz >> (apb1 - 3u) : hz;
}
