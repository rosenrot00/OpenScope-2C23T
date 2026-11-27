#include "fpga_clock.h"

#if !HW_TARGET_HW40
uint8_t fpga_adc_tick_ns = 25u;

void fpga_clock_select_legacy(uint32_t cfg) {
    // Legacy CLKOUT is SYSCLK/16. The 192 MHz loader yields 80 MHz DDS
    // and 40 MS/s capture; retain nominal timing for the 240 MHz profile.
    uint32_t code = ((cfg >> 18) & 15u) | (((cfg >> 29) & 3u) << 4);
    uint32_t multiplier = code + (code >= 15u ? 1u : 2u);
    uint32_t pll_in_mhz = (cfg & (1u << 16)) && !(cfg & (1u << 17)) ? 8u : 4u;
    fpga_adc_tick_ns = ((cfg & 12u) == 8u && pll_in_mhz * multiplier == 192u) ? 25u : 20u;
}
#endif
