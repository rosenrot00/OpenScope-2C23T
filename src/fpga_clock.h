#pragma once

#include <stdint.h>

#if HW_TARGET_HW40
#define FPGA_ADC_TICK_NS 20u
#else
extern uint8_t fpga_adc_tick_ns;
#define FPGA_ADC_TICK_NS fpga_adc_tick_ns
void fpga_clock_select_legacy(uint32_t cfgr);
#endif

// The sampling clock is half the DDS clock. Fold the reciprocal at compile
// time; a frequency change needs a single wide multiply, not a 64-bit divide.
static inline uint32_t fpga_phase_increment(uint32_t hz) {
    uint32_t scale = FPGA_ADC_TICK_NS == 25u ? 900719925u : 720575940u;
    return (uint32_t)(((uint64_t)hz * scale + (1u << 23)) >> 24);
}
