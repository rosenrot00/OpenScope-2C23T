#pragma once

#include <stdint.h>
#include "fpga_clock.h"

enum {
    SCOPE_HW_SPAN_MAX = 0x100000,
};

static inline uint8_t scope_dense_capture(uint32_t unit_ns) {
    return !HW_TARGET_HW40 && (unit_ns == 500000u || unit_ns == 1000000u || unit_ns == 2000000u);
}

// At 5-20 ms/div the legacy FPGA used to fill ~8.5 screens per capture.
// Fill ~2 instead: a whole display window plus room to search for an edge.
// Do not change the fastest spans, which determine legacy channel alignment.
static inline uint32_t scope_hw_span_for_unit_ns(uint32_t unit_ns) {
    uint32_t span = (scope_dense_capture(unit_ns) ? 209715200u : 52428800u) / unit_ns;

    return span > SCOPE_HW_SPAN_MAX ? SCOPE_HW_SPAN_MAX : (span ? span : 1u);
}

static inline uint32_t scope_hw_sample_period_ns(uint32_t unit_ns) {
    uint32_t span = scope_hw_span_for_unit_ns(unit_ns);

    return (FPGA_ADC_TICK_NS * SCOPE_HW_SPAN_MAX + span / 2u) / span;
}

void scope_hw_configure(uint8_t timebase, uint8_t vdiv);
void scope_hw_configure_channels(uint8_t timebase,
                                 uint8_t ch1_vdiv,
                                 uint8_t ch2_vdiv,
                                 uint8_t ch1_dc,
                                 uint8_t ch2_dc,
                                 uint16_t ch1_dac,
                                 uint16_t ch2_dac);
void scope_hw_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac);
void scope_hw_arm(void);
// A successful full-buffer read leaves the next arm to the acquisition owner.
uint8_t scope_hw_read_capture(uint8_t *dst, uint16_t len, uint8_t timebase);
uint8_t scope_hw_capture_point(uint8_t sample[2], uint8_t timebase);
void scope_hw_slow_start(uint8_t timebase);
void scope_hw_slow_stop(void);
uint8_t scope_hw_slow_snapshot(uint8_t *ch1,
                               uint8_t *ch2,
                               uint16_t max_points,
                               uint16_t *count,
                               uint16_t *seq);
void scope_hw_slow_irq_handler(void);
uint8_t scope_hw_ready(void);
uint8_t scope_hw_enabled(void);
uint16_t scope_hw_frame_count(void);
uint16_t scope_hw_wait_count(void);
uint16_t scope_hw_error_count(void);
uint8_t scope_hw_last_status(void);
