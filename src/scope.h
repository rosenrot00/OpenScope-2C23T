#pragma once

#include <stdint.h>

enum {
    SCOPE_HW_SPAN_MAX = 0x100000,
};

// The FPGA takes one sample every 20 ns * SCOPE_HW_SPAN_MAX / span.
static inline uint32_t scope_hw_span_for_unit_ns(uint32_t unit_ns) {
    uint32_t span = 52428800u / unit_ns;

    return span > SCOPE_HW_SPAN_MAX ? SCOPE_HW_SPAN_MAX : span;
}

static inline uint32_t scope_hw_sample_period_ns(uint32_t unit_ns) {
    uint32_t span = scope_hw_span_for_unit_ns(unit_ns);

    return (20u * SCOPE_HW_SPAN_MAX + span / 2u) / span;
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
uint8_t scope_hw_capture(uint8_t *dst, uint16_t len, uint8_t timebase);
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
