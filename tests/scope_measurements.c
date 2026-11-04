#define SCOPE_UI_SAFE_STUB 0
#include "../src/ui.c"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned math_lines;
static int16_t math_first_y;
static int16_t math_last_y;
static unsigned arm_count;
static unsigned configure_count;
static uint8_t configured_timebase;
static uint8_t configured_range[2];
static uint8_t configured_coupling[2];

void scope_hw_arm(void) { ++arm_count; }
void scope_hw_slow_start(uint8_t timebase) { (void)timebase; }
void scope_hw_slow_stop(void) {}
void scope_hw_configure_channels(uint8_t timebase, uint8_t range0, uint8_t range1,
                                 uint8_t dc0, uint8_t dc1, uint16_t dac0, uint16_t dac1) {
    (void)dac0; (void)dac1;
    ++configure_count;
    configured_timebase = timebase;
    configured_range[0] = range0;
    configured_range[1] = range1;
    configured_coupling[0] = dc0;
    configured_coupling[1] = dc1;
}

void compute_fft_bin(const float *input_real, uint16_t count, uint8_t window_type, uint16_t bin,
                     float *out_magnitude, float *out_phase_rad) {
    (void)input_real; (void)window_type; (void)bin;
    assert(count >= 2 && count <= FFT_SIZE);
    *out_magnitude = 1.0f;
    *out_phase_rad = 0.0f;
}

void lcd_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    (void)x0; (void)x1;
    if (color == RGB565(255, 0, 255)) {
        if (!math_lines) math_first_y = y0;
        math_last_y = y1;
        ++math_lines;
        assert(y0 >= 26 && y0 <= 173 && y1 >= 26 && y1 <= 173);
    }
}

void lcd_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale) {
    (void)x; (void)y; (void)text; (void)fg; (void)bg; (void)scale;
}

static void setup(void) {
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_timebase = 11;
    ui.active_ch = 1;
    ui.scope_trigger_source = 1;
    ui.scope_trigger_edge = 0;
    scope_frame_valid = 1;
    scope_trigger_locked = 0;
    scope_h_pos = 0;
    scope_measure_hist_ready = 0;
    scope_measure_hist_pos = 0;
    scope_slow_roll_count = scope_slow_roll_head = 0;
    scope_roll_display_offset = 0;
    for (uint8_t ch = 0; ch < 2; ++ch) {
        scope_ch_enabled[ch] = 1;
        scope_probe_x10[ch] = 0;
        scope_vdiv_ch[ch] = 5;
        scope_coupling_dc[ch] = 1;
    }
    math_lines = arm_count = configure_count = 0;
}

static void fill_square(uint16_t period, uint16_t high_samples, uint8_t low, uint8_t high) {
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; ++i) {
        scope_samples[i * 2u] = i % period < high_samples ? high : low;
        scope_samples[i * 2u + 1u] = 128;
    }
}

static void test_hardware_timing(void) {
    setup();
    for (uint8_t tb = 0; tb < SCOPE_TIMEBASE_COUNT; ++tb) {
        ui.scope_timebase = tb;
        assert(scope_slow_roll_active() == (tb >= (HW_TARGET_HW40 ? 21 : 18)));
        assert(scope_soft_roll_active() == (!HW_TARGET_HW40 && tb == 18));
        assert(scope_irq_roll_active() == (tb >= (HW_TARGET_HW40 ? 21 : 19)));
        uint32_t expected = tb <= (HW_TARGET_HW40 ? 5 : 3) ? 20u :
            (scope_timebase_unit_ns[tb] * 2u + 2u) / 5u;
        assert(scope_sample_period_ns() == expected);
    }
}

static void test_frequency(void) {
    setup();
    ui.scope_timebase = 3;
    fill_square(2, 1, 96, 160);
    assert(scope_estimate_freq_hz_window(0, 0, SCOPE_SAMPLE_COUNT, 96, 160) == 25000000u);
    fill_square(5, 2, 96, 160);
    assert(scope_estimate_freq_hz_window(0, 2, 2046, 96, 160) == 10000000u);
    assert(scope_estimate_freq_hz_window(0, 0, 2, 96, 160) == 0);
    for (uint8_t tb = 4; tb < SCOPE_TIMEBASE_COUNT; ++tb) {
        ui.scope_timebase = tb;
        fill_square(25, 12, 96, 160);
        uint32_t expected = (uint32_t)(1000000000ull / (25ull * scope_sample_period_ns()));
        assert(scope_estimate_freq_hz_window(0, 2, 2046, 96, 160) == expected);
    }
    ui.scope_timebase = 19;
    assert(scope_estimate_freq_hz_window(0, 2, 2046, 96, 160) == 10u);
}

static void test_mean(void) {
    setup();
    fill_square(256, 64, 128, 192);
    scope_compute_stats();
    assert(scope_filtered_mean_mv(0) == 602);
    char value[12];
    scope_measure_value_for(value, 0, SCOPE_MEASURE_VAVG);
    assert(strcmp(value, "0.602V") == 0);
    for (uint8_t ch = 0; ch < 2; ++ch) {
        for (uint8_t range = 0; range < SCOPE_VDIV_COUNT; ++range) {
            scope_vdiv_ch[ch] = range;
            for (uint8_t probe = 0; probe < 2; ++probe) {
                scope_probe_x10[ch] = probe;
                for (uint16_t count = 1; count <= SCOPE_SAMPLE_COUNT; count *= 2) {
                    for (uint16_t raw = 0; raw <= 255; raw += 17) {
                        assert(scope_sample_mean_mv(ch, (uint32_t)raw * count, count) ==
                               scope_raw_delta_mv(ch, (uint8_t)raw));
                    }
                }
            }
        }
    }
    setup();
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        scope_measure_history_store(128, 160, 0, 128, 128, 0, 0, 0,
                                    (int32_t)i * 100, -(int32_t)i * 100);
    }
    assert(scope_filtered_mean_mv(0) == 450);
    assert(scope_filtered_mean_mv(1) == -450);
    ui.scope_timebase = 17;
    assert(scope_filtered_mean_mv(0) == 900);
    assert(scope_filtered_mean_mv(1) == -900);
    scope_measure_hist_ready = 0;
    scope_slow_roll_count = 100;
    for (uint16_t i = 0; i < 100; ++i) {
        scope_slow_roll_raw[0][i] = i < 25 ? 192 : 128;
        scope_slow_roll_raw[1][i] = i < 25 ? 64 : 128;
    }
    scope_slow_roll_update_stats();
    assert(scope_filtered_mean_mv(0) == 603);
    assert(scope_filtered_mean_mv(1) == -603);
    scope_ch_enabled[1] = 0;
    scope_slow_roll_update_stats();
    assert(scope_filtered_mean_mv(1) == 0);
}

static void test_math(void) {
    setup();
    scope_math_mode = 1;
    scope_math_op = 1;
    scope_vdiv_ch[1] = 6;
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; ++i) {
        scope_samples[i * 2u] = 160;
        scope_samples[i * 2u + 1u] = 144;
    }
    assert(scope_raw_delta_mv(0, 160) == scope_raw_delta_mv(1, 144));
    // Deliberately wrong/clipped cached pixels must not affect the math result.
    scope_trace_cache[0].valid = scope_trace_cache[1].valid = 0;
    ui_draw_math_waveform(10, 25, 300, 150);
    assert(math_lines && math_first_y == 100 && math_last_y == 100);
    math_lines = 0;
    scope_math_op = 0;
    ui_draw_math_waveform(10, 25, 300, 150);
    assert(math_lines && math_first_y == 49 && math_last_y == 49);
    math_lines = 0;
    scope_math_op = 2;
    scope_probe_x10[1] = 1;
    ui_draw_math_waveform(10, 25, 300, 150);
    assert(math_lines && math_first_y == 89 && math_last_y == 89);
    math_lines = 0;
    scope_ch_enabled[1] = 0;
    ui_draw_math_waveform(10, 25, 300, 150);
    assert(math_lines == 0);
    scope_ch_enabled[1] = 1;
    scope_probe_x10[1] = 0;
    scope_math_op = 1;
    ui.scope_timebase = 21;
    scope_slow_roll_count = 20;
    for (uint16_t i = 0; i < 20; ++i) {
        scope_slow_roll_raw[0][i] = 160;
        scope_slow_roll_raw[1][i] = 144;
    }
    ui_draw_math_waveform(10, 25, 300, 150);
    assert(math_lines == 19 && math_first_y == 100 && math_last_y == 100);
}

static void test_bode_limits(void) {
    setup();
    scope_fft_src = 4;
    for (uint32_t value = 0; value <= 9999; ++value) {
        uint8_t clamped = bode_clamp_steps(value);
        assert(clamped >= SETTINGS_BODE_MIN_STEPS && clamped <= BODE_MAX_STEPS);
        if (value >= SETTINGS_BODE_MIN_STEPS && value <= BODE_MAX_STEPS) assert(clamped == value);
    }
    for (uint16_t value = 0; value <= 255; ++value) {
        bode_steps = (uint8_t)value;
        bode_is_sweeping = 0;
        ui_draw_bode(10, 25, 300, 150);
        bode_is_sweeping = 1;
        bode_current_step = 254;
        ui_draw_bode(10, 25, 300, 150);
    }
    bode_is_sweeping = 0;
    // Bode steps must never select a roll timebase, which leaves scope_samples stale.
    for (uint32_t freq = 10; freq <= SETTINGS_SIGGEN_MAX_FREQ_HZ; freq += freq < 1000 ? 1u : 997u) {
        assert(bode_auto_timebase(freq) < SCOPE_SLOW_ROLL_TIMEBASE_START);
    }
    assert(bode_auto_timebase(10) == (HW_TARGET_HW40 ? 18 : 17) && bode_auto_timebase(13) == 17);
    bode_step_freq_hz = 10;
    ui.scope_timebase = 17;
    assert(bode_measure_step(0));
    ui.scope_timebase = SCOPE_SLOW_ROLL_TIMEBASE_START;
    assert(!bode_measure_step(0));
}

static void test_auto_clipped(void) {
    setup();
    scope_auto_setup_request(3);
    assert(scope_coupling_dc[0] == 1 && scope_coupling_dc[1] == 1);
    scope_frame_valid = 1;
    scope_min_raw[0] = scope_max_raw[0] = 255;
    scope_min_raw[1] = scope_max_raw[1] = 0;
    assert(scope_auto_time_service());
    assert(ui.scope_timebase == 0 && scope_auto_time_steps_left == SCOPE_AUTO_TIMEBASE_MAX);
    assert(scope_vdiv_ch[0] == 6 && scope_vdiv_ch[1] == 6);
    assert(scope_auto_time_active && !scope_frame_valid && arm_count == 2);
    assert(configure_count == 2 && configured_timebase == 0);
    assert(configured_range[0] == 6 && configured_range[1] == 6);
    assert(configured_coupling[0] == 1 && configured_coupling[1] == 1);
    for (uint8_t ch = 0; ch < 2; ++ch) {
        scope_min_raw[ch] = 100;
        scope_max_raw[ch] = 150;
        scope_avg_raw[ch] = 125;
    }
    fill_square(50, 25, 100, 150);
    scope_frame_valid = 1;
    assert(scope_auto_time_service());
    assert(ui.scope_timebase == 1 && !scope_auto_time_active);
    assert(scope_auto_channel_mask == 3);
    setup();
    scope_auto_setup_request(1);
    scope_frame_valid = 1;
    scope_min_raw[0] = scope_max_raw[0] = 255;
    scope_min_raw[1] = scope_max_raw[1] = 255;
    assert(scope_auto_time_service());
    assert(scope_vdiv_ch[0] == 6 && scope_vdiv_ch[1] == 5);
    // Unrecoverable hardware overrange still exits the bounded search.
    for (unsigned i = 0; i < 30 && scope_auto_time_active; ++i) {
        scope_frame_valid = 1;
        scope_min_raw[0] = scope_max_raw[0] = scope_avg_raw[0] = 255;
        scope_auto_time_service();
    }
    assert(!scope_auto_time_active);
    assert(ui.scope_timebase == SCOPE_AUTO_TIMEBASE_MAX);
}

int main(void) {
    test_hardware_timing();
    test_frequency();
    test_mean();
    test_math();
    test_bode_limits();
    test_auto_clipped();
    printf("scope measurements regression tests passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
