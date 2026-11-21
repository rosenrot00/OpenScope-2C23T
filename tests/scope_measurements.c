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
static uint8_t capture_ready;

void scope_hw_arm(void) { ++arm_count; }
void scope_hw_set_offsets(uint16_t dac0, uint16_t dac1) { (void)dac0; (void)dac1; }
void settings_note(const settings_state_t *settings) { (void)settings; }
uint8_t scope_hw_enabled(void) { return 1; }
uint8_t scope_hw_capture(uint8_t *dst, uint16_t len, uint8_t timebase) {
    (void)timebase;
    assert(len == SCOPE_SAMPLE_BYTES);
    if (!capture_ready) return 0;
    memcpy(dst, scope_samples, len);
    return 1;
}
uint8_t scope_hw_capture_point(uint8_t sample[2], uint8_t timebase) {
    (void)sample; (void)timebase;
    return 0;
}
uint8_t scope_hw_slow_snapshot(uint8_t *ch1, uint8_t *ch2, uint16_t max_points,
                              uint16_t *count, uint16_t *seq) {
    (void)ch1; (void)ch2; (void)max_points; (void)count; (void)seq;
    return 0;
}
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

uint8_t lcd_collecting(void) { return 0; }
void lcd_mark_dirty(uint16_t x, uint16_t y, uint16_t w, uint16_t h) { (void)x; (void)y; (void)w; (void)h; }
void lcd_pixel(int16_t x, int16_t y, uint16_t color) { (void)x; (void)y; (void)color; }
void lcd_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    (void)x; (void)y; (void)w; (void)h; (void)color;
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
    ui.mode = UI_MODE_SCOPE;
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_timebase = 11;
    ui.active_ch = 1;
    ui.scope_trigger_source = 1;
    ui.scope_trigger_edge = 0;
    ui.scope_trigger_level = 128;
    ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    scope_frame_valid = 1;
    scope_frame_id = 0;
    scope_frame_age_ms = 0;
    scope_auto_zero_active = 0;
    scope_auto_cancel();
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
        scope_zero_offset[ch] = 0;
        scope_ch_pos[ch] = 0;
    }
    math_lines = arm_count = configure_count = 0;
    capture_ready = 0;
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
        uint32_t span = 52428800u / scope_timebase_unit_ns[tb];
        if (span > 0x100000u) span = 0x100000u;
        uint32_t expected = scope_slow_roll_active() ?
            (scope_timebase_unit_ns[tb] * 2u + 2u) / 5u :
            (20u * 0x100000u + span / 2u) / span;
        assert(scope_sample_period_ns() == expected);
        uint64_t screen_ns = (uint64_t)scope_timebase_unit_ns[tb] * 120u;
        uint64_t visible = (screen_ns + expected / 2u) / expected;
        if (visible < 2u) visible = 2u;
        if (visible > 300u) visible = 300u;
        assert(scope_visible_sample_count() == visible);
        if (scope_slow_roll_active()) assert(scope_visible_sample_count() == 300u);
    }
    ui.scope_timebase = 0;
    assert(scope_sample_period_ns() == 20u);
    assert(scope_visible_sample_count() == 30u);
    ui.scope_timebase = 4;
    assert(scope_sample_period_ns() == 40u);
    assert(scope_visible_sample_count() == 300u);
    ui.scope_timebase = 5;
    assert(scope_sample_period_ns() == 80u);
    ui.scope_timebase = 17;
    assert(scope_sample_period_ns() == 806597u);
    assert(scope_visible_sample_count() == 298u);
#if HW_TARGET_HW40
    // Truncated spans 10, 5 and 2 stretch the HW4 50MS, 100MS and 200MS captures.
    ui.scope_timebase = 18;
    assert(scope_sample_period_ns() == 2097152u);
    assert(scope_visible_sample_count() == 286u);
    ui.scope_timebase = 19;
    assert(scope_sample_period_ns() == 4194304u);
    assert(scope_visible_sample_count() == 286u);
    ui.scope_timebase = 20;
    assert(scope_sample_period_ns() == 10485760u);
    assert(scope_visible_sample_count() == 229u);
#endif
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
    assert(scope_estimate_freq_hz_window(0, 2, 2046, 96, 160) == (HW_TARGET_HW40 ? 9u : 10u));
#if HW_TARGET_HW40
    // A 1 Hz square wave at 200MS spans five divisions of real time.
    ui.scope_timebase = 20;
    fill_square(95, 48, 96, 160);
    assert(scope_estimate_freq_hz_window(0, 2, 2046, 96, 160) == 1u);
    assert(scope_visible_sample_count() * 5u / 12u == 95u);
#endif
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

static void auto_assert_applied(void) {
    assert(configured_timebase == ui.scope_timebase);
    assert(configured_range[0] == scope_vdiv_ch[0]);
    assert(configured_range[1] == scope_vdiv_ch[1]);
    assert(ui.scope_vdiv == scope_vdiv_ch[ui.active_ch == 2u ? 1u : 0u]);
}

static void auto_fill_channel(uint8_t ch, uint16_t period, uint8_t low, uint8_t high) {
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; ++i) {
        scope_samples[i * 2u + ch] = i % period < period / 2u ? high : low;
    }
}

static void auto_frame(void) {
    ++scope_frame_id;
    scope_frame_valid = 1;
    scope_compute_stats();
    if (!scope_auto_time_service()) {
        scope_auto_channel_service();
    }
    auto_assert_applied();
}

static unsigned auto_finish(void) {
    unsigned frames = 0;
    while ((scope_auto_time_active || scope_auto_channel_mask) && frames < 40u) {
        auto_frame();
        ++frames;
    }
    assert(!scope_auto_time_active && !scope_auto_channel_mask);
    return frames;
}

static void test_auto_noise_and_no_signal(void) {
    setup();
    scope_vdiv_ch[1] = 0;
    scope_auto_setup_request(3);
    assert(scope_vdiv_ch[0] == 1 && scope_vdiv_ch[1] == 1);
    assert(ui.scope_timebase == 0 && !scope_frame_valid);
    auto_fill_channel(0, 2, 125, 131);
    auto_fill_channel(1, 2, 124, 132);
    assert(auto_finish() == SCOPE_AUTO_TIMEBASE_MAX + 1u + SCOPE_AUTO_STABLE_FRAMES);
    assert(ui.scope_timebase == SCOPE_AUTO_TIMEBASE_MAX);
    assert(scope_vdiv_ch[0] == 1 && scope_vdiv_ch[1] == 1);

    setup();
    scope_vdiv_ch[0] = 8;
    scope_auto_setup_request(3);
    auto_fill_channel(0, 2, 128, 128);
    auto_fill_channel(1, 2, 128, 128);
    auto_finish();
    assert(scope_vdiv_ch[0] == 1 && scope_vdiv_ch[1] == 1);
    assert(ui.scope_timebase == SCOPE_AUTO_TIMEBASE_MAX);
}

static void test_auto_range_boundary_and_budget(void) {
    setup();
    scope_vdiv_ch[0] = 0;
    scope_auto_channel_request(1);
    assert(scope_vdiv_ch[0] == 1);
    unsigned frames = 0;
    while (scope_auto_channel_mask && frames < 20u) {
        // The old bidirectional rule oscillated between these two gain steps.
        uint8_t pp = configured_range[0] == 1u ? 71u : 178u;
        auto_fill_channel(0, 50, (uint8_t)(128u - pp / 2u), (uint8_t)(128u + (pp + 1u) / 2u));
        auto_fill_channel(1, 2, 128, 128);
        auto_frame();
        ++frames;
    }
    assert(frames == SCOPE_AUTO_STABLE_FRAMES && !scope_auto_channel_mask);
    assert(scope_vdiv_ch[0] == 1 && configure_count == 1);

    setup();
    scope_auto_channel_request(1);
    scope_auto_channel_steps_left[0] = 1;
    auto_fill_channel(0, 50, 28, 228);
    auto_fill_channel(1, 2, 128, 128);
    auto_frame();
    assert(scope_vdiv_ch[0] == 2 && !scope_auto_channel_steps_left[0]);
    unsigned configs = configure_count;
    auto_finish();
    assert(scope_vdiv_ch[0] == 2 && configure_count == configs);
    auto_assert_applied();

    setup();
    scope_auto_channel_request(1);
    scope_vdiv_ch[0] = 0;
    scope_apply_settings();
    auto_fill_channel(0, 50, 39, 217);
    auto_frame();
    assert(scope_vdiv_ch[0] == 1);
    auto_fill_channel(0, 50, 93, 164);
    auto_finish();
    assert(scope_vdiv_ch[0] == 1);
}

static void test_auto_channel_progress(void) {
    setup();
    scope_auto_channel_request(3);
    auto_fill_channel(0, 2, 125, 131);
    auto_fill_channel(1, 2, 255, 255);
    auto_frame();
    assert(scope_auto_channel_stable_count[0] == 1 && scope_auto_channel_mask == 3);
    scope_auto_channel_service();
    assert(scope_auto_channel_stable_count[0] == 1 && scope_auto_channel_mask == 3);
    auto_frame();
    assert(scope_auto_channel_mask == 2 && scope_vdiv_ch[0] == 1);
    auto_fill_channel(0, 50, 28, 228);
    auto_finish();
    assert(scope_vdiv_ch[0] == 1 && scope_vdiv_ch[1] == 8);

    setup();
    scope_frame_id = UINT16_MAX;
    scope_auto_channel_request(1);
    auto_fill_channel(0, 2, 128, 128);
    auto_fill_channel(1, 2, 128, 128);
    auto_frame();
    assert(scope_frame_id == 0 && scope_auto_channel_stable_count[0] == 1);
    auto_frame();
    assert(!scope_auto_channel_mask);
}

static void test_auto_selected_and_disabled(void) {
    setup();
    ui.active_ch = 2;
    scope_auto_channel_request(2);
    assert(!scope_auto_time_active && ui.scope_timebase == 11);
    assert(scope_vdiv_ch[0] == 5 && scope_vdiv_ch[1] == 1);
    auto_fill_channel(0, 2, 255, 255);
    auto_fill_channel(1, 2, 124, 132);
    assert(auto_finish() == SCOPE_AUTO_STABLE_FRAMES);
    assert(scope_vdiv_ch[0] == 5 && scope_vdiv_ch[1] == 1);

    setup();
    scope_ch_enabled[0] = 0;
    scope_ch_pos[0] = 21;
    scope_auto_setup_request(3);
    assert(scope_auto_time_mask == 2 && ui.scope_trigger_source == 2);
    assert(!scope_ch_enabled[0] && scope_vdiv_ch[0] == 5 && scope_ch_pos[0] == 21);
    auto_fill_channel(0, 2, 255, 255);
    auto_fill_channel(1, 2, 128, 128);
    auto_finish();
    assert(!scope_ch_enabled[0] && scope_vdiv_ch[0] == 5);

    setup();
    scope_ch_enabled[1] = 0;
    scope_auto_channel_request(2);
    assert(!scope_ch_enabled[1] && scope_vdiv_ch[1] == 5);
    assert(!scope_auto_channel_mask && !scope_auto_time_active && !configure_count);
    scope_ch_enabled[0] = 0;
    scope_auto_setup_request(3);
    assert(!scope_auto_time_active && !configure_count);

    setup();
    scope_coupling_dc[1] = 0;
    scope_probe_x10[1] = 1;
    scope_auto_channel_request(3);
    auto_fill_channel(0, 2, 128, 128);
    auto_fill_channel(1, 2, 128, 128);
    auto_frame();
    scope_ch_enabled[1] = 0;
    auto_finish();
    assert(!scope_ch_enabled[1] && !scope_coupling_dc[1] && scope_probe_x10[1]);
    assert(!configured_coupling[1]);
}

static void test_auto_weak_signal_and_source(void) {
    setup();
    scope_vdiv_ch[0] = 8;
    scope_auto_setup_request(1);
    auto_fill_channel(0, 50, 122, 134);
    auto_fill_channel(1, 2, 128, 128);
    auto_frame();
    assert(!scope_auto_time_active && ui.scope_timebase == 1);
    auto_finish();
    assert(scope_vdiv_ch[0] == 1 && scope_vdiv_ch[1] == 5);

    setup();
    scope_auto_setup_request(3);
    auto_fill_channel(0, 2, 128, 128);
    auto_fill_channel(1, 50, 122, 134);
    auto_frame();
    assert(!scope_auto_time_active && ui.scope_timebase == 1 && ui.scope_trigger_source == 2);
    auto_finish();
}

static void test_auto_clipped(void) {
    setup();
    scope_auto_setup_request(3);
    assert(scope_coupling_dc[0] == 1 && scope_coupling_dc[1] == 1);
    auto_fill_channel(0, 2, 255, 255);
    auto_fill_channel(1, 2, 0, 0);
    auto_frame();
    assert(ui.scope_timebase == 0 && scope_auto_time_steps_left == SCOPE_AUTO_TIMEBASE_MAX);
    assert(scope_vdiv_ch[0] == 2 && scope_vdiv_ch[1] == 2);
    assert(scope_auto_time_active && !scope_frame_valid && arm_count == 2);
    assert(configure_count == 2 && configured_timebase == 0);
    assert(configured_range[0] == 2 && configured_range[1] == 2);
    assert(configured_coupling[0] == 1 && configured_coupling[1] == 1);
    auto_fill_channel(0, 50, 100, 150);
    auto_fill_channel(1, 50, 100, 150);
    auto_frame();
    assert(ui.scope_timebase == 1 && !scope_auto_time_active);
    assert(scope_auto_channel_mask == 3);
    auto_finish();

    setup();
    scope_auto_setup_request(1);
    auto_fill_channel(0, 2, 255, 255);
    auto_fill_channel(1, 2, 255, 255);
    assert(auto_finish() <= SCOPE_AUTO_TIMEBASE_MAX + SCOPE_VDIV_COUNT + SCOPE_AUTO_STABLE_FRAMES);
    assert(scope_vdiv_ch[0] == 8 && scope_vdiv_ch[1] == 5);
    assert(ui.scope_timebase == SCOPE_AUTO_TIMEBASE_MAX);
}

static void test_auto_channel_trigger_modes(void) {
    setup();
    ui.scope_trigger_mode = SCOPE_TRIGGER_NORMAL;
    scope_auto_channel_request(1);
    capture_ready = 1;
    auto_fill_channel(0, 2, 128, 128);
    auto_fill_channel(1, 2, 128, 128);
    for (unsigned i = 0; i < SCOPE_AUTO_STABLE_FRAMES; ++i) {
        assert(scope_poll_frame());
        auto_assert_applied();
    }
    assert(!scope_auto_channel_mask && ui.scope_trigger_mode == SCOPE_TRIGGER_NORMAL);
    assert(!scope_poll_frame());

    setup();
    ui.scope_trigger_mode = SCOPE_TRIGGER_SINGLE;
    scope_auto_channel_request(1);
    capture_ready = 1;
    auto_fill_channel(0, 50, 100, 150);
    auto_fill_channel(1, 2, 128, 128);
    for (unsigned i = 0; i < SCOPE_AUTO_STABLE_FRAMES; ++i) {
        assert(scope_poll_frame() && ui.running);
        auto_assert_applied();
    }
    assert(!scope_auto_channel_mask && ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE);
    assert(scope_poll_frame() && !ui.running);
}

static void test_auto_timeout_and_cancel(void) {
    setup();
    scope_auto_setup_request(3);
    uint32_t timeout = scope_auto_frame_timeout_ms();
    unsigned configs = configure_count;
    scope_frame_age_ms = (uint16_t)(timeout - 1u);
    assert(!scope_poll_frame() && scope_auto_time_active);
    scope_frame_age_ms = (uint16_t)timeout;
    assert(!scope_poll_frame() && !scope_auto_time_active && !scope_auto_channel_mask);
    assert(configure_count == configs);
    auto_assert_applied();

    setup();
    ui.scope_timebase = 17;
    scope_auto_channel_request(1);
    uint32_t capture_ms = (uint32_t)(((uint64_t)scope_sample_period_ns() * SCOPE_SAMPLE_COUNT + 999999u) / 1000000u);
    timeout = scope_auto_frame_timeout_ms();
    assert(timeout >= capture_ms * 2u && timeout <= 60000u);
    scope_frame_age_ms = (uint16_t)capture_ms;
    assert(!scope_poll_frame() && scope_auto_channel_mask == 1);
    scope_frame_age_ms = (uint16_t)timeout;
    assert(!scope_poll_frame() && !scope_auto_channel_mask);
    auto_assert_applied();

    for (uint8_t tb = 0; tb < SCOPE_TIMEBASE_COUNT; ++tb) {
        setup();
        ui.scope_timebase = tb;
        scope_auto_channel_request(1);
        uint32_t points = scope_slow_roll_active() ? 1u : SCOPE_SAMPLE_COUNT;
        capture_ms = (uint32_t)(((uint64_t)scope_sample_period_ns() * points + 999999u) / 1000000u);
        timeout = scope_auto_frame_timeout_ms();
        assert(timeout >= capture_ms * 2u && timeout <= 60000u);
        scope_frame_age_ms = (uint16_t)(timeout - 1u);
        scope_auto_timeout_service();
        assert(scope_auto_channel_mask == 1);
        scope_frame_age_ms = (uint16_t)timeout;
        scope_auto_timeout_service();
        assert(!scope_auto_channel_mask);
        auto_assert_applied();
    }

    setup();
    scope_auto_setup_request(3);
    scope_cycle_active_channel_vdiv(1);
    assert(!scope_auto_time_active && !scope_auto_channel_mask);
    auto_assert_applied();
    scope_auto_setup_request(3);
    scope_step_timebase(1);
    assert(!scope_auto_time_active && !scope_auto_channel_mask);
    auto_assert_applied();
}

int main(void) {
    test_hardware_timing();
    test_frequency();
    test_mean();
    test_math();
    test_bode_limits();
    test_auto_noise_and_no_signal();
    test_auto_range_boundary_and_budget();
    test_auto_channel_progress();
    test_auto_selected_and_disabled();
    test_auto_weak_signal_and_source();
    test_auto_clipped();
    test_auto_channel_trigger_modes();
    test_auto_timeout_and_cancel();
    printf("scope measurements regression tests passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
