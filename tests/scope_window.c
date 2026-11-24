// Host regression test: include the UI implementation to exercise its static helpers.
#include "../src/ui.c"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void setup(void) {
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_timebase = 11u;
    ui.scope_trigger_source = 1u;
    ui.scope_trigger_edge = 0;
    ui.scope_trigger_level = 128u;
    ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    scope_h_pos = 0;
    scope_trigger_locked = 0;
    scope_trigger_offset = 0;
    scope_auto_zero_active = 0;
    scope_frame_valid = 1;
    scope_ch_enabled[0] = 1;
    scope_ch_enabled[1] = 1;
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        scope_vdiv_ch[ch] = 5u;
        scope_probe_x10[ch] = 0;
    }
}

static void fill_sine(void) {
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; ++i) {
        double phase = 6.283185307179586 * ((double)i - 50.0) / 250.0;
        uint8_t raw = (uint8_t)lround(128.0 + 90.0 * sin(phase));
        scope_capture_samples[i * 2u] = raw;
        scope_capture_samples[i * 2u + 1u] = (uint8_t)(255u - raw);
        scope_samples[i * 2u] = raw;
        scope_samples[i * 2u + 1u] = (uint8_t)(255u - raw);
    }
}

static void test_sine_seam(void) {
    setup();
    fill_sine();
    uint16_t offset = 0;
    assert(scope_trigger_crossed(scope_capture_samples[100u],
                                 scope_capture_samples[102u]));
    assert(scope_find_trigger_offset(scope_capture_samples, &offset));
    assert(offset == 151u);
    scope_trigger_locked = 1;
    scope_trigger_offset = offset;
    assert(scope_capture_window_start() == offset);

    // Pixel sampling and Bode input must read the same chronological window.
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        float values[FFT_SIZE];
        float mean = 0.0f;
        uint16_t visible = scope_visible_sample_count();
        for (uint16_t i = 0; i < FFT_SIZE; ++i) {
            float position = (float)i * (float)(visible - 1u) / (float)(FFT_SIZE - 1u);
            uint16_t low = (uint16_t)position;
            uint16_t high = low < visible - 1u ? (uint16_t)(low + 1u) : low;
            float a = (float)scope_raw_delta_mv(ch, scope_samples[(offset + low) * 2u + ch]);
            float b = (float)scope_raw_delta_mv(ch, scope_samples[(offset + high) * 2u + ch]);
            values[i] = a + (b - a) * (position - (float)low);
            mean += values[i];
        }
        mean /= (float)FFT_SIZE;
        float actual[FFT_SIZE];
        bode_extract_channel(ch, actual);
        for (uint16_t i = 0; i < FFT_SIZE; ++i) {
            assert(fabsf(actual[i] - (values[i] - mean)) < 0.001f);
        }
        for (uint16_t x = 0; x <= visible - 1u; ++x) {
            uint8_t raw = scope_samples[(offset + x) * 2u + ch];
            int16_t expected = scope_scaled_sample_y(ch, raw, 0, 30, -10000, 10000);
            assert(scope_sample_y(scope_plot_window(), x, ch, 0, 30, (uint16_t)(visible - 1u), -10000, 10000) == expected);
        }
    }
}

static void test_incomplete_trigger_windows(void) {
    setup();
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; ++i) {
        scope_capture_samples[i * 2u] = i == 10u || i >= 2000u ? 200u : 100u;
    }
    uint16_t offset = 123u;
    assert(!scope_find_trigger_offset(scope_capture_samples, &offset));
    assert(offset == 123u);
    assert(scope_trigger_accepts_capture(0));
    assert(scope_capture_window_start() == 874u);
    ui.scope_trigger_mode = SCOPE_TRIGGER_NORMAL;
    assert(!scope_trigger_accepts_capture(0));
    ui.scope_trigger_mode = SCOPE_TRIGGER_SINGLE;
    assert(!scope_trigger_accepts_capture(0));
}

static void test_window_bounds(void) {
    setup();
    fill_sine();
    for (uint8_t timebase = 0; timebase < SCOPE_TIMEBASE_COUNT; ++timebase) {
        ui.scope_timebase = timebase;
        uint16_t visible = scope_visible_sample_count();
        for (int16_t pos = -SCOPE_H_POS_LIMIT; pos <= SCOPE_H_POS_LIMIT; ++pos) {
            scope_h_pos = pos;
            scope_trigger_locked = 0;
            uint16_t start = scope_capture_window_start();
            assert(start + visible <= SCOPE_SAMPLE_COUNT);
            for (uint8_t ch = 0; ch < 2u; ++ch) {
                ui.scope_trigger_source = (uint8_t)(ch + 1u);
                for (uint8_t edge = 0; edge < 2u; ++edge) {
                    ui.scope_trigger_edge = edge;
                    uint16_t offset;
                    if (!scope_find_trigger_offset(scope_capture_samples, &offset)) {
                        continue;
                    }
                    uint16_t hit = (uint16_t)(offset + scope_trigger_screen_sample_pos());
                    assert(offset + visible <= SCOPE_SAMPLE_COUNT);
                    assert(hit > 0 && hit < SCOPE_SAMPLE_COUNT);
                    assert(scope_trigger_crossed(scope_capture_samples[(hit - 1u) * 2u + ch],
                                                 scope_capture_samples[hit * 2u + ch]));
                    scope_trigger_locked = 1;
                    scope_trigger_offset = offset;
                    assert(scope_capture_window_start() == offset);
                    scope_trigger_locked = 0;
                }
            }
        }
        scope_trigger_locked = 1;
        scope_trigger_offset = SCOPE_SAMPLE_COUNT - 1u;
        assert(scope_capture_window_start() == SCOPE_SAMPLE_COUNT - visible);
    }
}

int main(void) {
    setup();
    ui.scope_timebase = 0;
    scope_plot_window_t w = scope_plot_window();
    assert(w.count == 30 && w.zoom);
    scope_samples[2u * w.start] = 40;
    scope_samples[2u * (w.start + 1u)] = 200;
    assert(scope_plot_raw(w, 5, 0, 300, 1) == 120);
    assert(scope_plot_raw(w, 10, 0, 300, 1) == 200);
    assert(scope_display_sample_raw(5, 0, 300) == 40);
    w.start = SCOPE_SAMPLE_COUNT - w.count;
    scope_samples[2u * (SCOPE_SAMPLE_COUNT - 1u)] = 37;
    assert(scope_plot_raw(w, 300, 0, 300, 1) == 37);
    assert(scope_mv_to_pixels(1000, 1000, 190) == 24);
    assert(scope_mv_to_pixels(-1000, 1000, 190) == -24);
    assert(scope_mv_to_pixels(4000, 1000, 190) == 95);
    test_sine_seam();
    test_incomplete_trigger_windows();
    test_window_bounds();
    puts("scope window regression tests passed");
    return 0;
}
