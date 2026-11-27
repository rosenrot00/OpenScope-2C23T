#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define __asm__
#define volatile(...)
#define dmm_current_mode dmm_wire_mode
#include "../src/dmm.c"
#undef dmm_current_mode
#undef volatile
#undef __asm__
#define SCOPE_UI_SAFE_STUB 0
#include "../src/ui.c"
#include "../tools/host_display.h"
#include "../src/font.c"
#include "../src/fft.c"

static uint8_t test_pct = 87;
static uint8_t test_charging;
static uint8_t test_usb;
static uint16_t test_mv = 3950;
static uint16_t previous_frame[LCD_WIDTH * LCD_HEIGHT];
static uint16_t partial_header[LCD_WIDTH * DMM_STATUS_H];

uint8_t battery_percent(void) { return test_pct; }
uint16_t battery_millivolts(void) { return test_mv; }
uint8_t battery_is_charging(void) { return test_charging; }
uint8_t usb_cdc_is_configured(void) { return test_usb; }
uint8_t scope_hw_enabled(void) { return 1; }
uint8_t scope_hw_ready(void) { return 1; }
uint32_t load_counter_read(void) { return 0; }
uint32_t load_counter_elapsed(uint32_t start, uint32_t end) { return start - end; }
void fw_update_status(fw_update_status_t *s) { memset(s, 0, sizeof(*s)); }
uint8_t arb_file_count(void) { return 0; }
const char *arb_file_name(uint8_t index) { (void)index; return ""; }

// Run the actual collect/render paths, but not the MMIO DMA flush.
static void render(ui_render_job_t job) {
    current_render_job = job;
    full_scene_clear = 0;
    memcpy(previous_frame, frame_buffer, sizeof(previous_frame));
    lcd_render(ui_render_dispatch, 0);

    for (uint16_t y = 0; y < LCD_HEIGHT; ++y) {
        for (uint16_t x = 0; x < LCD_WIDTH; ++x) {
            uint32_t i = (uint32_t)y * LCD_WIDTH + x;
            if (previous_frame[i] == frame_buffer[i]) continue;
            uint8_t covered = 0;
            for (uint8_t r = 0; r < dirty_count; ++r) {
                covered |= x >= dirty_rects[r].x0 && x <= dirty_rects[r].x1 &&
                           y >= dirty_rects[r].y0 && y <= dirty_rects[r].y1;
            }
            assert(covered);
        }
    }
}

static void low_current_reading(const char digits[4], unsigned dots) {
    uint8_t frame[9] = {0x5A, 0xA5};
    uint8_t segments[4];
    for (unsigned i = 0; i < 4; ++i) {
        const char *p = strchr(dmm_segment_chars, digits[i]);
        assert(p);
        segments[i] = dmm_segment_codes[p - dmm_segment_chars];
        if (dots & (1u << i)) segments[i] |= 0x10;
    }
    frame[2] = segments[0] & 0xF0;
    frame[3] = (segments[1] & 0xF0) | (segments[0] & 0x0F);
    frame[4] = (segments[2] & 0xF0) | (segments[1] & 0x0F);
    frame[5] = (segments[3] & 0xF0) | (segments[2] & 0x0F);
    frame[6] = (segments[3] & 0x0F) | 0x10;
    frame[8] = 1;
    ui.mode = UI_MODE_DMM;
    ui.overlay = UI_OVERLAY_NONE;
    ui.auto_range = 1;
    dmm_wire_mode = ui.dmm_mode = DMM_MODE_DC_LO_CURR;
    dmm_hold_active = dmm_rel_active = 0;
    dmm_stats_reset();
    (void)parse_frame(frame);
    assert(dmm_has_reading() && dmm_value_is_numeric());
}

static void test_low_current_bar(void) {
    uint16_t y = DMM_CARD_Y + 115u;
    low_current_reading("5000", 0);
    assert(!strcmp(dmm_display_value(), "5000.0000"));
    render(UI_RENDER_FULL);
    assert(frame_buffer[y * LCD_WIDTH + DMM_IN_X + 100u] == D_ACCENT);
    assert(frame_buffer[y * LCD_WIDTH + DMM_IN_X + 200u] == D_TRACK);

    low_current_reading("5000", 1);
    render(UI_RENDER_FULL);
    assert(frame_buffer[y * LCD_WIDTH + DMM_IN_X + 100u] == D_TRACK);
    assert(frame_buffer[y * LCD_WIDTH + DMM_IN_X + 200u] == D_ACCENT);
}

static void test_bar_ratios(void) {
    uint32_t seed = 12345u;
    for (uint32_t fs = 1u; fs <= 100000000u; fs *= 10u) {
        for (uint16_t width = DMM_IN_W / 2u; width <= DMM_IN_W; width *= 2u) {
            assert(dmm_bar_length(0, fs, width) == 0);
            assert(dmm_bar_length(fs, fs, width) == width);
            assert(dmm_bar_length(fs + 1u, fs, width) == width);
            for (unsigned i = 0; i < 1000; ++i) {
                seed = seed * 1664525u + 1013904223u;
                uint32_t v = seed % fs;
                assert(dmm_bar_length(v, fs, width) == (uint64_t)v * width / fs);
            }
        }
    }
    assert(dmm_bar_length(1, 0, DMM_IN_W) == 0);
}

static void test_status_snapshot(void) {
    test_pct = 75;
    test_charging = test_usb = 0;
    test_mv = 3900;
    battery_snapshot_store();
    assert(!battery_snapshot_changed());
    test_mv = 4000; // The new header does not display battery voltage.
    assert(!battery_snapshot_changed());
    test_usb = 1;
    assert(battery_snapshot_changed());
    assert(!battery_snapshot_changed());
    test_charging = 1;
    assert(battery_snapshot_changed());
    test_pct = 76;
    assert(battery_snapshot_changed());
    gen_output_applied = 1;
    assert(battery_snapshot_changed());
    gen_output_applied = 0;
    assert(battery_snapshot_changed());
}

static void test_fw_update_error_overlay_times_out(void) {
    fw_update_status_t status = { .state = FW_UPDATE_STATE_ERROR, .sequence = 1 };
    assert(ui_fw_update_overlay_tick(&status, 20));
    assert(ui_fw_update_error_ms == FW_UPDATE_ERROR_OVERLAY_MS);
    assert(!ui_fw_update_overlay_tick(&status, FW_UPDATE_ERROR_OVERLAY_MS - 1u));
    assert(ui_fw_update_error_ms == 1);
    assert(ui_fw_update_overlay_tick(&status, 20));
    assert(!ui_fw_update_error_ms && !ui_fw_update_overlay_tick(&status, 20));
    assert(status.state == FW_UPDATE_STATE_ERROR);
}

static void test_no_manual_range_badge(void) {
    low_current_reading("5000", 0);
    ui.auto_range = 0;
    render(UI_RENDER_FULL);
    ui.auto_range = 1;
    render(UI_RENDER_FULL);
    assert(!memcmp(previous_frame, frame_buffer, sizeof(previous_frame)));
}

// Bounding box of the pixels in one key that differ from the key's fill.
typedef struct { int16_t x0, y0, x1, y1; uint16_t fg_pixels; } key_ink_t;

static key_ink_t key_ink(uint16_t key_x, uint16_t fill, uint16_t fg, int16_t from_x, int16_t to_x) {
    key_ink_t k = {32767, 32767, -1, -1, 0};
    for (int16_t y = DMM_KEYS_Y + 1; y < DMM_KEYS_Y + DMM_KEYS_H - 1; ++y) {
        for (int16_t x = (int16_t)(key_x + from_x); x <= (int16_t)(key_x + to_x); ++x) {
            uint16_t c = frame_buffer[y * LCD_WIDTH + x];
            if (c == fill) continue;
            if (x < k.x0) k.x0 = x;
            if (x > k.x1) k.x1 = x;
            if (y < k.y0) k.y0 = y;
            if (y > k.y1) k.y1 = y;
            k.fg_pixels = (uint16_t)(k.fg_pixels + (c == fg));
        }
    }
    return k;
}

static void test_dmm_softkeys(void) {
    enum { KEY_W = 71, TEST_X = 9 + 77, CENTER_Y = DMM_KEYS_Y + 13 };
    key_ink_t k;
    ui.mode = UI_MODE_DMM;
    ui.overlay = UI_OVERLAY_NONE;
    dmm_valid = 0;
    dmm_hold_active = dmm_rel_active = 0;
    dmm_group_choice[DMM_GROUP_TEST] = 0;

    // Diode + continuity, selected: drawn in the text colour on the accent,
    // centred in the key.
    ui.dmm_mode = DMM_MODE_DIODE;
    render(UI_RENDER_FULL);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + TEST_X + 35] == D_ACCENT);
    k = key_ink(TEST_X, D_ACCENT, D_BG, 6, KEY_W - 7);
    assert(k.fg_pixels > 40);
    assert(abs((k.x0 + k.x1) - (2 * TEST_X + KEY_W - 1)) <= 3);
    assert(abs((k.y0 + k.y1) - 2 * CENTER_Y) <= 3);

    // Capacitor: two separate plates on the symbol's middle row.
    ui.dmm_mode = DMM_MODE_CAP;
    dmm_group_choice[DMM_GROUP_TEST] = 1;
    render(UI_RENDER_LOCAL_CHANGE);
    k = key_ink(TEST_X, D_ACCENT, D_BG, 6, KEY_W - 7);
    assert(abs((k.x0 + k.x1) - (2 * TEST_X + KEY_W - 1)) <= 3);
    {
        uint8_t runs = 0, in_run = 0;
        int16_t y = (int16_t)(CENTER_Y - 4);
        for (int16_t x = k.x0; x <= k.x1; ++x) {
            uint8_t ink = frame_buffer[y * LCD_WIDTH + x] == D_BG;
            runs = (uint8_t)(runs + (ink && !in_run));
            in_run = ink;
        }
        assert(runs == 2);
    }

    // Not selected: fill and text colour swap; a reading update leaves the
    // keys untouched.
    ui.dmm_mode = DMM_MODE_AUTO;
    render(UI_RENDER_LOCAL_CHANGE);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + 44] == D_ACCENT);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + TEST_X + 35] == D_CARD);
    assert(key_ink(TEST_X, D_CARD, D_SUB, 6, KEY_W - 7).fg_pixels > 20);
    render(UI_RENDER_DMM_READING);
    assert(!memcmp(previous_frame + (DMM_KEYS_Y - 2) * LCD_WIDTH,
                   frame_buffer + (DMM_KEYS_Y - 2) * LCD_WIDTH,
                   (LCD_HEIGHT - DMM_KEYS_Y + 2) * LCD_WIDTH * sizeof(*frame_buffer)));

    // AC volts: the wave sits beside the V, centred on the capital letter,
    // with a stroke at least two pixels thick.
    ui.dmm_mode = DMM_MODE_ACV;
    render(UI_RENDER_LOCAL_CHANGE);
    {
        enum { V_KEY_X = 9 };
        int16_t v_w = (int16_t)gfx_text_width(&font_medium, "V");
        key_ink_t all = key_ink(V_KEY_X, D_ACCENT, D_BG, 6, KEY_W - 7);
        key_ink_t letter = key_ink(V_KEY_X, D_ACCENT, D_BG, all.x0 - V_KEY_X, all.x0 - V_KEY_X + v_w - 2);
        key_ink_t wave = key_ink(V_KEY_X, D_ACCENT, D_BG, all.x0 - V_KEY_X + v_w + 2, KEY_W - 7);
        uint8_t thick = 0;
        assert(wave.fg_pixels > 10);
        assert(abs((wave.y0 + wave.y1) - (letter.y0 + letter.y1)) <= 2);
        // The label as a whole sits in the middle of the pill.
        assert(abs((letter.y0 + letter.y1) - (2 * DMM_KEYS_Y + DMM_KEYS_H - 1)) <= 1);
        for (int16_t x = wave.x0; x <= wave.x1 && !thick; ++x) {
            for (int16_t y = wave.y0; y < wave.y1; ++y) {
                if (frame_buffer[y * LCD_WIDTH + x] == D_BG && frame_buffer[(y + 1) * LCD_WIDTH + x] == D_BG) {
                    thick = 1;
                    break;
                }
            }
        }
        assert(thick);
    }
}

static void test_partial_headers(void) {
    for (unsigned mode = UI_MODE_DMM; mode <= UI_MODE_GEN; ++mode) {
        for (uint8_t chrome = 0; chrome < 2; ++chrome) {
            ui.mode = (ui_mode_t)mode;
            ui.overlay = UI_OVERLAY_NONE;
            ui.chrome_visible = chrome;
            ui.softkeys_ms = 0;
            test_pct = 87;
            test_charging = test_usb = 0;
            render(UI_RENDER_FULL);
            test_pct = 86;
            test_charging = test_usb = 1;
            render(UI_RENDER_BATTERY);
            memcpy(partial_header, frame_buffer, sizeof(partial_header));
            assert(!memcmp(previous_frame + LCD_WIDTH * DMM_STATUS_H,
                           frame_buffer + LCD_WIDTH * DMM_STATUS_H,
                           sizeof(frame_buffer) - sizeof(partial_header)));
            render(UI_RENDER_FULL);
            assert(!memcmp(partial_header, frame_buffer, sizeof(partial_header)));
        }
    }
}

static void test_trigger_labels(void) {
    for (uint8_t ch = 0; ch < 2; ++ch) {
        ui.scope_trigger_source = ch + 1u;
        for (uint8_t range = 0; range < SCOPE_VDIV_COUNT; ++range) {
            scope_vdiv_ch[ch] = range;
            for (uint8_t probe = 0; probe < 2; ++probe) {
                scope_probe_x10[ch] = probe;
                for (uint16_t level = 12; level <= 243; ++level) {
                    char expected[11];
                    ui.scope_trigger_level = (uint8_t)level;
                    scope_format_signed_mv(expected, scope_raw_delta_mv(ch, (uint8_t)level));
                    assert(!strcmp(scope_trigger_level_label(), expected));
                }
            }
        }
    }
}

static void render_trigger_line(void *ctx) {
    (void)ctx;
    lcd_fill(D_PLOT);
    draw_scope_trigger_line(8, 28, 304, 190, D_PLOT);
}

static void test_trigger_line_position(void) {
    ui.chrome_visible = 0;
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_trigger_mode = SCOPE_TRIGGER_NORMAL;
    for (uint8_t ch = 0; ch < 2; ++ch) {
        ui.scope_trigger_source = ch + 1u;
        scope_ch_enabled[ch] = 1;
        scope_ch_pos[ch] = ch ? 14 : -12;
        scope_probe_x10[ch] = 0;
        scope_vdiv_ch[ch] = 5;
        for (uint16_t raw = 12; raw <= 243; raw += 17) {
            ui.scope_trigger_level = (uint8_t)raw;
            int16_t center = scope_channel_center(scope_channel_base_center_for(ch), ch);
            int16_t expected_y = scope_scaled_sample_y(ch, (uint8_t)raw, center, 190, 29, 216);
            lcd_render(render_trigger_line, 0);
            assert(frame_buffer[(uint32_t)expected_y * LCD_WIDTH + 33u] == scope_channel_color(ch + 1u));
        }
    }
}

static void test_scope_trace_columns(void) {
    uint8_t samples[302];
    memset(samples, 90, sizeof(samples));
    samples[2] = 40; // A narrow pulse between former stride-three vertices.
    samples[5] = samples[6] = 0;
    samples[8] = samples[9] = 255;
    samples[300] = 100;
    for (uint8_t extra = 0; extra <= 1; ++extra) {
        memset(frame_buffer, 0, sizeof(frame_buffer));
        host_display_begin_render();
        scope_draw_columns(samples, sizeof(samples), 9, 29, 216, extra, C_CH1);
        for (unsigned x = 0; x < LCD_WIDTH; ++x) {
            for (unsigned y = 0; y < LCD_HEIGHT; ++y) {
                int lit = 0;
                if (x >= 9 && x < 311 && y >= 29 && y <= 216) {
                    unsigned i = x - 9, a = samples[i];
                    unsigned b = i + 1 < sizeof(samples) ? samples[i + 1] : a;
                    lit = y >= (a < b ? a : b) && y <= (a > b ? a : b) + extra;
                }
                assert(frame_buffer[y * LCD_WIDTH + x] == (lit ? C_CH1 : 0));
            }
        }
    }
}

static void check_roll_pixels(int partial) {
    ui_render_job(UI_RENDER_SCOPE_FRAME);
    if (partial) {
        assert(dirty_count <= 1);
        if (dirty_count) assert(dirty_rects[0].y0 >= PLOT_Y && dirty_rects[0].y1 < PLOT_Y + PLOT_H);
    } else {
        assert(dirty_count == 1 && dirty_rects[0].x0 == 0 && dirty_rects[0].y0 == 0 &&
               dirty_rects[0].x1 == LCD_WIDTH - 1 && dirty_rects[0].y1 == LCD_HEIGHT - 1);
    }
    memcpy(previous_frame, frame_buffer, sizeof(previous_frame));
    lcd_render(ui_render_dispatch, 0);
    for (unsigned i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        if (previous_frame[i] != frame_buffer[i]) {
            fprintf(stderr, "Roll mismatch at %u,%u: %x != %x, count=%u offset=%d damage=%d,%d..%d,%d\n",
                    i % LCD_WIDTH, i / LCD_WIDTH, previous_frame[i], frame_buffer[i], scope_slow_roll_count,
                    scope_roll_display_offset, roll_damage.x0, roll_damage.y0, roll_damage.x1, roll_damage.y1);
            break;
        }
    }
    assert(!memcmp(previous_frame, frame_buffer, sizeof(previous_frame)));
}

static void test_roll_damage(void) {
    current_render_job = UI_RENDER_SCOPE_FRAME;
    ui.mode = UI_MODE_SCOPE; ui.overlay = UI_OVERLAY_NONE;
    ui.chrome_visible = ui.softkeys_ms = 0;
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_timebase = 21;
    scope_ch_enabled[0] = scope_ch_enabled[1] = ui.running = scope_frame_valid = 1;
    scope_hide_traces = scope_math_mode = scope_fft_src = 0;
    ui.scope_cursor_mode = SCOPE_CURSOR_TIME;
    scope_cursor_x[0] = 60; scope_cursor_x[1] = 200;
    scope_measure_visible = 1;
    scope_measure_mask[0] = scope_measure_mask[1] = scope_measure_bit(SCOPE_MEASURE_VPP);
    scope_slow_roll_count = scope_slow_roll_head = scope_slow_roll_seq = 0;
    roll_painted.valid = 0;
    for (unsigned i = 0; i < SCOPE_SLOW_ROLL_MAX_POINTS + 27; ++i) {
        scope_soft_roll_store_point(i % 31 ? 75 + i % 110 : 255, 170 - i % 85);
        if (i % 9 == 0 || i == SCOPE_SLOW_ROLL_MAX_POINTS - 1) {
            scope_min_raw[0] = i % 2 ? 0 : 75;
            check_roll_pixels(i != 0);
        }
    }
    check_roll_pixels(1);
    check_roll_pixels(1); // Same displayed history and measurements, no LCD window.
    assert(roll_damage.x1 < roll_damage.x0);
    for (int offset = -57; offset <= 57; offset += 114) {
        scope_roll_display_offset = offset;
        check_roll_pixels(1);
        scope_soft_roll_store_point(16, 225);
        check_roll_pixels(1);
    }
    ui.running = 0;
    check_roll_pixels(0); // HOLD replaces ROLL in the footer.
    ui.running = 1;
    check_roll_pixels(0);
    scope_soft_roll_store_point(130, 125);
    ui_render_job(UI_RENDER_BATTERY); // Header-only updates preserve the last painted plot.
    check_roll_pixels(1);
    ui_render_job(UI_RENDER_LOCAL_CHANGE);
    check_roll_pixels(0);
    // A flat input still extends to the new point; it does not repaint the grid.
    ui.scope_timebase = SCOPE_TIMEBASE_COUNT - 1;
    scope_roll_display_offset = 0;
    scope_slow_roll_count = scope_slow_roll_head = 0;
    scope_soft_roll_store_point(128, 128);
    check_roll_pixels(0);
    scope_soft_roll_store_point(128, 128);
    check_roll_pixels(1);
    assert(roll_damage.x0 == roll_x(0) && roll_damage.x1 == roll_x(1));
}

int main(void) {
    test_low_current_bar();
    test_bar_ratios();
    test_status_snapshot();
    test_fw_update_error_overlay_times_out();
    test_no_manual_range_badge();
    test_dmm_softkeys();
    test_partial_headers();
    test_trigger_labels();
    test_trigger_line_position();
    test_scope_trace_columns();
    test_roll_damage();
    printf("UI rendering: bars, headers, trigger, roll pixel equivalence and dirty coverage passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
