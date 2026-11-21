#include <assert.h>
#include <stdio.h>
#include <string.h>

#define __asm__
#define volatile(...)
#define dmm_current_mode dmm_wire_mode
#include "../src/dmm.c"
#undef dmm_current_mode
#undef volatile
#undef __asm__
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
}

static void test_no_manual_range_badge(void) {
    low_current_reading("5000", 0);
    ui.auto_range = 0;
    render(UI_RENDER_FULL);
    ui.auto_range = 1;
    render(UI_RENDER_FULL);
    assert(!memcmp(previous_frame, frame_buffer, sizeof(previous_frame)));
}

static void test_dmm_softkeys(void) {
    enum { TEST_X = 9 + 77, CENTER_Y = DMM_KEYS_Y + 13 };
    ui.mode = UI_MODE_DMM;
    ui.overlay = UI_OVERLAY_NONE;
    dmm_valid = 0;
    dmm_hold_active = dmm_rel_active = 0;
    dmm_group_choice[DMM_GROUP_TEST] = 0;
    ui.dmm_mode = DMM_MODE_DIODE;
    render(UI_RENDER_FULL);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + TEST_X + 35] == D_ACCENT);
    assert(frame_buffer[CENTER_Y * LCD_WIDTH + TEST_X + 41] == D_BG); // Sound mark.
    assert(frame_buffer[(CENTER_Y - 6) * LCD_WIDTH + TEST_X + 51] == D_BG); // Outer arc.
    assert(frame_buffer[(CENTER_Y - 6) * LCD_WIDTH + TEST_X + 52] == D_BG);
    assert(frame_buffer[CENTER_Y * LCD_WIDTH + TEST_X + 52] == D_ACCENT); // Gap between arcs.
    assert(frame_buffer[(CENTER_Y - 4) * LCD_WIDTH + TEST_X + 31] == D_BG); // Diode cathode.
    ui.dmm_mode = DMM_MODE_CAP;
    dmm_group_choice[DMM_GROUP_TEST] = 1;
    render(UI_RENDER_LOCAL_CHANGE);
    assert(frame_buffer[(CENTER_Y - 5) * LCD_WIDTH + TEST_X + 33] == D_BG); // Capacitor plate.
    assert(frame_buffer[(CENTER_Y - 5) * LCD_WIDTH + TEST_X + 34] == D_BG);
    assert(frame_buffer[(CENTER_Y - 5) * LCD_WIDTH + TEST_X + 35] == D_ACCENT); // Gap between plates.
    assert(frame_buffer[(CENTER_Y - 6) * LCD_WIDTH + TEST_X + 51] == D_ACCENT);
    ui.dmm_mode = DMM_MODE_AUTO;
    render(UI_RENDER_LOCAL_CHANGE);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + 44] == D_ACCENT);
    assert(frame_buffer[(DMM_KEYS_Y + 2) * LCD_WIDTH + TEST_X + 35] == D_CARD);
    assert(frame_buffer[(CENTER_Y - 5) * LCD_WIDTH + TEST_X + 33] == D_SUB);
    render(UI_RENDER_DMM_READING);
    assert(!memcmp(previous_frame + (DMM_KEYS_Y - 2) * LCD_WIDTH,
                   frame_buffer + (DMM_KEYS_Y - 2) * LCD_WIDTH,
                   (LCD_HEIGHT - DMM_KEYS_Y + 2) * LCD_WIDTH * sizeof(*frame_buffer)));

    ui.dmm_mode = DMM_MODE_ACV;
    render(UI_RENDER_LOCAL_CHANGE);
    uint16_t wave_x = 9u + (71u - 19u + gfx_text_width(&font_medium, "V")) / 2u + 6u;
    assert(frame_buffer[(CENTER_Y + 1) * LCD_WIDTH + wave_x] == D_BG);
    assert(frame_buffer[(CENTER_Y + 2) * LCD_WIDTH + wave_x] == D_BG); // Two-pixel AC stroke.
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
            int16_t expected_y = scope_scaled_sample_y(ch, (uint8_t)raw, center, 64, 29, 216);
            lcd_render(render_trigger_line, 0);
            assert(frame_buffer[(uint32_t)expected_y * LCD_WIDTH + 33u] == scope_channel_color(ch + 1u));
        }
    }
}

int main(void) {
    test_low_current_bar();
    test_bar_ratios();
    test_status_snapshot();
    test_no_manual_range_badge();
    test_dmm_softkeys();
    test_partial_headers();
    test_trigger_labels();
    test_trigger_line_position();
    printf("UI rendering: bar scaling, incremental headers, trigger labels/position and dirty coverage passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
