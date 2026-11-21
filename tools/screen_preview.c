// Host preview of every screen: runs the real UI drawing code into a
// framebuffer and writes one PPM per screen state.
//
//   make preview        -> build/preview/*.ppm (exact 320x240 framebuffers)
//   make screenshots    -> docs/screenshots/*.png (2x, pixel-exact)

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
#include "../src/font.c"
#include "host_display.h"
#include "../src/gfx.c"
#include "../src/ui_fonts.c"
#include "../src/fft.c"

static uint8_t batt_pct = 87;
uint8_t battery_percent(void) { return batt_pct; }
uint16_t battery_millivolts(void) { return 3950; }
static uint8_t charging;
uint8_t battery_is_charging(void) { return charging; }
uint8_t usb_cdc_is_configured(void) { return charging; }

uint8_t scope_hw_enabled(void) { return 1; }
uint8_t scope_hw_ready(void) { return 1; }
uint32_t load_counter_read(void) { return 0; }
uint32_t load_counter_elapsed(uint32_t start, uint32_t end) { return end - start; }
void fw_update_status(fw_update_status_t *status) { memset(status, 0, sizeof(*status)); }
uint8_t arb_file_count(void) { return 0; }
const char *arb_file_name(uint8_t index) { (void)index; return ""; }

static void save(const char *name) {
    char path[128];
    snprintf(path, sizeof(path), "build/preview/%s.ppm", name);
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return; }
    fprintf(f, "P6\n%u %u\n255\n", LCD_WIDTH, LCD_HEIGHT);
    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint16_t c = frame_buffer[i];
        uint8_t rgb[3] = {(uint8_t)(((c >> 11) & 31) * 255 / 31), (uint8_t)(((c >> 5) & 63) * 255 / 63),
                          (uint8_t)((c & 31) * 255 / 31)};
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

static void feed(uint8_t mode, const char digits[4], unsigned dots, uint8_t prefix, uint8_t fb_flags, uint8_t fc_flags) {
    uint8_t segments[4], frame[9];
    for (unsigned i = 0; i < 4; ++i) {
        const char *p = strchr(dmm_segment_chars, digits[i]);
        segments[i] = dmm_segment_codes[p - dmm_segment_chars];
        if (dots & (1u << i)) segments[i] |= 0x10;
    }
    frame[0] = 0x5A; frame[1] = 0xA5;
    frame[2] = segments[0] & 0xF0;
    frame[3] = (segments[1] & 0xF0) | (segments[0] & 0x0F);
    frame[4] = (segments[2] & 0xF0) | (segments[1] & 0x0F);
    frame[5] = (segments[3] & 0xF0) | (segments[2] & 0x0F);
    frame[6] = (segments[3] & 0x0F) | prefix;
    frame[7] = fb_flags; frame[8] = fc_flags;
    dmm_wire_mode = mode;
    parse_frame(frame);
    dmm_stats_update();
}

static void enter(uint8_t mode) {
    dmm_wire_mode = ui.dmm_mode = mode;
    ui.mode = UI_MODE_DMM;
    ui.overlay = UI_OVERLAY_NONE;
    ui.auto_range = 1;
    dmm_valid = dmm_synthetic_valid = dmm_numeric_valid = 0;
    dmm_hold_active = dmm_rel_active = 0;
    dmm_live_wire_alert = 0;
    dmm_stats_reset();
}

// Feed a synthetic capture through the firmware's own frame processing,
// so traces, statistics and measurements are what the device would show.
static void scope_fill(void) {
    for (uint32_t i = 0; i < SCOPE_SAMPLE_BYTES / 2u; ++i) {
        double t = (double)i / 100.0;
        scope_capture_samples[i * 2u] = (uint8_t)(128.0 + 60.0 * __builtin_sin(t * 6.2831853));
        scope_capture_samples[i * 2u + 1u] = (uint8_t)(((i / 70u) & 1u) ? 150 : 106);
    }
    scope_copy_capture_samples();
    scope_frame_valid = 1;
    ++scope_frame_id;
    for (uint8_t k = 0; k < 8u; ++k) {
        scope_compute_stats();
    }
    scope_trace_invalidate();
}

static void scope_shot(const char *name) {
    ui.mode = UI_MODE_SCOPE;
    ui.overlay = UI_OVERLAY_NONE;
    ui.chrome_visible = 0;
    scope_fill();
    draw_scope_immersive();
    if (ui.softkeys_ms || scope_any_menu_open()) {
        draw_softkeys();
    }
    save(name);
    ui.mode = UI_MODE_DMM;
}

static void shot(const char *name) {
    draw_dmm();
    save(name);
}

int main(void) {
    host_display_begin_render();
    // ---- Multimeter: every state the reading card can be in ----
    // Frames go through the real decoder; flags as in tests/dmm.c:
    // dots bit 0 = minus sign, bit n = decimal point before digit n.

    enter(DMM_MODE_DCV);
    shot("dmm-01-waiting");

    enter(DMM_MODE_DCV);
    feed(1, "1230", 4, 0, 0, 2); feed(1, "1238", 4, 0, 0, 2); feed(1, "1235", 4, 0, 0, 2);
    shot("dmm-02-dc-voltage");

    enter(DMM_MODE_DCV);
    feed(1, "1230", 5, 0, 0, 2); feed(1, "1238", 5, 0, 0, 2); feed(1, "1235", 5, 0, 0, 2);
    shot("dmm-03-dc-voltage-negative");

    enter(DMM_MODE_DCV);
    feed(1, "0003", 8, 0, 0, 2);
    shot("dmm-04-dc-voltage-near-zero");

    enter(DMM_MODE_DCV);
    ui.auto_range = 0;
    feed(1, "0503", 4, 0, 0, 2);
    shot("dmm-05-manual-range");

    enter(DMM_MODE_DCV);
    feed(1, "4520", 4, 0, 0, 2);
    shot("dmm-06-dc-caution");

    enter(DMM_MODE_ACV);
    feed(2, "2298", 8, 0, 0, 2); feed(2, "2331", 8, 0, 0, 2); feed(2, "2317", 8, 0, 0, 2);
    shot("dmm-07-ac-high-voltage");

    enter(DMM_MODE_DCV);
    feed(1, "1235", 4, 0, 0, 2);
    dmm_toggle_hold();
    shot("dmm-08-hold");

    enter(DMM_MODE_AUTO);
    feed(0, "AUTO", 0, 0, 0, 2);
    shot("dmm-09-auto-detecting");

    enter(DMM_MODE_AUTO);
    feed(0, "0503", 4, 0, 0, 2); feed(0, "0507", 4, 0, 0, 2);
    shot("dmm-10-auto-voltage");

    enter(DMM_MODE_AUTO);
    feed(0, "1204", 2, 0x40, 0x20, 0);
    shot("dmm-11-auto-resistance");

    enter(DMM_MODE_RES);
    feed(3, "1201", 2, 0x40, 0x20, 0); feed(3, "1210", 2, 0x40, 0x20, 0); feed(3, "1204", 2, 0x40, 0x20, 0);
    shot("dmm-12-resistance");

    enter(DMM_MODE_RES);
    feed(3, "00L0", 4, 0, 0x24, 0);
    shot("dmm-13-resistance-open");

    enter(DMM_MODE_RES);
    feed(3, "0998", 2, 0x40, 0x20, 0);
    dmm_toggle_relative();
    feed(3, "1201", 2, 0x40, 0x20, 0); feed(3, "1210", 2, 0x40, 0x20, 0); feed(3, "1204", 2, 0x40, 0x20, 0);
    shot("dmm-14-rel-positive");

    enter(DMM_MODE_RES);
    feed(3, "1204", 2, 0x40, 0x20, 0);
    dmm_toggle_relative();
    feed(3, "0998", 2, 0x40, 0x20, 0);
    shot("dmm-15-rel-negative");

    enter(DMM_MODE_RES);
    feed(3, "1204", 2, 0x40, 0x20, 0);
    dmm_toggle_relative();
    feed(3, "00L0", 4, 0, 0x24, 0);
    shot("dmm-16-rel-no-reading");

    enter(DMM_MODE_DIODE);
    feed(4, "0004", 8, 0, 0x20, 0);
    shot("dmm-17-continuity-closed");

    enter(DMM_MODE_DIODE);
    feed(4, "00L0", 4, 0, 0, 2);
    shot("dmm-18-continuity-open");

    enter(DMM_MODE_DIODE);
    feed(4, "0624", 2, 0, 0, 2);
    shot("dmm-19-diode");

    enter(DMM_MODE_CAP);
    feed(5, "0470", 8, 0x20, 0x10, 0);
    shot("dmm-20-capacitance");

    enter(DMM_MODE_TEMP);
    feed(8, "0238", 8, 0, 0, 0x20);
    shot("dmm-21-temperature-c");

    enter(DMM_MODE_TEMP);
    feed(8, "0748", 8, 0, 0, 0x10);
    shot("dmm-22-temperature-f");

    enter(DMM_MODE_AC_HI_CURR);
    feed(9, "1234", 2, 0, 0, 1);
    shot("dmm-23-ac-current-high");

    enter(DMM_MODE_DC_LO_CURR);
    feed(12, "1234", 2, 0, 1, 1);
    shot("dmm-24-dc-current-low");

    enter(DMM_MODE_LIVE);
    shot("dmm-25-live-idle");

    enter(DMM_MODE_LIVE);
    ui_set_live_wire_detected(1);
    shot("dmm-26-live-detected");

    enter(DMM_MODE_DCV);
    feed(1, "1235", 4, 0, 0, 2);
    batt_pct = 9; charging = 1;
    shot("dmm-27-low-battery-charging-usb");
    batt_pct = 87; charging = 0;

    ui.overlay = UI_OVERLAY_MODE_MENU;
    for (uint8_t i = 0; i < 4u; ++i) {
        char name[32];
        ui.menu_index = i;
        draw_mode_menu();
        snprintf(name, sizeof(name), "menu-%u", (unsigned)i + 1u);
        save(name);
    }
    ui.scope_timebase = 8;
    ui.running = 1;
    scope_shot("scope-1");
    ui.running = 0;
    ui.active_ch = 2;
    ui.scope_trigger_edge = 1;
    scope_shot("scope-2");
    ui.running = 1;
    ui.active_ch = 1;
    ui.scope_trigger_edge = 0;
    ui.softkeys_ms = 1000;
    ui.scope_param = SCOPE_PARAM_TIME;
    scope_shot("scope-row-main");
    ui.softkeys_ms = 0;
    ui.scope_channel_menu = 1; ui.scope_param = scope_channel_param_order[1];
    scope_shot("scope-row-channel");
    ui.scope_channel_menu = 0;
    ui.scope_trigger_menu = 1; ui.scope_param = scope_trigger_param_order[1];
    scope_shot("scope-row-trigger");
    ui.scope_trigger_menu = 0;
    ui.scope_measure_menu = 1; ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VALUE;
    scope_shot("scope-row-measure");
    ui.scope_measure_menu = 0;

    // Analytical / Math menu, FFT view, Bode menu and plot.
    ui_math_menu_visible = 1; scope_math_selected = 0; scope_fft_src = 1;
    scope_shot("scope-math-menu");
    ui_math_menu_visible = 0;
    scope_shot("scope-fft");
    scope_fft_src = 4; bode_is_sweeping = 0;
    for (uint8_t i = 0; i < BODE_MAX_STEPS; ++i) {
        float t = (float)i / (float)(BODE_MAX_STEPS - 1);
        bode_gain_db[i] = t < 0.5f ? 0.0f : -80.0f * (t - 0.5f);
        bode_phase_deg[i] = -170.0f * t;
    }
    scope_shot("scope-bode");
    ui_math_menu_visible = 1; scope_math_selected = 1; ui_edit_digit_active = 1; ui_edit_digit_index = 2;
    scope_shot("scope-bode-menu");
    ui_math_menu_visible = 0; ui_edit_digit_active = 0; scope_fft_src = 0;

    ui.mode = UI_MODE_GEN;
    ui.gen_wave = SIGGEN_WAVE_SINE; ui.running = 1;
    ui.gen_param = GEN_PARAM_FREQ; gen_freq_edit_pos = 1;
    draw_generator_immersive(); save("gen-1");
    ui.gen_wave = SIGGEN_WAVE_SQUARE; ui.running = 0;
    ui.gen_param = GEN_PARAM_DUTY; gen_freq_edit_pos = 1;
    draw_generator_immersive(); save("gen-2");
    ui.gen_wave = SIGGEN_WAVE_TRIANGLE; ui.running = 1;
    ui.gen_param = GEN_PARAM_WAVE;
    draw_generator_immersive(); save("gen-3");
    ui.gen_wave = SIGGEN_WAVE_SINE; ui.gen_param = GEN_PARAM_AMP; gen_freq_edit_pos = 0;
    draw_generator_immersive(); save("gen-4");
    gen_sweep_menu_active = 1; gen_sweep_mode = 1; gen_sweep_row_selected = 2;
    ui_edit_digit_active = 1; ui_edit_digit_index = 1;
    draw_generator_immersive(); save("gen-sweep-menu");
    gen_sweep_menu_active = 0; gen_sweep_mode = 0; ui_edit_digit_active = 0;
    ui.mode = UI_MODE_DMM; ui.running = 1;

    ui.overlay = UI_OVERLAY_SETTINGS;
    ui.storage_known = 1; ui.storage_total_kb = 8192; ui.storage_free_kb = 7373;
    for (uint8_t i = 0; i < 6u; ++i) {
        char name[32];
        ui.settings_row = i;
        draw_settings_menu();
        snprintf(name, sizeof(name), "settings-%u", (unsigned)i + 1u);
        save(name);
    }
    ui.settings_row = SETTINGS_ROW_RESET;
    ui.reset_confirm = 1;
    draw_settings_menu(); draw_settings_reset_overlay(); save("settings-reset-confirm");
    ui.reset_confirm = 0;
    ui.settings_row = SETTINGS_BOOT_EXPORT_ROW;
    ui.boot_export_state = BOOT_EXPORT_UI_SAVING; ui.boot_export_percent = 62;
    draw_settings_menu(); draw_boot_export_overlay(); save("settings-export-progress");
    ui.boot_export_state = BOOT_EXPORT_UI_NONE;
    ui.overlay = UI_OVERLAY_NONE;
    return 0;
}
