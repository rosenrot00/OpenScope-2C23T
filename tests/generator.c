#define SCOPE_UI_SAFE_STUB 0
#include "../src/ui.c"
#include "../src/fpga.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tuning_word;
static uint32_t scope_span;
static unsigned timing_writes;
static unsigned buffer_writes;
static uint8_t output_buffer[FPGA_SAMPLE_COUNT];
static uint8_t file_load_ok;
static uint8_t available_files = 2;
static unsigned boot_exports;

void fpga_init_once(void) {}
uint8_t fpga_ready(void) { return 1; }
void fpga_write_generator_timing(uint32_t word) {
    tuning_word = word;
    ++timing_writes;
}
void fpga_write_scope_timing(uint32_t span) { scope_span = span; }
void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    assert(len == FPGA_SAMPLE_COUNT);
    memcpy(output_buffer, data, len);
    ++buffer_writes;
}
void delay_ms(uint32_t ms) { (void)ms; }
void dmm_pause(void) {}
void dmm_hw4_set_mode_gate(uint8_t active) { (void)active; }
void dmm_reenter(uint8_t mode) { (void)mode; }
void dmm_set_mode(uint8_t mode) { (void)mode; }
void settings_note(const settings_state_t *settings) { (void)settings; }
void settings_flush(void) {}
void board_buzzer_set_volume(uint8_t level) { (void)level; }
void board_backlight_set_level(uint8_t level) { (void)level; }
boot_export_result_t usb_msc_export_bootloader(const char *version,
                                               boot_export_progress_fn_t progress) {
    assert(!strcmp(version, FIRMWARE_VERSION_TEXT));
    ++boot_exports;
    progress(100);
    return BOOT_EXPORT_OK;
}

void scope_hw_configure_channels(uint8_t tb, uint8_t r0, uint8_t r1,
                                 uint8_t dc0, uint8_t dc1, uint16_t dac0, uint16_t dac1) {
    (void)r0; (void)r1; (void)dc0; (void)dc1; (void)dac0; (void)dac1;
    fpga_write_scope_timing(50000u + tb);
}
void scope_hw_arm(void) {}
void scope_hw_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac) { (void)ch1_dac; (void)ch2_dac; }
uint8_t usb_msc_store_screenshot(void) { return 0; }
void scope_hw_slow_start(uint8_t timebase) { (void)timebase; }
void scope_hw_slow_stop(void) {}
uint8_t scope_hw_enabled(void) { return 1; }
uint8_t scope_hw_ready(void) { return 1; }
uint8_t scope_hw_capture(uint8_t *dst, uint16_t len, uint8_t tb) {
    (void)dst; (void)len; (void)tb; return 0;
}
uint8_t scope_hw_capture_point(uint8_t sample[2], uint8_t tb) {
    (void)sample; (void)tb; return 0;
}
uint8_t scope_hw_slow_snapshot(uint8_t *ch1, uint8_t *ch2, uint16_t max,
                               uint16_t *count, uint16_t *seq) {
    (void)ch1; (void)ch2; (void)max; (void)count; (void)seq; return 0;
}

uint8_t arb_file_count(void) { return available_files; }
uint8_t arb_scan_files(void) { return available_files != 0; }
uint16_t arb_file_sample_count(uint8_t index) { (void)index; return 3; }
uint8_t arb_load_file(uint8_t index, uint8_t *buffer, uint16_t max) {
    (void)index;
    assert(max >= 3);
    if (!file_load_ok) return 0;
    buffer[0] = 0; buffer[1] = 255; buffer[2] = 64;
    return 1;
}
const char *arb_file_name(uint8_t index) { (void)index; return "test"; }
uint8_t dmm_has_reading(void) { return 0; }
uint8_t dmm_value_is_numeric(void) { return 0; }
int32_t dmm_value_milli_units(void) { return 0; }
int32_t dmm_value_fixed_units(void) { return 0; }
const char *dmm_value_text(void) { return ""; }
const char *dmm_unit_text(void) { return ""; }
const char *dmm_status_text(void) { return ""; }
uint8_t dmm_reading_is_real(void) { return 0; }
void lcd_render(lcd_render_fn_t fn, void *ctx) { (void)fn; (void)ctx; }
void lcd_fill(uint16_t color) { (void)color; }
void lcd_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    (void)x; (void)y; (void)w; (void)h; (void)color;
}
void lcd_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    (void)x; (void)y; (void)w; (void)h; (void)color;
}
void lcd_line(int16_t x, int16_t y, int16_t x1, int16_t y1, uint16_t color) {
    (void)x; (void)y; (void)x1; (void)y1; (void)color;
}
void lcd_text(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale) {
    (void)x; (void)y; (void)str; (void)fg; (void)bg; (void)scale;
}
void lcd_text_transparent(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint8_t scale) {
    (void)x; (void)y; (void)str; (void)fg; (void)scale;
}
void lcd_text_center(uint16_t x, uint16_t y, uint16_t w, const char *str, uint16_t fg, uint16_t bg, uint8_t scale) {
    (void)x; (void)y; (void)w; (void)str; (void)fg; (void)bg; (void)scale;
}
uint16_t lcd_text_width(const char *text, uint8_t scale) { return (uint16_t)(strlen(text) * 6u * scale); }
uint32_t load_counter_read(void) { return 0; }
uint32_t load_counter_elapsed(uint32_t start, uint32_t end) { return end - start; }
void battery_update(void) {}
void battery_update_charging_status(void) {}
uint16_t battery_millivolts(void) { return 4000; }
uint8_t battery_percent(void) { return 80; }
uint8_t battery_is_charging(void) { return 0; }
void fw_update_status(fw_update_status_t *status) { memset(status, 0, sizeof(*status)); }

static void setup(void) {
    siggen_shutdown();
    gen_sweep_mode = gen_fm_mode = 0;
    gen_sweep_elapsed_ms = gen_fm_phase = 0;
    gen_deferred_apply = gen_deferred_apply_ms = 0;
    bode_siggen_active = bode_is_sweeping = bode_saved.valid = 0;
    scope_fft_src = 0;
    gen_running = ui.running = 1;
    gen_output_applied = 0;
    ui.mode = UI_MODE_GEN;
    ui.overlay = UI_OVERLAY_NONE;
    ui.gen_param = GEN_PARAM_FREQ;
    ui.gen_wave = SIGGEN_WAVE_SINE;
    ui.gen_freq_hz = 1000;
    ui.gen_duty_percent = 50;
    ui.gen_amp_tenths_v = 33;
    ui.scope_timebase = 7;
    gen_apply();
}

static void assert_zero_output(void) {
    for (unsigned i = 0; i < FPGA_SAMPLE_COUNT; ++i) assert(output_buffer[i] == 0);
}

static void test_mode_switches(void) {
    for (uint8_t modulation = 0; modulation < 3; ++modulation) {
        setup();
        gen_sweep_mode = modulation == 1;
        gen_fm_mode = modulation == 2;
        gen_sweep_ms = 1000;
        siggen_sweep_start_hz = 100;
        siggen_sweep_stop_hz = 10000;
        gen_apply();
        ui_switch_mode(UI_MODE_SCOPE);
        uint32_t span = scope_span;
        assert(gen_running && gen_output_applied && tuning_word);
        ui_tick(20);
        assert(gen_running && tuning_word && scope_span == span);
        ui_switch_mode(UI_MODE_DMM);
        assert(!tuning_word && !gen_running && !gen_output_applied && !gen_deferred_apply);
        assert_zero_output();
        unsigned writes = timing_writes;
        for (unsigned i = 0; i < 10; ++i) ui_tick(20);
        assert(!tuning_word && timing_writes == writes);
        ui_switch_mode(UI_MODE_GEN);
        assert(!ui.running && !gen_running && !tuning_word);
        ui.running = gen_running = 1;
        gen_apply();
        assert(tuning_word && gen_output_applied);
    }
}

static void test_failed_csv(void) {
    setup();
    ui.gen_wave = SIGGEN_WAVE_ARBITRARY;
    arb_file_index = 0;
    file_load_ok = 1;
    arb_load_waveform();
    gen_apply();
    uint16_t count;
    assert(siggen_get_arb_waveform(&count) && count == 3 && output_buffer[FPGA_SAMPLE_COUNT / 3u] > 200);
    unsigned uploads = buffer_writes;
    arb_file_index = 1;
    file_load_ok = 0;
    arb_load_waveform();
    gen_apply();
    assert(!arb_loaded && !arb_sample_count && !siggen_get_arb_waveform(&count) && !count);
    assert(buffer_writes == uploads + 1);
    assert_zero_output();
    ui.gen_amp_tenths_v = 20;
    gen_apply();
    assert_zero_output();
    file_load_ok = 1;
    arb_load_waveform();
    gen_apply();
    assert(arb_loaded && output_buffer[FPGA_SAMPLE_COUNT / 3u] > 100);
    available_files = 0;
    arb_load_waveform();
    gen_apply();
    assert(!siggen_get_arb_waveform(&count) && !count);
    assert_zero_output();
    available_files = 2;
}

static void test_deferred_uploads(void) {
    for (uint8_t fm = 0; fm < 2; ++fm) {
        setup();
        gen_sweep_mode = !fm;
        gen_fm_mode = fm;
        gen_fm_freq_hz = 5;
        gen_sweep_ms = 1000;
        siggen_sweep_start_hz = 100;
        siggen_sweep_stop_hz = 10000;
        gen_apply();
        for (uint8_t param = GEN_PARAM_DUTY; param <= GEN_PARAM_AMP; ++param) {
            ui.gen_wave = SIGGEN_WAVE_SQUARE;
            gen_apply();
            ui.gen_param = param;
            gen_freq_edit_pos = param == GEN_PARAM_DUTY ? 2u : 1u;
            unsigned uploads = buffer_writes;
            uint32_t old_word = tuning_word;
            assert(gen_adjust_current_value(-1, 0) == 2);
            for (unsigned i = 0; i < 4; ++i) ui_tick(20);
            assert(buffer_writes == uploads && gen_deferred_apply && gen_deferred_apply_ms == 20);
            assert(tuning_word != old_word);
            assert(gen_adjust_current_value(-1, 0) == 2);
            for (unsigned i = 0; i < 4; ++i) ui_tick(20);
            assert(buffer_writes == uploads && gen_deferred_apply);
            ui_tick(20);
            assert(buffer_writes == uploads + 1 && !gen_deferred_apply);
        }
    }
    siggen_shutdown();
    unsigned writes = timing_writes;
    siggen_set_frequency(2000);
    assert(!tuning_word && writes == timing_writes);
}

static void test_bode_ownership(void) {
    setup();
    gen_sweep_mode = 1;
    gen_sweep_ms = 1000;
    siggen_sweep_start_hz = 100;
    siggen_sweep_stop_hz = 10000;
    ui.gen_amp_tenths_v = 20;
    gen_schedule_deferred_apply();
    ui.mode = UI_MODE_SCOPE;
    ui.running = 0;
    bode_start_hz = 1000;
    bode_stop_hz = 10000;
    ui_settings.siggen_amp_tenths_v = 33;
    bode_begin_sweep();
    assert(bode_siggen_active);
    uint32_t word = tuning_word;
    uint32_t span = scope_span;
    unsigned uploads = buffer_writes;
    unsigned writes = timing_writes;
    for (unsigned i = 0; i < 10; ++i) ui_tick(20);
    assert(tuning_word == word && scope_span == span);
    assert(buffer_writes == uploads && timing_writes == writes && gen_deferred_apply);
    bode_shutdown(1);
    assert(!bode_siggen_active && gen_running && !gen_deferred_apply);
    ui_tick(20);
    assert(timing_writes > writes);
}

static void test_waveform_bounds(void) {
    static const uint8_t duties[] = {1, 10, 49, 50, 99, 100};
    uint8_t arb[] = {0, 255, 0, 64};
    siggen_set_arb_waveform(arb, sizeof(arb));
    for (uint8_t wave = 0; wave < SIGGEN_WAVE_COUNT; ++wave) {
        for (uint8_t amp = 1; amp <= 33; ++amp) {
            for (unsigned d = 0; d < sizeof(duties); ++d) {
                siggen_configure(1, wave, 1000, duties[d], amp);
                for (unsigned i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
                    assert(output_buffer[i] <= (uint32_t)amp * 100u * 255u / 3300u);
                }
            }
        }
    }
    siggen_shutdown();
    assert_zero_output();
    siggen_set_arb_waveform(0, 0);
}

static void test_sweep_units(void) {
    setup();
    ui_settings.siggen_sweep_start_hz = 50000;
    ui_settings.siggen_sweep_stop_hz = 500;
    sweep_start_unit = sweep_stop_unit = 0;
    ui_apply_saved_runtime_settings();
    assert(sweep_start_unit == 1 && freq_clamp_display(siggen_sweep_start_hz, sweep_start_unit) == 50);
    assert(sweep_stop_unit == 0 && freq_clamp_display(siggen_sweep_stop_hz, sweep_stop_unit) == 500);
    ui_settings.siggen_sweep_start_hz = 9999;
    ui_settings.siggen_sweep_stop_hz = 2000000;
    ui_apply_saved_runtime_settings();
    assert(sweep_start_unit == 0 && sweep_stop_unit == 1);
    assert(freq_clamp_display(siggen_sweep_stop_hz, sweep_stop_unit) == 2000);
    siggen_shutdown();
}

static void test_bode_units(void) {
    setup();
    ui.mode = UI_MODE_SCOPE;
    ui_math_menu_visible = 0;
    ui_settings.bode_start_hz = 100;
    ui_settings.bode_stop_hz = 100000;
    bode_start_unit = bode_stop_unit = 0;
    ui_handle_keys(KEY_CH1_LONG);
    assert(ui_math_menu_visible);
    assert(bode_start_unit == 0 && freq_clamp_display(bode_start_hz, bode_start_unit) == 100);
    assert(bode_stop_unit == 1 && freq_clamp_display(bode_stop_hz, bode_stop_unit) == 100);
    ui_math_menu_visible = 0;
    ui.mode = UI_MODE_GEN;
    siggen_shutdown();
}

static void test_settings_export(void) {
    ui.overlay = UI_OVERLAY_SETTINGS;
    ui.settings_row = SETTINGS_BOOT_EXPORT_ROW;
    ui.boot_export_state = BOOT_EXPORT_UI_NONE;
    assert(SETTINGS_BOOT_EXPORT_ROW == SETTINGS_ROW_COUNT - 2u);
    assert(!strcmp(settings_row_labels[SETTINGS_ROW_COUNT - 1u], "INFO"));
    ui_handle_menu_keys(KEY_AUTO);
    assert(ui.boot_export_state == BOOT_EXPORT_UI_NONE && !boot_exports);
    ui_handle_menu_keys(KEY_OK);
    assert(ui.boot_export_state == BOOT_EXPORT_UI_CONFIRM && !boot_exports);
    ui_handle_menu_keys(KEY_OK | KEY_REPEAT);
    assert(ui.boot_export_state == BOOT_EXPORT_UI_CONFIRM && !boot_exports);
    ui_handle_menu_keys(KEY_MENU);
    assert(ui.boot_export_state == BOOT_EXPORT_UI_NONE && ui.overlay == UI_OVERLAY_SETTINGS);
    ui_handle_menu_keys(KEY_OK);
    ui_handle_menu_keys(KEY_OK);
    assert(boot_exports == 1 && ui.boot_export_state == BOOT_EXPORT_UI_RESULT);
    assert(ui.boot_export_result == BOOT_EXPORT_OK && ui.boot_export_percent == 100);
    ui_handle_menu_keys(KEY_RIGHT);
    assert(ui.settings_row == 0 && ui.boot_export_state == BOOT_EXPORT_UI_NONE);
    for (uint8_t i = 0; i < SETTINGS_SELECTABLE_COUNT; ++i) {
        ui_handle_menu_keys(KEY_RIGHT);
        assert(ui.settings_row < SETTINGS_ROW_COUNT - 1u);
    }
}

int main(void) {
    test_mode_switches();
    test_failed_csv();
    test_deferred_uploads();
    test_bode_ownership();
    test_waveform_bounds();
    test_sweep_units();
    test_bode_units();
    test_settings_export();
    printf("generator: mode transitions, CSV failures, deferred uploads, Bode ownership and waveforms passed (HW4=%d)\n", HW_TARGET_HW40);
    return 0;
}
