#include "ui.h"

#include "board.h"
#include "dmm.h"
#include "display.h"
#include "fw_update.h"
#include "hw.h"
#include "screenshot.h"
#include "settings.h"
#include "siggen.h"
#include "scope.h"
#include "usb_msc.h"

#include <stdint.h>

#define GEN_DEFAULT_AMP_TENTHS 33u
#define GEN_DEFAULT_DUTY_PERCENT 50u
#define GEN_DEFAULT_FREQ_HZ 1000u
#define GEN_DEFAULT_FREQ_UNIT 0u
#define GEN_MIN_FREQ_HZ 1u
#define GEN_MAX_FREQ_HZ 2000000u
#define GEN_DEFAULT_PARAM 1u
#define UI_PARAM_ROW_X 7u
#define UI_PARAM_CHIP_W 72u
#define UI_PARAM_CHIP_GAP 6u
#define GEN_PARAM_ROW_X UI_PARAM_ROW_X
#define GEN_PARAM_CHIP_W UI_PARAM_CHIP_W
#define GEN_PARAM_CHIP_GAP UI_PARAM_CHIP_GAP
#define GEN_OUTPUT_X (GEN_PARAM_ROW_X + (GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP) * 3u)
#define GEN_OUTPUT_W GEN_PARAM_CHIP_W
#define GEN_FREQ_EDIT_DIGITS 4u
#define GEN_FREQ_EDIT_UNIT 4u
#define GEN_FREQ_UNIT_COUNT 3u
#define GEN_DUTY_EDIT_DIGITS 3u
#define GEN_AMP_EDIT_DIGITS 2u
#define FIRMWARE_VERSION_TEXT "v2026.04.1"
#ifndef SCOPE_UI_SAFE_STUB
#define SCOPE_UI_SAFE_STUB 1
#endif
#define SCOPE_ZERO_STEP 3
#define SCOPE_H_POS_LIMIT 120
#define SCOPE_V_POS_LIMIT 120
#define SCOPE_X_DIVS 12u
#define SCOPE_Y_DIVS 8u
#define SCOPE_TIMEBASE_DEFAULT 4u
#define SCOPE_MOVE_SEL_CHANNEL 0u
#define SCOPE_MOVE_SEL_Y 1u
#define SCOPE_MOVE_SEL_X 2u
#define SCOPE_MOVE_SEL_ZERO 3u
#define SCOPE_CURSOR_MENU_DELTA 0u
#define SCOPE_CURSOR_MENU_MODE 1u
#define SCOPE_CURSOR_MENU_FIRST 2u
#define SCOPE_CURSOR_MENU_SECOND 3u
#define SCOPE_CURSOR_MENU_COUNT 4u
#define SCOPE_MEASURE_MENU_CH1 0u
#define SCOPE_MEASURE_MENU_VALUE 1u
#define SCOPE_MEASURE_MENU_CH2 2u
#define SCOPE_MEASURE_MENU_VISIBLE 3u
#define SCOPE_MEASURE_MENU_COUNT 4u
#define SCREENSHOT_OVERLAY_MS 1400u

enum {
    SCREENSHOT_STATE_SAVING,
    SCREENSHOT_STATE_OK,
    SCREENSHOT_STATE_ERROR,
};

typedef enum {
    UI_MODE_DMM,
    UI_MODE_SCOPE,
    UI_MODE_GEN,
    UI_MODE_COUNT,
} ui_mode_t;

typedef enum {
    UI_OVERLAY_NONE,
    UI_OVERLAY_MODE_MENU,
    UI_OVERLAY_SETTINGS,
} ui_overlay_t;

typedef struct {
    ui_mode_t mode;
    ui_overlay_t overlay;
    uint8_t running;
    uint8_t auto_range;
    uint8_t chrome_visible;
    uint8_t active_ch;
    uint8_t dmm_mode;
    uint8_t scope_timebase;
    uint8_t scope_vdiv;
    uint8_t scope_display;
    uint8_t scope_cursor_mode;
    uint8_t scope_cursor_sel;
    uint8_t scope_param;
    uint8_t scope_channel_menu;
    uint8_t scope_trigger_menu;
    uint8_t scope_measure_menu;
    uint8_t scope_measure_menu_sel;
    uint8_t scope_cursor_menu;
    uint8_t scope_cursor_menu_sel;
    uint8_t scope_move_mode;
    uint8_t scope_move_sel;
    uint8_t scope_trigger_source;
    uint8_t scope_trigger_mode;
    uint8_t scope_trigger_edge;
    uint8_t scope_trigger_level;
    uint8_t scope_afterglow;
    uint8_t gen_wave;
    uint8_t gen_param;
    uint8_t gen_duty_percent;
    uint8_t gen_amp_tenths_v;
    uint32_t gen_freq_hz;
    uint32_t sleep_ms;
    uint16_t idle_ms;
    uint16_t battery_ms;
    uint16_t charger_ms;
    uint16_t battery_blink_ms;
    uint16_t softkeys_ms;
    uint16_t scope_move_menu_ms;
    uint16_t scope_ms;
    uint16_t dmm_render_ms;
    uint16_t screenshot_overlay_ms;
    uint16_t scope_phase;
    uint8_t battery_blink_on;
    uint8_t menu_index;
    uint8_t settings_row;
    uint8_t sleep_due;
    uint8_t dmm_render_pending;
    uint8_t screenshot_state;
} ui_state_t;

typedef struct {
    uint8_t valid;
    uint8_t pct;
    uint8_t charging;
    uint8_t full;
    uint16_t voltage_bucket;
} battery_snapshot_t;

typedef struct {
    uint8_t valid;
    uint8_t mode;
    uint8_t decimals;
    uint8_t zero_count;
    uint8_t zero_idle;
    uint16_t count;
    int32_t min_value;
    int32_t avg_value;
    int32_t max_value;
    char unit[6];
} dmm_stats_t;

enum {
    SCOPE_TRACE_STEP = 3,
    SCOPE_TRACE_MAX_POINTS = 101,
    SCOPE_SAMPLE_COUNT = 2048,
    SCOPE_SAMPLE_BYTES = SCOPE_SAMPLE_COUNT * 2,
    SCOPE_SAMPLES_PER_DIV = 25,
    SCOPE_VISIBLE_SAMPLE_COUNT = SCOPE_X_DIVS * SCOPE_SAMPLES_PER_DIV,
    SCOPE_FAST_HW_SAMPLE_NS = 20,
    SCOPE_SOFT_ROLL_TIMEBASE = 18,
    SCOPE_IRQ_ROLL_TIMEBASE_START = 19,
    SCOPE_SLOW_ROLL_TIMEBASE_START = SCOPE_SOFT_ROLL_TIMEBASE,
    SCOPE_SLOW_ROLL_MAX_POINTS = 300,
    SCOPE_SLOW_ROLL_MIN_MS = 20,
    SCOPE_SOFT_ROLL_MAX_POINTS_PER_FRAME = 12,
    SCOPE_MEASURE_WINDOW = 10,
    SCOPE_FREQ_AVG_START = 3,
    SCOPE_FREQ_AVG_COUNT = 4,
};

typedef struct {
    uint8_t valid;
    uint8_t ch2;
    uint8_t timebase;
    uint8_t count;
    uint16_t phase;
    uint16_t frame_id;
    uint16_t width;
    int16_t x0;
    int16_t center;
    int16_t amp;
    int16_t min_y;
    int16_t max_y;
    int16_t y[SCOPE_TRACE_MAX_POINTS];
} scope_trace_cache_t;

static ui_state_t ui = {
    .mode = UI_MODE_DMM,
    .overlay = UI_OVERLAY_NONE,
    .running = 1,
    .auto_range = 1,
    .chrome_visible = 1,
    .active_ch = 1,
    .gen_param = GEN_DEFAULT_PARAM,
    .gen_duty_percent = GEN_DEFAULT_DUTY_PERCENT,
    .gen_amp_tenths_v = GEN_DEFAULT_AMP_TENTHS,
    .gen_freq_hz = GEN_DEFAULT_FREQ_HZ,
    .battery_blink_on = 1,
};

static uint8_t ui_load_pct;
static uint8_t ui_load_peak_pct;
static uint16_t ui_render_ms_x10;
static uint16_t scope_frame_age_ms;
static uint16_t scope_frame_interval_x10ms;
static uint32_t ui_last_render_ticks;
static uint32_t ui_fw_update_sequence;
static scope_trace_cache_t scope_trace_cache[2];
static battery_snapshot_t battery_snapshot;
static dmm_stats_t dmm_stats;
static uint8_t dmm_hold_active;
static uint8_t dmm_rel_active;
static uint8_t dmm_live_wire_alert;
static uint8_t beep_preview_requested;
static uint8_t gen_freq_edit_pos;
static uint8_t gen_freq_unit = GEN_DEFAULT_FREQ_UNIT;
static uint8_t gen_deferred_apply;
static uint16_t gen_deferred_apply_ms;
static uint8_t gen_output_applied;
static uint8_t scope_samples[SCOPE_SAMPLE_BYTES] __attribute__((aligned(4)));
static uint8_t scope_capture_samples[SCOPE_SAMPLE_BYTES] __attribute__((aligned(4)));
static uint8_t scope_frame_valid;
static uint16_t scope_frame_id;
static uint8_t scope_min_raw[2];
static uint8_t scope_max_raw[2];
static uint8_t scope_avg_raw[2];
static uint32_t scope_sq_sum_raw[2];
static uint8_t scope_measure_min_hist[2][SCOPE_MEASURE_WINDOW];
static uint8_t scope_measure_max_hist[2][SCOPE_MEASURE_WINDOW];
static uint16_t scope_measure_rms_hist[2][SCOPE_MEASURE_WINDOW];
static uint32_t scope_measure_freq_hist[2][SCOPE_MEASURE_WINDOW];
static uint8_t scope_measure_hist_pos;
static uint8_t scope_measure_hist_ready;
static uint8_t scope_ch_enabled[2] = {1, 1};
static uint8_t scope_probe_x10[2];
static uint8_t scope_vdiv_ch[2];
static uint8_t scope_coupling_dc[2] = {1, 1};
static int16_t scope_zero_offset[2];
static uint8_t scope_auto_zero_active;
static uint8_t scope_auto_zero_mask;
static uint8_t scope_auto_zero_stable_count;
static uint8_t scope_auto_zero_steps_left;
static uint8_t scope_auto_channel_mask;
static uint8_t scope_auto_channel_steps_left;
static uint8_t scope_cursor_x[2] = {80, 176};
static uint8_t scope_cursor_y[2] = {86, 170};
static int16_t scope_cursor_time_value[2] = {-48, 48};
static uint8_t scope_cursor_timebase[2] = {SCOPE_TIMEBASE_DEFAULT, SCOPE_TIMEBASE_DEFAULT};
static int32_t scope_cursor_level_mv[2] = {263, -263};
static int8_t scope_ch_pos[2];
static int8_t scope_h_pos;
static int8_t scope_h_value_pos;
static uint8_t scope_h_value_timebase;
static uint8_t scope_measure_param;
static uint8_t scope_measure_mask[2] = {1u, 1u};
static uint8_t scope_measure_visible = 1;
static uint16_t scope_roll_offset;
static uint8_t scope_slow_roll_raw[2][SCOPE_SLOW_ROLL_MAX_POINTS];
static uint16_t scope_slow_roll_count;
static uint16_t scope_slow_roll_head;
static uint8_t scope_slow_roll_timebase = 0xFFu;
static uint16_t scope_slow_roll_seq = 0xFFFFu;
static uint16_t scope_trigger_offset;
static uint8_t scope_trigger_locked;
static uint8_t scope_roll_trigger_pending;
static uint16_t scope_roll_trigger_seq;
static uint16_t scope_roll_trigger_target_seq;
static int16_t scope_roll_display_offset;
static int16_t scope_after_y[2][SCOPE_TRACE_MAX_POINTS];
static uint8_t scope_after_count[2];
static uint8_t scope_after_valid[2];
static int32_t dmm_rel_ref_milli;
static char dmm_hold_value[10];
static char dmm_hold_unit[6];
static char dmm_rel_unit[6];
static char dmm_rel_value[10];
static char dmm_rel_ref_value[10];
static char dmm_low_current_value[10];
static char dmm_status_detail[18];
static settings_state_t ui_settings;

static void ui_settings_copy(settings_state_t *dst, const settings_state_t *src) {
    dst->dmm_mode = src->dmm_mode;
    dst->beep_level = src->beep_level;
    dst->brightness_level = src->brightness_level;
    dst->startup_screen = src->startup_screen;
    dst->last_screen = src->last_screen;
    dst->sleep_enabled = src->sleep_enabled;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            dst->scope_bias[ch][range] = src->scope_bias[ch][range];
            dst->scope_bias_rate[ch][range] = src->scope_bias_rate[ch][range];
        }
    }
}

enum {
    Y_HEADER = 0,
    H_HEADER = 27,
    Y_BODY = 27,
    H_BODY = 174,
    Y_SOFT = 201,
    H_SOFT = 39,
    CHROME_IDLE_MS = 2200,
    SOFTKEY_FLASH_MS = 1400,
    SCOPE_CHANNEL_MENU_MS = 5000,
    SCOPE_MOVE_MENU_MS = SCOPE_CHANNEL_MENU_MS,
    CHARGER_POLL_MS = 1000,
    BATTERY_MEASURE_MS = 10000,
    BATTERY_LOW_BLINK_MS = 2000,
    BATTERY_LOW_PCT = 10,
    SCOPE_FRAME_MS = 60,
    SCOPE_STUB_FRAME_MS = 90,
    DMM_FAST_RENDER_MS = 80,
    DMM_STATS_ZERO_RESET_SAMPLES = 8,
    SETTINGS_ROW_COUNT = 5,
    SETTINGS_SELECTABLE_COUNT = 4,
    SETTINGS_GRID_COLUMNS = 3,
    SETTINGS_SLEEP_TIMEOUT_MS = 600000,
    GEN_PREVIEW_CYCLES = 3,
    GEN_DEFERRED_APPLY_MS = 100,
};

enum {
    SCOPE_DISPLAY_YT,
    SCOPE_DISPLAY_ROLL,
    SCOPE_DISPLAY_XY,
    SCOPE_DISPLAY_COUNT,
};

enum {
    SCOPE_CURSOR_OFF,
    SCOPE_CURSOR_TIME,
    SCOPE_CURSOR_LEVEL,
    SCOPE_CURSOR_COUNT,
};

static const uint16_t C_BG = RGB565(5, 9, 14);
static const uint16_t C_TOP = RGB565(12, 18, 24);
static const uint16_t C_PANEL = RGB565(16, 25, 34);
static const uint16_t C_PANEL_2 = RGB565(22, 34, 45);
static const uint16_t C_GRID = RGB565(39, 53, 63);
static const uint16_t C_TEXT = RGB565(232, 240, 246);
static const uint16_t C_MUTED = RGB565(126, 144, 154);
static const uint16_t C_WARN = RGB565(238, 74, 80);
static const uint16_t C_CH1 = RGB565(245, 197, 66);
static const uint16_t C_CH2 = RGB565(48, 206, 230);
static const uint16_t C_DMM = RGB565(43, 202, 134);
static const uint16_t C_SCOPE = RGB565(116, 154, 255);
static const uint16_t C_GEN = RGB565(178, 118, 255);
static const uint16_t C_MENU_SCOPE = RGB565(180, 202, 220);
static const uint16_t C_MENU_GEN = RGB565(190, 118, 255);
static const uint16_t C_MENU_SETTINGS = RGB565(255, 126, 92);
static const uint16_t C_BATT = RGB565(232, 240, 246);
static const uint16_t C_BATT_FULL = RGB565(232, 240, 246);

enum {
    DMM_MODE_AUTO,
    DMM_MODE_DCV,
    DMM_MODE_ACV,
    DMM_MODE_RES,
    DMM_MODE_DIODE,
    DMM_MODE_CAP,
    DMM_MODE_LIVE,
    DMM_MODE_CONT,
    DMM_MODE_TEMP,
    DMM_MODE_AC_HI_CURR,
    DMM_MODE_DC_HI_CURR,
    DMM_MODE_AC_LO_CURR,
    DMM_MODE_DC_LO_CURR,
    DMM_MODE_COUNT,
};

enum {
    GEN_PARAM_WAVE,
    GEN_PARAM_FREQ,
    GEN_PARAM_DUTY,
    GEN_PARAM_AMP,
    GEN_PARAM_COUNT,
};

enum {
    SCOPE_PARAM_VIEW,
    SCOPE_PARAM_ENABLE,
    SCOPE_PARAM_PROBE,
    SCOPE_PARAM_GAIN,
    SCOPE_PARAM_COUPLING,
    SCOPE_PARAM_ZERO,
    SCOPE_PARAM_POSITION,
    SCOPE_PARAM_TIME,
    SCOPE_PARAM_TRIGGER,
    SCOPE_PARAM_CURSOR,
    SCOPE_PARAM_TRIG_SOURCE,
    SCOPE_PARAM_TRIG_LEVEL,
    SCOPE_PARAM_TRIG_TYPE,
    SCOPE_PARAM_TRIG_EDGE,
    SCOPE_PARAM_MEASURE,
    SCOPE_PARAM_COUNT,
};

enum {
    SCOPE_TRIGGER_AUTO,
    SCOPE_TRIGGER_NORMAL,
    SCOPE_TRIGGER_SINGLE,
    SCOPE_TRIGGER_COUNT,
};

enum {
    SCOPE_MEASURE_VPP,
    SCOPE_MEASURE_VMAX,
    SCOPE_MEASURE_VMIN,
    SCOPE_MEASURE_VAVG,
    SCOPE_MEASURE_VRMS,
    SCOPE_MEASURE_FREQ,
    SCOPE_MEASURE_COUNT,
};

static const char *const dmm_mode_names[] = {
    "AUTO DETECT", "DC VOLTAGE", "AC VOLTAGE", "RESISTANCE",
    "DIODE", "CAPACITANCE", "LIVE WIRE", "CONTINUITY",
    "TEMPERATURE", "AC HIGH CURR", "DC HIGH CURR",
    "AC LOW CURR", "DC LOW CURR",
};
static const char *const dmm_mode_short[] = {
    "AUTO", "DCV", "ACV", "RES", "DIODE", "CAP", "LIVE",
    "CONT", "TEMP", "AC HI", "DC HI", "AC LO", "DC LO",
};
static const char *const dmm_values[] = {
    "12.346", "12.346", "0.218", "1.204", "0.624", "0.047",
    "LIVE", "0.000", "23.8", "0.000", "0.000", "0.000", "0.000",
};
static const char *const dmm_units[] = {
    "V", "V", "V", "KOHM", "V", "UF", "", "OHM", "DEG", "A", "A", "MA", "MA",
};
static const uint32_t scope_timebase_unit_ns[] = {
    5u, 10u, 20u, 50u, 100u, 200u, 500u,
    1000u, 2000u, 5000u, 10000u, 20000u, 50000u,
    100000u, 200000u, 500000u, 1000000u, 2000000u, 5000000u,
    10000000u, 20000000u, 50000000u, 100000000u, 200000000u,
    500000000u, 1000000000u,
};
enum {
    SCOPE_TIMEBASE_COUNT = sizeof(scope_timebase_unit_ns) / sizeof(scope_timebase_unit_ns[0]),
};
static const char *const scope_timebases[] = {
    "50NS", "100NS", "200NS", "500NS", "1US", "2US", "5US",
    "10US", "20US", "50US", "100US", "200US", "500US",
    "1MS", "2MS", "5MS", "10MS", "20MS", "50MS",
    "100MS", "200MS", "500MS", "1S", "2S", "5S", "10S",
};
static const char *const scope_timebases_short[] = {
    "50N", "100N", "200N", "500N", "1U", "2U", "5U",
    "10U", "20U", "50U", "100U", "200U", "500U",
    "1M", "2M", "5M", "10M", "20M", "50M",
    "100M", "200M", "500M", "1S", "2S", "5S", "10S",
};
static const char *const scope_vdivs_short[] = {
    "20M", "50M", "100M", "200M", "500M", "1V", "2V", "5V", "10V",
};
static const char *const scope_vdivs_x10_short[] = {
    "200M", "500M", "1V", "2V", "5V", "10V", "20V", "50V", "100V",
};
static const uint32_t scope_vdiv_base_mv[] = {20u, 50u, 100u, 200u, 500u, 1000u, 2000u, 5000u, 10000u};
static const uint16_t scope_vdiv_range_code[] = {1000u, 400u, 200u, 100u, 40u, 20u, 10u, 4u, 2u};
enum {
    SCOPE_VDIV_COUNT = sizeof(scope_vdiv_base_mv) / sizeof(scope_vdiv_base_mv[0]),
};
static const char *const scope_display_labels[] = {"Y-T", "ROLL", "X-Y"};
static const char *const scope_display_short[] = {"YT", "ROLL", "XY"};
static const char *const scope_cursor_labels[] = {"CUR OFF", "T CUR", "Y CUR"};
static const char *const scope_cursor_short[] = {"OFF", "T", "Y"};
static const char *const scope_trigger_short[] = {"AUTO", "NORM", "SNGL"};
static const char *const scope_measure_labels[] = {"VPP", "VMAX", "VMIN", "VAVG", "VRMS", "FREQ"};
static const char *const scope_param_labels[] = {
    "VIEW", "ON", "PROBE", "V/DIV", "COUP", "ZERO", "POS", "TIME", "TRIG", "CUR",
    "SRC", "LEVEL", "MODE", "EDGE", "MEAS",
};
static const uint8_t scope_global_param_order[] = {
    SCOPE_PARAM_POSITION,
    SCOPE_PARAM_CURSOR,
    SCOPE_PARAM_TRIGGER,
    SCOPE_PARAM_MEASURE,
};
static const uint8_t scope_channel_param_order[] = {
    SCOPE_PARAM_ENABLE,
    SCOPE_PARAM_PROBE,
    SCOPE_PARAM_COUPLING,
    SCOPE_PARAM_GAIN,
};
static const uint8_t scope_trigger_param_order[] = {
    SCOPE_PARAM_TRIG_SOURCE,
    SCOPE_PARAM_TRIG_EDGE,
    SCOPE_PARAM_TRIG_TYPE,
    SCOPE_PARAM_TRIG_LEVEL,
};
static const char *const gen_wave_labels[] = {
    "SINE", "RECT", "SAW", "HALF", "FULL",
    "P STEP", "R STEP", "EXP UP", "EXP DN", "DC",
    "MULTI AUD", "SINK PLS", "LORENTZ", "TRIANGLE", "NOISE",
};
static const char *const gen_wave_short_labels[] = {
    "SINE", "RECT", "SAW", "HALF", "FULL",
    "PSTEP", "RSTEP", "EXP+", "EXP-", "DC",
    "MULTI", "SINK", "LOR", "TRI", "NOISE",
};
static const uint8_t gen_wave_order[] = {
    SIGGEN_WAVE_SINE,
    SIGGEN_WAVE_SQUARE,
    SIGGEN_WAVE_TRIANGLE,
    SIGGEN_WAVE_FULL,
    SIGGEN_WAVE_HALF,
    SIGGEN_WAVE_NOISE,
    SIGGEN_WAVE_DC,
    SIGGEN_WAVE_SAW,
    SIGGEN_WAVE_POS_STEP,
    SIGGEN_WAVE_REV_STEP,
    SIGGEN_WAVE_EXP_RISE,
    SIGGEN_WAVE_EXP_FALL,
    SIGGEN_WAVE_MULTI_AUDIO,
    SIGGEN_WAVE_SINKER_PULSE,
    SIGGEN_WAVE_LORENTZ,
};
static const char *const gen_param_labels[] = {"WAVE", "FREQ", "DUTY", "AMP"};
static const char *const menu_labels[] = {"MULTIMETER", "OSCILLOSCOPE", "SIGNAL GENERATOR", "SETTINGS"};
static const char *const settings_row_labels[] = {"BEEP", "DISPLAY", "START", "SLEEP", "INFO"};
static const char *const startup_labels[] = {"MENU", "DMM", "SCOPE", "GEN"};
static const char *const beep_level_labels[] = {"OFF", "LOW", "MEDIUM", "HIGH", "MAX"};
static const char *const brightness_level_labels[] = {"DIM", "LOW", "MEDIUM", "HIGH", "BRIGHT"};
static const char *const sleep_labels[] = {"OFF", "10 MIN"};
static const uint8_t beep_level_percent[SETTINGS_LEVEL_COUNT] = {0, 10, 50, 75, 100};
static const uint8_t brightness_level_percent[SETTINGS_LEVEL_COUNT] = {20, 40, 60, 80, 100};

static const int8_t scope_sine_lut[64] = {
    0, 6, 13, 19, 25, 31, 36, 41,
    45, 49, 53, 56, 59, 61, 63, 64,
    64, 64, 63, 61, 59, 56, 53, 49,
    45, 41, 36, 31, 25, 19, 13, 6,
    0, -6, -13, -19, -25, -31, -36, -41,
    -45, -49, -53, -56, -59, -61, -63, -64,
    -64, -64, -63, -61, -59, -56, -53, -49,
    -45, -41, -36, -31, -25, -19, -13, -6,
};

static void dmm_pause_for_menu_overlay(void);
static void dmm_apply_selected_mode(void);
static void gen_prepare_state(void);
static void gen_apply(void);
static void gen_normalize_param(void);
static void draw_gen_param_row(uint16_t x, uint16_t y);
static void draw_scope_param_row(uint16_t x, uint16_t y);
static void draw_scope_channel_param_row(uint16_t x, uint16_t y);
static void draw_scope_trigger_param_row(uint16_t x, uint16_t y);
static uint16_t scope_channel_color(uint8_t ch);
static uint32_t scope_vdiv_mv_for_channel(uint8_t idx);
static uint32_t scope_vdiv_mv(void);
static int32_t scope_raw_delta_mv(uint8_t idx, uint8_t raw);
static void scope_format_signed_mv(char out[11], int32_t mv);
static void scope_format_u32(char *out, uint32_t value);
static void scope_format_delta_time(char out[14]);
static void scope_format_delta_level(char out[14]);
static void scope_text_copy(char *dst, const char *src, uint8_t max_len);
static void ui_text_append(char *dst, const char *src, uint8_t max_len);
static void scope_step_measure_param(int8_t dir);
static void scope_toggle_measure_for_channel(uint8_t idx);
static void scope_toggle_measure_visible(void);
static uint32_t scope_timebase_unit_ns_value(void);
static uint32_t scope_sample_period_ns(void);
static int8_t scope_clamp_channel_pos(uint8_t idx, int16_t pos);
static uint32_t scope_estimate_freq_hz_window(uint8_t idx, uint16_t start, uint16_t end, uint8_t min_raw, uint8_t max_raw);
static void scope_auto_zero_begin(uint8_t mask);
static void scope_auto_zero_service(void);
static void scope_auto_channel_service(void);
static void scope_set_channel_pos_from_avg(uint8_t idx);
static uint8_t scope_any_menu_open(void);
static uint8_t scope_softkey_menu_open(void);
static void scope_clear_menus(void);

static uint16_t mode_accent(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        return C_SCOPE;
    }
    if (ui.mode == UI_MODE_GEN) {
        return C_GEN;
    }
    return C_DMM;
}

static const char *mode_title(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        return "OSCILLOSCOPE";
    }
    if (ui.mode == UI_MODE_GEN) {
        return "SIGNAL GENERATOR";
    }
    return "MULTIMETER";
}

static const char *mode_short(ui_mode_t mode) {
    if (mode == UI_MODE_SCOPE) {
        return "OSC";
    }
    if (mode == UI_MODE_GEN) {
        return "SIG";
    }
    return "DMM";
}

static void format_percent(char out[5]) {
    uint8_t pct = battery_percent();
    if (pct >= 100u) {
        out[0] = '1';
        out[1] = '0';
        out[2] = '0';
        out[3] = '%';
        out[4] = 0;
    } else if (pct >= 10u) {
        out[0] = (char)('0' + pct / 10u);
        out[1] = (char)('0' + pct % 10u);
        out[2] = '%';
        out[3] = 0;
    } else {
        out[0] = (char)('0' + pct);
        out[1] = '%';
        out[2] = 0;
    }
}

static void format_u8_2(uint8_t value, char out[3]) {
    if (value > 99u) {
        value = 99u;
    }
    out[0] = (char)('0' + value / 10u);
    out[1] = (char)('0' + value % 10u);
    out[2] = 0;
}

static void format_u16_3(uint16_t value, char out[4]) {
    if (value > 999u) {
        value = 999u;
    }
    out[0] = (char)('0' + value / 100u);
    out[1] = (char)('0' + (value / 10u) % 10u);
    out[2] = (char)('0' + value % 10u);
    out[3] = 0;
}

static void format_battery_voltage(char out[6]) {
    uint16_t mv = battery_millivolts();
    uint16_t whole = (uint16_t)(mv / 1000u);
    uint16_t frac = (uint16_t)((mv % 1000u + 5u) / 10u);
    if (frac >= 100u) {
        whole = (uint16_t)(whole + 1u);
        frac = 0;
    }

    out[0] = (char)('0' + whole);
    out[1] = '.';
    out[2] = (char)('0' + frac / 10u);
    out[3] = (char)('0' + frac % 10u);
    out[4] = 'V';
    out[5] = 0;
}

static uint32_t gen_freq_unit_multiplier(uint8_t unit) {
    if (unit == 2u) {
        return 1000000u;
    }
    if (unit == 1u) {
        return 1000u;
    }
    return 1u;
}

static uint16_t gen_freq_unit_max_value(uint8_t unit) {
    uint32_t max_value = GEN_MAX_FREQ_HZ / gen_freq_unit_multiplier(unit);

    if (max_value < 1u) {
        return 1u;
    }
    if (max_value > 9999u) {
        return 9999u;
    }
    return (uint16_t)max_value;
}

static uint16_t gen_freq_editor_value(void) {
    uint32_t multiplier = gen_freq_unit_multiplier(gen_freq_unit);
    uint32_t value = ui.gen_freq_hz / multiplier;
    uint16_t max_value = gen_freq_unit_max_value(gen_freq_unit);

    if (value < 1u) {
        value = 1u;
    } else if (value > max_value) {
        value = max_value;
    }
    return (uint16_t)value;
}

static void gen_freq_set_editor_value(uint16_t value) {
    uint16_t max_value = gen_freq_unit_max_value(gen_freq_unit);
    uint32_t hz;

    if (value < 1u) {
        value = 1u;
    } else if (value > max_value) {
        value = max_value;
    }
    hz = (uint32_t)value * gen_freq_unit_multiplier(gen_freq_unit);
    if (hz < GEN_MIN_FREQ_HZ) {
        hz = GEN_MIN_FREQ_HZ;
    } else if (hz > GEN_MAX_FREQ_HZ) {
        hz = GEN_MAX_FREQ_HZ;
    }
    ui.gen_freq_hz = hz;
}

static void gen_freq_step_editor_unit(int8_t dir) {
    uint16_t value = gen_freq_editor_value();

    if (dir > 0) {
        if (gen_freq_unit + 1u < GEN_FREQ_UNIT_COUNT) {
            ++gen_freq_unit;
        }
    } else if (gen_freq_unit > 0u) {
        --gen_freq_unit;
    }
    gen_freq_set_editor_value(value);
}

static void gen_freq_sync_editor_unit(void) {
    if (gen_freq_unit >= GEN_FREQ_UNIT_COUNT) {
        gen_freq_unit = 0;
    }
    while (gen_freq_unit + 1u < GEN_FREQ_UNIT_COUNT &&
           ui.gen_freq_hz > (uint32_t)gen_freq_unit_max_value(gen_freq_unit) *
                            gen_freq_unit_multiplier(gen_freq_unit)) {
        ++gen_freq_unit;
    }
}

static void format_gen_freq_digits(char out[5]) {
    uint16_t value = gen_freq_editor_value();

    out[0] = (char)('0' + (value / 1000u) % 10u);
    out[1] = (char)('0' + (value / 100u) % 10u);
    out[2] = (char)('0' + (value / 10u) % 10u);
    out[3] = (char)('0' + value % 10u);
    out[4] = 0;
}

static void gen_freq_draw_unit(uint16_t x, uint16_t y, uint8_t unit, uint16_t fg, uint16_t bg, uint8_t selected) {
    uint16_t text = selected ? C_TEXT : fg;

    if (unit > 2u) {
        unit = 0;
    }
    if (unit == 0u) {
        lcd_text(x, (uint16_t)(y + 4u), "HZ", text, bg, 1);
        if (selected) {
            lcd_rect(x, (uint16_t)(y + 14u), 12, 2, C_TEXT);
        }
    } else {
        char prefix[2] = {unit == 1u ? 'K' : 'M', 0};
        lcd_text((uint16_t)(x + 3u), y, prefix, text, bg, 1);
        lcd_text(x, (uint16_t)(y + 10u), "HZ", text, bg, 1);
        if (selected) {
            lcd_rect(x, (uint16_t)(y + 20u), 12, 2, C_TEXT);
        }
    }
}

static uint8_t battery_display_full(void) {
    return battery_percent() >= 100u;
}

static uint8_t battery_charge_indicator_visible(void) {
    return battery_is_charging();
}

static uint8_t battery_low_alert(void) {
    (void)BATTERY_LOW_PCT;
    return 0;
}

static uint16_t battery_alert_color(uint16_t normal) {
    if (!battery_low_alert()) {
        return normal;
    }
    return ui.battery_blink_on ? C_WARN : C_MUTED;
}

static uint16_t battery_voltage_bucket(void) {
    return (uint16_t)((battery_millivolts() + 25u) / 50u);
}

static void battery_snapshot_store(void) {
    battery_snapshot.valid = 1;
    battery_snapshot.pct = battery_percent();
    battery_snapshot.charging = battery_is_charging();
    battery_snapshot.full = battery_display_full();
    battery_snapshot.voltage_bucket = battery_voltage_bucket();
}

static uint8_t battery_snapshot_changed(void) {
    uint8_t pct = battery_percent();
    uint8_t charging = battery_is_charging();
    uint8_t full = battery_display_full();
    uint16_t voltage_bucket = battery_voltage_bucket();

    if (!battery_snapshot.valid ||
        pct != battery_snapshot.pct ||
        charging != battery_snapshot.charging ||
        full != battery_snapshot.full ||
        voltage_bucket != battery_snapshot.voltage_bucket) {
        battery_snapshot.valid = 1;
        battery_snapshot.pct = pct;
        battery_snapshot.charging = charging;
        battery_snapshot.full = full;
        battery_snapshot.voltage_bucket = voltage_bucket;
        return 1;
    }
    return 0;
}

static void draw_charge_bolt(uint16_t x, uint16_t y, uint16_t color) {
    static const uint8_t row_x[12] = {3, 3, 2, 2, 1, 1, 0, 3, 2, 2, 1, 0};
    static const uint8_t row_w[12] = {4, 4, 4, 4, 4, 7, 8, 5, 4, 4, 3, 2};

    for (uint8_t row = 0; row < 12u; ++row) {
        lcd_rect((uint16_t)(x + row_x[row]), (uint16_t)(y + row), row_w[row], 1, color);
    }
}

static void draw_battery(uint16_t x, uint16_t y, uint16_t bg) {
    uint8_t pct = battery_percent();
    uint8_t low = battery_low_alert();
    uint8_t alert_on = (uint8_t)(!low || ui.battery_blink_on);
    uint16_t fill = battery_display_full() ? 20u : (uint16_t)((uint32_t)pct * 20u / 100u);
    uint16_t frame_color = low ? (alert_on ? C_WARN : C_MUTED) : C_BATT;
    uint16_t fill_color = battery_display_full() ? C_BATT_FULL : (low ? C_WARN : C_BATT);

    lcd_rect(x, (uint16_t)(y - 5u), 40, 22, bg);
    lcd_frame(x, y, 24, 10, frame_color);
    lcd_rect((uint16_t)(x + 24), (uint16_t)(y + 3), 2, 4, frame_color);
    if (fill && alert_on) {
        lcd_rect((uint16_t)(x + 2), (uint16_t)(y + 2), fill, 6, fill_color);
    }
    if (battery_charge_indicator_visible()) {
        draw_charge_bolt((uint16_t)(x + 30u), y, C_BATT_FULL);
    }
}

static void draw_battery_status_header(void) {
    char pct[5];
    char volts[6];

    format_percent(pct);
    format_battery_voltage(volts);

    lcd_rect(252, 4, 33, 21, C_TOP);
    lcd_text(254, 5, pct, battery_alert_color(C_TEXT), C_TOP, 1);
    lcd_text(254, 16, volts, battery_alert_color(C_MUTED), C_TOP, 1);
    draw_battery(280, 7, C_TOP);
}

static void draw_battery_status_overlay(uint16_t bg) {
    char pct[5];

    format_percent(pct);
    lcd_rect(252, 5, 68, 15, bg);
    lcd_text(254, 9, pct, battery_alert_color(C_TEXT), bg, 1);
    draw_battery(280, 7, bg);
}

static void draw_header(void) {
    uint16_t accent = mode_accent();
    lcd_rect(0, Y_HEADER, LCD_WIDTH, H_HEADER, C_TOP);
    lcd_rect(0, (uint16_t)(H_HEADER - 2), LCD_WIDTH, 2, accent);
    lcd_text(8, 6, mode_title(), C_TEXT, C_TOP, 2);

    draw_battery_status_header();
}

static void draw_micro_status(uint16_t x, uint16_t y, uint16_t bg) {
    uint16_t accent = mode_accent();
    lcd_rect(x, y, 118, 13, bg);
    lcd_rect(x, y, 3, 13, accent);
    lcd_text((uint16_t)(x + 7), (uint16_t)(y + 4), mode_title(), C_TEXT, bg, 1);
}

static const char *fw_update_status_text(const fw_update_status_t *status) {
    if (status->state == FW_UPDATE_STATE_STAGING) {
        return "RECEIVING";
    }
    if (status->state == FW_UPDATE_STATE_READY) {
        return "READY TO FLASH";
    }
    if (status->state == FW_UPDATE_STATE_APPLYING) {
        return "FLASHING";
    }
    if (status->state == FW_UPDATE_STATE_ERROR) {
        if (status->error == FW_UPDATE_ERR_VECTOR) {
            return "NOT A FIRMWARE FILE";
        }
        if (status->error == FW_UPDATE_ERR_RANGE) {
            return "FILE TOO LARGE";
        }
        return "UPDATE ERROR";
    }
    return "";
}

static void draw_fw_update_overlay(uint16_t bg) {
    fw_update_status_t status;
    char bytes[11];
    char line[32];
    uint16_t x = 36;
    uint16_t y = 82;
    uint16_t w = 248;
    uint16_t h = 72;
    uint16_t box_bg;
    uint16_t box_fg;

    fw_update_status(&status);
    if (status.state == FW_UPDATE_STATE_IDLE) {
        return;
    }

    box_bg = status.state == FW_UPDATE_STATE_ERROR ? RGB565(64, 24, 28) : C_TEXT;
    box_fg = status.state == FW_UPDATE_STATE_ERROR ? C_TEXT : C_BG;
    lcd_rect(x, y, w, h, box_bg);
    lcd_frame(x, y, w, h, status.state == FW_UPDATE_STATE_ERROR ? C_WARN : C_PANEL_2);
    lcd_text_center(x, (uint16_t)(y + 9u), w, "FIRMWARE UPDATE", box_fg, box_bg, 2);
    lcd_text_center(x, (uint16_t)(y + 34u), w, fw_update_status_text(&status), box_fg, box_bg, 1);

    scope_format_u32(bytes, status.bytes / 1024u);
    line[0] = 0;
    ui_text_append(line, bytes, sizeof(line));
    ui_text_append(line, " KB", sizeof(line));
    lcd_text_center(x, (uint16_t)(y + 50u), w, line, box_fg, box_bg, 1);

    (void)bg;
}

static void draw_screenshot_overlay(uint16_t bg) {
    uint16_t x = 46;
    uint16_t y = 86;
    uint16_t w = 228;
    uint16_t h = 66;
    uint16_t box_bg;
    uint16_t box_fg;
    uint16_t accent;

    if (!ui.screenshot_overlay_ms) {
        return;
    }

    if (ui.screenshot_state == SCREENSHOT_STATE_SAVING) {
        box_bg = C_TEXT;
        box_fg = C_BG;
        accent = C_PANEL_2;
    } else if (ui.screenshot_state == SCREENSHOT_STATE_OK) {
        box_bg = C_TEXT;
        box_fg = C_BG;
        accent = C_DMM;
    } else {
        box_bg = RGB565(64, 24, 28);
        box_fg = C_TEXT;
        accent = C_WARN;
    }
    lcd_rect(x, y, w, h, box_bg);
    lcd_frame(x, y, w, h, accent);
    lcd_rect(x, y, w, 4, accent);
    lcd_text_center(x, (uint16_t)(y + 12u), w, "SCREENSHOT", box_fg, box_bg, 2);
    if (ui.screenshot_state == SCREENSHOT_STATE_SAVING) {
        lcd_text_center(x, (uint16_t)(y + 38u), w, "SAVING TO USB", box_fg, box_bg, 1);
    } else if (ui.screenshot_state == SCREENSHOT_STATE_OK) {
        lcd_text_center(x, (uint16_t)(y + 38u), w, "SAVED TO USB", box_fg, box_bg, 1);
    } else {
        lcd_text_center(x, (uint16_t)(y + 38u), w, "SAVE ERROR", box_fg, box_bg, 1);
    }
    (void)bg;
}

static void draw_mode_tabs(void) {
    uint16_t x = 10;
    for (uint8_t i = 0; i < (uint8_t)UI_MODE_COUNT; ++i) {
        uint16_t bg = i == (uint8_t)ui.mode ? mode_accent() : C_PANEL_2;
        uint16_t fg = i == (uint8_t)ui.mode ? C_BG : C_MUTED;
        lcd_rect(x, 32, 46, 16, bg);
        lcd_text_center(x, 36, 46, mode_short((ui_mode_t)i), fg, bg, 1);
        x = (uint16_t)(x + 50);
    }
}

static uint8_t dmm_group_first(uint8_t mode) {
    if (mode >= DMM_MODE_AC_HI_CURR) {
        return DMM_MODE_AC_HI_CURR;
    }
    if (mode == DMM_MODE_DIODE || mode == DMM_MODE_CAP) {
        return DMM_MODE_DIODE;
    }
    if (mode >= DMM_MODE_LIVE) {
        return DMM_MODE_LIVE;
    }
    return DMM_MODE_DCV;
}

static uint8_t dmm_group_count(uint8_t first) {
    if (first == DMM_MODE_AC_HI_CURR) {
        return 4;
    }
    if (first == DMM_MODE_LIVE) {
        return 2;
    }
    if (first == DMM_MODE_DIODE) {
        return 2;
    }
    return 3;
}

static uint8_t scope_safe_display(void) {
    return ui.scope_display < SCOPE_DISPLAY_COUNT ? ui.scope_display : SCOPE_DISPLAY_YT;
}

static uint8_t scope_safe_cursor(void) {
    return ui.scope_cursor_mode < SCOPE_CURSOR_COUNT ? ui.scope_cursor_mode : SCOPE_CURSOR_OFF;
}

static uint8_t scope_safe_timebase(void) {
    return ui.scope_timebase < SCOPE_TIMEBASE_COUNT ? ui.scope_timebase : SCOPE_TIMEBASE_DEFAULT;
}

static uint8_t scope_slow_roll_active(void) {
    return ui.scope_display == SCOPE_DISPLAY_YT &&
           scope_safe_timebase() >= SCOPE_SLOW_ROLL_TIMEBASE_START;
}

static uint8_t scope_soft_roll_active(void) {
    return ui.scope_display == SCOPE_DISPLAY_YT &&
           scope_safe_timebase() == SCOPE_SOFT_ROLL_TIMEBASE;
}

#if !SCOPE_UI_SAFE_STUB
static uint8_t scope_irq_roll_active(void) {
    return ui.scope_display == SCOPE_DISPLAY_YT &&
           scope_safe_timebase() >= SCOPE_IRQ_ROLL_TIMEBASE_START;
}
#endif

static void scope_slow_roll_reset(void) {
    scope_slow_roll_count = 0;
    scope_slow_roll_head = 0;
    scope_slow_roll_timebase = scope_safe_timebase();
    scope_slow_roll_seq = 0xFFFFu;
    scope_roll_trigger_pending = 0;
    scope_roll_trigger_seq = 0;
    scope_roll_trigger_target_seq = 0;
    scope_roll_display_offset = 0;
    for (uint16_t i = 0; i < SCOPE_SLOW_ROLL_MAX_POINTS; ++i) {
        scope_slow_roll_raw[0][i] = 128;
        scope_slow_roll_raw[1][i] = 128;
    }
}

static uint16_t scope_slow_roll_physical_index(uint16_t logical_index) {
    uint16_t index;

    if (scope_slow_roll_count < SCOPE_SLOW_ROLL_MAX_POINTS) {
        return logical_index;
    }
    index = (uint16_t)(scope_slow_roll_head + logical_index);
    return index >= SCOPE_SLOW_ROLL_MAX_POINTS ?
        (uint16_t)(index - SCOPE_SLOW_ROLL_MAX_POINTS) : index;
}

static uint8_t scope_slow_roll_raw_at(uint8_t ch, uint16_t logical_index) {
    return scope_slow_roll_raw[ch][scope_slow_roll_physical_index(logical_index)];
}

static void scope_roll_trigger_reset(void) {
    scope_roll_trigger_pending = 0;
    scope_roll_trigger_seq = 0;
    scope_roll_trigger_target_seq = 0;
    scope_roll_display_offset = 0;
}

static uint8_t scope_safe_vdiv(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;

    return scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
}

static uint8_t scope_channel_range_index(uint8_t idx) {
    if (idx >= SETTINGS_SCOPE_CHANNEL_COUNT) {
        idx = 0;
    }
    return scope_vdiv_ch[idx] < SETTINGS_SCOPE_RANGE_COUNT ? scope_vdiv_ch[idx] : 0u;
}

#if !SCOPE_UI_SAFE_STUB
static uint16_t scope_bias_dac_for_channel(uint8_t idx) {
    uint8_t range = scope_channel_range_index(idx);
    int32_t dac = (int32_t)ui_settings.scope_bias[idx][range] +
                  ((int32_t)ui_settings.scope_bias_rate[idx][range] * scope_zero_offset[idx]) / 100;

    if (dac < 0) {
        return 0;
    }
    if (dac > 4095) {
        return 4095;
    }
    return (uint16_t)dac;
}
#endif

static uint8_t scope_param_in_order(const uint8_t *order, uint8_t count, uint8_t param) {
    for (uint8_t i = 0; i < count; ++i) {
        if (order[i] == param) {
            return 1;
        }
    }
    return 0;
}

static uint8_t scope_param_is_channel(uint8_t param) {
    return scope_param_in_order(scope_channel_param_order,
                                (uint8_t)sizeof(scope_channel_param_order),
                                param);
}

static uint8_t scope_param_is_trigger(uint8_t param) {
    return scope_param_in_order(scope_trigger_param_order,
                                (uint8_t)sizeof(scope_trigger_param_order),
                                param);
}

static uint8_t scope_param_is_global(uint8_t param) {
    return scope_param_in_order(scope_global_param_order,
                                (uint8_t)sizeof(scope_global_param_order),
                                param);
}

static uint8_t scope_measure_bit(uint8_t measure) {
    if (measure >= SCOPE_MEASURE_COUNT) {
        measure = SCOPE_MEASURE_VPP;
    }
    return (uint8_t)(1u << measure);
}

static uint8_t scope_measure_valid_mask(void) {
    return (uint8_t)((1u << SCOPE_MEASURE_COUNT) - 1u);
}

static uint8_t scope_measure_enabled_for(uint8_t idx, uint8_t measure) {
    if (idx >= 2u) {
        idx = 0;
    }
    return (scope_measure_mask[idx] & scope_measure_bit(measure)) ? 1u : 0u;
}

static uint8_t scope_trigger_source_index(void) {
    return ui.scope_trigger_source == 2u ? 1u : 0u;
}

static void scope_sanitize_state(void) {
    ui.scope_display = scope_safe_display();
    ui.scope_cursor_mode = scope_safe_cursor();
    ui.scope_timebase = scope_safe_timebase();
    if (ui.scope_cursor_sel > 1u) {
        ui.scope_cursor_sel = 0;
    }
    if (ui.scope_param >= SCOPE_PARAM_COUNT) {
        ui.scope_param = SCOPE_PARAM_POSITION;
    }
    ui.scope_channel_menu = ui.scope_channel_menu ? 1u : 0u;
    ui.scope_trigger_menu = ui.scope_trigger_menu ? 1u : 0u;
    ui.scope_measure_menu = ui.scope_measure_menu ? 1u : 0u;
    ui.scope_cursor_menu = ui.scope_cursor_menu ? 1u : 0u;
    if (ui.scope_cursor_menu_sel < SCOPE_CURSOR_MENU_MODE ||
        ui.scope_cursor_menu_sel > SCOPE_CURSOR_MENU_SECOND) {
        ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
    }
    ui.scope_move_mode = ui.scope_move_mode ? 1u : 0u;
    if (ui.scope_move_sel > SCOPE_MOVE_SEL_ZERO) {
        ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
    }
    if (ui.scope_channel_menu) {
        ui.scope_trigger_menu = 0;
        ui.scope_measure_menu = 0;
        ui.scope_cursor_menu = 0;
        ui.scope_move_mode = 0;
        if (!scope_param_is_channel(ui.scope_param)) {
            ui.scope_param = SCOPE_PARAM_ENABLE;
        }
    } else if (ui.scope_trigger_menu) {
        ui.scope_measure_menu = 0;
        ui.scope_cursor_menu = 0;
        ui.scope_move_mode = 0;
        if (!scope_param_is_trigger(ui.scope_param)) {
            ui.scope_param = SCOPE_PARAM_TRIG_TYPE;
        }
    } else if (ui.scope_measure_menu) {
        ui.scope_cursor_menu = 0;
        ui.scope_move_mode = 0;
        ui.scope_param = SCOPE_PARAM_MEASURE;
        if (ui.scope_measure_menu_sel >= SCOPE_MEASURE_MENU_COUNT) {
            ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VALUE;
        }
    } else if (ui.scope_cursor_menu) {
        ui.scope_move_mode = 0;
        ui.scope_param = SCOPE_PARAM_CURSOR;
    } else if (!scope_param_is_global(ui.scope_param)) {
        ui.scope_param = SCOPE_PARAM_POSITION;
    }
    if (!scope_any_menu_open()) {
        ui.scope_move_menu_ms = 0;
    }
    if (ui.scope_trigger_source < 1u || ui.scope_trigger_source > 2u) {
        ui.scope_trigger_source = 1;
    }
    if (ui.scope_trigger_mode >= SCOPE_TRIGGER_COUNT) {
        ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    }
    if (scope_measure_param >= SCOPE_MEASURE_COUNT) {
        scope_measure_param = SCOPE_MEASURE_VPP;
    }
    scope_measure_mask[0] &= scope_measure_valid_mask();
    scope_measure_mask[1] &= scope_measure_valid_mask();
    if (ui.scope_trigger_level < 12u) {
        ui.scope_trigger_level = 12;
    } else if (ui.scope_trigger_level > 243u) {
        ui.scope_trigger_level = 243;
    }
    ui.scope_trigger_edge = ui.scope_trigger_edge ? 1u : 0u;
    if (ui.active_ch < 1u || ui.active_ch > 2u) {
        ui.active_ch = 1;
    }
    for (uint8_t i = 0; i < 2u; ++i) {
        scope_ch_enabled[i] = scope_ch_enabled[i] ? 1u : 0u;
        scope_probe_x10[i] = scope_probe_x10[i] ? 1u : 0u;
        if (scope_vdiv_ch[i] >= SCOPE_VDIV_COUNT) {
            scope_vdiv_ch[i] = 0;
        }
        if (scope_cursor_time_value[i] < -128) {
            scope_cursor_time_value[i] = -128;
        } else if (scope_cursor_time_value[i] > 127) {
            scope_cursor_time_value[i] = 127;
        }
        if (scope_cursor_timebase[i] >= SCOPE_TIMEBASE_COUNT) {
            scope_cursor_timebase[i] = scope_safe_timebase();
        }
        scope_coupling_dc[i] = scope_coupling_dc[i] ? 1u : 0u;
        if (scope_zero_offset[i] < -100) {
            scope_zero_offset[i] = -100;
        } else if (scope_zero_offset[i] > 100) {
            scope_zero_offset[i] = 100;
        }
    }
    ui.scope_vdiv = scope_safe_vdiv();
    for (uint8_t i = 0; i < 2u; ++i) {
        scope_ch_pos[i] = scope_clamp_channel_pos(i, scope_ch_pos[i]);
    }
    if (scope_h_pos < -SCOPE_H_POS_LIMIT) {
        scope_h_pos = -SCOPE_H_POS_LIMIT;
    } else if (scope_h_pos > SCOPE_H_POS_LIMIT) {
        scope_h_pos = SCOPE_H_POS_LIMIT;
    }
    if (scope_h_value_pos < -SCOPE_H_POS_LIMIT) {
        scope_h_value_pos = -SCOPE_H_POS_LIMIT;
    } else if (scope_h_value_pos > SCOPE_H_POS_LIMIT) {
        scope_h_value_pos = SCOPE_H_POS_LIMIT;
    }
    if (scope_h_value_timebase >= SCOPE_TIMEBASE_COUNT) {
        scope_h_value_timebase = scope_safe_timebase();
    }
}

static const char *scope_display_label(void) {
    return scope_display_labels[scope_safe_display()];
}

static const char *scope_cursor_label(void) {
    return scope_cursor_labels[scope_safe_cursor()];
}

static const char *scope_cursor_chip_label(void) {
    return scope_cursor_short[scope_safe_cursor()];
}

static const char *scope_timebase_label(void) {
    return scope_timebases[scope_safe_timebase()];
}

static const char *scope_timebase_chip_label(void) {
    return scope_timebases_short[scope_safe_timebase()];
}

static const char *scope_vdiv_chip_label(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    uint8_t vdiv = scope_safe_vdiv();

    return scope_probe_x10[idx] ? scope_vdivs_x10_short[vdiv] : scope_vdivs_short[vdiv];
}

static const char *scope_vdiv_chip_label_for_channel(uint8_t idx) {
    uint8_t vdiv;

    if (idx >= 2u) {
        idx = 0;
    }
    vdiv = scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
    return scope_probe_x10[idx] ? scope_vdivs_x10_short[vdiv] : scope_vdivs_short[vdiv];
}

static void draw_scope_scale_status(uint16_t y, uint16_t bg) {
    char ch1[10];
    char ch2[10];

    ch1[0] = 'C';
    ch1[1] = 'H';
    ch1[2] = '1';
    ch1[3] = ' ';
    scope_text_copy(&ch1[4], scope_vdiv_chip_label_for_channel(0), 6);
    ch2[0] = 'C';
    ch2[1] = 'H';
    ch2[2] = '2';
    ch2[3] = ' ';
    scope_text_copy(&ch2[4], scope_vdiv_chip_label_for_channel(1), 6);

    lcd_text(0, y, ch1, C_CH1, bg, 2);
    lcd_text_center(96, y, 72, scope_timebase_label(), C_TEXT, bg, 2);
    lcd_text_center(168, y, 56, ui.running ? (scope_slow_roll_active() ? "ROLL" : "RUN") : "HOLD",
                    ui.running ? C_DMM : C_WARN, bg, 2);
    lcd_text(224, y, ch2, C_CH2, bg, 2);
}

static const char *scope_channel_enable_label(void) {
    return scope_ch_enabled[ui.active_ch == 2u ? 1u : 0u] ? "ON" : "OFF";
}

static const char *scope_param_label(uint8_t param) {
    if (param == SCOPE_PARAM_ENABLE && ui.scope_channel_menu) {
        return ui.active_ch == 2u ? "CH2" : "CH1";
    }
    if (param == SCOPE_PARAM_POSITION && !ui.scope_channel_menu) {
        return "MOVE";
    }
    return scope_param_labels[param];
}

static const char *scope_probe_label(void) {
    return scope_probe_x10[ui.active_ch == 2u ? 1u : 0u] ? "X10" : "X1";
}

static const char *scope_coupling_label(void) {
    return scope_coupling_dc[ui.active_ch == 2u ? 1u : 0u] ? "DC" : "AC";
}

static const char *scope_zero_label(void) {
    static char label[5];
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int16_t units = scope_zero_offset[idx];
    uint8_t mag = (uint8_t)(units < 0 ? -units : units);
    uint8_t pos = 0;

    if (scope_auto_zero_active) {
        return "CAL";
    }
    label[pos++] = units < 0 ? '-' : '+';
    if (mag >= 100u) {
        label[pos++] = (char)('0' + mag / 100u);
        mag %= 100u;
        label[pos++] = (char)('0' + mag / 10u);
    } else if (mag >= 10u) {
        label[pos++] = (char)('0' + mag / 10u);
    }
    label[pos++] = (char)('0' + mag % 10u);
    label[pos] = 0;
    return label;
}

static const char *scope_position_label(void) {
    static char label[5];
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int8_t pos = scope_ch_pos[idx];
    uint8_t mag = (uint8_t)(pos < 0 ? -pos : pos);
    uint8_t out = 0;

    label[out++] = pos < 0 ? '-' : '+';
    if (mag >= 10u) {
        label[out++] = (char)('0' + mag / 10u);
    }
    label[out++] = (char)('0' + mag % 10u);
    label[out] = 0;
    return label;
}

static const char *scope_trigger_chip_label(void) {
    uint8_t mode = ui.scope_trigger_mode < SCOPE_TRIGGER_COUNT ? ui.scope_trigger_mode : SCOPE_TRIGGER_AUTO;

    return scope_trigger_short[mode];
}

static const char *scope_trigger_source_label(void) {
    return ui.scope_trigger_source == 2u ? "CH2" : "CH1";
}

static const char *scope_trigger_level_label(void) {
    static char label[11];
    uint8_t idx = scope_trigger_source_index();
    int32_t mv = ((int32_t)ui.scope_trigger_level - 128) *
                 (int32_t)scope_vdiv_mv_for_channel(idx) * 8 / 255;

    scope_format_signed_mv(label, mv);
    return label;
}

static const char *scope_trigger_edge_label(void) {
    return ui.scope_trigger_edge ? "FALL" : "RISE";
}

static void draw_softkey(uint8_t slot, const char *label, uint16_t accent) {
    uint16_t x = (uint16_t)(slot * 80u);
    lcd_rect(x, Y_SOFT, 79, H_SOFT, C_TOP);
    lcd_rect(x, Y_SOFT, 79, 2, accent);
    lcd_text_center(x, (uint16_t)(Y_SOFT + 16), 79, label, C_TEXT, C_TOP, 1);
}

static uint8_t dmm_group_selected(uint8_t group) {
    if (group == 0u) {
        return ui.dmm_mode == DMM_MODE_DCV || ui.dmm_mode == DMM_MODE_ACV || ui.dmm_mode == DMM_MODE_RES;
    }
    if (group == 1u) {
        return ui.dmm_mode == DMM_MODE_DIODE || ui.dmm_mode == DMM_MODE_CAP;
    }
    if (group == 2u) {
        return ui.dmm_mode == DMM_MODE_LIVE || ui.dmm_mode == DMM_MODE_TEMP;
    }
    return ui.dmm_mode >= DMM_MODE_AC_HI_CURR;
}

static const char *dmm_group_value(uint8_t group) {
    if (dmm_group_selected(group)) {
        return dmm_mode_short[ui.dmm_mode];
    }
    if (group == 0u) {
        return "DCV ACV RES";
    }
    if (group == 1u) {
        return "DIODE CAP";
    }
    if (group == 2u) {
        return "LIVE TEMP";
    }
    return "DC AC";
}

static void draw_dmm_selected_group_value(uint16_t x, uint16_t y, uint16_t w, const char *value, uint16_t bg) {
    uint16_t text_w = lcd_text_width(value, 2);
    uint16_t text_x = (uint16_t)(x + (w > text_w ? (w - text_w) / 2u : 0u));
    uint16_t underline_w = text_w;
    uint16_t underline_x;

    if (underline_w > w - 12u) {
        underline_w = (uint16_t)(w - 12u);
    }
    underline_x = (uint16_t)(x + (w - underline_w) / 2u);
    lcd_text(text_x, (uint16_t)(y + 13u), value, C_TEXT, bg, 2);
    lcd_rect(underline_x, (uint16_t)(y + 29u), underline_w, 2, C_TEXT);
}

static void draw_dmm_group_chip(uint16_t x, uint16_t y, uint16_t w, uint8_t group, const char *label) {
    uint8_t active = dmm_group_selected(group);
    uint16_t bg = active ? C_DMM : C_PANEL_2;
    uint16_t fg = active ? C_BG : C_MUTED;

    lcd_rect(x, y, w, 32, bg);
    lcd_rect(x, y, w, 2, C_DMM);
    lcd_text((uint16_t)(x + 6u), (uint16_t)(y + 4u), label, fg, bg, 1);
    if (active) {
        draw_dmm_selected_group_value(x, y, w, dmm_group_value(group), bg);
    } else if (group == 3u) {
        lcd_text_center(x, (uint16_t)(y + 13u), w, "HI LO", fg, bg, 1);
        lcd_text_center(x, (uint16_t)(y + 23u), w, "DC AC", fg, bg, 1);
    } else {
        lcd_text_center(x, (uint16_t)(y + 18u), w, dmm_group_value(group), fg, bg, 1);
    }
}

static void draw_dmm_softkeys(void) {
    lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, C_BG);
    draw_dmm_group_chip(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u), UI_PARAM_CHIP_W, 0, "V/OHM");
    draw_dmm_group_chip((uint16_t)(UI_PARAM_ROW_X + UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP),
                        (uint16_t)(Y_SOFT + 4u),
                        UI_PARAM_CHIP_W,
                        1,
                        "TEST");
    draw_dmm_group_chip((uint16_t)(UI_PARAM_ROW_X + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 2u),
                        (uint16_t)(Y_SOFT + 4u),
                        UI_PARAM_CHIP_W,
                        2,
                        "SENSE");
    draw_dmm_group_chip((uint16_t)(UI_PARAM_ROW_X + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 3u),
                        (uint16_t)(Y_SOFT + 4u),
                        UI_PARAM_CHIP_W,
                        3,
                        "CURRENT");
}

static const char *scope_measure_chip_label(void) {
    scope_sanitize_state();
    if (!scope_measure_visible) {
        return "OFF";
    }
    return scope_measure_labels[scope_measure_param];
}

static const char *scope_param_value(uint8_t param) {
    if (param == SCOPE_PARAM_ENABLE) {
        return scope_channel_enable_label();
    }
    if (param == SCOPE_PARAM_PROBE) {
        return scope_probe_label();
    }
    if (param == SCOPE_PARAM_GAIN) {
        return scope_vdiv_chip_label();
    }
    if (param == SCOPE_PARAM_COUPLING) {
        return scope_coupling_label();
    }
    if (param == SCOPE_PARAM_ZERO) {
        return scope_zero_label();
    }
    if (param == SCOPE_PARAM_POSITION) {
        return scope_position_label();
    }
    if (param == SCOPE_PARAM_TIME) {
        return scope_timebase_chip_label();
    }
    if (param == SCOPE_PARAM_TRIGGER) {
        return scope_trigger_chip_label();
    }
    if (param == SCOPE_PARAM_CURSOR) {
        return scope_cursor_chip_label();
    }
    if (param == SCOPE_PARAM_TRIG_SOURCE) {
        return scope_trigger_source_label();
    }
    if (param == SCOPE_PARAM_TRIG_LEVEL) {
        return scope_trigger_level_label();
    }
    if (param == SCOPE_PARAM_TRIG_TYPE) {
        return scope_trigger_chip_label();
    }
    if (param == SCOPE_PARAM_TRIG_EDGE) {
        return scope_trigger_edge_label();
    }
    if (param == SCOPE_PARAM_MEASURE) {
        return scope_measure_chip_label();
    }
    return scope_display_short[scope_safe_display()];
}

static uint16_t scope_menu_accent(void) {
    if (ui.scope_channel_menu) {
        return scope_channel_color(ui.active_ch);
    }
    if (ui.scope_trigger_menu) {
        return scope_channel_color(ui.scope_trigger_source);
    }
    return C_SCOPE;
}

static void draw_scope_param_chip(uint16_t x, uint16_t y, uint16_t w, uint8_t param) {
    uint8_t active = ui.scope_param == param;
    uint16_t accent = scope_menu_accent();
    uint16_t chip_bg = active ? accent : C_PANEL_2;
    uint16_t fg = active ? C_BG : C_MUTED;
    const char *value = scope_param_value(param);
    uint8_t value_scale = lcd_text_width(value, 2) <= w ? 2u : 1u;
    uint16_t value_y = (uint16_t)(y + (value_scale == 2u ? 13u : 18u));

    lcd_rect(x, y, w, 32, chip_bg);
    lcd_rect(x, y, w, 2, accent);
    lcd_text((uint16_t)(x + 6u), (uint16_t)(y + 4u), scope_param_label(param), fg, chip_bg, 1);
    lcd_text_center(x, value_y, w, value, fg, chip_bg, value_scale);
    if (active) {
        uint16_t text_w = lcd_text_width(value, value_scale);
        uint16_t underline_w = text_w;
        if (underline_w > w - 12u) {
            underline_w = (uint16_t)(w - 12u);
        }
        lcd_rect((uint16_t)(x + (w - underline_w) / 2u),
                 (uint16_t)(y + 29u),
                 underline_w,
                 2,
                 fg);
    }
}

static void draw_scope_param_row(uint16_t x, uint16_t y) {
    scope_sanitize_state();
    for (uint8_t i = 0; i < (uint8_t)sizeof(scope_global_param_order); ++i) {
        draw_scope_param_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * i),
                              y,
                              UI_PARAM_CHIP_W,
                              scope_global_param_order[i]);
    }
}

static void draw_scope_channel_param_row(uint16_t x, uint16_t y) {
    scope_sanitize_state();
    for (uint8_t i = 0; i < (uint8_t)sizeof(scope_channel_param_order); ++i) {
        draw_scope_param_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * i),
                              y,
                              UI_PARAM_CHIP_W,
                              scope_channel_param_order[i]);
    }
}

static void draw_scope_trigger_param_row(uint16_t x, uint16_t y) {
    scope_sanitize_state();
    for (uint8_t i = 0; i < (uint8_t)sizeof(scope_trigger_param_order); ++i) {
        draw_scope_param_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * i),
                              y,
                              UI_PARAM_CHIP_W,
                              scope_trigger_param_order[i]);
    }
}

static const char *scope_measure_channel_state_label(uint8_t idx) {
    return scope_measure_enabled_for(idx, scope_measure_param) ? "ON" : "OFF";
}

static const char *scope_measure_visible_label(void) {
    return scope_measure_visible ? "ON" : "OFF";
}

static void draw_scope_measure_chip(uint16_t x,
                                    uint16_t y,
                                    uint16_t w,
                                    const char *label,
                                    const char *value,
                                    uint16_t accent,
                                    uint8_t active,
                                    uint8_t selected) {
    uint16_t bg = active ? accent : C_PANEL_2;
    uint16_t fg = active ? C_BG : C_MUTED;
    uint8_t value_scale = lcd_text_width(value, 2) <= w ? 2u : 1u;
    uint16_t value_y = (uint16_t)(y + (value_scale == 2u ? 13u : 18u));

    lcd_rect(x, y, w, 32, bg);
    lcd_rect(x, y, w, 2, accent);
    lcd_text((uint16_t)(x + 6u), (uint16_t)(y + 4u), label, fg, bg, 1);
    lcd_text_center(x, value_y, w, value, fg, bg, value_scale);
    if (selected) {
        uint16_t text_w = lcd_text_width(value, value_scale);
        uint16_t underline_w = text_w > (uint16_t)(w - 12u) ? (uint16_t)(w - 12u) : text_w;
        lcd_rect((uint16_t)(x + (w - underline_w) / 2u),
                 (uint16_t)(y + 29u),
                 underline_w,
                 2,
                 active ? fg : accent);
    }
}

static void draw_scope_measure_param_row(uint16_t x, uint16_t y) {
    scope_sanitize_state();
    draw_scope_measure_chip(x,
                            y,
                            UI_PARAM_CHIP_W,
                            "CH1",
                            scope_measure_channel_state_label(0),
                            C_CH1,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH1,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH1);
    draw_scope_measure_chip((uint16_t)(x + UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP),
                            y,
                            UI_PARAM_CHIP_W,
                            "VALUE",
                            scope_measure_labels[scope_measure_param],
                            C_SCOPE,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VALUE,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VALUE);
    draw_scope_measure_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 2u),
                            y,
                            UI_PARAM_CHIP_W,
                            "CH2",
                            scope_measure_channel_state_label(1),
                            C_CH2,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH2,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH2);
    draw_scope_measure_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 3u),
                            y,
                            UI_PARAM_CHIP_W,
                            "MEAS",
                            scope_measure_visible_label(),
                            C_SCOPE,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VISIBLE,
                            ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VISIBLE);
}

static uint16_t scope_visible_sample_count(void) {
    uint8_t timebase = scope_safe_timebase();

    if (timebase <= 3u) {
        uint32_t screen_ns = scope_timebase_unit_ns[timebase] * SCOPE_X_DIVS * 10u;
        uint32_t count = (screen_ns + SCOPE_FAST_HW_SAMPLE_NS / 2u) / SCOPE_FAST_HW_SAMPLE_NS;
        if (count < 2u) {
            count = 2u;
        }
        if (count > SCOPE_VISIBLE_SAMPLE_COUNT) {
            count = SCOPE_VISIBLE_SAMPLE_COUNT;
        }
        return (uint16_t)count;
    }
    return SCOPE_VISIBLE_SAMPLE_COUNT;
}

static int32_t scope_h_pos_sample_offset(void) {
    uint16_t visible_samples = scope_visible_sample_count();
    int32_t numerator = (int32_t)scope_h_pos * (int32_t)(visible_samples / 2u);

    if (numerator >= 0) {
        numerator += SCOPE_H_POS_LIMIT / 2;
    } else {
        numerator -= SCOPE_H_POS_LIMIT / 2;
    }
    return numerator / SCOPE_H_POS_LIMIT;
}

static uint16_t scope_trigger_screen_sample_pos(void) {
    uint16_t visible_samples = scope_visible_sample_count();
    int32_t pos = (int32_t)(visible_samples / 2u) + scope_h_pos_sample_offset();

    if (pos < 0) {
        return 0;
    }
    if (pos >= (int32_t)visible_samples) {
        return (uint16_t)(visible_samples - 1u);
    }
    return (uint16_t)pos;
}

static uint8_t scope_append_text(char *out, uint8_t pos, uint8_t max_len, const char *text) {
    while (*text && pos < (uint8_t)(max_len - 1u)) {
        out[pos++] = *text++;
    }
    out[pos] = 0;
    return pos;
}

static const char *scope_h_pos_label(void) {
    static char label[12];
    char number[11];
    int8_t pos = scope_h_value_pos;
    uint32_t mag = (uint32_t)(pos < 0 ? -pos : pos);
    uint32_t unit_ns = scope_timebase_unit_ns[scope_h_value_timebase < SCOPE_TIMEBASE_COUNT ?
                                               scope_h_value_timebase :
                                               scope_safe_timebase()];
    uint8_t out = 0;

    label[out++] = pos < 0 ? '-' : '+';
    if (unit_ns < 100u) {
        uint32_t ns = (mag * 60u * unit_ns + SCOPE_H_POS_LIMIT / 2u) / SCOPE_H_POS_LIMIT;
        if (ns < 1000u) {
            scope_format_u32(number, ns);
            out = scope_append_text(label, out, sizeof(label), number);
            out = scope_append_text(label, out, sizeof(label), "NS");
        } else {
            uint32_t us_x10 = (ns + 50u) / 100u;
            scope_format_u32(number, us_x10 / 10u);
            out = scope_append_text(label, out, sizeof(label), number);
            out = scope_append_text(label, out, sizeof(label), ".");
            if (out < (uint8_t)(sizeof(label) - 1u)) {
                label[out++] = (char)('0' + us_x10 % 10u);
                label[out] = 0;
            }
            out = scope_append_text(label, out, sizeof(label), "US");
        }
    } else if (unit_ns < 100000u) {
        uint32_t us_x10 = (mag * 6u * unit_ns + SCOPE_H_POS_LIMIT * 5u) /
            (SCOPE_H_POS_LIMIT * 10u);
        scope_format_u32(number, us_x10 / 10u);
        out = scope_append_text(label, out, sizeof(label), number);
        out = scope_append_text(label, out, sizeof(label), ".");
        if (out < (uint8_t)(sizeof(label) - 1u)) {
            label[out++] = (char)('0' + us_x10 % 10u);
            label[out] = 0;
        }
        out = scope_append_text(label, out, sizeof(label), "US");
    } else if (unit_ns < 100000000u) {
        uint32_t ms_x10 = (mag * 6u * (unit_ns / 1000u) + SCOPE_H_POS_LIMIT * 5u) /
            (SCOPE_H_POS_LIMIT * 10u);
        scope_format_u32(number, ms_x10 / 10u);
        out = scope_append_text(label, out, sizeof(label), number);
        out = scope_append_text(label, out, sizeof(label), ".");
        if (out < (uint8_t)(sizeof(label) - 1u)) {
            label[out++] = (char)('0' + ms_x10 % 10u);
            label[out] = 0;
        }
        out = scope_append_text(label, out, sizeof(label), "MS");
    } else {
        uint32_t s_x10 = (mag * 6u * (unit_ns / 1000000u) + SCOPE_H_POS_LIMIT * 5u) /
            (SCOPE_H_POS_LIMIT * 10u);
        scope_format_u32(number, s_x10 / 10u);
        out = scope_append_text(label, out, sizeof(label), number);
        out = scope_append_text(label, out, sizeof(label), ".");
        if (out < (uint8_t)(sizeof(label) - 1u)) {
            label[out++] = (char)('0' + s_x10 % 10u);
            label[out] = 0;
        }
        out = scope_append_text(label, out, sizeof(label), "S");
    }
    (void)out;
    return label;
}

static uint16_t scope_visible_grid_height(void) {
    return ui.chrome_visible ? 124u : 190u;
}

static int16_t scope_visible_grid_top_y(void) {
    return ui.chrome_visible ? 55 : 28;
}

static int16_t scope_visible_grid_mid_y(void) {
    return (int16_t)(scope_visible_grid_top_y() + scope_visible_grid_height() / 2u);
}

static int16_t scope_channel_base_center_for(uint8_t idx) {
    if (ui.chrome_visible) {
#if SCOPE_UI_SAFE_STUB
        return idx ? 118 : 91;
#else
        return idx ? 111 : 91;
#endif
    }
#if SCOPE_UI_SAFE_STUB
    return idx ? (int16_t)(28 + 190 / 2 + 34) : (int16_t)(28 + 190 / 2 - 30);
#else
    return idx ? (int16_t)(28 + 190 / 2 + 28) : (int16_t)(28 + 190 / 2 - 28);
#endif
}

static int8_t scope_clamp_channel_pos(uint8_t idx, int16_t pos) {
    int16_t base = scope_channel_base_center_for(idx >= 2u ? 0u : idx);
    int16_t min_pos = (int16_t)(scope_visible_grid_top_y() - base);
    int16_t max_pos = (int16_t)(scope_visible_grid_top_y() +
                                (int16_t)scope_visible_grid_height() - 1 -
                                base);

    if (min_pos < -SCOPE_V_POS_LIMIT) {
        min_pos = -SCOPE_V_POS_LIMIT;
    }
    if (max_pos > SCOPE_V_POS_LIMIT) {
        max_pos = SCOPE_V_POS_LIMIT;
    }
    if (pos < min_pos) {
        pos = min_pos;
    } else if (pos > max_pos) {
        pos = max_pos;
    }
    return (int8_t)pos;
}

static int32_t scope_signed_round_div(int32_t value, int32_t divisor) {
    if (value >= 0) {
        return (value + divisor / 2) / divisor;
    }
    return (value - divisor / 2) / divisor;
}

static int16_t scope_rescale_cursor_value_for_timebase(int16_t value,
                                                       uint32_t old_unit_ns,
                                                       uint32_t new_unit_ns) {
    enum {
        LIMIT = 127,
    };
    uint8_t negative = 0;
    uint16_t mag;
    uint32_t whole;
    uint32_t rem_step;
    uint32_t rem = 0;

    if (!value || !old_unit_ns || !new_unit_ns) {
        return 0;
    }
    if (value < 0) {
        negative = 1;
        mag = (uint16_t)(-value);
    } else {
        mag = (uint16_t)value;
    }
    whole = old_unit_ns / new_unit_ns;
    rem_step = old_unit_ns % new_unit_ns;
    if (whole > (uint32_t)LIMIT / mag) {
        return negative ? (int16_t)-LIMIT : (int16_t)LIMIT;
    }
    whole *= mag;
    for (uint16_t i = 0; i < mag; ++i) {
        rem += rem_step;
        if (rem >= new_unit_ns) {
            rem -= new_unit_ns;
            ++whole;
            if (whole >= LIMIT) {
                return negative ? (int16_t)-LIMIT : (int16_t)LIMIT;
            }
        }
    }
    if (rem >= (new_unit_ns + 1u) / 2u) {
        ++whole;
    }
    if (whole > LIMIT) {
        whole = LIMIT;
    }
    return negative ? (int16_t)-(int16_t)whole : (int16_t)whole;
}

static uint8_t scope_cursor_x_from_value(uint8_t idx) {
    uint8_t timebase;
    int16_t pos;
    int16_t raw;

    idx = idx ? 1u : 0u;
    timebase = scope_cursor_timebase[idx] < SCOPE_TIMEBASE_COUNT ?
        scope_cursor_timebase[idx] :
        scope_safe_timebase();
    pos = scope_rescale_cursor_value_for_timebase(scope_cursor_time_value[idx],
                                                  scope_timebase_unit_ns[timebase],
                                                  scope_timebase_unit_ns[scope_safe_timebase()]);
    raw = (int16_t)(128 + pos);
    if (raw < 0) {
        raw = 0;
    } else if (raw > 255) {
        raw = 255;
    }
    return (uint8_t)raw;
}

static void scope_cursor_time_sync_screen(void) {
    for (uint8_t i = 0; i < 2u; ++i) {
        scope_cursor_x[i] = scope_cursor_x_from_value(i);
    }
}

static void scope_cursor_store_time_from_raw(uint8_t idx) {
    idx = idx ? 1u : 0u;
    scope_cursor_time_value[idx] = (int16_t)scope_cursor_x[idx] - 128;
    scope_cursor_timebase[idx] = scope_safe_timebase();
}

static int32_t scope_cursor_level_mv_from_raw(uint8_t raw) {
    return scope_signed_round_div(((int32_t)128 - (int32_t)raw) *
                                      (int32_t)scope_vdiv_mv() *
                                      (int32_t)SCOPE_Y_DIVS,
                                  255);
}

static uint8_t scope_cursor_y_from_level_mv(int32_t mv) {
    int32_t full_scale = (int32_t)scope_vdiv_mv() * (int32_t)SCOPE_Y_DIVS;
    int32_t raw_delta;
    int32_t raw;

    if (full_scale <= 0) {
        return 128;
    }
    raw_delta = scope_signed_round_div(mv * 255, full_scale);
    if (raw_delta < -127) {
        raw_delta = -127;
    } else if (raw_delta > 128) {
        raw_delta = 128;
    }
    raw = 128 - raw_delta;
    if (raw < 0) {
        raw = 0;
    } else if (raw > 255) {
        raw = 255;
    }
    return (uint8_t)raw;
}

static void scope_cursor_level_sync_screen(void) {
    for (uint8_t i = 0; i < 2u; ++i) {
        scope_cursor_y[i] = scope_cursor_y_from_level_mv(scope_cursor_level_mv[i]);
    }
}

static void scope_cursor_store_level_from_raw(uint8_t idx) {
    idx = idx ? 1u : 0u;
    scope_cursor_level_mv[idx] = scope_cursor_level_mv_from_raw(scope_cursor_y[idx]);
}

static const char *scope_y_pos_label(void) {
    static char label[11];
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int16_t pixels = (int16_t)(scope_visible_grid_mid_y() -
                               scope_channel_base_center_for(idx) -
                               (int16_t)scope_ch_pos[idx]);
    int32_t mv = scope_signed_round_div((int32_t)pixels *
                                             (int32_t)scope_vdiv_mv_for_channel(idx) *
                                             (int32_t)SCOPE_Y_DIVS,
                                         (int32_t)scope_visible_grid_height());

    scope_format_signed_mv(label, mv);
    return label;
}

static void scope_format_cursor_time_pos(char out[12], int16_t value, uint8_t timebase) {
    uint32_t unit_ns = scope_timebase_unit_ns[timebase < SCOPE_TIMEBASE_COUNT ?
                                              timebase :
                                              scope_safe_timebase()];
    char number[11];
    uint8_t n = 0;
    uint8_t negative = value < 0 ? 1u : 0u;
    uint16_t mag = negative ? (uint16_t)(-value) : (uint16_t)value;

    out[n++] = mag ? (negative ? '-' : '+') : '0';
    out[n] = 0;
    if (!mag) {
        return;
    }
    if (unit_ns < 100u) {
        uint32_t ns = (uint32_t)mag * SCOPE_X_DIVS * 10u * unit_ns / 255u;
        if (ns < 1000u) {
            scope_format_u32(number, ns);
            n = scope_append_text(out, n, 12, number);
            n = scope_append_text(out, n, 12, "N");
        } else {
            uint32_t us_x10 = (ns + 50u) / 100u;
            if (us_x10 < 1000u) {
                scope_format_u32(number, us_x10 / 10u);
                n = scope_append_text(out, n, 12, number);
                n = scope_append_text(out, n, 12, ".");
                if (n < 11u) {
                    out[n++] = (char)('0' + us_x10 % 10u);
                    out[n] = 0;
                }
            } else {
                scope_format_u32(number, us_x10 / 10u);
                n = scope_append_text(out, n, 12, number);
            }
            n = scope_append_text(out, n, 12, "U");
        }
    } else if (unit_ns < 100000u) {
        uint32_t us_x10 = ((uint32_t)mag * SCOPE_X_DIVS * unit_ns + 1275u) / 2550u;
        if (us_x10 >= 1000u) {
            scope_format_u32(number, us_x10 / 10u);
            n = scope_append_text(out, n, 12, number);
            n = scope_append_text(out, n, 12, "U");
            (void)n;
            return;
        }
        scope_format_u32(number, us_x10 / 10u);
        n = scope_append_text(out, n, 12, number);
        n = scope_append_text(out, n, 12, ".");
        if (n < 11u) {
                out[n++] = (char)('0' + us_x10 % 10u);
                out[n] = 0;
        }
        n = scope_append_text(out, n, 12, "U");
    } else if (unit_ns < 10000000u) {
        uint32_t ms_x10 = ((uint32_t)mag * SCOPE_X_DIVS * (unit_ns / 1000u) + 1275u) / 2550u;
        if (ms_x10 >= 1000u) {
            scope_format_u32(number, ms_x10 / 10u);
            n = scope_append_text(out, n, 12, number);
            n = scope_append_text(out, n, 12, "M");
            (void)n;
            return;
        }
        scope_format_u32(number, ms_x10 / 10u);
        n = scope_append_text(out, n, 12, number);
        n = scope_append_text(out, n, 12, ".");
        if (n < 11u) {
                out[n++] = (char)('0' + ms_x10 % 10u);
                out[n] = 0;
        }
        n = scope_append_text(out, n, 12, "M");
    } else {
        uint32_t s_x10 = ((uint32_t)mag * SCOPE_X_DIVS * (unit_ns / 10000000u) + 127u) / 255u;
        if (s_x10 >= 1000u) {
            scope_format_u32(number, s_x10 / 10u);
            n = scope_append_text(out, n, 12, number);
            n = scope_append_text(out, n, 12, "S");
            (void)n;
            return;
        }
        scope_format_u32(number, s_x10 / 10u);
        n = scope_append_text(out, n, 12, number);
        n = scope_append_text(out, n, 12, ".");
        if (n < 11u) {
            out[n++] = (char)('0' + s_x10 % 10u);
            out[n] = 0;
        }
        n = scope_append_text(out, n, 12, "S");
    }
    (void)n;
}

static const char *scope_cursor_slot_label(uint8_t idx) {
    if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        return idx ? "T2" : "T1";
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        return idx ? "Y2" : "Y1";
    }
    return idx ? "C2" : "C1";
}

static const char *scope_cursor_pos_label(uint8_t idx) {
    static char label[2][12];

    idx = idx ? 1u : 0u;
    if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
        label[idx][0] = '-';
        label[idx][1] = '-';
        label[idx][2] = 0;
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        scope_format_cursor_time_pos(label[idx],
                                     scope_cursor_time_value[idx],
                                     scope_cursor_timebase[idx]);
    } else {
        int32_t mv = scope_cursor_level_mv[idx];
        int32_t abs_mv = mv < 0 ? -mv : mv;
        if (abs_mv >= 10000) {
            char number[11];
            uint8_t n = 0;
            label[idx][n++] = mv < 0 ? '-' : '+';
            scope_format_u32(number, (uint32_t)((abs_mv + 500) / 1000));
            n = scope_append_text(label[idx], n, sizeof(label[idx]), number);
            n = scope_append_text(label[idx], n, sizeof(label[idx]), "V");
            (void)n;
        } else {
            scope_format_signed_mv(label[idx], mv);
        }
    }
    return label[idx];
}

static const char *scope_cursor_delta_chip_label(void) {
    static char label[14];

    if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
        label[0] = '-';
        label[1] = '-';
        label[2] = 0;
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        scope_format_delta_time(label);
    } else {
        scope_format_delta_level(label);
    }
    if ((label[0] == 'D') && (label[2] == ' ')) {
        return &label[3];
    }
    return label;
}

static void draw_scope_cursor_param_row(uint16_t x, uint16_t y) {
    scope_sanitize_state();
    draw_scope_measure_chip(x,
                            y,
                            UI_PARAM_CHIP_W,
                            "DELTA",
                            scope_cursor_delta_chip_label(),
                            C_GRID,
                            0,
                            0);
    draw_scope_measure_chip((uint16_t)(x + UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP),
                            y,
                            UI_PARAM_CHIP_W,
                            "CURSOR",
                            scope_cursor_chip_label(),
                            C_SCOPE,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_MODE,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_MODE);
    draw_scope_measure_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 2u),
                            y,
                            UI_PARAM_CHIP_W,
                            scope_cursor_slot_label(0),
                            scope_cursor_pos_label(0),
                            C_SCOPE,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_FIRST,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_FIRST);
    draw_scope_measure_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 3u),
                            y,
                            UI_PARAM_CHIP_W,
                            scope_cursor_slot_label(1),
                            scope_cursor_pos_label(1),
                            C_SCOPE,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND,
                            ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND);
}

static void draw_scope_move_chip(uint16_t x,
                                 uint16_t y,
                                 uint16_t w,
                                 const char *label,
                                 const char *value,
                                 uint8_t active) {
    uint16_t accent = scope_channel_color(ui.active_ch);
    uint16_t bg = active ? accent : C_PANEL_2;
    uint16_t fg = active ? C_BG : C_MUTED;
    uint8_t value_scale = lcd_text_width(value, 2) <= w ? 2u : 1u;
    uint16_t value_y = (uint16_t)(y + (value_scale == 2u ? 13u : 18u));

    lcd_rect(x, y, w, 32, bg);
    lcd_rect(x, y, w, 2, accent);
    lcd_text((uint16_t)(x + 6u), (uint16_t)(y + 4u), label, fg, bg, 1);
    lcd_text_center(x, value_y, w, value, fg, bg, value_scale);
    if (active) {
        uint16_t text_w = lcd_text_width(value, value_scale);
        uint16_t underline_w = text_w > (uint16_t)(w - 12u) ? (uint16_t)(w - 12u) : text_w;
        lcd_rect((uint16_t)(x + (w - underline_w) / 2u), (uint16_t)(y + 29u), underline_w, 2, fg);
    }
}

static void draw_scope_move_param_row(uint16_t x, uint16_t y) {
    draw_scope_move_chip(x,
                         y,
                         UI_PARAM_CHIP_W,
                         "MOVE",
                         ui.active_ch == 2u ? "CH2" : "CH1",
                         ui.scope_move_sel == SCOPE_MOVE_SEL_CHANNEL);
    draw_scope_move_chip((uint16_t)(x + UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP),
                         y,
                         UI_PARAM_CHIP_W,
                         "Y POS",
                         scope_y_pos_label(),
                         ui.scope_move_sel == SCOPE_MOVE_SEL_Y);
    draw_scope_move_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 2u),
                         y,
                         UI_PARAM_CHIP_W,
                         "X POS",
                         scope_h_pos_label(),
                         ui.scope_move_sel == SCOPE_MOVE_SEL_X);
    draw_scope_move_chip((uint16_t)(x + (UI_PARAM_CHIP_W + UI_PARAM_CHIP_GAP) * 3u),
                         y,
                         UI_PARAM_CHIP_W,
                         "RESET",
                         "ZERO",
                         ui.scope_move_sel == SCOPE_MOVE_SEL_ZERO);
}

static void draw_softkeys(void) {
    uint16_t accent = mode_accent();
    if (ui.mode == UI_MODE_DMM) {
        draw_dmm_softkeys();
        return;
    }
    if (ui.mode == UI_MODE_GEN) {
        lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, C_BG);
        draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        return;
    }
    if (ui.mode == UI_MODE_SCOPE) {
        lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, C_BG);
        if (ui.scope_move_mode) {
            draw_scope_move_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        } else if (ui.scope_cursor_menu) {
            draw_scope_cursor_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        } else if (ui.scope_channel_menu) {
            draw_scope_channel_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        } else if (ui.scope_trigger_menu) {
            draw_scope_trigger_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        } else if (ui.scope_measure_menu) {
            draw_scope_measure_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        } else {
            draw_scope_param_row(UI_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        }
        return;
    }
    const char *f1 = ui.mode == UI_MODE_SCOPE ? scope_display_label() : "WAVE";
    const char *f2 = ui.mode == UI_MODE_SCOPE ? "VDIV" : "FREQ";
    const char *f3 = ui.running ? "RUN" : "HOLD";
    const char *f4 = ui.mode == UI_MODE_SCOPE ? scope_cursor_label() : "FREQ";
    if (ui.mode == UI_MODE_GEN) {
        f2 = "PARAM";
        f3 = ui.running ? "STOP" : "START";
        f4 = "VALUE";
    }
    lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, C_BG);
    draw_softkey(0, f1, accent);
    draw_softkey(1, f2, accent);
    draw_softkey(2, f3, accent);
    draw_softkey(3, f4, accent);
}

static uint8_t dmm_mode_uses_real_reading(void) {
    return ui.dmm_mode < DMM_MODE_COUNT;
}

static uint8_t dmm_sanitize_mode(uint8_t mode) {
    if (mode == DMM_MODE_CONT) {
        return DMM_MODE_DIODE;
    }
    return mode < DMM_MODE_COUNT ? mode : DMM_MODE_AUTO;
}

static uint8_t text_is_numeric_reading(const char *text) {
    uint8_t has_digit = 0;

    if (*text == '-') {
        ++text;
    }

    while (*text) {
        if (*text >= '0' && *text <= '9') {
            has_digit = 1;
        } else if (*text != '.') {
            return 0;
        }
        ++text;
    }

    return has_digit;
}

static void ui_text_copy(char *dst, const char *src, uint8_t max_len) {
    uint8_t i = 0;

    if (!max_len) {
        return;
    }
    while (i < (uint8_t)(max_len - 1u) && src[i]) {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = 0;
}

static void ui_text_append(char *dst, const char *src, uint8_t max_len) {
    uint8_t pos = 0;

    if (!max_len) {
        return;
    }
    while (pos < (uint8_t)(max_len - 1u) && dst[pos]) {
        ++pos;
    }
    while (pos < (uint8_t)(max_len - 1u) && *src) {
        dst[pos++] = *src++;
    }
    dst[pos] = 0;
}

static uint8_t ui_text_equal(const char *a, const char *b) {
    while (*a && *b) {
        if (*a++ != *b++) {
            return 0;
        }
    }
    return *a == *b;
}

static void format_signed_milli(int32_t value, char out[10]) {
    uint8_t pos = 0;
    uint32_t abs_value;
    uint32_t whole;
    uint16_t frac;

    if (value < 0) {
        out[pos++] = '-';
        abs_value = (uint32_t)(-value);
    } else {
        abs_value = (uint32_t)value;
    }

    whole = abs_value / 1000u;
    frac = (uint16_t)(abs_value % 1000u);
    if (whole > 9999u) {
        ui_text_copy(out, value < 0 ? "-OVER" : "OVER", 10);
        return;
    }

    if (whole >= 1000u) {
        out[pos++] = (char)('0' + whole / 1000u);
    }
    if (whole >= 100u) {
        out[pos++] = (char)('0' + (whole / 100u) % 10u);
    }
    if (whole >= 10u) {
        out[pos++] = (char)('0' + (whole / 10u) % 10u);
    }
    out[pos++] = (char)('0' + whole % 10u);
    out[pos++] = '.';
    out[pos++] = (char)('0' + frac / 100u);
    out[pos++] = (char)('0' + (frac / 10u) % 10u);
    out[pos++] = (char)('0' + frac % 10u);
    out[pos] = 0;
}

static uint8_t dmm_low_current_mode(void) {
    return ui.dmm_mode == DMM_MODE_AC_LO_CURR || ui.dmm_mode == DMM_MODE_DC_LO_CURR;
}

static void format_low_current_value(const char *text, char out[10]) {
    uint8_t pos = 0;
    uint8_t frac = 0;
    uint8_t seen_dot = 0;

    if (!text_is_numeric_reading(text)) {
        ui_text_copy(out, text, 10);
        return;
    }

    if (*text == '-') {
        out[pos++] = *text++;
    }

    while (*text && pos < 9u) {
        if (*text == '.') {
            seen_dot = 1;
            out[pos++] = *text++;
            break;
        }
        out[pos++] = *text++;
    }

    if (!seen_dot && pos < 9u) {
        out[pos++] = '.';
    }

    while (*text && frac < 4u && pos < 9u) {
        if (*text >= '0' && *text <= '9') {
            out[pos++] = *text;
            ++frac;
        }
        ++text;
    }
    while (frac < 4u && pos < 9u) {
        out[pos++] = '0';
        ++frac;
    }
    out[pos] = 0;
}

static void dmm_stats_reset(void) {
    dmm_stats.valid = 0;
    dmm_stats.count = 0;
    dmm_stats.zero_count = 0;
    dmm_stats.zero_idle = 0;
}

static uint8_t dmm_stat_decimals(void) {
    return dmm_low_current_mode() ? 4u : 3u;
}

static uint8_t dmm_stats_near_zero(int32_t value) {
    if (value < 0) {
        value = -value;
    }
    return value <= 5;
}

static uint8_t parse_fixed_decimal(const char *text, uint8_t decimals, int32_t *out) {
    uint8_t negative = 0;
    uint8_t seen_dot = 0;
    uint8_t digits = 0;
    uint8_t frac_digits = 0;
    int32_t value = 0;

    if (*text == '-') {
        negative = 1;
        ++text;
    }

    while (*text) {
        if (*text >= '0' && *text <= '9') {
            ++digits;
            if (!seen_dot) {
                value = value * 10 + (int32_t)(*text - '0');
            } else if (frac_digits < decimals) {
                value = value * 10 + (int32_t)(*text - '0');
                ++frac_digits;
            }
        } else if (*text == '.' && !seen_dot) {
            seen_dot = 1;
        } else {
            return 0;
        }
        ++text;
    }

    if (!digits) {
        return 0;
    }
    while (frac_digits < decimals) {
        value *= 10;
        ++frac_digits;
    }

    *out = negative ? -value : value;
    return 1;
}

static const char *dmm_display_value(void);
static const char *dmm_display_unit(void);

static void dmm_stats_update(void) {
    const char *unit;
    int32_t sample;
    int32_t delta;
    uint8_t decimals;
    uint8_t near_zero;

    if (ui.mode != UI_MODE_DMM || dmm_hold_active) {
        return;
    }
    if (!dmm_has_reading() || !dmm_value_is_numeric()) {
        dmm_stats_reset();
        return;
    }

    unit = dmm_display_unit();
    decimals = dmm_stat_decimals();
    if (!parse_fixed_decimal(dmm_display_value(), decimals, &sample)) {
        dmm_stats_reset();
        return;
    }
    near_zero = dmm_stats_near_zero(sample);

    if (near_zero) {
        if (dmm_stats.zero_count < DMM_STATS_ZERO_RESET_SAMPLES) {
            ++dmm_stats.zero_count;
        }
        if (dmm_stats.zero_count >= DMM_STATS_ZERO_RESET_SAMPLES) {
            dmm_stats.valid = 0;
            dmm_stats.count = 0;
            dmm_stats.zero_idle = 1;
            dmm_stats.zero_count = DMM_STATS_ZERO_RESET_SAMPLES;
        }
    } else if (dmm_stats.zero_idle) {
        dmm_stats.valid = 0;
        dmm_stats.count = 0;
        dmm_stats.zero_idle = 0;
        dmm_stats.zero_count = 0;
    } else {
        dmm_stats.zero_count = 0;
    }

    if (!dmm_stats.valid ||
        dmm_stats.mode != ui.dmm_mode ||
        dmm_stats.decimals != decimals ||
        !ui_text_equal(dmm_stats.unit, unit)) {
        dmm_stats.valid = 1;
        dmm_stats.mode = ui.dmm_mode;
        dmm_stats.decimals = decimals;
        dmm_stats.count = 1;
        dmm_stats.min_value = sample;
        dmm_stats.avg_value = sample;
        dmm_stats.max_value = sample;
        ui_text_copy(dmm_stats.unit, unit, sizeof(dmm_stats.unit));
        if (!near_zero) {
            dmm_stats.zero_idle = 0;
        }
        return;
    }

    if (sample < dmm_stats.min_value) {
        dmm_stats.min_value = sample;
    }
    if (sample > dmm_stats.max_value) {
        dmm_stats.max_value = sample;
    }

    if (dmm_stats.count < 60000u) {
        ++dmm_stats.count;
    }
    delta = (int32_t)(sample - dmm_stats.avg_value);
    dmm_stats.avg_value = (int32_t)(dmm_stats.avg_value + delta / (int32_t)dmm_stats.count);
}

static uint8_t dmm_live_open_display(void) {
    return dmm_has_reading() && ui_text_equal(dmm_value_text(), "OPEN");
}

static uint8_t dmm_live_wire_detected(void) {
    return ui.dmm_mode == DMM_MODE_LIVE && dmm_live_wire_alert;
}

static const char *dmm_live_display_value(void) {
    if (dmm_live_open_display()) {
        return "OPEN";
    }
    if (!dmm_has_reading()) {
        return "";
    }
    if (ui.dmm_mode == DMM_MODE_LIVE) {
        return "LIVE";
    }
    if (dmm_low_current_mode()) {
        if (!dmm_value_is_numeric()) {
            return "";
        }
        format_low_current_value(dmm_value_text(), dmm_low_current_value);
        return dmm_low_current_value;
    }
    return dmm_mode_uses_real_reading() ? dmm_value_text() : dmm_values[ui.dmm_mode];
}

static const char *dmm_live_display_unit(void) {
    if (ui.dmm_mode == DMM_MODE_AUTO) {
        return dmm_has_reading() ? dmm_unit_text() : "";
    }
    if (dmm_live_open_display()) {
        return "";
    }
    if (!dmm_has_reading()) {
        return "";
    }
    if (ui.dmm_mode == DMM_MODE_LIVE) {
        return "";
    }
    if (dmm_low_current_mode()) {
        return "MA";
    }
    return dmm_mode_uses_real_reading() ? dmm_unit_text() : dmm_units[ui.dmm_mode];
}

static const char *dmm_rel_display_value(void) {
    const char *unit = dmm_live_display_unit();

    if (!dmm_has_reading() || !dmm_value_is_numeric()) {
        return "-----";
    }
    if (!ui_text_equal(unit, dmm_rel_unit)) {
        return "RANGE";
    }

    format_signed_milli((int32_t)(dmm_value_milli_units() - dmm_rel_ref_milli), dmm_rel_value);
    return dmm_rel_value;
}

static const char *dmm_display_value(void) {
    if (dmm_hold_active) {
        return dmm_hold_value;
    }
    return dmm_rel_active ? dmm_rel_display_value() : dmm_live_display_value();
}

static const char *dmm_display_unit(void) {
    if (dmm_hold_active) {
        return dmm_hold_unit;
    }
    return dmm_rel_active ? dmm_rel_unit : dmm_live_display_unit();
}

static uint8_t dmm_current_mode(void) {
    return ui.dmm_mode == DMM_MODE_AC_HI_CURR ||
           ui.dmm_mode == DMM_MODE_DC_HI_CURR ||
           ui.dmm_mode == DMM_MODE_AC_LO_CURR ||
           ui.dmm_mode == DMM_MODE_DC_LO_CURR;
}

static uint8_t dmm_high_current_mode(void) {
    return ui.dmm_mode == DMM_MODE_AC_HI_CURR || ui.dmm_mode == DMM_MODE_DC_HI_CURR;
}

static uint8_t dmm_voltage_warning_level(void) {
    uint32_t mv;
    uint32_t danger_mv;

    if (!dmm_has_reading() || !dmm_value_is_numeric() || !ui_text_equal(dmm_unit_text(), "V")) {
        return 0;
    }
    if (ui.dmm_mode != DMM_MODE_AUTO && ui.dmm_mode != DMM_MODE_DCV && ui.dmm_mode != DMM_MODE_ACV) {
        return 0;
    }

    mv = dmm_value_milli_units() < 0 ? (uint32_t)(-dmm_value_milli_units()) : (uint32_t)dmm_value_milli_units();
    danger_mv = ui.dmm_mode == DMM_MODE_DCV ? 60000u : 30000u;
    if (mv >= danger_mv) {
        return 2;
    }
    if (mv >= 30000u) {
        return 1;
    }
    return 0;
}

static uint16_t dmm_measure_color(void) {
    uint8_t warning = dmm_voltage_warning_level();
    if (dmm_live_wire_detected()) {
        return C_WARN;
    }
    if (warning >= 2u) {
        return C_WARN;
    }
    if (warning) {
        return C_CH1;
    }
    return C_TEXT;
}

static const char *dmm_alert_label(void) {
    uint8_t warning = dmm_voltage_warning_level();
    if (dmm_live_wire_detected()) {
        return "LIVE WIRE";
    }
    if (warning >= 2u) {
        return "DANGER V";
    }
    if (warning) {
        return "HIGH V";
    }
    if (dmm_current_mode()) {
        return dmm_high_current_mode() ? "USE HI JACK" : "USE LO JACK";
    }
    return 0;
}

static uint16_t dmm_alert_color(void) {
    uint8_t warning = dmm_voltage_warning_level();
    if (dmm_live_wire_detected()) {
        return C_WARN;
    }
    if (warning >= 2u) {
        return C_WARN;
    }
    return C_CH1;
}

static uint8_t dmm_auto_range_display(void) {
    return ui.auto_range || ui.dmm_mode == DMM_MODE_AUTO;
}

static const char *dmm_mode_status_label(void) {
    if (dmm_hold_active) {
        return "HOLD";
    }
    if (dmm_rel_active) {
        return "REL";
    }
    return dmm_auto_range_display() ? "AUTO MODE" : "MANUAL MODE";
}

static const char *dmm_mode_status_detail(void) {
    const char *value;
    const char *unit;

    if (!dmm_rel_active) {
        return 0;
    }

    value = dmm_rel_ref_value;
    unit = dmm_rel_unit;
    ui_text_copy(dmm_status_detail, value, sizeof(dmm_status_detail));
    if (unit[0] && !ui_text_equal(value, "-----") && !ui_text_equal(value, "RANGE")) {
        ui_text_append(dmm_status_detail, " ", sizeof(dmm_status_detail));
        ui_text_append(dmm_status_detail, unit, sizeof(dmm_status_detail));
    }
    return dmm_status_detail;
}

static void lcd_text_right_fit(uint16_t left,
                               uint16_t right,
                               uint16_t y,
                               const char *text,
                               uint16_t fg,
                               uint16_t bg,
                               uint8_t preferred_scale,
                               uint8_t min_scale) {
    uint16_t width;
    uint16_t span = right > left ? (uint16_t)(right - left) : 0u;
    uint8_t scale = preferred_scale;

    while (scale > min_scale && lcd_text_width(text, scale) > span) {
        --scale;
    }

    width = lcd_text_width(text, scale);
    lcd_text(width >= span ? left : (uint16_t)(right - width), y, text, fg, bg, scale);
}

static void draw_dmm_mode_status(uint16_t x, uint16_t y, uint16_t right, uint16_t bg) {
    const char *label = dmm_mode_status_label();
    const char *detail = dmm_mode_status_detail();
    uint16_t label_w = lcd_text_width(label, 2);
    uint16_t label_color = dmm_hold_active ? C_WARN : C_TEXT;

    lcd_text(x, y, label, label_color, bg, 2);
    if (detail && right > (uint16_t)(x + label_w + 10u)) {
        uint16_t detail_x = (uint16_t)(x + label_w + 10u);
        uint16_t detail_right = right;
        uint16_t detail_span = detail_right > detail_x ? (uint16_t)(detail_right - detail_x) : 0u;
        uint8_t scale = lcd_text_width(detail, 2) <= detail_span ? 2u : 1u;
        lcd_text_right_fit(detail_x,
                           detail_right,
                           (uint16_t)(y + (scale == 2u ? 0u : 4u)),
                           detail,
                           C_TEXT,
                           bg,
                           scale,
                           scale);
    }
}

static void draw_dmm_alert_chip(uint16_t x, uint16_t y, uint16_t w, const char *label, uint16_t color) {
    if (!label) {
        return;
    }

    lcd_rect(x, y, w, 20, color);
    lcd_frame(x, y, w, 20, C_TEXT);
    lcd_text_center(x, (uint16_t)(y + 6u), w, label, color == C_CH1 ? C_BG : C_TEXT, color, 1);
}

static void lcd_thick_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    lcd_line(x0, y0, x1, y1, color);
    lcd_line((int16_t)(x0 + 1), y0, (int16_t)(x1 + 1), y1, color);
    lcd_line((int16_t)(x0 - 1), y0, (int16_t)(x1 - 1), y1, color);
    lcd_line(x0, (int16_t)(y0 + 1), x1, (int16_t)(y1 + 1), color);
    lcd_line(x0, (int16_t)(y0 - 1), x1, (int16_t)(y1 - 1), color);
}

static void draw_high_voltage_symbol(uint16_t cx, uint16_t top, uint16_t size, uint16_t bg) {
    int16_t half = (int16_t)(size / 2u);
    int16_t x0 = (int16_t)((int16_t)cx - half);
    int16_t x1 = (int16_t)((int16_t)cx + half);
    int16_t y0 = (int16_t)top;
    int16_t y1 = (int16_t)(top + size);
    int16_t bolt_top = (int16_t)(top + size / 5u);
    int16_t bolt_mid = (int16_t)(top + size / 2u);
    int16_t bolt_bot = (int16_t)(top + size - size / 7u);

    (void)bg;
    lcd_thick_line((int16_t)cx, y0, x0, y1, C_WARN);
    lcd_thick_line(x0, y1, x1, y1, C_WARN);
    lcd_thick_line(x1, y1, (int16_t)cx, y0, C_WARN);
    lcd_rect((uint16_t)(cx - size / 20u), (uint16_t)(top + size / 4u), (uint16_t)(size / 10u + 1u), (uint16_t)(size / 3u), C_WARN);
    lcd_rect((uint16_t)(cx - size / 20u), (uint16_t)(top + size - size / 5u), (uint16_t)(size / 10u + 1u), (uint16_t)(size / 10u + 1u), C_WARN);
    lcd_thick_line((int16_t)(cx + size / 9u - 2), bolt_top, (int16_t)(cx - size / 8u - 2), bolt_mid, C_TEXT);
    lcd_thick_line((int16_t)(cx - size / 8u - 2), bolt_mid, (int16_t)(cx + size / 10u - 2), (int16_t)(bolt_mid - size / 10u), C_TEXT);
    lcd_thick_line((int16_t)(cx + size / 10u - 2), (int16_t)(bolt_mid - size / 10u), (int16_t)(cx - size / 8u - 2), bolt_bot, C_TEXT);
}

static void draw_dmm_panel(void) {
    enum {
        VALUE_LEFT = 22,
        VALUE_RIGHT = 300,
        VALUE_Y = 100,
        UNIT_Y = 150,
    };
    uint8_t alert = dmm_voltage_warning_level();
    uint8_t live_detected = dmm_live_wire_detected();
    uint16_t value_right = live_detected ? 218u : VALUE_RIGHT;
    uint16_t value_color = dmm_measure_color();
    uint16_t unit_color = alert ? value_color : C_DMM;

    lcd_rect(10, 55, 300, 105, C_PANEL);
    lcd_rect(10, 55, 5, 105, C_DMM);
    lcd_text(22, 58, dmm_mode_names[ui.dmm_mode], C_DMM, C_PANEL, 3);
    if (dmm_hold_active || dmm_rel_active) {
        draw_dmm_mode_status(22, 80, 300u, C_PANEL);
    }
    if (!dmm_hold_active && !dmm_rel_active) {
        draw_dmm_alert_chip(176, 84, 124, dmm_alert_label(), dmm_alert_color());
    }
    lcd_text_right_fit(VALUE_LEFT, value_right, VALUE_Y, dmm_display_value(), value_color, C_PANEL, 6, 4);
    lcd_text_right_fit(VALUE_LEFT, VALUE_RIGHT, UNIT_Y, dmm_display_unit(), unit_color, C_PANEL, 3, 2);
    if (live_detected) {
        draw_high_voltage_symbol(264, 100, 42, C_PANEL);
    }
}

static void draw_dmm_immersive_panel(void) {
    enum {
        VALUE_LEFT = 16,
        VALUE_RIGHT = 310,
        VALUE_Y = 88,
        UNIT_Y = 148,
    };
    uint8_t alert = dmm_voltage_warning_level();
    uint8_t live_detected = dmm_live_wire_detected();
    uint16_t value_right = live_detected ? 222u : VALUE_RIGHT;
    uint16_t value_color = dmm_measure_color();
    uint16_t unit_color = alert ? value_color : C_DMM;

    lcd_rect(0, 24, LCD_WIDTH, 174, C_BG);
    lcd_text(18, 27, dmm_mode_names[ui.dmm_mode], C_DMM, C_BG, 3);
    if (dmm_hold_active || dmm_rel_active) {
        draw_dmm_mode_status(18, 68, 304u, C_BG);
    }
    if (!dmm_hold_active && !dmm_rel_active) {
        draw_dmm_alert_chip(176, 55, 128, dmm_alert_label(), dmm_alert_color());
    }
    lcd_text_right_fit(VALUE_LEFT, value_right, VALUE_Y, dmm_display_value(), value_color, C_BG, 7, 5);
    lcd_text_right_fit(VALUE_LEFT, VALUE_RIGHT, UNIT_Y, dmm_display_unit(), unit_color, C_BG, 4, 3);
    if (live_detected) {
        draw_high_voltage_symbol(266, 92, 58, C_BG);
    }
}

static void draw_dmm(void) {
    lcd_rect(0, Y_BODY, LCD_WIDTH, H_BODY, C_BG);
    draw_mode_tabs();
    draw_dmm_panel();
}

static void draw_dmm_immersive(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG);
    draw_micro_status(8, 6, C_BG);
    draw_battery_status_overlay(C_BG);
    draw_dmm_immersive_panel();
    draw_dmm_softkeys();
}

static int16_t scope_sine(uint16_t phase) {
    return scope_sine_lut[(phase >> 2) & 63u];
}

static int16_t scope_sine_interp(uint16_t phase) {
    uint8_t index = (uint8_t)((phase >> 2) & 63u);
    uint8_t next = (uint8_t)((index + 1u) & 63u);
    uint8_t frac = (uint8_t)(phase & 3u);
    int16_t a = scope_sine_lut[index];
    int16_t b = scope_sine_lut[next];

    return (int16_t)(a + ((int16_t)(b - a) * frac + 2) / 4);
}

static int16_t scope_sim_y(uint16_t x, uint8_t ch2, int16_t center, int16_t amp) {
    uint16_t timebase = scope_safe_timebase();
    uint16_t step = (uint16_t)(2u + (timebase < 10u ? timebase : 10u));
    uint16_t phase = (uint16_t)(ui.scope_phase + x * step);

    if (ch2) {
        uint16_t p = (uint16_t)(phase + 80u);
        int16_t level = (p & 0x80u) ? (int16_t)-amp : amp;
        uint8_t edge = (uint8_t)(p & 0x1Fu);
        if (edge < 4u) {
            level = (int16_t)(level * edge / 4);
        }
        return (int16_t)(center + level);
    }

    int16_t fundamental = scope_sine(phase);
    int16_t harmonic = (int16_t)(scope_sine((uint16_t)(phase * 3u + 24u)) / 5);
    int16_t sample = (int16_t)(fundamental + harmonic);
    return (int16_t)(center - sample * amp / 64);
}

static void scope_trace_invalidate(void) {
    if (ui.scope_afterglow) {
        for (uint8_t ch = 0; ch < 2u; ++ch) {
            if (scope_trace_cache[ch].valid) {
                scope_after_count[ch] = scope_trace_cache[ch].count;
                for (uint8_t i = 0; i < scope_trace_cache[ch].count; ++i) {
                    scope_after_y[ch][i] = scope_trace_cache[ch].y[i];
                }
                scope_after_valid[ch] = 1;
            }
        }
    } else {
        scope_after_valid[0] = 0;
        scope_after_valid[1] = 0;
    }

    scope_trace_cache[0].valid = 0;
    scope_trace_cache[1].valid = 0;
}

static uint32_t scope_isqrt_u32(uint32_t value) {
    uint32_t root = 0;
    uint32_t bit = 1ul << 30;

    while (bit > value) {
        bit >>= 2;
    }
    while (bit) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

static uint8_t scope_measure_current_slot(void) {
    return scope_measure_hist_pos == 0u ? (uint8_t)(SCOPE_MEASURE_WINDOW - 1u) : (uint8_t)(scope_measure_hist_pos - 1u);
}

static void scope_measure_history_store(uint8_t min0,
                                        uint8_t max0,
                                        uint16_t rms0,
                                        uint8_t min1,
                                        uint8_t max1,
                                        uint16_t rms1,
                                        uint32_t freq0,
                                        uint32_t freq1) {
    if (!scope_measure_hist_ready) {
        for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
            scope_measure_min_hist[0][i] = min0;
            scope_measure_max_hist[0][i] = max0;
            scope_measure_rms_hist[0][i] = rms0;
            scope_measure_freq_hist[0][i] = freq0;
            scope_measure_min_hist[1][i] = min1;
            scope_measure_max_hist[1][i] = max1;
            scope_measure_rms_hist[1][i] = rms1;
            scope_measure_freq_hist[1][i] = freq1;
        }
        scope_measure_hist_pos = 1;
        scope_measure_hist_ready = 1;
        return;
    }

    scope_measure_min_hist[0][scope_measure_hist_pos] = min0;
    scope_measure_max_hist[0][scope_measure_hist_pos] = max0;
    scope_measure_rms_hist[0][scope_measure_hist_pos] = rms0;
    scope_measure_freq_hist[0][scope_measure_hist_pos] = freq0;
    scope_measure_min_hist[1][scope_measure_hist_pos] = min1;
    scope_measure_max_hist[1][scope_measure_hist_pos] = max1;
    scope_measure_rms_hist[1][scope_measure_hist_pos] = rms1;
    scope_measure_freq_hist[1][scope_measure_hist_pos] = freq1;
    ++scope_measure_hist_pos;
    if (scope_measure_hist_pos >= SCOPE_MEASURE_WINDOW) {
        scope_measure_hist_pos = 0;
    }
}

static void scope_compute_stats(void) {
    uint32_t sum0 = 0;
    uint32_t sum1 = 0;
    uint32_t sq0 = 0;
    uint32_t sq1 = 0;
    uint8_t min0 = 255;
    uint8_t min1 = 255;
    uint8_t max0 = 0;
    uint8_t max1 = 0;
    uint16_t start = scope_safe_timebase() < 21u ? 2u : 0u;
    uint16_t end = scope_safe_timebase() < 21u ? 2046u : SCOPE_VISIBLE_SAMPLE_COUNT;
    uint16_t count = 0;

    if (end > SCOPE_SAMPLE_COUNT) {
        end = SCOPE_SAMPLE_COUNT;
    }
    if (start >= end) {
        start = 0;
        end = SCOPE_SAMPLE_COUNT;
    }
    count = (uint16_t)(end - start);

    for (uint16_t i = start; i < end; ++i) {
        uint8_t s0 = scope_samples[(uint16_t)(i * 2u)];
        uint8_t s1 = scope_samples[(uint16_t)(i * 2u + 1u)];

        if (s0 < min0) {
            min0 = s0;
        }
        if (s0 > max0) {
            max0 = s0;
        }
        if (s1 < min1) {
            min1 = s1;
        }
        if (s1 > max1) {
            max1 = s1;
        }
        sum0 += s0;
        sum1 += s1;
        int16_t d0 = (int16_t)s0 - 128;
        int16_t d1 = (int16_t)s1 - 128;
        sq0 += (uint32_t)(d0 * d0);
        sq1 += (uint32_t)(d1 * d1);
    }

    scope_min_raw[0] = min0;
    scope_min_raw[1] = min1;
    scope_max_raw[0] = max0;
    scope_max_raw[1] = max1;
    scope_avg_raw[0] = (uint8_t)(sum0 / count);
    scope_avg_raw[1] = (uint8_t)(sum1 / count);
    scope_sq_sum_raw[0] = sq0;
    scope_sq_sum_raw[1] = sq1;
    scope_measure_history_store(min0,
                                max0,
                                (uint16_t)scope_isqrt_u32((sq0 + count / 2u) / count),
                                min1,
                                max1,
                                (uint16_t)scope_isqrt_u32((sq1 + count / 2u) / count),
                                scope_estimate_freq_hz_window(0, start, end, min0, max0),
                                scope_estimate_freq_hz_window(1, start, end, min1, max1));
}

static uint16_t scope_slow_roll_interval_ms(void) {
    uint32_t div_ms;
    uint32_t interval;

    div_ms = (scope_timebase_unit_ns[scope_safe_timebase()] + 50000u) / 100000u;
    if (!div_ms) {
        div_ms = 1u;
    }
    interval = (div_ms * SCOPE_X_DIVS + SCOPE_SLOW_ROLL_MAX_POINTS / 2u) /
               SCOPE_SLOW_ROLL_MAX_POINTS;
    if (interval < SCOPE_SLOW_ROLL_MIN_MS) {
        interval = SCOPE_SLOW_ROLL_MIN_MS;
    }
    if (interval > 60000u) {
        interval = 60000u;
    }
    return (uint16_t)interval;
}

static uint8_t scope_soft_roll_points_per_frame(void) {
    uint32_t div_ms = (scope_timebase_unit_ns[scope_safe_timebase()] + 50000u) / 100000u;
    uint32_t screen_ms;
    uint32_t points;

    if (!div_ms) {
        div_ms = 1u;
    }
    screen_ms = div_ms * SCOPE_X_DIVS;
    points = ((uint32_t)scope_slow_roll_interval_ms() * SCOPE_SLOW_ROLL_MAX_POINTS +
              screen_ms / 2u) / screen_ms;
    if (!points) {
        points = 1u;
    } else if (points > SCOPE_SOFT_ROLL_MAX_POINTS_PER_FRAME) {
        points = SCOPE_SOFT_ROLL_MAX_POINTS_PER_FRAME;
    }
    return (uint8_t)points;
}

static uint16_t scope_normal_frame_interval_ms(void) {
    uint8_t timebase = scope_safe_timebase();

    if (timebase >= 15u && timebase <= 18u) {
        return 20u;
    }
    if (timebase >= 13u) {
        return 30u;
    }
    return SCOPE_FRAME_MS;
}

static uint16_t scope_frame_interval_ms(void) {
    return scope_slow_roll_active() ? scope_slow_roll_interval_ms() : scope_normal_frame_interval_ms();
}

static void scope_slow_roll_update_stats(void) {
    uint32_t sum0 = 0;
    uint32_t sum1 = 0;
    uint32_t sq0 = 0;
    uint32_t sq1 = 0;
    uint8_t min0 = 255;
    uint8_t min1 = 255;
    uint8_t max0 = 0;
    uint8_t max1 = 0;
    uint16_t count = scope_slow_roll_count ? scope_slow_roll_count : 1u;

    for (uint16_t i = 0; i < count; ++i) {
        uint8_t s0 = scope_slow_roll_raw_at(0, i);
        uint8_t s1 = scope_slow_roll_raw_at(1, i);
        int16_t d0 = (int16_t)s0 - 128;
        int16_t d1 = (int16_t)s1 - 128;

        if (s0 < min0) {
            min0 = s0;
        }
        if (s0 > max0) {
            max0 = s0;
        }
        if (s1 < min1) {
            min1 = s1;
        }
        if (s1 > max1) {
            max1 = s1;
        }
        sum0 += s0;
        sum1 += s1;
        sq0 += (uint32_t)(d0 * d0);
        sq1 += (uint32_t)(d1 * d1);
    }

    scope_min_raw[0] = min0;
    scope_min_raw[1] = min1;
    scope_max_raw[0] = max0;
    scope_max_raw[1] = max1;
    scope_avg_raw[0] = (uint8_t)(sum0 / count);
    scope_avg_raw[1] = (uint8_t)(sum1 / count);
    scope_sq_sum_raw[0] = sq0;
    scope_sq_sum_raw[1] = sq1;
    scope_measure_history_store(min0,
                                max0,
                                (uint16_t)scope_isqrt_u32((sq0 + count / 2u) / count),
                                min1,
                                max1,
                                (uint16_t)scope_isqrt_u32((sq1 + count / 2u) / count),
                                0,
                                0);
}

static void scope_soft_roll_store_point(uint8_t ch1, uint8_t ch2) {
    uint16_t write_index;

    if (scope_slow_roll_count < SCOPE_SLOW_ROLL_MAX_POINTS) {
        write_index = scope_slow_roll_count;
        ++scope_slow_roll_count;
    } else {
        write_index = scope_slow_roll_head;
        ++scope_slow_roll_head;
        if (scope_slow_roll_head >= SCOPE_SLOW_ROLL_MAX_POINTS) {
            scope_slow_roll_head = 0;
        }
    }
    scope_slow_roll_raw[0][write_index] = ch1;
    scope_slow_roll_raw[1][write_index] = ch2;
    ++scope_slow_roll_seq;
}

static uint8_t scope_trigger_crossed(uint8_t prev, uint8_t cur) {
    uint8_t level = ui.scope_trigger_level;

    if (ui.scope_trigger_edge) {
        return prev >= level && cur < level;
    }
    return prev <= level && cur > level;
}

static uint8_t scope_slow_roll_find_trigger(uint16_t count, uint16_t new_points, uint16_t *hit_index) {
    uint8_t src = scope_trigger_source_index();
    uint16_t start;

    if (!hit_index ||
        ui.scope_display != SCOPE_DISPLAY_YT ||
        !scope_ch_enabled[src] ||
        count < 2u ||
        !new_points) {
        return 0;
    }
    if (new_points >= count) {
        start = 1u;
    } else {
        start = (uint16_t)(count - new_points);
        if (!start) {
            start = 1u;
        }
    }
    for (uint16_t i = start; i < count; ++i) {
        if (scope_trigger_crossed(scope_slow_roll_raw_at(src, (uint16_t)(i - 1u)),
                                  scope_slow_roll_raw_at(src, i))) {
            *hit_index = i;
            return 1;
        }
    }
    return 0;
}

static uint16_t scope_slow_roll_index_seq(uint16_t hw_seq, uint16_t count, uint16_t index) {
    return (uint16_t)(hw_seq - count + 1u + index);
}

static uint8_t scope_seq_reached(uint16_t current, uint16_t target) {
    return (int16_t)(current - target) >= 0;
}

static uint8_t scope_slow_roll_seq_index(uint16_t sample_seq,
                                         uint16_t hw_seq,
                                         uint16_t count,
                                         uint16_t *index) {
    uint16_t oldest;
    uint16_t delta;

    if (!index || !count) {
        return 0;
    }
    oldest = (uint16_t)(hw_seq - count + 1u);
    delta = (uint16_t)(sample_seq - oldest);
    if (delta >= count) {
        return 0;
    }
    *index = delta;
    return 1;
}

static uint8_t scope_find_trigger_offset(const uint8_t *samples, uint16_t *offset) {
    uint8_t src = scope_trigger_source_index();
    uint16_t trigger_screen_pos = scope_trigger_screen_sample_pos();

    if (!samples || !offset ||
        ui.scope_display != SCOPE_DISPLAY_YT ||
        !scope_ch_enabled[src]) {
        return 0;
    }

    for (uint16_t i = 1; i < SCOPE_SAMPLE_COUNT; ++i) {
        uint8_t prev = samples[(uint16_t)((i - 1u) * 2u + src)];
        uint8_t cur = samples[(uint16_t)(i * 2u + src)];
        if (scope_trigger_crossed(prev, cur)) {
            *offset = (uint16_t)((i - trigger_screen_pos) & (SCOPE_SAMPLE_COUNT - 1u));
            return 1;
        }
    }
    return 0;
}

static void scope_copy_capture_samples(void) {
    uint32_t *dst = (uint32_t *)(void *)scope_samples;
    const uint32_t *src = (const uint32_t *)(const void *)scope_capture_samples;

    for (uint16_t i = 0; i < (uint16_t)(SCOPE_SAMPLE_BYTES / 4u); ++i) {
        dst[i] = src[i];
    }
}

static uint8_t scope_trigger_accepts_capture(uint8_t found) {
    if (scope_auto_zero_active) {
        return 1;
    }
    if (scope_slow_roll_active()) {
        return 1;
    }
    if (ui.scope_display != SCOPE_DISPLAY_YT ||
        !scope_ch_enabled[scope_trigger_source_index()]) {
        return 1;
    }
    if (ui.scope_trigger_mode == SCOPE_TRIGGER_AUTO) {
        return 1;
    }
    return found;
}

static uint8_t scope_poll_frame(void) {
    if (!scope_hw_enabled()) {
        scope_frame_valid = 0;
        ++scope_frame_id;
        if (ui.scope_display == SCOPE_DISPLAY_ROLL) {
            scope_roll_offset = (uint16_t)((scope_roll_offset + 19u) & (SCOPE_SAMPLE_COUNT - 1u));
        }
        scope_trace_invalidate();
        return 1;
    }

    uint16_t trigger_offset = 0;
    uint8_t trigger_found;
    uint8_t slow_roll = scope_slow_roll_active();

    if (slow_roll) {
        uint16_t hw_count = 0;
        uint16_t hw_seq = 0;
        uint8_t soft_roll = scope_soft_roll_active();
        uint8_t timebase_changed = scope_slow_roll_timebase != scope_safe_timebase();
        uint8_t added = 0;
        uint16_t prev_seq = scope_slow_roll_seq;
        uint16_t new_points = 0;
        uint16_t hit_index = 0;
        uint8_t trigger_found = 0;

        if (timebase_changed) {
            scope_slow_roll_reset();
            prev_seq = scope_slow_roll_seq;
        }
        if (soft_roll) {
            uint8_t sample[2];
            uint8_t points = scope_soft_roll_points_per_frame();
            for (uint8_t i = 0; i < points; ++i) {
                if (!scope_hw_capture_point(sample, ui.scope_timebase)) {
                    break;
                }
                scope_soft_roll_store_point(sample[0], sample[1]);
                added = 1;
            }
            hw_count = scope_slow_roll_count;
            hw_seq = scope_slow_roll_seq;
            new_points = (uint16_t)(hw_seq - prev_seq);
        } else {
            if (timebase_changed) {
                scope_hw_slow_start(ui.scope_timebase);
            }
            if (!scope_hw_slow_snapshot(scope_slow_roll_raw[0],
                                        scope_slow_roll_raw[1],
                                        SCOPE_SLOW_ROLL_MAX_POINTS,
                                        &hw_count,
                                        &hw_seq)) {
                return 0;
            }
            scope_slow_roll_head = 0;
            new_points = (uint16_t)(hw_seq - prev_seq);
        }
        if (!hw_count) {
            return 0;
        }
        if (new_points > hw_count) {
            new_points = hw_count;
        }
        if ((!soft_roll && scope_slow_roll_seq == hw_seq && scope_frame_valid) ||
            (soft_roll && !added && scope_frame_valid)) {
            return 0;
        }
        scope_slow_roll_seq = hw_seq;
        scope_slow_roll_count = hw_count;
        if (ui.scope_trigger_mode != SCOPE_TRIGGER_AUTO) {
            trigger_found = scope_slow_roll_find_trigger(hw_count, new_points, &hit_index);
            if (!scope_roll_trigger_pending && trigger_found) {
                uint16_t trigger_pos = scope_trigger_screen_sample_pos();
                uint16_t post_samples = (SCOPE_SLOW_ROLL_MAX_POINTS - 1u) - trigger_pos;
                scope_roll_trigger_pending = 1u;
                scope_roll_trigger_seq = scope_slow_roll_index_seq(hw_seq, hw_count, hit_index);
                scope_roll_trigger_target_seq = (uint16_t)(scope_roll_trigger_seq + post_samples);
            }
            if (scope_roll_trigger_pending) {
                uint16_t trigger_pos = scope_trigger_screen_sample_pos();
                uint16_t current_index = 0;

                if (!scope_seq_reached(hw_seq, scope_roll_trigger_target_seq)) {
                    return 0;
                }
                if (scope_slow_roll_seq_index(scope_roll_trigger_seq,
                                              hw_seq,
                                              hw_count,
                                              &current_index)) {
                    scope_roll_display_offset = (int16_t)trigger_pos - (int16_t)current_index;
                } else {
                    scope_roll_display_offset = 0;
                }
                scope_roll_trigger_pending = 0;
                scope_roll_trigger_seq = 0;
                scope_roll_trigger_target_seq = 0;
                if (ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE) {
                    ui.running = 0;
                }
            } else {
                return 0;
            }
        } else {
            scope_roll_trigger_reset();
        }
        scope_frame_valid = 1;
        ++scope_frame_id;
        scope_trigger_locked = 0;
        scope_trigger_offset = 0;
        scope_slow_roll_update_stats();
        scope_auto_zero_service();
        scope_auto_channel_service();
        scope_trace_invalidate();
        return 1;
    }

    if (!scope_hw_capture(scope_capture_samples, SCOPE_SAMPLE_BYTES, scope_safe_timebase())) {
        return 0;
    }
    if (!scope_auto_zero_active &&
        ui.scope_display == SCOPE_DISPLAY_YT &&
        scope_ch_enabled[scope_trigger_source_index()]) {
        trigger_found = scope_find_trigger_offset(scope_capture_samples, &trigger_offset);
    } else {
        trigger_found = 0;
    }
    if (!scope_trigger_accepts_capture(trigger_found)) {
        return 0;
    }
    scope_copy_capture_samples();

    scope_frame_valid = 1;
    ++scope_frame_id;
    if (ui.scope_display == SCOPE_DISPLAY_ROLL) {
        scope_roll_offset = (uint16_t)((scope_roll_offset + 19u) & (SCOPE_SAMPLE_COUNT - 1u));
    }
    scope_compute_stats();
    scope_trigger_locked = trigger_found ? 1u : 0u;
    scope_trigger_offset = trigger_found ? trigger_offset : 0;
    if (trigger_found && ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE) {
        ui.running = 0;
    }
    scope_auto_zero_service();
    scope_auto_channel_service();
    scope_trace_invalidate();
    return 1;
}

static void scope_apply_settings_keep_frame(uint8_t keep_frame) {
    scope_auto_zero_active = 0;
    scope_auto_zero_mask = 0;
    scope_auto_zero_stable_count = 0;
    if (!keep_frame) {
        scope_frame_valid = 0;
        scope_trigger_offset = 0;
        scope_trigger_locked = 0;
        scope_slow_roll_reset();
        scope_frame_age_ms = 0;
        scope_frame_interval_x10ms = 0;
    }
    scope_measure_hist_ready = 0;
    scope_measure_hist_pos = 0;
#if SCOPE_UI_SAFE_STUB
    if (!keep_frame) {
        scope_trace_cache[0].valid = 0;
        scope_trace_cache[1].valid = 0;
        scope_after_valid[0] = 0;
        scope_after_valid[1] = 0;
    }
    return;
#else
    if (!keep_frame) {
        scope_trace_invalidate();
    }
    scope_sanitize_state();
    scope_hw_slow_stop();
    scope_hw_configure_channels(ui.scope_timebase,
                                scope_vdiv_ch[0],
                                scope_vdiv_ch[1],
                                scope_coupling_dc[0],
                                scope_coupling_dc[1],
                                scope_bias_dac_for_channel(0),
                                scope_bias_dac_for_channel(1));
    if (scope_irq_roll_active()) {
        scope_hw_slow_start(ui.scope_timebase);
    }
#endif
}

static void scope_apply_settings(void) {
    scope_apply_settings_keep_frame(0);
}

static int16_t scope_scaled_sample_y(uint8_t ch, uint8_t raw, int16_t center, int16_t amp, int16_t min_y, int16_t max_y) {
    int32_t mv = scope_raw_delta_mv(ch, raw);
    int32_t vdiv = (int32_t)scope_vdiv_mv_for_channel(ch);
    int32_t pixels;
    int32_t y;

    if (vdiv <= 0) {
        vdiv = 1;
    }
    pixels = (mv * amp) / (vdiv * 3);
    y = (int32_t)center - pixels;
    if (y < min_y) {
        return min_y;
    }
    if (y > max_y) {
        return max_y;
    }
    return (int16_t)y;
}

static int16_t scope_sample_y(uint16_t x,
                              uint8_t ch2,
                              int16_t center,
                              int16_t amp,
                              uint16_t width,
                              int16_t min_y,
                              int16_t max_y) {
    if (!scope_frame_valid || !width) {
        int16_t y = scope_sim_y(x, ch2, center, amp);
        if (y < min_y) {
            return min_y;
        }
        if (y > max_y) {
            return max_y;
        }
        return y;
    }

    uint16_t visible_samples = scope_visible_sample_count();
    uint16_t idx = (uint16_t)((uint32_t)x * (visible_samples - 1u) / width);
    int32_t h_offset = scope_trigger_locked ? 0 : scope_h_pos_sample_offset();
    if (ui.scope_display == SCOPE_DISPLAY_ROLL) {
        idx = (uint16_t)(((uint32_t)((int32_t)idx + (int32_t)scope_roll_offset + h_offset)) &
                         (SCOPE_SAMPLE_COUNT - 1u));
    } else {
        idx = (uint16_t)(((uint32_t)((int32_t)idx + (int32_t)scope_trigger_offset + h_offset)) &
                         (SCOPE_SAMPLE_COUNT - 1u));
    }
    return scope_scaled_sample_y(ch2 ? 1u : 0u,
                                 scope_samples[(uint16_t)(idx * 2u + (ch2 ? 1u : 0u))],
                                 center,
                                 amp,
                                 min_y,
                                 max_y);
}

static uint8_t scope_sample_raw_at(uint16_t idx, uint8_t ch2) {
    idx &= (SCOPE_SAMPLE_COUNT - 1u);
    if (scope_frame_valid) {
        return scope_samples[(uint16_t)(idx * 2u + (ch2 ? 1u : 0u))];
    }

    uint16_t phase = (uint16_t)(((idx >> 3) + ui.scope_phase + (ch2 ? 64u : 0u)) & 0xFFu);
    int16_t sample = scope_sine_interp(phase);
    if (ch2) {
        sample = (int16_t)(sample * 3 / 4);
    }
    return (uint8_t)(128 + sample);
}

static uint16_t scope_dim_color(uint16_t color) {
    if (color == C_CH1) {
        return RGB565(88, 65, 26);
    }
    return RGB565(18, 72, 82);
}

static uint32_t scope_timebase_unit_ns_value(void) {
    return scope_timebase_unit_ns[scope_safe_timebase()];
}

static uint32_t scope_sample_period_ns(void) {
    if (scope_safe_timebase() <= 3u) {
        return SCOPE_FAST_HW_SAMPLE_NS;
    }

    uint32_t unit_ns = scope_timebase_unit_ns_value();

    return (unit_ns * 2u + 2u) / 5u;
}

static uint32_t scope_vdiv_mv_for_channel(uint8_t idx) {
    uint8_t vdiv;
    uint32_t mv;

    if (idx >= 2u) {
        idx = 0;
    }
    vdiv = scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
    mv = scope_vdiv_base_mv[vdiv];
    return scope_probe_x10[idx] ? mv * 10u : mv;
}

static uint16_t scope_range_code_for_channel(uint8_t idx) {
    uint8_t vdiv;

    if (idx >= 2u) {
        idx = 0;
    }
    vdiv = scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
    return scope_vdiv_range_code[vdiv] ? scope_vdiv_range_code[vdiv] : 1u;
}

static uint32_t scope_vdiv_mv(void) {
    return scope_vdiv_mv_for_channel(ui.active_ch == 2u ? 1u : 0u);
}

static void scope_format_u32(char *out, uint32_t value) {
    char tmp[11];
    uint8_t i = 0;
    uint8_t j = 0;

    if (!value) {
        out[0] = '0';
        out[1] = 0;
        return;
    }
    while (value && i < sizeof(tmp)) {
        tmp[i++] = (char)('0' + value % 10u);
        value /= 10u;
    }
    while (i) {
        out[j++] = tmp[--i];
    }
    out[j] = 0;
}

static void scope_text_copy(char *dst, const char *src, uint8_t max_len) {
    uint8_t i = 0;

    if (!max_len) {
        return;
    }
    while (i < (uint8_t)(max_len - 1u) && src[i]) {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = 0;
}

static uint8_t scope_text_len(const char *text) {
    uint8_t len = 0;

    while (text[len]) {
        ++len;
    }
    return len;
}

static void scope_format_delta_time(char out[14]) {
    uint8_t x0 = scope_cursor_x[0];
    uint8_t x1 = scope_cursor_x[1];
    uint8_t dx = x0 > x1 ? (uint8_t)(x0 - x1) : (uint8_t)(x1 - x0);
    uint32_t unit_ns = scope_timebase_unit_ns_value();
    char number[11];
    uint8_t n;

    out[0] = 'D';
    out[1] = 'T';
    out[2] = ' ';
    if (unit_ns < 100u) {
        uint32_t ns = (uint32_t)dx * SCOPE_X_DIVS * 10u * unit_ns / 255u;
        if (ns < 1000u) {
            scope_format_u32(number, ns);
            scope_text_copy(&out[3], number, 7);
            n = (uint8_t)(3u + scope_text_len(number));
            out[n++] = 'N';
            out[n++] = 'S';
            out[n] = 0;
            return;
        }
        uint32_t us_x10 = (ns + 50u) / 100u;
        scope_format_u32(number, us_x10 / 10u);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = '.';
        out[n++] = (char)('0' + us_x10 % 10u);
        out[n++] = 'U';
        out[n++] = 'S';
        out[n] = 0;
    } else if (unit_ns < 100000u) {
        uint32_t us_x10 = ((uint32_t)dx * SCOPE_X_DIVS * unit_ns + 1275u) / 2550u;
        scope_format_u32(number, us_x10 / 10u);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = '.';
        out[n++] = (char)('0' + us_x10 % 10u);
        out[n++] = 'U';
        out[n++] = 'S';
        out[n] = 0;
    } else if (unit_ns < 10000000u) {
        uint32_t ms_x10 = ((uint32_t)dx * SCOPE_X_DIVS * (unit_ns / 1000u) + 1275u) / 2550u;
        scope_format_u32(number, ms_x10 / 10u);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = '.';
        out[n++] = (char)('0' + ms_x10 % 10u);
        out[n++] = 'M';
        out[n++] = 'S';
        out[n] = 0;
    } else {
        uint32_t s_x10 = ((uint32_t)dx * SCOPE_X_DIVS * (unit_ns / 10000000u) + 127u) / 255u;
        scope_format_u32(number, s_x10 / 10u);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = '.';
        out[n++] = (char)('0' + s_x10 % 10u);
        out[n++] = 'S';
        out[n] = 0;
    }
}

static void scope_format_delta_level(char out[14]) {
    uint8_t y0 = scope_cursor_y[0];
    uint8_t y1 = scope_cursor_y[1];
    uint8_t dy = y0 > y1 ? (uint8_t)(y0 - y1) : (uint8_t)(y1 - y0);
    uint32_t mv = (uint32_t)dy * scope_vdiv_mv() * SCOPE_Y_DIVS / 255u;
    char number[11];
    uint8_t n;

    out[0] = 'D';
    out[1] = 'V';
    out[2] = ' ';
    if (mv < 1000u) {
        scope_format_u32(number, mv);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = 'M';
        out[n++] = 'V';
        out[n] = 0;
    } else {
        uint32_t v_x10 = (mv + 50u) / 100u;
        scope_format_u32(number, v_x10 / 10u);
        scope_text_copy(&out[3], number, 7);
        n = (uint8_t)(3u + scope_text_len(number));
        out[n++] = '.';
        out[n++] = (char)('0' + v_x10 % 10u);
        out[n++] = 'V';
        out[n] = 0;
    }
}

static void scope_format_mv(char out[10], uint32_t mv, const char *suffix) {
    char number[11];
    uint8_t n;

    if (mv < 1000u) {
        scope_format_u32(number, mv);
        scope_text_copy(out, number, 6);
        n = scope_text_len(number);
        out[n++] = 'M';
        out[n++] = 'V';
    } else {
        uint32_t v_x10 = (mv + 50u) / 100u;
        scope_format_u32(number, v_x10 / 10u);
        scope_text_copy(out, number, 6);
        n = scope_text_len(number);
        out[n++] = '.';
        out[n++] = (char)('0' + v_x10 % 10u);
        out[n++] = 'V';
    }
    while (*suffix && n < 9u) {
        out[n++] = *suffix++;
    }
    out[n] = 0;
}

static void scope_format_signed_mv(char out[11], int32_t mv) {
    char mag[10];
    uint32_t abs_mv;
    uint8_t pos = 0;

    if (mv < 0) {
        out[pos++] = '-';
        abs_mv = (uint32_t)(-mv);
    } else {
        out[pos++] = '+';
        abs_mv = (uint32_t)mv;
    }
    scope_format_mv(mag, abs_mv, "");
    for (uint8_t i = 0; mag[i] && pos < 10u; ++i) {
        out[pos++] = mag[i];
    }
    out[pos] = 0;
}

static void scope_format_measure_voltage(char out[12], int32_t mv) {
    char number[11];
    uint32_t abs_mv;
    uint32_t frac;
    uint8_t pos = 0;
    uint8_t max_int_chars;

    if (mv < 0) {
        out[pos++] = '-';
        abs_mv = (uint32_t)(-mv);
        max_int_chars = 4;
    } else {
        abs_mv = (uint32_t)mv;
        max_int_chars = 5;
    }
    scope_format_u32(number, abs_mv / 1000u);
    for (uint8_t i = 0; number[i] && i < max_int_chars; ++i) {
        out[pos++] = number[i];
    }
    frac = abs_mv % 1000u;
    out[pos++] = '.';
    out[pos++] = (char)('0' + (frac / 100u) % 10u);
    out[pos++] = (char)('0' + (frac / 10u) % 10u);
    out[pos++] = (char)('0' + frac % 10u);
    out[pos++] = 'V';
    out[pos] = 0;
}

#if SCOPE_UI_SAFE_STUB
static void scope_format_freq(char out[10]) {
    uint32_t hz = 1000000000u / ((uint32_t)SCOPE_SAMPLE_COUNT * scope_sample_period_ns());
    char number[11];
    uint8_t n;

    if (hz < 1000u) {
        scope_format_u32(number, hz);
        scope_text_copy(out, number, 6);
        n = scope_text_len(number);
        out[n++] = 'H';
        out[n++] = 'Z';
        out[n] = 0;
    } else {
        uint32_t khz_x10 = (hz + 50u) / 100u;
        scope_format_u32(number, khz_x10 / 10u);
        scope_text_copy(out, number, 6);
        n = scope_text_len(number);
        out[n++] = '.';
        out[n++] = (char)('0' + khz_x10 % 10u);
        out[n++] = 'K';
        out[n++] = 'H';
        out[n++] = 'Z';
        out[n] = 0;
    }
}
#endif

static void draw_scope_cursor_readout(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    (void)gx;
    (void)gy;
    (void)gw;
    (void)gh;
    (void)bg;
}

static void draw_scope_calibration_overlay(void) {
    uint16_t x = 38;
    uint16_t y = 78;
    uint16_t w = 244;
    uint16_t h = 84;
    uint16_t box_bg = C_TEXT;
    uint16_t box_fg = C_BG;
    uint16_t done;

    if (!scope_auto_zero_active) {
        return;
    }

    lcd_rect(x, y, w, h, box_bg);
    lcd_frame(x, y, w, h, C_SCOPE);
    lcd_rect(x, y, w, 4, C_SCOPE);
    lcd_text_center(x, (uint16_t)(y + 13u), w, "BASELINE CAL", box_fg, box_bg, 2);
    if ((scope_auto_zero_mask & 0x03u) == 0x03u) {
        uint16_t ch1_w = lcd_text_width("CH1", 2);
        uint16_t sep_w = lcd_text_width(" & ", 2);
        uint16_t ch2_w = lcd_text_width("CH2", 2);
        uint16_t tx = (uint16_t)(x + (w - ch1_w - sep_w - ch2_w) / 2u);
        lcd_text(tx, (uint16_t)(y + 38u), "CH1", C_CH1, box_bg, 2);
        tx = (uint16_t)(tx + ch1_w);
        lcd_text(tx, (uint16_t)(y + 38u), " & ", box_fg, box_bg, 2);
        tx = (uint16_t)(tx + sep_w);
        lcd_text(tx, (uint16_t)(y + 38u), "CH2", C_CH2, box_bg, 2);
    } else if (scope_auto_zero_mask & 0x02u) {
        lcd_text_center(x, (uint16_t)(y + 38u), w, "CH2", C_CH2, box_bg, 2);
    } else {
        lcd_text_center(x, (uint16_t)(y + 38u), w, "CH1", C_CH1, box_bg, 2);
    }
    lcd_text_center(x, (uint16_t)(y + 58u), w, "REMOVE PROBE  WAIT", C_PANEL_2, box_bg, 1);
    done = (uint16_t)(120u - (scope_auto_zero_steps_left > 120u ? 120u : scope_auto_zero_steps_left));
    lcd_frame((uint16_t)(x + 18u), (uint16_t)(y + 72u), (uint16_t)(w - 36u), 5, C_GRID);
    lcd_rect((uint16_t)(x + 19u),
             (uint16_t)(y + 73u),
             (uint16_t)(((uint32_t)(w - 38u) * done) / 120u),
             3,
             C_SCOPE);
}

static int32_t scope_raw_delta_mv(uint8_t idx, uint8_t raw) {
    int32_t delta = (int32_t)raw - 128;
    int32_t numerator;
    int32_t mv;
    uint16_t range;

    if (idx >= 2u) {
        idx = 0;
    }
    range = scope_range_code_for_channel(idx);
    numerator = delta * 754;
    mv = numerator >= 0 ?
        (numerator + (int32_t)range / 2) / (int32_t)range :
        (numerator - (int32_t)range / 2) / (int32_t)range;
    return scope_probe_x10[idx] ? mv * 10 : mv;
}

static uint32_t scope_raw_span_mv(uint8_t idx, uint8_t raw_span) {
    uint32_t mv;
    uint16_t range;

    if (idx >= 2u) {
        idx = 0;
    }
    range = scope_range_code_for_channel(idx);
    mv = ((uint32_t)raw_span * 754u + range / 2u) / range;
    return scope_probe_x10[idx] ? mv * 10u : mv;
}

static uint32_t scope_estimate_freq_hz_window(uint8_t idx, uint16_t start, uint16_t end, uint8_t min_raw, uint8_t max_raw) {
    uint8_t level;
    uint8_t prev;
    uint16_t first = 0u;
    uint16_t last = 0u;
    uint8_t crossings = 0;
    uint32_t span_samples;
    uint32_t period_ns;

    if (idx >= 2u) {
        idx = 0;
    }
    if (end > SCOPE_SAMPLE_COUNT) {
        end = SCOPE_SAMPLE_COUNT;
    }
    if (start + 2u >= end) {
        return 0;
    }
    if ((uint8_t)(max_raw - min_raw) < 8u) {
        return 0;
    }
    level = (uint8_t)(((uint16_t)min_raw + (uint16_t)max_raw) / 2u);
    prev = scope_samples[(uint16_t)(start * 2u + idx)];
    for (uint16_t i = (uint16_t)(start + 1u); i < end; ++i) {
        uint8_t cur = scope_samples[(uint16_t)(i * 2u + idx)];
        if (prev <= level && cur > level) {
            if (!crossings) {
                first = i;
            }
            last = i;
            if (crossings < 255u) {
                ++crossings;
            }
        }
        prev = cur;
    }
    if (crossings < 2u || last <= first) {
        return 0;
    }
    span_samples = (uint32_t)(last - first);
    period_ns = (span_samples * scope_sample_period_ns()) / (uint32_t)(crossings - 1u);
    if (!period_ns) {
        return 0;
    }
    return 1000000000u / period_ns;
}

static uint8_t scope_filtered_min_raw(uint8_t idx) {
    uint32_t sum = 0;

    if (idx >= 2u) {
        idx = 0;
    }
    if (!scope_measure_hist_ready) {
        return scope_min_raw[idx];
    }
    if (scope_safe_timebase() >= 17u) {
        return scope_measure_min_hist[idx][scope_measure_current_slot()];
    }
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        sum += scope_measure_min_hist[idx][i];
    }
    return (uint8_t)((sum + SCOPE_MEASURE_WINDOW / 2u) / SCOPE_MEASURE_WINDOW);
}

static uint8_t scope_filtered_max_raw(uint8_t idx) {
    uint32_t sum = 0;

    if (idx >= 2u) {
        idx = 0;
    }
    if (!scope_measure_hist_ready) {
        return scope_max_raw[idx];
    }
    if (scope_safe_timebase() >= 17u) {
        return scope_measure_max_hist[idx][scope_measure_current_slot()];
    }
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        sum += scope_measure_max_hist[idx][i];
    }
    return (uint8_t)((sum + SCOPE_MEASURE_WINDOW / 2u) / SCOPE_MEASURE_WINDOW);
}

static uint16_t scope_filtered_rms_raw(uint8_t idx) {
    uint32_t sum = 0;

    if (idx >= 2u) {
        idx = 0;
    }
    if (!scope_measure_hist_ready) {
        return (uint16_t)scope_isqrt_u32(scope_sq_sum_raw[idx] / SCOPE_SAMPLE_COUNT);
    }
    if (scope_safe_timebase() >= 17u) {
        return scope_measure_rms_hist[idx][scope_measure_current_slot()];
    }
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        sum += scope_measure_rms_hist[idx][i];
    }
    return (uint16_t)((sum + SCOPE_MEASURE_WINDOW / 2u) / SCOPE_MEASURE_WINDOW);
}

static uint32_t scope_rms_raw_mv(uint8_t idx, uint16_t rms_raw) {
    uint32_t mv;
    uint16_t range;

    if (idx >= 2u) {
        idx = 0;
    }
    range = scope_range_code_for_channel(idx);
    mv = ((uint32_t)rms_raw * 754u + range / 2u) / range;
    return scope_probe_x10[idx] ? mv * 10u : mv;
}

static void scope_sort_u32(uint32_t values[SCOPE_MEASURE_WINDOW]) {
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW - 1u; ++i) {
        uint8_t swapped = 0;
        for (uint8_t j = (uint8_t)(SCOPE_MEASURE_WINDOW - 1u); j > i; --j) {
            if (values[j - 1u] > values[j]) {
                uint32_t tmp = values[j - 1u];
                values[j - 1u] = values[j];
                values[j] = tmp;
                swapped = 1;
            }
        }
        if (!swapped) {
            break;
        }
    }
}

static uint32_t scope_filtered_freq_hz(uint8_t idx) {
    uint32_t values[SCOPE_MEASURE_WINDOW];
    uint32_t sum = 0;

    if (idx >= 2u) {
        idx = 0;
    }
    if (!scope_measure_hist_ready) {
        return scope_estimate_freq_hz_window(idx, 0, SCOPE_SAMPLE_COUNT, scope_min_raw[idx], scope_max_raw[idx]);
    }
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        values[i] = scope_measure_freq_hist[idx][i];
    }
    scope_sort_u32(values);
    for (uint8_t i = SCOPE_FREQ_AVG_START; i < (uint8_t)(SCOPE_FREQ_AVG_START + SCOPE_FREQ_AVG_COUNT); ++i) {
        sum += values[i];
    }
    return (sum + SCOPE_FREQ_AVG_COUNT / 2u) / SCOPE_FREQ_AVG_COUNT;
}

static void scope_format_frequency_reading(char out[12], uint32_t hz) {
    char number[11];
    uint32_t value_x10;
    const char *unit;
    uint8_t n;

    if (hz < 1000u) {
        value_x10 = hz * 10u;
        unit = "HZ";
    } else if (hz < 1000000u) {
        value_x10 = (hz + 50u) / 100u;
        unit = "KHZ";
    } else {
        value_x10 = (hz + 50000u) / 100000u;
        unit = "MHZ";
    }
    scope_format_u32(number, value_x10 / 10u);
    scope_text_copy(out, number, 8);
    n = scope_text_len(number);
    out[n++] = '.';
    out[n++] = (char)('0' + value_x10 % 10u);
    while (*unit && n < 11u) {
        out[n++] = *unit++;
    }
    out[n] = 0;
}

static void scope_measure_value_for(char out[12], uint8_t idx, uint8_t measure) {
    uint8_t min_raw;
    uint8_t max_raw;
    uint8_t span_raw;
    int32_t min_mv;
    int32_t max_mv;
    int32_t avg_mv;

    if (idx >= 2u) {
        idx = 0;
    }
    if (measure >= SCOPE_MEASURE_COUNT) {
        measure = SCOPE_MEASURE_VPP;
    }
    min_raw = scope_filtered_min_raw(idx);
    max_raw = scope_filtered_max_raw(idx);
    span_raw = max_raw >= min_raw ? (uint8_t)(max_raw - min_raw) : 0u;
    min_mv = scope_raw_delta_mv(idx, min_raw);
    max_mv = scope_raw_delta_mv(idx, max_raw);
    avg_mv = (min_mv + max_mv) / 2;
    if (measure == SCOPE_MEASURE_VPP) {
        scope_format_measure_voltage(out, (int32_t)scope_raw_span_mv(idx, span_raw));
    } else if (measure == SCOPE_MEASURE_VMAX) {
        scope_format_measure_voltage(out, max_mv);
    } else if (measure == SCOPE_MEASURE_VMIN) {
        scope_format_measure_voltage(out, min_mv);
    } else if (measure == SCOPE_MEASURE_VAVG) {
        scope_format_measure_voltage(out, avg_mv);
    } else if (measure == SCOPE_MEASURE_VRMS) {
        scope_format_measure_voltage(out, (int32_t)scope_rms_raw_mv(idx, scope_filtered_rms_raw(idx)));
    } else {
        scope_format_frequency_reading(out, scope_filtered_freq_hz(idx));
    }
}

static void draw_scope_measure_text(uint16_t left,
                                    uint16_t right,
                                    uint16_t y,
                                    uint8_t measure,
                                    uint8_t idx,
                                    uint16_t color,
                                    uint16_t bg,
                                    uint8_t align_right) {
    char value[12];
    const char *label;
    uint16_t width = right > left ? (uint16_t)(right - left) : 0u;
    uint16_t label_w;
    uint16_t value_w;
    uint16_t total_w;
    uint16_t x = left;
    uint8_t value_scale = 2;

    (void)bg;
    if (measure >= SCOPE_MEASURE_COUNT) {
        measure = SCOPE_MEASURE_VPP;
    }
    label = scope_measure_labels[measure];
    scope_measure_value_for(value, idx, measure);

    label_w = lcd_text_width(label, 1);
    value_w = lcd_text_width(value, value_scale);
    total_w = (uint16_t)(label_w + 4u + value_w);
    if (total_w > width) {
        value_scale = 1;
        value_w = lcd_text_width(value, value_scale);
        total_w = (uint16_t)(label_w + 4u + value_w);
    }
    if (align_right && total_w < width) {
        x = (uint16_t)(right - total_w);
    }

    lcd_text_transparent(x, (uint16_t)(y + (value_scale == 2u ? 4u : 0u)), label, color, 1);
    lcd_text_transparent((uint16_t)(x + label_w + 4u), y, value, color, value_scale);
}

static uint8_t scope_measure_enabled_count(uint8_t ch) {
    uint8_t count = 0;

    if (ch >= 2u || !scope_ch_enabled[ch]) {
        return 0;
    }
    for (uint8_t measure = 0; measure < SCOPE_MEASURE_COUNT; ++measure) {
        if (scope_measure_enabled_for(ch, measure)) {
            ++count;
        }
    }
    return count;
}

static void draw_scope_measure_readout(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    enum {
        ROW_H = 18,
        BOTTOM_PAD = 20,
    };
    uint16_t mid = (uint16_t)(gx + gw / 2u);
    uint16_t ch1_right = mid > 4u ? (uint16_t)(mid - 4u) : mid;
    uint16_t ch2_left = (uint16_t)(mid + 4u);
    uint16_t bottom_y = (uint16_t)(gy + gh - BOTTOM_PAD);

    if (!scope_measure_visible || !scope_frame_valid) {
        return;
    }
    scope_sanitize_state();
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        uint8_t count = scope_measure_enabled_count(ch);
        uint16_t y;

        if (!scope_ch_enabled[ch]) {
            continue;
        }
        if (!count) {
            continue;
        }
        y = (uint16_t)(bottom_y - (uint16_t)(count - 1u) * ROW_H);
        for (uint8_t measure = 0; measure < SCOPE_MEASURE_COUNT; ++measure) {
            if (!scope_measure_enabled_for(ch, measure)) {
                continue;
            }
            if (ch == 0u) {
                draw_scope_measure_text(gx, ch1_right, y, measure, ch, C_CH1, bg, 0);
            } else {
                draw_scope_measure_text(ch2_left, (uint16_t)(gx + gw), y, measure, ch, C_CH2, bg, 1);
            }
            y = (uint16_t)(y + ROW_H);
        }
    }
}

static uint16_t scope_channel_color(uint8_t ch) {
    return ch == 2u ? C_CH2 : C_CH1;
}

static uint16_t scope_channel_header_color(uint8_t ch) {
    uint8_t idx = ch == 2u ? 1u : 0u;

    if (!scope_ch_enabled[idx]) {
        return C_GRID;
    }
    return ui.active_ch == ch ? C_TEXT : scope_channel_color(ch);
}

static int16_t scope_channel_center(int16_t base, uint8_t ch2) {
    int8_t pos = scope_ch_pos[ch2 ? 1u : 0u];

    return (int16_t)(base + (int16_t)pos);
}

static void draw_scope_move_marker(uint16_t gx, int16_t y, uint16_t color, char label[2], uint16_t bg) {
    int16_t cy = y;

    if (cy < 7) {
        cy = 7;
    } else if (cy > (int16_t)(LCD_HEIGHT - 8u)) {
        cy = (int16_t)(LCD_HEIGHT - 8u);
    }

    for (int8_t dy = -5; dy <= 5; ++dy) {
        uint8_t mag = (uint8_t)(dy < 0 ? -dy : dy);
        uint8_t len = (uint8_t)(12u - mag);
        lcd_rect((uint16_t)(gx + 1u), (uint16_t)(cy + dy), len, 1, color);
    }
    lcd_rect((uint16_t)(gx + 2u), (uint16_t)(cy - 3), 5, 7, color);
    lcd_text((uint16_t)(gx + 3u), (uint16_t)(cy - 4), label, C_BG, color, 1);
    (void)bg;
}

static uint8_t scope_h_value_outside_visible(void) {
    uint8_t value_timebase = scope_h_value_timebase < SCOPE_TIMEBASE_COUNT ?
        scope_h_value_timebase :
        scope_safe_timebase();
    uint32_t value_unit = scope_timebase_unit_ns[value_timebase];
    uint32_t visible_unit = scope_timebase_unit_ns[scope_safe_timebase()];
    uint8_t mag = (uint8_t)(scope_h_value_pos < 0 ? -scope_h_value_pos : scope_h_value_pos);
    uint32_t whole;
    uint32_t rem;
    uint32_t base;
    uint8_t remaining;
    uint8_t extra = 0;
    uint32_t acc = 0;

    if (!mag || !visible_unit) {
        return 0;
    }
    whole = value_unit / visible_unit;
    rem = value_unit % visible_unit;
    if (whole > (uint32_t)SCOPE_H_POS_LIMIT / mag) {
        return 1;
    }
    base = whole * mag;
    if (base > SCOPE_H_POS_LIMIT) {
        return 1;
    }
    remaining = (uint8_t)(SCOPE_H_POS_LIMIT - base);
    if (remaining >= mag || !rem) {
        return 0;
    }
    for (uint8_t i = 0; i < mag; ++i) {
        acc += rem;
        if (acc >= visible_unit) {
            acc -= visible_unit;
            ++extra;
        }
    }
    return (uint8_t)(extra > remaining || (extra == remaining && acc));
}

static void draw_scope_hpos_label(int16_t x, uint16_t gy, uint16_t bg, uint8_t outside) {
    int16_t cx = x;
    const char *label = outside ? (scope_h_value_pos < 0 ? "<<" : ">>") : scope_h_pos_label();
    uint16_t label_w = lcd_text_width(label, 1);
    uint16_t label_x;
    uint16_t label_y = gy >= 9u ? (uint16_t)(gy - 9u) : 0u;

    if (!outside && !scope_h_value_pos) {
        return;
    }
    if (cx < 1) {
        cx = 1;
    } else if (cx > (int16_t)(LCD_WIDTH - 2u)) {
        cx = (int16_t)(LCD_WIDTH - 2u);
    }
    label_x = cx > (int16_t)(label_w / 2u) ? (uint16_t)(cx - (int16_t)(label_w / 2u)) : 0u;
    if (label_x > LCD_WIDTH - label_w) {
        label_x = (uint16_t)(LCD_WIDTH - label_w);
    }
    lcd_text(label_x, label_y, label, C_TEXT, bg, 1);
}

static void draw_scope_hpos_line(int16_t x, uint16_t gy, uint16_t bg, uint8_t outside) {
    int16_t cx = x;

    if (cx < 1) {
        cx = 1;
    } else if (cx > (int16_t)(LCD_WIDTH - 2u)) {
        cx = (int16_t)(LCD_WIDTH - 2u);
    }
    lcd_rect((uint16_t)cx, gy, 2, 10, C_TEXT);
    draw_scope_hpos_label(cx, gy, bg, outside);
}

static void draw_scope_hpos_marker(int16_t x, uint16_t gy) {
    int16_t cx = x;
    uint16_t top_y = gy >= 8u ? (uint16_t)(gy - 8u) : 0u;
    uint16_t color = C_TEXT;

    if (cx < 5) {
        cx = 5;
    } else if (cx > (int16_t)(LCD_WIDTH - 6u)) {
        cx = (int16_t)(LCD_WIDTH - 6u);
    }

    for (int8_t dx = -4; dx <= 4; ++dx) {
        uint8_t mag = (uint8_t)(dx < 0 ? -dx : dx);
        uint8_t len = (uint8_t)(9u - mag);
        lcd_rect((uint16_t)(cx + dx), top_y, 1, len, color);
    }
    lcd_rect((uint16_t)(cx - 2), (uint16_t)(top_y + 2u), 5, 4, color);
}

static void draw_scope_hpos_indicator(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    int16_t center;
    int16_t x;
    uint16_t range;
    uint8_t outside;

    if (ui.scope_display == SCOPE_DISPLAY_XY) {
        return;
    }

    range = (uint16_t)(gw / 2u - 1u);
    center = (int16_t)(gx + gw / 2u);
    x = (int16_t)(center + ((int16_t)scope_h_pos * (int16_t)range) / SCOPE_H_POS_LIMIT);
    outside = scope_h_value_outside_visible();
    if (ui.scope_move_mode && ui.scope_move_sel == SCOPE_MOVE_SEL_X) {
        draw_scope_hpos_marker(x, gy);
        if (outside) {
            draw_scope_hpos_label(x, gy, bg, 1);
        }
    } else {
        draw_scope_hpos_line(x, gy, bg, outside);
    }

    (void)center;
    (void)range;
    (void)gh;
    (void)bg;
}

static void draw_scope_channel_markers(uint16_t gx,
                                       uint16_t gy,
                                       uint16_t gh,
                                       int16_t ch1_center,
                                       int16_t ch2_center,
                                       uint16_t bg) {
    int16_t centers[2] = {ch1_center, ch2_center};

    for (uint8_t i = 0; i < 2u; ++i) {
        int16_t y = centers[i];
        uint16_t color = scope_channel_color((uint8_t)(i + 1u));
        char label[2] = {(char)('1' + i), 0};

        if (!scope_ch_enabled[i]) {
            continue;
        }
        if (y < (int16_t)(gy + 2u)) {
            y = (int16_t)(gy + 2u);
        } else if (y > (int16_t)(gy + gh - 8u)) {
            y = (int16_t)(gy + gh - 8u);
        }
        if (ui.scope_move_mode &&
            ui.active_ch == i + 1u &&
            (ui.scope_move_sel == SCOPE_MOVE_SEL_CHANNEL ||
             ui.scope_move_sel == SCOPE_MOVE_SEL_Y)) {
            draw_scope_move_marker(gx, y, color, label, bg);
            continue;
        }
        lcd_rect(gx, (uint16_t)y, 6, 2, color);
        lcd_text(gx >= 7u ? (uint16_t)(gx - 7u) : 0u,
                 y >= 3 ? (uint16_t)(y - 3) : 0u,
                 label,
                 ui.active_ch == i + 1u ? C_TEXT : color,
                 ui.chrome_visible ? C_BG : bg,
                 1);
    }
}

static void draw_scope_trigger_line(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    int16_t y;
    uint8_t src = scope_trigger_source_index();
    uint16_t color = ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE ? C_WARN : scope_channel_color((uint8_t)(src + 1u));
    char level[11];
    char edge[2] = {ui.scope_trigger_edge ? 'F' : 'R', 0};
    int32_t mv = scope_raw_delta_mv(src, ui.scope_trigger_level);
    int16_t center;
    int16_t delta;
    int16_t amp = ui.chrome_visible ? 48 : 64;

    if (ui.scope_display == SCOPE_DISPLAY_XY ||
        !scope_ch_enabled[src]) {
        return;
    }
    center = scope_channel_center(scope_channel_base_center_for(src), src);
    delta = (int16_t)ui.scope_trigger_level - 128;
    y = (int16_t)(center - (delta * amp) / 96);
    if (y < (int16_t)(gy + 1u)) {
        y = (int16_t)(gy + 1u);
    } else if (y > (int16_t)(gy + gh - 2u)) {
        y = (int16_t)(gy + gh - 2u);
    }
    for (uint16_t x = (uint16_t)(gx + 1u); x < (uint16_t)(gx + gw - 1u); x = (uint16_t)(x + 8u)) {
        lcd_rect(x, (uint16_t)y, 5, 1, color);
    }
    scope_format_signed_mv(level, mv);
    lcd_text((uint16_t)(gx + gw - 58u),
             (uint16_t)(y > (int16_t)(gy + 9u) ? y - 8 : y + 2),
             level,
             color,
             bg,
             1);
    lcd_text((uint16_t)(gx + gw - 10u),
             (uint16_t)(y > (int16_t)(gy + 9u) ? y - 8 : y + 2),
             edge,
             color,
             bg,
             1);
}

#if SCOPE_UI_SAFE_STUB
static int16_t scope_stub_amp(uint8_t ch2);

static void draw_scope_measurements(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    char ch1[10];
    char ch2[10];
    char freq[10];
    uint32_t ch1_mv = (uint32_t)scope_stub_amp(0) * 16u * scope_vdiv_mv_for_channel(0) / gh;
    uint32_t ch2_mv = (uint32_t)scope_stub_amp(1) * 16u * scope_vdiv_mv_for_channel(1) / gh;

    scope_format_mv(ch1, ch1_mv, "PP");
    scope_format_mv(ch2, ch2_mv, "PP");
    scope_format_freq(freq);
    if (scope_ch_enabled[0]) {
        lcd_text((uint16_t)(gx + 5u), (uint16_t)(gy + 15u), ch1, C_CH1, bg, 1);
    }
    if (scope_ch_enabled[1]) {
        lcd_text((uint16_t)(gx + 76u), (uint16_t)(gy + 15u), ch2, C_CH2, bg, 1);
    }
    lcd_text((uint16_t)(gx + gw - 58u), (uint16_t)(gy + 15u), freq, C_TEXT, bg, 1);
}
#endif

#if SCOPE_UI_SAFE_STUB
static void draw_scope_stub_grid(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    lcd_rect(gx, gy, gw, gh, bg);
    lcd_frame(gx, gy, gw, gh, C_GRID);
    for (uint8_t div = 1; div < SCOPE_X_DIVS; ++div) {
        uint16_t x = (uint16_t)(gx + ((uint32_t)gw * div) / SCOPE_X_DIVS);
        lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), C_GRID);
    }
    for (uint8_t div = 1; div < SCOPE_Y_DIVS; ++div) {
        uint16_t y = (uint16_t)(gy + ((uint32_t)gh * div) / SCOPE_Y_DIVS);
        lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, C_GRID);
    }
    lcd_rect((uint16_t)(gx + ((uint32_t)gw * 6u) / SCOPE_X_DIVS), (uint16_t)(gy + 1u), 2, (uint16_t)(gh - 2u), RGB565(67, 85, 95));
    lcd_rect((uint16_t)(gx + 1u), (uint16_t)(gy + ((uint32_t)gh * 4u) / SCOPE_Y_DIVS), (uint16_t)(gw - 2u), 2, RGB565(67, 85, 95));
}

static int16_t scope_stub_y(uint16_t px, uint16_t width, int16_t center, int16_t amp, uint8_t phase_offset) {
    uint8_t phase = width ? (uint8_t)(((uint32_t)px * 256u / width + phase_offset) & 0xFFu) : phase_offset;
    return (int16_t)(center - scope_sine_interp(phase) * amp / 64);
}

static int16_t scope_stub_amp(uint8_t ch2) {
    uint8_t vdiv = scope_safe_vdiv();

    if (vdiv == 1u) {
        return ch2 ? 27 : 36;
    }
    if (vdiv == 2u) {
        return ch2 ? 21 : 28;
    }
    return ch2 ? 34 : 46;
}

static void draw_scope_stub_trace(uint16_t gx,
                                  uint16_t gy,
                                  uint16_t gw,
                                  uint16_t gh,
                                  int16_t center,
                                  int16_t amp,
                                  uint16_t color,
                                  uint8_t phase_offset) {
    uint16_t plot_w = (uint16_t)(gw - 3u);
    int16_t x0 = (int16_t)(gx + 1u);
    int16_t prev_x = x0;
    int16_t prev_y = scope_stub_y(0, plot_w, center, amp, phase_offset);

    (void)gy;
    (void)gh;
    for (uint16_t px = 4; px <= plot_w; px = (uint16_t)(px + 4u)) {
        int16_t x = (int16_t)(x0 + px);
        int16_t y = scope_stub_y(px, plot_w, center, amp, phase_offset);
        lcd_line(prev_x, prev_y, x, y, color);
        prev_x = x;
        prev_y = y;
    }
}
#endif

static void draw_scope_afterglow_trace(uint8_t ch,
                                       int16_t x0,
                                       uint16_t width,
                                       uint16_t color) {
    if (!ui.scope_afterglow || !scope_after_valid[ch] || scope_after_count[ch] < 2u) {
        return;
    }
    if (ch >= 2u || !scope_ch_enabled[ch]) {
        return;
    }

    int16_t prev_x = x0;
    int16_t prev_y = scope_after_y[ch][0];
    uint16_t i = SCOPE_TRACE_STEP;
    uint16_t dim = scope_dim_color(color);

    for (uint8_t point = 1; point < scope_after_count[ch]; ++point, i = (uint16_t)(i + SCOPE_TRACE_STEP)) {
        if (i > width + SCOPE_TRACE_STEP) {
            break;
        }
        int16_t x = (int16_t)(x0 + i);
        int16_t y = scope_after_y[ch][point];
        lcd_line(prev_x, prev_y, x, y, dim);
        prev_x = x;
        prev_y = y;
    }
}

static void draw_scope_xy(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t color) {
    int16_t prev_x = -1;
    int16_t prev_y = -1;
    uint16_t start = ui.scope_display == SCOPE_DISPLAY_ROLL ? scope_roll_offset : 0u;

    if (!scope_ch_enabled[0] || !scope_ch_enabled[1]) {
        return;
    }
    for (uint16_t i = 0; i < SCOPE_SAMPLE_COUNT; i = (uint16_t)(i + 16u)) {
        uint16_t idx = (uint16_t)((start + i) & (SCOPE_SAMPLE_COUNT - 1u));
        uint8_t raw_x = scope_sample_raw_at(idx, 0);
        uint8_t raw_y = scope_sample_raw_at(idx, 1);
        int16_t x = (int16_t)(gx + 2u + (uint32_t)raw_x * (gw - 5u) / 255u);
        int16_t y = (int16_t)(gy + gh - 3u - (uint32_t)raw_y * (gh - 5u) / 255u);

        if (prev_x >= 0) {
            lcd_line(prev_x, prev_y, x, y, color);
        } else {
            lcd_rect((uint16_t)x, (uint16_t)y, 1, 1, color);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void draw_scope_cursors(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    (void)bg;
    if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        for (uint8_t i = 0; i < 2u; ++i) {
            uint16_t x = (uint16_t)(gx + 1u + (uint32_t)scope_cursor_x[i] * (gw - 3u) / 255u);
            uint16_t color = i == ui.scope_cursor_sel ? C_TEXT : C_MUTED;
            lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), color);
            lcd_text((uint16_t)(x > 8u ? x - 7u : x), (uint16_t)(gy + 3u), i ? "T2" : "T1", color, bg, 1);
        }
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        for (uint8_t i = 0; i < 2u; ++i) {
            uint16_t y = (uint16_t)(gy + 1u + (uint32_t)scope_cursor_y[i] * (gh - 3u) / 255u);
            uint16_t color = i == ui.scope_cursor_sel ? C_TEXT : C_MUTED;
            lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, color);
            lcd_text((uint16_t)(gx + 4u), (uint16_t)(y > 8u ? y - 7u : y), i ? "Y2" : "Y1", color, bg, 1);
        }
    }
}

static scope_trace_cache_t *scope_trace_prepare(uint8_t ch2,
                                                int16_t x0,
                                                int16_t center,
                                                int16_t amp,
                                                uint16_t width,
                                                int16_t min_y,
                                                int16_t max_y) {
    scope_trace_cache_t *cache = &scope_trace_cache[ch2 ? 1u : 0u];
    if (cache->valid &&
        cache->ch2 == ch2 &&
        cache->timebase == ui.scope_timebase &&
        cache->phase == ui.scope_phase &&
        cache->frame_id == scope_frame_id &&
        cache->width == width &&
        cache->x0 == x0 &&
        cache->center == center &&
        cache->amp == amp &&
        cache->min_y == min_y &&
        cache->max_y == max_y) {
        return cache;
    }

    uint8_t count = 0;
    for (uint16_t x = 0; x <= width && count < SCOPE_TRACE_MAX_POINTS; x = (uint16_t)(x + SCOPE_TRACE_STEP)) {
        cache->y[count++] = scope_sample_y(x, ch2, center, amp, width, min_y, max_y);
        if ((uint16_t)(x + SCOPE_TRACE_STEP) < x) {
            break;
        }
    }

    cache->valid = 1;
    cache->ch2 = ch2;
    cache->timebase = ui.scope_timebase;
    cache->count = count;
    cache->phase = ui.scope_phase;
    cache->frame_id = scope_frame_id;
    cache->width = width;
    cache->x0 = x0;
    cache->center = center;
    cache->amp = amp;
    cache->min_y = min_y;
    cache->max_y = max_y;
    return cache;
}

static int16_t scope_raw_to_y(uint8_t ch, uint8_t raw, int16_t center, int16_t amp, uint16_t gy, uint16_t gh) {
    int16_t min_y = (int16_t)(gy + 1u);
    int16_t max_y = (int16_t)(gy + gh - 2u);

    return scope_scaled_sample_y(ch, raw, center, amp, min_y, max_y);
}

static void draw_scope_slow_roll_trace(uint8_t ch,
                                       int16_t x0,
                                       uint16_t width,
                                       int16_t center,
                                       int16_t amp,
                                       uint16_t gy,
                                       uint16_t gh,
                                       uint16_t color) {
    if (ch >= 2u || !scope_ch_enabled[ch] || !scope_slow_roll_count) {
        return;
    }

    uint16_t count = scope_slow_roll_count;
    int16_t prev_x = 0;
    int16_t prev_y = 0;
    uint8_t have_prev = 0;

    for (uint16_t i = 0; i < count; ++i) {
        int16_t display_i = (int16_t)i + scope_roll_display_offset;
        int16_t x;
        int16_t y = scope_raw_to_y(ch, scope_slow_roll_raw_at(ch, i), center, amp, gy, gh);

        if (display_i < 0 || display_i >= (int16_t)SCOPE_SLOW_ROLL_MAX_POINTS) {
            have_prev = 0;
            continue;
        }
        x = (int16_t)(x0 + ((uint32_t)(uint16_t)display_i * width) / (SCOPE_SLOW_ROLL_MAX_POINTS - 1u));
        if (!have_prev) {
            lcd_rect((uint16_t)x, (uint16_t)y, 2, 2, color);
            prev_x = x;
            prev_y = y;
            have_prev = 1;
            continue;
        }
        lcd_line(prev_x, prev_y, x, y, color);
        if (!ch) {
            lcd_line(prev_x, (int16_t)(prev_y + 1), x, (int16_t)(y + 1), color);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void draw_scope_trace(uint16_t color,
                             uint8_t ch2,
                             int16_t x0,
                             int16_t center,
                             int16_t amp,
                             uint16_t width,
                             uint16_t gy,
                             uint16_t gh) {
    scope_trace_cache_t *trace;

    if (!scope_ch_enabled[ch2 ? 1u : 0u]) {
        return;
    }
    trace = scope_trace_prepare(ch2,
                                x0,
                                center,
                                amp,
                                width,
                                (int16_t)(gy + 1u),
                                (int16_t)(gy + gh - 2u));
    if (trace->count < 2u) {
        return;
    }

    int16_t prev_x = x0;
    int16_t prev_y = trace->y[0];
    uint16_t i = SCOPE_TRACE_STEP;
    for (uint8_t point = 1; point < trace->count; ++point, i = (uint16_t)(i + SCOPE_TRACE_STEP)) {
        int16_t x = (int16_t)(x0 + i);
        int16_t y = trace->y[point];
        lcd_line(prev_x, prev_y, x, y, color);
        if (!ch2) {
            lcd_line(prev_x, (int16_t)(prev_y + 1), x, (int16_t)(y + 1), color);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void draw_scope_chrome_live(void) {
    uint16_t gx = 10;
    uint16_t gy = 55;
    uint16_t gw = 300;
    uint16_t gh = 124;
    uint16_t grid_bg = RGB565(6, 12, 16);

#if SCOPE_UI_SAFE_STUB
    scope_sanitize_state();
    int16_t ch1_center = scope_channel_center(91, 0);
    int16_t ch2_center = scope_channel_center(118, 1);
    lcd_rect(0, 55, LCD_WIDTH, 140, C_BG);
    draw_scope_stub_grid(gx, gy, gw, gh, grid_bg);
    if (ui.scope_display == SCOPE_DISPLAY_XY) {
        draw_scope_xy(gx, gy, gw, gh, C_SCOPE);
    } else {
        if (ui.scope_afterglow) {
            draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, scope_stub_amp(0), scope_dim_color(C_CH1), (uint8_t)(ui.scope_phase - 18u));
            draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, scope_stub_amp(1), scope_dim_color(C_CH2), (uint8_t)(ui.scope_phase + 46u));
        }
        draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, scope_stub_amp(0), C_CH1, (uint8_t)ui.scope_phase);
        draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, scope_stub_amp(1), C_CH2, (uint8_t)(ui.scope_phase + 64u));
        draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
    }
    draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
    draw_scope_cursors(gx, gy, gw, gh, grid_bg);
    draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
    draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
    draw_scope_measurements(gx, gy, gw, gh, grid_bg);
    lcd_text(18, 61, "CH1", scope_channel_header_color(1), grid_bg, 1);
    lcd_text(262, 61, "CH2", scope_channel_header_color(2), grid_bg, 1);
    draw_scope_scale_status(182, C_BG);
    return;
#endif

    lcd_rect(0, 55, LCD_WIDTH, 124, C_BG);
    lcd_rect(gx, gy, gw, gh, grid_bg);
    lcd_frame(gx, gy, gw, gh, C_GRID);
    for (uint8_t div = 1; div < SCOPE_X_DIVS; ++div) {
        uint16_t x = (uint16_t)(gx + ((uint32_t)gw * div) / SCOPE_X_DIVS);
        lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), C_GRID);
    }
    for (uint8_t div = 1; div < SCOPE_Y_DIVS; ++div) {
        uint16_t y = (uint16_t)(gy + ((uint32_t)gh * div) / SCOPE_Y_DIVS);
        lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, C_GRID);
    }
    lcd_rect((uint16_t)(gx + ((uint32_t)gw * 6u) / SCOPE_X_DIVS), (uint16_t)(gy + 1u), 2, (uint16_t)(gh - 2u), RGB565(67, 85, 95));
    lcd_rect((uint16_t)(gx + 1u), (uint16_t)(gy + ((uint32_t)gh * 4u) / SCOPE_Y_DIVS), (uint16_t)(gw - 2u), 2, RGB565(67, 85, 95));

    if (scope_frame_valid || !scope_hw_enabled()) {
        if (ui.scope_display == SCOPE_DISPLAY_XY) {
            draw_scope_xy(gx, gy, gw, gh, C_SCOPE);
        } else {
            int16_t plot_x0 = (int16_t)(gx + 1u);
            uint16_t plot_w = (uint16_t)(gw - 3u);
            int16_t ch1_center = scope_channel_center(91, 0);
            int16_t ch2_center = scope_channel_center(111, 1);
            if (scope_slow_roll_active()) {
                draw_scope_slow_roll_trace(0, plot_x0, plot_w, ch1_center, 48, gy, gh, C_CH1);
                draw_scope_slow_roll_trace(1, plot_x0, plot_w, ch2_center, 48, gy, gh, C_CH2);
            } else {
                draw_scope_afterglow_trace(0, plot_x0, plot_w, C_CH1);
                draw_scope_afterglow_trace(1, plot_x0, plot_w, C_CH2);
                draw_scope_trace(C_CH1, 0, plot_x0, ch1_center, 48, plot_w, gy, gh);
                draw_scope_trace(C_CH2, 1, plot_x0, ch2_center, 48, plot_w, gy, gh);
            }
            draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
        }
        draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
        draw_scope_cursors(gx, gy, gw, gh, grid_bg);
        draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
        draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
        draw_scope_measure_readout((uint16_t)(gx + 5u), gy, (uint16_t)(gw - 10u), gh, grid_bg);
    } else {
        lcd_text_center(gx, 103, gw, scope_hw_ready() ? "WAIT" : "FPGA", C_MUTED, grid_bg, 2);
    }

    lcd_text(18, 61, "CH1", scope_channel_header_color(1), grid_bg, 1);
    lcd_text(262, 61, "CH2", scope_channel_header_color(2), grid_bg, 1);

    lcd_rect(0, 180, LCD_WIDTH, 19, C_BG);
    draw_scope_scale_status(182, C_BG);
}

static void draw_scope(void) {
    lcd_rect(0, Y_BODY, LCD_WIDTH, H_BODY, C_BG);
    draw_scope_chrome_live();
}

static void draw_scope_immersive(void) {
    const uint16_t grid_bg = RGB565(4, 8, 11);
    uint16_t gx = 8;
    uint16_t gy = 28;
    uint16_t gw = 304;
    uint16_t gh = 190;

    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, grid_bg);
    draw_micro_status(6, 6, grid_bg);
    draw_battery_status_overlay(grid_bg);

#if SCOPE_UI_SAFE_STUB
    scope_sanitize_state();
    int16_t ch1_center = scope_channel_center((int16_t)(gy + gh / 2u - 30u), 0);
    int16_t ch2_center = scope_channel_center((int16_t)(gy + gh / 2u + 34u), 1);
    draw_scope_stub_grid(gx, gy, gw, gh, grid_bg);
    if (ui.scope_display == SCOPE_DISPLAY_XY) {
        draw_scope_xy(gx, gy, gw, gh, C_SCOPE);
    } else {
        if (ui.scope_afterglow) {
            draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, (int16_t)(scope_stub_amp(0) + 16), scope_dim_color(C_CH1), (uint8_t)(ui.scope_phase - 18u));
            draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, (int16_t)(scope_stub_amp(1) + 12), scope_dim_color(C_CH2), (uint8_t)(ui.scope_phase + 46u));
        }
        draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, (int16_t)(scope_stub_amp(0) + 16), C_CH1, (uint8_t)ui.scope_phase);
        draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, (int16_t)(scope_stub_amp(1) + 12), C_CH2, (uint8_t)(ui.scope_phase + 64u));
        draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
    }
    draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
    draw_scope_cursors(gx, gy, gw, gh, grid_bg);
    draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
    draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
    draw_scope_measurements(gx, gy, gw, gh, grid_bg);
    lcd_text(16, 34, "CH1", scope_channel_header_color(1), grid_bg, 1);
    lcd_text(276, 34, "CH2", scope_channel_header_color(2), grid_bg, 1);
    draw_scope_scale_status(220, grid_bg);
    return;
#endif

    lcd_frame(gx, gy, gw, gh, C_GRID);
    for (uint8_t div = 1; div < SCOPE_X_DIVS; ++div) {
        uint16_t x = (uint16_t)(gx + ((uint32_t)gw * div) / SCOPE_X_DIVS);
        lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), C_GRID);
    }
    for (uint8_t div = 1; div < SCOPE_Y_DIVS; ++div) {
        uint16_t y = (uint16_t)(gy + ((uint32_t)gh * div) / SCOPE_Y_DIVS);
        lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, C_GRID);
    }
    lcd_rect((uint16_t)(gx + ((uint32_t)gw * 6u) / SCOPE_X_DIVS), (uint16_t)(gy + 1u), 2, (uint16_t)(gh - 2u), RGB565(67, 85, 95));
    lcd_rect((uint16_t)(gx + 1u), (uint16_t)(gy + ((uint32_t)gh * 4u) / SCOPE_Y_DIVS), (uint16_t)(gw - 2u), 2, RGB565(67, 85, 95));

    if (scope_frame_valid || !scope_hw_enabled()) {
        if (ui.scope_display == SCOPE_DISPLAY_XY) {
            draw_scope_xy(gx, gy, gw, gh, C_SCOPE);
        } else {
            int16_t plot_x0 = (int16_t)(gx + 1u);
            uint16_t plot_w = (uint16_t)(gw - 3u);
            int16_t ch1_center = scope_channel_center((int16_t)(gy + gh / 2u - 28u), 0);
            int16_t ch2_center = scope_channel_center((int16_t)(gy + gh / 2u + 28u), 1);
            if (scope_slow_roll_active()) {
                draw_scope_slow_roll_trace(0, plot_x0, plot_w, ch1_center, 64, gy, gh, C_CH1);
                draw_scope_slow_roll_trace(1, plot_x0, plot_w, ch2_center, 64, gy, gh, C_CH2);
            } else {
                draw_scope_afterglow_trace(0, plot_x0, plot_w, C_CH1);
                draw_scope_afterglow_trace(1, plot_x0, plot_w, C_CH2);
                draw_scope_trace(C_CH1, 0, plot_x0, ch1_center, 64, plot_w, gy, gh);
                draw_scope_trace(C_CH2, 1, plot_x0, ch2_center, 64, plot_w, gy, gh);
            }
            draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
        }
        draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
        draw_scope_cursors(gx, gy, gw, gh, grid_bg);
        draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
        draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
        draw_scope_measure_readout((uint16_t)(gx + 6u), gy, (uint16_t)(gw - 12u), gh, grid_bg);
    } else {
        lcd_text_center(gx, (uint16_t)(gy + 82u), gw, scope_hw_ready() ? "WAIT" : "FPGA", C_MUTED, grid_bg, 2);
    }

    draw_scope_scale_status(220, grid_bg);
}

static uint8_t gen_clamped_duty(void) {
    uint8_t duty = ui.gen_duty_percent;

    if (duty < 1u) {
        duty = 1u;
    } else if (duty > 100u) {
        duty = 100u;
    }
    return duty;
}

static uint8_t gen_clamped_amp_tenths(void) {
    uint8_t amp_tenths = ui.gen_amp_tenths_v;

    if (amp_tenths < 1u) {
        amp_tenths = 1u;
    } else if (amp_tenths > 33u) {
        amp_tenths = 33u;
    }
    return amp_tenths;
}

static void gen_cycle_wave(int8_t dir) {
    uint8_t idx = 0;
    uint8_t count = (uint8_t)sizeof(gen_wave_order);

    for (uint8_t i = 0; i < count; ++i) {
        if (gen_wave_order[i] == ui.gen_wave) {
            idx = i;
            break;
        }
    }

    if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % count);
    } else {
        idx = idx == 0u ? (uint8_t)(count - 1u) : (uint8_t)(idx - 1u);
    }
    ui.gen_wave = gen_wave_order[idx];
    gen_normalize_param();
}

static uint8_t gen_wave_uses_duty(void) {
    return ui.gen_wave == SIGGEN_WAVE_SQUARE ||
           ui.gen_wave == SIGGEN_WAVE_TRIANGLE ||
           ui.gen_wave == SIGGEN_WAVE_SINKER_PULSE;
}

static uint8_t gen_param_available(uint8_t param) {
    return param != GEN_PARAM_DUTY || gen_wave_uses_duty();
}

static void gen_normalize_param(void) {
    if (ui.gen_param >= GEN_PARAM_COUNT || !gen_param_available(ui.gen_param)) {
        ui.gen_param = GEN_PARAM_FREQ;
    }
}

static uint8_t gen_preview_cycles_for(uint32_t freq_hz) {
    if (freq_hz < 100u) {
        return 1;
    }
    if (freq_hz < 1000u) {
        return 2;
    }
    if (freq_hz < 10000u) {
        return 3;
    }
    if (freq_hz < 100000u) {
        return 5;
    }
    if (freq_hz < 1000000u) {
        return 7;
    }
    return 9;
}

static uint8_t gen_preview_cycles(void) {
    return gen_preview_cycles_for(ui.gen_freq_hz);
}

static uint16_t gen_preview_exp_rise(uint8_t phase) {
    uint32_t x = phase;
    return (uint16_t)((x * x * x) / (255u * 255u));
}

static uint16_t gen_preview_lorentz(uint8_t phase) {
    int32_t x = (int32_t)phase - 128;
    uint32_t g2 = 23u * 23u;
    uint32_t x2 = (uint32_t)(x * x);
    return (uint16_t)(255u * g2 / (x2 + g2));
}

static uint16_t gen_preview_sample(uint8_t phase, uint16_t point) {
    uint8_t duty = gen_clamped_duty();
    uint16_t split = (uint16_t)((uint32_t)duty * 255u / 100u);
    uint16_t value;

    if (ui.gen_wave == SIGGEN_WAVE_SQUARE && duty >= 100u) {
        return 255u;
    }
    if (split < 1u) {
        split = 1u;
    } else if (split > 254u) {
        split = 254u;
    }

    if (ui.gen_wave == SIGGEN_WAVE_SQUARE) {
        return phase < split ? 255u : 0u;
    } else if (ui.gen_wave == SIGGEN_WAVE_SAW) {
        return phase;
    } else if (ui.gen_wave == SIGGEN_WAVE_TRIANGLE) {
        if (phase < split) {
            return (uint16_t)((uint32_t)phase * 255u / split);
        }
        return (uint16_t)((uint32_t)(255u - phase) * 255u / (255u - split));
    } else if (ui.gen_wave == SIGGEN_WAVE_HALF) {
        int16_t signed_sample = phase < 128u ? scope_sine_interp(phase) : 0;
        if (signed_sample <= 0) {
            return 0;
        }
        return (uint16_t)((uint16_t)signed_sample * 255u / 64u);
    } else if (ui.gen_wave == SIGGEN_WAVE_FULL) {
        int16_t signed_sample = scope_sine_interp(phase);
        if (signed_sample < 0) {
            signed_sample = (int16_t)-signed_sample;
        }
        return (uint16_t)((uint16_t)signed_sample * 255u / 64u);
    } else if (ui.gen_wave == SIGGEN_WAVE_POS_STEP) {
        value = (uint16_t)((phase / 51u) * 64u);
        if (value > 255u) {
            value = 255u;
        }
        return value;
    } else if (ui.gen_wave == SIGGEN_WAVE_REV_STEP) {
        value = (uint16_t)(255u - (phase / 51u) * 64u);
        if (value > 255u) {
            value = 0;
        }
        return value;
    } else if (ui.gen_wave == SIGGEN_WAVE_EXP_RISE) {
        return gen_preview_exp_rise(phase);
    } else if (ui.gen_wave == SIGGEN_WAVE_EXP_FALL) {
        return (uint16_t)(255u - gen_preview_exp_rise(phase));
    } else if (ui.gen_wave == SIGGEN_WAVE_NOISE) {
        uint32_t n = (uint32_t)point * 1103515245u + (uint32_t)phase * 12345u + 0x9E37u;
        return (uint16_t)((n >> 24) & 0xFFu);
    } else if (ui.gen_wave == SIGGEN_WAVE_DC) {
        return 255u;
    } else if (ui.gen_wave == SIGGEN_WAVE_MULTI_AUDIO) {
        int16_t value = (int16_t)(scope_sine_interp(phase) +
                                  scope_sine_interp((uint16_t)(phase * 3u)) / 2 +
                                  scope_sine_interp((uint16_t)(phase * 5u)) / 3);
        if (value < -64) {
            value = -64;
        } else if (value > 64) {
            value = 64;
        }
        return (uint16_t)((uint16_t)(value + 64) * 255u / 128u);
    } else if (ui.gen_wave == SIGGEN_WAVE_SINKER_PULSE) {
        if (phase < split) {
            return (uint16_t)(255u - (uint32_t)phase * 255u / split);
        }
        return 0;
    } else if (ui.gen_wave == SIGGEN_WAVE_LORENTZ) {
        return gen_preview_lorentz(phase);
    } else {
        int16_t signed_sample = scope_sine_interp(phase);
        return (uint16_t)((uint16_t)(signed_sample + 64) * 255u / 128u);
    }
}

static int16_t gen_wave_preview_y(uint16_t point, uint16_t width, int16_t center, int16_t amp) {
    uint16_t repeated_point;
    uint8_t phase;
    uint16_t sample;
    int16_t scaled_amp;

    if (!width) {
        return center;
    }

    repeated_point = (uint16_t)((uint32_t)point * 256u * gen_preview_cycles() / width);
    phase = (uint8_t)repeated_point;
    sample = gen_preview_sample(phase, repeated_point);
    scaled_amp = (int16_t)((int32_t)amp * gen_clamped_amp_tenths() / 33);
    if (scaled_amp < 1) {
        scaled_amp = 1;
    }
    return (int16_t)(center + scaled_amp - (int16_t)((uint32_t)sample * (uint16_t)(scaled_amp * 2) / 255u));
}

static void draw_gen_wave_preview(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bg) {
    uint16_t grid_bg = RGB565(6, 12, 18);
    uint16_t center_y = (uint16_t)(y + h / 2u);
    uint16_t plot_x0 = (uint16_t)(x + 8u);
    uint16_t plot_w = (uint16_t)(w - 16u);
    int16_t prev_x = (int16_t)plot_x0;
    int16_t prev_y = gen_wave_preview_y(0, plot_w, (int16_t)center_y, (int16_t)(h / 2u - 11u));

    (void)bg;
    lcd_rect(x, y, w, h, grid_bg);
    lcd_frame(x, y, w, h, C_GRID);
    for (uint16_t gx = (uint16_t)(x + w / 4u); gx < (uint16_t)(x + w); gx = (uint16_t)(gx + w / 4u)) {
        lcd_rect(gx, (uint16_t)(y + 1u), 1, (uint16_t)(h - 2u), C_PANEL_2);
    }
    lcd_rect((uint16_t)(x + 1u), center_y, (uint16_t)(w - 2u), 1, RGB565(58, 74, 88));

    for (uint16_t px = 3; px <= plot_w; px = (uint16_t)(px + 3u)) {
        int16_t sx = (int16_t)(plot_x0 + px);
        int16_t sy = gen_wave_preview_y(px, plot_w, (int16_t)center_y, (int16_t)(h / 2u - 11u));
        lcd_line(prev_x, prev_y, sx, sy, C_GEN);
        lcd_line(prev_x, (int16_t)(prev_y + 1), sx, (int16_t)(sy + 1), C_GEN);
        prev_x = sx;
        prev_y = sy;
    }

}

static void draw_gen_output_panel(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bg) {
    uint16_t panel_bg = ui.running ? RGB565(12, 56, 39) : C_PANEL;
    uint16_t accent = ui.running ? C_DMM : C_WARN;

    (void)bg;
    lcd_rect(x, y, w, h, panel_bg);
    lcd_frame(x, y, w, h, accent);
    lcd_rect(x, y, w, 4, accent);
    lcd_text_center(x, (uint16_t)(y + h / 2u - 10u), w, ui.running ? "ON" : "OFF", accent, panel_bg, 3);
}

static void draw_gen_editor_digit(uint16_t x, uint16_t y, char digit, uint8_t selected, uint16_t fg, uint16_t bg) {
    char text[2] = {digit, 0};

    lcd_text(x, y, text, selected ? C_TEXT : fg, bg, 2);
    if (selected) {
        lcd_rect((uint16_t)(x + 1u), (uint16_t)(y + 15u), 10, 2, C_TEXT);
    }
}

static void draw_gen_param_chip(uint16_t x, uint16_t y, uint16_t w, uint8_t param, const char *value) {
    uint8_t enabled = gen_param_available(param);
    uint8_t active = enabled && ui.gen_param == param;
    uint16_t chip_bg = active ? C_GEN : (enabled ? C_PANEL_2 : C_TOP);
    uint16_t fg = active ? C_BG : (enabled ? C_MUTED : C_GRID);
    const char *shown_value = enabled ? value : "--";
    uint8_t unit = gen_freq_unit < GEN_FREQ_UNIT_COUNT ? gen_freq_unit : 0u;
    char digits[5];

    lcd_rect(x, y, w, 32, chip_bg);
    lcd_rect(x, y, w, 2, C_GEN);
    lcd_text((uint16_t)(x + 6u), (uint16_t)(y + 4u), gen_param_labels[param], fg, chip_bg, 1);
    if (param == GEN_PARAM_FREQ) {
        uint16_t value_y = (uint16_t)(y + 15u);
        uint16_t digit_x = (uint16_t)(x + 4u);
        format_gen_freq_digits(digits);
        for (uint8_t i = 0; i < GEN_FREQ_EDIT_DIGITS; ++i) {
            uint16_t px = (uint16_t)(digit_x + i * 12u);
            uint8_t selected = (uint8_t)(active && gen_freq_edit_pos == i);
            draw_gen_editor_digit(px, value_y, digits[i], selected, fg, chip_bg);
        }
        gen_freq_draw_unit((uint16_t)(x + 54u), (uint16_t)(y + 12u), unit, fg, chip_bg,
                           (uint8_t)(active && gen_freq_edit_pos == GEN_FREQ_EDIT_UNIT));
        (void)shown_value;
        return;
    }
    if (param == GEN_PARAM_DUTY && enabled) {
        char duty_digits[4];
        uint16_t value_y = (uint16_t)(y + 15u);
        uint16_t digit_x = (uint16_t)(x + 8u);
        format_u16_3(gen_clamped_duty(), duty_digits);
        for (uint8_t i = 0; i < GEN_DUTY_EDIT_DIGITS; ++i) {
            uint16_t px = (uint16_t)(digit_x + i * 12u);
            uint8_t selected = (uint8_t)(active && gen_freq_edit_pos == i);
            draw_gen_editor_digit(px, value_y, duty_digits[i], selected, fg, chip_bg);
        }
        lcd_text((uint16_t)(x + 50u), (uint16_t)(y + 18u), "%", fg, chip_bg, 1);
        (void)shown_value;
        return;
    }
    if (param == GEN_PARAM_AMP && enabled) {
        char amp_digits[3];
        uint16_t value_y = (uint16_t)(y + 15u);
        uint16_t digit_x = (uint16_t)(x + 15u);
        format_u8_2(gen_clamped_amp_tenths(), amp_digits);
        for (uint8_t i = 0; i < GEN_AMP_EDIT_DIGITS; ++i) {
            uint16_t px = (uint16_t)(i == 0u ? digit_x : (uint16_t)(digit_x + 24u));
            uint8_t selected = (uint8_t)(active && gen_freq_edit_pos == i);
            draw_gen_editor_digit(px, value_y, amp_digits[i], selected, fg, chip_bg);
        }
        lcd_text((uint16_t)(digit_x + 14u), (uint16_t)(value_y + 8u), ".", fg, chip_bg, 1);
        lcd_text((uint16_t)(x + 51u), (uint16_t)(y + 18u), "V", fg, chip_bg, 1);
        (void)shown_value;
        return;
    }
    lcd_text_center(x, (uint16_t)(y + 13u), w, shown_value, fg, chip_bg, 2);
}

static void draw_gen_param_row(uint16_t x, uint16_t y) {
    uint8_t wave = ui.gen_wave < SIGGEN_WAVE_COUNT ? ui.gen_wave : SIGGEN_WAVE_SINE;

    draw_gen_param_chip(x, y, GEN_PARAM_CHIP_W, GEN_PARAM_WAVE, gen_wave_short_labels[wave]);
    draw_gen_param_chip((uint16_t)(x + GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_FREQ,
                        "");
    draw_gen_param_chip((uint16_t)(x + (GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP) * 2u),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_DUTY,
                        "");
    draw_gen_param_chip((uint16_t)(x + (GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP) * 3u),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_AMP,
                        "");
}

static void draw_generator(void) {
    lcd_rect(0, Y_BODY, LCD_WIDTH, H_BODY, C_BG);
    draw_mode_tabs();
    lcd_rect(10, 55, 303, 136, C_PANEL);
    lcd_rect(10, 55, 5, 136, C_GEN);
    lcd_text(22, 58, gen_wave_labels[ui.gen_wave], C_GEN, C_PANEL, 3);
    draw_gen_wave_preview(22, 101, 204, 76, C_PANEL);
    draw_gen_output_panel(GEN_OUTPUT_X, 101, GEN_OUTPUT_W, 76, C_PANEL);
}

static void draw_generator_immersive(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG);
    draw_micro_status(6, 6, C_BG);
    draw_battery_status_overlay(C_BG);
    lcd_text(18, 31, gen_wave_labels[ui.gen_wave], C_GEN, C_BG, 3);
    draw_gen_wave_preview(14, 68, 220, 123, C_BG);
    draw_gen_output_panel(GEN_OUTPUT_X, 68, GEN_OUTPUT_W, 123, C_BG);
    draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
}

static uint16_t menu_item_color(uint8_t index) {
    if (index == 1u) {
        return C_MENU_SCOPE;
    }
    if (index == 2u) {
        return C_MENU_GEN;
    }
    if (index == 3u) {
        return C_MENU_SETTINGS;
    }
    return C_DMM;
}

static void draw_menu_icon(uint8_t index, uint16_t x, uint16_t y, uint16_t color, uint16_t bg) {
    if (index == 0u) {
        lcd_frame((uint16_t)(x + 2u), y, 42, 52, color);
        lcd_rect((uint16_t)(x + 8u), (uint16_t)(y + 6u), 30, 13, bg);
        lcd_frame((uint16_t)(x + 8u), (uint16_t)(y + 6u), 30, 13, color);
        lcd_text((uint16_t)(x + 13u), (uint16_t)(y + 24u), "V", color, bg, 1);
        lcd_text((uint16_t)(x + 25u), (uint16_t)(y + 24u), "OHM", color, bg, 1);
        for (uint8_t i = 0; i < 4u; ++i) {
            uint16_t px = (uint16_t)(x + 7u + i * 9u);
            lcd_frame(px, (uint16_t)(y + 40u), 6, 6, color);
            lcd_rect((uint16_t)(px + 2u), (uint16_t)(y + 42u), 2, 2, color);
        }
    } else if (index == 1u) {
        lcd_text((uint16_t)(x + 1u), y, "CH1", C_CH1, bg, 1);
        lcd_text((uint16_t)(x + 25u), y, "CH2", C_CH2, bg, 1);
        lcd_line((int16_t)(x + 9u), (int16_t)(y + 11u), (int16_t)(x + 9u), (int16_t)(y + 17u), C_CH1);
        lcd_line((int16_t)(x + 33u), (int16_t)(y + 11u), (int16_t)(x + 33u), (int16_t)(y + 17u), C_CH2);
        lcd_line((int16_t)(x + 9u), (int16_t)(y + 17u), (int16_t)(x + 13u), (int16_t)(y + 13u), C_CH1);
        lcd_line((int16_t)(x + 9u), (int16_t)(y + 17u), (int16_t)(x + 5u), (int16_t)(y + 13u), C_CH1);
        lcd_line((int16_t)(x + 33u), (int16_t)(y + 17u), (int16_t)(x + 37u), (int16_t)(y + 13u), C_CH2);
        lcd_line((int16_t)(x + 33u), (int16_t)(y + 17u), (int16_t)(x + 29u), (int16_t)(y + 13u), C_CH2);
        lcd_frame(x, (uint16_t)(y + 18u), 46, 34, color);
        for (uint16_t gx = (uint16_t)(x + 11u); gx < (uint16_t)(x + 44u); gx = (uint16_t)(gx + 11u)) {
            lcd_rect(gx, (uint16_t)(y + 19u), 1, 32, C_GRID);
        }
        for (uint16_t gy = (uint16_t)(y + 27u); gy < (uint16_t)(y + 50u); gy = (uint16_t)(gy + 8u)) {
            lcd_rect((uint16_t)(x + 1u), gy, 44, 1, C_GRID);
        }
        lcd_line((int16_t)(x + 4u), (int16_t)(y + 42u), (int16_t)(x + 12u), (int16_t)(y + 29u), C_CH1);
        lcd_line((int16_t)(x + 12u), (int16_t)(y + 29u), (int16_t)(x + 22u), (int16_t)(y + 43u), C_CH1);
        lcd_line((int16_t)(x + 22u), (int16_t)(y + 43u), (int16_t)(x + 32u), (int16_t)(y + 28u), C_CH1);
        lcd_line((int16_t)(x + 32u), (int16_t)(y + 28u), (int16_t)(x + 42u), (int16_t)(y + 41u), C_CH1);
    } else if (index == 2u) {
        lcd_text((uint16_t)(x + 5u), y, "DDS", C_TEXT, bg, 1);
        lcd_line((int16_t)(x + 38u), (int16_t)(y + 16u), (int16_t)(x + 38u), (int16_t)(y + 7u), C_TEXT);
        lcd_line((int16_t)(x + 38u), (int16_t)(y + 7u), (int16_t)(x + 34u), (int16_t)(y + 11u), C_TEXT);
        lcd_line((int16_t)(x + 38u), (int16_t)(y + 7u), (int16_t)(x + 42u), (int16_t)(y + 11u), C_TEXT);
        lcd_frame(x, (uint16_t)(y + 18u), 46, 34, color);
        lcd_line((int16_t)(x + 5u), (int16_t)(y + 43u), (int16_t)(x + 11u), (int16_t)(y + 43u), color);
        lcd_line((int16_t)(x + 11u), (int16_t)(y + 43u), (int16_t)(x + 11u), (int16_t)(y + 30u), color);
        lcd_line((int16_t)(x + 11u), (int16_t)(y + 30u), (int16_t)(x + 21u), (int16_t)(y + 30u), color);
        lcd_line((int16_t)(x + 21u), (int16_t)(y + 30u), (int16_t)(x + 21u), (int16_t)(y + 43u), color);
        lcd_line((int16_t)(x + 21u), (int16_t)(y + 43u), (int16_t)(x + 33u), (int16_t)(y + 43u), color);
        lcd_line((int16_t)(x + 33u), (int16_t)(y + 43u), (int16_t)(x + 33u), (int16_t)(y + 30u), color);
        lcd_line((int16_t)(x + 33u), (int16_t)(y + 30u), (int16_t)(x + 41u), (int16_t)(y + 30u), color);
    } else {
        static const int8_t outer[][2] = {
            {19, 10}, {27, 10}, {28, 16}, {31, 17}, {36, 13}, {41, 18},
            {37, 23}, {38, 26}, {43, 26}, {43, 30}, {38, 30}, {37, 33},
            {41, 38}, {36, 43}, {31, 39}, {28, 40}, {27, 46}, {19, 46},
            {18, 40}, {15, 39}, {10, 43}, {5, 38}, {9, 33}, {8, 30},
            {3, 30}, {3, 26}, {8, 26}, {9, 23}, {5, 18}, {10, 13},
            {15, 17}, {18, 16},
        };
        static const int8_t inner[][2] = {
            {23, 20}, {28, 22}, {31, 25}, {32, 28}, {31, 31}, {28, 34},
            {23, 36}, {18, 34}, {15, 31}, {14, 28}, {15, 25}, {18, 22},
        };
        for (uint8_t i = 0; i < (uint8_t)(sizeof(outer) / sizeof(outer[0])); ++i) {
            uint8_t next = (uint8_t)((i + 1u) % (uint8_t)(sizeof(outer) / sizeof(outer[0])));
            lcd_line((int16_t)(x + outer[i][0]), (int16_t)(y + outer[i][1]),
                     (int16_t)(x + outer[next][0]), (int16_t)(y + outer[next][1]),
                     color);
        }
        for (uint8_t i = 0; i < (uint8_t)(sizeof(inner) / sizeof(inner[0])); ++i) {
            uint8_t next = (uint8_t)((i + 1u) % (uint8_t)(sizeof(inner) / sizeof(inner[0])));
            lcd_line((int16_t)(x + inner[i][0]), (int16_t)(y + inner[i][1]),
                     (int16_t)(x + inner[next][0]), (int16_t)(y + inner[next][1]),
                     color);
        }
    }
}

static void draw_mode_menu_item(uint8_t index, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint8_t selected = ui.menu_index == index;
    uint16_t accent = menu_item_color(index);
    uint16_t bg = selected ? C_PANEL_2 : C_PANEL;
    uint16_t fg = selected ? C_TEXT : C_MUTED;

    lcd_rect(x, y, w, h, bg);
    lcd_frame(x, y, w, h, selected ? accent : C_GRID);
    lcd_rect(x, y, w, 5, accent);
    draw_menu_icon(index, (uint16_t)(x + (w - 46u) / 2u), (uint16_t)(y + 34u), selected ? accent : C_MUTED, bg);
    if (index == 2u) {
        lcd_text_center(x, (uint16_t)(y + h - 36u), w, "SIGNAL", fg, bg, 1);
        lcd_text_center(x, (uint16_t)(y + h - 24u), w, "GENERATOR", fg, bg, 1);
    } else {
        lcd_text_center(x, (uint16_t)(y + h - 30u), w, menu_labels[index], fg, bg, 1);
    }
    if (selected) {
        lcd_rect((uint16_t)(x + 8u), (uint16_t)(y + h - 11u), (uint16_t)(w - 16u), 4, accent);
    }
}

static void draw_mode_menu(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG);
    draw_battery_status_overlay(C_BG);

    lcd_text(14, 18, "MENU", C_TEXT, C_BG, 3);

    draw_mode_menu_item(0, 6, 62, 74, 154);
    draw_mode_menu_item(1, 86, 62, 74, 154);
    draw_mode_menu_item(2, 166, 62, 74, 154);
    draw_mode_menu_item(3, 246, 62, 68, 154);
}

static const char *startup_label(void) {
    if (ui_settings.startup_screen == SETTINGS_START_MENU) {
        return startup_labels[0];
    }
    if (ui_settings.startup_screen == SETTINGS_START_DMM) {
        return startup_labels[1];
    }
    if (ui_settings.startup_screen == SETTINGS_START_SCOPE) {
        return startup_labels[2];
    }
    if (ui_settings.startup_screen == SETTINGS_START_GEN) {
        return startup_labels[3];
    }
    return startup_labels[1];
}

static const char *settings_value_text(uint8_t row, char out[12]) {
    (void)out;
    if (row == 0u) {
        return beep_level_labels[ui_settings.beep_level];
    }
    if (row == 1u) {
        return brightness_level_labels[ui_settings.brightness_level];
    }
    if (row == 2u) {
        return startup_label();
    }
    if (row == 3u) {
        return sleep_labels[ui_settings.sleep_enabled ? 1u : 0u];
    }
    return FIRMWARE_VERSION_TEXT;
}

static void draw_settings_tile(uint8_t row, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    char value[12];
    uint8_t selectable = row < SETTINGS_SELECTABLE_COUNT;
    uint8_t selected = selectable && ui.settings_row == row;
    uint16_t accent = selectable ? menu_item_color(3) : C_GRID;
    uint16_t bg = selected ? C_PANEL_2 : C_PANEL;
    uint16_t label_color = selectable ? (selected ? C_TEXT : C_MUTED) : C_GRID;
    uint16_t value_color = selectable ? (selected ? accent : C_TEXT) : C_MUTED;
    const char *value_text = settings_value_text(row, value);
    uint8_t value_scale = (row == 2u || lcd_text_width(value_text, 2) <= (uint16_t)(w - 10u)) ? 2u : 1u;
    uint16_t value_y = (uint16_t)(y + (value_scale == 2u ? 28u : 34u));

    lcd_rect(x, y, w, h, bg);
    lcd_frame(x, y, w, h, selected ? accent : C_GRID);
    lcd_rect(x, y, w, 4, accent);
    lcd_text((uint16_t)(x + 8u), (uint16_t)(y + 8u), settings_row_labels[row], label_color, bg, 1);
    lcd_text_center(x, value_y, w, value_text, value_color, bg, value_scale);
    if (selected) {
        lcd_rect((uint16_t)(x + 10u), (uint16_t)(y + h - 7u), (uint16_t)(w - 20u), 3, accent);
    }
}

static void draw_settings_menu(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG);
    draw_battery_status_overlay(C_BG);

    lcd_text(14, 18, "SETTINGS", C_TEXT, C_BG, 3);
    for (uint8_t row = 0; row < SETTINGS_ROW_COUNT; ++row) {
        uint8_t col = (uint8_t)(row % SETTINGS_GRID_COLUMNS);
        uint8_t line = (uint8_t)(row / SETTINGS_GRID_COLUMNS);
        draw_settings_tile(row,
                           (uint16_t)(8u + col * 105u),
                           (uint16_t)(72u + line * 66u),
                           98,
                           56);
    }
}

typedef enum {
    UI_RENDER_FULL,
    UI_RENDER_MODE_CONTENT,
    UI_RENDER_LOCAL_CHANGE,
    UI_RENDER_DMM_READING,
    UI_RENDER_BATTERY,
    UI_RENDER_SCOPE_FRAME,
    UI_RENDER_GEN_PARAMS,
    UI_RENDER_GEN_PREVIEW_PARAMS,
} ui_render_job_t;

static ui_render_job_t current_render_job;

static void ui_draw_full_scene(void) {
    if (!ui.chrome_visible) {
        if (ui.mode == UI_MODE_SCOPE) {
            draw_scope_immersive();
        } else if (ui.mode == UI_MODE_GEN) {
            draw_generator_immersive();
        } else {
            draw_dmm_immersive();
        }
        return;
    }

    draw_header();
    if (ui.mode == UI_MODE_SCOPE) {
        draw_scope();
    } else if (ui.mode == UI_MODE_GEN) {
        draw_generator();
    } else {
        draw_dmm();
    }
    draw_softkeys();
}

static void ui_draw_mode_content_scene(void) {
    if (!ui.chrome_visible) {
        if (ui.mode == UI_MODE_SCOPE) {
            draw_scope_immersive();
        } else if (ui.mode == UI_MODE_GEN) {
            draw_generator_immersive();
        } else {
            draw_dmm_immersive();
        }
        return;
    }

    if (ui.mode == UI_MODE_SCOPE) {
        draw_scope();
    } else if (ui.mode == UI_MODE_GEN) {
        draw_generator();
    } else {
        draw_dmm();
    }
    draw_softkeys();
}

static void ui_draw_local_change_scene(void) {
    if (ui.mode != UI_MODE_DMM) {
        ui_draw_mode_content_scene();
        return;
    }

    if (ui.chrome_visible) {
        draw_dmm_panel();
        draw_softkeys();
    } else {
        draw_dmm_immersive_panel();
        draw_dmm_softkeys();
    }
}

static void ui_draw_dmm_reading_scene(void) {
    if (ui.mode != UI_MODE_DMM) {
        ui_draw_mode_content_scene();
        return;
    }

    if (ui.chrome_visible) {
        draw_dmm_panel();
    } else {
        draw_dmm_immersive_panel();
    }
}

static void ui_draw_battery_status_scene(void) {
    if (ui.chrome_visible) {
        draw_battery_status_header();
    } else if (ui.mode == UI_MODE_SCOPE) {
        draw_battery_status_overlay(RGB565(4, 8, 11));
    } else {
        draw_battery_status_overlay(C_BG);
    }
}

static void ui_draw_scope_frame_scene(void) {
    if (ui.mode != UI_MODE_SCOPE) {
        ui_draw_mode_content_scene();
    } else if (ui.chrome_visible) {
        draw_scope_chrome_live();
    } else {
        draw_scope_immersive();
    }
}

static void ui_draw_gen_params_scene(void) {
    if (ui.mode != UI_MODE_GEN) {
        ui_draw_local_change_scene();
        return;
    }
    draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
}

static void ui_draw_gen_preview_params_scene(void) {
    if (ui.mode != UI_MODE_GEN) {
        ui_draw_local_change_scene();
        return;
    }
    if (ui.chrome_visible) {
        lcd_rect(10, 55, 220, 136, C_PANEL);
        lcd_rect(10, 55, 5, 136, C_GEN);
        lcd_text(22, 58, gen_wave_labels[ui.gen_wave], C_GEN, C_PANEL, 3);
        draw_gen_wave_preview(22, 101, 204, 76, C_PANEL);
    } else {
        lcd_rect(14, 31, 220, 160, C_BG);
        lcd_text(18, 31, gen_wave_labels[ui.gen_wave], C_GEN, C_BG, 3);
        draw_gen_wave_preview(14, 68, 220, 123, C_BG);
    }
    draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
}

static void ui_render_dispatch(void *ctx) {
    (void)ctx;
    if (ui.overlay == UI_OVERLAY_MODE_MENU) {
        draw_mode_menu();
        draw_fw_update_overlay(C_BG);
        draw_screenshot_overlay(C_BG);
        return;
    }
    if (ui.overlay == UI_OVERLAY_SETTINGS) {
        draw_settings_menu();
        draw_fw_update_overlay(C_BG);
        draw_screenshot_overlay(C_BG);
        return;
    }
    if (current_render_job == UI_RENDER_MODE_CONTENT) {
        ui_draw_mode_content_scene();
    } else if (current_render_job == UI_RENDER_LOCAL_CHANGE) {
        ui_draw_local_change_scene();
    } else if (current_render_job == UI_RENDER_DMM_READING) {
        ui_draw_dmm_reading_scene();
    } else if (current_render_job == UI_RENDER_BATTERY) {
        ui_draw_battery_status_scene();
    } else if (current_render_job == UI_RENDER_SCOPE_FRAME) {
        ui_draw_scope_frame_scene();
    } else if (current_render_job == UI_RENDER_GEN_PARAMS) {
        ui_draw_gen_params_scene();
    } else if (current_render_job == UI_RENDER_GEN_PREVIEW_PARAMS) {
        ui_draw_gen_preview_params_scene();
    } else {
        ui_draw_full_scene();
    }
    draw_scope_calibration_overlay();
    if (!ui.chrome_visible && (ui.softkeys_ms || scope_any_menu_open())) {
        draw_softkeys();
    }
    draw_fw_update_overlay((!ui.chrome_visible && ui.mode == UI_MODE_SCOPE) ? RGB565(4, 8, 11) : C_BG);
    draw_screenshot_overlay((!ui.chrome_visible && ui.mode == UI_MODE_SCOPE) ? RGB565(4, 8, 11) : C_BG);
}

static void ui_render_job(ui_render_job_t job) {
    uint32_t start = load_counter_read();
    current_render_job = job;
    lcd_render(ui_render_dispatch, 0);
    ui_last_render_ticks = load_counter_elapsed(start, load_counter_read());
    if (job == UI_RENDER_FULL || job == UI_RENDER_BATTERY ||
        (!ui.chrome_visible && (job == UI_RENDER_MODE_CONTENT || job == UI_RENDER_SCOPE_FRAME))) {
        battery_snapshot_store();
    }
}

static void ui_render(void) {
    ui_render_job(UI_RENDER_FULL);
}

static void ui_render_mode_content(void) {
    ui_render_job(UI_RENDER_MODE_CONTENT);
}

static void ui_render_local_change(void) {
    ui_render_job(UI_RENDER_LOCAL_CHANGE);
}

static void ui_render_dmm_reading(void) {
    ui_render_job(UI_RENDER_DMM_READING);
}

static void ui_redraw_battery_status(void) {
    ui_render_job(UI_RENDER_BATTERY);
}

static void ui_render_scope_frame(void) {
    ui_render_job(UI_RENDER_SCOPE_FRAME);
}

static void ui_render_gen_params(void) {
    ui_render_job(UI_RENDER_GEN_PARAMS);
}

static void ui_render_gen_preview_params(void) {
    ui_render_job(UI_RENDER_GEN_PREVIEW_PARAMS);
}

static uint8_t battery_blink_tick(uint32_t elapsed_ms) {
    if (!battery_low_alert()) {
        ui.battery_blink_ms = 0;
        if (!ui.battery_blink_on) {
            ui.battery_blink_on = 1;
            return 1;
        }
        return 0;
    }

    uint32_t next = (uint32_t)ui.battery_blink_ms + elapsed_ms;
    if (next >= BATTERY_LOW_BLINK_MS) {
        ui.battery_blink_ms = 0;
        ui.battery_blink_on ^= 1u;
        return 1;
    }

    ui.battery_blink_ms = (uint16_t)next;
    return 0;
}

static void ui_switch_mode(ui_mode_t mode) {
    ui_mode_t old_mode = ui.mode;
    uint8_t gen_was_running = ui.running;

    if (ui.mode != mode) {
        dmm_stats_reset();
    }
    if (old_mode == UI_MODE_GEN && mode != UI_MODE_GEN) {
        ui.running = 0;
        if (gen_was_running || gen_output_applied) {
            gen_apply();
        } else {
            gen_deferred_apply = 0;
            gen_deferred_apply_ms = 0;
        }
    }
    if (old_mode == UI_MODE_SCOPE && mode != UI_MODE_SCOPE) {
        scope_hw_slow_stop();
    }
    if (mode != UI_MODE_DMM) {
        dmm_live_wire_alert = 0;
        dmm_pause();
    }
    ui.mode = mode;
    ui.overlay = UI_OVERLAY_NONE;
    ui.chrome_visible = 0;
    ui.idle_ms = 0;
    ui.softkeys_ms = 0;
    ui.scope_move_menu_ms = 0;
    scope_clear_menus();
    ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VALUE;
    ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
    if (mode == UI_MODE_SCOPE) {
        ui.active_ch = 1;
        ui.scope_param = SCOPE_PARAM_POSITION;
        scope_auto_channel_mask = 0;
        scope_auto_channel_steps_left = 0;
        scope_sanitize_state();
        ui.running = 1;
        ui.scope_ms = 0;
        scope_frame_valid = 0;
#if !SCOPE_UI_SAFE_STUB
        scope_apply_settings();
#else
        scope_trace_cache[0].valid = 0;
        scope_trace_cache[1].valid = 0;
#endif
    } else if (mode == UI_MODE_GEN) {
        ui.running = 0;
        gen_freq_edit_pos = 0;
        gen_freq_sync_editor_unit();
        gen_prepare_state();
        gen_deferred_apply = 0;
        gen_deferred_apply_ms = 0;
    } else {
        ui.dmm_mode = DMM_MODE_AUTO;
        ui.auto_range = 1;
        dmm_hold_active = 0;
        dmm_rel_active = 0;
        dmm_stats_reset();
        dmm_set_mode(DMM_MODE_AUTO);
        ui_settings.dmm_mode = DMM_MODE_AUTO;
    }
    ui_settings.last_screen = (uint8_t)mode;
    settings_note(&ui_settings);
}

void ui_init(void) {
    settings_state_t saved;
    ui_mode_t start_mode = UI_MODE_DMM;
    uint8_t start_in_menu;

    battery_update();
    (void)settings_load(&saved);
    ui_settings_copy(&ui_settings, &saved);

    start_in_menu = ui_settings.startup_screen == SETTINGS_START_MENU ? 1u : 0u;

    if (ui_settings.startup_screen == SETTINGS_START_SCOPE) {
        start_mode = UI_MODE_SCOPE;
    } else if (ui_settings.startup_screen == SETTINGS_START_GEN) {
        start_mode = UI_MODE_GEN;
    }

    ui.gen_duty_percent = GEN_DEFAULT_DUTY_PERCENT;
    ui.gen_amp_tenths_v = GEN_DEFAULT_AMP_TENTHS;
    ui.gen_freq_hz = GEN_DEFAULT_FREQ_HZ;
    gen_freq_unit = GEN_DEFAULT_FREQ_UNIT;
    gen_freq_edit_pos = 0;

    ui.mode = UI_MODE_DMM;
    ui.overlay = UI_OVERLAY_NONE;
    ui.chrome_visible = 1;
    ui.sleep_ms = 0;
    ui.sleep_due = 0;
    ui.scope_timebase = SCOPE_TIMEBASE_DEFAULT;
    ui.scope_vdiv = 3;
    scope_ch_enabled[0] = 1;
    scope_ch_enabled[1] = 1;
    scope_probe_x10[0] = 0;
    scope_probe_x10[1] = 0;
    scope_vdiv_ch[0] = 3;
    scope_vdiv_ch[1] = 3;
    scope_coupling_dc[0] = 1;
    scope_coupling_dc[1] = 1;
    scope_zero_offset[0] = 0;
    scope_zero_offset[1] = 0;
    scope_auto_zero_active = 0;
    scope_auto_zero_mask = 0;
    scope_auto_zero_stable_count = 0;
    scope_auto_zero_steps_left = 0;
    scope_auto_channel_mask = 0;
    scope_auto_channel_steps_left = 0;
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_cursor_mode = SCOPE_CURSOR_OFF;
    ui.scope_cursor_sel = 0;
    ui.scope_param = SCOPE_PARAM_POSITION;
    scope_clear_menus();
    ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VALUE;
    ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
    ui.scope_move_menu_ms = 0;
    ui.scope_trigger_source = 1;
    ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    ui.scope_trigger_edge = 0;
    ui.scope_trigger_level = 128;
    scope_measure_param = SCOPE_MEASURE_VPP;
    scope_measure_mask[0] = scope_measure_bit(SCOPE_MEASURE_VPP);
    scope_measure_mask[1] = scope_measure_bit(SCOPE_MEASURE_VPP);
    scope_measure_visible = 1;
    scope_ch_pos[0] = 0;
    scope_ch_pos[1] = 0;
    scope_h_pos = 0;
    scope_h_value_pos = 0;
    scope_h_value_timebase = scope_safe_timebase();
    scope_cursor_x[0] = 80;
    scope_cursor_x[1] = 176;
    scope_cursor_y[0] = 86;
    scope_cursor_y[1] = 170;
    for (uint8_t i = 0; i < 2u; ++i) {
        scope_cursor_store_time_from_raw(i);
        scope_cursor_store_level_from_raw(i);
    }
    ui.dmm_mode = dmm_sanitize_mode(ui_settings.dmm_mode);
    if (start_mode == UI_MODE_DMM) {
        ui.dmm_mode = DMM_MODE_AUTO;
    }
    ui.auto_range = ui.dmm_mode == DMM_MODE_AUTO ? 1u : 0u;
    ui_settings.dmm_mode = ui.dmm_mode;
    board_buzzer_set_volume(beep_level_percent[ui_settings.beep_level]);
    board_backlight_set_level(brightness_level_percent[ui_settings.brightness_level]);
    dmm_set_mode(ui.dmm_mode);
    dmm_stats_reset();
    ui_switch_mode(start_mode);
    if (start_in_menu) {
        dmm_pause_for_menu_overlay();
        ui.overlay = UI_OVERLAY_MODE_MENU;
        ui.menu_index = (uint8_t)start_mode;
        if (ui.menu_index >= 4u) {
            ui.menu_index = 0;
        }
        ui.chrome_visible = 1;
    }
    battery_snapshot_store();
    ui_render();
}

static void cycle_u8(uint8_t *value, uint8_t count, int8_t dir) {
    if (dir > 0) {
        *value = (uint8_t)((*value + 1u) % count);
    } else {
        *value = *value == 0 ? (uint8_t)(count - 1u) : (uint8_t)(*value - 1u);
    }
}

static uint8_t scope_any_menu_open(void) {
    return (uint8_t)(ui.scope_move_mode || ui.scope_channel_menu || ui.scope_trigger_menu ||
                     ui.scope_measure_menu || ui.scope_cursor_menu);
}

static uint8_t scope_softkey_menu_open(void) {
    return (uint8_t)(ui.scope_channel_menu || ui.scope_trigger_menu ||
                     ui.scope_measure_menu || ui.scope_cursor_menu);
}

static void scope_clear_menus(void) {
    ui.scope_channel_menu = 0;
    ui.scope_trigger_menu = 0;
    ui.scope_measure_menu = 0;
    ui.scope_cursor_menu = 0;
    ui.scope_move_mode = 0;
}

static void gen_step_param(int8_t dir) {
    do {
        cycle_u8(&ui.gen_param, GEN_PARAM_COUNT, dir);
    } while (!gen_param_available(ui.gen_param));
    gen_freq_edit_pos = 0;
    if (ui.gen_param == GEN_PARAM_FREQ) {
        gen_freq_sync_editor_unit();
    }
}

static void gen_select_param(uint8_t param) {
    if (param < GEN_PARAM_COUNT && gen_param_available(param)) {
        ui.gen_param = param;
    }
    gen_freq_edit_pos = 0;
    if (ui.gen_param == GEN_PARAM_FREQ) {
        gen_freq_sync_editor_unit();
    }
}

static uint8_t gen_param_edit_last_pos(uint8_t param) {
    if (param == GEN_PARAM_FREQ) {
        return GEN_FREQ_EDIT_UNIT;
    }
    if (param == GEN_PARAM_DUTY) {
        return (uint8_t)(GEN_DUTY_EDIT_DIGITS - 1u);
    }
    if (param == GEN_PARAM_AMP) {
        return (uint8_t)(GEN_AMP_EDIT_DIGITS - 1u);
    }
    return 0;
}

static uint8_t gen_param_has_editor(uint8_t param) {
    return param == GEN_PARAM_FREQ || param == GEN_PARAM_DUTY || param == GEN_PARAM_AMP;
}

static void gen_advance_editor_or_param(int8_t dir) {
    uint8_t last = gen_param_edit_last_pos(ui.gen_param);

    if (!gen_param_has_editor(ui.gen_param)) {
        gen_step_param(dir);
    } else if (dir > 0) {
        if (gen_freq_edit_pos < last) {
            ++gen_freq_edit_pos;
        } else {
            gen_step_param(1);
        }
    } else {
        if (gen_freq_edit_pos > 0u) {
            --gen_freq_edit_pos;
        } else {
            gen_step_param(-1);
            gen_freq_edit_pos = gen_param_edit_last_pos(ui.gen_param);
        }
    }
}

static void gen_select_or_advance_param(uint8_t param) {
    if (ui.gen_param == param && gen_param_has_editor(param)) {
        gen_advance_editor_or_param(1);
    } else {
        if (param < GEN_PARAM_COUNT && gen_param_available(param)) {
            ui.gen_param = param;
        }
        gen_freq_edit_pos = 0;
        if (ui.gen_param == GEN_PARAM_FREQ) {
            gen_freq_sync_editor_unit();
        }
    }
}

static void gen_adjust_frequency(int8_t dir) {
    if (gen_freq_edit_pos == GEN_FREQ_EDIT_UNIT) {
        gen_freq_step_editor_unit(dir);
    } else {
        static const uint16_t place_values[GEN_FREQ_EDIT_DIGITS] = {1000u, 100u, 10u, 1u};
        int32_t value = (int32_t)gen_freq_editor_value();
        uint16_t place = place_values[gen_freq_edit_pos];

        if (dir > 0) {
            value += place;
        } else {
            value -= place;
        }
        gen_freq_set_editor_value(value < 1 ? 1u : (uint16_t)value);
    }
}

static void gen_adjust_duty(int8_t dir) {
    static const uint8_t place_values[GEN_DUTY_EDIT_DIGITS] = {100u, 10u, 1u};
    int16_t value = ui.gen_duty_percent;
    uint8_t pos = gen_freq_edit_pos >= GEN_DUTY_EDIT_DIGITS ? (GEN_DUTY_EDIT_DIGITS - 1u) : gen_freq_edit_pos;

    if (dir > 0) {
        value = (int16_t)(value + place_values[pos]);
    } else {
        value = (int16_t)(value - place_values[pos]);
    }
    if (value < 1) {
        value = 1;
    } else if (value > 100) {
        value = 100;
    }
    ui.gen_duty_percent = (uint8_t)value;
}

static void gen_adjust_amp(int8_t dir) {
    static const uint8_t place_values[GEN_AMP_EDIT_DIGITS] = {10u, 1u};
    int16_t value = ui.gen_amp_tenths_v;
    uint8_t pos = gen_freq_edit_pos >= GEN_AMP_EDIT_DIGITS ? (GEN_AMP_EDIT_DIGITS - 1u) : gen_freq_edit_pos;

    if (dir > 0) {
        value = (int16_t)(value + place_values[pos]);
    } else {
        value = (int16_t)(value - place_values[pos]);
    }
    if (value < 1) {
        value = 1;
    } else if (value > 33) {
        value = 33;
    }
    ui.gen_amp_tenths_v = (uint8_t)value;
}

static void softkeys_flash_for(uint16_t ms) {
    if (!ui.chrome_visible) {
        ui.softkeys_ms = ms;
    }
}

static void softkeys_flash(void) {
    softkeys_flash_for(SOFTKEY_FLASH_MS);
}

static void capture_screenshot_now(void) {
    ui.screenshot_state = SCREENSHOT_STATE_SAVING;
    ui.screenshot_overlay_ms = SCREENSHOT_OVERLAY_MS;
    draw_screenshot_overlay(C_BG);

    ui.screenshot_state = usb_msc_store_screenshot() ? SCREENSHOT_STATE_OK : SCREENSHOT_STATE_ERROR;
    ui.screenshot_overlay_ms = SCREENSHOT_OVERLAY_MS;
    ui_render();
}

static void dmm_pause_for_menu_overlay(void) {
    if (ui.mode != UI_MODE_DMM) {
        return;
    }

    dmm_live_wire_alert = 0;
    dmm_pause();
}

static void dmm_apply_selected_mode(void) {
    dmm_hold_active = 0;
    dmm_rel_active = 0;
    dmm_stats_reset();
    ui_settings.dmm_mode = ui.dmm_mode;
    settings_note(&ui_settings);
    if (dmm_mode_uses_real_reading()) {
        dmm_set_mode(ui.dmm_mode);
    }
}

static void gen_schedule_deferred_apply(void) {
    gen_deferred_apply = 1;
    gen_deferred_apply_ms = GEN_DEFERRED_APPLY_MS;
}

static void gen_prepare_state(void) {
    if (ui.gen_wave >= SIGGEN_WAVE_COUNT) {
        ui.gen_wave = SIGGEN_WAVE_SINE;
    }
    if (ui.gen_freq_hz < GEN_MIN_FREQ_HZ) {
        ui.gen_freq_hz = GEN_MIN_FREQ_HZ;
    } else if (ui.gen_freq_hz > GEN_MAX_FREQ_HZ) {
        ui.gen_freq_hz = GEN_MAX_FREQ_HZ;
    }
    if (ui.gen_param >= GEN_PARAM_COUNT) {
        ui.gen_param = GEN_PARAM_FREQ;
    }
    gen_normalize_param();
    if (ui.gen_duty_percent < 1u) {
        ui.gen_duty_percent = GEN_DEFAULT_DUTY_PERCENT;
    } else if (ui.gen_duty_percent > 100u) {
        ui.gen_duty_percent = 100;
    }
    if (ui.gen_amp_tenths_v < 1u) {
        ui.gen_amp_tenths_v = GEN_DEFAULT_AMP_TENTHS;
    } else if (ui.gen_amp_tenths_v > 33u) {
        ui.gen_amp_tenths_v = 33;
    }
}

static void gen_apply(void) {
    gen_prepare_state();
    if (!ui.running && !gen_output_applied) {
        gen_deferred_apply = 0;
        gen_deferred_apply_ms = 0;
        return;
    }
    siggen_configure(ui.running,
                     ui.gen_wave,
                     ui.gen_freq_hz,
                     ui.gen_duty_percent,
                     ui.gen_amp_tenths_v);
    gen_output_applied = ui.running ? 1u : 0u;
    gen_deferred_apply = 0;
    gen_deferred_apply_ms = 0;
}

static uint8_t dmm_hold_supported(void) {
    return ui.dmm_mode != DMM_MODE_DIODE &&
           ui.dmm_mode != DMM_MODE_LIVE &&
           ui.dmm_mode != DMM_MODE_CONT;
}

static uint8_t dmm_relative_supported(void) {
    return ui.dmm_mode != DMM_MODE_AUTO &&
           ui.dmm_mode != DMM_MODE_DIODE &&
           ui.dmm_mode != DMM_MODE_LIVE &&
           ui.dmm_mode != DMM_MODE_CONT;
}

static uint8_t dmm_has_real_numeric_reading(void) {
    return dmm_reading_is_real() && dmm_has_reading() && dmm_value_is_numeric();
}

static void dmm_toggle_hold(void) {
    if (dmm_hold_active) {
        dmm_hold_active = 0;
        dmm_stats_reset();
        return;
    }
    if (!dmm_hold_supported() || !dmm_has_real_numeric_reading()) {
        return;
    }

    ui_text_copy(dmm_hold_value, dmm_live_display_value(), sizeof(dmm_hold_value));
    ui_text_copy(dmm_hold_unit, dmm_live_display_unit(), sizeof(dmm_hold_unit));
    dmm_hold_active = 1;
    dmm_stats_reset();
}

static void dmm_toggle_relative(void) {
    if (dmm_rel_active) {
        dmm_rel_active = 0;
        dmm_stats_reset();
        return;
    }
    if (!dmm_relative_supported() || !dmm_has_real_numeric_reading()) {
        return;
    }

    dmm_hold_active = 0;
    dmm_rel_ref_milli = dmm_value_milli_units();
    ui_text_copy(dmm_rel_ref_value, dmm_live_display_value(), sizeof(dmm_rel_ref_value));
    ui_text_copy(dmm_rel_unit, dmm_live_display_unit(), sizeof(dmm_rel_unit));
    format_signed_milli(0, dmm_rel_value);
    dmm_rel_active = 1;
    dmm_stats_reset();
}

static void dmm_step_diode_group(int8_t dir) {
    static const uint8_t modes[] = {DMM_MODE_DIODE, DMM_MODE_CAP};
    uint8_t idx = 0;
    uint8_t found = 0;

    for (uint8_t i = 0; i < (uint8_t)sizeof(modes); ++i) {
        if (ui.dmm_mode == modes[i]) {
            idx = i;
            found = 1;
            break;
        }
    }

    if (!found) {
        idx = 0;
    } else if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % (uint8_t)sizeof(modes));
    } else {
        idx = idx == 0 ? (uint8_t)(sizeof(modes) - 1u) : (uint8_t)(idx - 1u);
    }

    ui.dmm_mode = modes[idx];
    ui.auto_range = 0;
    dmm_apply_selected_mode();
}

static void dmm_step_live_group(int8_t dir) {
    static const uint8_t modes[] = {DMM_MODE_LIVE, DMM_MODE_TEMP};
    uint8_t idx = ui.dmm_mode == DMM_MODE_TEMP ? 1u : 0u;
    uint8_t found = ui.dmm_mode == DMM_MODE_LIVE || ui.dmm_mode == DMM_MODE_TEMP;

    if (!found) {
        idx = 0;
    } else if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % (uint8_t)sizeof(modes));
    } else {
        idx = idx == 0 ? (uint8_t)(sizeof(modes) - 1u) : (uint8_t)(idx - 1u);
    }

    ui.dmm_mode = modes[idx];
    ui.auto_range = 0;
    dmm_apply_selected_mode();
}

static void dmm_step_current_group(int8_t dir) {
    static const uint8_t modes[] = {
        DMM_MODE_AC_HI_CURR,
        DMM_MODE_DC_HI_CURR,
        DMM_MODE_AC_LO_CURR,
        DMM_MODE_DC_LO_CURR,
    };
    uint8_t idx = 0;
    uint8_t found = 0;

    for (uint8_t i = 0; i < (uint8_t)sizeof(modes); ++i) {
        if (ui.dmm_mode == modes[i]) {
            idx = i;
            found = 1;
            break;
        }
    }

    if (!found) {
        idx = 0;
    } else if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % (uint8_t)sizeof(modes));
    } else {
        idx = idx == 0 ? (uint8_t)(sizeof(modes) - 1u) : (uint8_t)(idx - 1u);
    }

    ui.dmm_mode = modes[idx];
    ui.auto_range = 0;
    dmm_apply_selected_mode();
}

static void dmm_step_group(uint8_t first, uint8_t count, int8_t dir) {
    if (first == DMM_MODE_DIODE) {
        dmm_step_diode_group(dir);
        return;
    }
    if (first == DMM_MODE_LIVE) {
        dmm_step_live_group(dir);
        return;
    }
    if (first == DMM_MODE_AC_HI_CURR) {
        dmm_step_current_group(dir);
        return;
    }

    if (ui.dmm_mode < first || ui.dmm_mode >= (uint8_t)(first + count)) {
        ui.dmm_mode = first;
    } else if (dir > 0) {
        ui.dmm_mode = (uint8_t)(first + (uint8_t)((ui.dmm_mode - first + 1u) % count));
    } else {
        ui.dmm_mode = ui.dmm_mode == first ? (uint8_t)(first + count - 1u) : (uint8_t)(ui.dmm_mode - 1u);
    }
    ui.auto_range = 0;
    dmm_apply_selected_mode();
}

static void scope_cycle_display(int8_t dir) {
    cycle_u8(&ui.scope_display, SCOPE_DISPLAY_COUNT, dir);
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_cycle_cursor(void) {
    cycle_u8(&ui.scope_cursor_mode, SCOPE_CURSOR_COUNT, 1);
    if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        scope_cursor_time_sync_screen();
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        scope_cursor_level_sync_screen();
    }
    scope_trace_invalidate();
}

static void scope_step_cursor_menu(int8_t dir) {
    if (ui.scope_cursor_menu_sel < SCOPE_CURSOR_MENU_MODE ||
        ui.scope_cursor_menu_sel > SCOPE_CURSOR_MENU_SECOND) {
        ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
    } else if (dir > 0) {
        ui.scope_cursor_menu_sel = ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND ?
            SCOPE_CURSOR_MENU_MODE :
            (uint8_t)(ui.scope_cursor_menu_sel + 1u);
    } else {
        ui.scope_cursor_menu_sel = ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_MODE ?
            SCOPE_CURSOR_MENU_SECOND :
            (uint8_t)(ui.scope_cursor_menu_sel - 1u);
    }
    if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_FIRST) {
        ui.scope_cursor_sel = 0;
    } else if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND) {
        ui.scope_cursor_sel = 1;
    }
}

static void scope_step_measure_menu(int8_t dir) {
    if (ui.scope_measure_menu_sel >= SCOPE_MEASURE_MENU_COUNT) {
        ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VALUE;
    } else if (dir > 0) {
        ui.scope_measure_menu_sel = (uint8_t)((ui.scope_measure_menu_sel + 1u) % SCOPE_MEASURE_MENU_COUNT);
    } else {
        ui.scope_measure_menu_sel = ui.scope_measure_menu_sel == 0u ?
            (uint8_t)(SCOPE_MEASURE_MENU_COUNT - 1u) :
            (uint8_t)(ui.scope_measure_menu_sel - 1u);
    }
}

static void scope_select_measure_menu_item(uint8_t item) {
    if (item >= SCOPE_MEASURE_MENU_COUNT) {
        item = SCOPE_MEASURE_MENU_VALUE;
    }
    ui.scope_measure_menu_sel = item;
}

static void scope_activate_measure_menu_item(void) {
    if (ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH1) {
        scope_toggle_measure_for_channel(0);
    } else if (ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_CH2) {
        scope_toggle_measure_for_channel(1);
    } else if (ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VISIBLE) {
        scope_toggle_measure_visible();
    } else {
        scope_step_measure_param(1);
    }
}

static void scope_adjust_measure_menu_item(int8_t dir) {
    if (ui.scope_measure_menu_sel == SCOPE_MEASURE_MENU_VALUE) {
        scope_step_measure_param(dir);
    } else {
        (void)dir;
        scope_activate_measure_menu_item();
    }
}

static void scope_select_cursor_menu_item(uint8_t item) {
    if (item < SCOPE_CURSOR_MENU_MODE || item > SCOPE_CURSOR_MENU_SECOND) {
        item = SCOPE_CURSOR_MENU_MODE;
    }
    ui.scope_cursor_menu_sel = item;
    if (item == SCOPE_CURSOR_MENU_FIRST) {
        ui.scope_cursor_sel = 0;
    } else if (item == SCOPE_CURSOR_MENU_SECOND) {
        ui.scope_cursor_sel = 1;
    }
}

static void scope_step_param_order(const uint8_t *order, uint8_t count, int8_t dir) {
    uint8_t idx = 0;
    uint8_t found = 0;

    for (uint8_t i = 0; i < count; ++i) {
        if (order[i] == ui.scope_param) {
            idx = i;
            found = 1;
            break;
        }
    }
    if (!found) {
        idx = 0;
    } else if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % count);
    } else {
        idx = idx == 0 ? (uint8_t)(count - 1u) : (uint8_t)(idx - 1u);
    }
    ui.scope_param = order[idx];
}

static void scope_step_param(int8_t dir) {
    scope_sanitize_state();
    if (ui.scope_channel_menu) {
        scope_step_param_order(scope_channel_param_order,
                               (uint8_t)sizeof(scope_channel_param_order),
                               dir);
    } else if (ui.scope_trigger_menu) {
        scope_step_param_order(scope_trigger_param_order,
                               (uint8_t)sizeof(scope_trigger_param_order),
                               dir);
    } else {
        scope_step_param_order(scope_global_param_order,
                               (uint8_t)sizeof(scope_global_param_order),
                               dir);
    }
}

static int8_t scope_rescale_h_pos_for_timebase(int8_t pos, uint32_t old_unit_ns, uint32_t new_unit_ns) {
    uint8_t negative = 0;
    uint8_t mag;
    uint32_t whole;
    uint32_t rem_step;
    uint32_t rem = 0;

    if (!pos || !old_unit_ns || !new_unit_ns) {
        return 0;
    }
    if (pos < 0) {
        negative = 1;
        mag = (uint8_t)(-pos);
    } else {
        mag = (uint8_t)pos;
    }

    whole = old_unit_ns / new_unit_ns;
    rem_step = old_unit_ns % new_unit_ns;
    if (whole > (uint32_t)SCOPE_H_POS_LIMIT / mag) {
        return negative ? (int8_t)-SCOPE_H_POS_LIMIT : (int8_t)SCOPE_H_POS_LIMIT;
    }

    whole *= mag;
    for (uint8_t i = 0; i < mag; ++i) {
        rem += rem_step;
        if (rem >= new_unit_ns) {
            rem -= new_unit_ns;
            ++whole;
            if (whole >= SCOPE_H_POS_LIMIT) {
                return negative ? (int8_t)-SCOPE_H_POS_LIMIT : (int8_t)SCOPE_H_POS_LIMIT;
            }
        }
    }
    if (rem >= (new_unit_ns + 1u) / 2u) {
        ++whole;
    }
    if (whole > SCOPE_H_POS_LIMIT) {
        whole = SCOPE_H_POS_LIMIT;
    }
    return negative ? (int8_t)-(int8_t)whole : (int8_t)whole;
}

static int8_t scope_h_pos_from_value(void) {
    uint8_t value_timebase = scope_h_value_timebase < SCOPE_TIMEBASE_COUNT ?
        scope_h_value_timebase :
        scope_safe_timebase();

    return scope_rescale_h_pos_for_timebase(scope_h_value_pos,
                                            scope_timebase_unit_ns[value_timebase],
                                            scope_timebase_unit_ns[scope_safe_timebase()]);
}

static void scope_step_timebase(int8_t dir) {
    cycle_u8(&ui.scope_timebase, SCOPE_TIMEBASE_COUNT, dir);
    scope_h_pos = scope_h_pos_from_value();
    scope_cursor_time_sync_screen();
    scope_apply_settings();
}

static void scope_adjust_cursor_steps(int8_t dir, uint8_t step) {
    uint8_t *value = ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL ?
        &scope_cursor_y[ui.scope_cursor_sel] :
        &scope_cursor_x[ui.scope_cursor_sel];
    int16_t next = (int16_t)*value + (int16_t)(dir * (int8_t)step);

    if (next < 0) {
        next = 0;
    } else if (next > 255) {
        next = 255;
    }
    *value = (uint8_t)next;
    if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        scope_cursor_store_level_from_raw(ui.scope_cursor_sel);
    } else {
        scope_cursor_store_time_from_raw(ui.scope_cursor_sel);
    }
    scope_trace_invalidate();
}

static void scope_adjust_cursor(int8_t dir) {
    scope_adjust_cursor_steps(dir, 5);
}

static void scope_adjust_cursor_menu_value(int8_t dir, uint8_t repeat) {
    uint8_t step = repeat ? 6u : 2u;

    if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_MODE) {
        scope_cycle_cursor();
        return;
    }
    if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_FIRST) {
        ui.scope_cursor_sel = 0;
    } else if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND) {
        ui.scope_cursor_sel = 1;
    } else {
        return;
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
        ui.scope_cursor_mode = SCOPE_CURSOR_TIME;
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        dir = (int8_t)-dir;
    }
    scope_adjust_cursor_steps(dir, step);
}

static void scope_adjust_active_channel_zero(int8_t dir) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int16_t next = (int16_t)(scope_zero_offset[idx] + dir * SCOPE_ZERO_STEP);

    scope_auto_zero_active = 0;
    if (next < -100) {
        next = -100;
    } else if (next > 100) {
        next = 100;
    }
    scope_zero_offset[idx] = next;
    scope_apply_settings();
}

static void scope_adjust_active_channel_pos(int8_t dir) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int16_t next = (int16_t)scope_ch_pos[idx] - dir;

    scope_ch_pos[idx] = scope_clamp_channel_pos(idx, next);
    ui.scope_move_sel = SCOPE_MOVE_SEL_Y;
    scope_trace_invalidate();
}

static void scope_adjust_h_pos(int8_t dir) {
    int16_t next = (int16_t)scope_h_pos + dir;

    if (next < -SCOPE_H_POS_LIMIT) {
        next = -SCOPE_H_POS_LIMIT;
    } else if (next > SCOPE_H_POS_LIMIT) {
        next = SCOPE_H_POS_LIMIT;
    }
    scope_h_pos = (int8_t)next;
    scope_h_value_pos = scope_h_pos;
    scope_h_value_timebase = scope_safe_timebase();
    ui.scope_move_sel = SCOPE_MOVE_SEL_X;
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_transient_menu_touch(void) {
    ui.scope_move_menu_ms = ui.scope_move_mode ? SCOPE_MOVE_MENU_MS : SCOPE_CHANNEL_MENU_MS;
}

static void scope_reset_move_positions(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;
    int16_t pos = (int16_t)(scope_visible_grid_mid_y() - scope_channel_base_center_for(idx));

    scope_ch_pos[idx] = scope_clamp_channel_pos(idx, pos);
    scope_h_pos = 0;
    scope_h_value_pos = 0;
    scope_h_value_timebase = scope_safe_timebase();
    ui.scope_move_sel = SCOPE_MOVE_SEL_ZERO;
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_select_or_reset_move_zero(void) {
    if (ui.scope_move_sel == SCOPE_MOVE_SEL_ZERO) {
        scope_reset_move_positions();
    } else {
        ui.scope_move_sel = SCOPE_MOVE_SEL_ZERO;
    }
}

static void scope_cycle_move_channel(void) {
    scope_clear_menus();
    ui.scope_move_mode = 1u;
    ui.active_ch = ui.active_ch == 2u ? 1u : 2u;
    scope_cursor_level_sync_screen();
    scope_ch_enabled[ui.active_ch == 2u ? 1u : 0u] = 1u;
    ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
    ui.scope_param = SCOPE_PARAM_POSITION;
    scope_transient_menu_touch();
    softkeys_flash_for(SCOPE_MOVE_MENU_MS);
}

static void scope_close_move_mode(void) {
    ui.scope_move_mode = 0;
    ui.scope_move_menu_ms = 0;
    ui.softkeys_ms = 0;
    ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
}

static void scope_move_key_in_menu(void) {
    scope_close_move_mode();
}

static void scope_open_move_mode(void) {
    scope_clear_menus();
    ui.scope_move_mode = 1u;
    ui.scope_param = SCOPE_PARAM_POSITION;
    ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
    scope_ch_enabled[ui.active_ch == 2u ? 1u : 0u] = 1u;
    scope_transient_menu_touch();
    softkeys_flash_for(SCOPE_MOVE_MENU_MS);
}

static void scope_close_cursor_menu(void) {
    ui.scope_cursor_menu = 0;
    ui.scope_move_menu_ms = 0;
    ui.softkeys_ms = 0;
    ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
}

static void scope_open_cursor_menu(void) {
    scope_clear_menus();
    ui.scope_cursor_menu = 1u;
    ui.scope_cursor_menu_sel = SCOPE_CURSOR_MENU_MODE;
    ui.scope_param = SCOPE_PARAM_CURSOR;
    scope_transient_menu_touch();
    softkeys_flash_for(SCOPE_CHANNEL_MENU_MS);
}

static void scope_toggle_active_channel_enabled(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;

    scope_ch_enabled[idx] ^= 1u;
    scope_trace_invalidate();
}

static void scope_toggle_active_channel_probe(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;

    scope_probe_x10[idx] ^= 1u;
    scope_cursor_level_sync_screen();
    scope_trace_invalidate();
}

static void scope_cycle_active_channel_vdiv(int8_t dir) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;

    cycle_u8(&scope_vdiv_ch[idx], SCOPE_VDIV_COUNT, dir);
    ui.scope_vdiv = scope_safe_vdiv();
    scope_cursor_level_sync_screen();
    scope_apply_settings_keep_frame(0);
    scope_hw_arm();
}

static void scope_toggle_active_channel_coupling(void) {
    uint8_t idx = ui.active_ch == 2u ? 1u : 0u;

    scope_coupling_dc[idx] ^= 1u;
    scope_apply_settings();
}

static void scope_cycle_trigger_source(int8_t dir) {
    (void)dir;
    ui.scope_trigger_source = ui.scope_trigger_source == 2u ? 1u : 2u;
    scope_ch_enabled[scope_trigger_source_index()] = 1u;
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_adjust_trigger_level_steps(int8_t dir, uint8_t step) {
    int16_t next = (int16_t)ui.scope_trigger_level + (int16_t)(dir * step);

    if (next < 12) {
        next = 12;
    } else if (next > 243) {
        next = 243;
    }
    ui.scope_trigger_level = (uint8_t)next;
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_adjust_trigger_level(int8_t dir) {
    scope_adjust_trigger_level_steps(dir, 1);
}

static void scope_adjust_trigger_level_key(int8_t dir, uint8_t repeat) {
    scope_adjust_trigger_level_steps((int8_t)-dir, repeat ? 3u : 1u);
}

static void scope_cycle_trigger_mode(int8_t dir) {
    cycle_u8(&ui.scope_trigger_mode, SCOPE_TRIGGER_COUNT, dir);
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_toggle_trigger_edge(void) {
    ui.scope_trigger_edge ^= 1u;
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_step_measure_param(int8_t dir) {
    cycle_u8(&scope_measure_param, SCOPE_MEASURE_COUNT, dir);
}

static void scope_toggle_measure_for_channel(uint8_t idx) {
    if (idx >= 2u) {
        idx = 0;
    }
    scope_measure_mask[idx] ^= scope_measure_bit(scope_measure_param);
}

static void scope_toggle_measure_visible(void) {
    scope_measure_visible ^= 1u;
}

static void scope_open_measure_menu(void) {
    scope_clear_menus();
    ui.scope_measure_menu = 1;
    ui.scope_measure_menu_sel = SCOPE_MEASURE_MENU_VISIBLE;
    ui.scope_param = SCOPE_PARAM_MEASURE;
    scope_transient_menu_touch();
    softkeys_flash_for(SCOPE_CHANNEL_MENU_MS);
}

static void scope_close_measure_menu(void) {
    ui.scope_measure_menu = 0;
    ui.scope_move_menu_ms = 0;
    ui.softkeys_ms = 0;
    ui.scope_param = SCOPE_PARAM_MEASURE;
}

static void scope_open_trigger_menu(uint8_t param) {
    scope_clear_menus();
    ui.scope_trigger_menu = 1;
    if (!scope_param_is_trigger(param)) {
        param = SCOPE_PARAM_TRIG_TYPE;
    }
    ui.scope_param = param;
    scope_ch_enabled[scope_trigger_source_index()] = 1u;
    scope_transient_menu_touch();
    softkeys_flash_for(SCOPE_CHANNEL_MENU_MS);
}

static void scope_close_trigger_menu(void) {
    ui.scope_trigger_menu = 0;
    ui.scope_move_menu_ms = 0;
    ui.softkeys_ms = 0;
    ui.scope_param = SCOPE_PARAM_TRIGGER;
}

static uint8_t scope_auto_zero_long_mask(void) {
    if (ui.scope_channel_menu) {
        return (uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u);
    }
    return 0x03u;
}

static void scope_apply_zero_offsets_only(void) {
#if !SCOPE_UI_SAFE_STUB
    scope_hw_set_offsets(scope_bias_dac_for_channel(0), scope_bias_dac_for_channel(1));
#endif
}

static uint8_t scope_auto_zero_channel_step(uint8_t idx) {
    uint8_t range = scope_channel_range_index(idx);
    int16_t delta = (int16_t)scope_avg_raw[idx] - 128;
    uint16_t bias = ui_settings.scope_bias[idx][range];
    uint8_t mag = (uint8_t)(delta < 0 ? -delta : delta);
    uint8_t step;

    if (mag <= 2u) {
        return 0;
    }
    step = mag >= 40u ? 4u : (mag >= 20u ? 2u : 1u);
    if (delta > 0) {
        bias = bias > step ? (uint16_t)(bias - step) : 0u;
    } else {
        bias = bias + step > 4095u ? 4095u : (uint16_t)(bias + step);
    }
    if (bias == ui_settings.scope_bias[idx][range]) {
        return 0;
    }
    ui_settings.scope_bias[idx][range] = bias;
    return 1;
}

static void scope_auto_zero_begin(uint8_t mask) {
    if (!scope_hw_enabled()) {
        return;
    }
    mask &= 0x03u;
    if (!mask) {
        mask = 0x03u;
    }
    if (mask & 0x01u) {
        scope_zero_offset[0] = 0;
        scope_ch_enabled[0] = 1;
    }
    if (mask & 0x02u) {
        scope_zero_offset[1] = 0;
        scope_ch_enabled[1] = 1;
    }
    scope_auto_zero_mask = mask;
    scope_auto_zero_stable_count = 0;
    scope_auto_zero_active = 1;
    scope_auto_zero_steps_left = 120;
    scope_auto_channel_mask = 0;
    scope_auto_channel_steps_left = 0;
    scope_trigger_offset = 0;
    scope_trigger_locked = 0;
    ui.running = 1;
    scope_clear_menus();
    ui.scope_move_menu_ms = 0;
    ui.chrome_visible = 0;
    scope_apply_zero_offsets_only();
    scope_hw_arm();
}

static void scope_auto_zero_service(void) {
    uint8_t changed;

    if (!scope_auto_zero_active || !scope_frame_valid) {
        return;
    }
    changed = 0;
    if (scope_auto_zero_mask & 0x01u) {
        changed = (uint8_t)(changed | scope_auto_zero_channel_step(0));
    }
    if (scope_auto_zero_mask & 0x02u) {
        changed = (uint8_t)(changed | scope_auto_zero_channel_step(1));
    }
    if (scope_auto_zero_steps_left) {
        --scope_auto_zero_steps_left;
    }
    if (changed) {
        scope_auto_zero_stable_count = 0;
        settings_note(&ui_settings);
        scope_apply_zero_offsets_only();
    } else if (scope_auto_zero_stable_count < 30u) {
        ++scope_auto_zero_stable_count;
    }
    if (scope_auto_zero_stable_count >= 12u || !scope_auto_zero_steps_left) {
        scope_auto_zero_active = 0;
        scope_auto_zero_mask = 0;
    }
}

static uint8_t scope_channel_pp_raw(uint8_t idx) {
    return (uint8_t)(scope_max_raw[idx] - scope_min_raw[idx]);
}

static uint8_t scope_channel_mid_raw(uint8_t idx) {
    if (idx >= 2u) {
        idx = 0;
    }
    return (uint8_t)(((uint16_t)scope_min_raw[idx] + (uint16_t)scope_max_raw[idx]) / 2u);
}

static uint8_t scope_auto_channel_tune_vdiv(uint8_t idx) {
    uint8_t pp = scope_channel_pp_raw(idx);
    uint8_t vdiv = scope_vdiv_ch[idx];
    uint8_t next = vdiv;
    uint8_t clipped = (uint8_t)(scope_min_raw[idx] < 5u || scope_max_raw[idx] > 250u);

    if (vdiv >= SCOPE_VDIV_COUNT) {
        vdiv = 3;
        next = vdiv;
    }

    if ((clipped || pp > 230u) && vdiv + 2u < SCOPE_VDIV_COUNT) {
        next = (uint8_t)(vdiv + 2u);
    } else if ((clipped || pp > 196u) && vdiv + 1u < SCOPE_VDIV_COUNT) {
        next = (uint8_t)(vdiv + 1u);
    } else if (pp >= 10u && pp < 54u && vdiv > 0u && !clipped) {
        next = (uint8_t)(vdiv - 1u);
    }

    if (next == scope_vdiv_ch[idx]) {
        return 0;
    }
    scope_vdiv_ch[idx] = next;
    if (idx == (ui.active_ch == 2u ? 1u : 0u)) {
        ui.scope_vdiv = scope_safe_vdiv();
        scope_cursor_level_sync_screen();
    }
    return 1;
}

static uint8_t scope_auto_channel_tune(uint8_t idx) {
    uint8_t changed = 0;

    if (idx >= 2u) {
        return 0;
    }
    scope_ch_enabled[idx] = 1;
    changed = scope_auto_channel_tune_vdiv(idx);
    scope_set_channel_pos_from_avg(idx);
    if (idx == scope_trigger_source_index()) {
        ui.scope_trigger_level = scope_channel_mid_raw(idx);
    }
    return changed;
}

static void scope_auto_channel_request(uint8_t mask) {
    mask &= 0x03u;
    if (!mask) {
        return;
    }
    ui.running = 1;
    scope_auto_channel_mask = mask;
    scope_auto_channel_steps_left = 6;
    if (scope_frame_valid) {
        scope_auto_channel_service();
    }
    scope_hw_arm();
}

static void scope_auto_channel_service(void) {
    uint8_t changed = 0;

    if (!scope_auto_channel_mask || !scope_frame_valid) {
        return;
    }
    if (scope_auto_channel_mask & 0x01u) {
        changed = (uint8_t)(changed | scope_auto_channel_tune(0));
    }
    if (scope_auto_channel_mask & 0x02u) {
        changed = (uint8_t)(changed | scope_auto_channel_tune(1));
    }
    scope_trace_invalidate();
    if (changed && scope_auto_channel_steps_left) {
        --scope_auto_channel_steps_left;
        scope_apply_settings();
        scope_hw_arm();
        return;
    }
    scope_auto_channel_mask = 0;
    scope_auto_channel_steps_left = 0;
}

static void scope_set_channel_pos_from_avg(uint8_t idx) {
    int16_t delta = (int16_t)scope_avg_raw[idx] - 128;
    int16_t pos = (int16_t)((delta * 7 + (delta >= 0 ? 6 : -6)) / 12);

    scope_ch_pos[idx] = scope_clamp_channel_pos(idx, pos);
}

static void scope_adjust_selected_param(int8_t dir) {
    scope_sanitize_state();

    if (ui.scope_param == SCOPE_PARAM_VIEW) {
        scope_cycle_display(dir);
    } else if (ui.scope_param == SCOPE_PARAM_ENABLE) {
        scope_toggle_active_channel_enabled();
    } else if (ui.scope_param == SCOPE_PARAM_PROBE) {
        scope_toggle_active_channel_probe();
    } else if (ui.scope_param == SCOPE_PARAM_GAIN) {
        scope_cycle_active_channel_vdiv(dir);
    } else if (ui.scope_param == SCOPE_PARAM_COUPLING) {
        scope_toggle_active_channel_coupling();
    } else if (ui.scope_param == SCOPE_PARAM_ZERO) {
        scope_adjust_active_channel_zero(dir);
    } else if (ui.scope_param == SCOPE_PARAM_POSITION) {
        scope_adjust_active_channel_pos(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TIME) {
        scope_step_timebase(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TRIGGER) {
        scope_cycle_trigger_mode(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TRIG_SOURCE) {
        scope_cycle_trigger_source(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TRIG_LEVEL) {
        scope_adjust_trigger_level(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TRIG_TYPE) {
        scope_cycle_trigger_mode(dir);
    } else if (ui.scope_param == SCOPE_PARAM_TRIG_EDGE) {
        scope_toggle_trigger_edge();
    } else if (ui.scope_param == SCOPE_PARAM_MEASURE) {
        scope_step_measure_param(dir);
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        scope_adjust_cursor(dir);
    } else {
        cycle_u8(&ui.scope_cursor_mode, SCOPE_CURSOR_COUNT, dir);
    }
}

static void scope_select_or_advance_param(uint8_t param) {
    if (param >= SCOPE_PARAM_COUNT) {
        param = SCOPE_PARAM_POSITION;
    }
    if (ui.scope_channel_menu && !scope_param_is_channel(param)) {
        ui.scope_channel_menu = 0;
    }
    if (ui.scope_trigger_menu && !scope_param_is_trigger(param)) {
        ui.scope_trigger_menu = 0;
    }
    if (ui.scope_param == param) {
        if (param == SCOPE_PARAM_CURSOR) {
            scope_cycle_cursor();
        } else if (param == SCOPE_PARAM_TRIGGER) {
            scope_open_trigger_menu(SCOPE_PARAM_TRIG_TYPE);
        } else if (param == SCOPE_PARAM_ENABLE) {
            scope_toggle_active_channel_enabled();
        } else if (param == SCOPE_PARAM_PROBE) {
            scope_toggle_active_channel_probe();
        } else if (param == SCOPE_PARAM_GAIN) {
            scope_cycle_active_channel_vdiv(1);
        } else if (param == SCOPE_PARAM_COUPLING) {
            scope_toggle_active_channel_coupling();
        } else if (param == SCOPE_PARAM_ZERO) {
            scope_adjust_active_channel_zero(1);
        } else if (param == SCOPE_PARAM_POSITION) {
            scope_adjust_active_channel_pos(1);
        } else if (param == SCOPE_PARAM_TIME) {
            scope_step_timebase(1);
        } else if (param == SCOPE_PARAM_TRIG_SOURCE) {
            scope_cycle_trigger_source(1);
        } else if (param == SCOPE_PARAM_TRIG_LEVEL) {
            scope_adjust_trigger_level(1);
        } else if (param == SCOPE_PARAM_TRIG_TYPE) {
            scope_cycle_trigger_mode(1);
        } else if (param == SCOPE_PARAM_TRIG_EDGE) {
            scope_toggle_trigger_edge();
        } else if (param == SCOPE_PARAM_MEASURE) {
            if (ui.scope_measure_menu) {
                scope_step_measure_param(1);
            } else {
                scope_open_measure_menu();
            }
        } else {
            scope_adjust_selected_param(1);
        }
    } else {
        ui.scope_param = param;
    }
}

static uint8_t scope_play_menu_value(void) {
    if (ui.mode != UI_MODE_SCOPE) {
        return 0;
    }
    if (ui.scope_move_mode) {
        if (ui.scope_move_sel == SCOPE_MOVE_SEL_CHANNEL) {
            scope_cycle_move_channel();
        } else if (ui.scope_move_sel == SCOPE_MOVE_SEL_ZERO) {
            scope_reset_move_positions();
        }
        scope_transient_menu_touch();
        return 1;
    }
    if (ui.scope_measure_menu) {
        scope_activate_measure_menu_item();
        scope_transient_menu_touch();
        return 1;
    }
    if (ui.scope_cursor_menu) {
        if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_MODE) {
            scope_cycle_cursor();
        } else {
            ui.scope_cursor_sel ^= 1u;
            ui.scope_cursor_menu_sel = ui.scope_cursor_sel ?
                SCOPE_CURSOR_MENU_SECOND :
                SCOPE_CURSOR_MENU_FIRST;
        }
        scope_transient_menu_touch();
        return 1;
    }
    if (ui.scope_channel_menu || ui.scope_trigger_menu) {
        if (ui.scope_param == SCOPE_PARAM_GAIN) {
            scope_cycle_active_channel_vdiv(1);
            scope_transient_menu_touch();
            return 1;
        }
        if (ui.scope_param == SCOPE_PARAM_TIME) {
            scope_transient_menu_touch();
            return 1;
        }
        scope_select_or_advance_param(ui.scope_param);
        scope_transient_menu_touch();
        return 1;
    }
    return 0;
}

static void primary_action(void) {
    if (ui.mode == UI_MODE_DMM) {
        dmm_step_group(DMM_MODE_DCV, 3, 1);
    } else if (ui.mode == UI_MODE_SCOPE) {
        scope_open_move_mode();
    } else {
        gen_cycle_wave(1);
        gen_apply();
    }
}

static void dmm_step_all_modes(int8_t dir) {
    static const uint8_t modes[] = {
        DMM_MODE_DCV,
        DMM_MODE_ACV,
        DMM_MODE_RES,
        DMM_MODE_DIODE,
        DMM_MODE_CAP,
        DMM_MODE_LIVE,
        DMM_MODE_TEMP,
        DMM_MODE_AC_HI_CURR,
        DMM_MODE_DC_HI_CURR,
        DMM_MODE_AC_LO_CURR,
        DMM_MODE_DC_LO_CURR,
    };
    uint8_t idx = 0;
    uint8_t found = 0;

    for (uint8_t i = 0; i < (uint8_t)sizeof(modes); ++i) {
        if (ui.dmm_mode == modes[i]) {
            idx = i;
            found = 1;
            break;
        }
    }

    if (!found) {
        idx = 0;
    } else if (dir > 0) {
        idx = (uint8_t)((idx + 1u) % (uint8_t)sizeof(modes));
    } else {
        idx = idx == 0 ? (uint8_t)(sizeof(modes) - 1u) : (uint8_t)(idx - 1u);
    }

    ui.dmm_mode = modes[idx];
    ui.auto_range = 0;
    dmm_apply_selected_mode();
}

static uint8_t gen_adjust_current_value(int8_t dir, uint8_t repeat) {
    if (ui.gen_param == GEN_PARAM_WAVE) {
        gen_cycle_wave((int8_t)-dir);
        gen_apply();
        return 2;
    }
    if (repeat) {
        return 0;
    }
    if (ui.gen_param == GEN_PARAM_DUTY) {
        gen_adjust_duty(dir);
        gen_schedule_deferred_apply();
        return 2;
    }
    if (ui.gen_param == GEN_PARAM_AMP) {
        gen_adjust_amp(dir);
        gen_schedule_deferred_apply();
        return 2;
    }

    uint8_t old_cycles = gen_preview_cycles();
    gen_adjust_frequency(dir);
    gen_apply();
    return old_cycles == gen_preview_cycles() ? 1u : 2u;
}

static void adjust_current(int8_t dir, uint8_t repeat) {
    if (ui.mode == UI_MODE_DMM) {
        uint8_t first = dmm_group_first(ui.dmm_mode);
        dmm_step_group(first, dmm_group_count(first), dir);
    } else if (ui.mode == UI_MODE_SCOPE) {
        (void)repeat;
        scope_adjust_selected_param(dir);
    } else {
        (void)gen_adjust_current_value(dir, repeat);
    }
}

static void secondary_action(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        scope_select_or_advance_param(SCOPE_PARAM_TIME);
    } else if (ui.mode == UI_MODE_DMM) {
        dmm_step_group(DMM_MODE_DIODE, 2, 1);
    } else {
        gen_step_param(1);
    }
}

static void scope_channel_key(uint8_t channel) {
    uint8_t idx;

    if (channel < 1u || channel > 2u) {
        channel = 1;
    }
    idx = (uint8_t)(channel - 1u);
    if (ui.scope_channel_menu && ui.active_ch == channel) {
        ui.scope_channel_menu = 0;
        ui.softkeys_ms = 0;
        ui.scope_move_menu_ms = 0;
        return;
    }
    ui.active_ch = channel;
    ui.scope_cursor_sel = idx;
    scope_cursor_level_sync_screen();
    scope_clear_menus();
    ui.scope_channel_menu = 1;
    scope_transient_menu_touch();
    ui.scope_param = SCOPE_PARAM_ENABLE;
    softkeys_flash_for(SCOPE_CHANNEL_MENU_MS);
}

static void settings_cycle_startup(int8_t dir) {
    uint8_t value = ui_settings.startup_screen;

    if (value >= SETTINGS_START_COUNT) {
        value = SETTINGS_START_DMM;
    } else if (dir > 0) {
        value = value == SETTINGS_START_GEN ? SETTINGS_START_MENU : (uint8_t)(value + 1u);
    } else {
        value = value == SETTINGS_START_MENU ? SETTINGS_START_GEN : (uint8_t)(value - 1u);
    }

    ui_settings.startup_screen = value;
}

static void settings_adjust_current(int8_t dir) {
    uint8_t changed = 1;

    if (ui.settings_row == 0u) {
        cycle_u8(&ui_settings.beep_level, SETTINGS_LEVEL_COUNT, dir);
        board_buzzer_set_volume(beep_level_percent[ui_settings.beep_level]);
        if (ui_settings.beep_level) {
            beep_preview_requested = 1;
        }
    } else if (ui.settings_row == 1u) {
        cycle_u8(&ui_settings.brightness_level, SETTINGS_LEVEL_COUNT, dir);
        board_backlight_set_level(brightness_level_percent[ui_settings.brightness_level]);
    } else if (ui.settings_row == 2u) {
        settings_cycle_startup(dir);
    } else if (ui.settings_row == 3u) {
        (void)dir;
        ui_settings.sleep_enabled ^= 1u;
        ui.sleep_ms = 0;
        ui.sleep_due = 0;
    } else {
        (void)dir;
        changed = 0;
    }
    if (changed) {
        settings_note(&ui_settings);
    }
}

static void settings_move_vertical(int8_t dir) {
    uint8_t row = ui.settings_row;
    uint8_t col;
    uint8_t next;

    if (row >= SETTINGS_SELECTABLE_COUNT) {
        row = 0;
    }
    col = (uint8_t)(row % SETTINGS_GRID_COLUMNS);
    if (dir > 0) {
        next = (uint8_t)(row + SETTINGS_GRID_COLUMNS);
        if (next >= SETTINGS_SELECTABLE_COUNT) {
            next = col;
        }
        ui.settings_row = next;
    } else {
        if (row >= SETTINGS_GRID_COLUMNS) {
            ui.settings_row = (uint8_t)(row - SETTINGS_GRID_COLUMNS);
        } else {
            next = col;
            while ((uint8_t)(next + SETTINGS_GRID_COLUMNS) < SETTINGS_SELECTABLE_COUNT) {
                next = (uint8_t)(next + SETTINGS_GRID_COLUMNS);
            }
            ui.settings_row = next;
        }
    }
}

static void ui_render_gen_adjust_result(uint8_t render) {
    if (render == 2u) {
        ui_render_gen_preview_params();
    } else if (render == 1u) {
        ui_render_gen_params();
    }
}

static void ui_handle_horizontal_key(int8_t dir) {
    if (ui.mode == UI_MODE_DMM) {
        dmm_step_all_modes(dir);
        ui_render_local_change();
    } else if (ui.mode == UI_MODE_SCOPE) {
        if (ui.scope_move_mode) {
            scope_adjust_h_pos(dir);
        } else if (ui.scope_cursor_menu) {
            softkeys_flash();
            scope_step_cursor_menu(dir);
        } else if (ui.scope_measure_menu) {
            softkeys_flash();
            scope_step_measure_menu(dir);
        } else if (ui.scope_channel_menu || ui.scope_trigger_menu) {
            softkeys_flash();
            scope_step_param(dir);
        } else {
            scope_step_timebase(dir);
        }
        ui_render_mode_content();
    } else if (ui.mode == UI_MODE_GEN) {
        gen_advance_editor_or_param(dir);
        ui_render_gen_params();
    }
}

static void ui_handle_vertical_key(int8_t screen_dir, uint8_t repeat) {
    if (ui.mode == UI_MODE_GEN) {
        ui_render_gen_adjust_result(gen_adjust_current_value(screen_dir, repeat));
        return;
    }

    if (ui.mode == UI_MODE_SCOPE && scope_softkey_menu_open()) {
        softkeys_flash();
    }
    if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
        scope_adjust_active_channel_pos(screen_dir);
    } else if (ui.mode == UI_MODE_SCOPE && ui.scope_cursor_menu) {
        scope_adjust_cursor_menu_value(screen_dir, repeat);
    } else if (ui.mode == UI_MODE_SCOPE && ui.scope_measure_menu) {
        scope_adjust_measure_menu_item(screen_dir);
    } else if (ui.mode == UI_MODE_SCOPE && ui.scope_channel_menu) {
        ui.scope_param = SCOPE_PARAM_GAIN;
        scope_cycle_active_channel_vdiv(screen_dir);
    } else if (ui.mode == UI_MODE_SCOPE &&
               ui.scope_trigger_menu &&
               ui.scope_param == SCOPE_PARAM_TRIG_LEVEL) {
        scope_adjust_trigger_level_key((int8_t)-screen_dir, repeat);
    } else {
        adjust_current((int8_t)-screen_dir, repeat);
    }
    ui_render_local_change();
}

static void ui_open_mode_menu_item(uint8_t index) {
    if (index > 3u) {
        index = 0;
    }
    ui.menu_index = index;
    if (index < 3u) {
        ui_switch_mode((ui_mode_t)index);
    } else {
        ui.overlay = UI_OVERLAY_SETTINGS;
        ui.settings_row = 0;
    }
}

static void ui_handle_menu_keys(uint32_t events) {
    if (ui.overlay == UI_OVERLAY_MODE_MENU) {
        if (events & KEY_MENU) {
            cycle_u8(&ui.menu_index, 4, 1);
            ui_render();
            return;
        }
        if (events & KEY_MOVE) {
            ui_open_mode_menu_item(0);
            ui_render();
            return;
        }
        if (events & KEY_F2) {
            ui_open_mode_menu_item(1);
            ui_render();
            return;
        }
        if (events & KEY_F3) {
            ui_open_mode_menu_item(2);
            ui_render();
            return;
        }
        if (events & KEY_F4) {
            ui_open_mode_menu_item(3);
            ui_render();
            return;
        }
        if (events & KEY_OK) {
            ui_open_mode_menu_item(ui.menu_index);
            ui_render();
            return;
        }
        if (events & KEY_RIGHT) {
            cycle_u8(&ui.menu_index, 4, 1);
            ui_render();
            return;
        }
        if (events & KEY_LEFT) {
            cycle_u8(&ui.menu_index, 4, -1);
            ui_render();
            return;
        }
        return;
    }

    if (events & KEY_MENU) {
        dmm_pause_for_menu_overlay();
        ui.overlay = UI_OVERLAY_MODE_MENU;
        ui.menu_index = 3;
        ui_render();
        return;
    }

    if (events & KEY_RIGHT) {
        cycle_u8(&ui.settings_row, SETTINGS_SELECTABLE_COUNT, 1);
        ui_render();
        return;
    }
    if (events & KEY_LEFT) {
        cycle_u8(&ui.settings_row, SETTINGS_SELECTABLE_COUNT, -1);
        ui_render();
        return;
    }
    if (events & KEY_UP) {
        settings_move_vertical(-1);
        ui_render();
        return;
    }
    if (events & KEY_DOWN) {
        settings_move_vertical(1);
        ui_render();
        return;
    }
    if (events & (KEY_OK | KEY_AUTO)) {
        settings_adjust_current(1);
        ui_render();
        return;
    }
}

void ui_handle_keys(uint32_t events) {
    ui.idle_ms = 0;
    ui.sleep_ms = 0;
    ui.sleep_due = 0;

    if (events & KEY_SAVE_LONG) {
        capture_screenshot_now();
        return;
    }

    if (ui.overlay != UI_OVERLAY_NONE) {
        ui_handle_menu_keys(events);
        return;
    }

    if ((events & KEY_AUTO_LONG) && ui.mode == UI_MODE_SCOPE) {
        scope_auto_zero_begin(scope_auto_zero_long_mask());
        ui_render();
        return;
    }

    if (scope_auto_zero_active && ui.mode == UI_MODE_SCOPE) {
        return;
    }

    if (ui.mode == UI_MODE_SCOPE && scope_any_menu_open()) {
        scope_transient_menu_touch();
    }

    if (events & KEY_MENU) {
        scope_clear_menus();
        ui.scope_move_menu_ms = 0;
        ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
        dmm_pause_for_menu_overlay();
        ui.overlay = UI_OVERLAY_MODE_MENU;
        ui.menu_index = (uint8_t)ui.mode;
        if (ui.menu_index >= 4u) {
            ui.menu_index = 0;
        }
        ui.chrome_visible = 1;
        ui_render();
        return;
    }

    if (events & KEY_RIGHT) {
        ui_handle_horizontal_key(1);
        return;
    }
    if (events & KEY_LEFT) {
        ui_handle_horizontal_key(-1);
        return;
    }
    if (events & KEY_MOVE_LONG) {
        return;
    }
    if (events & KEY_F2_LONG) {
        return;
    }
    if (events & KEY_F3_LONG) {
        return;
    }
    if (events & KEY_MOVE) {
        softkeys_flash();
        if (ui.mode == UI_MODE_GEN) {
            gen_select_param(GEN_PARAM_WAVE);
            ui_render_gen_params();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_cursor_menu) {
            scope_select_cursor_menu_item(SCOPE_CURSOR_MENU_MODE);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_measure_menu) {
            scope_select_measure_menu_item(SCOPE_MEASURE_MENU_CH1);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
            scope_move_key_in_menu();
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && !ui.scope_channel_menu && !ui.scope_trigger_menu &&
                   !ui.scope_cursor_menu) {
            scope_open_move_mode();
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_channel_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_ENABLE);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_trigger_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_TRIG_SOURCE);
            ui_render_local_change();
        } else {
            primary_action();
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_F2) {
        softkeys_flash();
        if (ui.mode == UI_MODE_GEN) {
            gen_select_or_advance_param(GEN_PARAM_FREQ);
            ui_render_gen_params();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_measure_menu) {
            scope_select_measure_menu_item(SCOPE_MEASURE_MENU_VALUE);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
            ui.scope_move_sel = SCOPE_MOVE_SEL_Y;
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_channel_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_PROBE);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_trigger_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_TRIG_EDGE);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_cursor_menu) {
            scope_close_cursor_menu();
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE) {
            scope_open_cursor_menu();
            ui_render_local_change();
        } else {
            secondary_action();
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_AUTO) {
        if (ui.mode == UI_MODE_DMM) {
            ui.dmm_mode = DMM_MODE_AUTO;
            ui.auto_range = 1;
            dmm_hold_active = 0;
            dmm_rel_active = 0;
            dmm_stats_reset();
            dmm_set_mode(DMM_MODE_AUTO);
            ui_settings.dmm_mode = DMM_MODE_AUTO;
            settings_note(&ui_settings);
        } else if (ui.mode == UI_MODE_SCOPE) {
            if (ui.scope_param == SCOPE_PARAM_ZERO) {
                scope_auto_zero_begin((uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u));
            } else if (ui.scope_trigger_menu) {
                uint8_t idx = scope_trigger_source_index();
                if (scope_frame_valid) {
                    ui.scope_trigger_level = scope_channel_mid_raw(idx);
                }
            } else if (ui.scope_channel_menu) {
                scope_auto_channel_request((uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u));
            } else {
                uint8_t mask = (uint8_t)((scope_ch_enabled[0] ? 0x01u : 0u) |
                                         (scope_ch_enabled[1] ? 0x02u : 0u));
                if (!mask) {
                    mask = (uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u);
                }
                scope_auto_channel_request(mask);
            }
        } else {
            gen_select_or_advance_param(GEN_PARAM_DUTY);
        }
        if (ui.mode == UI_MODE_GEN) {
            ui_render_gen_params();
        } else {
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_F3) {
        softkeys_flash();
        if (ui.mode == UI_MODE_DMM) {
            dmm_step_group(DMM_MODE_LIVE, 2, 1);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_GEN) {
            gen_select_or_advance_param(GEN_PARAM_DUTY);
            ui_render_gen_params();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
            ui.scope_move_sel = SCOPE_MOVE_SEL_X;
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_cursor_menu) {
            scope_select_cursor_menu_item(SCOPE_CURSOR_MENU_FIRST);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_measure_menu) {
            scope_select_measure_menu_item(SCOPE_MEASURE_MENU_CH2);
            ui_render_local_change();
        } else if (ui.scope_channel_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_COUPLING);
            ui_render_local_change();
        } else if (ui.scope_trigger_menu) {
            scope_close_trigger_menu();
            ui_render_local_change();
        } else {
            scope_open_trigger_menu(SCOPE_PARAM_TRIG_TYPE);
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_OK) {
        if (ui.mode == UI_MODE_DMM) {
            dmm_toggle_hold();
            ui_render_local_change();
        } else if (scope_play_menu_value()) {
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
            scope_select_or_reset_move_zero();
            ui_render_local_change();
        } else {
            ui.running ^= 1u;
            if (ui.mode == UI_MODE_GEN) {
                gen_apply();
            } else if (ui.running) {
                scope_hw_arm();
            }
            ui_render_mode_content();
        }
        return;
    }
    if (events & KEY_UP) {
        ui_handle_vertical_key(-1, (events & KEY_REPEAT) ? 1u : 0u);
        return;
    }
    if (events & KEY_DOWN) {
        ui_handle_vertical_key(1, (events & KEY_REPEAT) ? 1u : 0u);
        return;
    }
    if (events & KEY_CH1) {
        if (ui.mode == UI_MODE_SCOPE) {
            scope_channel_key(1);
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_CH2) {
        if (ui.mode == UI_MODE_SCOPE) {
            scope_channel_key(2);
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_F4) {
        softkeys_flash();
        if (ui.mode == UI_MODE_DMM) {
            dmm_step_group(DMM_MODE_AC_HI_CURR, 4, 1);
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_GEN) {
            gen_select_or_advance_param(GEN_PARAM_AMP);
            ui_render_gen_params();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_move_mode) {
            scope_select_or_reset_move_zero();
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_measure_menu) {
            scope_close_measure_menu();
            ui_render_local_change();
        } else if (ui.mode == UI_MODE_SCOPE && ui.scope_cursor_menu) {
            scope_select_cursor_menu_item(SCOPE_CURSOR_MENU_SECOND);
            ui_render_local_change();
        } else if (ui.scope_channel_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_GAIN);
            ui_render_local_change();
        } else if (ui.scope_trigger_menu) {
            scope_select_or_advance_param(SCOPE_PARAM_TRIG_LEVEL);
            ui_render_local_change();
        } else {
            scope_open_measure_menu();
            ui_render_local_change();
        }
        return;
    }
    if (events & KEY_SAVE) {
        if (ui.mode == UI_MODE_DMM) {
            dmm_toggle_relative();
            ui_render_local_change();
        } else {
            if (ui.mode == UI_MODE_SCOPE) {
                ui.scope_afterglow ^= 1u;
                if (!ui.scope_afterglow) {
                    scope_after_valid[0] = 0;
                    scope_after_valid[1] = 0;
                }
            }
            ui_render_local_change();
        }
    }
}

void ui_dmm_measurement_updated(void) {
    if (ui.mode == UI_MODE_DMM) {
        dmm_stats_update();
        if (ui.overlay != UI_OVERLAY_NONE) {
            return;
        }
        ui_render_dmm_reading();
    }
}

uint8_t ui_consume_beep_preview(void) {
    uint8_t requested = beep_preview_requested;
    beep_preview_requested = 0;
    return requested;
}

uint8_t ui_auto_sleep_due(void) {
    return ui.sleep_due;
}

uint8_t ui_diode_beep_enabled(void) {
    return ui.mode == UI_MODE_DMM && ui.overlay == UI_OVERLAY_NONE && ui.dmm_mode == DMM_MODE_DIODE;
}

uint8_t ui_live_beep_enabled(void) {
    return ui.mode == UI_MODE_DMM && ui.overlay == UI_OVERLAY_NONE && ui.dmm_mode == DMM_MODE_LIVE;
}

void ui_set_live_wire_detected(uint8_t detected) {
    detected = detected ? 1u : 0u;
    if (dmm_live_wire_alert == detected) {
        return;
    }
    dmm_live_wire_alert = detected;
    if (ui.mode == UI_MODE_DMM && ui.overlay == UI_OVERLAY_NONE) {
        ui_render_dmm_reading();
    }
}

void ui_set_load_sample(uint32_t active_ticks, uint32_t idle_ticks) {
    uint32_t total = active_ticks + idle_ticks;
    if (!total) {
        return;
    }

    uint8_t load = (uint8_t)((active_ticks * 100u + total / 2u) / total);
    if (load > 100u) {
        load = 100;
    }
    ui_load_pct = (uint8_t)(((uint16_t)ui_load_pct * 3u + load + 2u) / 4u);
    if (idle_ticks && ui_last_render_ticks) {
        uint32_t render_x10 = (ui_last_render_ticks * 200u + idle_ticks / 2u) / idle_ticks;
        ui_render_ms_x10 = render_x10 > 999u ? 999u : (uint16_t)render_x10;
    }
    if (load > ui_load_peak_pct) {
        ui_load_peak_pct = load;
    } else if (ui_load_peak_pct > ui_load_pct + 1u) {
        ui_load_peak_pct--;
    } else {
        ui_load_peak_pct = ui_load_pct;
    }
}

void ui_tick(uint32_t elapsed_ms) {
    uint8_t rendered = 0;
    uint8_t battery_changed = 0;
    uint8_t fw_update_changed = 0;
    fw_update_status_t fw_status;

    if (ui_settings.sleep_enabled) {
        uint32_t sleep_next = ui.sleep_ms + elapsed_ms;
        if (sleep_next >= SETTINGS_SLEEP_TIMEOUT_MS) {
            ui.sleep_ms = SETTINGS_SLEEP_TIMEOUT_MS;
            ui.sleep_due = 1;
        } else {
            ui.sleep_ms = sleep_next;
        }
    } else {
        ui.sleep_ms = 0;
        ui.sleep_due = 0;
    }

    uint32_t charger_next = (uint32_t)ui.charger_ms + elapsed_ms;
    if (charger_next >= CHARGER_POLL_MS) {
        ui.charger_ms = 0;
        battery_update_charging_status();
        battery_changed = battery_snapshot_changed();
    } else {
        ui.charger_ms = (uint16_t)charger_next;
    }

    uint32_t battery_next = (uint32_t)ui.battery_ms + elapsed_ms;
    if (battery_next >= BATTERY_MEASURE_MS) {
        ui.battery_ms = 0;
        battery_update();
        battery_changed = (uint8_t)(battery_snapshot_changed() || battery_changed);
    } else {
        ui.battery_ms = (uint16_t)battery_next;
    }

    battery_changed = (uint8_t)(battery_blink_tick(elapsed_ms) || battery_changed);

    fw_update_status(&fw_status);
    if (fw_status.sequence != ui_fw_update_sequence) {
        ui_fw_update_sequence = fw_status.sequence;
        fw_update_changed = 1;
    }

    if (ui.screenshot_overlay_ms) {
        if (elapsed_ms >= ui.screenshot_overlay_ms) {
            ui.screenshot_overlay_ms = 0;
            ui_render();
            rendered = 1;
        } else {
            ui.screenshot_overlay_ms = (uint16_t)(ui.screenshot_overlay_ms - elapsed_ms);
        }
    }

    if (gen_deferred_apply) {
        if (elapsed_ms >= gen_deferred_apply_ms) {
            gen_apply();
        } else {
            gen_deferred_apply_ms = (uint16_t)(gen_deferred_apply_ms - elapsed_ms);
        }
    }

    if (ui.overlay != UI_OVERLAY_NONE) {
        if (battery_changed || fw_update_changed) {
            ui_render();
        }
        return;
    }

    if (fw_update_changed) {
        ui_render();
        rendered = 1;
    }

    if (battery_changed) {
        ui_redraw_battery_status();
        rendered = 1;
    }

    if (ui.mode == UI_MODE_SCOPE && scope_any_menu_open() && ui.scope_move_menu_ms) {
        if (elapsed_ms >= ui.scope_move_menu_ms) {
            scope_clear_menus();
            ui.scope_move_menu_ms = 0;
            ui.softkeys_ms = 0;
            ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
            if (!rendered) {
                ui_render();
                rendered = 1;
            }
        } else {
            ui.scope_move_menu_ms = (uint16_t)(ui.scope_move_menu_ms - elapsed_ms);
        }
    }

    if (ui.chrome_visible) {
        ui.softkeys_ms = 0;
    } else if (ui.softkeys_ms) {
        if (elapsed_ms >= ui.softkeys_ms) {
            ui.softkeys_ms = 0;
            ui_render();
            rendered = 1;
        } else {
            ui.softkeys_ms = (uint16_t)(ui.softkeys_ms - elapsed_ms);
        }
    }

    if (SCOPE_UI_SAFE_STUB && ui.mode == UI_MODE_SCOPE && ui.running) {
        uint32_t scope_next = (uint32_t)ui.scope_ms + elapsed_ms;
        if (scope_next >= SCOPE_STUB_FRAME_MS) {
            ui.scope_ms = 0;
            ui.scope_phase = (uint16_t)(ui.scope_phase + 3u + scope_safe_timebase() * 3u);
            if (!rendered) {
                ui_render_scope_frame();
            }
        } else {
            ui.scope_ms = (uint16_t)scope_next;
        }
    } else if (!SCOPE_UI_SAFE_STUB && ui.mode == UI_MODE_SCOPE && ui.running) {
        uint16_t frame_ms = scope_frame_interval_ms();
        uint32_t scope_next = (uint32_t)ui.scope_ms + elapsed_ms;
        uint32_t frame_age = (uint32_t)scope_frame_age_ms + elapsed_ms;

        scope_frame_age_ms = frame_age > 60000u ? 60000u : (uint16_t)frame_age;
        if (scope_next >= frame_ms) {
            ui.scope_ms = (uint16_t)(scope_next - frame_ms);
            ui.scope_phase = (uint16_t)(ui.scope_phase + 7u + scope_safe_timebase() * 3u);
            if (scope_poll_frame()) {
                uint16_t interval = (uint16_t)((scope_frame_age_ms + 5u) / 10u);
                scope_frame_interval_x10ms = interval > 999u ? 999u : interval;
                scope_frame_age_ms = 0;
                if (!rendered) {
                    ui_render_scope_frame();
                }
            }
        } else {
            ui.scope_ms = (uint16_t)scope_next;
        }
    } else {
        ui.scope_ms = 0;
        scope_frame_age_ms = 0;
    }

    if (!ui.chrome_visible) {
        return;
    }

    uint32_t next = (uint32_t)ui.idle_ms + elapsed_ms;
    if (next >= CHROME_IDLE_MS) {
        ui.chrome_visible = 0;
        ui.idle_ms = 0;
        if (!rendered) {
            ui_render();
        }
    } else {
        ui.idle_ms = (uint16_t)next;
    }
}
