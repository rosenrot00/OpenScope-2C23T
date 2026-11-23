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
#include "bootloader.h"
#include "fft.h"
#include "gfx.h"
#include "ui_icons.h"
#include "arb_csv.h"

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
#define FIRMWARE_VERSION_TEXT "v2026.11.7"
#ifndef SCOPE_UI_SAFE_STUB
#define SCOPE_UI_SAFE_STUB 1
#endif
#define SCOPE_ZERO_STEP 3
#define SCOPE_H_POS_LIMIT 120
#define SCOPE_V_POS_LIMIT 120
#define SCOPE_X_DIVS 12u
#define SCOPE_Y_DIVS 8u
#define SCOPE_TIMEBASE_DEFAULT 4u
#define SCOPE_AUTO_TIMEBASE_MAX 16u
#define SCOPE_AUTO_VDIV_START 1u
#define SCOPE_AUTO_STABLE_FRAMES 2u
#define SCOPE_AUTO_HALFPERIOD_MIN 5u
#define SCOPE_AUTO_HALFPERIOD_MAX 60u
#define SCOPE_AUTO_Y_TARGET_MAX 176u
#define SCOPE_AUTO_Y_CLIP_LOW 8u
#define SCOPE_AUTO_Y_CLIP_HIGH 247u
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

typedef enum {
    MENU_ITEM_PLOT = 0,
    MENU_ITEM_FFT_WINDOW,
    MENU_ITEM_FFT_DISPLAY,
    MENU_ITEM_MATH_MODE,
    MENU_ITEM_MATH_OP,
    MENU_ITEM_HIDE_TRACES,

    MENU_ITEM_BODE_START,
    MENU_ITEM_BODE_STOP,
    MENU_ITEM_BODE_STEPS,
    MENU_ITEM_BODE_SWEEP
} math_menu_item_t;

static const uint8_t math_menu_order[] = {
    MENU_ITEM_PLOT,
    MENU_ITEM_FFT_WINDOW,
    MENU_ITEM_FFT_DISPLAY,
    MENU_ITEM_MATH_MODE,
    MENU_ITEM_MATH_OP,
    MENU_ITEM_HIDE_TRACES
};

static const uint8_t bode_menu_order[] = {
    MENU_ITEM_PLOT,
    MENU_ITEM_BODE_START,
    MENU_ITEM_BODE_STOP,
    MENU_ITEM_BODE_STEPS,
    MENU_ITEM_BODE_SWEEP
};


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
    uint16_t boot_export_overlay_ms;
    uint16_t scope_phase;
    uint8_t battery_blink_on;
    uint8_t menu_index;
    uint8_t settings_row;
    uint8_t sleep_due;
    uint8_t dmm_render_pending;
    uint8_t screenshot_state;
    uint8_t screenshot_reason;
    uint8_t reset_confirm;
    uint8_t storage_known;
    uint32_t storage_free_kb;
    uint32_t storage_total_kb;
    uint8_t boot_export_state;
    uint8_t boot_export_result;
    uint8_t boot_export_percent;
} ui_state_t;

typedef struct {
    uint8_t valid;
    uint8_t pct;
    uint8_t charging;
    uint8_t full;
    uint8_t usb_configured;
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
#if HW_TARGET_HW40
    SCOPE_SOFT_ROLL_TIMEBASE = 255,
    SCOPE_IRQ_ROLL_TIMEBASE_START = 21,
    SCOPE_SLOW_ROLL_TIMEBASE_START = 21,
#else
    SCOPE_SOFT_ROLL_TIMEBASE = 18,
    SCOPE_IRQ_ROLL_TIMEBASE_START = 19,
    SCOPE_SLOW_ROLL_TIMEBASE_START = SCOPE_SOFT_ROLL_TIMEBASE,
#endif
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
static uint8_t gen_running;
// What the user asked for, as opposed to what the hardware is doing.
// Entering the multimeter silences the generator through gen_stop(), which
// clears gen_running; this flag survives so the output can be brought back
// on the way out instead of disappearing without the user asking.
static uint8_t gen_user_enabled;
static uint8_t gen_sweep_menu_active = 0;
static uint8_t gen_sweep_row_selected = 0; // 0 to 4
static uint8_t ui_edit_digit_index = 0; // 0 to 3 (which of the 4 digits is active)
static uint8_t ui_edit_digit_active = 0; // 1 if we clicked into a digit editor field 
static uint8_t gen_sweep_mode = 0;
static uint32_t siggen_sweep_start_hz = 0;
static uint32_t siggen_sweep_stop_hz = 9999;
static uint16_t gen_sweep_ms = 1000;
static uint8_t gen_fm_mode = 0;
static uint8_t gen_fm_source = 0;
static uint32_t gen_fm_freq_hz = 5;
static uint32_t gen_sweep_elapsed_ms = 0;
static uint8_t sweep_start_unit = 0;
static uint8_t sweep_stop_unit = 0;
static uint8_t fm_freq_unit = 0;
static uint16_t gen_fm_phase;
static uint8_t arb_file_index;
static uint8_t arb_loaded;
static uint16_t arb_sample_count;
static uint8_t arb_samples[2048];
static uint8_t scope_samples[SCOPE_SAMPLE_BYTES] __attribute__((aligned(4)));
static uint8_t scope_capture_samples[SCOPE_SAMPLE_BYTES] __attribute__((aligned(4)));
static uint8_t scope_frame_valid;
static uint16_t scope_frame_id;
static uint8_t scope_min_raw[2];
static uint8_t scope_max_raw[2];
static uint8_t scope_avg_raw[2];
static uint32_t scope_sq_sum_raw[2];
static int32_t scope_mean_mv[2];
static uint8_t scope_measure_min_hist[2][SCOPE_MEASURE_WINDOW];
static uint8_t scope_measure_max_hist[2][SCOPE_MEASURE_WINDOW];
static uint16_t scope_measure_rms_hist[2][SCOPE_MEASURE_WINDOW];
static uint32_t scope_measure_freq_hist[2][SCOPE_MEASURE_WINDOW];
static int32_t scope_measure_mean_hist[2][SCOPE_MEASURE_WINDOW];
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
static uint8_t scope_auto_time_active;
static uint8_t scope_auto_time_mask;
static uint8_t scope_auto_time_steps_left;
static uint8_t scope_auto_channel_mask;
static uint8_t scope_auto_channel_steps_left[2];
static uint8_t scope_auto_channel_stable_count[2];
static uint16_t scope_auto_last_frame;
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
static uint8_t ui_math_menu_visible = 0; // 0 = Hidden, 1 = Displayed on Screen
static uint8_t scope_math_selected = 0; // 0=MODE, 1=MATH OP, 2=FFT DISPLAY/XY, 3=WINDOW, 4=CLEAR HISTORY, 5=HIDE TRACES
static uint8_t scope_math_mode = 0; // 0 = OFF, 1 = ON
static uint8_t scope_math_op = 0; // 0 = A+B, 1 = A-B, 2 = B-A
static uint8_t scope_fft_src = 0; // 0 = OFF, 1 = CH1, 2 = CH2, 3 = XY MODE, 4 = BODE
static uint8_t scope_fft_window = 0;  // 0 = HANN, 1 = HAMMING, 2 = BLACKMAN, 3 = RECTANGLE
static uint8_t scope_fft_display = 0; // 0 = NORMAL, 1 = AVERAGING, 2 = MAX HOLD
// Static array to preserve history across frames for display processing
static float fft_history[FFT_SIZE / 2] = {0.0f};
static float fft_input_buffer[FFT_SIZE];
static float fft_output_buffer[FFT_SIZE / 2];
static uint8_t scope_hide_traces = 0; // 0=NONE, 1=CH1, 2=CH2, 3=ALL
static uint32_t bode_start_hz = 100u;
static uint32_t bode_stop_hz = 100000u;
static uint8_t bode_steps = 40u;
static uint8_t bode_start_unit = 0;
static uint8_t bode_stop_unit = 0;
static uint8_t bode_current_step = 0;
static uint8_t bode_is_sweeping = 0;
static uint8_t bode_cursor_sub = 0;  // 0=F (freq), 1=DB, 2=DEG
enum {
    BODE_MAX_STEPS = SETTINGS_BODE_MAX_STEPS,
    BODE_DWELL_FRAMES = 4u,
};
static float bode_gain_db[BODE_MAX_STEPS];
static float bode_phase_deg[BODE_MAX_STEPS];
static uint8_t bode_dwell_frames = 0;
static uint32_t bode_step_freq_hz = 0;
static uint8_t bode_siggen_active = 0;
static uint8_t scope_fft_src_menu_saved = 0;

static uint8_t bode_clamp_steps(uint32_t steps) {
    if (steps < SETTINGS_BODE_MIN_STEPS) return SETTINGS_BODE_MIN_STEPS;
    if (steps > BODE_MAX_STEPS) return BODE_MAX_STEPS;
    return (uint8_t)steps;
}
typedef struct {
    uint8_t valid;
    uint8_t scope_timebase;
    uint8_t trigger_source;
    uint8_t trigger_mode;
    uint8_t ch_enabled[2];
    uint8_t hide_traces;
} bode_saved_state_t;
static bode_saved_state_t bode_saved;
static int32_t dmm_rel_ref_fixed;
static char dmm_hold_value[10];
static uint8_t dmm_hold_range_decimals;
static char dmm_hold_unit[6];
static char dmm_rel_unit[6];
static char dmm_rel_value[12];
static char dmm_rel_ref_value[10];
static char dmm_low_current_value[10];
static settings_state_t ui_settings;

static void ui_settings_copy(settings_state_t *dst, const settings_state_t *src) {
    dst->dmm_mode = src->dmm_mode;
    dst->beep_level = src->beep_level;
    dst->brightness_level = src->brightness_level;
    dst->startup_screen = src->startup_screen;
    dst->last_screen = src->last_screen;
    dst->sleep_enabled = src->sleep_enabled;
    dst->scope_timebase = src->scope_timebase;
    dst->scope_display = src->scope_display;
    dst->scope_cursor_mode = src->scope_cursor_mode;
    dst->scope_trigger_source = src->scope_trigger_source;
    dst->scope_trigger_mode = src->scope_trigger_mode;
    dst->scope_trigger_edge = src->scope_trigger_edge;
    dst->scope_trigger_level = src->scope_trigger_level;
    dst->scope_afterglow = src->scope_afterglow;
    dst->scope_active_ch = src->scope_active_ch;
    dst->scope_measure_param = src->scope_measure_param;
    dst->scope_measure_visible = src->scope_measure_visible;
    dst->scope_h_value_pos = src->scope_h_value_pos;
    dst->scope_h_value_timebase = src->scope_h_value_timebase;
    dst->scope_math_mode = src->scope_math_mode;
    dst->scope_math_op = src->scope_math_op;
    dst->scope_fft_src = src->scope_fft_src;
    dst->scope_math_selected = src->scope_math_selected;
    dst->scope_fft_window = src->scope_fft_window;
    dst->scope_fft_display = src->scope_fft_display;
    dst->scope_hide_traces = src->scope_hide_traces;
    dst->siggen_wave = src->siggen_wave;
    dst->siggen_param = src->siggen_param;
    dst->siggen_duty_percent = src->siggen_duty_percent;
    dst->siggen_amp_tenths_v = src->siggen_amp_tenths_v;
    dst->siggen_freq_unit = src->siggen_freq_unit;
    dst->siggen_running = src->siggen_running;
    dst->siggen_freq_hz = src->siggen_freq_hz;
    dst->siggen_sweep_mode = src->siggen_sweep_mode;
    dst->siggen_sweep_ms = src->siggen_sweep_ms;
    dst->siggen_sweep_start_hz = src->siggen_sweep_start_hz;
    dst->siggen_sweep_stop_hz = src->siggen_sweep_stop_hz;
    dst->siggen_fm_mode = src->siggen_fm_mode;
    dst->siggen_fm_source = src->siggen_fm_source;
    dst->siggen_fm_freq_hz = src->siggen_fm_freq_hz;
    dst->bode_start_hz = src->bode_start_hz;
    dst->bode_stop_hz = src->bode_stop_hz;
    dst->bode_steps = src->bode_steps;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        dst->scope_ch_enabled[ch] = src->scope_ch_enabled[ch];
        dst->scope_probe_x10[ch] = src->scope_probe_x10[ch];
        dst->scope_vdiv[ch] = src->scope_vdiv[ch];
        dst->scope_coupling_dc[ch] = src->scope_coupling_dc[ch];
        dst->scope_ch_pos[ch] = src->scope_ch_pos[ch];
        dst->scope_measure_mask[ch] = src->scope_measure_mask[ch];
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
    // The settings page is a 3-wide grid of tiles with one wide tile on the
    // last line. INFO is that wide tile: it is read-only, so it sits past
    // the selectable rows and the cursor simply never reaches it.
    SETTINGS_ROW_BEEP = 0,
    SETTINGS_ROW_DISPLAY = 1,
    SETTINGS_ROW_START = 2,
    SETTINGS_ROW_SLEEP = 3,
    SETTINGS_ROW_BOOT_EXPORT = 4,
    SETTINGS_ROW_RESET = 5,
    SETTINGS_ROW_INFO = 6,
    SETTINGS_ROW_COUNT = 7,
    SETTINGS_SELECTABLE_COUNT = SETTINGS_ROW_INFO,
    SETTINGS_BOOT_EXPORT_ROW = SETTINGS_ROW_BOOT_EXPORT,
    SETTINGS_GRID_COLUMNS = 3,
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
    SCOPE_CURSOR_BODE,
    SCOPE_CURSOR_COUNT,
};

static const uint16_t C_BG = RGB565(2, 4, 8);
static const uint16_t C_TOP = RGB565(8, 14, 22);
static const uint16_t C_PANEL_2 = RGB565(20, 30, 44);
static const uint16_t C_GRID = RGB565(26, 36, 48);
static const uint16_t C_TEXT = RGB565(250, 252, 255);
static const uint16_t C_MUTED = RGB565(148, 163, 184);
static const uint16_t C_WARN = RGB565(244, 63, 94);
static const uint16_t C_CH1 = RGB565(250, 204, 21);
static const uint16_t C_CH2 = RGB565(0, 242, 254);
static const uint16_t C_DMM = RGB565(52, 211, 153);
static const uint16_t C_SCOPE = RGB565(56, 189, 248);
static const uint16_t C_GEN = RGB565(192, 132, 252);

// Modern UI palette (multimeter, menu, settings, scope).
static const uint16_t D_BG = RGB565(11, 14, 19);
static const uint16_t D_CARD = RGB565(21, 26, 34);
static const uint16_t D_TRACK = RGB565(38, 45, 58);
static const uint16_t D_TEXT = RGB565(243, 245, 249);
static const uint16_t D_SUB = RGB565(199, 207, 219);
static const uint16_t D_MUTED = RGB565(138, 148, 166);
static const uint16_t D_ACCENT = RGB565(90, 200, 250);
static const uint16_t D_ACCENT_TINT = RGB565(31, 58, 73);
static const uint16_t D_AMBER = RGB565(255, 181, 71);
static const uint16_t D_RED = RGB565(255, 92, 108);
static const uint16_t D_RED_TEXT = RGB565(255, 122, 134);
static const uint16_t D_RED_CARD = RGB565(58, 18, 25);
static const uint16_t D_RED_SUB = RGB565(255, 201, 207);
static const uint16_t D_GREEN = RGB565(52, 211, 153);
static const uint16_t D_GREEN_CARD = RGB565(16, 36, 28);
static const uint16_t D_GREEN_TINT = RGB565(27, 68, 52);
static const uint16_t D_GREEN_SUB = RGB565(185, 233, 211);
static const uint16_t D_GEN = RGB565(167, 139, 250);
static const uint16_t D_SCOPE = RGB565(52, 211, 153);
static const uint16_t D_PLOT = RGB565(14, 18, 25);
static const uint16_t D_PLOT_DOT = RGB565(52, 62, 78);
static const uint16_t D_PLOT_AXIS = RGB565(86, 100, 118);

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
    DMM_MODE_TEMP_F, // UI only; the meter IC has one temperature command.
    DMM_MODE_COUNT,
};

enum {
    DMM_GROUP_VOLTAGE,
    DMM_GROUP_TEST,
    DMM_GROUP_SENSE,
    DMM_GROUP_CURRENT,
    DMM_GROUP_COUNT,
};

static const struct {
    uint8_t count;
    uint8_t modes[4];
} dmm_groups[DMM_GROUP_COUNT] = {
    {3, {DMM_MODE_DCV, DMM_MODE_ACV, DMM_MODE_RES}},
    {2, {DMM_MODE_DIODE, DMM_MODE_CAP}},
    {3, {DMM_MODE_TEMP, DMM_MODE_TEMP_F, DMM_MODE_LIVE}},
    {4, {DMM_MODE_AC_HI_CURR, DMM_MODE_DC_HI_CURR, DMM_MODE_AC_LO_CURR, DMM_MODE_DC_LO_CURR}},
};
static uint8_t dmm_group_choice[DMM_GROUP_COUNT] = {0, 0, 0, 1};

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
// Screen labels, with the SI prefixes spelled properly: "US" is not a unit
// and "MS" could be read as megaseconds. Display only -- the USB protocol
// uses scope_timebases_short[] below, which keeps its own spelling.
static const char *const scope_timebases[] = {
    "50ns", "100ns", "200ns", "500ns", "1us", "2us", "5us",
    "10us", "20us", "50us", "100us", "200us", "500us",
    "1ms", "2ms", "5ms", "10ms", "20ms", "50ms",
    "100ms", "200ms", "500ms", "1s", "2s", "5s", "10s",
};
static const char *const scope_timebases_short[] = {
    "50N", "100N", "200N", "500N", "1U", "2U", "5U",
    "10U", "20U", "50U", "100U", "200U", "500U",
    "1M", "2M", "5M", "10M", "20M", "50M",
    "100M", "200M", "500M", "1S", "2S", "5S", "10S",
};
// Sent over USB, where the host has parsed this spelling since before we
// touched it. The screen uses the tables below instead.
static const char *const scope_vdivs_short[] = {
    "20M", "50M", "100M", "200M", "500M", "1V", "2V", "5V", "10V",
};
static const char *const scope_vdivs_x10_short[] = {
    "200M", "500M", "1V", "2V", "5V", "10V", "20V", "50V", "100V",
};
// Shown on screen, kept in two pieces so the bar can shrink the unit on
// its own when the pair will not fit. See draw_scope_channel_range().
static const char *const scope_vdiv_value[] = {
    "20", "50", "100", "200", "500", "1", "2", "5", "10",
};
static const char *const scope_vdiv_unit[] = {
    "mV", "mV", "mV", "mV", "mV", "V", "V", "V", "V",
};
static const char *const scope_vdiv_x10_value[] = {
    "200", "500", "1", "2", "5", "10", "20", "50", "100",
};
static const char *const scope_vdiv_x10_unit[] = {
    "mV", "mV", "V", "V", "V", "V", "V", "V", "V",
};
static const uint32_t scope_vdiv_base_mv[] = {20u, 50u, 100u, 200u, 500u, 1000u, 2000u, 5000u, 10000u};
static const uint16_t scope_vdiv_range_code[] = {1000u, 400u, 200u, 100u, 40u, 20u, 10u, 4u, 2u};
enum {
    SCOPE_VDIV_COUNT = sizeof(scope_vdiv_base_mv) / sizeof(scope_vdiv_base_mv[0]),
};
static const char *const scope_display_labels[] = {"Y-T", "ROLL", "X-Y"};
static const char *const scope_display_short[] = {"YT", "ROLL", "XY"};
static const char *const scope_cursor_labels[] = {"CUR OFF", "T CUR", "Y CUR", "B CUR"};
static const char *const scope_cursor_short[] = {"OFF", "T", "Y", "B"};
static const char *const scope_trigger_short[] = {"AUTO", "NORM", "SNGL"};
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
static const uint8_t gen_wave_order[] = {
    SIGGEN_WAVE_SINE,
    SIGGEN_WAVE_SQUARE,
    SIGGEN_WAVE_TRIANGLE,
    SIGGEN_WAVE_FULL,
    SIGGEN_WAVE_HALF,
    SIGGEN_WAVE_NOISE,
    SIGGEN_WAVE_DC,
    SIGGEN_WAVE_ARBITRARY,
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
// One table for the frequency units. Five places used to declare their
// own copy, which is how "KHZ" survived in some of them long after the
// rest of the screen had learned to spell it.
static const char *const freq_unit_labels[] = {"Hz", "kHz", "MHz"};
static const char *const startup_labels[] = {"MENU", "DMM", "SCOPE", "GEN"};
static const char *const beep_level_labels[] = {"OFF", "LOW", "MEDIUM", "HIGH", "MAX"};
static const char *const brightness_level_labels[] = {"DIM", "LOW", "MEDIUM", "HIGH", "BRIGHT"};
static const char *const sleep_labels[] = {"OFF", "10 MIN", "20 MIN", "30 MIN"};
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
static void dmm_apply_selected_mode(uint8_t mode);
static uint8_t dmm_text_decimals(const char *text);
static int32_t dmm_pow10(uint8_t n);
static void dmm_format_fixed(int32_t value, uint8_t stored, uint8_t shown, uint8_t trim, char out[12]);
static void gen_prepare_state(void);
static uint32_t gen_get_current_sweep_freq(void);
static void gen_apply(void);
static void gen_stop(void);
static void gen_normalize_param(void);
static void draw_gen_param_row(uint16_t x, uint16_t y);
static void draw_scope_param_row(uint16_t x, uint16_t y);
static void draw_scope_channel_param_row(uint16_t x, uint16_t y);
static void draw_scope_trigger_param_row(uint16_t x, uint16_t y);
static uint16_t scope_channel_color(uint8_t ch);
static uint32_t scope_vdiv_mv_for_channel(uint8_t idx);
static uint16_t scope_range_code_for_channel(uint8_t idx);
static uint32_t scope_vdiv_mv(void);
static int32_t scope_raw_delta_mv(uint8_t idx, uint8_t raw);
static void scope_format_signed_mv(char out[11], int32_t mv);
static void scope_format_u32(char *out, uint32_t value);
static uint16_t scope_visible_sample_count(void);
static int32_t scope_h_pos_sample_offset(void);
static uint16_t scope_capture_window_start(void);
static void scope_format_delta_time(char out[14]);
static void scope_format_delta_level(char out[14]);
static void scope_format_frequency_reading(char out[12], uint32_t hz);
static void scope_text_copy(char *dst, const char *src, uint8_t max_len);
static uint8_t scope_text_len(const char *text);
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
static void scope_auto_cancel(void);
static void scope_auto_timeout_service(void);
static void scope_auto_setup_request(uint8_t mask);
static uint8_t scope_auto_time_service(void);
static void scope_auto_channel_request(uint8_t mask);
static void scope_auto_channel_service(void);
static uint8_t scope_auto_channel_tune_vdiv(uint8_t idx);
static void scope_set_channel_pos_from_avg(uint8_t idx);
static void ui_apply_saved_runtime_settings(void);
static uint8_t scope_any_menu_open(void);
static uint8_t scope_softkey_menu_open(void);
static void scope_clear_menus(void);
static void scope_trace_invalidate(void);
static void scope_apply_settings_keep_frame(uint8_t keep_frame);
static void ui_render_scope_frame(void);
static uint32_t bode_step_freq_for(uint8_t step, uint8_t total_steps);
static uint8_t bode_auto_timebase(uint32_t freq_hz);
static void bode_begin_sweep(void);
static void bode_shutdown(uint8_t restore_scope_hw);
static void bode_apply_step_freq(void);
static uint8_t bode_measure_step(uint8_t step);
static void bode_sweep_service(void);
static uint16_t bode_extract_channel(uint8_t target_ch, float *dst);
static void scope_fft_src_apply_from(uint8_t old_src, uint8_t new_src);
static void ui_draw_bode(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh);

static uint16_t mode_accent(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        return C_SCOPE;
    }
    if (ui.mode == UI_MODE_GEN) {
        return C_GEN;
    }
    return C_DMM;
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

static uint32_t gen_freq_unit_multiplier(uint8_t unit) {
    if (unit == 2u) {
        return 1000000u;
    }
    if (unit == 1u) {
        return 1000u;
    }
    return 1u;
}

static uint32_t freq_clamp_display(uint32_t hz, uint8_t unit) {
    uint32_t mult = gen_freq_unit_multiplier(unit);
    if (!mult) return 1u;
    uint32_t value = hz / mult;
    uint32_t max_val = GEN_MAX_FREQ_HZ / mult;
    if (max_val > 9999u) max_val = 9999u;
    if (value < 1u) return 1u;
    if (value > max_val) return max_val;
    return value;
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

static void freq_sync_display_unit(uint8_t *unit, uint32_t hz) {
    if (*unit >= GEN_FREQ_UNIT_COUNT) {
        *unit = 0;
    }
    while (*unit + 1u < GEN_FREQ_UNIT_COUNT &&
           hz > (uint32_t)gen_freq_unit_max_value(*unit) * gen_freq_unit_multiplier(*unit)) {
        ++*unit;
    }
}

static void gen_freq_sync_editor_unit(void) {
    freq_sync_display_unit(&gen_freq_unit, ui.gen_freq_hz);
}

static void format_gen_freq_digits(char out[5]) {
    uint16_t value = gen_freq_editor_value();

    out[0] = (char)('0' + (value / 1000u) % 10u);
    out[1] = (char)('0' + (value / 100u) % 10u);
    out[2] = (char)('0' + (value / 10u) % 10u);
    out[3] = (char)('0' + value % 10u);
    out[4] = 0;
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

static void battery_snapshot_store(void) {
    battery_snapshot.valid = 1;
    battery_snapshot.pct = battery_percent();
    battery_snapshot.charging = battery_is_charging();
    battery_snapshot.full = battery_display_full();
    battery_snapshot.usb_configured = usb_cdc_is_configured();
}

static uint8_t battery_snapshot_changed(void) {
    uint8_t pct = battery_percent();
    uint8_t charging = battery_is_charging();
    uint8_t full = battery_display_full();
    uint8_t usb_configured = usb_cdc_is_configured();

    if (!battery_snapshot.valid ||
        pct != battery_snapshot.pct ||
        charging != battery_snapshot.charging ||
        full != battery_snapshot.full ||
        usb_configured != battery_snapshot.usb_configured) {
        battery_snapshot.valid = 1;
        battery_snapshot.pct = pct;
        battery_snapshot.charging = charging;
        battery_snapshot.full = full;
        battery_snapshot.usb_configured = usb_configured;
        return 1;
    }
    return 0;
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

// Redrawn as rounded dialogs further down, next to the other modern UI.
static void draw_fw_update_overlay(uint16_t bg);
static void draw_screenshot_overlay(uint16_t bg);

static uint8_t dmm_mode_group(uint8_t mode, uint8_t *choice) {
    if (mode == DMM_MODE_CONT) mode = DMM_MODE_DIODE;
    for (uint8_t group = 0; group < DMM_GROUP_COUNT; ++group) {
        for (uint8_t i = 0; i < dmm_groups[group].count; ++i) {
            if (dmm_groups[group].modes[i] == mode) {
                if (choice) *choice = i;
                return group;
            }
        }
    }
    return DMM_GROUP_COUNT;
}

static uint8_t dmm_is_temperature(uint8_t mode) {
    return mode == DMM_MODE_TEMP || mode == DMM_MODE_TEMP_F;
}

static uint8_t dmm_wire_mode_for_ui(uint8_t mode) {
    return mode == DMM_MODE_TEMP_F ? DMM_MODE_TEMP : mode;
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
    if (scope_hide_traces >= 4u ||
        (scope_hide_traces == 3u && !scope_math_mode && !scope_fft_src)) {
        scope_hide_traces = 0u;
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
    uint8_t mode = scope_safe_cursor();

    if (scope_fft_src == 4u && mode == SCOPE_CURSOR_BODE) {
        if (bode_cursor_sub == 0u) return "F CUR";
        if (bode_cursor_sub == 1u) return "DB CUR";
        return "PH CUR";
    }
    if ((scope_fft_src == 1u || scope_fft_src == 2u) && mode == SCOPE_CURSOR_TIME) {
        return "F CUR";
    }
    return scope_cursor_labels[mode];
}

static const char *scope_cursor_chip_label(void) {
    uint8_t mode = scope_safe_cursor();

    if (scope_fft_src == 4u && mode == SCOPE_CURSOR_BODE) {
        if (bode_cursor_sub == 1u) return "DB";
        if (bode_cursor_sub == 2u) return "DEG";
        return "F";
    }
    if ((scope_fft_src == 1u || scope_fft_src == 2u) && mode == SCOPE_CURSOR_TIME) {
        return "F";
    }
    return scope_cursor_short[mode];
}

static const char *scope_timebase_label(void) {
    return scope_timebases[scope_safe_timebase()];
}

static const char *scope_timebase_chip_label(void) {
    return scope_timebases_short[scope_safe_timebase()];
}

// The same range for the eye rather than for the host: "500mV", not the
// "500M" that reads as five hundred megavolts.
static void scope_vdiv_display(uint8_t idx, const char **value, const char **unit) {
    uint8_t vdiv;

    if (idx >= 2u) {
        idx = 0;
    }
    vdiv = scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
    if (scope_probe_x10[idx]) {
        *value = scope_vdiv_x10_value[vdiv];
        *unit = scope_vdiv_x10_unit[vdiv];
    } else {
        *value = scope_vdiv_value[vdiv];
        *unit = scope_vdiv_unit[vdiv];
    }
}

enum {
    // What the bottom bar has for one channel before the next field
    // starts. Eight characters at scale 2, and no wrapping: a range that
    // overflows writes over the timebase beside it.
    SCOPE_RANGE_FIELD_W = 96,
};

static const char *scope_vdiv_menu_label(void) {
    static char label[8];
    const char *value;
    const char *unit;

    scope_vdiv_display(ui.active_ch == 2u ? 1u : 0u, &value, &unit);
    scope_text_copy(label, value, sizeof(label));
    ui_text_append(label, unit, sizeof(label));
    return label;
}

static const char *scope_vdiv_chip_label_for_channel(uint8_t idx) {
    uint8_t vdiv;

    if (idx >= 2u) {
        idx = 0;
    }
    vdiv = scope_vdiv_ch[idx] < SCOPE_VDIV_COUNT ? scope_vdiv_ch[idx] : 0u;
    return scope_probe_x10[idx] ? scope_vdivs_x10_short[vdiv] : scope_vdivs_short[vdiv];
}

static void draw_scope_scale_status(uint16_t y, uint16_t bg);

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
    int32_t mv = scope_raw_delta_mv(idx, ui.scope_trigger_level);

    scope_format_signed_mv(label, mv);
    return label;
}

static const char *scope_trigger_edge_label(void) {
    return ui.scope_trigger_edge ? "FALL" : "RISE";
}

static void math_menu_step_selected(int8_t dir) {
    uint8_t rows;
    
    if (scope_fft_src == 4u) {
        rows = sizeof(bode_menu_order) / sizeof(bode_menu_order[0]);
    } else {
        rows = sizeof(math_menu_order) / sizeof(math_menu_order[0]);
    }

    if (dir > 0) {
        scope_math_selected = (scope_math_selected + 1) % rows;
    } else {
        if (scope_math_selected == 0) {
            scope_math_selected = rows - 1;
        } else {
            scope_math_selected--;
        }
    }
}


// ---------------------------------------------------------------------------
// Shared helpers for the modern scope overlays and list menus.
// ---------------------------------------------------------------------------

static void scope_pretty_value(const char *in, char *out, uint8_t size);
static void ui_text_append(char *dst, const char *src, uint8_t max_len);
static uint8_t editor_dim_digits(const char *digits, uint8_t count);
static void ui_text_copy(char *dst, const char *src, uint8_t max_len);
static uint16_t scope_tint(uint16_t color, uint16_t under);
static uint16_t scope_pill_width(const char *label, const char *value);
static void scope_draw_pill(int16_t x, uint16_t y, const char *label, const char *value,
                            uint16_t fill, uint16_t label_fg, uint16_t value_fg, uint16_t under);

// A label on the plot: small smooth type, units spaced and cased.
static void scope_plot_label(int16_t x, int16_t y, const char *text, uint16_t color) {
    char pretty[20];
    scope_pretty_value(text, pretty, sizeof(pretty));
    gfx_text(x, (int16_t)(y - 2), &font_small, pretty, color, D_PLOT);
}

static void scope_plot_label_right(int16_t right, int16_t y, const char *text, uint16_t color) {
    char pretty[20];
    scope_pretty_value(text, pretty, sizeof(pretty));
    gfx_text_right(right, (int16_t)(y - 2), &font_small, pretty, color, D_PLOT);
}

static void ui_draw_editor_digits(int16_t x, uint16_t y, const char *digits, uint8_t count,
                                  int8_t selected, uint8_t dim, uint16_t accent, uint16_t fg, uint16_t bg);

// Modern list panel: a rounded card with a title row and value rows. Used
// by the scope's Analytical/Math menu and the generator's Sweep/FM menu.
enum {
    UI_LIST_X = 28,
    UI_LIST_W = 264,
    UI_LIST_ROW_H = 19,
    UI_LIST_HEAD_H = 32,
};

static uint16_t ui_list_begin(const char *title, const char *icon, uint16_t accent, uint16_t tint, uint8_t rows) {
    uint16_t h = (uint16_t)(UI_LIST_HEAD_H + rows * UI_LIST_ROW_H + 8u);
    uint16_t y = (uint16_t)((LCD_HEIGHT - h) / 2u + 4u);
    gfx_round_rect(UI_LIST_X - 4u, (uint16_t)(y - 4u), UI_LIST_W + 8u, (uint16_t)(h + 8u), 18, D_BG, D_BG);
    gfx_round_rect(UI_LIST_X, y, UI_LIST_W, h, 14, accent, D_BG);
    gfx_round_rect(UI_LIST_X + 2u, (uint16_t)(y + 2u), UI_LIST_W - 4u, (uint16_t)(h - 4u), 12, D_CARD, accent);
    gfx_round_rect(UI_LIST_X + 10u, (uint16_t)(y + 8u), 20, 20, 6, tint, D_CARD);
    gfx_text((int16_t)(UI_LIST_X + 14u), (int16_t)(y + 12u), &font_icons, icon, accent, tint);
    gfx_text(UI_LIST_X + 38, (int16_t)(y + 9u), &font_medium, title, D_TEXT, D_CARD);
    return (uint16_t)(y + UI_LIST_HEAD_H);
}

// One row; returns the row's fill so a caller can draw a custom value.
static uint16_t ui_list_row(uint16_t y, const char *label, const char *value,
                            uint8_t selected, uint8_t enabled, uint16_t accent) {
    uint16_t fill = selected ? gfx_blend(accent, D_CARD, 4) : D_CARD;
    uint16_t label_fg = !enabled ? D_TRACK : (selected ? D_TEXT : D_MUTED);
    uint16_t value_fg = !enabled ? D_TRACK : (selected ? accent : D_SUB);
    if (selected) {
        gfx_round_rect(UI_LIST_X + 8u, y, UI_LIST_W - 16u, UI_LIST_ROW_H - 2u, 6, fill, D_CARD);
    }
    gfx_text(UI_LIST_X + 16, (int16_t)(y + 1u), &font_medium, label, label_fg, fill);
    if (value) {
        gfx_text_right(UI_LIST_X + UI_LIST_W - 16, (int16_t)(y + 1u), &font_medium, value, value_fg, fill);
    }
    return fill;
}

// A four-digit number and unit, right-aligned in a row, editable digit by
// digit (edit_index 4 is the unit).
static void ui_list_digits(uint16_t y, uint32_t value, uint8_t editing, uint8_t edit_index,
                           const char *unit, uint8_t selected, uint8_t enabled, uint16_t accent, uint16_t fill) {
    char digits[5];
    uint16_t adv = gfx_text_width(&font_medium, "0");
    uint16_t unit_w = unit ? (uint16_t)(gfx_text_width(&font_medium, unit) + 4u) : 0u;
    int16_t x = (int16_t)(UI_LIST_X + UI_LIST_W - 16 - (int16_t)(adv * 4u + unit_w));
    uint16_t fg = !enabled ? D_TRACK : (selected ? accent : D_SUB);

    digits[0] = (char)('0' + ((value / 1000u) % 10u));
    digits[1] = (char)('0' + ((value / 100u) % 10u));
    digits[2] = (char)('0' + ((value / 10u) % 10u));
    digits[3] = (char)('0' + (value % 10u));
    digits[4] = 0;
    ui_draw_editor_digits(x, (uint16_t)(y + 1u), digits, 4, editing && edit_index < 4u ? (int8_t)edit_index : -1,
                          editor_dim_digits(digits, 4), accent, fg, fill);
    if (unit) {
        int16_t ux = (int16_t)(x + (int16_t)(adv * 4u) + 4);
        if (editing && edit_index == 4u) {
            gfx_round_rect((uint16_t)(ux - 2), (uint16_t)(y + 1u), (uint16_t)(unit_w), 16, 4, accent, fill);
            gfx_text(ux, (int16_t)(y + 1u), &font_medium, unit, D_BG, accent);
        } else {
            gfx_text(ux, (int16_t)(y + 1u), &font_medium, unit, fg, fill);
        }
    }
}

static void ui_draw_math_menu(void) {
    static const char *const math_status_text[] = {"Off", "On"};
    static const char *const math_op_text[] = {"CH1 + CH2", "CH1 - CH2", "CH2 - CH1"};
    static const char *const fft_status_text[] = {"Normal", "FFT CH1", "FFT CH2", "X-Y", "Bode plot"};
    static const char *const fft_window_text[] = {"Hann", "Hamming", "Blackman", "Rectangle"};
    static const char *const fft_display_text[] = {"Normal", "Averaging", "Max hold"};
    static const char *const hide_traces_text[] = {"None", "CH1", "CH2", "All"};
    const uint8_t *menu;
    uint8_t rows;
    uint8_t fft_on = (uint8_t)(scope_fft_src == 1u || scope_fft_src == 2u);
    uint16_t y;

    if (!ui_math_menu_visible) {
        return;
    }
    if (scope_fft_src == 4u) {
        menu = bode_menu_order;
        rows = sizeof(bode_menu_order) / sizeof(bode_menu_order[0]);
    } else {
        menu = math_menu_order;
        rows = sizeof(math_menu_order) / sizeof(math_menu_order[0]);
    }

    y = ui_list_begin(scope_fft_src == 4u ? "Bode plot" : "Analytical / Math", ICON_SCOPE_SM, D_SCOPE,
                      gfx_blend(D_SCOPE, D_CARD, 4), rows);
    for (uint8_t row = 0; row < rows; ++row, y = (uint16_t)(y + UI_LIST_ROW_H)) {
        uint8_t sel = scope_math_selected == row;
        uint8_t editing = (uint8_t)(ui_edit_digit_active && sel);
        uint16_t fill;
        switch (menu[row]) {
        case MENU_ITEM_PLOT:
            ui_list_row(y, "Plot", fft_status_text[scope_fft_src], sel, 1, D_SCOPE);
            break;
        case MENU_ITEM_FFT_WINDOW:
            ui_list_row(y, "FFT window", fft_window_text[scope_fft_window], sel, fft_on, D_SCOPE);
            break;
        case MENU_ITEM_FFT_DISPLAY:
            ui_list_row(y, "FFT display", fft_display_text[scope_fft_display], sel, fft_on, D_SCOPE);
            break;
        case MENU_ITEM_MATH_MODE:
            ui_list_row(y, "Math", math_status_text[scope_math_mode ? 1 : 0], sel, 1, D_SCOPE);
            break;
        case MENU_ITEM_MATH_OP:
            ui_list_row(y, "Operation", math_op_text[scope_math_op], sel, scope_math_mode, D_SCOPE);
            break;
        case MENU_ITEM_HIDE_TRACES:
            ui_list_row(y, "Hide traces", hide_traces_text[scope_hide_traces], sel, 1, D_SCOPE);
            break;
        case MENU_ITEM_BODE_START:
            fill = ui_list_row(y, "Start freq", 0, sel, 1, D_SCOPE);
            ui_list_digits(y, freq_clamp_display(bode_start_hz, bode_start_unit), editing, ui_edit_digit_index,
                           freq_unit_labels[bode_start_unit < 3u ? bode_start_unit : 0u], sel, 1, D_SCOPE, fill);
            break;
        case MENU_ITEM_BODE_STOP:
            fill = ui_list_row(y, "Stop freq", 0, sel, 1, D_SCOPE);
            ui_list_digits(y, freq_clamp_display(bode_stop_hz, bode_stop_unit), editing, ui_edit_digit_index,
                           freq_unit_labels[bode_stop_unit < 3u ? bode_stop_unit : 0u], sel, 1, D_SCOPE, fill);
            break;
        case MENU_ITEM_BODE_STEPS:
            fill = ui_list_row(y, "Steps", 0, sel, 1, D_SCOPE);
            ui_list_digits(y, bode_steps, editing, ui_edit_digit_index, 0, sel, 1, D_SCOPE, fill);
            break;
        case MENU_ITEM_BODE_SWEEP:
            ui_list_row(y, "Sweep", bode_is_sweeping ? "Running" : "Start again", sel, 1, D_SCOPE);
            break;
        }
    }
}

static void ui_draw_fft_spectrum(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh) {
    if (scope_fft_src != 1u && scope_fft_src != 2u) {
        return;
    }
    if (ui.scope_display != SCOPE_DISPLAY_YT || scope_slow_roll_active() || !scope_frame_valid) {
        return;
    }

    uint8_t target_ch = (uint8_t)(scope_fft_src - 1u);
    if (target_ch >= 2u || !scope_ch_enabled[target_ch]) {
        return;
    }

    uint16_t visible_samples = scope_visible_sample_count();
    if (visible_samples < 2u) {
        return;
    }
    if (visible_samples > SCOPE_SAMPLE_COUNT) {
        visible_samples = SCOPE_SAMPLE_COUNT;
    }

    uint16_t source_count = visible_samples < FFT_SIZE ? visible_samples : FFT_SIZE;
    uint16_t base = scope_capture_window_start();
    float sum = 0.0f;

    for (uint16_t i = 0; i < FFT_SIZE; ++i) {
        fft_input_buffer[i] = 0.0f;
    }

    for (uint16_t i = 0; i < source_count; ++i) {
        float sample_pos;
        uint16_t idx_low;
        uint16_t idx_high;
        float weight;
        float value;

        if (visible_samples <= FFT_SIZE) {
            sample_pos = (float)i;
        } else {
            sample_pos = ((float)i * (float)(visible_samples - 1u)) / (float)(FFT_SIZE - 1u);
        }
        idx_low = (uint16_t)sample_pos;
        idx_high = (uint16_t)(idx_low + 1u);
        if (idx_high >= visible_samples) {
            idx_high = (uint16_t)(visible_samples - 1u);
        }
        weight = sample_pos - (float)idx_low;

        uint16_t sample_idx0 = (uint16_t)(base + idx_low);
        uint16_t sample_idx1 = (uint16_t)(base + idx_high);
        float v0 = (float)scope_raw_delta_mv(target_ch,
            scope_samples[(uint16_t)(sample_idx0 * 2u + target_ch)]);
        float v1 = (float)scope_raw_delta_mv(target_ch,
            scope_samples[(uint16_t)(sample_idx1 * 2u + target_ch)]);
        value = v0 + (v1 - v0) * weight;
        fft_input_buffer[i] = value;
        sum += value;
    }

    float mean = sum / (float)source_count;
    for (uint16_t i = 0; i < source_count; ++i) {
        fft_input_buffer[i] -= mean;
    }

    compute_fft(fft_input_buffer, source_count, fft_output_buffer, scope_fft_window);

    uint16_t num_bins = FFT_SIZE / 2u;
    uint16_t peak_bin = 1u;
    float max_amplitude = 0.0f;
    for (uint16_t i = 1u; i < num_bins; ++i) {
        if (scope_fft_display == 1u) {
            fft_history[i] = (0.25f * fft_output_buffer[i]) + (0.75f * fft_history[i]);
            fft_output_buffer[i] = fft_history[i];
        } else if (scope_fft_display == 2u) {
            if (fft_output_buffer[i] > fft_history[i]) {
                fft_history[i] = fft_output_buffer[i];
            }
            fft_output_buffer[i] = fft_history[i];
        } else {
            fft_history[i] = fft_output_buffer[i];
        }
        if (fft_output_buffer[i] > max_amplitude) {
            max_amplitude = fft_output_buffer[i];
            peak_bin = i;
        }
    }
    fft_history[0] = fft_output_buffer[0];
    if (max_amplitude <= 0.0f) {
        return;
    }

    float effective_sample_ns = (float)scope_sample_period_ns();
    if (visible_samples > FFT_SIZE) {
        effective_sample_ns *= (float)(visible_samples - 1u) / (float)(FFT_SIZE - 1u);
    }
    float sample_rate_hz = 1000000000.0f / effective_sample_ns;
    float f_max_hz = sample_rate_hz / 2.0f;
    float hz_per_bin = sample_rate_hz / (float)FFT_SIZE;

    uint16_t bar_color = target_ch ? C_CH2 : C_CH1;
    uint16_t axis_color = D_PLOT_AXIS;
    uint16_t y_bottom = (uint16_t)(gy + gh - 2u);
    uint16_t y_axis_top = (uint16_t)(gy + 2u);
    char freq_label[12];

    lcd_line(gx, y_axis_top, (uint16_t)(gx + gw), y_axis_top, axis_color);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(y_axis_top + 4u), "0 Hz", D_MUTED);

    scope_format_frequency_reading(freq_label, (uint32_t)(f_max_hz + 0.5f));
    scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(y_axis_top + 4u), freq_label, D_MUTED);

    for (uint16_t i = 1u; i < num_bins; ++i) {
        uint16_t x_pos = (uint16_t)(gx + ((uint32_t)i * gw) / num_bins);
        int16_t bar_height = (int16_t)((fft_output_buffer[i] * (float)(gh - 8u)) / max_amplitude);
        if (bar_height > (int16_t)(gh - 6u)) {
            bar_height = (int16_t)(gh - 6u);
        }
        if (bar_height > 1) {
            lcd_line(x_pos, y_bottom, x_pos, (uint16_t)(y_bottom - bar_height), bar_color);
        }
    }

    scope_format_frequency_reading(freq_label, (uint32_t)((float)peak_bin * hz_per_bin + 0.5f));
    {
        char pretty[20];
        uint16_t w;
        scope_pretty_value(freq_label, pretty, sizeof(pretty));
        w = scope_pill_width("Peak", pretty);
        scope_draw_pill((int16_t)(gx + (gw - w) / 2u), (uint16_t)(y_axis_top + 3u), "Peak", pretty,
                        scope_tint(D_AMBER, D_PLOT), D_AMBER, D_TEXT, D_PLOT);
    }
}

static float fast_ln(float x) {
    int8_t exp = 0;

    if (x <= 0.0f) {
        return -20.0f;
    }
    while (x >= 2.0f) {
        x *= 0.5f;
        ++exp;
    }
    while (x < 1.0f) {
        x *= 2.0f;
        --exp;
    }
    float y = (x - 1.0f) / (x + 1.0f);
    float y2 = y * y;
    float term = y;
    float sum = term;
    term *= y2;
    sum += term * 0.33333333f;
    term *= y2;
    sum += term * 0.2f;
    term *= y2;
    sum += term * 0.14285714f;
    term *= y2;
    sum += term * 0.11111111f;
    return 2.0f * sum + (float)exp * 0.69314718f;
}

static float fast_exp(float x) {
    int8_t power = 0;

    while (x > 0.34657359f) {
        x -= 0.69314718f;
        ++power;
    }
    while (x < -0.34657359f) {
        x += 0.69314718f;
        --power;
    }

    float x2 = x * x;
    float result = 1.0f + x + x2 * 0.5f + x2 * x * 0.16666667f +
                   x2 * x2 * 0.04166667f + x2 * x2 * x * 0.00833333f;
    while (power > 0) {
        result *= 2.0f;
        --power;
    }
    while (power < 0) {
        result *= 0.5f;
        ++power;
    }
    return result;
}

static uint32_t log_interpolate_u32(uint32_t start, uint32_t stop, float progress) {
    if (start < 1u) start = 1u;
    if (stop < 1u) stop = 1u;
    if (progress <= 0.0f) return start;
    if (progress >= 1.0f) return stop;

    float ratio = (float)stop / (float)start;
    float value = (float)start * fast_exp(fast_ln(ratio) * progress);
    uint32_t rounded = (uint32_t)(value + 0.5f);
    if (stop >= start) {
        if (rounded < start) rounded = start;
        if (rounded > stop) rounded = stop;
    } else {
        if (rounded > start) rounded = start;
        if (rounded < stop) rounded = stop;
    }
    return rounded;
}

static float bode_gain_db_from_ratio(float mag_out, float mag_in) {
    if (mag_in <= 0.0f || mag_out <= 0.0f) {
        return -120.0f;
    }
    return 8.6858896f * fast_ln(mag_out / mag_in);
}

static float bode_wrap_deg(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static uint32_t bode_step_freq_for(uint8_t step, uint8_t total_steps) {
    uint32_t start = bode_start_hz;
    uint32_t stop = bode_stop_hz;
    if (total_steps <= 1u) {
        return start ? start : 1u;
    }
    if (start == 0u) {
        start = 1u;
    }
    if (stop == 0u) {
        stop = 1u;
    }
    if (step >= total_steps) {
        step = (uint8_t)(total_steps - 1u);
    }

    return log_interpolate_u32(start, stop,
                               (float)step / (float)(total_steps - 1u));
}

static uint8_t bode_auto_timebase(uint32_t freq_hz) {
    uint64_t min_screen_ns;

    if (freq_hz < 1u) {
        freq_hz = 1u;
    }
    min_screen_ns = 3000000000ull / freq_hz;
    // Roll timebases never refresh scope_samples, so stay on captured frames.
    for (uint8_t i = 0; i < SCOPE_SLOW_ROLL_TIMEBASE_START; ++i) {
        uint64_t screen_ns = (uint64_t)scope_timebase_unit_ns[i] * SCOPE_X_DIVS * 10u;
        if (screen_ns >= min_screen_ns) {
            return i;
        }
    }
    return (uint8_t)(SCOPE_SLOW_ROLL_TIMEBASE_START - 1u);
}

static void bode_restore_generator(void) {
    if (!bode_siggen_active) {
        return;
    }
    bode_siggen_active = 0;
    if (gen_running) {
        gen_apply();
    } else {
        siggen_shutdown();
        gen_output_applied = 0;
    }
}

static void bode_shutdown(uint8_t restore_scope_hw) {
    bode_restore_generator();
    bode_is_sweeping = 0;
    if (!bode_saved.valid) {
        return;
    }

    ui.scope_timebase = bode_saved.scope_timebase;
    ui.scope_trigger_source = bode_saved.trigger_source;
    ui.scope_trigger_mode = bode_saved.trigger_mode;
    scope_ch_enabled[0] = bode_saved.ch_enabled[0];
    scope_ch_enabled[1] = bode_saved.ch_enabled[1];
    scope_hide_traces = bode_saved.hide_traces;
    bode_saved.valid = 0;

    if (restore_scope_hw && ui.mode == UI_MODE_SCOPE) {
        scope_apply_settings_keep_frame(0);
    }
}

static void bode_apply_step_freq(void) {
    uint32_t freq = bode_step_freq_for(bode_current_step, bode_steps);

    bode_step_freq_hz = freq;
    bode_dwell_frames = 0;
    ui.scope_timebase = bode_auto_timebase(freq);
    ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    ui.scope_trigger_source = 1;
    scope_ch_enabled[0] = 1;
    scope_ch_enabled[1] = 1;
    scope_apply_settings_keep_frame(0);
    siggen_configure(1u,
                     SIGGEN_WAVE_SINE,
                     freq,
                     50u,
                     ui_settings.siggen_amp_tenths_v);
    bode_siggen_active = 1;
}

static void bode_begin_sweep(void) {
    uint16_t i;

    bode_steps = bode_clamp_steps(bode_steps);
    if (!bode_saved.valid) {
        bode_saved.valid = 1;
        bode_saved.scope_timebase = ui.scope_timebase;
        bode_saved.trigger_source = ui.scope_trigger_source;
        bode_saved.trigger_mode = ui.scope_trigger_mode;
        bode_saved.ch_enabled[0] = scope_ch_enabled[0];
        bode_saved.ch_enabled[1] = scope_ch_enabled[1];
        bode_saved.hide_traces = scope_hide_traces;
    }
    bode_restore_generator();
    bode_current_step = 0;
    bode_is_sweeping = 1;
    bode_dwell_frames = 0;
    scope_hide_traces = 3;
    for (i = 0; i < BODE_MAX_STEPS; ++i) {
        bode_gain_db[i] = 0.0f;
        bode_phase_deg[i] = 0.0f;
    }
    bode_apply_step_freq();
}

static void scope_fft_src_apply_from(uint8_t old_src, uint8_t new_src) {
    if (new_src == old_src) {
        return;
    }
    if (old_src == 4u) {
        bode_shutdown(1);
    }
    scope_fft_src = new_src;
    if (new_src == 4u) {
        if (scope_math_selected == 0u) {
            scope_math_selected = 1u;
        }
        bode_begin_sweep();
        bode_cursor_sub = 0;
        ui.scope_display = SCOPE_DISPLAY_YT;
    } else if (new_src == 3u) {
        ui.scope_display = SCOPE_DISPLAY_XY;
    } else {
        ui.scope_display = SCOPE_DISPLAY_YT;
    }
    scope_trace_invalidate();
}

static uint16_t bode_extract_channel(uint8_t target_ch, float *dst) {
    uint16_t visible_samples = scope_visible_sample_count();
    uint16_t source_count;
    uint16_t base;
    float sum = 0.0f;
    uint16_t i;

    if (visible_samples < 2u) {
        for (i = 0; i < FFT_SIZE; ++i) {
            dst[i] = 0.0f;
        }
        return 0;
    }
    if (visible_samples > SCOPE_SAMPLE_COUNT) {
        visible_samples = SCOPE_SAMPLE_COUNT;
    }

    source_count = visible_samples < FFT_SIZE ? visible_samples : FFT_SIZE;
    base = scope_capture_window_start();

    for (i = 0; i < FFT_SIZE; ++i) {
        dst[i] = 0.0f;
    }

    for (i = 0; i < source_count; ++i) {
        float sample_pos;
        uint16_t idx_low;
        uint16_t idx_high;
        float weight;
        float value;

        if (visible_samples <= FFT_SIZE) {
            sample_pos = (float)i;
        } else {
            sample_pos = ((float)i * (float)(visible_samples - 1u)) / (float)(FFT_SIZE - 1u);
        }
        idx_low = (uint16_t)sample_pos;
        idx_high = (uint16_t)(idx_low + 1u);
        if (idx_high >= visible_samples) {
            idx_high = (uint16_t)(visible_samples - 1u);
        }
        weight = sample_pos - (float)idx_low;

        {
            uint16_t sample_idx0 = (uint16_t)(base + idx_low);
            uint16_t sample_idx1 = (uint16_t)(base + idx_high);
            float v0 = (float)scope_raw_delta_mv(target_ch,
                scope_samples[(uint16_t)(sample_idx0 * 2u + target_ch)]);
            float v1 = (float)scope_raw_delta_mv(target_ch,
                scope_samples[(uint16_t)(sample_idx1 * 2u + target_ch)]);
            value = v0 + (v1 - v0) * weight;
        }
        dst[i] = value;
        sum += value;
    }

    {
        float mean = sum / (float)source_count;
        for (i = 0; i < source_count; ++i) {
            dst[i] -= mean;
        }
    }
    return source_count;
}

static uint8_t bode_measure_step(uint8_t step) {
    float mag_ch1;
    float mag_ch2;
    float phase_ch1;
    float phase_ch2;
    float effective_sample_ns;
    float sample_rate_hz;
    float hz_per_bin;
    uint16_t bin;
    uint16_t source_count;
    uint16_t visible_samples = scope_visible_sample_count();

    if (scope_slow_roll_active() || !scope_ch_enabled[0] || !scope_ch_enabled[1]) {
        return 0;
    }

    source_count = bode_extract_channel(0, fft_input_buffer);
    if (!source_count) {
        return 0;
    }
    effective_sample_ns = (float)scope_sample_period_ns();
    if (visible_samples > FFT_SIZE) {
        effective_sample_ns *= (float)(visible_samples - 1u) / (float)(FFT_SIZE - 1u);
    }
    sample_rate_hz = 1000000000.0f / effective_sample_ns;
    hz_per_bin = sample_rate_hz / (float)FFT_SIZE;
    bin = (uint16_t)((float)bode_step_freq_hz / hz_per_bin + 0.5f);
    if (bin < 1u) {
        bin = 1u;
    }
    if (bin >= (uint16_t)(FFT_SIZE / 2u)) {
        bin = (uint16_t)((FFT_SIZE / 2u) - 1u);
    }

    compute_fft_bin(fft_input_buffer, source_count, scope_fft_window, bin, &mag_ch1, &phase_ch1);
    bode_extract_channel(1, fft_input_buffer);
    compute_fft_bin(fft_input_buffer, source_count, scope_fft_window, bin, &mag_ch2, &phase_ch2);

    if (mag_ch1 <= 0.0f || mag_ch2 <= 0.0f) {
        return 0;
    }

    if (step < BODE_MAX_STEPS) {
        bode_gain_db[step] = bode_gain_db_from_ratio(mag_ch2, mag_ch1);
        bode_phase_deg[step] = bode_wrap_deg((phase_ch2 - phase_ch1) * 57.2957795f);
    }
    return 1;
}

static void bode_sweep_service(void) {
    if (scope_fft_src != 4u || !bode_is_sweeping) {
        return;
    }
    if (bode_current_step >= bode_steps) {
        bode_is_sweeping = 0;
        bode_restore_generator();
        return;
    }
    if (!scope_frame_valid) {
        return;
    }
    if (bode_dwell_frames < BODE_DWELL_FRAMES) {
        ++bode_dwell_frames;
        return;
    }
    if (!bode_measure_step(bode_current_step)) {
        return;
    }

    ++bode_current_step;
    if (bode_current_step >= bode_steps) {
        bode_is_sweeping = 0;
        bode_restore_generator();
        ui_render_scope_frame();
        return;
    }

    bode_apply_step_freq();
    ui_render_scope_frame();
}

static int16_t bode_map_y(uint16_t y_top, uint16_t panel_h, float value, float min_v, float max_v) {
    float span = max_v - min_v;
    float norm;

    if (span <= 0.0f) {
        return (int16_t)(y_top + panel_h / 2u);
    }
    if (value < min_v) {
        value = min_v;
    } else if (value > max_v) {
        value = max_v;
    }
    norm = (value - min_v) / span;
    return (int16_t)(y_top + panel_h - 1u - (uint16_t)(norm * (float)(panel_h - 1u)));
}

static void ui_draw_bode(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh) {
    uint16_t mag_h;
    uint16_t phase_y;
    uint16_t phase_h;
    uint16_t axis_color = D_PLOT_AXIS;
    uint16_t mag_color = D_ACCENT;
    uint16_t phase_color = D_AMBER;
    uint8_t completed;
    uint8_t steps = bode_clamp_steps(bode_steps);
    char label[16];
    uint16_t x_start;
    uint16_t x_end;
    uint8_t i;

    if (scope_fft_src != 4u || ui.scope_display != SCOPE_DISPLAY_YT) {
        return;
    }

    mag_h = (uint16_t)((gh - 10u) / 2u);
    phase_y = (uint16_t)(gy + mag_h + 6u);
    phase_h = (uint16_t)(gh - mag_h - 6u);
    completed = bode_is_sweeping ? (bode_current_step + 1u) : steps;
    if (completed > steps) {
        completed = steps;
    }

    x_start = gx;
    x_end = (uint16_t)(gx + gw);


    uint16_t mag_0db_y = (uint16_t)(gy + (mag_h / 4u));
    lcd_line(x_start, mag_0db_y, x_end, mag_0db_y, axis_color);
    lcd_line(x_start, (uint16_t)(gy + mag_h), x_end, (uint16_t)(gy + mag_h), axis_color);
    uint16_t phase_center_y = (uint16_t)(phase_y + (phase_h / 2u));
    lcd_line(x_start, phase_center_y, x_end, phase_center_y, axis_color);
    lcd_line(x_start, (uint16_t)(phase_y + phase_h), x_end, (uint16_t)(phase_y + phase_h), axis_color);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(gy + 4u), "GAIN", mag_color);
    scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(gy + 4u), "+20DB", D_MUTED);
    scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(mag_0db_y + 4u), "0DB", D_MUTED);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(gy + mag_h - 12u), "-60 dB", D_MUTED);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(phase_y + 3u), "PHASE", phase_color);
    scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(phase_y + 3u), "+180", D_MUTED);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(phase_center_y - 12u), "0", D_MUTED);
    scope_plot_label((int16_t)(gx + 6u), (int16_t)(phase_y + phase_h - 12u), "-180", D_MUTED);

    scope_format_frequency_reading(label, bode_start_hz);
    scope_plot_label((int16_t)(gx + 52u), (int16_t)(gy + mag_h - 12u), label, D_SUB);
    scope_format_frequency_reading(label, bode_stop_hz);
    scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(gy + mag_h - 12u), label, D_SUB);

    for (i = 1; i < completed; ++i) {
        uint16_t x0 = (uint16_t)(gx + ((uint32_t)(i - 1u) * gw) / (uint32_t)(steps - 1u));
        uint16_t x1 = (uint16_t)(gx + ((uint32_t)i * gw) / (uint32_t)(steps - 1u));
        int16_t y0 = bode_map_y(gy, mag_h, bode_gain_db[i - 1u], -60.0f, 20.0f);
        int16_t y1 = bode_map_y(gy, mag_h, bode_gain_db[i], -60.0f, 20.0f);
        int16_t p0 = bode_map_y(phase_y, phase_h, bode_phase_deg[i - 1u], -180.0f, 180.0f);
        int16_t p1 = bode_map_y(phase_y, phase_h, bode_phase_deg[i], -180.0f, 180.0f);
        lcd_line(x0, (uint16_t)y0, x1, (uint16_t)y1, mag_color);
        lcd_line(x0, (uint16_t)p0, x1, (uint16_t)p1, phase_color);
    }

    if (bode_is_sweeping) {
        char progress[20];
        char total[8];
        ui_text_copy(progress, "Sweeping ", sizeof(progress));
        scope_format_u32(total, (uint32_t)(bode_current_step + 1u));
        ui_text_append(progress, total, sizeof(progress));
        ui_text_append(progress, " / ", sizeof(progress));
        scope_format_u32(total, (uint32_t)steps);
        ui_text_append(progress, total, sizeof(progress));
        gfx_text((int16_t)(gx + 6u), (int16_t)(gy + gh - 14u), &font_small, progress, D_AMBER, D_PLOT);
        scope_format_frequency_reading(label, bode_step_freq_hz);
        scope_plot_label_right((int16_t)(gx + gw - 6u), (int16_t)(gy + gh - 12u), label, D_AMBER);
    } else if (completed > 0u) {
        gfx_text((int16_t)(gx + 6u), (int16_t)(gy + gh - 14u), &font_small, "Sweep done", D_GREEN, D_PLOT);
    }
}

static void draw_softkey(uint8_t slot, const char *label, uint16_t accent) {
    uint16_t x = (uint16_t)(slot * 80u);
    lcd_rect(x, Y_SOFT, 79, H_SOFT, C_TOP);
    lcd_rect(x, Y_SOFT, 79, 2, accent);
    lcd_text_center(x, (uint16_t)(Y_SOFT + 16), 79, label, C_TEXT, C_TOP, 1);
}

static void draw_dmm_softkeys(void);

static const char *scope_measure_pretty_label(uint8_t measure);

static const char *scope_measure_chip_label(void) {
    scope_sanitize_state();
    if (!scope_measure_visible) {
        return "OFF";
    }
    return scope_measure_pretty_label(scope_measure_param);
}

static const char *scope_param_value(uint8_t param) {
    if (param == SCOPE_PARAM_ENABLE) {
        return scope_channel_enable_label();
    }
    if (param == SCOPE_PARAM_PROBE) {
        return scope_probe_label();
    }
    if (param == SCOPE_PARAM_GAIN) {
        return scope_vdiv_menu_label();
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

// ---------------------------------------------------------------------------
// Parameter chips (scope and generator rows) in the modern style.
// ---------------------------------------------------------------------------

static void scope_pretty_value(const char *in, char *out, uint8_t size);
static void ui_text_copy(char *dst, const char *src, uint8_t max_len);
static uint8_t ui_text_equal(const char *a, const char *b);

// Values as the chips show them: units spaced and cased ("20 mV", "1 µs"),
// words in sentence case ("Auto", "Normal"), short codes kept (CH1, DC, X10).
static void ui_chip_value(const char *in, char out[16]) {
    uint8_t digits = 0;
    uint8_t len = 0;
    for (const char *p = in; *p; ++p, ++len) {
        if (*p >= '0' && *p <= '9') {
            digits = 1;
        }
    }
    if (digits) {
        scope_pretty_value(in, out, 16);
        return;
    }
    ui_text_copy(out, in, 16);
    if (len >= 4u || ui_text_equal(in, "ON") || ui_text_equal(in, "OFF")) {
        for (uint8_t i = 1; out[i]; ++i) {
            if (out[i] >= 'A' && out[i] <= 'Z' && out[i - 1u] != ' ') {
                out[i] = (char)(out[i] + 32);
            }
        }
    }
}

// Card behind a chip; returns its fill so the text can blend into it.
static uint16_t ui_chip_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t accent, uint8_t active) {
    uint16_t bg = active ? gfx_blend(accent, D_BG, 3) : D_CARD;
    if (active) {
        gfx_round_rect(x, y, w, 32, 10, accent, D_BG);
        gfx_round_rect((uint16_t)(x + 2u), (uint16_t)(y + 2u), (uint16_t)(w - 4u), 28, 8, bg, accent);
    } else {
        gfx_round_rect(x, y, w, 32, 10, bg, D_BG);
    }
    return bg;
}

static void ui_draw_chip(uint16_t x, uint16_t y, uint16_t w, const char *label, const char *value,
                         uint16_t accent, uint8_t active, uint8_t enabled) {
    char shown[16];
    uint16_t bg = ui_chip_frame(x, y, w, accent, active);
    uint16_t label_fg = !enabled ? D_TRACK : (active ? accent : D_MUTED);
    uint16_t value_fg = !enabled ? D_TRACK : (active ? D_TEXT : D_SUB);
    const lcd_font_t *font = &font_medium;

    ui_chip_value(enabled ? value : "--", shown);
    gfx_text((int16_t)(x + 8u), (int16_t)(y + 3u), &font_small, label, label_fg, bg);
    if (gfx_text_width(font, shown) > (uint16_t)(w - 10u)) {
        font = &font_small;
    }
    gfx_text_center((int16_t)x, (int16_t)(y + (font == &font_medium ? 14u : 17u)), w, font, shown, value_fg, bg);
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
    ui_draw_chip(x, y, w, scope_param_label(param), scope_param_value(param),
                 scope_menu_accent(), ui.scope_param == param, 1);
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

static const char *scope_measure_pretty_label(uint8_t measure) {
    static const char *const labels[] = {"Vpp", "Vmax", "Vmin", "Vavg", "Vrms", "Freq"};
    return labels[measure < 6u ? measure : 0u];
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
    (void)selected;
    ui_draw_chip(x, y, w, label, value, accent == C_GRID ? D_MUTED : accent, active, 1);
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
                            scope_measure_pretty_label(scope_measure_param),
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
    uint32_t screen_ns;
    uint32_t sample_ns;
    uint32_t count;

    if (timebase >= SCOPE_SLOW_ROLL_TIMEBASE_START) {
        return SCOPE_VISIBLE_SAMPLE_COUNT;
    }
    // Capture timebases stay within 2.4 s per screen, so this fits in 32 bits.
    screen_ns = scope_timebase_unit_ns[timebase] * SCOPE_X_DIVS * 10u;
    sample_ns = scope_sample_period_ns();
    count = (screen_ns + sample_ns / 2u) / sample_ns;
    if (count < 2u) {
        count = 2u;
    }
    if (count > SCOPE_VISIBLE_SAMPLE_COUNT) {
        count = SCOPE_VISIBLE_SAMPLE_COUNT;
    }
    return (uint16_t)count;
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

static uint16_t scope_capture_window_start(void) {
    uint16_t max_start = (uint16_t)(SCOPE_SAMPLE_COUNT - scope_visible_sample_count());
    int32_t start = scope_trigger_locked ? (int32_t)scope_trigger_offset :
        (int32_t)(max_start / 2u) + scope_h_pos_sample_offset();

    // Capture boundaries are not adjacent in time; never wrap a display window.
    if (start < 0) {
        return 0;
    }
    if (start > max_start) {
        return max_start;
    }
    return (uint16_t)start;
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
            out = scope_append_text(label, out, sizeof(label), "ns");
        } else {
            uint32_t us_x10 = (ns + 50u) / 100u;
            scope_format_u32(number, us_x10 / 10u);
            out = scope_append_text(label, out, sizeof(label), number);
            out = scope_append_text(label, out, sizeof(label), ".");
            if (out < (uint8_t)(sizeof(label) - 1u)) {
                label[out++] = (char)('0' + us_x10 % 10u);
                label[out] = 0;
            }
            out = scope_append_text(label, out, sizeof(label), "us");
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
        out = scope_append_text(label, out, sizeof(label), "us");
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
        out = scope_append_text(label, out, sizeof(label), "ms");
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
        out = scope_append_text(label, out, sizeof(label), "s");
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
    if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        if (bode_cursor_sub == 0u) return idx ? "F2" : "F1";
        if (bode_cursor_sub == 1u) return idx ? "DB2" : "DB1";
        return idx ? "P2" : "P1";
    }
    if (scope_fft_src == 4u) {
        if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
            return idx ? "DB2" : "DB1";
        }
        if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
            return idx ? "P2" : "P1";
        }
    }
    if ((scope_fft_src == 1u || scope_fft_src == 2u) && ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        return idx ? "F2" : "F1";
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        return idx ? "T2" : "T1";
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        return idx ? "Y2" : "Y1";
    }
    return idx ? "C2" : "C1";
}

static void scope_format_signed_float(char out[12], float value, const char *suffix) {
    uint8_t pos = 0;
    uint32_t scaled;
    char number[11];

    if (value < 0) {
        out[pos++] = '-';
        value = -value;
    } else {
        out[pos++] = '+';
    }
    scaled = (uint32_t)(value * 10.0f + 0.5f);
    scope_format_u32(number, scaled / 10u);
    pos = scope_append_text(out, pos, 12, number);
    if (pos < 11u) {
        out[pos++] = '.';
    }
    if (pos < 11u) {
        out[pos++] = (char)('0' + scaled % 10u);
    }
    while (*suffix && pos < 11u) {
        out[pos++] = *suffix++;
    }
    out[pos] = 0;
}

static const char *scope_cursor_pos_label(uint8_t idx) {
    static char label[2][12];
    uint8_t steps = bode_clamp_steps(bode_steps);

    idx = idx ? 1u : 0u;
    if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
        label[idx][0] = '-';
        label[idx][1] = '-';
        label[idx][2] = 0;
    } else if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        if (bode_cursor_sub == 0u) {
            uint8_t step = (uint8_t)((uint32_t)scope_cursor_x[idx] * (uint32_t)(steps - 1u) / 255u);
            scope_format_frequency_reading(label[idx], bode_step_freq_for(step, steps));
        } else {
            float val;
            if (bode_cursor_sub == 1u) {
                val = 20.0f - (float)scope_cursor_y[idx] * 80.0f / 255.0f;
                scope_format_signed_float(label[idx], val, "DB");
            } else {
                val = 180.0f - (float)scope_cursor_y[idx] * 360.0f / 255.0f;
                scope_format_signed_float(label[idx], val, "~");
            }
        }
    } else if ((scope_fft_src == 1u || scope_fft_src == 2u) && ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        float effective_sample_ns = (float)scope_sample_period_ns();
        uint16_t vis = scope_visible_sample_count();
        if (vis > SCOPE_SAMPLE_COUNT) vis = SCOPE_SAMPLE_COUNT;
        if (vis > FFT_SIZE) {
            effective_sample_ns *= (float)(vis - 1u) / (float)(FFT_SIZE - 1u);
        }
        float sample_rate_hz = 1000000000.0f / effective_sample_ns;
        uint32_t freq = (uint32_t)(((float)scope_cursor_x[idx] / 255.0f) * (sample_rate_hz / 2.0f) + 0.5f);
        scope_format_frequency_reading(label[idx], freq);
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
    uint8_t steps = bode_clamp_steps(bode_steps);

    if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
        label[0] = '-';
        label[1] = '-';
        label[2] = 0;
    } else if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        if (bode_cursor_sub == 0u) {
            uint8_t s0 = (uint8_t)((uint32_t)scope_cursor_x[0] * (uint32_t)(steps - 1u) / 255u);
            uint8_t s1 = (uint8_t)((uint32_t)scope_cursor_x[1] * (uint32_t)(steps - 1u) / 255u);
            uint32_t f0 = bode_step_freq_for(s0, steps);
            uint32_t f1 = bode_step_freq_for(s1, steps);
            uint32_t df = f0 > f1 ? f0 - f1 : f1 - f0;
            label[0] = 'D'; label[1] = 'F'; label[2] = ' ';
            scope_format_frequency_reading(&label[3], df);
        } else if (bode_cursor_sub == 1u) {
            float d0 = 20.0f - (float)scope_cursor_y[0] * 80.0f / 255.0f;
            float d1 = 20.0f - (float)scope_cursor_y[1] * 80.0f / 255.0f;
            float ddb = d0 > d1 ? d0 - d1 : d1 - d0;
            label[0] = 'D'; label[1] = 'D'; label[2] = ' ';
            scope_format_signed_float(&label[3], ddb, "DB");
        } else {
            float p0 = 180.0f - (float)scope_cursor_y[0] * 360.0f / 255.0f;
            float p1 = 180.0f - (float)scope_cursor_y[1] * 360.0f / 255.0f;
            float ddeg = p0 > p1 ? p0 - p1 : p1 - p0;
            label[0] = 'D'; label[1] = 'P'; label[2] = ' ';
            scope_format_signed_float(&label[3], ddeg, "~");
        }
    } else if ((scope_fft_src == 1u || scope_fft_src == 2u) && ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        float eff_ns = (float)scope_sample_period_ns();
        uint16_t vis = scope_visible_sample_count();
        if (vis > SCOPE_SAMPLE_COUNT) vis = SCOPE_SAMPLE_COUNT;
        if (vis > FFT_SIZE) eff_ns *= (float)(vis - 1u) / (float)(FFT_SIZE - 1u);
        float sr_hz = 1000000000.0f / eff_ns;
        float f_max = sr_hz / 2.0f;
        uint32_t f0 = (uint32_t)((float)scope_cursor_x[0] / 255.0f * f_max + 0.5f);
        uint32_t f1 = (uint32_t)((float)scope_cursor_x[1] / 255.0f * f_max + 0.5f);
        uint32_t df = f0 > f1 ? f0 - f1 : f1 - f0;
        label[0] = 'D'; label[1] = 'F'; label[2] = ' ';
        scope_format_frequency_reading(&label[3], df);
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
    ui_draw_chip(x, y, w, label, value, scope_channel_color(ui.active_ch), active, 1);
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
        lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, D_BG);
        draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
        return;
    }
    if (ui.mode == UI_MODE_SCOPE) {
        lcd_rect(0, (uint16_t)(Y_SOFT - 4u), LCD_WIDTH, (uint16_t)(H_SOFT + 4u), D_BG);
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

static void format_signed_fixed(int32_t value, char out[12]) {
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

    whole = abs_value / DMM_VALUE_SCALE;
    frac = (uint16_t)(abs_value % DMM_VALUE_SCALE);
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
    out[pos++] = (char)('0' + frac / 1000u);
    out[pos++] = (char)('0' + (frac / 100u) % 10u);
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

static int32_t dmm_display_fixed_value(void) {
    int32_t value = dmm_value_fixed_units();
    // The four-digit input leaves ample room for the factors in 32 bits.
    if (ui.dmm_mode == DMM_MODE_TEMP_F && !dmm_unit_is_fahrenheit()) {
        return value * 9 / 5 + 32 * DMM_VALUE_SCALE;
    }
    if (ui.dmm_mode == DMM_MODE_TEMP && dmm_unit_is_fahrenheit()) {
        return (value - 32 * DMM_VALUE_SCALE) * 5 / 9;
    }
    return value;
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
    if (dmm_is_temperature(ui.dmm_mode) && dmm_value_is_numeric() &&
        (ui.dmm_mode == DMM_MODE_TEMP_F) != dmm_unit_is_fahrenheit()) {
        static char converted[12];
        uint8_t decimals = dmm_text_decimals(dmm_value_text());
        if (decimals > 4u) decimals = 4u;
        int32_t step = dmm_pow10((uint8_t)(4u - decimals));
        int32_t value = dmm_display_fixed_value();
        int32_t rounded = (value + (value < 0 ? -step / 2 : step / 2)) / step;
        dmm_format_fixed(rounded, decimals, decimals, 0, converted);
        return converted;
    }
    if (dmm_low_current_mode()) {
        if (!dmm_value_is_numeric()) {
            return "";
        }
        format_low_current_value(dmm_value_text(), dmm_low_current_value);
        return dmm_low_current_value;
    }
    return dmm_value_text();
}

// Units are spelled in upper case internally and over USB ("MV", "MOHM",
// "UF"), but case is exactly what tells milli from mega: "MOHM" read the same
// whether it meant milliohm or megohm, and "MV" looked like megavolt. Spell
// them properly for the screen; the USB protocol keeps its own spelling so
// existing PC software is unaffected.
static const char *dmm_unit_si_text(const char *raw) {
    if (!raw) {
        return "";
    }
    if (ui_text_equal(raw, "MV")) return "mV";
    if (ui_text_equal(raw, "MA")) return "mA";
    if (ui_text_equal(raw, "UA")) return "uA";
    if (ui_text_equal(raw, "OHM")) return "Ohm";
    if (ui_text_equal(raw, "KOHM")) return "kOhm";
    if (ui_text_equal(raw, "MOHM")) return "MOhm";
    if (ui_text_equal(raw, "NF")) return "nF";
    if (ui_text_equal(raw, "UF")) return "uF";
    if (ui_text_equal(raw, "MF")) return "mF";
    // Temperature. The font carries the degree sign on '~' (see font.c), so
    // the screen can show 23.8 degC instead of the bare word DEG -- and which
    // of the two scales it is, which "DEG" on its own cannot say.
    if (ui_text_equal(raw, "DEG")) return dmm_unit_is_fahrenheit() ? "~F" : "~C";
    return raw;   // V, A and F are unambiguous as they stand
}

static const char *dmm_live_display_unit(void) {
    if (ui.dmm_mode == DMM_MODE_AUTO) {
        return dmm_has_reading() && !ui_text_equal(dmm_value_text(), "AUTO") ? dmm_unit_si_text(dmm_unit_text()) : "";
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
    if (dmm_is_temperature(ui.dmm_mode)) {
        return ui.dmm_mode == DMM_MODE_TEMP_F ? "~F" : "~C";
    }
    return dmm_unit_si_text(dmm_unit_text());
}

static const char *dmm_rel_display_value(void) {
    const char *unit = dmm_live_display_unit();

    if (!dmm_has_reading() || !dmm_value_is_numeric()) {
        return "-----";
    }
    if (!ui_text_equal(unit, dmm_rel_unit)) {
        return "RANGE";
    }

    format_signed_fixed(dmm_display_fixed_value() - dmm_rel_ref_fixed, dmm_rel_value);
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

// ---------------------------------------------------------------------------
// Multimeter screen: flat dark theme, rounded cards, anti-aliased type.
// Layout (320x240): status bar 0-22, reading card 26-162, stats 168-204,
// group pills 210-236. Continuity and live-wire states use one tall card
// (26-204) instead of the reading card plus stats.
// ---------------------------------------------------------------------------

enum {
    DMM_X = 8,
    DMM_W = 304,
    DMM_IN_X = 20,
    DMM_IN_W = 280,
    DMM_STATUS_H = 22,
    DMM_CARD_Y = 26,
    DMM_CARD_H = 136,
    DMM_TALL_H = 178,
    DMM_STATS_Y = 168,
    DMM_STATS_H = 36,
    DMM_KEYS_Y = 210,
    DMM_KEYS_H = 26,
};

static const char *const dmm_mode_names[] = {
    "Auto Detect", "DC Voltage", "AC Voltage", "Resistance",
    "Diode / Continuity", "Capacitance", "Live Wire", "Continuity",
    "Temperature", "AC Current (High)", "DC Current (High)",
    "AC Current (Low)", "DC Current (Low)", "Temperature",
};

// The SI spellings from dmm_unit_si_text(), with the real symbols the
// anti-aliased fonts carry.
static const char *dmm_unit_pretty(const char *unit) {
    if (ui_text_equal(unit, "Ohm")) return "\xCE\xA9";
    if (ui_text_equal(unit, "kOhm")) return "k\xCE\xA9";
    if (ui_text_equal(unit, "MOhm")) return "M\xCE\xA9";
    if (ui_text_equal(unit, "uA")) return "\xC2\xB5" "A";
    if (ui_text_equal(unit, "uF")) return "\xC2\xB5" "F";
    if (ui_text_equal(unit, "~C")) return "\xC2\xB0" "C";
    if (ui_text_equal(unit, "~F")) return "\xC2\xB0" "F";
    return unit;
}

static uint8_t dmm_text_is_number(const char *text) {
    uint8_t digits = 0;
    for (; *text; ++text) {
        if (*text >= '0' && *text <= '9') {
            ++digits;
        } else if (*text != '.' && *text != '-' && *text != '+') {
            return 0;
        }
    }
    return digits != 0;
}

static uint8_t dmm_text_decimals(const char *text) {
    while (*text && *text != '.') {
        ++text;
    }
    if (!*text) {
        return 0;
    }
    uint8_t n = 0;
    while (*++text) {
        ++n;
    }
    return n;
}

static int32_t dmm_pow10(uint8_t n) {
    int32_t v = 1;
    while (n--) {
        v *= 10;
    }
    return v;
}

// Fixed-point (`stored` decimals) to text with `shown` decimals; `trim` drops
// trailing zeros for axis labels.
static void dmm_format_fixed(int32_t value, uint8_t stored, uint8_t shown, uint8_t trim, char out[12]) {
    uint8_t pos = 0;
    char digits[12];
    uint8_t n = 0;
    uint32_t v;

    if (shown > stored) {
        shown = stored;
    }
    if (value < 0) {
        out[pos++] = '-';
        v = (uint32_t)(-value);
    } else {
        v = (uint32_t)value;
    }
    v /= (uint32_t)dmm_pow10((uint8_t)(stored - shown));
    do {
        digits[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v || n <= shown);
    while (n > 0 && pos < 10u) {
        if (n == shown && shown) {
            out[pos++] = '.';
        }
        out[pos++] = digits[--n];
    }
    out[pos] = 0;
    if (trim && shown) {
        while (pos > 0 && out[pos - 1u] == '0') {
            out[--pos] = 0;
        }
        if (pos > 0 && out[pos - 1u] == '.') {
            out[--pos] = 0;
        }
    }
}

// The meter IC shows four digits, so the position of its decimal point is
// its current range: "12.35" V is on the 0-100 V range, "1.204" kOhm on the
// 0-10 kOhm range. The bar graph shows the reading against that range, like
// the bar graph of a bench meter, and only rescales when the meter does.
enum { DMM_DISPLAY_DIGITS = 4 };

static int8_t dmm_range_decimals(void) {
    if (dmm_hold_active) {
        return (int8_t)dmm_hold_range_decimals;
    }
    if (!dmm_has_reading() || !dmm_value_is_numeric()) {
        return -1;
    }
    return (int8_t)dmm_text_decimals(dmm_value_text());
}

// Full scale of the current range in fixed-point units with `decimals`
// places; 0 when there is no numeric reading to take a range from.
static int32_t dmm_range_full_scale(uint8_t decimals) {
    int8_t n = dmm_range_decimals();
    if (n < 0) {
        return 0;
    }
    if (n > DMM_DISPLAY_DIGITS) {
        n = DMM_DISPLAY_DIGITS;
    }
    return dmm_pow10((uint8_t)(DMM_DISPLAY_DIGITS - n + decimals));
}

// Draws an icon from font_icons with the top of its box at y.
static int16_t dmm_icon(int16_t x, int16_t y, const char *icon, uint16_t fg, uint16_t bg) {
    return gfx_text(x, y, &font_icons, icon, fg, bg);
}

static void dmm_draw_pill_icon(int16_t right, uint16_t y, const char *icon, const char *label,
                               uint16_t fill, uint16_t fg, uint16_t under, int16_t *left_out) {
    uint16_t icon_w = icon ? (uint16_t)(gfx_text_width(&font_icons, icon) + 3u) : 0u;
    uint16_t w = (uint16_t)(gfx_text_width(&font_small, label) + icon_w + 12u);
    int16_t x = (int16_t)(right - (int16_t)w);
    gfx_round_rect((uint16_t)x, y, w, 15, 7, fill, under);
    if (icon) {
        // Small icons are 9-12 px square; centre them in the 15 px pill.
        int16_t size = (int16_t)(icon_w - 3u);
        dmm_icon((int16_t)(x + 6), (int16_t)(y + (15 - size) / 2), icon, fg, fill);
    }
    gfx_text((int16_t)(x + 6 + (int16_t)icon_w), (int16_t)(y + 1), &font_small, label, fg, fill);
    *left_out = (int16_t)(x - 4);
}

static void dmm_draw_pill(int16_t right, uint16_t y, const char *label, uint16_t fill, uint16_t fg, uint16_t under, int16_t *left_out) {
    dmm_draw_pill_icon(right, y, 0, label, fill, fg, under, left_out);
}

static void ui_draw_status_bar(const char *title, uint16_t dot) {
    uint8_t pct_value = battery_percent();
    int16_t x;

    lcd_rect(0, 0, LCD_WIDTH, DMM_STATUS_H, D_BG);
    gfx_round_rect(8, 8, 6, 6, 3, dot, D_BG);
    gfx_text(19, 5, &font_small, title, D_TEXT, D_BG);

    // Battery: a filled body with the percentage inside, split in two colours
    // exactly at the fill edge; a small tip, straight left and rounded right.
    {
        enum { BX = 279, BY = 5, BW = 30, BH = 13, BR = 3, TIP_W = 2, TIP_H = 6, TIP_R = 2 };
        uint8_t full = battery_display_full();
        uint8_t charging = battery_charge_indicator_visible();
        uint16_t level = (charging || full) ? D_GREEN : (pct_value <= 10u ? D_RED : (pct_value <= 20u ? D_AMBER : D_TEXT));
        uint16_t fill = full ? BW : (uint16_t)((uint32_t)(pct_value > 100u ? 100u : pct_value) * BW / 100u);
        uint16_t tx = BX + BW + 1u;
        uint16_t ty = BY + (BH - TIP_H) / 2u;
        char num[4];
        uint16_t tw;
        int16_t nx;

        // Tip first, as the right half of a wider rounded shape; the body
        // then covers its left half and one background column keeps the gap.
        gfx_round_rect((uint16_t)(tx + TIP_W - 2u * TIP_R), ty, 2u * TIP_R, TIP_H, TIP_R, D_TRACK, D_BG);
        lcd_rect((uint16_t)(tx - 1u), BY, 1, BH, D_BG);
        // Body, then the level: the same rounded body in the level colour,
        // cut off where the fill ends. The number is drawn twice, cut at the
        // same edge: dark on the fill, light on the empty part.
        gfx_round_rect(BX, BY, BW, BH, BR, D_TRACK, D_BG);
        if (fill) {
            gfx_clip(-32768, (int16_t)(BX + fill - 1u));
            gfx_round_rect(BX, BY, BW, BH, BR, level, D_BG);
            gfx_clip_reset();
        }
        scope_format_u32(num, full ? 100u : pct_value);
        tw = gfx_text_width(&font_small, num);
        nx = (int16_t)(BX + (BW - (int16_t)tw) / 2);
        gfx_clip((int16_t)(BX + fill), 32767);
        gfx_text(nx, BY, &font_small, num, D_TEXT, D_TRACK);
        gfx_clip(-32768, (int16_t)(BX + fill - 1u));
        gfx_text(nx, BY, &font_small, num, D_BG, level);
        gfx_clip_reset();
        x = (int16_t)(BX - 4);
        if (charging) {
            dmm_icon((int16_t)(x - 11), 6, ICON_CHARGE_SM, D_GREEN, D_BG);
            x = (int16_t)(x - 15);
        }
    }
    if (usb_cdc_is_configured()) {
        dmm_icon((int16_t)(x - 12), 5, ICON_USB_SM, D_MUTED, D_BG);
        x = (int16_t)(x - 16);
    }
    if (gen_running || gen_output_applied) {
        int16_t left;
        dmm_draw_pill(x, 4, "GEN", D_GEN, D_BG, D_BG, &left);
    }
}

static void dmm_draw_status_bar(void) {
    ui_draw_status_bar("Multimeter", D_ACCENT);
}

static uint16_t dmm_value_color(uint8_t warning) {
    if (warning >= 2u) {
        return D_RED_TEXT;
    }
    if (warning) {
        return D_AMBER;
    }
    return D_TEXT;
}

// A solid tag placed left-aligned at x; returns the x after it.
static int16_t dmm_draw_tag_left(int16_t x, uint16_t y, const char *icon, const char *label,
                                 uint16_t fill, uint16_t fg, uint16_t under) {
    int16_t left;
    uint16_t icon_w = icon ? (uint16_t)(gfx_text_width(&font_icons, icon) + 3u) : 0u;
    uint16_t w = (uint16_t)(gfx_text_width(&font_small, label) + icon_w + 12u);
    dmm_draw_pill_icon((int16_t)(x + (int16_t)w), y, icon, label, fill, fg, under, &left);
    return (int16_t)(x + (int16_t)w);
}

static void dmm_draw_card_header(uint16_t card_color, uint16_t title_color, uint8_t warning) {
    int16_t right = (int16_t)(DMM_X + DMM_W - 12);
    int16_t title_end;

    title_end = gfx_text(DMM_IN_X, DMM_CARD_Y + 7, &font_medium, dmm_mode_names[ui.dmm_mode], title_color, card_color);

    if (dmm_hold_active) {
        dmm_draw_pill_icon(right, DMM_CARD_Y + 8, ICON_PAUSE_SM, "HOLD", D_AMBER, D_BG, card_color, &right);
    }
    if (dmm_rel_active) {
        dmm_draw_pill(right, DMM_CARD_Y + 8, "REL \xCE\x94", D_ACCENT, D_BG, card_color, &right);
    }
    if (warning) {
        // A solid tag, not a frame: frames are what dialogs look like.
        const char *label = warning >= 2u ? "HIGH VOLTAGE" : "CAUTION";
        uint16_t fill = warning >= 2u ? D_RED : D_AMBER;
        int16_t x = (int16_t)(title_end + 8);
        if (x + 27 + (int16_t)gfx_text_width(&font_small, label) <= right) {
            dmm_draw_tag_left(x, DMM_CARD_Y + 8, ICON_ALERT_SM, label, fill, D_BG, card_color);
        }
    } else if (dmm_current_mode()) {
        const char *label = dmm_high_current_mode() ? "USE HI JACK" : "USE LO JACK";
        int16_t x = (int16_t)(title_end + 8);
        if (x + 15 + (int16_t)gfx_text_width(&font_small, label) <= right) {
            dmm_icon(x, DMM_CARD_Y + 8, ICON_PLUG_SM, D_AMBER, card_color);
            gfx_text((int16_t)(x + 15), DMM_CARD_Y + 9, &font_small, label, D_AMBER, card_color);
        }
    } else if (ui.dmm_mode == DMM_MODE_DIODE && !dmm_hold_active && dmm_has_reading()) {
        // Continuity state: a small tag; the reading itself stays the big number.
        int16_t x = (int16_t)(title_end + 8);
        // Only the closed state gets a tag: an open circuit already reads OL.
        if (dmm_diode_continuity_active()) {
            dmm_draw_tag_left(x, DMM_CARD_Y + 8, 0, "CLOSED", D_GREEN, D_BG, card_color);
        }
    }
}

static void dmm_bar_label(char *out, uint8_t size, const char *sign, int32_t fs, uint8_t decimals, const char *unit) {
    char num[12];
    dmm_format_fixed(fs, decimals, decimals, 1, num);
    out[0] = 0;
    ui_text_append(out, sign, size);
    ui_text_append(out, num, size);
    if (unit[0]) {
        ui_text_append(out, " ", size);
        ui_text_append(out, unit, size);
    }
}

static uint16_t dmm_bar_length(uint32_t magnitude, uint32_t full_scale, uint16_t width) {
    if (!full_scale) {
        return 0;
    }
    if (magnitude >= full_scale) {
        return width;
    }
    // Reduce the 140/280-pixel span against the power-of-ten range first.
    // Even at four decimals the remaining product then fits in 32 bits.
    uint32_t divisor = full_scale;
    uint32_t factor = width;
    while (factor) {
        uint32_t remainder = divisor % factor;
        divisor = factor;
        factor = remainder;
    }
    return (uint16_t)(magnitude * (width / divisor) / (full_scale / divisor));
}

// Bar graph of the reading against the meter's current range. Positive
// readings fill from the left on a 0..range axis, negative ones from the
// right on a -range..0 axis. In REL the bar is centred on zero.
static void dmm_draw_bar(const char *value, const char *unit, uint16_t card_color, uint16_t fill_color) {
    enum { BAR_Y = DMM_CARD_Y + 113, LABEL_Y = DMM_CARD_Y + 120 };
    uint8_t decimals = dmm_stat_decimals();
    int32_t fs = dmm_range_full_scale(decimals);
    int32_t v = 0;
    int32_t mag;
    char label[24];

    gfx_round_rect(DMM_IN_X, BAR_Y, DMM_IN_W, 6, 3, D_TRACK, card_color);
    if (!fs || !dmm_text_is_number(value) || !parse_fixed_decimal(value[0] == '+' ? value + 1 : value, decimals, &v)) {
        return;
    }
    mag = v < 0 ? -v : v;

    if (dmm_rel_active && !dmm_hold_active) {
        uint16_t half = DMM_IN_W / 2u;
        uint16_t len = dmm_bar_length((uint32_t)mag, (uint32_t)fs, half);
        if (len) {
            gfx_round_rect(v < 0 ? (uint16_t)(DMM_IN_X + half - len) : (uint16_t)(DMM_IN_X + half),
                           BAR_Y, len, 6, len < 6u ? 0u : 3u, fill_color, D_TRACK);
        }
        lcd_rect(DMM_IN_X + half - 1u, BAR_Y - 3u, 2, 12, D_TEXT);
        dmm_bar_label(label, sizeof(label), "-", fs, decimals, unit);
        gfx_text(DMM_IN_X, LABEL_Y, &font_small, label, D_MUTED, card_color);
        dmm_bar_label(label, sizeof(label), "+", fs, decimals, unit);
        gfx_text_right(DMM_IN_X + DMM_IN_W, LABEL_Y, &font_small, label, D_MUTED, card_color);
        ui_text_copy(label, "Ref ", sizeof(label));
        ui_text_append(label, dmm_rel_ref_value, sizeof(label));
        gfx_text_center(DMM_IN_X, LABEL_Y, DMM_IN_W, &font_small, label, D_MUTED, card_color);
        return;
    }

    {
        uint16_t len = dmm_bar_length((uint32_t)mag, (uint32_t)fs, DMM_IN_W);
        if (len) {
            gfx_round_rect(v < 0 ? (uint16_t)(DMM_IN_X + DMM_IN_W - len) : DMM_IN_X, BAR_Y, len, 6,
                           len < 6u ? 0u : 3u, fill_color, D_TRACK);
        }
        for (uint8_t i = 1; i < 4u; ++i) {
            lcd_rect((uint16_t)(DMM_IN_X + DMM_IN_W * i / 4u), BAR_Y + 8u, 1, 3, D_TRACK);
        }
        if (v < 0) {
            dmm_bar_label(label, sizeof(label), "-", fs, decimals, unit);
            gfx_text(DMM_IN_X, LABEL_Y, &font_small, label, D_MUTED, card_color);
            gfx_text_right(DMM_IN_X + DMM_IN_W, LABEL_Y, &font_small, "0", D_MUTED, card_color);
        } else {
            gfx_text(DMM_IN_X, LABEL_Y, &font_small, "0", D_MUTED, card_color);
            dmm_bar_label(label, sizeof(label), "", fs, decimals, unit);
            gfx_text_right(DMM_IN_X + DMM_IN_W, LABEL_Y, &font_small, label, D_MUTED, card_color);
        }
    }
}

static void dmm_draw_reading_card(void) {
    enum { VALUE_Y = DMM_CARD_Y + 27 };
    uint8_t warning = dmm_voltage_warning_level();
    uint16_t card = D_CARD;
    const char *value = dmm_display_value();
    const char *unit = dmm_unit_pretty(dmm_display_unit());
    char signed_value[12];
    uint8_t closed = (uint8_t)(ui.dmm_mode == DMM_MODE_DIODE && !dmm_hold_active && dmm_diode_continuity_active());
    uint16_t value_color = closed ? D_GREEN : dmm_value_color(warning);
    uint16_t unit_color = (warning || closed) ? value_color : D_ACCENT;
    uint16_t bar_color = warning >= 2u ? D_RED : (warning ? D_AMBER : D_ACCENT);
    int16_t x;

    gfx_round_rect(DMM_X, DMM_CARD_Y, DMM_W, DMM_CARD_H, 14, card, D_BG);
    dmm_draw_card_header(card, D_SUB, warning);

    // A relative reading is a difference: show its sign either way.
    if (dmm_rel_active && !dmm_hold_active && dmm_text_is_number(value) && value[0] != '-') {
        signed_value[0] = '+';
        signed_value[1] = 0;
        ui_text_append(signed_value, value, sizeof(signed_value));
        value = signed_value;
    }
    // ...and with the meter's own precision, not the four decimals of the
    // fixed-point difference.
    if (dmm_rel_active && !dmm_hold_active && dmm_text_is_number(value)) {
        uint8_t keep = dmm_text_decimals(dmm_rel_ref_value);
        uint8_t has = dmm_text_decimals(value);
        if (value != signed_value) {
            ui_text_copy(signed_value, value, sizeof(signed_value));
            value = signed_value;
        }
        if (has > keep) {
            uint8_t len = 0;
            while (signed_value[len]) {
                ++len;
            }
            len = (uint8_t)(len - (has - keep) - (keep ? 0u : 1u));
            signed_value[len] = 0;
        }
    }

    if (!value[0] || ui_text_equal(value, "-----")) {
        // Waiting for the first reading.
        gfx_text(DMM_IN_X, VALUE_Y, &font_huge, "----", D_TRACK, card);
    } else if (ui_text_equal(value, "OPEN")) {
        // Over limit / open circuit, written the way meters write it.
        x = gfx_text(DMM_IN_X, VALUE_Y, &font_huge, "OL", D_SUB, card);
        if (unit[0]) {
            gfx_text((int16_t)(x + 6), (int16_t)(VALUE_Y + font_huge.ascent - font_large.ascent), &font_large, unit, D_MUTED, card);
        }
    } else if (ui_text_equal(value, "AUTO")) {
        // AUTO DETECT before the meter has settled on a quantity.
        gfx_text(DMM_IN_X, VALUE_Y + 26, &font_large, "Detecting", D_MUTED, card);
    } else if (dmm_text_is_number(value) &&
               gfx_text_width(&font_huge, value) + 6u + gfx_text_width(&font_large, unit) <= DMM_IN_W) {
        x = gfx_text(DMM_IN_X, VALUE_Y, &font_huge, value, value_color, card);
        // Units sit on the reading's baseline.
        gfx_text((int16_t)(x + 6), (int16_t)(VALUE_Y + font_huge.ascent - font_large.ascent), &font_large, unit, unit_color, card);
    } else {
        uint16_t color = dmm_text_is_number(value) ? value_color : D_SUB;
        x = gfx_text(DMM_IN_X, VALUE_Y + 26, &font_large, value, color, card);
        if (unit[0]) {
            gfx_text((int16_t)(x + 6), VALUE_Y + 26, &font_large, unit, unit_color, card);
        }
    }

    // A temperature has no meaningful "fraction of range".
    if (!dmm_is_temperature(ui.dmm_mode)) {
        dmm_draw_bar(value, unit, card, bar_color);
    }
}

static void dmm_draw_stats(void) {
    static const char *const labels[3] = {"MIN", "AVG", "MAX"};
    const char *value = dmm_display_value();
    uint8_t valid = dmm_stats.valid && dmm_stats.mode == ui.dmm_mode &&
                    ui_text_equal(dmm_stats.unit, dmm_display_unit()) && dmm_text_is_number(value);
    uint8_t shown = dmm_text_decimals(value);

    for (uint8_t i = 0; i < 3u; ++i) {
        uint16_t x = (uint16_t)(DMM_X + i * 103u);
        char text[12];
        gfx_round_rect(x, DMM_STATS_Y, 98, DMM_STATS_H, 10, D_CARD, D_BG);
        gfx_text((int16_t)(x + 9u), DMM_STATS_Y + 4, &font_small, labels[i], D_MUTED, D_CARD);
        if (valid) {
            int32_t v = i == 0u ? dmm_stats.min_value : (i == 1u ? dmm_stats.avg_value : dmm_stats.max_value);
            dmm_format_fixed(v, dmm_stats.decimals, shown, 0, text);
            gfx_text((int16_t)(x + 9u), DMM_STATS_Y + 16, &font_medium, text, D_TEXT, D_CARD);
        } else {
            gfx_text((int16_t)(x + 9u), DMM_STATS_Y + 16, &font_medium, "\xE2\x80\x94", D_MUTED, D_CARD);
        }
    }
}

// Live wire: one tall card. Detected turns it red; the idle card says how
// to use the mode, which is the one thing people get wrong with it.
static void dmm_draw_live_card(uint8_t detected) {
    enum { CX = DMM_X + 44, CY = DMM_CARD_Y + 86 };
    uint16_t card = detected ? D_RED_CARD : D_CARD;
    uint16_t icon_bg = detected ? D_RED : D_TRACK;

    gfx_round_rect(DMM_X, DMM_CARD_Y, DMM_W, DMM_TALL_H, 14, card, D_BG);
    gfx_text(DMM_IN_X, DMM_CARD_Y + 7, &font_medium, dmm_mode_names[ui.dmm_mode], detected ? D_RED_SUB : D_SUB, card);
    gfx_round_rect(CX - 28, CY - 28, 56, 56, 28, icon_bg, card);
    dmm_icon(CX - 16, CY - 16, ICON_ZAP_XL, detected ? D_BG : D_MUTED, icon_bg);
    if (detected) {
        gfx_text(DMM_X + 88, CY - 28, &font_large, "Detected", D_TEXT, card);
        gfx_text(DMM_X + 89, CY + 6, &font_medium, "Confirm with a voltage reading", D_RED_SUB, card);
    } else {
        gfx_text(DMM_X + 88, CY - 28, &font_large, "No live wire", D_SUB, card);
        gfx_text(DMM_X + 89, CY + 6, &font_medium, "Touch the red probe to the wire", D_MUTED, card);
    }
}

static void draw_dmm_immersive_panel(void) {
    lcd_rect(0, DMM_STATUS_H, LCD_WIDTH, DMM_KEYS_Y - DMM_STATUS_H - 2u, D_BG);

    if (ui.dmm_mode == DMM_MODE_LIVE) {
        dmm_draw_live_card(dmm_live_wire_detected());
        return;
    }

    dmm_draw_reading_card();
    dmm_draw_stats();
}

static void dmm_draw_softkey_label(uint8_t mode, int16_t x, uint16_t fg, uint16_t fill) {
    // Smooth versions of the symbols printed on the device's keys, from
    // font_icons. Unit marks sit centred on the capital letters beside them.
    enum {
        W = 71,
        // Capitals (2 px below the text top, 11 px tall) centred in the
        // 26 px pill: rows 7..17 of 0..25.
        TEXT_Y = DMM_KEYS_Y + 5,
        CAP_MID_X2 = 2 * (TEXT_Y + 2) + 11, // twice the capitals' centre (top 2 px, 11 px tall)
        CENTER_Y = DMM_KEYS_Y + 13,
        MARK_H = 11,
        MARK_GAP = 5,
        TEST_H = 20,
        CAP_H = 19,
    };
    const char *text;
    const char *mark;
    switch (mode) {
    case DMM_MODE_DCV:
    case DMM_MODE_ACV:
        text = "V";
        mark = mode == DMM_MODE_ACV ? ICON_KEY_AC : ICON_KEY_DC;
        break;
    case DMM_MODE_AC_HI_CURR:
    case DMM_MODE_DC_HI_CURR:
    case DMM_MODE_AC_LO_CURR:
    case DMM_MODE_DC_LO_CURR:
        text = (mode == DMM_MODE_AC_LO_CURR || mode == DMM_MODE_DC_LO_CURR) ? "mA" : "A";
        mark = (mode == DMM_MODE_AC_HI_CURR || mode == DMM_MODE_AC_LO_CURR) ? ICON_KEY_AC : ICON_KEY_DC;
        break;
    case DMM_MODE_DIODE:
    case DMM_MODE_CONT:
        gfx_text((int16_t)(x + (W - (int16_t)gfx_text_width(&font_icons, ICON_KEY_DIODE_CONT)) / 2),
                 (int16_t)(CENTER_Y - TEST_H / 2), &font_icons, ICON_KEY_DIODE_CONT, fg, fill);
        return;
    case DMM_MODE_CAP:
        gfx_text((int16_t)(x + (W - (int16_t)gfx_text_width(&font_icons, ICON_KEY_CAP)) / 2),
                 (int16_t)(CENTER_Y - CAP_H / 2), &font_icons, ICON_KEY_CAP, fg, fill);
        return;
    default:
        if (mode == DMM_MODE_RES) text = "\xCE\xA9";
        else if (mode == DMM_MODE_TEMP) text = "\xC2\xB0" "C";
        else if (mode == DMM_MODE_TEMP_F) text = "\xC2\xB0" "F";
        else if (mode == DMM_MODE_LIVE) text = "LIVE";
        else text = "AUTO";
        gfx_text_center(x, TEXT_Y, W, &font_medium, text, fg, fill);
        return;
    }
    int16_t total = (int16_t)(gfx_text_width(&font_medium, text) + MARK_GAP + gfx_text_width(&font_icons, mark));
    int16_t end = gfx_text((int16_t)(x + (W - total) / 2), TEXT_Y, &font_medium, text, fg, fill);
    gfx_text((int16_t)(end + MARK_GAP), (int16_t)((CAP_MID_X2 - MARK_H + 1) / 2), &font_icons, mark, fg, fill);
}

static void draw_dmm_softkeys(void) {
    uint8_t selected = ui.dmm_mode == DMM_MODE_AUTO ? DMM_GROUP_VOLTAGE : dmm_mode_group(ui.dmm_mode, 0);
    lcd_rect(0, DMM_KEYS_Y - 2u, LCD_WIDTH, LCD_HEIGHT - DMM_KEYS_Y + 2u, D_BG);
    for (uint8_t i = 0; i < DMM_GROUP_COUNT; ++i) {
        uint16_t x = (uint16_t)(9u + i * 77u);
        uint8_t active = i == selected;
        uint8_t mode = active ? ui.dmm_mode : dmm_groups[i].modes[dmm_group_choice[i]];
        uint16_t fill = active ? D_ACCENT : D_CARD;
        gfx_round_rect(x, DMM_KEYS_Y, 71, DMM_KEYS_H, 13, fill, D_BG);
        dmm_draw_softkey_label(mode, (int16_t)x, active ? D_BG : D_SUB, fill);
    }
}

static void draw_dmm(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, D_BG);
    dmm_draw_status_bar();
    draw_dmm_immersive_panel();
    draw_dmm_softkeys();
}

static void draw_dmm_immersive(void) {
    draw_dmm();
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
            if (scope_ch_enabled[ch] && scope_trace_cache[ch].valid) {
                scope_after_count[ch] = scope_trace_cache[ch].count;
                for (uint8_t i = 0; i < scope_trace_cache[ch].count; ++i) {
                    scope_after_y[ch][i] = scope_trace_cache[ch].y[i];
                }
                scope_after_valid[ch] = 1;
            } else if (!scope_ch_enabled[ch]) {
                scope_after_valid[ch] = 0;
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

static int32_t scope_sample_mean_mv(uint8_t idx, uint32_t sum, uint16_t count) {
    if (!count) return 0;
    int32_t numerator = ((int32_t)sum - (int32_t)count * 128) * 754;
    int32_t divisor = (int32_t)count * scope_range_code_for_channel(idx);
    int32_t mv = (numerator + (numerator >= 0 ? divisor / 2 : -divisor / 2)) / divisor;
    return scope_probe_x10[idx] ? mv * 10 : mv;
}

static void scope_measure_history_store(uint8_t min0,
                                        uint8_t max0,
                                        uint16_t rms0,
                                        uint8_t min1,
                                        uint8_t max1,
                                        uint16_t rms1,
                                        uint32_t freq0,
                                        uint32_t freq1,
                                        int32_t mean0,
                                        int32_t mean1) {
    scope_mean_mv[0] = mean0;
    scope_mean_mv[1] = mean1;
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
            scope_measure_mean_hist[0][i] = mean0;
            scope_measure_mean_hist[1][i] = mean1;
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
    scope_measure_mean_hist[0][scope_measure_hist_pos] = mean0;
    scope_measure_mean_hist[1][scope_measure_hist_pos] = mean1;
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

    if (!scope_ch_enabled[0]) {
        min0 = 128;
        max0 = 128;
        sum0 = (uint32_t)count * 128u;
        sq0 = 0;
    }
    if (!scope_ch_enabled[1]) {
        min1 = 128;
        max1 = 128;
        sum1 = (uint32_t)count * 128u;
        sq1 = 0;
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
                                scope_ch_enabled[0] ? scope_estimate_freq_hz_window(0, start, end, min0, max0) : 0,
                                scope_ch_enabled[1] ? scope_estimate_freq_hz_window(1, start, end, min1, max1) : 0,
                                scope_sample_mean_mv(0, sum0, count),
                                scope_sample_mean_mv(1, sum1, count));
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

    if (!scope_ch_enabled[0]) {
        min0 = 128;
        max0 = 128;
        sum0 = (uint32_t)count * 128u;
        sq0 = 0;
    }
    if (!scope_ch_enabled[1]) {
        min1 = 128;
        max1 = 128;
        sum1 = (uint32_t)count * 128u;
        sq1 = 0;
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
                                0,
                                scope_sample_mean_mv(0, sum0, count),
                                scope_sample_mean_mv(1, sum1, count));
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
    uint16_t visible_samples = scope_visible_sample_count();
    uint16_t trigger_screen_pos = scope_trigger_screen_sample_pos();
    uint16_t first = trigger_screen_pos ? trigger_screen_pos : 1u;
    uint16_t last = (uint16_t)(SCOPE_SAMPLE_COUNT - visible_samples + trigger_screen_pos);

    if (!samples || !offset ||
        ui.scope_display != SCOPE_DISPLAY_YT ||
        !scope_ch_enabled[src]) {
        return 0;
    }

    for (uint16_t i = first; i <= last; ++i) {
        uint8_t prev = samples[(uint16_t)((i - 1u) * 2u + src)];
        uint8_t cur = samples[(uint16_t)(i * 2u + src)];
        if (scope_trigger_crossed(prev, cur)) {
            *offset = (uint16_t)(i - trigger_screen_pos);
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
    if (scope_auto_zero_active || scope_auto_channel_mask) {
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
    scope_auto_timeout_service();
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
        if (ui.scope_trigger_mode != SCOPE_TRIGGER_AUTO && !scope_auto_channel_mask) {
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
    if (trigger_found && ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE && !scope_auto_channel_mask) {
        ui.running = 0;
    }
    scope_auto_zero_service();
    if (scope_auto_time_service()) {
        scope_trace_invalidate();
        return 1;
    }
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

static uint8_t scope_display_sample_raw(uint16_t x, uint8_t ch, uint16_t width) {
    if (!width) return 128u;
    if (x > width) x = width;
    uint16_t visible_samples = scope_visible_sample_count();
    uint16_t idx = (uint16_t)((uint32_t)x * (visible_samples - 1u) / width);
    if (ui.scope_display == SCOPE_DISPLAY_ROLL) {
        int32_t h_offset = scope_trigger_locked ? 0 : scope_h_pos_sample_offset();
        idx = (uint16_t)(((uint32_t)((int32_t)idx + (int32_t)scope_roll_offset + h_offset)) &
                         (SCOPE_SAMPLE_COUNT - 1u));
    } else {
        idx = (uint16_t)(scope_capture_window_start() + idx);
    }
    return scope_samples[(uint16_t)(idx * 2u + (ch ? 1u : 0u))];
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

    return scope_scaled_sample_y(ch2 ? 1u : 0u,
                                 scope_display_sample_raw(x, ch2, width),
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
    uint32_t unit_ns = scope_timebase_unit_ns_value();

    if (scope_safe_timebase() >= SCOPE_SLOW_ROLL_TIMEBASE_START) {
        return (unit_ns * 2u + 2u) / 5u;
    }
    // The FPGA span is truncated, so slow captures take fewer than 25 samples per division.
    return scope_hw_sample_period_ns(unit_ns);
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
    uint16_t crossings = 0;
    uint32_t span_samples;
    float period_ns;

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
            ++crossings;
        }
        prev = cur;
    }
    if (crossings < 2u || last <= first) {
        return 0;
    }
    span_samples = (uint32_t)(last - first);
    period_ns = ((float)span_samples * (float)scope_sample_period_ns()) / (float)(crossings - 1u);
    if (period_ns <= 0.0f) {
        return 0;
    }
    return (uint32_t)(1000000000.0f / period_ns);
}

static int32_t scope_filtered_mean_mv(uint8_t idx) {
    int32_t sum = 0;

    if (!scope_measure_hist_ready) return scope_mean_mv[idx];
    if (scope_safe_timebase() >= 17u) {
        return scope_measure_mean_hist[idx][scope_measure_current_slot()];
    }
    for (uint8_t i = 0; i < SCOPE_MEASURE_WINDOW; ++i) {
        sum += scope_measure_mean_hist[idx][i];
    }
    return (sum + (sum >= 0 ? SCOPE_MEASURE_WINDOW / 2 : -SCOPE_MEASURE_WINDOW / 2)) /
           SCOPE_MEASURE_WINDOW;
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

    // Lower case k, and Hz with only the H capital: the case is the unit.
    if (hz < 1000u) {
        value_x10 = hz * 10u;
        unit = "Hz";
    } else if (hz < 1000000u) {
        value_x10 = (hz + 50u) / 100u;
        unit = "kHz";
    } else {
        value_x10 = (hz + 50000u) / 100000u;
        unit = "MHz";
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
    if (measure == SCOPE_MEASURE_VPP) {
        scope_format_measure_voltage(out, (int32_t)scope_raw_span_mv(idx, span_raw));
    } else if (measure == SCOPE_MEASURE_VMAX) {
        scope_format_measure_voltage(out, max_mv);
    } else if (measure == SCOPE_MEASURE_VMIN) {
        scope_format_measure_voltage(out, min_mv);
    } else if (measure == SCOPE_MEASURE_VAVG) {
        scope_format_measure_voltage(out, scope_filtered_mean_mv(idx));
    } else if (measure == SCOPE_MEASURE_VRMS) {
        scope_format_measure_voltage(out, (int32_t)scope_rms_raw_mv(idx, scope_filtered_rms_raw(idx)));
    } else {
        scope_format_frequency_reading(out, scope_filtered_freq_hz(idx));
    }
}

// ---------------------------------------------------------------------------
// Oscilloscope chrome in the modern style. The traces themselves are drawn
// by the existing fast code; only what surrounds them changes here.
// ---------------------------------------------------------------------------

// "20mV" -> "20 mV", "1us" -> "1 µs", "0.090V" -> "0.090 V".
static void scope_pretty_value(const char *in, char *out, uint8_t size) {
    uint8_t o = 0;
    uint8_t spaced = 0;
    for (; *in && o + 3u < size; ++in) {
        char c = *in;
        uint8_t alpha = (uint8_t)((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'));
        uint8_t after_number = (uint8_t)(o && ((out[o - 1u] >= '0' && out[o - 1u] <= '9') || out[o - 1u] == '.'));
        if (alpha && !spaced && after_number) {
            out[o++] = ' ';
            spaced = 1;
            if (c == 'u' && in[1] >= 'A') {
                out[o++] = (char)0xC2;
                out[o++] = (char)0xB5;
                continue;
            }
        }
        out[o++] = c;
    }
    out[o] = 0;
    // Scope units arrive upper case ("MV", "KHZ"); on a scope M is milli
    // for volts and seconds, and k/M for hertz.
    {
        static const char *const from[] = {"MV", "UV", "KHZ", "MHZ", "HZ", "MS", "US", "NS", "DB"};
        static const char *const to[] = {"mV", "\xC2\xB5V", "kHz", "MHz", "Hz", "ms", "\xC2\xB5s", "ns", "dB"};
        char *unit = out;
        while (*unit && *unit != ' ') {
            ++unit;
        }
        if (*unit == ' ') {
            ++unit;
            for (uint8_t i = 0; i < 9u; ++i) {
                if (ui_text_equal(unit, from[i])) {
                    ui_text_copy(unit, to[i], (uint8_t)(size - (uint8_t)(unit - out)));
                    break;
                }
            }
        }
    }
}

static uint16_t scope_tint(uint16_t color, uint16_t under) {
    return gfx_blend(color, under, 4);
}

// A pill with an optional small label in front of a value, sized to fit.
static uint16_t scope_pill_width(const char *label, const char *value) {
    uint16_t w = (uint16_t)(gfx_text_width(&font_medium, value) + 14u);
    if (label) {
        w = (uint16_t)(w + gfx_text_width(&font_small, label) + 5u);
    }
    return w;
}

static void scope_draw_pill(int16_t x, uint16_t y, const char *label, const char *value,
                            uint16_t fill, uint16_t label_fg, uint16_t value_fg, uint16_t under) {
    uint16_t w = scope_pill_width(label, value);
    int16_t tx = (int16_t)(x + 7);
    gfx_round_rect((uint16_t)x, y, w, 17, 8, fill, under);
    if (label) {
        tx = gfx_text(tx, (int16_t)(y + 3u), &font_small, label, label_fg, fill);
        tx = (int16_t)(tx + 5);
    }
    gfx_text(tx, (int16_t)(y + 1u), &font_medium, value, value_fg, fill);
}

static void scope_channel_range_text(uint8_t idx, char out[16]) {
    char raw[12];
    const char *value;
    const char *unit;
    scope_vdiv_display(idx, &value, &unit);
    scope_text_copy(raw, value, sizeof(raw));
    ui_text_append(raw, unit, sizeof(raw));
    scope_pretty_value(raw, out, 16);
}

// Bottom bar: [CH1 20 mV]   [50 ns] [RUN]   [CH2 20 mV]
static void draw_scope_scale_status(uint16_t y, uint16_t bg) {
    char ch1[16];
    char ch2[16];
    char tb[16];
    const char *run = ui.running ? (scope_slow_roll_active() ? "ROLL" : "RUN") : "HOLD";
    uint16_t run_color = ui.running ? D_GREEN : D_AMBER;
    uint16_t tb_w;
    uint16_t run_w;
    int16_t x;

    lcd_rect(0, y, LCD_WIDTH, 19, bg);
    scope_channel_range_text(0, ch1);
    scope_channel_range_text(1, ch2);
    scope_pretty_value(scope_timebase_label(), tb, sizeof(tb));

    scope_draw_pill(8, (uint16_t)(y + 1u), "CH1", scope_ch_enabled[0] ? ch1 : "off",
                    scope_ch_enabled[0] ? scope_tint(C_CH1, bg) : D_CARD,
                    scope_ch_enabled[0] ? C_CH1 : D_MUTED, scope_ch_enabled[0] ? D_TEXT : D_MUTED, bg);
    scope_draw_pill((int16_t)(312 - (int16_t)scope_pill_width("CH2", scope_ch_enabled[1] ? ch2 : "off")),
                    (uint16_t)(y + 1u), "CH2", scope_ch_enabled[1] ? ch2 : "off",
                    scope_ch_enabled[1] ? scope_tint(C_CH2, bg) : D_CARD,
                    scope_ch_enabled[1] ? C_CH2 : D_MUTED, scope_ch_enabled[1] ? D_TEXT : D_MUTED, bg);

    tb_w = scope_pill_width(0, tb);
    run_w = scope_pill_width(0, run);
    x = (int16_t)((LCD_WIDTH - (tb_w + 6u + run_w)) / 2u);
    scope_draw_pill(x, (uint16_t)(y + 1u), 0, tb, D_CARD, D_TEXT, D_TEXT, bg);
    scope_draw_pill((int16_t)(x + (int16_t)tb_w + 6), (uint16_t)(y + 1u), 0, run,
                    scope_tint(run_color, bg), run_color, run_color, bg);
}

// Dotted graticule: dots every 4 px on division lines, a finer dotted
// centre cross with 1/5-division ticks. Collect pass marks the area once.
static void scope_draw_graticule(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh) {
    uint16_t cx = (uint16_t)(gx + ((uint32_t)gw * (SCOPE_X_DIVS / 2u)) / SCOPE_X_DIVS);
    uint16_t cy = (uint16_t)(gy + ((uint32_t)gh * (SCOPE_Y_DIVS / 2u)) / SCOPE_Y_DIVS);

    gfx_round_rect(gx, gy, gw, gh, 10, D_PLOT, D_BG);
    if (lcd_collecting()) {
        return;
    }
    for (uint8_t div = 1; div < SCOPE_X_DIVS; ++div) {
        uint16_t x = (uint16_t)(gx + ((uint32_t)gw * div) / SCOPE_X_DIVS);
        for (uint16_t y = (uint16_t)(gy + 4u); y < (uint16_t)(gy + gh - 3u); y = (uint16_t)(y + 4u)) {
            lcd_pixel((int16_t)x, (int16_t)y, D_PLOT_DOT);
        }
    }
    for (uint8_t div = 1; div < SCOPE_Y_DIVS; ++div) {
        uint16_t y = (uint16_t)(gy + ((uint32_t)gh * div) / SCOPE_Y_DIVS);
        for (uint16_t x = (uint16_t)(gx + 4u); x < (uint16_t)(gx + gw - 3u); x = (uint16_t)(x + 4u)) {
            lcd_pixel((int16_t)x, (int16_t)y, D_PLOT_DOT);
        }
    }
    for (uint16_t y = (uint16_t)(gy + 3u); y < (uint16_t)(gy + gh - 2u); y = (uint16_t)(y + 2u)) {
        lcd_pixel((int16_t)cx, (int16_t)y, D_PLOT_AXIS);
    }
    for (uint16_t x = (uint16_t)(gx + 3u); x < (uint16_t)(gx + gw - 2u); x = (uint16_t)(x + 2u)) {
        lcd_pixel((int16_t)x, (int16_t)cy, D_PLOT_AXIS);
    }
    for (uint8_t t = 1; t < SCOPE_X_DIVS * 5u; ++t) {
        uint16_t x = (uint16_t)(gx + ((uint32_t)gw * t) / (SCOPE_X_DIVS * 5u));
        lcd_rect(x, (uint16_t)(cy - 2u), 1, 5, D_PLOT_AXIS);
    }
    for (uint8_t t = 1; t < SCOPE_Y_DIVS * 5u; ++t) {
        uint16_t y = (uint16_t)(gy + ((uint32_t)gh * t) / (SCOPE_Y_DIVS * 5u));
        lcd_rect((uint16_t)(cx - 2u), y, 5, 1, D_PLOT_AXIS);
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
    label = scope_measure_pretty_label(measure);
    scope_measure_value_for(value, idx, measure);

    {
        char pretty[16];
        uint16_t fill = scope_tint(color, D_PLOT);
        scope_pretty_value(value, pretty, sizeof(pretty));
        total_w = scope_pill_width(label, pretty);
        (void)label_w;
        (void)value_w;
        (void)value_scale;
        if (align_right && total_w < width) {
            x = (uint16_t)(right - total_w);
        }
        scope_draw_pill((int16_t)x, y, label, pretty, fill, color, D_TEXT, D_PLOT);
    }
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
    // A small downward pointer at the top of the plot.
    for (uint8_t r = 0; r < 5u; ++r) {
        lcd_rect((uint16_t)(cx - 4 + r), (uint16_t)(gy + 1u + r), (uint16_t)(9u - 2u * r), 1, D_TEXT);
    }
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
        {
            // A small rounded tag on the plot's left edge, pointing at the
            // channel's zero line; the active channel's tag is outlined.
            int16_t ty = (int16_t)(y - 6);
            uint8_t active = ui.active_ch == i + 1u;
            if (ty < (int16_t)(gy + 1u)) {
                ty = (int16_t)(gy + 1u);
            }
            if (active) {
                gfx_round_rect((uint16_t)(gx + 1u), (uint16_t)ty, 15, 13, 4, D_TEXT, bg);
                gfx_round_rect((uint16_t)(gx + 2u), (uint16_t)(ty + 1), 13, 11, 3, color, D_TEXT);
            } else {
                gfx_round_rect((uint16_t)(gx + 2u), (uint16_t)(ty + 1), 13, 11, 3, color, bg);
            }
            gfx_text_center((int16_t)(gx + 2u), (int16_t)(ty + 1), 13, &font_small, label, D_BG, color);
            lcd_rect((uint16_t)(gx + 15u), (uint16_t)y, 3, 1, color);
        }
    }
}

static void draw_scope_trigger_line(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh, uint16_t bg) {
    int16_t y;
    uint8_t src = scope_trigger_source_index();
    uint16_t color = ui.scope_trigger_mode == SCOPE_TRIGGER_SINGLE ? C_WARN : scope_channel_color((uint8_t)(src + 1u));
    const char *level = scope_trigger_level_label();
    int16_t center;
    int16_t amp = ui.chrome_visible ? 48 : 64;

    if (ui.scope_display == SCOPE_DISPLAY_XY ||
        !scope_ch_enabled[src]) {
        return;
    }
    center = scope_channel_center(scope_channel_base_center_for(src), src);
    y = scope_scaled_sample_y(src, ui.scope_trigger_level, center, amp,
                              (int16_t)(gy + 1u), (int16_t)(gy + gh - 2u));
    for (uint16_t x = (uint16_t)(gx + 1u); x < (uint16_t)(gx + gw - 1u); x = (uint16_t)(x + 8u)) {
        lcd_rect(x, (uint16_t)y, 5, 1, color);
    }
    {
        char pretty[16];
        const char *icon = ui.scope_trigger_edge ? ICON_EDGE_FALL_SM : ICON_EDGE_RISE_SM;
        uint16_t fill = scope_tint(color, bg);
        uint16_t w;
        int16_t px;
        int16_t py = (int16_t)(y > (int16_t)(gy + 18u) ? y - 16 : y + 3);

        scope_pretty_value(level, pretty, sizeof(pretty));
        w = (uint16_t)(gfx_text_width(&font_small, pretty) + 11u + 4u + 12u);
        px = (int16_t)(gx + gw - 6u - w);
        gfx_round_rect((uint16_t)px, (uint16_t)py, w, 14, 7, fill, bg);
        gfx_text((int16_t)(px + 6), (int16_t)(py + 2), &font_icons, icon, color, fill);
        gfx_text((int16_t)(px + 21), (int16_t)(py + 1), &font_small, pretty, color, fill);
    }
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

    if (ch == 0 && (scope_hide_traces == 1 || scope_hide_traces == 3)) {
        return;
    }
    if (ch == 1 && (scope_hide_traces == 2 || scope_hide_traces == 3)) {
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

    if (scope_hide_traces > 0) {
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
    if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        uint16_t half_h = (uint16_t)(gh / 2u);
        uint8_t sub = bode_cursor_sub;
        for (uint8_t i = 0; i < 2u; ++i) {
            uint16_t color = i == ui.scope_cursor_sel ? C_TEXT : C_MUTED;
            if (sub == 0u) {
                uint16_t x = (uint16_t)(gx + 1u + (uint32_t)scope_cursor_x[i] * (gw - 3u) / 255u);
                lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), color);
                lcd_text((uint16_t)(x > 8u ? x - 7u : x), (uint16_t)(gy + 3u), i ? "F2" : "F1", color, bg, 1);
            } else {
                uint16_t base = (sub == 1u) ? gy : (uint16_t)(gy + half_h);
                uint16_t y = (uint16_t)(base + 1u + (uint32_t)scope_cursor_y[i] * (half_h - 3u) / 255u);
                lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, color);
                const char *cl = (sub == 1u) ? (i ? "DB2" : "DB1") : (i ? "P2" : "P1");
                lcd_text((uint16_t)(gx + 4u), (uint16_t)(y > 8u ? y - 7u : y), cl, color, bg, 1);
            }
        }
        return;
    }
    if (ui.scope_cursor_mode == SCOPE_CURSOR_TIME) {
        for (uint8_t i = 0; i < 2u; ++i) {
            uint16_t x = (uint16_t)(gx + 1u + (uint32_t)scope_cursor_x[i] * (gw - 3u) / 255u);
            uint16_t color = i == ui.scope_cursor_sel ? C_TEXT : C_MUTED;
            lcd_rect(x, (uint16_t)(gy + 1u), 1, (uint16_t)(gh - 2u), color);
            const char *cl;
            if (scope_fft_src == 4u) {
                cl = i ? "DB2" : "DB1";
            } else if (scope_fft_src == 1u || scope_fft_src == 2u) {
                cl = i ? "F2" : "F1";
            } else {
                cl = i ? "T2" : "T1";
            }
            lcd_text((uint16_t)(x > 8u ? x - 7u : x), (uint16_t)(gy + 3u), cl, color, bg, 1);
        }
    } else if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        uint16_t half_h = (uint16_t)(gh / 2u);
        for (uint8_t i = 0; i < 2u; ++i) {
            uint16_t y;
            if (scope_fft_src == 4u) {
                uint16_t base = (i == 0u) ? gy : (uint16_t)(gy + half_h);
                y = (uint16_t)(base + 1u + (uint32_t)scope_cursor_y[i] * (half_h - 3u) / 255u);
            } else {
                y = (uint16_t)(gy + 1u + (uint32_t)scope_cursor_y[i] * (gh - 3u) / 255u);
            }
            uint16_t color = i == ui.scope_cursor_sel ? C_TEXT : C_MUTED;
            lcd_rect((uint16_t)(gx + 1u), y, (uint16_t)(gw - 2u), 1, color);
            const char *cl = (scope_fft_src == 4u) ? (i ? "P2" : "P1") : (i ? "Y2" : "Y1");
            lcd_text((uint16_t)(gx + 4u), (uint16_t)(y > 8u ? y - 7u : y), cl, color, bg, 1);
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

    if (ch == 0 && (scope_hide_traces == 1 || scope_hide_traces == 3)) {
        return;
    }
    if (ch == 1 && (scope_hide_traces == 2 || scope_hide_traces == 3)) {
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

    if (!ch2 && (scope_hide_traces == 1 || scope_hide_traces == 3)) {
        return; 
    }
    if (ch2 && (scope_hide_traces == 2 || scope_hide_traces == 3)) {
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

static void ui_draw_math_waveform(uint16_t gx, uint16_t gy, uint16_t gw, uint16_t gh) {
    if (!scope_math_mode || !scope_frame_valid ||
        !scope_ch_enabled[0] || !scope_ch_enabled[1] || gw < 4u || gh < 4u) {
        return;
    }

    int16_t center_y = (int16_t)(gy + gh / 2u);
    int16_t prev_y = -1;
    int16_t prev_x = 0;
    uint16_t math_color = RGB565(255, 0, 255);
    uint16_t width = (uint16_t)(gw - 3u);
    uint8_t slow_roll = scope_slow_roll_active();
    uint16_t sample_count = slow_roll ? scope_slow_roll_count :
        (uint16_t)(width / SCOPE_TRACE_STEP + 1u);
    if (!slow_roll && sample_count > SCOPE_TRACE_MAX_POINTS) sample_count = SCOPE_TRACE_MAX_POINTS;
    if (slow_roll && sample_count > SCOPE_SLOW_ROLL_MAX_POINTS) sample_count = SCOPE_SLOW_ROLL_MAX_POINTS;
    // Math uses the first operand's V/div and probe scale, independent of trace positions.
    uint8_t reference_ch = scope_math_op == 2u ? 1u : 0u;
    int32_t vdiv = (int32_t)scope_vdiv_mv_for_channel(reference_ch);

    for (uint16_t i = 0; i < sample_count; ++i) {
        uint16_t x_offset;
        uint8_t raw0;
        uint8_t raw1;
        if (slow_roll) {
            int16_t display_i = (int16_t)i + scope_roll_display_offset;
            if (display_i < 0 || display_i >= SCOPE_SLOW_ROLL_MAX_POINTS) {
                prev_y = -1;
                continue;
            }
            x_offset = (uint16_t)((uint32_t)display_i * width / (SCOPE_SLOW_ROLL_MAX_POINTS - 1u));
            raw0 = scope_slow_roll_raw_at(0, i);
            raw1 = scope_slow_roll_raw_at(1, i);
        } else {
            x_offset = (uint16_t)(i * SCOPE_TRACE_STEP);
            raw0 = scope_display_sample_raw(x_offset, 0, width);
            raw1 = scope_display_sample_raw(x_offset, 1, width);
        }
        int32_t mv0 = scope_raw_delta_mv(0, raw0);
        int32_t mv1 = scope_raw_delta_mv(1, raw1);
        int32_t mv = scope_math_op == 0u ? mv0 + mv1 :
            (scope_math_op == 1u ? mv0 - mv1 : mv1 - mv0);
        int32_t computed_y = center_y - (mv * 64) / (vdiv * 3);
        if (computed_y < gy + 1) computed_y = gy + 1;
        if (computed_y > gy + gh - 2) computed_y = gy + gh - 2;
        int16_t x = (int16_t)(gx + 1u + x_offset);
        if (prev_y != -1) {
            lcd_line(prev_x, prev_y, x, (int16_t)computed_y, math_color);
        }
        prev_x = x;
        prev_y = (int16_t)computed_y;
    }
}

static void draw_scope_immersive(void) {
    const uint16_t grid_bg = D_PLOT;
    uint16_t gx = 8;
    uint16_t gy = 28;
    uint16_t gw = 304;
    uint16_t gh = 190;

    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, D_BG);
    ui_draw_status_bar("Oscilloscope", D_SCOPE);

#if SCOPE_UI_SAFE_STUB
    scope_sanitize_state();
    int16_t ch1_center = scope_channel_center((int16_t)(gy + gh / 2u - 30u), 0);
    int16_t ch2_center = scope_channel_center((int16_t)(gy + gh / 2u + 34u), 1);
    draw_scope_stub_grid(gx, gy, gw, gh, grid_bg);
    if (ui.scope_display == SCOPE_DISPLAY_XY) {
        draw_scope_xy(gx, gy, gw, gh, C_SCOPE);
    } else {
        if (ui.scope_afterglow) {
            if (scope_ch_enabled[0]) {
                draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, (int16_t)(scope_stub_amp(0) + 16), scope_dim_color(C_CH1), (uint8_t)(ui.scope_phase - 18u));
            }
            if (scope_ch_enabled[1]) {
                draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, (int16_t)(scope_stub_amp(1) + 12), scope_dim_color(C_CH2), (uint8_t)(ui.scope_phase + 46u));
            }
        }
        if (scope_ch_enabled[0]) {
            draw_scope_stub_trace(gx, gy, gw, gh, ch1_center, (int16_t)(scope_stub_amp(0) + 16), C_CH1, (uint8_t)ui.scope_phase);
        }
        if (scope_ch_enabled[1]) {
            draw_scope_stub_trace(gx, gy, gw, gh, ch2_center, (int16_t)(scope_stub_amp(1) + 12), C_CH2, (uint8_t)(ui.scope_phase + 64u));
        }
        draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
    }
    draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
    draw_scope_cursors(gx, gy, gw, gh, grid_bg);
    draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
    draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
    draw_scope_measurements(gx, gy, gw, gh, grid_bg);
    draw_scope_scale_status(220, grid_bg);
    return;
#endif

    scope_draw_graticule(gx, gy, gw, gh);

    if (scope_frame_valid || !scope_hw_enabled() || scope_fft_src == 4u) {
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
            } else if (scope_fft_src == 4u) {
                ui_draw_bode(gx, gy, gw, gh);
            } else {
                draw_scope_afterglow_trace(0, plot_x0, plot_w, C_CH1);
                draw_scope_afterglow_trace(1, plot_x0, plot_w, C_CH2);
                draw_scope_trace(C_CH1, 0, plot_x0, ch1_center, 64, plot_w, gy, gh);
                draw_scope_trace(C_CH2, 1, plot_x0, ch2_center, 64, plot_w, gy, gh);
            }
            if (scope_fft_src != 4u) {
                ui_draw_math_waveform(gx, gy, gw, gh);
                ui_draw_fft_spectrum(gx, gy, gw, gh);
                draw_scope_channel_markers(gx, gy, gh, ch1_center, ch2_center, grid_bg);
                draw_scope_trigger_line(gx, gy, gw, gh, grid_bg);
            }
        }
        draw_scope_cursors(gx, gy, gw, gh, grid_bg);
        draw_scope_cursor_readout(gx, gy, gw, gh, grid_bg);
        draw_scope_hpos_indicator(gx, gy, gw, gh, grid_bg);
        draw_scope_measure_readout((uint16_t)(gx + 6u), gy, (uint16_t)(gw - 12u), gh, grid_bg);
    } else {
        gfx_text_center((int16_t)gx, (int16_t)(gy + 80u), gw, &font_medium,
                        scope_hw_ready() ? "Waiting for trigger" : "Starting FPGA", D_MUTED, grid_bg);
    }

    draw_scope_scale_status(220, D_BG);
    ui_draw_math_menu();
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

static void arb_load_waveform(void);

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
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        ui.gen_param = GEN_PARAM_FREQ;
        arb_scan_files();
        if (arb_file_index >= arb_file_count() && arb_file_count() > 0) {
            arb_file_index = 0;
        }
        arb_load_waveform();
    }
}

static uint8_t gen_wave_uses_duty(void) {
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        return 1;
    }
    return ui.gen_wave == SIGGEN_WAVE_SQUARE ||
           ui.gen_wave == SIGGEN_WAVE_TRIANGLE ||
           ui.gen_wave == SIGGEN_WAVE_SINKER_PULSE;
}

static uint8_t gen_param_available(uint8_t param) {
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        return 1;
    }
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
    } else if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        uint16_t count = arb_sample_count;
        uint16_t idx;

        if (count == 0) {
            return 0;
        }
        idx = (uint16_t)((uint32_t)phase * count / 255u);
        if (idx >= count) {
            idx = (uint16_t)(count - 1u);
        }
        return arb_samples[idx];
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

static const char *const gen_wave_names[] = {
    "Sine", "Square", "Sawtooth", "Half wave", "Full wave",
    "Step up", "Step down", "Exp rise", "Exp fall", "DC",
    "Multi-audio", "Sinker pulse", "Lorentz", "Triangle", "Noise", "Arbitrary",
};

// Preview: a rounded plot with a dotted centre line and the waveform drawn
// anti-aliased in the generator's violet.
static void draw_gen_wave_preview(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bg) {
    uint16_t center_y = (uint16_t)(y + h / 2u);
    uint16_t plot_x0 = (uint16_t)(x + 8u);
    uint16_t plot_w = (uint16_t)(w - 16u);
    int16_t amp = (int16_t)(h / 2u - 12u);
    int16_t prev_x = (int16_t)plot_x0;
    int16_t prev_y = gen_wave_preview_y(0, plot_w, (int16_t)center_y, amp);

    gfx_round_rect(x, y, w, h, 10, D_PLOT, bg);
    if (!lcd_collecting()) {
        for (uint16_t px = (uint16_t)(x + 6u); px < (uint16_t)(x + w - 6u); px = (uint16_t)(px + 3u)) {
            lcd_pixel((int16_t)px, (int16_t)center_y, D_PLOT_AXIS);
        }
        for (uint8_t q = 1; q < 4u; ++q) {
            uint16_t gx = (uint16_t)(x + (uint32_t)w * q / 4u);
            for (uint16_t py = (uint16_t)(y + 5u); py < (uint16_t)(y + h - 5u); py = (uint16_t)(py + 4u)) {
                lcd_pixel((int16_t)gx, (int16_t)py, D_PLOT_DOT);
            }
        }
    }
    // One column at a time: a solid span from this point to the next, two
    // pixels thick, with a soft pixel at each end. Unlike joined line
    // segments this never paints over itself, so the trace stays even.
    (void)prev_x;
    for (uint16_t px = 0; px < plot_w; ++px) {
        int16_t y0 = px ? prev_y : gen_wave_preview_y(0, plot_w, (int16_t)center_y, amp);
        int16_t y1 = gen_wave_preview_y((uint16_t)(px + 1u), plot_w, (int16_t)center_y, amp);
        int16_t lo = y0 < y1 ? y0 : y1;
        int16_t hi = (int16_t)((y0 > y1 ? y0 : y1) + 1);
        int16_t sx = (int16_t)(plot_x0 + px);
        lcd_rect((uint16_t)sx, (uint16_t)lo, 1, (uint16_t)(hi - lo + 1), D_GEN);
        lcd_pixel(sx, (int16_t)(lo - 1), gfx_blend(D_GEN, D_PLOT, 6));
        lcd_pixel(sx, (int16_t)(hi + 1), gfx_blend(D_GEN, D_PLOT, 6));
        prev_y = y1;
    }
}


// Wave name and preview in one card.
static void draw_gen_wave_card(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint8_t wave = ui.gen_wave < SIGGEN_WAVE_COUNT ? ui.gen_wave : SIGGEN_WAVE_SINE;
    gfx_round_rect(x, y, w, h, 14, D_CARD, D_BG);
    gfx_text((int16_t)(x + 12u), (int16_t)(y + 6u), &font_small, "WAVEFORM", D_MUTED, D_CARD);
    gfx_text((int16_t)(x + 12u), (int16_t)(y + 17u), &font_large, gen_wave_names[wave], D_TEXT, D_CARD);
    draw_gen_wave_preview((uint16_t)(x + 8u), (uint16_t)(y + 54u), (uint16_t)(w - 16u), (uint16_t)(h - 62u), D_CARD);
}

// Output state: a tall card with a power icon, green when the output is on.
static void draw_gen_output_panel(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bg) {
    uint8_t on = ui.running;
    uint16_t card = on ? D_GREEN_CARD : D_CARD;
    uint16_t accent = on ? D_GREEN : D_MUTED;
    uint16_t cx = (uint16_t)(x + w / 2u);
    uint16_t cy = (uint16_t)(y + h / 2u - 6u);

    gfx_round_rect(x, y, w, h, 14, card, bg);
    gfx_text_center((int16_t)x, (int16_t)(y + 6u), w, &font_small, "OUTPUT", on ? D_GREEN_SUB : D_MUTED, card);
    gfx_round_rect((uint16_t)(cx - 22u), (uint16_t)(cy - 22u), 44, 44, 22, on ? D_GREEN : D_TRACK, card);
    gfx_text((int16_t)(cx - 8u), (int16_t)(cy - 8u), &font_icons, ICON_START_MD, on ? D_BG : D_SUB, on ? D_GREEN : D_TRACK);
    gfx_text_center((int16_t)x, (int16_t)(cy + 28u), w, &font_large, on ? "On" : "Off", on ? D_TEXT : D_SUB, card);
    (void)accent;
}

// How many leading digits are zeros that carry nothing. The last place is
// never counted, so a value of zero still shows one solid 0 rather than
// fading away altogether.
static uint8_t editor_dim_digits(const char *digits, uint8_t count) {
    uint8_t dim = 0;

    if (!digits || count < 2u) {
        return 0;
    }
    while (dim < (uint8_t)(count - 1u) && digits[dim] == '0') {
        ++dim;
    }
    return dim;
}

// Digit-by-digit editors: fixed-width digits; the place being edited sits
// on a solid accent block, leading zeros are dimmed.
// Digit-by-digit editors: fixed-width digits; the place being edited sits
// on a solid accent block, leading zeros are dimmed.
static int16_t draw_editor_digits_dot(int16_t x, uint16_t y, const char *digits, uint8_t count,
                                      int8_t selected, uint8_t dim, int8_t dot_after,
                                      uint16_t accent, uint16_t fg, uint16_t bg) {
    uint16_t adv = gfx_text_width(&font_medium, "0");
    for (uint8_t i = 0; i < count; ++i) {
        char text[2] = {digits[i], 0};
        if ((int8_t)i == selected) {
            gfx_round_rect((uint16_t)(x - 2), (uint16_t)(y + 1u), (uint16_t)(adv + 3u), 16, 4, accent, bg);
            gfx_text(x, (int16_t)y, &font_medium, text, D_BG, accent);
        } else {
            gfx_text(x, (int16_t)y, &font_medium, text, i < dim ? D_TRACK : fg, bg);
        }
        x = (int16_t)(x + (int16_t)adv);
        if ((int8_t)i == dot_after) {
            x = gfx_text(x, (int16_t)y, &font_medium, ".", fg, bg);
        }
    }
    return x;
}

static void ui_draw_editor_digits(int16_t x, uint16_t y, const char *digits, uint8_t count,
                                  int8_t selected, uint8_t dim, uint16_t accent, uint16_t fg, uint16_t bg) {
    (void)draw_editor_digits_dot(x, y, digits, count, selected, dim, -1, accent, fg, bg);
}

static int16_t draw_gen_editor_digits(int16_t x, uint16_t y, const char *digits, uint8_t count,
                                      int8_t selected, uint8_t dim, int8_t dot_after,
                                      uint16_t fg, uint16_t bg) {
    return draw_editor_digits_dot(x, y, digits, count, selected, dim, dot_after, D_GEN, fg, bg);
}

static void draw_gen_editor_unit(int16_t x, uint16_t y, const char *unit, uint8_t selected, uint16_t fg, uint16_t bg) {
    if (selected) {
        uint16_t w = (uint16_t)(gfx_text_width(&font_small, unit) + 4u);
        gfx_round_rect((uint16_t)(x - 2), (uint16_t)(y + 3u), w, 13, 3, D_GEN, bg);
        gfx_text(x, (int16_t)(y + 3u), &font_small, unit, D_BG, D_GEN);
    } else {
        gfx_text(x, (int16_t)(y + 3u), &font_small, unit, fg, bg);
    }
}

static void draw_gen_param_chip(uint16_t x, uint16_t y, uint16_t w, uint8_t param, const char *value) {
    uint8_t enabled = gen_param_available(param);
    uint8_t active = enabled && ui.gen_param == param;
    uint8_t arbitrary = ui.gen_wave == SIGGEN_WAVE_ARBITRARY;
    uint16_t bg;
    uint16_t fg = active ? D_TEXT : D_SUB;
    uint16_t vy = (uint16_t)(y + 14u);
    int8_t sel = active ? (int8_t)gen_freq_edit_pos : -1;

    if ((param == GEN_PARAM_FREQ && !arbitrary) ||
        (param == GEN_PARAM_DUTY && enabled && !arbitrary) ||
        (param == GEN_PARAM_AMP && enabled)) {
        char digits[5];
        int16_t ex;
        bg = ui_chip_frame(x, y, w, D_GEN, active);
        gfx_text((int16_t)(x + 8u), (int16_t)(y + 3u), &font_small, gen_param_labels[param], active ? D_GEN : D_MUTED, bg);
        if (param == GEN_PARAM_FREQ) {
            static const char *const units[] = {"Hz", "kHz", "MHz"};
            uint8_t unit = gen_freq_unit < GEN_FREQ_UNIT_COUNT ? gen_freq_unit : 0u;
            format_gen_freq_digits(digits);
            ex = draw_gen_editor_digits((int16_t)(x + 7u), vy, digits, GEN_FREQ_EDIT_DIGITS, sel,
                                        editor_dim_digits(digits, GEN_FREQ_EDIT_DIGITS), -1, fg, bg);
            draw_gen_editor_unit((int16_t)(ex + 3), vy, units[unit < 3u ? unit : 0u],
                                 (uint8_t)(active && gen_freq_edit_pos == GEN_FREQ_EDIT_UNIT), fg, bg);
        } else if (param == GEN_PARAM_DUTY) {
            format_u16_3(gen_clamped_duty(), digits);
            ex = draw_gen_editor_digits((int16_t)(x + 12u), vy, digits, GEN_DUTY_EDIT_DIGITS, sel,
                                        editor_dim_digits(digits, GEN_DUTY_EDIT_DIGITS), -1, fg, bg);
            gfx_text((int16_t)(ex + 2), (int16_t)vy, &font_medium, "%", fg, bg);
        } else {
            format_u8_2(gen_clamped_amp_tenths(), digits);
            ex = draw_gen_editor_digits((int16_t)(x + 18u), vy, digits, GEN_AMP_EDIT_DIGITS, sel, 0, 0, fg, bg);
            gfx_text((int16_t)(ex + 2), (int16_t)vy, &font_medium, "V", fg, bg);
        }
        return;
    }
    ui_draw_chip(x, y, w, gen_param_labels[param], value, D_GEN, active, enabled);
}

static void draw_gen_param_row(uint16_t x, uint16_t y) {
    uint8_t wave = ui.gen_wave < SIGGEN_WAVE_COUNT ? ui.gen_wave : SIGGEN_WAVE_SINE;
    const char *freq_val = "";
    const char *duty_val = "";
    const char *amp_val = "";

    if (wave == SIGGEN_WAVE_ARBITRARY) {
        freq_val = arb_file_index < arb_file_count() ? arb_file_name(arb_file_index) : "NO FILE";
        {
            static char pts_buf[8];
            uint16_t cnt;
            uint16_t div;
            uint8_t pos;
            uint8_t started;

            cnt = arb_sample_count;
            if (cnt == 0) {
                cnt = 1;
            }
            pos = 0;
            started = 0;
            div = 1000;
            while (div > 0) {
                uint8_t digit = (uint8_t)(cnt / div);
                cnt %= div;
                if (digit > 0 || started || div == 1) {
                    started = 1;
                    pts_buf[pos++] = (char)('0' + digit);
                }
                div /= 10;
            }
            if (pos >= 4) {
                pts_buf[pos++] = 'p';
                pts_buf[pos++] = 't';
            } else {
                pts_buf[pos++] = 'p';
                pts_buf[pos++] = 't';
                pts_buf[pos++] = 's';
            }
            pts_buf[pos] = 0;
            duty_val = pts_buf;
        }
    }

    draw_gen_param_chip(x, y, GEN_PARAM_CHIP_W, GEN_PARAM_WAVE, gen_wave_names[wave]);
    draw_gen_param_chip((uint16_t)(x + GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_FREQ,
                        freq_val);
    draw_gen_param_chip((uint16_t)(x + (GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP) * 2u),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_DUTY,
                        duty_val);
    draw_gen_param_chip((uint16_t)(x + (GEN_PARAM_CHIP_W + GEN_PARAM_CHIP_GAP) * 3u),
                        y,
                        GEN_PARAM_CHIP_W,
                        GEN_PARAM_AMP,
                        amp_val);
}

static void ui_draw_gen_sweep_menu(void) {
    static const char *const sweep_names[] = {"Off", "Linear", "Log"};
    static const char *const fm_names[] = {"Off", "On"};
    static const char *const fm_wave_names[] = {"Sine", "Triangle", "Square"};
    uint8_t sel = gen_sweep_row_selected;
    uint16_t y;
    uint16_t fill;

    if (!gen_sweep_menu_active) {
        return;
    }
    y = ui_list_begin("Sweep and modulation", ICON_GENERATOR_SM, D_GEN, gfx_blend(D_GEN, D_CARD, 4), 7);

    ui_list_row(y, "Sweep", sweep_names[gen_sweep_mode < 3u ? gen_sweep_mode : 0u], sel == 0u, 1, D_GEN);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    fill = ui_list_row(y, "Sweep time", 0, sel == 1u, gen_sweep_mode != 0u, D_GEN);
    ui_list_digits(y, gen_sweep_ms, (uint8_t)(ui_edit_digit_active && sel == 1u), ui_edit_digit_index,
                   "ms", sel == 1u, gen_sweep_mode != 0u, D_GEN, fill);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    fill = ui_list_row(y, "Start freq", 0, sel == 2u, gen_sweep_mode != 0u, D_GEN);
    ui_list_digits(y, freq_clamp_display(siggen_sweep_start_hz, sweep_start_unit),
                   (uint8_t)(ui_edit_digit_active && sel == 2u), ui_edit_digit_index,
                   freq_unit_labels[sweep_start_unit < 3u ? sweep_start_unit : 0u], sel == 2u, gen_sweep_mode != 0u, D_GEN, fill);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    fill = ui_list_row(y, "Stop freq", 0, sel == 3u, gen_sweep_mode != 0u, D_GEN);
    ui_list_digits(y, freq_clamp_display(siggen_sweep_stop_hz, sweep_stop_unit),
                   (uint8_t)(ui_edit_digit_active && sel == 3u), ui_edit_digit_index,
                   freq_unit_labels[sweep_stop_unit < 3u ? sweep_stop_unit : 0u], sel == 3u, gen_sweep_mode != 0u, D_GEN, fill);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    ui_list_row(y, "FM", fm_names[gen_fm_mode ? 1u : 0u], sel == 4u, 1, D_GEN);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    ui_list_row(y, "FM wave", fm_wave_names[gen_fm_source < 3u ? gen_fm_source : 0u], sel == 5u, gen_fm_mode != 0u, D_GEN);
    y = (uint16_t)(y + UI_LIST_ROW_H);
    fill = ui_list_row(y, "FM freq", 0, sel == 6u, gen_fm_mode != 0u, D_GEN);
    ui_list_digits(y, freq_clamp_display(gen_fm_freq_hz, fm_freq_unit),
                   (uint8_t)(ui_edit_digit_active && sel == 6u), ui_edit_digit_index,
                   freq_unit_labels[fm_freq_unit < 3u ? fm_freq_unit : 0u], sel == 6u, gen_fm_mode != 0u, D_GEN, fill);
}

static void draw_generator_immersive(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, D_BG);
    ui_draw_status_bar("Signal Generator", D_GEN);
    draw_gen_wave_card(8, 26, 226, 172);
    draw_gen_output_panel(240, 26, 72, 172, D_BG);
    lcd_rect(0, Y_SOFT, LCD_WIDTH, H_SOFT, D_BG);
    draw_gen_param_row(GEN_PARAM_ROW_X, (uint16_t)(Y_SOFT + 4u));
    ui_draw_gen_sweep_menu();
}

// ---------------------------------------------------------------------------
// Main menu: a 2x2 grid of tool cards in the multimeter screen's style.
// ---------------------------------------------------------------------------

typedef struct {
    const char *icon;
    const char *title;
    const char *subtitle;
    uint16_t accent;
    uint16_t tint;
} menu_card_t;

static const menu_card_t menu_cards[4] = {
    {ICON_MULTIMETER_MD, "Multimeter", "Volts, ohms, current", RGB565(90, 200, 250), RGB565(31, 58, 73)},
    {ICON_SCOPE_MD, "Oscilloscope", "2 channels, FFT, math", RGB565(52, 211, 153), RGB565(22, 56, 44)},
    {ICON_GENERATOR_MD, "Signal Generator", "Waves, sweep, CSV", RGB565(167, 139, 250), RGB565(49, 40, 82)},
    {ICON_SETTINGS_MD, "Settings", "Display, sound, storage", RGB565(255, 181, 71), RGB565(66, 48, 20)},
};

static void draw_mode_menu_card(uint8_t index) {
    enum { W = 148, H = 84, GAP = 8, X0 = 8, Y0 = 30 };
    const menu_card_t *card = &menu_cards[index];
    uint8_t selected = ui.menu_index == index;
    uint16_t x = (uint16_t)(X0 + (index % 2u) * (W + GAP));
    uint16_t y = (uint16_t)(Y0 + (index / 2u) * (H + GAP));
    uint16_t bg = selected ? RGB565(30, 37, 48) : D_CARD;

    if (selected) {
        gfx_round_rect(x, y, W, H, 14, card->accent, D_BG);
        gfx_round_rect((uint16_t)(x + 2u), (uint16_t)(y + 2u), W - 4u, H - 4u, 12, bg, card->accent);
    } else {
        gfx_round_rect(x, y, W, H, 14, bg, D_BG);
    }

    // Icon tile: tinted when idle, solid accent when selected.
    gfx_round_rect((uint16_t)(x + 12u), (uint16_t)(y + 11u), 36, 36, 10,
                   selected ? card->accent : card->tint, bg);
    gfx_text((int16_t)(x + 17u), (int16_t)(y + 16u), &font_icons, card->icon,
             selected ? D_BG : card->accent, selected ? card->accent : card->tint);


    gfx_text((int16_t)(x + 12u), (int16_t)(y + 48u), &font_medium, card->title, selected ? D_TEXT : D_SUB, bg);
    gfx_text((int16_t)(x + 12u), (int16_t)(y + 64u), &font_small, card->subtitle, D_MUTED, bg);
}

static void ui_draw_key_hints(uint16_t y, const char *move, const char *ok, const char *back);

static void draw_mode_menu(void) {
    enum { HINT_Y = 216 };

    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, D_BG);
    ui_draw_status_bar("Menu", D_TEXT);
    for (uint8_t i = 0; i < 4u; ++i) {
        draw_mode_menu_card(i);
    }

    ui_draw_key_hints(HINT_Y, "choose", "open", 0);
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
        uint8_t sleep = ui_settings.sleep_enabled < SETTINGS_SLEEP_COUNT ? ui_settings.sleep_enabled : 0u;
        return sleep_labels[sleep];
    }
    if (row == SETTINGS_BOOT_EXPORT_ROW) {
        return "EXPORT";
    }
    if (row == SETTINGS_ROW_RESET) {
        // Seven characters, so it is drawn at the same size as the values
        // on the tiles beside it; "DEFAULTS" was one too many and dropped
        // to half size, which made the tile look disabled.
        return "FACTORY";
    }
    return FIRMWARE_VERSION_TEXT;
}

// The revision decides which binary may be flashed, and until now the only
// way to tell was to open the case. HW4.0 drives a Gowin FPGA, everything
// before it an AGM part, and the two builds are not interchangeable.
static const char *hardware_revision_text(void) {
#if HW_TARGET_HW40
    return "HW4.0";
#else
    return "HW3.x";
#endif
}

// Free space on the capture drive, read once when the page opens. Walking
// the FAT takes a flash read and locks the host out of the drive for its
// duration, so it is not something to do on every frame.
static void settings_refresh_storage(void) {
    uint32_t free_kb = 0;
    uint32_t total_kb = 0;

    ui.storage_known = usb_msc_storage_free_kb(&free_kb, &total_kb);
    ui.storage_free_kb = free_kb;
    ui.storage_total_kb = total_kb;
}


enum {
    BOOT_EXPORT_UI_NONE,
    BOOT_EXPORT_UI_CONFIRM,
    BOOT_EXPORT_UI_SAVING,
    BOOT_EXPORT_UI_RESULT,
};

// ---------------------------------------------------------------------------
// Key hints and dialogs shared by the menu, settings and pop-ups.
// ---------------------------------------------------------------------------

static const uint16_t D_SETTINGS = RGB565(255, 181, 71);
static const uint16_t D_SETTINGS_TINT = RGB565(66, 48, 20);
static const uint16_t D_RED_TINT = RGB565(74, 24, 32);

// A key cap: a small rounded chip holding an icon or a word.
static int16_t ui_draw_keycap(int16_t x, int16_t y, const char *icon, const char *word, uint16_t under) {
    uint16_t inner = icon ? gfx_text_width(&font_icons, icon) : gfx_text_width(&font_small, word);
    uint16_t w = (uint16_t)(inner + 10u);
    gfx_round_rect((uint16_t)x, (uint16_t)y, w, 15, 5, D_TRACK, under);
    if (icon) {
        gfx_text((int16_t)(x + 5), (int16_t)(y + 2), &font_icons, icon, D_TEXT, D_TRACK);
    } else {
        gfx_text((int16_t)(x + 5), (int16_t)(y + 1), &font_small, word, D_TEXT, D_TRACK);
    }
    return (int16_t)(x + (int16_t)w);
}

static uint16_t ui_key_hints_width(const char *move, const char *ok, const char *back) {
    uint16_t w = (uint16_t)(gfx_text_width(&font_icons, ICON_MOVE_SM) + 10u + 5u + gfx_text_width(&font_small, move) +
                            14u + gfx_text_width(&font_icons, ICON_PLAYPAUSE_SM) + 10u + 5u +
                            gfx_text_width(&font_small, ok));
    if (back) {
        w = (uint16_t)(w + 14u + gfx_text_width(&font_small, "MENU") + 10u + 5u + gfx_text_width(&font_small, back));
    }
    return w;
}

// [arrows] move   [‖▶] ok   [MENU] back -- centred on the screen.
static void ui_draw_key_hints(uint16_t y, const char *move, const char *ok, const char *back) {
    int16_t x = (int16_t)((LCD_WIDTH - ui_key_hints_width(move, ok, back)) / 2u);
    x = ui_draw_keycap(x, (int16_t)y, ICON_MOVE_SM, 0, D_BG);
    x = gfx_text((int16_t)(x + 5), (int16_t)(y + 1), &font_small, move, D_MUTED, D_BG);
    x = ui_draw_keycap((int16_t)(x + 14), (int16_t)y, ICON_PLAYPAUSE_SM, 0, D_BG);
    x = gfx_text((int16_t)(x + 5), (int16_t)(y + 1), &font_small, ok, D_MUTED, D_BG);
    if (back) {
        x = ui_draw_keycap((int16_t)(x + 14), (int16_t)y, 0, "MENU", D_BG);
        gfx_text((int16_t)(x + 5), (int16_t)(y + 1), &font_small, back, D_MUTED, D_BG);
    }
}

enum {
    MODAL_X = 24,
    MODAL_Y = 60,
    MODAL_W = 272,
    MODAL_H = 120,
};

// A rounded dialog: icon + title, up to two lines, an optional progress bar
// and an optional "‖▶ confirm / MENU cancel" row.
static void ui_draw_modal(const char *icon, uint16_t tone, uint16_t tint,
                          const char *title, const char *line1, const char *line2,
                          int16_t progress, const char *confirm, const char *cancel) {
    enum { X = MODAL_X, Y = MODAL_Y, W = MODAL_W, H = MODAL_H };

    gfx_round_rect(X - 4u, Y - 4u, W + 8u, H + 8u, 20, D_BG, D_BG);
    gfx_round_rect(X, Y, W, H, 16, tone, D_BG);
    gfx_round_rect(X + 2u, Y + 2u, W - 4u, H - 4u, 14, D_CARD, tone);

    gfx_round_rect(X + 14u, Y + 14u, 30, 30, 9, tint, D_CARD);
    gfx_text((int16_t)(X + 14 + (30 - (int16_t)gfx_text_width(&font_icons, icon)) / 2),
             (int16_t)(Y + 14 + (30 - (int16_t)gfx_text_width(&font_icons, icon)) / 2),
             &font_icons, icon, tone, tint);
    gfx_text(X + 54, Y + 20, &font_medium, title, D_TEXT, D_CARD);

    if (line1) {
        gfx_text(X + 16, Y + 54, &font_small, line1, D_SUB, D_CARD);
    }
    if (line2) {
        gfx_text(X + 16, Y + 69, &font_small, line2, D_MUTED, D_CARD);
    }
    if (progress >= 0) {
        uint16_t len = (uint16_t)((uint32_t)(progress > 100 ? 100 : progress) * (W - 32u) / 100u);
        gfx_round_rect(X + 16u, Y + 90u, W - 32u, 6, 3, D_TRACK, D_CARD);
        if (len) {
            gfx_round_rect(X + 16u, Y + 90u, len, 6, len < 6u ? 0u : 3u, tone, D_TRACK);
        }
    }
    if (confirm) {
        int16_t x = X + 16;
        x = ui_draw_keycap(x, Y + 92, ICON_PLAYPAUSE_SM, 0, D_CARD);
        x = gfx_text((int16_t)(x + 5), Y + 93, &font_small, confirm, D_SUB, D_CARD);
        if (cancel) {
            x = ui_draw_keycap((int16_t)(x + 14), Y + 92, 0, "MENU", D_CARD);
            gfx_text((int16_t)(x + 5), Y + 93, &font_small, cancel, D_SUB, D_CARD);
        }
    }
}

static void draw_fw_update_overlay(uint16_t bg) {
    fw_update_status_t status;
    char line[32];
    char bytes[11];
    uint8_t error;

    (void)bg;
    fw_update_status(&status);
    if (status.state == FW_UPDATE_STATE_IDLE) {
        return;
    }
    error = status.state == FW_UPDATE_STATE_ERROR;
    scope_format_u32(bytes, status.bytes / 1024u);
    ui_text_copy(line, bytes, sizeof(line));
    ui_text_append(line, " KB received", sizeof(line));
    ui_draw_modal(error ? ICON_ERROR_MD : ICON_EXPORT_MD,
                  error ? D_RED : D_ACCENT, error ? D_RED_TINT : D_ACCENT_TINT,
                  "Firmware update", fw_update_status_text(&status), line, -1, 0, 0);
}

static void draw_screenshot_overlay(uint16_t bg) {
    const char *line1 = "Saving to USB drive";
    const char *line2 = 0;
    const char *icon = ICON_EXPORT_MD;
    uint16_t tone = D_ACCENT;
    uint16_t tint = D_ACCENT_TINT;

    (void)bg;
    if (!ui.screenshot_overlay_ms) {
        return;
    }
    if (ui.screenshot_state == SCREENSHOT_STATE_OK) {
        line1 = "Saved to USB drive";
        icon = ICON_OK_MD;
        tone = D_GREEN;
        tint = D_GREEN_TINT;
    } else if (ui.screenshot_state != SCREENSHOT_STATE_SAVING) {
        icon = ICON_ERROR_MD;
        tone = D_RED;
        tint = D_RED_TINT;
        line1 = "Save error";
        if (ui.screenshot_reason == SCREENSHOT_SAVE_FULL) {
            line1 = "Drive full";
            line2 = "Delete IMG files from the PC";
        } else if (ui.screenshot_reason == SCREENSHOT_SAVE_NO_SLOT) {
            line1 = "Too many files";
            line2 = "Delete IMG files from the PC";
        } else if (ui.screenshot_reason == SCREENSHOT_SAVE_BUSY) {
            line1 = "Drive busy";
            line2 = "The PC is reading it - try again";
        } else if (ui.screenshot_reason == SCREENSHOT_SAVE_NO_VOLUME) {
            line1 = "No USB drive";
        }
    }
    ui_draw_modal(icon, tone, tint, "Screenshot", line1, line2, -1, 0, 0);
}

static void draw_boot_export_overlay(void) {
    static const char *const errors[] = {
        "Saved and verified", "Storage busy - try again", "Bootloader is read-protected",
        "Invalid bootloader vectors", "Storage write error",
        "Not enough free space", "Backup verify error",
    };
    if (ui.overlay != UI_OVERLAY_SETTINGS || ui.boot_export_state == BOOT_EXPORT_UI_NONE) {
        return;
    }
    if (ui.boot_export_state == BOOT_EXPORT_UI_CONFIRM) {
        ui_draw_modal(ICON_EXPORT_MD, D_SETTINGS, D_SETTINGS_TINT, "Export bootloader",
                      "Eject the USB drive on the PC first.", "Replaces any existing backup.",
                      -1, "Export", "Cancel");
    } else if (ui.boot_export_state == BOOT_EXPORT_UI_SAVING) {
        char line[40];
        scope_format_u32(line, ui.boot_export_percent);
        ui_text_append(line, "% - reading and verifying", sizeof(line));
        ui_draw_modal(ICON_EXPORT_MD, D_SETTINGS, D_SETTINGS_TINT, "Export bootloader",
                      line, 0, ui.boot_export_percent, 0, 0);
    } else {
        uint8_t result = ui.boot_export_result;
        uint8_t error = result != BOOT_EXPORT_OK;
        ui_draw_modal(error ? ICON_ERROR_MD : ICON_OK_MD, error ? D_RED : D_GREEN, error ? D_RED_TINT : D_GREEN_TINT,
                      "Export bootloader",
                      errors[result <= BOOT_EXPORT_VERIFY_ERROR ? result : BOOT_EXPORT_STORAGE_ERROR],
                      error ? 0 : BOOTLOADER_FILE_NAME, -1, 0, 0);
    }
}

static void draw_settings_reset_overlay(void) {
    if (ui.overlay != UI_OVERLAY_SETTINGS || !ui.reset_confirm) {
        return;
    }
    ui_draw_modal(ICON_RESET_MD, D_RED, D_RED_TINT, "Factory reset",
                  "All settings and calibration will be lost.", "Screenshots and captures are kept.",
                  -1, "Reset", "Cancel");
}

// ---------------------------------------------------------------------------
// Settings page: 3x2 tiles plus a wide info card, same navigation as before.
// ---------------------------------------------------------------------------

static const char *const settings_tile_labels[] = {
    "Beep", "Display", "Start", "Sleep", "Bootloader", "Reset",
};
static const char *const settings_tile_icons[] = {
    ICON_BEEP_MD, ICON_DISPLAY_MD, ICON_START_MD, ICON_SLEEP_MD, ICON_EXPORT_MD, ICON_RESET_MD,
};

// Sentence case for the upper-case setting values: "MEDIUM" -> "Medium".
static void settings_display_value(uint8_t row, char out[16]) {
    char scratch[12];
    const char *raw;

    if (row == 2u) {
        static const char *const starts[] = {"Menu", "Multimeter", "Scope", "Generator"};
        const char *label = startup_label();
        for (uint8_t i = 0; i < 4u; ++i) {
            if (ui_text_equal(label, startup_labels[i])) {
                ui_text_copy(out, starts[i], 16);
                return;
            }
        }
    }
    if (row == SETTINGS_BOOT_EXPORT_ROW) {
        ui_text_copy(out, "Export", 16);
        return;
    }
    if (row == SETTINGS_ROW_RESET) {
        ui_text_copy(out, "Factory", 16);
        return;
    }
    raw = settings_value_text(row, scratch);
    ui_text_copy(out, raw, 16);
    for (uint8_t i = 1; out[i]; ++i) {
        if (out[i] >= 'A' && out[i] <= 'Z') {
            out[i] = (char)(out[i] + 32);
        }
    }
}

static void draw_settings_tile(uint8_t row, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    char value[16];
    uint8_t selected = ui.settings_row == row;
    uint8_t danger = row == SETTINGS_ROW_RESET;
    uint16_t accent = danger ? D_RED : D_SETTINGS;
    uint16_t tint = danger ? D_RED_TINT : D_SETTINGS_TINT;
    uint16_t bg = selected ? RGB565(30, 37, 48) : D_CARD;

    if (selected) {
        gfx_round_rect(x, y, w, h, 12, accent, D_BG);
        gfx_round_rect((uint16_t)(x + 2u), (uint16_t)(y + 2u), (uint16_t)(w - 4u), (uint16_t)(h - 4u), 10, bg, accent);
    } else {
        gfx_round_rect(x, y, w, h, 12, bg, D_BG);
    }
    gfx_round_rect((uint16_t)(x + 9u), (uint16_t)(y + 9u), 24, 24, 7, selected ? accent : tint, bg);
    gfx_text((int16_t)(x + 13u), (int16_t)(y + 13u), &font_icons, settings_tile_icons[row],
             selected ? D_BG : accent, selected ? accent : tint);
    gfx_text((int16_t)(x + 39u), (int16_t)(y + 14u), &font_small, settings_tile_labels[row], D_MUTED, bg);

    settings_display_value(row, value);
    gfx_text((int16_t)(x + 10u), (int16_t)(y + 35u), &font_medium, value, selected ? accent : D_TEXT, bg);

    // Level rows get a step meter beside the value: one bar per level above
    // the lowest setting for beep (OFF has none), one per setting for
    // brightness (DIM is already one step of light).
    if (row == 0u || row == 1u) {
        uint8_t bars = row == 0u ? (uint8_t)(SETTINGS_LEVEL_COUNT - 1u) : (uint8_t)SETTINGS_LEVEL_COUNT;
        uint8_t level = row == 0u ? ui_settings.beep_level : (uint8_t)(ui_settings.brightness_level + 1u);
        for (uint8_t i = 0; i < bars; ++i) {
            uint16_t bx = (uint16_t)(x + w - 12u - (uint16_t)(bars - 1u - i) * 6u - 4u);
            uint16_t bh = (uint16_t)(4u + i * 2u);
            gfx_round_rect(bx, (uint16_t)(y + 48u - bh), 4, bh, 1, i < level ? accent : D_TRACK, bg);
        }
    }
}

static void draw_settings_info(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    char storage[20];
    int16_t right;
    int16_t tx;

    gfx_round_rect(x, y, w, h, 12, D_CARD, D_BG);
    gfx_round_rect((uint16_t)(x + 9u), (uint16_t)(y + (h - 24u) / 2u), 24, 24, 7, D_TRACK, D_CARD);
    gfx_text((int16_t)(x + 13u), (int16_t)(y + (h - 24u) / 2u + 4u), &font_icons, ICON_INFO_MD, D_SUB, D_TRACK);
    gfx_text((int16_t)(x + 42u), (int16_t)(y + 8u), &font_small, "Firmware", D_MUTED, D_CARD);
    tx = gfx_text((int16_t)(x + 42u), (int16_t)(y + 21u), &font_medium, FIRMWARE_VERSION_TEXT, D_TEXT, D_CARD);
    {
        int16_t left;
        uint16_t pw = (uint16_t)(gfx_text_width(&font_small, hardware_revision_text()) + 12u);
        dmm_draw_pill((int16_t)(tx + 8 + (int16_t)pw), (uint16_t)(y + 22u), hardware_revision_text(),
                      D_SETTINGS_TINT, D_SETTINGS, D_CARD, &left);
    }

    if (!ui.storage_known || !ui.storage_total_kb) {
        ui_text_copy(storage, "Storage --", sizeof(storage));
    } else {
        char number[12];
        scope_format_u32(number, ui.storage_free_kb / 1024u);
        ui_text_copy(storage, number, sizeof(storage));
        ui_text_append(storage, ".", sizeof(storage));
        scope_format_u32(number, ((ui.storage_free_kb % 1024u) * 10u) / 1024u);
        ui_text_append(storage, number, sizeof(storage));
        ui_text_append(storage, " MB free", sizeof(storage));
    }
    right = (int16_t)(x + w - 12u);
    gfx_text_right(right, (int16_t)(y + 21u), &font_small, storage, D_SUB, D_CARD);
    gfx_text((int16_t)(right - (int16_t)gfx_text_width(&font_small, storage) - 16), (int16_t)(y + 20u),
             &font_icons, ICON_STORAGE_SM, D_MUTED, D_CARD);
    gfx_text_right(right, (int16_t)(y + 8u), &font_small, "USB drive", D_MUTED, D_CARD);
}

static void draw_settings_menu(void) {
    lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, D_BG);
    ui_draw_status_bar("Settings", D_SETTINGS);
    for (uint8_t row = 0; row < SETTINGS_ROW_INFO; ++row) {
        uint8_t col = (uint8_t)(row % SETTINGS_GRID_COLUMNS);
        uint8_t line = (uint8_t)(row / SETTINGS_GRID_COLUMNS);
        draw_settings_tile(row, (uint16_t)(8u + col * 103u), (uint16_t)(28u + line * 60u), 98, 54);
    }
    draw_settings_info(8, 148, 304, 50);
    ui_draw_key_hints(214, "move",
                      ui.settings_row >= SETTINGS_BOOT_EXPORT_ROW ? "open" : "change", "back");
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

// The renderer only pushes to the glass the rectangles something drew
// into, which is what keeps it fast. The cost is that a screen drawing
// less than the one before it leaves the difference behind: switching
// modes could show the scope's title over the multimeter's reading over
// the scope's bottom bar, all at once. These two say when a full render
// is a different screen from the last one and so has to start from a
// clean background.
static uint8_t last_full_scene_key = 0xFF;
static uint8_t full_scene_clear;

static uint8_t full_scene_key(void) {
    // The overlays paint their own background over everything, so they
    // need no clear of their own -- but they do make the next mode screen
    // need one, which is why they take a key at all.
    if (ui.overlay != UI_OVERLAY_NONE) {
        return (uint8_t)(0x80u | (uint8_t)ui.overlay);
    }
    return (uint8_t)(((uint8_t)ui.mode << 1) | (ui.chrome_visible ? 1u : 0u));
}

// Every tool draws its own full-screen view; the old header-and-tabs
// layout only ever showed behind the menu and has been retired.
static void ui_draw_mode_screen(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        draw_scope_immersive();
    } else if (ui.mode == UI_MODE_GEN) {
        draw_generator_immersive();
    } else {
        draw_dmm_immersive();
    }
}

static void ui_draw_full_scene(void) {
    if (full_scene_clear) {
        lcd_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG);
    }
    ui_draw_mode_screen();
}

static void ui_draw_mode_content_scene(void) {
    ui_draw_mode_screen();
}

static void ui_draw_local_change_scene(void) {
    if (ui.mode != UI_MODE_DMM) {
        ui_draw_mode_content_scene();
        return;
    }

    draw_dmm_immersive_panel();
    draw_dmm_softkeys();
}

static void ui_draw_dmm_reading_scene(void) {
    if (ui.mode != UI_MODE_DMM) {
        ui_draw_mode_content_scene();
        return;
    }

    draw_dmm_immersive_panel();
}

static void ui_draw_battery_status_scene(void) {
    if (ui.overlay == UI_OVERLAY_MODE_MENU) {
        ui_draw_status_bar("Menu", D_TEXT);
    } else if (ui.overlay == UI_OVERLAY_SETTINGS) {
        ui_draw_status_bar("Settings", D_SETTINGS);
    } else if (ui.mode == UI_MODE_DMM && ui.overlay == UI_OVERLAY_NONE) {
        dmm_draw_status_bar();
    } else if (ui.mode == UI_MODE_SCOPE) {
        ui_draw_status_bar("Oscilloscope", D_SCOPE);
    } else {
        ui_draw_status_bar("Signal Generator", D_GEN);
    }
}

static void ui_draw_scope_frame_scene(void) {
    if (ui.mode != UI_MODE_SCOPE) {
        ui_draw_mode_content_scene();
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
    draw_gen_wave_card(8, 26, 226, 172);
    draw_gen_output_panel(240, 26, 72, 172, D_BG);
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
        draw_boot_export_overlay();
        draw_settings_reset_overlay();
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
    // Decided before the render and held for its duration: lcd_render()
    // runs the callback twice, once to collect the rectangles and once to
    // fill them, and the two passes have to agree on what they are drawing.
    full_scene_clear = 0;
    if (job == UI_RENDER_FULL) {
        uint8_t key = full_scene_key();
        if (key != last_full_scene_key) {
            full_scene_clear = 1;
            last_full_scene_key = key;
        }
    }
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

// Leaving the multimeter: gen_stop() silenced the generator on the way in,
// but the user never asked for that. Restore the output so it does not stay
// off without any indication. The stock firmware keeps it running throughout.
static void gen_resume_after_dmm(ui_mode_t old_mode) {
    if (old_mode != UI_MODE_DMM || !gen_user_enabled || gen_running) {
        return;
    }
    gen_running = 1;
    gen_apply();
}

static void ui_switch_mode(ui_mode_t mode) {
    ui_mode_t old_mode = ui.mode;

    if (ui.mode != mode) {
        dmm_stats_reset();
    }
    if (old_mode == UI_MODE_GEN && mode != UI_MODE_GEN) {
        gen_running = ui.running ? 1u : 0u;
        gen_user_enabled = gen_running;
        if (gen_running || gen_output_applied || gen_deferred_apply) {
            gen_apply();
        } else {
            gen_deferred_apply = 0;
            gen_deferred_apply_ms = 0;
        }
    }
    if (old_mode == UI_MODE_SCOPE && mode != UI_MODE_SCOPE) {
        scope_hw_slow_stop();
        if (scope_fft_src == 4u) {
            bode_shutdown(0);
        }
    }
    if (mode != UI_MODE_DMM) {
        dmm_live_wire_alert = 0;
        dmm_pause();
        dmm_hw4_set_mode_gate(0);
    }
        if (mode == UI_MODE_SCOPE) {
        usb_cdc_send_bytes((const uint8_t *)"$MODE,SCOPE\r\n", 13);
    } else if (mode == UI_MODE_DMM) {
        usb_cdc_send_bytes((const uint8_t *)"$MODE,DMM\r\n", 11);
    } else if (mode == UI_MODE_GEN) {
        usb_cdc_send_bytes((const uint8_t *)"$MODE,GEN\r\n", 11);
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
        ui.scope_param = SCOPE_PARAM_POSITION;
        scope_auto_cancel();
        scope_sanitize_state();
        gen_resume_after_dmm(old_mode);
        ui.running = 1;
        ui.scope_ms = 0;
        scope_frame_valid = 0;
#if !SCOPE_UI_SAFE_STUB
        scope_apply_settings();
        if (scope_fft_src == 4u) {
            bode_begin_sweep();
        }
#else
        scope_trace_cache[0].valid = 0;
        scope_trace_cache[1].valid = 0;
#endif
    } else if (mode == UI_MODE_GEN) {
        gen_resume_after_dmm(old_mode);
        ui.running = gen_running;
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
        if (old_mode == UI_MODE_DMM) {
            dmm_set_mode(DMM_MODE_AUTO);
        } else {
            gen_stop();
            dmm_reenter(DMM_MODE_AUTO);
        }
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
    scope_auto_cancel();
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
    ui_apply_saved_runtime_settings();
    scope_math_mode = ui_settings.scope_math_mode;
    scope_math_op = ui_settings.scope_math_op;
    scope_fft_src = ui_settings.scope_fft_src;
    scope_math_selected = ui_settings.scope_math_selected;
    scope_fft_window = ui_settings.scope_fft_window;
    scope_fft_display = ui_settings.scope_fft_display;
    scope_hide_traces = ui_settings.scope_hide_traces;
    bode_start_hz = ui_settings.bode_start_hz;
    bode_stop_hz = ui_settings.bode_stop_hz;
    // Bode units are not stored; pick the smallest unit that fits.
    freq_sync_display_unit(&bode_start_unit, bode_start_hz);
    freq_sync_display_unit(&bode_stop_unit, bode_stop_hz);
    bode_steps = bode_clamp_steps(ui_settings.bode_steps);
    ui.dmm_mode = dmm_sanitize_mode(ui_settings.dmm_mode);
    if (start_mode == UI_MODE_DMM) {
        ui.dmm_mode = DMM_MODE_AUTO;
    }
    ui.auto_range = ui.dmm_mode == DMM_MODE_AUTO ? 1u : 0u;
    ui_settings.dmm_mode = dmm_wire_mode_for_ui(ui.dmm_mode);
    board_buzzer_set_volume(beep_level_percent[ui_settings.beep_level]);
    board_backlight_set_level(brightness_level_percent[ui_settings.brightness_level]);
    dmm_set_mode(dmm_wire_mode_for_ui(ui.dmm_mode));
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
    if (ui.gen_param == GEN_PARAM_FREQ && ui.gen_wave != SIGGEN_WAVE_ARBITRARY) {
        gen_freq_sync_editor_unit();
    }
}

static void gen_select_param(uint8_t param) {
    if (param < GEN_PARAM_COUNT && gen_param_available(param)) {
        ui.gen_param = param;
    }
    gen_freq_edit_pos = 0;
    if (ui.gen_param == GEN_PARAM_FREQ && ui.gen_wave != SIGGEN_WAVE_ARBITRARY) {
        gen_freq_sync_editor_unit();
    }
}

static uint8_t gen_param_edit_last_pos(uint8_t param) {
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        return 0;
    }
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
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        return 0;
    }
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
        if (ui.gen_param == GEN_PARAM_FREQ && ui.gen_wave != SIGGEN_WAVE_ARBITRARY) {
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

// Set from the USB command handler, acted on in ui_tick(). The handler
// already runs in the main loop, but it runs *inside* usb_msc_poll(), and
// a capture re-enters usb_msc to take the storage and walk the FAT. Doing
// that from within the poll would nest the storage state machine inside
// itself; ui_tick() is the next moment where nothing else holds it.
static uint8_t screenshot_requested;

// Same reasoning for injected key events: ui_handle_keys() can reach the
// capture too, by way of a long press on SAVE.
static uint32_t keys_injected;

static void capture_screenshot_now(void) {
    ui.screenshot_state = SCREENSHOT_STATE_SAVING;
    ui.screenshot_overlay_ms = SCREENSHOT_OVERLAY_MS;
    draw_screenshot_overlay(C_BG);

    screenshot_save_t saved = usb_msc_save_screenshot();

    ui.screenshot_reason = (uint8_t)saved;
    ui.screenshot_state = saved == SCREENSHOT_SAVE_OK ? SCREENSHOT_STATE_OK : SCREENSHOT_STATE_ERROR;
    ui.screenshot_overlay_ms = SCREENSHOT_OVERLAY_MS;
    // The drive just changed. The settings page caches what it last read,
    // so re-read it now rather than leave it showing "--" until the page
    // is closed and opened again.
    ui.storage_known = 0;
    if (ui.overlay == UI_OVERLAY_SETTINGS) {
        settings_refresh_storage();
    }
    ui_render();

    // A host that asked for the capture has no other way of knowing the file
    // was written: the drive is not mounted while the device owns it.
    if (usb_cdc_is_configured()) {
        // The reason is appended after ERR rather than replacing it, so a
        // host that only looks for "$SHOT,ERR" keeps working.
        if (saved == SCREENSHOT_SAVE_OK) {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,OK\r\n", 10);
        } else if (saved == SCREENSHOT_SAVE_FULL) {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,ERR,FULL\r\n", 16);
        } else if (saved == SCREENSHOT_SAVE_NO_SLOT) {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,ERR,DIRFULL\r\n", 19);
        } else if (saved == SCREENSHOT_SAVE_BUSY) {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,ERR,BUSY\r\n", 16);
        } else if (saved == SCREENSHOT_SAVE_NO_VOLUME) {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,ERR,NOFS\r\n", 16);
        } else {
            usb_cdc_send_bytes((const uint8_t *)"$SHOT,ERR\r\n", 11);
        }
    }
}

static void dmm_pause_for_menu_overlay(void) {
    if (ui.mode != UI_MODE_DMM) {
        return;
    }

    dmm_live_wire_alert = 0;
    dmm_pause();
}

static void dmm_apply_selected_mode(uint8_t mode) {
    uint8_t choice;
    uint8_t group = dmm_mode_group(mode, &choice);
    uint8_t scale_only = ui.mode == UI_MODE_DMM && ui.overlay == UI_OVERLAY_NONE &&
                         mode != ui.dmm_mode && dmm_is_temperature(mode) && dmm_is_temperature(ui.dmm_mode);
    if (group < DMM_GROUP_COUNT) dmm_group_choice[group] = choice;
    ui.dmm_mode = mode;
    ui.auto_range = mode == DMM_MODE_AUTO;
    dmm_hold_active = 0;
    dmm_rel_active = 0;
    dmm_live_wire_alert = 0;
    dmm_stats_reset();
    ui_settings.dmm_mode = dmm_wire_mode_for_ui(mode);
    settings_note(&ui_settings);
    if (scale_only) {
        dmm_stats_update();
    } else {
        dmm_set_mode(ui_settings.dmm_mode);
    }
    extern void usb_cdc_send_dmm(const char *mode, const char *val, const char *unit, uint8_t valid);
    usb_cdc_send_dmm(dmm_mode_name(), dmm_value_text(), dmm_unit_text(), dmm_has_reading());
}

void ui_note_runtime_settings(void) {
    scope_sanitize_state();
    gen_prepare_state();

    ui_settings.dmm_mode = dmm_wire_mode_for_ui(ui.dmm_mode);
    ui_settings.scope_timebase = scope_safe_timebase();
    ui_settings.scope_display = scope_safe_display();
    ui_settings.scope_cursor_mode = scope_safe_cursor();
    ui_settings.scope_trigger_source = ui.scope_trigger_source;
    ui_settings.scope_trigger_mode = ui.scope_trigger_mode;
    ui_settings.scope_trigger_edge = ui.scope_trigger_edge ? 1u : 0u;
    ui_settings.scope_trigger_level = ui.scope_trigger_level;
    ui_settings.scope_afterglow = ui.scope_afterglow ? 1u : 0u;
    ui_settings.scope_active_ch = ui.active_ch;
    ui_settings.scope_measure_param = scope_measure_param;
    ui_settings.scope_measure_visible = scope_measure_visible ? 1u : 0u;
    ui_settings.scope_h_value_pos = scope_h_value_pos;
    ui_settings.scope_h_value_timebase = scope_h_value_timebase;
    ui_settings.scope_math_mode = scope_math_mode;
    ui_settings.scope_math_op = scope_math_op;
    ui_settings.scope_fft_src = scope_fft_src;
    ui_settings.scope_math_selected = scope_math_selected;
    ui_settings.scope_fft_window = scope_fft_window;
    ui_settings.scope_fft_display = scope_fft_display;
    ui_settings.scope_hide_traces = scope_hide_traces;
    ui_settings.bode_start_hz = bode_start_hz;
    ui_settings.bode_stop_hz = bode_stop_hz;
    ui_settings.bode_steps = bode_steps;

    for (uint8_t ch = 0; ch < 2u; ++ch) {
        ui_settings.scope_ch_enabled[ch] = scope_ch_enabled[ch] ? 1u : 0u;
        ui_settings.scope_probe_x10[ch] = scope_probe_x10[ch] ? 1u : 0u;
        ui_settings.scope_vdiv[ch] = scope_vdiv_ch[ch];
        ui_settings.scope_coupling_dc[ch] = scope_coupling_dc[ch] ? 1u : 0u;
        ui_settings.scope_ch_pos[ch] = scope_ch_pos[ch];
        ui_settings.scope_measure_mask[ch] = (uint8_t)(scope_measure_mask[ch] & scope_measure_valid_mask());
    }

    ui_settings.siggen_wave = ui.gen_wave;
    ui_settings.siggen_param = ui.gen_param;
    ui_settings.siggen_duty_percent = ui.gen_duty_percent;
    ui_settings.siggen_amp_tenths_v = ui.gen_amp_tenths_v;
    ui_settings.siggen_freq_unit = gen_freq_unit;
    ui_settings.siggen_running = 0;
    ui_settings.siggen_freq_hz = ui.gen_freq_hz;
    ui_settings.siggen_sweep_mode = gen_sweep_mode;
    ui_settings.siggen_sweep_ms = gen_sweep_ms;
    ui_settings.siggen_sweep_start_hz = siggen_sweep_start_hz;
    ui_settings.siggen_sweep_stop_hz = siggen_sweep_stop_hz;
    ui_settings.siggen_fm_mode = gen_fm_mode;
    ui_settings.siggen_fm_source = gen_fm_source;
    ui_settings.siggen_fm_freq_hz = gen_fm_freq_hz;
    settings_note(&ui_settings);
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

static uint32_t gen_get_current_sweep_freq(void) {
    if (gen_sweep_mode != 0 && gen_sweep_ms != 0) {
        uint32_t start = siggen_sweep_start_hz;
        uint32_t stop = siggen_sweep_stop_hz;
        uint32_t duration = gen_sweep_ms;
        float progress;

        if (gen_sweep_elapsed_ms >= duration) {
            return stop;
        }
        progress = (float)gen_sweep_elapsed_ms / (float)duration;

        if (gen_sweep_mode == 1) {
            float value = (float)start + ((float)stop - (float)start) * progress;
            return (uint32_t)(value + 0.5f);
        }
        if (gen_sweep_mode == 2) {
            return log_interpolate_u32(start, stop, progress);
        }
    }

    if (gen_fm_mode != 0 && gen_fm_freq_hz != 0) {
        uint32_t carrier = ui.gen_freq_hz;
        uint32_t deviation = carrier >> 2;
        if (deviation > 1000u) deviation = 1000u;
        if (deviation == 0)    deviation = 1u;
        int32_t offset = 0;

        if (gen_fm_source == 0) {
            uint8_t index = (uint8_t)(gen_fm_phase >> 10);
            uint8_t next = (uint8_t)((index + 1u) & 63u);
            uint16_t frac = (uint16_t)(gen_fm_phase & 1023u);
            int32_t sine = (int32_t)scope_sine_lut[index] * (1024 - frac) +
                           (int32_t)scope_sine_lut[next] * frac;
            sine = (sine + (sine >= 0 ? 512 : -512)) / 1024;
            offset = sine * (int32_t)deviation / 64;
        } else if (gen_fm_source == 1) {
            int32_t triangle = gen_fm_phase < 32768u ?
                (int32_t)gen_fm_phase * 2 - 32768 :
                98303 - (int32_t)gen_fm_phase * 2;
            offset = triangle * (int32_t)deviation / 32768;
        } else if (gen_fm_source == 2) {
            offset = gen_fm_phase < 32768u ?
                (int32_t)deviation : -(int32_t)deviation;
        }

        int32_t final_freq = (int32_t)carrier + offset;
        if (final_freq < 1)    return 1u;
        if (final_freq > (int32_t)GEN_MAX_FREQ_HZ) return GEN_MAX_FREQ_HZ;
        return (uint32_t)final_freq;
    }

    return ui.gen_freq_hz;
}

static void gen_apply(void) {
    gen_prepare_state();
    if (bode_siggen_active) {
        return;
    }
    if (!gen_running && !gen_output_applied) {
        gen_deferred_apply = 0;
        gen_deferred_apply_ms = 0;
        return;
    }
    uint32_t target_freq = (gen_running && (gen_sweep_mode != 0 || gen_fm_mode != 0)) ? 
                           gen_get_current_sweep_freq() : ui.gen_freq_hz;
    siggen_configure(gen_running,
                     ui.gen_wave,
                     target_freq,
                     ui.gen_duty_percent,
                     ui.gen_amp_tenths_v);
    gen_output_applied = gen_running ? 1u : 0u;
    gen_deferred_apply = 0;
    gen_deferred_apply_ms = 0;
}

static void gen_stop(void) {
    gen_running = 0;
    gen_output_applied = 0;
    gen_deferred_apply = 0;
    gen_deferred_apply_ms = 0;
    gen_sweep_elapsed_ms = 0;
    gen_fm_phase = 0;
    siggen_shutdown();
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
    dmm_hold_range_decimals = dmm_text_decimals(dmm_value_text());
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
    dmm_rel_ref_fixed = dmm_display_fixed_value();
    ui_text_copy(dmm_rel_ref_value, dmm_live_display_value(), sizeof(dmm_rel_ref_value));
    ui_text_copy(dmm_rel_unit, dmm_live_display_unit(), sizeof(dmm_rel_unit));
    format_signed_fixed(0, dmm_rel_value);
    dmm_rel_active = 1;
    dmm_stats_reset();
}

static void dmm_step_group(uint8_t group, int8_t dir) {
    uint8_t choice;
    if (dmm_mode_group(ui.dmm_mode, &choice) == group) {
        cycle_u8(&choice, dmm_groups[group].count, dir);
    } else {
        choice = dmm_group_choice[group];
    }
    dmm_apply_selected_mode(dmm_groups[group].modes[choice]);
}

static void scope_cycle_display(int8_t dir) {
    cycle_u8(&ui.scope_display, SCOPE_DISPLAY_COUNT, dir);
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_trace_invalidate();
}

static void scope_cycle_cursor(void) {
    if (scope_fft_src == 4u) {
        if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
            ui.scope_cursor_mode = SCOPE_CURSOR_BODE;
            bode_cursor_sub = 0;
        } else if (bode_cursor_sub == 0u) {
            bode_cursor_sub = 1;
        } else if (bode_cursor_sub == 1u) {
            bode_cursor_sub = 2;
        } else {
            ui.scope_cursor_mode = SCOPE_CURSOR_OFF;
        }
    } else if (scope_fft_src == 1u || scope_fft_src == 2u) {
        if (ui.scope_cursor_mode == SCOPE_CURSOR_OFF) {
            ui.scope_cursor_mode = SCOPE_CURSOR_TIME;
        } else {
            ui.scope_cursor_mode = SCOPE_CURSOR_OFF;
        }
    } else {
        cycle_u8(&ui.scope_cursor_mode, SCOPE_CURSOR_COUNT, 1);
    }
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

static void ui_apply_saved_runtime_settings(void) {
    ui.scope_timebase = ui_settings.scope_timebase;
    ui.scope_display = ui_settings.scope_display;
    ui.scope_cursor_mode = ui_settings.scope_cursor_mode;
    ui.scope_trigger_source = ui_settings.scope_trigger_source;
    ui.scope_trigger_mode = ui_settings.scope_trigger_mode;
    ui.scope_trigger_edge = ui_settings.scope_trigger_edge;
    ui.scope_trigger_level = ui_settings.scope_trigger_level;
    ui.scope_afterglow = ui_settings.scope_afterglow;
    ui.active_ch = ui_settings.scope_active_ch;
    scope_measure_param = ui_settings.scope_measure_param;
    scope_measure_visible = ui_settings.scope_measure_visible;
    scope_h_value_pos = ui_settings.scope_h_value_pos;
    scope_h_value_timebase = ui_settings.scope_h_value_timebase;

    for (uint8_t ch = 0; ch < 2u; ++ch) {
        scope_ch_enabled[ch] = ui_settings.scope_ch_enabled[ch];
        scope_probe_x10[ch] = ui_settings.scope_probe_x10[ch];
        scope_vdiv_ch[ch] = ui_settings.scope_vdiv[ch];
        scope_coupling_dc[ch] = ui_settings.scope_coupling_dc[ch];
        scope_ch_pos[ch] = ui_settings.scope_ch_pos[ch];
        scope_measure_mask[ch] = ui_settings.scope_measure_mask[ch];
    }

    ui.gen_wave = ui_settings.siggen_wave;
    ui.gen_param = ui_settings.siggen_param;
    ui.gen_duty_percent = ui_settings.siggen_duty_percent;
    ui.gen_amp_tenths_v = ui_settings.siggen_amp_tenths_v;
    ui.gen_freq_hz = ui_settings.siggen_freq_hz;
    gen_freq_unit = ui_settings.siggen_freq_unit;
    gen_sweep_mode = ui_settings.siggen_sweep_mode;
    gen_sweep_ms = (uint16_t)ui_settings.siggen_sweep_ms;
    siggen_sweep_start_hz = ui_settings.siggen_sweep_start_hz;
    siggen_sweep_stop_hz = ui_settings.siggen_sweep_stop_hz;
    // Sweep units are not stored; pick the smallest unit that fits.
    sweep_start_unit = 0;
    sweep_stop_unit = 0;
    freq_sync_display_unit(&sweep_start_unit, siggen_sweep_start_hz);
    freq_sync_display_unit(&sweep_stop_unit, siggen_sweep_stop_hz);
    gen_fm_mode = ui_settings.siggen_fm_mode;
    gen_fm_source = ui_settings.siggen_fm_source;
    gen_fm_freq_hz = ui_settings.siggen_fm_freq_hz;
    if (gen_sweep_mode && gen_fm_mode) {
        gen_fm_mode = 0;
    }
    if (gen_fm_freq_hz < 1u || gen_fm_freq_hz > SETTINGS_SIGGEN_FM_MAX_HZ) {
        gen_fm_freq_hz = 5u;
    }
    fm_freq_unit = 0;
    gen_running = 0;
    gen_prepare_state();
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        ui.gen_param = GEN_PARAM_FREQ;
        arb_scan_files();
        if (arb_file_index >= arb_file_count() && arb_file_count() > 0) {
            arb_file_index = 0;
        }
        arb_load_waveform();
    }

    scope_sanitize_state();
    scope_h_pos = scope_h_pos_from_value();
    scope_cursor_time_sync_screen();
    scope_cursor_level_sync_screen();
}

static void scope_step_timebase(int8_t dir) {
    scope_auto_cancel();
    cycle_u8(&ui.scope_timebase, SCOPE_TIMEBASE_COUNT, dir);
    scope_h_pos = scope_h_pos_from_value();
    scope_cursor_time_sync_screen();
    scope_apply_settings();
}

static void scope_adjust_cursor_steps(int8_t dir, uint8_t step) {
    uint8_t *value;
    if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        value = (bode_cursor_sub == 0u) ?
            &scope_cursor_x[ui.scope_cursor_sel] :
            &scope_cursor_y[ui.scope_cursor_sel];
    } else {
        value = ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL ?
            &scope_cursor_y[ui.scope_cursor_sel] :
            &scope_cursor_x[ui.scope_cursor_sel];
    }
    int16_t next = (int16_t)*value + (int16_t)(dir * (int8_t)step);

    if (next < 0) {
        next = 0;
    } else if (next > 255) {
        next = 255;
    }
    *value = (uint8_t)next;
    if (ui.scope_cursor_mode == SCOPE_CURSOR_LEVEL) {
        scope_cursor_store_level_from_raw(ui.scope_cursor_sel);
    } else if (ui.scope_cursor_mode != SCOPE_CURSOR_BODE || scope_fft_src != 4u) {
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
    if (scope_fft_src == 4u && ui.scope_cursor_mode == SCOPE_CURSOR_BODE) {
        if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_FIRST) {
            ui.scope_cursor_sel = 0;
        } else if (ui.scope_cursor_menu_sel == SCOPE_CURSOR_MENU_SECOND) {
            ui.scope_cursor_sel = 1;
        }
        if (bode_cursor_sub != 0u) {
            dir = (int8_t)-dir;
        }
        scope_adjust_cursor_steps(dir, step);
        return;
    }

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
    scope_auto_cancel();
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
    scope_auto_cancel();
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

static void scope_auto_cancel(void) {
    scope_auto_time_active = 0;
    scope_auto_time_mask = 0;
    scope_auto_time_steps_left = 0;
    scope_auto_channel_mask = 0;
    scope_auto_last_frame = scope_frame_id;
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        scope_auto_channel_steps_left[ch] = 0;
        scope_auto_channel_stable_count[ch] = 0;
    }
}

static uint32_t scope_auto_frame_timeout_ms(void) {
    uint32_t points = scope_slow_roll_active() ? 1u : SCOPE_SAMPLE_COUNT;
    uint32_t period_ns = scope_sample_period_ns();
    // Split before multiplying: at most 2048 samples, no 64-bit division runtime.
    uint32_t capture_ms = (period_ns / 1000000u) * points +
        ((period_ns % 1000000u) * points + 999999u) / 1000000u;
    uint32_t timeout_ms = capture_ms * 3u + 1000u;

    return timeout_ms < 60000u ? timeout_ms : 60000u;
}

static void scope_auto_timeout_service(void) {
    // Frame age uses UI elapsed ticks, not a wall clock; allow several captures.
    if ((scope_auto_time_active || scope_auto_channel_mask) &&
        scope_frame_age_ms >= scope_auto_frame_timeout_ms()) {
        scope_auto_cancel();
    }
}

static uint8_t scope_auto_enabled_mask(uint8_t mask) {
    return (uint8_t)(mask & ((scope_ch_enabled[0] ? 0x01u : 0u) |
                             (scope_ch_enabled[1] ? 0x02u : 0u)));
}

static void scope_auto_seed_ranges(uint8_t mask) {
    // Stock 2.0.2 starts at 50 mV/div, then only widens the input range.
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        if (mask & (uint8_t)(1u << ch)) {
            scope_vdiv_ch[ch] = SCOPE_AUTO_VDIV_START;
            scope_auto_channel_steps_left[ch] = SCOPE_VDIV_COUNT - 1u - SCOPE_AUTO_VDIV_START;
        }
    }
    ui.scope_vdiv = scope_safe_vdiv();
    scope_cursor_level_sync_screen();
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

static uint8_t scope_raw_crossed_level(uint8_t prev, uint8_t cur, uint8_t level, uint8_t falling) {
    if (falling) {
        return prev >= level && cur < level;
    }
    return prev <= level && cur > level;
}

static uint8_t scope_auto_find_edge(uint8_t idx,
                                    uint16_t start,
                                    uint16_t end,
                                    uint8_t level,
                                    uint8_t falling,
                                    uint16_t *pos) {
    if (!pos || idx >= 2u || start < 1u) {
        return 0;
    }
    if (end > SCOPE_SAMPLE_COUNT) {
        end = SCOPE_SAMPLE_COUNT;
    }
    for (uint16_t i = start; i < end; ++i) {
        uint8_t prev = scope_samples[(uint16_t)((i - 1u) * 2u + idx)];
        uint8_t cur = scope_samples[(uint16_t)(i * 2u + idx)];
        if (scope_raw_crossed_level(prev, cur, level, falling)) {
            *pos = i;
            return 1;
        }
    }
    return 0;
}

static uint8_t scope_auto_measure_half_period(uint8_t idx, uint16_t *half_period) {
    uint16_t first = 0;
    uint16_t opposite = 0;
    uint16_t next = 0;
    uint8_t pp;
    uint8_t level;
    uint8_t falling;

    if (!half_period || idx >= 2u || !scope_ch_enabled[idx]) {
        return 0;
    }
    pp = scope_channel_pp_raw(idx);
    if (pp < 10u) {
        return 0;
    }
    level = scope_channel_mid_raw(idx);
    falling = ui.scope_trigger_edge ? 1u : 0u;
    if (!scope_auto_find_edge(idx, 2u, 2046u, level, falling, &first)) {
        return 0;
    }
    if (!scope_auto_find_edge(idx, (uint16_t)(first + 3u), 2046u, level, (uint8_t)!falling, &opposite)) {
        return 0;
    }
    if (!scope_auto_find_edge(idx, (uint16_t)(first + 3u), 2046u, level, falling, &next) ||
        next <= opposite) {
        return 0;
    }
    *half_period = (uint16_t)(opposite - first);
    return 1;
}

static uint8_t scope_auto_target_channel(void) {
    uint8_t idx = scope_trigger_source_index();

    if (idx < 2u && (scope_auto_time_mask & (uint8_t)(1u << idx)) && scope_ch_enabled[idx]) {
        return idx;
    }
    if ((scope_auto_time_mask & 0x01u) && scope_ch_enabled[0]) {
        return 0;
    }
    if ((scope_auto_time_mask & 0x02u) && scope_ch_enabled[1]) {
        return 1;
    }
    return ui.active_ch == 2u ? 1u : 0u;
}

static uint8_t scope_auto_time_finish(uint8_t final_slow_step) {
    uint8_t changed = 0;

    scope_auto_time_active = 0;
    scope_auto_channel_mask = scope_auto_enabled_mask(scope_auto_time_mask);
    if (final_slow_step && ui.scope_timebase < SCOPE_AUTO_TIMEBASE_MAX) {
        ++ui.scope_timebase;
        changed = 1;
    }
    if (changed) {
        ui.running = 1;
        scope_h_pos = 0;
        scope_h_value_pos = 0;
        scope_h_value_timebase = scope_safe_timebase();
        scope_cursor_time_sync_screen();
        scope_trigger_locked = 0;
        scope_roll_trigger_reset();
        scope_apply_settings();
        scope_hw_arm();
    }
    return changed;
}

static uint8_t scope_auto_time_service(void) {
    uint16_t half_period = 0;
    uint8_t idx;
    uint8_t valid;

    if (!scope_auto_time_active || !scope_frame_valid || scope_auto_last_frame == scope_frame_id) {
        return 0;
    }
    scope_auto_last_frame = scope_frame_id;
    scope_auto_time_mask = scope_auto_enabled_mask(scope_auto_time_mask);
    if (!scope_auto_time_mask) {
        scope_auto_cancel();
        return 0;
    }
    // Resolve overrange before looking for edges, including fully clipped DC.
    uint8_t changed = 0;
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        if (scope_auto_time_mask & (uint8_t)(1u << ch)) {
            changed |= scope_auto_channel_tune_vdiv(ch);
        }
    }
    if (changed) {
        scope_apply_settings();
        scope_hw_arm();
        return 1;
    }
    idx = scope_auto_target_channel();
    valid = scope_auto_measure_half_period(idx, &half_period);
    if (!valid) {
        for (uint8_t ch = 0; ch < 2u; ++ch) {
            if (ch != idx && (scope_auto_time_mask & (uint8_t)(1u << ch)) &&
                scope_auto_measure_half_period(ch, &half_period)) {
                ui.scope_trigger_source = (uint8_t)(ch + 1u);
                ui.scope_trigger_level = scope_channel_mid_raw(ch);
                valid = 1;
                break;
            }
        }
    }
    if (valid &&
        half_period >= SCOPE_AUTO_HALFPERIOD_MIN &&
        half_period <= SCOPE_AUTO_HALFPERIOD_MAX) {
        return scope_auto_time_finish(1);
    }
    if ((!valid || half_period > SCOPE_AUTO_HALFPERIOD_MAX) &&
        ui.scope_timebase < SCOPE_AUTO_TIMEBASE_MAX &&
        scope_auto_time_steps_left) {
        --scope_auto_time_steps_left;
        ++ui.scope_timebase;
        scope_h_pos = 0;
        scope_h_value_pos = 0;
        scope_h_value_timebase = scope_safe_timebase();
        scope_cursor_time_sync_screen();
        scope_trigger_locked = 0;
        scope_roll_trigger_reset();
        scope_apply_settings();
        scope_hw_arm();
        return 1;
    }
    return scope_auto_time_finish(0);
}

static void scope_auto_setup_request(uint8_t mask) {
    mask &= 0x03u;
    if (!mask) {
        mask = (uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u);
    }
    mask = scope_auto_enabled_mask(mask);
    scope_auto_cancel();
    if (!mask) {
        return;
    }
    scope_auto_zero_active = 0;
    scope_auto_seed_ranges(mask);
    scope_auto_time_mask = mask;
    scope_auto_time_steps_left = SCOPE_AUTO_TIMEBASE_MAX;
    scope_auto_time_active = 1;
    ui.running = 1;
    ui.scope_display = SCOPE_DISPLAY_YT;
    ui.scope_trigger_mode = SCOPE_TRIGGER_AUTO;
    if ((ui.active_ch == 2u && (mask & 0x02u)) ||
        !(mask & 0x01u)) {
        ui.scope_trigger_source = 2u;
    } else {
        ui.scope_trigger_source = 1u;
    }
    ui.scope_trigger_level = 128;
    ui.scope_timebase = 0;
    if ((mask & 0x03u) == 0x03u) {
        scope_ch_pos[0] = scope_clamp_channel_pos(0, 50);
        scope_ch_pos[1] = scope_clamp_channel_pos(1, -50);
    } else {
        if (mask & 0x01u) {
            scope_ch_pos[0] = 0;
        }
        if (mask & 0x02u) {
            scope_ch_pos[1] = 0;
        }
    }
    scope_h_pos = 0;
    scope_h_value_pos = 0;
    scope_h_value_timebase = scope_safe_timebase();
    scope_cursor_time_sync_screen();
    scope_trigger_locked = 0;
    scope_roll_trigger_reset();
    scope_apply_settings();
    scope_hw_arm();
}

static uint8_t scope_auto_channel_tune_vdiv(uint8_t idx) {
    // Widen-only tuning cannot oscillate at gain boundaries or amplify noise.
    if (idx >= 2u || !scope_ch_enabled[idx] || !scope_auto_channel_steps_left[idx]) {
        return 0;
    }
    uint8_t pp = scope_channel_pp_raw(idx);
    uint8_t vdiv = scope_vdiv_ch[idx];
    uint8_t clipped = (uint8_t)(scope_min_raw[idx] <= SCOPE_AUTO_Y_CLIP_LOW ||
                                scope_max_raw[idx] >= SCOPE_AUTO_Y_CLIP_HIGH);

    if ((!clipped && pp <= SCOPE_AUTO_Y_TARGET_MAX) || vdiv + 1u >= SCOPE_VDIV_COUNT) {
        return 0;
    }
    --scope_auto_channel_steps_left[idx];
    scope_auto_channel_stable_count[idx] = 0;
    scope_vdiv_ch[idx] = (uint8_t)(vdiv + 1u);
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
    changed = scope_auto_channel_tune_vdiv(idx);
    scope_set_channel_pos_from_avg(idx);
    if (idx == scope_trigger_source_index()) {
        ui.scope_trigger_level = scope_channel_mid_raw(idx);
    }
    return changed;
}

static void scope_auto_channel_request(uint8_t mask) {
    mask = scope_auto_enabled_mask((uint8_t)(mask & 0x03u));
    scope_auto_cancel();
    if (!mask) {
        return;
    }
    scope_auto_zero_active = 0;
    ui.running = 1;
    scope_auto_seed_ranges(mask);
    scope_auto_channel_mask = mask;
    scope_apply_settings();
    scope_hw_arm();
}

static void scope_auto_channel_service(void) {
    uint8_t changed = 0;

    if (!scope_auto_channel_mask || !scope_frame_valid || scope_auto_last_frame == scope_frame_id) {
        return;
    }
    scope_auto_last_frame = scope_frame_id;
    scope_auto_channel_mask = scope_auto_enabled_mask(scope_auto_channel_mask);
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        uint8_t bit = (uint8_t)(1u << ch);
        if (!(scope_auto_channel_mask & bit)) {
            continue;
        }
        if (scope_auto_channel_tune(ch)) {
            changed = 1;
        } else if (++scope_auto_channel_stable_count[ch] >= SCOPE_AUTO_STABLE_FRAMES) {
            scope_auto_channel_mask &= (uint8_t)~bit;
        }
    }
    scope_trace_invalidate();
    if (changed) {
        scope_apply_settings();
        scope_hw_arm();
    }
    if (!scope_auto_channel_mask) {
        scope_auto_cancel();
    }
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
        dmm_step_group(DMM_GROUP_VOLTAGE, 1);
    } else if (ui.mode == UI_MODE_SCOPE) {
        scope_open_move_mode();
    } else {
        gen_cycle_wave(1);
        gen_apply();
    }
}

static void dmm_step_all_modes(int8_t dir) {
    uint8_t choice;
    uint8_t group = dmm_mode_group(ui.dmm_mode, &choice);
    if (group == DMM_GROUP_COUNT) {
        group = DMM_GROUP_VOLTAGE;
        choice = 0;
    } else if (dir > 0 && choice + 1u == dmm_groups[group].count) {
        cycle_u8(&group, DMM_GROUP_COUNT, dir);
        choice = 0;
    } else if (dir < 0 && choice == 0) {
        cycle_u8(&group, DMM_GROUP_COUNT, dir);
        choice = dmm_groups[group].count - 1u;
    } else {
        choice = (uint8_t)(choice + dir);
    }
    dmm_apply_selected_mode(dmm_groups[group].modes[choice]);
}

static void arb_load_waveform(void) {
    if (arb_file_index < arb_file_count()) {
        if (arb_load_file(arb_file_index, arb_samples, 2048)) {
            arb_sample_count = arb_file_sample_count(arb_file_index);
            if (arb_sample_count == 0) {
                arb_sample_count = 1;
            }
            siggen_set_arb_waveform(arb_samples, arb_sample_count);
            arb_loaded = 1;
            return;
        }
    }
    arb_sample_count = 0;
    arb_loaded = 0;
    siggen_set_arb_waveform(0, 0);
}

static uint8_t gen_adjust_current_value(int8_t dir, uint8_t repeat) {
    if (ui.gen_wave == SIGGEN_WAVE_ARBITRARY) {
        if (ui.gen_param == GEN_PARAM_WAVE) {
            gen_cycle_wave((int8_t)-dir);
            gen_apply();
            return 2;
        }
        if (repeat) {
            return 0;
        }
        if (ui.gen_param == GEN_PARAM_FREQ) {
            uint8_t count = arb_file_count();
            if (count > 0) {
                if (dir > 0) {
                    arb_file_index = (uint8_t)((arb_file_index + 1u) % count);
                } else {
                    arb_file_index = arb_file_index == 0 ? (uint8_t)(count - 1u) : (uint8_t)(arb_file_index - 1u);
                }
                arb_load_waveform();
                gen_apply();
            }
            return 2;
        }
        if (ui.gen_param == GEN_PARAM_DUTY) {
            return 0;
        }
    }
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
        uint8_t group = dmm_mode_group(ui.dmm_mode, 0);
        dmm_step_group(group < DMM_GROUP_COUNT ? group : DMM_GROUP_VOLTAGE, dir);
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
        dmm_step_group(DMM_GROUP_TEST, 1);
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

static void boot_export_progress(uint8_t percent) {
    if (percent < 100u && percent < ui.boot_export_percent + 5u) {
        return;
    }
    ui.boot_export_percent = percent;
    ui_render();
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
    } else if (ui.settings_row == SETTINGS_ROW_SLEEP) {
        cycle_u8(&ui_settings.sleep_enabled, SETTINGS_SLEEP_COUNT, dir);
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
    ui.boot_export_state = BOOT_EXPORT_UI_NONE;
    ui.boot_export_overlay_ms = 0;
    if (index > 3u) {
        index = 0;
    }
    ui.menu_index = index;
    if (index < 3u) {
        ui_switch_mode((ui_mode_t)index);
    } else {
        ui.overlay = UI_OVERLAY_SETTINGS;
        ui.settings_row = 0;
        ui.reset_confirm = 0;
        settings_refresh_storage();
    }
}

static void ui_direct_mode(uint8_t index) {
    scope_clear_menus();
    ui.scope_move_menu_ms = 0;
    ui.scope_move_sel = SCOPE_MOVE_SEL_CHANNEL;
    dmm_pause_for_menu_overlay();
    ui.overlay = UI_OVERLAY_NONE;
    ui.chrome_visible = 0;
    ui_open_mode_menu_item(index);
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
        if (events & (KEY_UP | KEY_DOWN)) {
            // Two rows of two: up and down swap rows in the same column.
            ui.menu_index = (uint8_t)((ui.menu_index & 3u) ^ 2u);
            ui_render();
            return;
        }
        return;
    }

    if (ui.boot_export_state == BOOT_EXPORT_UI_CONFIRM) {
        if (events & KEY_REPEAT) {
            return;
        }
        if (events & KEY_MENU) {
            ui.boot_export_state = BOOT_EXPORT_UI_NONE;
        } else if (events & KEY_OK) {
            ui.boot_export_state = BOOT_EXPORT_UI_SAVING;
            ui.boot_export_percent = 0;
            ui_render();
            ui.boot_export_result = usb_msc_export_bootloader(FIRMWARE_VERSION_TEXT, boot_export_progress);
            ui.boot_export_state = BOOT_EXPORT_UI_RESULT;
            ui.boot_export_overlay_ms = 6000;
        }
        ui_render();
        return;
    }
    if (ui.reset_confirm) {
        if (events & KEY_REPEAT) {
            return;
        }
        if (events & KEY_MENU) {
            ui.reset_confirm = 0;
            ui_render();
        } else if (events & KEY_OK) {
            gen_stop();
            settings_factory_reset();
            // Reboot rather than try to unwind the live state by hand:
            // ui_init() reads the settings once into static variables that
            // nothing else re-reads, so only a restart is honest here.
            board_reboot();
        }
        return;
    }
    ui.boot_export_state = BOOT_EXPORT_UI_NONE;
    ui.boot_export_overlay_ms = 0;

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
        if (ui.settings_row == SETTINGS_BOOT_EXPORT_ROW) {
            if (events & KEY_OK) {
                ui.boot_export_state = BOOT_EXPORT_UI_CONFIRM;
            }
        } else if (ui.settings_row == SETTINGS_ROW_RESET) {
            if (events & KEY_OK) {
                ui.reset_confirm = 1;
            }
        } else {
            settings_adjust_current(1);
        }
        ui_render();
        return;
    }
}

static uint8_t ui_handle_keys_gen_sweep_menu(uint32_t events) {
    if (!gen_sweep_menu_active) {
        return 0;
    }

    if (events & KEY_SAVE_LONG) {
        capture_screenshot_now();
        return 1; 
    }

    if (events & KEY_OK) {
        if ((gen_sweep_row_selected == 1 || gen_sweep_row_selected == 2 || gen_sweep_row_selected == 3) && gen_sweep_mode == 0) {
            return 1;
        }
        if ((gen_sweep_row_selected == 5 || gen_sweep_row_selected == 6) && gen_fm_mode == 0) {
            return 1;
        }

        if (gen_sweep_row_selected == 1 || gen_sweep_row_selected == 2 || 
            gen_sweep_row_selected == 3 || gen_sweep_row_selected == 6) {
            ui_edit_digit_active = !ui_edit_digit_active;
            ui_edit_digit_index = 0;
        } else {
            ui_edit_digit_active = 0;
        }

        ui_render_local_change();
        return 1;
    }

    if (ui_edit_digit_active) {
        if (((gen_sweep_row_selected == 1 || gen_sweep_row_selected == 2 || gen_sweep_row_selected == 3) && gen_sweep_mode == 0) ||
            (gen_sweep_row_selected == 6 && gen_fm_mode == 0)) {
            ui_edit_digit_active = 0;
            ui_render_local_change();
            return 1;
        }

        uint32_t current_val;
        uint32_t multiplier = 1u;
        uint8_t *unit_ptr = 0;
        if (gen_sweep_row_selected == 1) {
            current_val = gen_sweep_ms;
        } else if (gen_sweep_row_selected == 2) {
            multiplier = gen_freq_unit_multiplier(sweep_start_unit);
            current_val = freq_clamp_display(siggen_sweep_start_hz, sweep_start_unit);
            unit_ptr = &sweep_start_unit;
        } else if (gen_sweep_row_selected == 3) {
            multiplier = gen_freq_unit_multiplier(sweep_stop_unit);
            current_val = freq_clamp_display(siggen_sweep_stop_hz, sweep_stop_unit);
            unit_ptr = &sweep_stop_unit;
        } else {
            current_val = gen_fm_freq_hz;
        }

        uint8_t max_edit_index = gen_sweep_row_selected == 6 ? 3u : 4u;
        uint32_t thousands = (current_val / 1000) % 10;
        uint32_t hundreds = (current_val / 100) % 10;
        uint32_t tens = (current_val / 10) % 10;
        uint32_t ones = current_val % 10;

        if (events & KEY_LEFT) {
            if (ui_edit_digit_index > 0) {
                ui_edit_digit_index--;
            } else {
                ui_edit_digit_index = max_edit_index;
            }
        } 
        else if (events & KEY_RIGHT) {
            if (ui_edit_digit_index < max_edit_index) {
                ui_edit_digit_index++;
            } else {
                ui_edit_digit_index = 0;
            }
        } 
        else if (events & (KEY_UP | KEY_DOWN)) {
            int8_t delta = (events & KEY_DOWN) ? 1 : -1;
            
            if (ui_edit_digit_index < 4) {
                if (ui_edit_digit_index == 0) thousands = (thousands + delta + 10) % 10;
                else if (ui_edit_digit_index == 1) hundreds = (hundreds + delta + 10) % 10;
                else if (ui_edit_digit_index == 2) tens = (tens + delta + 10) % 10;
                else if (ui_edit_digit_index == 3) ones = (ones + delta + 10) % 10;

                uint32_t new_val = (thousands * 1000) + (hundreds * 100) + (tens * 10) + ones;

                if (gen_sweep_row_selected == 1) {
                    if (new_val < 100) new_val = 100;
                    if (new_val > 9999) new_val = 9999;
                    gen_sweep_ms = (uint16_t)new_val;
                }
                else if (gen_sweep_row_selected == 2) {
                    if (new_val < 1) new_val = 1;
                    new_val *= multiplier;
                    if (new_val > GEN_MAX_FREQ_HZ) new_val = GEN_MAX_FREQ_HZ;
                    siggen_sweep_start_hz = new_val;
                }
                else if (gen_sweep_row_selected == 3) {
                    if (new_val < 1) new_val = 1;
                    new_val *= multiplier;
                    if (new_val > GEN_MAX_FREQ_HZ) new_val = GEN_MAX_FREQ_HZ;
                    siggen_sweep_stop_hz = new_val;
                }
                else if (gen_sweep_row_selected == 6) {
                    if (new_val < 1) new_val = 1;
                    if (new_val > SETTINGS_SIGGEN_FM_MAX_HZ) new_val = SETTINGS_SIGGEN_FM_MAX_HZ;
                    gen_fm_freq_hz = new_val;
                }
            } else if (unit_ptr) {
                int8_t new_unit = (int8_t)(*unit_ptr) + delta;
                if (new_unit < 0) new_unit = 0;
                if (new_unit > 2) new_unit = 2;
                *unit_ptr = (uint8_t)new_unit;
            }
        }
        ui_render_local_change();
        return 1;
    }

    if (events & KEY_DOWN) {
        if (gen_sweep_row_selected > 0) {
            gen_sweep_row_selected--;
        } else {
            gen_sweep_row_selected = 6;
        }
        ui_render_local_change();
        return 1;
    } 
    else if (events & KEY_UP) {
        if (gen_sweep_row_selected < 6) {
            gen_sweep_row_selected++;
        } else {
            gen_sweep_row_selected = 0;
        }
        ui_render_local_change();
        return 1;
    } 
    else if (events & (KEY_LEFT | KEY_RIGHT)) {
        int8_t step = (events & KEY_RIGHT) ? 1 : -1;
        
        switch (gen_sweep_row_selected) {
            case 0:
                gen_sweep_mode = (uint8_t)((gen_sweep_mode + step + 3) % 3);
                if (gen_sweep_mode) {
                    gen_fm_mode = 0;
                    gen_fm_phase = 0;
                }
                break;
            case 4:
                gen_fm_mode = (uint8_t)((gen_fm_mode + step + 2) % 2);
                if (gen_fm_mode) {
                    gen_sweep_mode = 0;
                    gen_sweep_elapsed_ms = 0;
                }
                break;
            case 5:
                gen_fm_source = (uint8_t)((gen_fm_source + step + 3) % 3);
                break;
        }
        ui_render_local_change();
        return 1;
    }

    if (events & KEY_MENU) {
        gen_sweep_menu_active = 0;
        ui_edit_digit_active = 0;
        ui_render(); 
        return 1;
    }

    return 1;
}

static uint8_t ui_handle_keys_math_menu(uint32_t events) {
    if (!ui_math_menu_visible) {
        return 0;
    }

    if (events & KEY_SAVE_LONG) {
        capture_screenshot_now();
        return 1;
    }

    uint8_t current_item;
    if (scope_fft_src == 4u) {
        current_item = bode_menu_order[scope_math_selected];
    } else {
        current_item = math_menu_order[scope_math_selected];
    }


    if (events & KEY_OK) {
        if (current_item == MENU_ITEM_BODE_START || 
            current_item == MENU_ITEM_BODE_STOP || 
            current_item == MENU_ITEM_BODE_STEPS) {
            ui_edit_digit_active = !ui_edit_digit_active;
            ui_edit_digit_index = 0;
        } else if (current_item == MENU_ITEM_BODE_SWEEP) {
            ui_edit_digit_active = 0;
            bode_begin_sweep();
        } else {
            ui_edit_digit_active = 0;
        }

        ui_render_local_change();
        return 1;
    }

    if (ui_edit_digit_active) {
        if (current_item != MENU_ITEM_BODE_START && 
            current_item != MENU_ITEM_BODE_STOP && 
            current_item != MENU_ITEM_BODE_STEPS) {
            ui_edit_digit_active = 0;
            ui_render_local_change();
            return 1;
        }

        uint32_t current_val;
        uint32_t multiplier = 1u;
        uint8_t *unit_ptr = 0;
        if (current_item == MENU_ITEM_BODE_START) {
            multiplier = gen_freq_unit_multiplier(bode_start_unit);
            current_val = freq_clamp_display(bode_start_hz, bode_start_unit);
            unit_ptr = &bode_start_unit;
        } else if (current_item == MENU_ITEM_BODE_STOP) {
            multiplier = gen_freq_unit_multiplier(bode_stop_unit);
            current_val = freq_clamp_display(bode_stop_hz, bode_stop_unit);
            unit_ptr = &bode_stop_unit;
        } else {
            current_val = bode_steps;
        }
        
        uint32_t thousands = (current_val / 1000) % 10;
        uint32_t hundreds  = (current_val / 100) % 10;
        uint32_t tens      = (current_val / 10) % 10;
        uint32_t ones      = current_val % 10;

        if (events & KEY_LEFT) {
            if (ui_edit_digit_index > 0) {
                ui_edit_digit_index--;
            } else {
                ui_edit_digit_index = (current_item == MENU_ITEM_BODE_STEPS) ? 3 : 4;
            }
        } 
        else if (events & KEY_RIGHT) {
            uint8_t max_idx = (current_item == MENU_ITEM_BODE_STEPS) ? 3 : 4;
            if (ui_edit_digit_index < max_idx) {
                ui_edit_digit_index++;
            } else {
                ui_edit_digit_index = 0;
            }
        } 
        else if (events & (KEY_UP | KEY_DOWN)) {
            int8_t delta = (events & KEY_DOWN) ? 1 : -1;
            
            if (ui_edit_digit_index < 4) {
                if (ui_edit_digit_index == 0) thousands = (thousands + delta + 10) % 10;
                else if (ui_edit_digit_index == 1) hundreds = (hundreds + delta + 10) % 10;
                else if (ui_edit_digit_index == 2) tens = (tens + delta + 10) % 10;
                else if (ui_edit_digit_index == 3) ones = (ones + delta + 10) % 10;

                uint32_t new_val = (thousands * 1000) + (hundreds * 100) + (tens * 10) + ones;

                if (current_item == MENU_ITEM_BODE_START) {
                    if (new_val < 1) new_val = 1;
                    new_val *= multiplier;
                    if (new_val > GEN_MAX_FREQ_HZ) new_val = GEN_MAX_FREQ_HZ;
                    bode_start_hz = new_val;
                } else if (current_item == MENU_ITEM_BODE_STOP) {
                    if (new_val < 1) new_val = 1;
                    new_val *= multiplier;
                    if (new_val > GEN_MAX_FREQ_HZ) new_val = GEN_MAX_FREQ_HZ;
                    bode_stop_hz = new_val;
                } else {
                    bode_steps = bode_clamp_steps(new_val);
                }
            } else if (unit_ptr) {
                int8_t new_unit = (int8_t)(*unit_ptr) + delta;
                if (new_unit < 0) new_unit = 0;
                if (new_unit > 2) new_unit = 2;
                *unit_ptr = (uint8_t)new_unit;
            }
        }
        ui_render_local_change();
        return 1;
    }

    if (events & KEY_UP) {
        math_menu_step_selected(1); 
        ui_render_local_change();
        return 1;
    }
    else if (events & KEY_DOWN) {
        math_menu_step_selected(-1);
        ui_render_local_change();
        return 1;
    }
    else if (events & (KEY_LEFT | KEY_RIGHT)) {
        int8_t step = (events & KEY_RIGHT) ? 1 : -1;

        switch (current_item) {
            case MENU_ITEM_PLOT:
                if (step > 0) {
                    if (scope_fft_src < 4) ++scope_fft_src; else scope_fft_src = 0;
                } else {
                    if (scope_fft_src > 0) --scope_fft_src; else scope_fft_src = 4;
                }
                ui.scope_display = (scope_fft_src == 3) ? SCOPE_DISPLAY_XY : SCOPE_DISPLAY_YT;
                break;

            case MENU_ITEM_MATH_MODE:
                scope_math_mode = !scope_math_mode;
                break;

            case MENU_ITEM_MATH_OP:
                if (scope_math_mode) {
                    scope_math_op = (uint8_t)((scope_math_op + step + 3) % 3);
                }
                break;

            case MENU_ITEM_FFT_WINDOW:
                if (scope_fft_src == 1 || scope_fft_src == 2) {
                    scope_fft_window = (uint8_t)((scope_fft_window + step + 4) % 4);
                }
                break;

            case MENU_ITEM_FFT_DISPLAY:
                if (scope_fft_src == 1 || scope_fft_src == 2) {
                    scope_fft_display = (uint8_t)((scope_fft_display + step + 3) % 3);
                    for (uint16_t i = 0; i < FFT_SIZE / 2u; ++i) {
                        fft_history[i] = 0.0f;
                    }
                }
                break;

            case MENU_ITEM_HIDE_TRACES:
                scope_hide_traces = (uint8_t)((scope_hide_traces + step + 4) % 4);
                break;

            case MENU_ITEM_BODE_STEPS:
                break;
        }
        ui_render_local_change();
        return 1;
    }
    else if (events & KEY_MENU) {
        ui_math_menu_visible = 0;
        ui_edit_digit_active = 0;

        ui_settings.scope_math_mode = scope_math_mode; 
        ui_settings.scope_math_op = scope_math_op;
        ui_settings.scope_fft_window = scope_fft_window;
        ui_settings.scope_fft_display = scope_fft_display;
        ui_settings.scope_math_selected = scope_math_selected;
        ui_settings.bode_start_hz = bode_start_hz;
        ui_settings.bode_stop_hz = bode_stop_hz;
        ui_settings.bode_steps = bode_steps;

        if (scope_fft_src == 4u && bode_is_sweeping) {
            scope_fft_src_menu_saved = scope_fft_src;
            ui_settings.scope_fft_src = scope_fft_src;
            ui_settings.scope_hide_traces = scope_hide_traces;
            settings_note(&ui_settings);
            settings_flush();
            
            ui_render_local_change(); 
        } else {
            scope_fft_src_apply_from(scope_fft_src_menu_saved, scope_fft_src);
            scope_fft_src_menu_saved = scope_fft_src;
            ui_settings.scope_fft_src = scope_fft_src;
            ui_settings.scope_hide_traces = scope_hide_traces;

            settings_note(&ui_settings);
            settings_flush();
        }
    }

    return 1;
}

void ui_handle_keys(uint32_t events) {
    ui.idle_ms = 0;
    ui.sleep_ms = 0;
    ui.sleep_due = 0;

    if (events == 0) return;

    if (ui.overlay == UI_OVERLAY_SETTINGS && ui.boot_export_state == BOOT_EXPORT_UI_CONFIRM) {
        ui_handle_menu_keys(events);
        return;
    }

    if (gen_sweep_menu_active && ui_handle_keys_gen_sweep_menu(events)) {
        return;
    }
    if (ui_math_menu_visible && ui_handle_keys_math_menu(events)) {
        return;
    }

    if (ui_math_menu_visible && (ui.mode != UI_MODE_SCOPE || ui.overlay != UI_OVERLAY_NONE)) {
        ui_math_menu_visible = 0;
    }

    if ((events & KEY_CH1_LONG) &&
        ui.mode == UI_MODE_SCOPE &&
        ui.overlay == UI_OVERLAY_NONE &&
        (!scope_any_menu_open() || ui_math_menu_visible)) {
        ui_math_menu_visible = !ui_math_menu_visible;
        if (ui_math_menu_visible) {
            scope_math_mode = ui_settings.scope_math_mode;
            scope_math_op = ui_settings.scope_math_op;
            scope_fft_src = ui_settings.scope_fft_src;
            scope_fft_src_menu_saved = scope_fft_src;
            scope_math_selected = ui_settings.scope_math_selected;
            scope_fft_window = ui_settings.scope_fft_window;
            scope_fft_display = ui_settings.scope_fft_display;
            scope_hide_traces = ui_settings.scope_hide_traces;
            bode_start_hz = ui_settings.bode_start_hz;
            bode_stop_hz = ui_settings.bode_stop_hz;
            freq_sync_display_unit(&bode_start_unit, bode_start_hz);
            freq_sync_display_unit(&bode_stop_unit, bode_stop_hz);
            bode_steps = bode_clamp_steps(ui_settings.bode_steps);
            if (scope_fft_src == 3) {
                ui.scope_display = SCOPE_DISPLAY_XY; 
            } else {
                ui.scope_display = SCOPE_DISPLAY_YT;
            }
        }
        ui_render_local_change();
        return; 
    }

    if (gen_sweep_menu_active && (ui.mode != UI_MODE_GEN || ui.overlay != UI_OVERLAY_NONE)) {
        gen_sweep_menu_active = 0;
        ui_edit_digit_active = 0;
        gen_sweep_row_selected = 0;
    }

    if ((events & KEY_CH1_LONG) && ui.mode == UI_MODE_GEN && !gen_sweep_menu_active) {
        gen_sweep_menu_active = 1;
        gen_sweep_row_selected = 0;
        ui_edit_digit_active = 0;
        ui_edit_digit_index = 0;

        ui_render_local_change();
        return;
    }

    if (events & KEY_SAVE_LONG) {
        capture_screenshot_now();
        return;
    }

    if (events & KEY_MOVE_LONG) {
        ui_direct_mode(0);
        ui_render();
        return;
    }
    if (events & KEY_F2_LONG) {
        ui_direct_mode(1);
        ui_render();
        return;
    }
    if (events & KEY_F3_LONG) {
        ui_direct_mode(2);
        ui_render();
        return;
    }
    if (events & KEY_F4_LONG) {
        ui_direct_mode(3);
        ui_render();
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
            dmm_apply_selected_mode(DMM_MODE_AUTO);
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
                scope_auto_setup_request(mask);
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
            dmm_step_group(DMM_GROUP_SENSE, 1);
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
                gen_running = ui.running ? 1u : 0u;
                gen_user_enabled = gen_running;
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
            dmm_step_group(DMM_GROUP_CURRENT, 1);
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

static uint32_t settings_sleep_timeout_ms(void) {
    static const uint32_t timeouts_ms[SETTINGS_SLEEP_COUNT] = {
        0u,
        10u * 60u * 1000u,
        20u * 60u * 1000u,
        30u * 60u * 1000u,
    };
    uint8_t sleep = ui_settings.sleep_enabled;
    if (sleep >= SETTINGS_SLEEP_COUNT) {
        return 0;
    }
    return timeouts_ms[sleep];
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

static void scope_stream_telemetry(uint32_t elapsed_ms) {
    static uint32_t scope_stream_acc_ms = 0;
    if (!usb_cdc_is_configured() || ui.mode != UI_MODE_SCOPE) {
        scope_stream_acc_ms = 0;
        return;
    }

    scope_stream_acc_ms += elapsed_ms;
    if (scope_stream_acc_ms < 66u) {
        return;
    }
    scope_stream_acc_ms = 0;

    // 1. Meta packet: $SCOPE,<tb>,<v1>,<v2>,<f1>,<vpp1>,<f2>,<vpp2>,<run>,<ch1_en>,<ch2_en>\r\n
    char meta[128];
    uint16_t mpos = 0;
    const char *p = "$SCOPE,";
    while (*p) meta[mpos++] = *p++;

    const char *tb = scope_timebase_chip_label();
    while (*tb) meta[mpos++] = *tb++;
    meta[mpos++] = ',';

    const char *v1 = scope_vdiv_chip_label_for_channel(0);
    while (*v1) meta[mpos++] = *v1++;
    meta[mpos++] = ',';

    const char *v2 = scope_vdiv_chip_label_for_channel(1);
    while (*v2) meta[mpos++] = *v2++;
    meta[mpos++] = ',';

    char num_buf[16];
    uint32_t f1 = scope_ch_enabled[0] ? scope_filtered_freq_hz(0) : 0;
    uint8_t min0 = scope_filtered_min_raw(0);
    uint8_t max0 = scope_filtered_max_raw(0);
    uint32_t vpp1 = (scope_ch_enabled[0] && max0 >= min0) ?
                    (uint32_t)scope_raw_span_mv(0, (uint8_t)(max0 - min0)) : 0;

    scope_format_u32(num_buf, f1);
    p = num_buf;
    while (*p) meta[mpos++] = *p++;
    meta[mpos++] = ',';

    scope_format_u32(num_buf, vpp1);
    p = num_buf;
    while (*p) meta[mpos++] = *p++;
    meta[mpos++] = ',';

    uint32_t f2 = scope_ch_enabled[1] ? scope_filtered_freq_hz(1) : 0;
    uint8_t min1 = scope_filtered_min_raw(1);
    uint8_t max1 = scope_filtered_max_raw(1);
    uint32_t vpp2 = (scope_ch_enabled[1] && max1 >= min1) ?
                    (uint32_t)scope_raw_span_mv(1, (uint8_t)(max1 - min1)) : 0;

    scope_format_u32(num_buf, f2);
    p = num_buf;
    while (*p) meta[mpos++] = *p++;
    meta[mpos++] = ',';

    scope_format_u32(num_buf, vpp2);
    p = num_buf;
    while (*p) meta[mpos++] = *p++;
    meta[mpos++] = ',';

    meta[mpos++] = ui.running ? '1' : '0';
    meta[mpos++] = ',';
    meta[mpos++] = scope_ch_enabled[0] ? '1' : '0';
    meta[mpos++] = ',';
    meta[mpos++] = scope_ch_enabled[1] ? '1' : '0';
    meta[mpos++] = '\r';
    meta[mpos++] = '\n';

    usb_cdc_send_bytes((const uint8_t *)meta, mpos);

    // 2. Waveforms
    static const char hex_digits[] = "0123456789ABCDEF";
    // Stream every point the screen shows, so the PC measures on the same data
    // the device does. At 180 the trace was decimated from these 300 samples,
    // which cost precision on peaks (a 3318 mV span read as 2698 mV) and forced
    // the PC to recompute what the device already measured.
    // Cost: 610 bytes per channel per frame, ~19 kB/s with both channels at
    // 15 frames/s -- about a third of what USB CDC carries.
    enum { SCOPE_STREAM_POINTS = SCOPE_VISIBLE_SAMPLE_COUNT };
    static char wave_buf[8 + SCOPE_STREAM_POINTS * 2 + 3];

    for (uint8_t ch = 0; ch < 2u; ++ch) {
        if (!scope_ch_enabled[ch]) continue;

        uint16_t wpos = 0;
        wave_buf[wpos++] = '$';
        wave_buf[wpos++] = 'W';
        wave_buf[wpos++] = 'A';
        wave_buf[wpos++] = 'V';
        wave_buf[wpos++] = 'E';
        wave_buf[wpos++] = ',';
        wave_buf[wpos++] = (char)('1' + ch);
        wave_buf[wpos++] = ',';

        uint8_t slow_roll = scope_slow_roll_active();
        uint16_t r_count = scope_slow_roll_count;

        for (uint16_t i = 0; i < SCOPE_STREAM_POINTS; ++i) {
            uint8_t sample;
            if (slow_roll) {
                uint16_t roll_idx = (r_count > 1u) ? (uint16_t)((uint32_t)i * (r_count - 1u) / (SCOPE_STREAM_POINTS - 1u)) : 0;
                sample = scope_slow_roll_raw_at(ch, roll_idx);
            } else {
                sample = scope_display_sample_raw(i, ch, SCOPE_STREAM_POINTS - 1u);
            }
            wave_buf[wpos++] = hex_digits[(sample >> 4) & 0x0F];
            wave_buf[wpos++] = hex_digits[sample & 0x0F];
        }

        wave_buf[wpos++] = '\r';
        wave_buf[wpos++] = '\n';

        usb_cdc_send_bytes((const uint8_t *)wave_buf, wpos);
    }
}

void ui_tick(uint32_t elapsed_ms) {
    uint8_t rendered = 0;


    if (keys_injected) {
        uint32_t events = keys_injected;
        keys_injected = 0;
        ui_handle_keys(events);
    }

    if (screenshot_requested) {
        screenshot_requested = 0;
        capture_screenshot_now();
    }

    uint8_t battery_changed = 0;
    uint8_t fw_update_changed = 0;
    fw_update_status_t fw_status;
    uint32_t sleep_timeout_ms = settings_sleep_timeout_ms();

    if (sleep_timeout_ms) {
        uint32_t sleep_next = ui.sleep_ms + elapsed_ms;
        if (sleep_next >= sleep_timeout_ms) {
            ui.sleep_ms = sleep_timeout_ms;
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

    if (ui.boot_export_overlay_ms) {
        if (elapsed_ms >= ui.boot_export_overlay_ms) {
            ui.boot_export_overlay_ms = 0;
            ui.boot_export_state = BOOT_EXPORT_UI_NONE;
            ui_render();
            rendered = 1;
        } else {
            ui.boot_export_overlay_ms = (uint16_t)(ui.boot_export_overlay_ms - elapsed_ms);
        }
    }

    if (gen_deferred_apply && !bode_siggen_active) {
        if (elapsed_ms >= gen_deferred_apply_ms) {
            gen_apply();
        } else {
            gen_deferred_apply_ms = (uint16_t)(gen_deferred_apply_ms - elapsed_ms);
        }
    }

    if (gen_running && !bode_siggen_active && (gen_sweep_mode != 0 || gen_fm_mode != 0)) {

        if (gen_sweep_mode != 0) {
            gen_sweep_elapsed_ms += elapsed_ms;
            if (gen_sweep_elapsed_ms >= gen_sweep_ms) {
                gen_sweep_elapsed_ms = 0; 
            }
        } else {
            gen_sweep_elapsed_ms = 0;
        }

        if (gen_fm_mode != 0) {
            uint32_t advance = gen_fm_freq_hz * elapsed_ms * 65536u / 1000u;
            gen_fm_phase = (uint16_t)(gen_fm_phase + advance);
        } else {
            gen_fm_phase = 0;
        }

        siggen_set_frequency(gen_get_current_sweep_freq());
    } else {
        gen_sweep_elapsed_ms = 0;
        gen_fm_phase = 0;
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
                
                bode_sweep_service();
                if (scope_fft_src == 4u && bode_is_sweeping) {
                    ui_render_scope_frame();
                } else if (!rendered) {
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

        if (ui.mode == UI_MODE_SCOPE) {
        scope_stream_telemetry(elapsed_ms);
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

void ui_set_dmm_mode(uint8_t mode) {
    if (mode < DMM_MODE_COUNT) {
        mode = dmm_sanitize_mode(mode);
        dmm_apply_selected_mode(mode);
        ui_render_local_change();
    }
}

void ui_set_main_mode(uint8_t mode) {
    if (mode >= 3u) mode = 0;
    if (ui.mode != mode) {
        ui_direct_mode(mode);
        // Every key-driven shortcut repaints right here (KEY_MOVE_LONG and
        // its neighbours). This one did not, so a mode change asked for
        // over USB left the previous screen's title and measurements on the
        // glass until something else happened to ask for a full render.
        ui_render();
    }
}

void ui_scope_set_running(uint8_t run) {
    if (ui.mode == UI_MODE_SCOPE) {
        ui.running = run ? 1u : 0u;
        ui_render_mode_content();
    }
}

void ui_scope_trigger_auto(void) {
    if (ui.mode == UI_MODE_SCOPE) {
        uint8_t mask = (uint8_t)((scope_ch_enabled[0] ? 0x01u : 0u) |
                                 (scope_ch_enabled[1] ? 0x02u : 0u));
        if (!mask) {
            mask = (uint8_t)(ui.active_ch == 2u ? 0x02u : 0x01u);
        }
        scope_auto_setup_request(mask);
    }
}

void ui_scope_step_timebase(int8_t dir) {
    if (ui.mode == UI_MODE_SCOPE) {
        scope_step_timebase(dir);
        ui_render_mode_content();
    }
}

// Asks for a screenshot; the capture happens on the next ui_tick().
void ui_request_screenshot(void) {
    screenshot_requested = 1;
}

// Queues key events as if the buttons had been pressed, so the interface can
// be driven from a PC. Handled on the next ui_tick().
void ui_inject_keys(uint32_t events) {
    keys_injected |= events;
}

void ui_scope_step_vdiv(uint8_t ch, int8_t dir) {
    if (ui.mode == UI_MODE_SCOPE && ch < 2u) {
        cycle_u8(&scope_vdiv_ch[ch], SCOPE_VDIV_COUNT, dir);
        scope_auto_cancel();
        ui.scope_vdiv = scope_safe_vdiv();
        scope_cursor_level_sync_screen();
        scope_apply_settings();
        ui_render_mode_content();
    }
}

// Remote generator control. These apply from any screen so the output can be
// driven while the scope stays on screen and keeps streaming telemetry.
// gen_apply() runs gen_prepare_state() first, which clamps every field.
static void ui_gen_refresh(void) {
    if (ui.mode == UI_MODE_GEN) {
        ui.running = gen_running;
        ui_render_mode_content();
    }
}

void ui_gen_set_output(uint8_t run) {
    gen_user_enabled = run ? 1u : 0u;
    gen_running = gen_user_enabled && ui.mode != UI_MODE_DMM;
    gen_apply();
    ui_gen_refresh();
}

void ui_gen_set_frequency(uint32_t hz) {
    ui.gen_freq_hz = hz;
    gen_apply();
    ui_gen_refresh();
}

void ui_gen_set_wave(uint8_t wave) {
    if (wave >= SIGGEN_WAVE_COUNT) {
        return;
    }
    ui.gen_wave = wave;
    gen_apply();
    ui_gen_refresh();
}

void ui_gen_set_amplitude(uint8_t tenths_v) {
    ui.gen_amp_tenths_v = tenths_v;
    gen_apply();
    ui_gen_refresh();
}

void ui_gen_set_duty(uint8_t duty_percent) {
    ui.gen_duty_percent = duty_percent;
    gen_apply();
    ui_gen_refresh();
}

uint8_t ui_gen_is_running(void) {
    return gen_running;
}

uint32_t ui_gen_frequency(void) {
    return ui.gen_freq_hz;
}

uint8_t ui_gen_wave(void) {
    return ui.gen_wave;
}

uint8_t ui_gen_amplitude(void) {
    return ui.gen_amp_tenths_v;
}
