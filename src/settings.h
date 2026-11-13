#pragma once

#include <stdint.h>

enum {
    SETTINGS_START_MENU,
    SETTINGS_START_DMM,
    SETTINGS_START_SCOPE,
    SETTINGS_START_GEN,
    SETTINGS_START_COUNT,
    SETTINGS_LEVEL_COUNT = 5,
    SETTINGS_SLEEP_COUNT = 4,
    SETTINGS_SCOPE_CHANNEL_COUNT = 2,
    SETTINGS_SCOPE_RANGE_COUNT = 9,
    SETTINGS_SCOPE_TIMEBASE_DEFAULT = 4,
    SETTINGS_SCOPE_TIMEBASE_COUNT = 26,
    SETTINGS_SCOPE_DISPLAY_COUNT = 3,
    SETTINGS_SCOPE_CURSOR_COUNT = 3,
    SETTINGS_SCOPE_TRIGGER_COUNT = 3,
    SETTINGS_SCOPE_MEASURE_COUNT = 6,
    SETTINGS_SIGGEN_WAVE_COUNT = 16,
    SETTINGS_SIGGEN_PARAM_COUNT = 4,
    SETTINGS_SIGGEN_FREQ_UNIT_COUNT = 3,
    SETTINGS_SIGGEN_DEFAULT_WAVE = 0,
    SETTINGS_SIGGEN_DEFAULT_PARAM = 1,
    SETTINGS_SIGGEN_DEFAULT_DUTY = 50,
    SETTINGS_SIGGEN_DEFAULT_AMP_TENTHS = 33,
    SETTINGS_SIGGEN_DEFAULT_FREQ_HZ = 1000,
    SETTINGS_SIGGEN_MAX_FREQ_HZ = 2000000,
    SETTINGS_SIGGEN_FM_MAX_HZ = 10,
    SETTINGS_SIGGEN_SWEEP_MODE_COUNT = 3, // 0 = OFF, 1 = LINEAR, 2 = LOG
    SETTINGS_SIGGEN_FM_MODE_COUNT = 2, // 0 = OFF, 1 = ON
    SETTINGS_SIGGEN_FM_SOURCE_COUNT = 3, // 0 = SINE, 1 = TRIANGLE, 2 = SQUARE
    SETTINGS_BODE_MIN_STEPS = 10,
    SETTINGS_BODE_MAX_STEPS = 80,
};

typedef struct {
    uint8_t dmm_mode;
    uint8_t beep_level;
    uint8_t brightness_level;
    uint8_t startup_screen;
    uint8_t last_screen;
    uint8_t sleep_enabled;
    uint8_t scope_timebase;
    uint8_t scope_display;
    uint8_t scope_cursor_mode;
    uint8_t scope_trigger_source;
    uint8_t scope_trigger_mode;
    uint8_t scope_trigger_edge;
    uint8_t scope_trigger_level;
    uint8_t scope_afterglow;
    uint8_t scope_active_ch;
    uint8_t scope_measure_param;
    uint8_t scope_measure_visible;
    uint8_t scope_ch_enabled[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_probe_x10[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_vdiv[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_coupling_dc[SETTINGS_SCOPE_CHANNEL_COUNT];
    int8_t scope_ch_pos[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_measure_mask[SETTINGS_SCOPE_CHANNEL_COUNT];
    int8_t scope_h_value_pos;
    uint8_t scope_h_value_timebase;
    uint8_t siggen_wave;
    uint8_t siggen_param;
    uint8_t siggen_duty_percent;
    uint8_t siggen_amp_tenths_v;
    uint8_t siggen_freq_unit;
    uint8_t siggen_running;
    uint32_t siggen_freq_hz;
    uint16_t scope_bias[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
    uint16_t scope_bias_rate[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
    uint8_t scope_math_mode;     // 0 = OFF, 1 = MATH (A±B)
    uint8_t scope_math_op;       // 0 = CH1 + CH2, 1 = CH1 - CH2, 2 = CH2 - CH1
    uint8_t scope_fft_src;       // 0 = OFF, 1 = CH1, 2 = CH2, 3 = XY mode, 4 = Bode
    uint8_t scope_math_selected; // 0..5, selected row in the scope math menu
    uint8_t scope_fft_window;    // 0 = HANN, 1 = HAMMING, 2 = BLACKMAN, 3 = RECTANGLE
    uint8_t scope_fft_display;   // 0 = NORMAL, 1 = AVERAGE, 2 = MAX HOLD
    uint8_t scope_hide_traces;   // 0=NONE, 1=CH1, 2=CH2, 3=ALL
    uint8_t siggen_sweep_mode;
    uint32_t siggen_sweep_ms;    // Range: 100ms to 10000ms (10 seconds)
    uint32_t siggen_sweep_start_hz;
    uint32_t siggen_sweep_stop_hz;
    uint8_t siggen_fm_mode;
    uint8_t siggen_fm_source;
    uint32_t siggen_fm_freq_hz;  // Low-rate MCU modulation: 1Hz to 10Hz
    uint32_t bode_start_hz;
    uint32_t bode_stop_hz;
    uint8_t bode_steps;

} settings_state_t;

uint8_t settings_load(settings_state_t *settings);
void settings_note(const settings_state_t *settings);
uint8_t settings_load_dmm_mode(uint8_t *mode);
void settings_note_dmm_mode(uint8_t mode);
void settings_flush(void);
void settings_factory_reset(void);
