#include "settings.h"

#include "hw.h"

#include <stdint.h>

enum {
    SETTINGS_FLASH_BASE = 0x080FF800u,
    SETTINGS_FLASH_SIZE = 2048u,
    SETTINGS_RECORD_MAGIC = 0xC6000000u,
    SETTINGS_RECORD_MAGIC_MASK = 0xFF000000u,
    SETTINGS_SCOPE_CAL_LEGACY_MAGIC = 0xC8000000u,
    SETTINGS_SCOPE_CAL_MAGIC = 0xC7000000u,
    SETTINGS_SCOPE_CAL_MAGIC_MASK = 0xFF000000u,
    SETTINGS_LEGACY_MAGIC = 0xE1A60000u,
    SETTINGS_LEGACY_MAGIC_MASK = 0xFFFF0000u,
    SETTINGS_STATE_SCOPE_CORE = 0xC9u,
    SETTINGS_STATE_SCOPE_TRIGGER = 0xCAu,
    SETTINGS_STATE_SCOPE_CH0 = 0xCBu,
    SETTINGS_STATE_SCOPE_CH1 = 0xCCu,
    SETTINGS_STATE_SCOPE_MEASURE = 0xCDu,
    SETTINGS_STATE_SIGGEN_CORE = 0xCEu,
    SETTINGS_STATE_SIGGEN_FREQ_LO = 0xCFu,
    SETTINGS_STATE_SIGGEN_FREQ_HI = 0xD0u,
    SETTINGS_STATE_SCOPE_POS = 0xD1u,
    SETTINGS_STATE_MATH_MENU = 0xD2u,
    SETTINGS_STATE_SIGGEN_SWEEP_FM = 0xD3u,
    SETTINGS_STATE_BODE_START_LO = 0xD4u,
    SETTINGS_STATE_BODE_START_HI = 0xD5u,
    SETTINGS_STATE_BODE_STOP_LO = 0xD6u,
    SETTINGS_STATE_BODE_STOP_HI = 0xD7u,
    SETTINGS_STATE_BODE_CFG = 0xD8u,
    SETTINGS_STATE_SIGGEN_SWEEP_MS_LO = 0xD9u,
    SETTINGS_STATE_SIGGEN_SWEEP_MS_HI = 0xDAu,
    SETTINGS_STATE_SIGGEN_FM_FREQ_LO = 0xDBu,
    SETTINGS_STATE_SIGGEN_FM_FREQ_HI = 0xDCu,
    SETTINGS_STATE_SIGGEN_SWEEP_START_LO = 0xDDu,
    SETTINGS_STATE_SIGGEN_SWEEP_START_HI = 0xDEu,
    SETTINGS_STATE_SIGGEN_SWEEP_STOP_LO = 0xDFu,
    SETTINGS_STATE_SIGGEN_SWEEP_STOP_HI = 0xE0u,
    SETTINGS_SCOPE_BIAS_DEFAULT = 1861u,
    SETTINGS_SCOPE_BIAS_RATE_DEFAULT = 505u,
    SETTINGS_SCOPE_CAL_WORDS = SETTINGS_SCOPE_CHANNEL_COUNT * SETTINGS_SCOPE_RANGE_COUNT * 2,
    SETTINGS_STATE_WORDS = SETTINGS_STATE_SIGGEN_SWEEP_STOP_HI -
                           SETTINGS_STATE_SCOPE_CORE + 1u,
    SETTINGS_WRITE_BYTES = 4u * (1u + SETTINGS_SCOPE_CAL_WORDS + SETTINGS_STATE_WORDS),
    FLASH_BANK2_BASE = 0x08080000u,
    FLASH_STS_BSY = 1u << 0,
    FLASH_STS_PGERR = 1u << 2,
    FLASH_STS_WRPRTERR = 1u << 4,
    FLASH_STS_EOP = 1u << 5,
    FLASH_CTRL_PG = 1u << 0,
    FLASH_CTRL_PER = 1u << 1,
    FLASH_CTRL_STRT = 1u << 6,
    FLASH_CTRL_LOCK = 1u << 7,
};

static uint8_t settings_loaded;
static settings_state_t settings_cached;
static settings_state_t settings_pending;
static uint8_t settings_dirty;

static void flash_program_word(uint32_t addr, uint32_t value);

static void settings_scope_cal_defaults(settings_state_t *settings) {
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            settings->scope_bias[ch][range] = SETTINGS_SCOPE_BIAS_DEFAULT;
            settings->scope_bias_rate[ch][range] = SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
        }
    }
}

static void settings_defaults(settings_state_t *settings) {
    settings->dmm_mode = 0;
    settings->beep_level = 3;
    settings->brightness_level = 4;
    settings->startup_screen = SETTINGS_START_DMM;
    settings->last_screen = 0;
    settings->sleep_enabled = 0;
    settings->scope_timebase = SETTINGS_SCOPE_TIMEBASE_DEFAULT;
    settings->scope_display = 0;
    settings->scope_cursor_mode = 0;
    settings->scope_trigger_source = 1;
    settings->scope_trigger_mode = 0;
    settings->scope_trigger_edge = 0;
    settings->scope_trigger_level = 128;
    settings->scope_afterglow = 0;
    settings->scope_active_ch = 1;
    settings->scope_measure_param = 0;
    settings->scope_measure_visible = 1;
    settings->scope_h_value_pos = 0;
    settings->scope_h_value_timebase = SETTINGS_SCOPE_TIMEBASE_DEFAULT;
    settings->scope_math_mode = 0;
    settings->scope_math_op = 0;
    settings->scope_fft_src = 0;
    settings->scope_math_selected = 0;
    settings->scope_fft_window = 0;
    settings->scope_fft_display = 0;
    settings->scope_hide_traces = 0;
    settings->siggen_wave = SETTINGS_SIGGEN_DEFAULT_WAVE;
    settings->siggen_param = SETTINGS_SIGGEN_DEFAULT_PARAM;
    settings->siggen_duty_percent = SETTINGS_SIGGEN_DEFAULT_DUTY;
    settings->siggen_amp_tenths_v = SETTINGS_SIGGEN_DEFAULT_AMP_TENTHS;
    settings->siggen_freq_unit = 0;
    settings->siggen_running = 0;
    settings->siggen_freq_hz = SETTINGS_SIGGEN_DEFAULT_FREQ_HZ;
    settings->siggen_sweep_mode = 0;
    settings->siggen_sweep_ms = 1000;
    settings->siggen_sweep_start_hz = 0;
    settings->siggen_sweep_stop_hz = 4999;
    settings->siggen_fm_mode = 0;
    settings->siggen_fm_source = 0;
    settings->siggen_fm_freq_hz = 5;
    settings->bode_start_hz = 100u;
    settings->bode_stop_hz = 100000u;
    settings->bode_steps = 40u;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        settings->scope_ch_enabled[ch] = 1;
        settings->scope_probe_x10[ch] = 0;
        settings->scope_vdiv[ch] = 3;
        settings->scope_coupling_dc[ch] = 1;
        settings->scope_ch_pos[ch] = 0;
        settings->scope_measure_mask[ch] = 1;
    }
    settings_scope_cal_defaults(settings);
}

static void settings_copy(settings_state_t *dst, const settings_state_t *src) {
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
    dst->siggen_sweep_ms   = src->siggen_sweep_ms;
    dst->siggen_sweep_start_hz = src->siggen_sweep_start_hz;
    dst->siggen_sweep_stop_hz = src->siggen_sweep_stop_hz;
    dst->siggen_fm_mode    = src->siggen_fm_mode;
    dst->siggen_fm_source  = src->siggen_fm_source;
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

static uint8_t settings_equal(const settings_state_t *a, const settings_state_t *b) {
    if (a->dmm_mode != b->dmm_mode ||
        a->beep_level != b->beep_level ||
        a->brightness_level != b->brightness_level ||
        a->startup_screen != b->startup_screen ||
        a->last_screen != b->last_screen ||
        a->sleep_enabled != b->sleep_enabled ||
        a->scope_timebase != b->scope_timebase ||
        a->scope_display != b->scope_display ||
        a->scope_cursor_mode != b->scope_cursor_mode ||
        a->scope_trigger_source != b->scope_trigger_source ||
        a->scope_trigger_mode != b->scope_trigger_mode ||
        a->scope_trigger_edge != b->scope_trigger_edge ||
        a->scope_trigger_level != b->scope_trigger_level ||
        a->scope_afterglow != b->scope_afterglow ||
        a->scope_active_ch != b->scope_active_ch ||
        a->scope_measure_param != b->scope_measure_param ||
        a->scope_measure_visible != b->scope_measure_visible ||
        a->scope_h_value_pos != b->scope_h_value_pos ||
        a->scope_h_value_timebase != b->scope_h_value_timebase ||
        a->scope_math_mode != b->scope_math_mode ||
        a->scope_math_op != b->scope_math_op ||
        a->scope_fft_src != b->scope_fft_src ||
        a->scope_math_selected != b->scope_math_selected ||
        a->scope_fft_window != b->scope_fft_window ||
        a->scope_fft_display != b->scope_fft_display ||
        a->scope_hide_traces != b->scope_hide_traces ||
        a->siggen_wave != b->siggen_wave ||
        a->siggen_param != b->siggen_param ||
        a->siggen_duty_percent != b->siggen_duty_percent ||
        a->siggen_amp_tenths_v != b->siggen_amp_tenths_v ||
        a->siggen_freq_unit != b->siggen_freq_unit ||
        a->siggen_running != b->siggen_running ||
        a->siggen_freq_hz != b->siggen_freq_hz ||
        a->siggen_sweep_mode != b->siggen_sweep_mode ||
        a->siggen_sweep_ms != b->siggen_sweep_ms ||
        a->siggen_sweep_start_hz != b->siggen_sweep_start_hz ||
        a->siggen_sweep_stop_hz != b->siggen_sweep_stop_hz ||
        a->siggen_fm_mode != b->siggen_fm_mode ||
        a->siggen_fm_source != b->siggen_fm_source ||
        a->siggen_fm_freq_hz != b->siggen_fm_freq_hz ||
        a->bode_start_hz != b->bode_start_hz ||
        a->bode_stop_hz != b->bode_stop_hz ||
        a->bode_steps != b->bode_steps) {
        return 0;
    }
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        if (a->scope_ch_enabled[ch] != b->scope_ch_enabled[ch] ||
            a->scope_probe_x10[ch] != b->scope_probe_x10[ch] ||
            a->scope_vdiv[ch] != b->scope_vdiv[ch] ||
            a->scope_coupling_dc[ch] != b->scope_coupling_dc[ch] ||
            a->scope_ch_pos[ch] != b->scope_ch_pos[ch] ||
            a->scope_measure_mask[ch] != b->scope_measure_mask[ch]) {
            return 0;
        }
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            if (a->scope_bias[ch][range] != b->scope_bias[ch][range] ||
                a->scope_bias_rate[ch][range] != b->scope_bias_rate[ch][range]) {
                return 0;
            }
        }
    }
    return 1;
}

static void settings_clamp(settings_state_t *settings) {
    if (settings->beep_level >= SETTINGS_LEVEL_COUNT) {
        settings->beep_level = 3;
    }
    if (settings->brightness_level >= SETTINGS_LEVEL_COUNT) {
        settings->brightness_level = 4;
    }
    if (settings->startup_screen >= SETTINGS_START_COUNT) {
        settings->startup_screen = SETTINGS_START_DMM;
    }
    if (settings->last_screen > 2u) {
        settings->last_screen = 0;
    }
    if (settings->sleep_enabled >= SETTINGS_SLEEP_COUNT) {
        settings->sleep_enabled = 0;
    }
    if (settings->scope_timebase >= SETTINGS_SCOPE_TIMEBASE_COUNT) {
        settings->scope_timebase = SETTINGS_SCOPE_TIMEBASE_DEFAULT;
    }
    if (settings->scope_display >= SETTINGS_SCOPE_DISPLAY_COUNT) {
        settings->scope_display = 0;
    }
    if (settings->scope_cursor_mode >= SETTINGS_SCOPE_CURSOR_COUNT) {
        settings->scope_cursor_mode = 0;
    }
    if (settings->scope_trigger_source < 1u || settings->scope_trigger_source > 2u) {
        settings->scope_trigger_source = 1;
    }
    if (settings->scope_trigger_mode >= SETTINGS_SCOPE_TRIGGER_COUNT) {
        settings->scope_trigger_mode = 0;
    }
    settings->scope_trigger_edge = settings->scope_trigger_edge ? 1u : 0u;
    if (settings->scope_trigger_level < 12u) {
        settings->scope_trigger_level = 12;
    } else if (settings->scope_trigger_level > 243u) {
        settings->scope_trigger_level = 243;
    }
    settings->scope_afterglow = settings->scope_afterglow ? 1u : 0u;
    if (settings->scope_active_ch < 1u || settings->scope_active_ch > 2u) {
        settings->scope_active_ch = 1;
    }
    if (settings->scope_measure_param >= SETTINGS_SCOPE_MEASURE_COUNT) {
        settings->scope_measure_param = 0;
    }
    settings->scope_measure_visible = settings->scope_measure_visible ? 1u : 0u;
    if (settings->scope_h_value_pos < -120) {
        settings->scope_h_value_pos = -120;
    } else if (settings->scope_h_value_pos > 120) {
        settings->scope_h_value_pos = 120;
    }
    if (settings->scope_h_value_timebase >= SETTINGS_SCOPE_TIMEBASE_COUNT) {
        settings->scope_h_value_timebase = settings->scope_timebase;
    }
    if (settings->scope_math_mode >= 2u) {
        settings->scope_math_mode = 0u; // Default to OFF
    }
    if (settings->scope_math_op >= 3u) {
        settings->scope_math_op = 0u;   // Default to CH1 + CH2
    }
    if (settings->scope_fft_src >= 5u) {
        settings->scope_fft_src = 0u;
    }
    if (settings->scope_math_selected >= 6u ||
        (settings->scope_fft_src == 4u && settings->scope_math_selected >= 5u)) {
        settings->scope_math_selected = 0u; // Default to Row 0
    }
    if (settings->scope_fft_window >= 4u) {
        settings->scope_fft_window = 0u; // Default to Hann window for clean spectrums
    }
    if (settings->scope_fft_display >= 3u) {
        settings->scope_fft_display = 0u; // Default to Normal real-time tracking
    }
    if (settings->scope_hide_traces >= 4u) {
        settings->scope_hide_traces = 0u;
    }
    if (settings->siggen_wave >= SETTINGS_SIGGEN_WAVE_COUNT) {
        settings->siggen_wave = SETTINGS_SIGGEN_DEFAULT_WAVE;
    }
    if (settings->siggen_param >= SETTINGS_SIGGEN_PARAM_COUNT) {
        settings->siggen_param = SETTINGS_SIGGEN_DEFAULT_PARAM;
    }
    if (settings->siggen_duty_percent < 1u || settings->siggen_duty_percent > 100u) {
        settings->siggen_duty_percent = SETTINGS_SIGGEN_DEFAULT_DUTY;
    }
    if (settings->siggen_amp_tenths_v < 1u || settings->siggen_amp_tenths_v > SETTINGS_SIGGEN_DEFAULT_AMP_TENTHS) {
        settings->siggen_amp_tenths_v = SETTINGS_SIGGEN_DEFAULT_AMP_TENTHS;
    }
    if (settings->siggen_freq_unit >= SETTINGS_SIGGEN_FREQ_UNIT_COUNT) {
        settings->siggen_freq_unit = 0;
    }
    settings->siggen_running = settings->siggen_running ? 1u : 0u;
    if (settings->siggen_freq_hz < 1u || settings->siggen_freq_hz > SETTINGS_SIGGEN_MAX_FREQ_HZ) {
        settings->siggen_freq_hz = SETTINGS_SIGGEN_DEFAULT_FREQ_HZ;
    }
    if (settings->siggen_sweep_mode >= 3) settings->siggen_sweep_mode = 0;
    if (settings->siggen_sweep_ms < 100) settings->siggen_sweep_ms = 100;
    if (settings->siggen_sweep_ms > 9999) settings->siggen_sweep_ms = 9999;
    
    if (settings->siggen_sweep_start_hz < 1) settings->siggen_sweep_start_hz = 1;
    if (settings->siggen_sweep_start_hz > SETTINGS_SIGGEN_MAX_FREQ_HZ) settings->siggen_sweep_start_hz = SETTINGS_SIGGEN_MAX_FREQ_HZ;

    if (settings->siggen_sweep_stop_hz < 1) settings->siggen_sweep_stop_hz = 1;
    if (settings->siggen_sweep_stop_hz > SETTINGS_SIGGEN_MAX_FREQ_HZ) settings->siggen_sweep_stop_hz = SETTINGS_SIGGEN_MAX_FREQ_HZ;
    
    if (settings->siggen_fm_mode >= 2) settings->siggen_fm_mode = 0;
    if (settings->siggen_fm_source >= 3) settings->siggen_fm_source = 0;
    
    if (settings->siggen_fm_freq_hz < 1) settings->siggen_fm_freq_hz = 1;
    if (settings->siggen_fm_freq_hz > SETTINGS_SIGGEN_FM_MAX_HZ) settings->siggen_fm_freq_hz = SETTINGS_SIGGEN_FM_MAX_HZ;
    if (settings->bode_start_hz < 10u) {
        settings->bode_start_hz = 10u;
    } else if (settings->bode_start_hz > SETTINGS_SIGGEN_MAX_FREQ_HZ) {
        settings->bode_start_hz = SETTINGS_SIGGEN_MAX_FREQ_HZ;
    }
    if (settings->bode_stop_hz < 100u) {
        settings->bode_stop_hz = 100u;
    } else if (settings->bode_stop_hz > SETTINGS_SIGGEN_MAX_FREQ_HZ) {
        settings->bode_stop_hz = SETTINGS_SIGGEN_MAX_FREQ_HZ;
    }
    if (settings->bode_steps < SETTINGS_BODE_MIN_STEPS) {
        settings->bode_steps = SETTINGS_BODE_MIN_STEPS;
    } else if (settings->bode_steps > SETTINGS_BODE_MAX_STEPS) {
        settings->bode_steps = SETTINGS_BODE_MAX_STEPS;
    }
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        settings->scope_ch_enabled[ch] = settings->scope_ch_enabled[ch] ? 1u : 0u;
        settings->scope_probe_x10[ch] = settings->scope_probe_x10[ch] ? 1u : 0u;
        if (settings->scope_vdiv[ch] >= SETTINGS_SCOPE_RANGE_COUNT) {
            settings->scope_vdiv[ch] = 3;
        }
        settings->scope_coupling_dc[ch] = settings->scope_coupling_dc[ch] ? 1u : 0u;
        if (settings->scope_ch_pos[ch] < -120) {
            settings->scope_ch_pos[ch] = -120;
        } else if (settings->scope_ch_pos[ch] > 120) {
            settings->scope_ch_pos[ch] = 120;
        }
        settings->scope_measure_mask[ch] &= (uint8_t)((1u << SETTINGS_SCOPE_MEASURE_COUNT) - 1u);
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            if (settings->scope_bias[ch][range] > 4095u) {
                settings->scope_bias[ch][range] = SETTINGS_SCOPE_BIAS_DEFAULT;
            }
            if (!settings->scope_bias_rate[ch][range] || settings->scope_bias_rate[ch][range] > 4095u) {
                settings->scope_bias_rate[ch][range] = SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
            }
        }
    }
}

static uint8_t settings_checksum(uint16_t payload) {
    return (uint8_t)((payload & 0xFFu) ^ (payload >> 8) ^ 0xA5u);
}

static uint8_t settings_state_checksum(uint8_t type, uint16_t payload) {
    return (uint8_t)(type ^ (payload & 0xFFu) ^ (payload >> 8) ^ 0x3Du);
}

static uint16_t settings_payload(const settings_state_t *settings) {
    return (uint16_t)((settings->dmm_mode & 0x0Fu) |
                      ((settings->beep_level & 0x07u) << 4) |
                      ((settings->brightness_level & 0x07u) << 7) |
                      ((settings->startup_screen & 0x03u) << 10) |
                      ((settings->last_screen & 0x03u) << 12) |
                      ((settings->sleep_enabled & 0x03u) << 14));
}

static uint32_t settings_record_for(const settings_state_t *settings) {
    uint16_t payload = settings_payload(settings);
    return SETTINGS_RECORD_MAGIC | ((uint32_t)payload << 8) | settings_checksum(payload);
}

static uint32_t settings_state_record(uint8_t type, uint16_t payload) {
    return ((uint32_t)type << 24) | ((uint32_t)payload << 8) | settings_state_checksum(type, payload);
}

static uint8_t settings_encode_pos(int8_t pos) {
    int16_t value = (int16_t)pos + 128;

    if (value < 0) {
        value = 0;
    } else if (value > 255) {
        value = 255;
    }
    return (uint8_t)value;
}

static int8_t settings_decode_pos(uint8_t value) {
    return (int8_t)((int16_t)value - 128);
}

static uint8_t settings_scope_cal_legacy_checksum(uint16_t payload) {
    return (uint8_t)((payload & 0xFFu) ^ (payload >> 8) ^ 0x5Cu);
}

static uint8_t settings_scope_cal_checksum(uint32_t payload) {
    return (uint8_t)((payload ^ (payload >> 6) ^ (payload >> 12) ^ 0x15u) & 0x3Fu);
}

static uint32_t settings_scope_cal_record(uint8_t type, uint8_t ch, uint8_t range, uint16_t value) {
    uint32_t payload = ((uint32_t)(type & 1u) << 17) |
                       ((uint32_t)(ch & 1u) << 16) |
                       ((uint32_t)(range & 0x0Fu) << 12) |
                       (value & 0x0FFFu);
    return SETTINGS_SCOPE_CAL_MAGIC | (payload << 6) | settings_scope_cal_checksum(payload);
}

static uint8_t settings_scope_cal_record_valid(uint32_t record, settings_state_t *settings) {
    uint8_t type;
    uint8_t ch;
    uint8_t range;
    uint16_t value;

    if ((record & SETTINGS_SCOPE_CAL_MAGIC_MASK) == SETTINGS_SCOPE_CAL_LEGACY_MAGIC) {
        uint16_t payload = (uint16_t)(record >> 8);
        uint8_t checksum = (uint8_t)record;
        if (checksum != settings_scope_cal_legacy_checksum(payload)) {
            return 0;
        }
        type = (uint8_t)((payload >> 15) & 1u);
        ch = (uint8_t)((payload >> 14) & 1u);
        range = (uint8_t)((payload >> 12) & 3u);
        value = (uint16_t)(payload & 0x0FFFu);
    } else if ((record & SETTINGS_SCOPE_CAL_MAGIC_MASK) == SETTINGS_SCOPE_CAL_MAGIC) {
        uint32_t payload = (record >> 6) & 0x3FFFFu;
        uint8_t checksum = (uint8_t)(record & 0x3Fu);
        if (checksum != settings_scope_cal_checksum(payload)) {
            return 0;
        }
        type = (uint8_t)((payload >> 17) & 1u);
        ch = (uint8_t)((payload >> 16) & 1u);
        range = (uint8_t)((payload >> 12) & 0x0Fu);
        value = (uint16_t)(payload & 0x0FFFu);
    } else {
        return 0;
    }

    if (ch >= SETTINGS_SCOPE_CHANNEL_COUNT || range >= SETTINGS_SCOPE_RANGE_COUNT) {
        return 0;
    }
    if (type) {
        settings->scope_bias_rate[ch][range] = value ? value : SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
    } else {
        settings->scope_bias[ch][range] = value;
    }
    return 1;
}

static uint8_t settings_record_valid(uint32_t record, settings_state_t *settings) {
    uint16_t payload = (uint16_t)(record >> 8);
    uint8_t checksum = (uint8_t)record;

    if ((record & SETTINGS_RECORD_MAGIC_MASK) != SETTINGS_RECORD_MAGIC) {
        return 0;
    }
    if (checksum != settings_checksum(payload)) {
        return 0;
    }

    settings_defaults(settings);
    settings->dmm_mode = (uint8_t)(payload & 0x0Fu);
    settings->beep_level = (uint8_t)((payload >> 4) & 0x07u);
    settings->brightness_level = (uint8_t)((payload >> 7) & 0x07u);
    settings->startup_screen = (uint8_t)((payload >> 10) & 0x03u);
    settings->last_screen = (uint8_t)((payload >> 12) & 0x03u);
    settings->sleep_enabled = (uint8_t)((payload >> 14) & 0x03u);
    settings_clamp(settings);
    return 1;
}

static uint8_t settings_state_record_valid(uint32_t record, settings_state_t *settings) {
    uint8_t type = (uint8_t)(record >> 24);
    uint16_t payload = (uint16_t)(record >> 8);
    uint8_t checksum = (uint8_t)record;
    uint8_t ch;

    if (checksum != settings_state_checksum(type, payload)) {
        return 0;
    }

    if (type == SETTINGS_STATE_SCOPE_CORE) {
        settings->scope_timebase = (uint8_t)(payload & 0x1Fu);
        settings->scope_display = (uint8_t)((payload >> 5) & 0x03u);
        settings->scope_cursor_mode = (uint8_t)((payload >> 7) & 0x03u);
        settings->scope_trigger_source = (payload & (1u << 9)) ? 2u : 1u;
        settings->scope_trigger_mode = (uint8_t)((payload >> 10) & 0x03u);
        settings->scope_trigger_edge = (uint8_t)((payload >> 12) & 0x01u);
        settings->scope_afterglow = (uint8_t)((payload >> 13) & 0x01u);
        settings->scope_measure_visible = (uint8_t)((payload >> 14) & 0x01u);
        settings->scope_active_ch = (payload & (1u << 15)) ? 2u : 1u;
    } else if (type == SETTINGS_STATE_SCOPE_TRIGGER) {
        settings->scope_trigger_level = (uint8_t)(payload & 0xFFu);
        settings->scope_measure_param = (uint8_t)((payload >> 8) & 0x07u);
    } else if (type == SETTINGS_STATE_SCOPE_CH0 || type == SETTINGS_STATE_SCOPE_CH1) {
        ch = type == SETTINGS_STATE_SCOPE_CH1 ? 1u : 0u;
        settings->scope_ch_enabled[ch] = (uint8_t)(payload & 0x01u);
        settings->scope_probe_x10[ch] = (uint8_t)((payload >> 1) & 0x01u);
        settings->scope_vdiv[ch] = (uint8_t)((payload >> 2) & 0x0Fu);
        settings->scope_coupling_dc[ch] = (uint8_t)((payload >> 6) & 0x01u);
        settings->scope_ch_pos[ch] = settings_decode_pos((uint8_t)(payload >> 7));
    } else if (type == SETTINGS_STATE_SCOPE_MEASURE) {
        settings->scope_measure_mask[0] = (uint8_t)(payload & 0x3Fu);
        settings->scope_measure_mask[1] = (uint8_t)((payload >> 6) & 0x3Fu);
    } else if (type == SETTINGS_STATE_SIGGEN_CORE) {
        settings->siggen_wave = (uint8_t)(payload & 0x1Fu);
        settings->siggen_param = (uint8_t)((payload >> 5) & 0x03u);
        settings->siggen_duty_percent = (uint8_t)((payload >> 7) & 0x7Fu);
        settings->siggen_running = (uint8_t)((payload >> 14) & 0x01u);
        settings->siggen_freq_unit = (uint8_t)((payload >> 15) & 0x01u);
    } else if (type == SETTINGS_STATE_SIGGEN_FREQ_LO) {
        settings->siggen_freq_hz = (settings->siggen_freq_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_SIGGEN_FREQ_HI) {
        settings->siggen_freq_hz = (settings->siggen_freq_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_SCOPE_POS) {
        settings->scope_h_value_pos = settings_decode_pos((uint8_t)payload);
        settings->scope_h_value_timebase = (uint8_t)((payload >> 8) & 0x1Fu);
        settings->siggen_freq_unit |= (uint8_t)((payload >> 12) & 0x02u);
    } else if (type == SETTINGS_STATE_MATH_MENU) {
        settings->scope_math_mode     = (uint8_t)(payload & 0x01u);
        settings->scope_math_op       = (uint8_t)((payload >> 1) & 0x03u);
        settings->scope_fft_src       = (uint8_t)(((payload >> 3) & 0x03u) |
                                                  ((payload >> 12) & 0x04u));
        settings->scope_math_selected = (uint8_t)((payload >> 5) & 0x07u);
        settings->scope_fft_window    = (uint8_t)((payload >> 8) & 0x03u);
        settings->scope_fft_display   = (uint8_t)((payload >> 10) & 0x03u);
        settings->scope_hide_traces   = (uint8_t)((payload >> 12) & 0x03u);
    } else if (type == SETTINGS_STATE_BODE_START_LO) {
        settings->bode_start_hz = (settings->bode_start_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_BODE_START_HI) {
        settings->bode_start_hz = (settings->bode_start_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_BODE_STOP_LO) {
        settings->bode_stop_hz = (settings->bode_stop_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_BODE_STOP_HI) {
        settings->bode_stop_hz = (settings->bode_stop_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_BODE_CFG) {
        settings->bode_steps = (uint8_t)(payload & 0xFFu);
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_FM) {
        settings->siggen_sweep_mode = (uint8_t)(payload & 3u);
        settings->siggen_fm_mode    = (uint8_t)((payload >> 2) & 1u);
        settings->siggen_fm_source  = (uint8_t)((payload >> 3) & 3u);
        return 1;
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_MS_LO) {
        settings->siggen_sweep_ms = (settings->siggen_sweep_ms & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_MS_HI) {
        settings->siggen_sweep_ms = (settings->siggen_sweep_ms & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_SIGGEN_FM_FREQ_LO) {
        settings->siggen_fm_freq_hz = (settings->siggen_fm_freq_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_SIGGEN_FM_FREQ_HI) {
        settings->siggen_fm_freq_hz = (settings->siggen_fm_freq_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_START_LO) {
        settings->siggen_sweep_start_hz = (settings->siggen_sweep_start_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_START_HI) {
        settings->siggen_sweep_start_hz = (settings->siggen_sweep_start_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_STOP_LO) {
        settings->siggen_sweep_stop_hz = (settings->siggen_sweep_stop_hz & 0xFFFF0000u) | payload;
    } else if (type == SETTINGS_STATE_SIGGEN_SWEEP_STOP_HI) {
        settings->siggen_sweep_stop_hz = (settings->siggen_sweep_stop_hz & 0x0000FFFFu) | ((uint32_t)payload << 16);
    } else {
        return 0;
    }

    return 1;
}

static void settings_write_state_records(uint32_t *addr, const settings_state_t *settings) {
    uint16_t payload;

    payload = (uint16_t)((settings->scope_timebase & 0x1Fu) |
                         ((settings->scope_display & 0x03u) << 5) |
                         ((settings->scope_cursor_mode & 0x03u) << 7) |
                         ((settings->scope_trigger_source == 2u ? 1u : 0u) << 9) |
                         ((settings->scope_trigger_mode & 0x03u) << 10) |
                         ((settings->scope_trigger_edge & 0x01u) << 12) |
                         ((settings->scope_afterglow & 0x01u) << 13) |
                         ((settings->scope_measure_visible & 0x01u) << 14) |
                         ((settings->scope_active_ch == 2u ? 1u : 0u) << 15));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SCOPE_CORE, payload));
    *addr += 4u;

    payload = (uint16_t)((settings->scope_trigger_level & 0xFFu) |
                         ((settings->scope_measure_param & 0x07u) << 8));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SCOPE_TRIGGER, payload));
    *addr += 4u;

    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        payload = (uint16_t)((settings->scope_ch_enabled[ch] & 0x01u) |
                             ((settings->scope_probe_x10[ch] & 0x01u) << 1) |
                             ((settings->scope_vdiv[ch] & 0x0Fu) << 2) |
                             ((settings->scope_coupling_dc[ch] & 0x01u) << 6) |
                             ((uint16_t)settings_encode_pos(settings->scope_ch_pos[ch]) << 7));
        flash_program_word(*addr,
                           settings_state_record(ch ? SETTINGS_STATE_SCOPE_CH1 : SETTINGS_STATE_SCOPE_CH0,
                                                 payload));
        *addr += 4u;
    }

    payload = (uint16_t)((settings->scope_measure_mask[0] & 0x3Fu) |
                         ((settings->scope_measure_mask[1] & 0x3Fu) << 6));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SCOPE_MEASURE, payload));
    *addr += 4u;

    payload = (uint16_t)((settings->siggen_wave & 0x1Fu) |
                         ((settings->siggen_param & 0x03u) << 5) |
                         ((settings->siggen_duty_percent & 0x7Fu) << 7) |
                         ((settings->siggen_running & 0x01u) << 14) |
                         ((settings->siggen_freq_unit & 0x01u) << 15));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_CORE, payload));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_FREQ_LO,
                                                    (uint16_t)settings->siggen_freq_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_FREQ_HI,
                                                    (uint16_t)(settings->siggen_freq_hz >> 16)));
    *addr += 4u;

    payload = (uint16_t)((settings_encode_pos(settings->scope_h_value_pos) & 0xFFu) |
                         ((settings->scope_h_value_timebase & 0x1Fu) << 8) |
                         ((settings->siggen_freq_unit & 0x02u) << 12));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SCOPE_POS, payload));
    *addr += 4u;

    payload = (uint16_t)((settings->scope_math_mode & 0x01u) |
                         ((settings->scope_math_op & 0x03u) << 1) |
                         ((settings->scope_fft_src & 0x03u) << 3) |
                         ((settings->scope_math_selected & 0x07u) << 5) |
                         ((settings->scope_fft_window & 0x03u) << 8) |
                         ((settings->scope_fft_display & 0x03u) << 10) |
                         ((settings->scope_hide_traces & 0x03u) << 12) |
                         ((settings->scope_fft_src & 0x04u) << 12));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_MATH_MENU, payload));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_BODE_START_LO,
                                                    (uint16_t)settings->bode_start_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_BODE_START_HI,
                                                    (uint16_t)(settings->bode_start_hz >> 16)));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_BODE_STOP_LO,
                                                    (uint16_t)settings->bode_stop_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_BODE_STOP_HI,
                                                    (uint16_t)(settings->bode_stop_hz >> 16)));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_BODE_CFG,
                                                    (uint16_t)settings->bode_steps));
    *addr += 4u;

    payload = (uint16_t)((settings->siggen_sweep_mode & 3u) |
                         ((settings->siggen_fm_mode & 1u) << 2) |
                         ((settings->siggen_fm_source & 3u) << 3));
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_FM, payload));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_MS_LO,
                                                    (uint16_t)settings->siggen_sweep_ms));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_MS_HI,
                                                    (uint16_t)(settings->siggen_sweep_ms >> 16)));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_FM_FREQ_LO,
                                                    (uint16_t)settings->siggen_fm_freq_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_FM_FREQ_HI,
                                                    (uint16_t)(settings->siggen_fm_freq_hz >> 16)));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_START_LO,
                                                    (uint16_t)settings->siggen_sweep_start_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_START_HI,
                                                    (uint16_t)(settings->siggen_sweep_start_hz >> 16)));
    *addr += 4u;

    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_STOP_LO,
                                                    (uint16_t)settings->siggen_sweep_stop_hz));
    *addr += 4u;
    flash_program_word(*addr, settings_state_record(SETTINGS_STATE_SIGGEN_SWEEP_STOP_HI,
                                                    (uint16_t)(settings->siggen_sweep_stop_hz >> 16)));
    *addr += 4u;
}

static uint8_t settings_legacy_record_valid(uint32_t record, uint8_t *mode) {
    uint8_t value = (uint8_t)(record >> 8);
    uint8_t checksum = (uint8_t)record;

    if ((record & SETTINGS_LEGACY_MAGIC_MASK) != SETTINGS_LEGACY_MAGIC) {
        return 0;
    }
    if (checksum != (uint8_t)(value ^ 0x5Au)) {
        return 0;
    }
    *mode = value;
    return 1;
}

uint8_t settings_load(settings_state_t *settings) {
    uint8_t found = 0;
    settings_state_t latest;

    settings_defaults(&latest);
    for (uint32_t addr = SETTINGS_FLASH_BASE; addr < SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE; addr += 4u) {
        uint32_t record = REG32(addr);
        settings_state_t value;
        uint8_t legacy_mode;

        if (record == 0xFFFFFFFFu) {
            break;
        }
        if (settings_record_valid(record, &value)) {
            settings_copy(&latest, &value);
            found = 1;
        } else if (settings_scope_cal_record_valid(record, &latest)) {
            found = 1;
        } else if (settings_state_record_valid(record, &latest)) {
            found = 1;
        } else if (settings_legacy_record_valid(record, &legacy_mode)) {
            latest.dmm_mode = legacy_mode;
            found = 1;
        }
    }

    settings_clamp(&latest);
    settings_copy(&settings_cached, &latest);
    settings_copy(&settings_pending, &latest);
    settings_loaded = 1;
    settings_dirty = 0;
    if (settings) {
        settings_copy(settings, &latest);
    }
    return found;
}

static void settings_ensure_loaded(void) {
    if (!settings_loaded) {
        (void)settings_load(0);
    }
}

static volatile uint32_t *flash_sts_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x4Cu) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x0Cu);
}

static volatile uint32_t *flash_ctrl_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x50u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x10u);
}

static volatile uint32_t *flash_addr_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x54u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x14u);
}

static volatile uint32_t *flash_keyr_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x44u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x04u);
}

static void flash_wait(uint32_t addr) {
    volatile uint32_t *sts = flash_sts_reg(addr);

    while (*sts & FLASH_STS_BSY) {
    }
}

static void flash_unlock(uint32_t addr) {
    volatile uint32_t *ctrl = flash_ctrl_reg(addr);
    volatile uint32_t *keyr = flash_keyr_reg(addr);

    if (*ctrl & FLASH_CTRL_LOCK) {
        *keyr = 0x45670123u;
        *keyr = 0xCDEF89ABu;
    }
}

static void flash_lock(uint32_t addr) {
    *flash_ctrl_reg(addr) |= FLASH_CTRL_LOCK;
}

static void flash_clear_status(uint32_t addr) {
    *flash_sts_reg(addr) = FLASH_STS_EOP | FLASH_STS_PGERR | FLASH_STS_WRPRTERR;
}

static void flash_erase_settings_page(void) {
    volatile uint32_t *ctrl = flash_ctrl_reg(SETTINGS_FLASH_BASE);
    volatile uint32_t *addr_reg = flash_addr_reg(SETTINGS_FLASH_BASE);

    flash_wait(SETTINGS_FLASH_BASE);
    flash_clear_status(SETTINGS_FLASH_BASE);
    *ctrl |= FLASH_CTRL_PER;
    *addr_reg = SETTINGS_FLASH_BASE;
    *ctrl |= FLASH_CTRL_STRT;
    flash_wait(SETTINGS_FLASH_BASE);
    *ctrl &= ~FLASH_CTRL_PER;
    flash_clear_status(SETTINGS_FLASH_BASE);
}

static void flash_program_halfword(uint32_t addr, uint16_t value) {
    volatile uint32_t *ctrl = flash_ctrl_reg(addr);

    flash_wait(addr);
    flash_clear_status(addr);
    *ctrl |= FLASH_CTRL_PG;
    REG16(addr) = value;
    flash_wait(addr);
    *ctrl &= ~FLASH_CTRL_PG;
    flash_clear_status(addr);
}

static void flash_program_word(uint32_t addr, uint32_t value) {
    flash_program_halfword(addr, (uint16_t)value);
    flash_program_halfword(addr + 2u, (uint16_t)(value >> 16));
}

static uint32_t settings_first_free_addr(void) {
    for (uint32_t addr = SETTINGS_FLASH_BASE; addr < SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE; addr += 4u) {
        if (REG32(addr) == 0xFFFFFFFFu) {
            return addr;
        }
    }
    return 0;
}

static void settings_write(const settings_state_t *settings) {
    uint32_t addr;
    uint32_t record = settings_record_for(settings);

    __asm__ volatile("cpsid i" ::: "memory");
    flash_unlock(SETTINGS_FLASH_BASE);
    addr = settings_first_free_addr();
    if (!addr || addr + SETTINGS_WRITE_BYTES > SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE) {
        flash_erase_settings_page();
        addr = SETTINGS_FLASH_BASE;
    }
    flash_program_word(addr, record);
    addr += 4u;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            flash_program_word(addr, settings_scope_cal_record(0, ch, range, settings->scope_bias[ch][range]));
            addr += 4u;
            flash_program_word(addr, settings_scope_cal_record(1, ch, range, settings->scope_bias_rate[ch][range]));
            addr += 4u;
        }
    }
    settings_write_state_records(&addr, settings);
    flash_lock(SETTINGS_FLASH_BASE);
    __asm__ volatile("cpsie i" ::: "memory");

    settings_copy(&settings_cached, settings);
    settings_loaded = 1;
}

void settings_note(const settings_state_t *settings) {
    settings_ensure_loaded();
    settings_copy(&settings_pending, settings);
    settings_clamp(&settings_pending);
    settings_dirty = settings_equal(&settings_pending, &settings_cached) ? 0u : 1u;
}

uint8_t settings_load_dmm_mode(uint8_t *mode) {
    settings_state_t settings;
    uint8_t found = settings_load(&settings);
    if (mode) {
        *mode = settings.dmm_mode;
    }
    return found;
}

void settings_note_dmm_mode(uint8_t mode) {
    settings_ensure_loaded();
    settings_pending.dmm_mode = mode;
    settings_dirty = settings_equal(&settings_pending, &settings_cached) ? 0u : 1u;
}

void settings_flush(void) {
    settings_ensure_loaded();
    if (!settings_dirty) {
        return;
    }

    settings_write(&settings_pending);
    settings_dirty = 0;
}
