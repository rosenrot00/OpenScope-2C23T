#include "siggen.h"

#include "board.h"
#include "fpga.h"

#include <stdint.h>

enum {
    FPGA_DDS_CLOCK_HZ = 100000000u,
    FPGA_FULL_SCALE_MV = 3300,
    SIGGEN_TIMING_SETTLE_MS = 2,
    SIGGEN_BUFFER_SETTLE_MS = 2,
};

static uint8_t last_enabled = 0xFFu;
static uint8_t last_wave = 0xFFu;
static uint8_t last_duty_percent = 0xFFu;
static uint8_t last_amplitude_tenths_v = 0xFFu;
static uint32_t last_freq_hz;
static uint8_t sample_buffer[FPGA_SAMPLE_COUNT];

static const int8_t sine_lut[64] = {
    0, 6, 13, 19, 25, 31, 36, 41,
    45, 49, 53, 56, 59, 61, 63, 64,
    64, 64, 63, 61, 59, 56, 53, 49,
    45, 41, 36, 31, 25, 19, 13, 6,
    0, -6, -13, -19, -25, -31, -36, -41,
    -45, -49, -53, -56, -59, -61, -63, -64,
    -64, -64, -63, -61, -59, -56, -53, -49,
    -45, -41, -36, -31, -25, -19, -13, -6,
};

static int16_t sine_interp_sample(uint16_t phase) {
    uint8_t index = (uint8_t)((phase >> 5) & 63u);
    uint8_t next = (uint8_t)((index + 1u) & 63u);
    uint8_t frac = (uint8_t)(phase & 31u);
    int16_t a = sine_lut[index];
    int16_t b = sine_lut[next];

    return (int16_t)(a + ((int16_t)(b - a) * frac + 16) / 32);
}

static void sample_buffer_fill(uint8_t value) {
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        sample_buffer[i] = value;
    }
}

static uint8_t full_scale_for_amp(uint8_t amplitude_tenths_v) {
    uint32_t full_scale_mv = (uint32_t)amplitude_tenths_v * 100u;

    if (amplitude_tenths_v < 1u) {
        full_scale_mv = 100u;
    } else if (amplitude_tenths_v > 33u) {
        full_scale_mv = FPGA_FULL_SCALE_MV;
    }

    return (uint8_t)(full_scale_mv * 255u / FPGA_FULL_SCALE_MV);
}

static uint8_t scale_sample(uint16_t value, uint16_t max_value, uint8_t full_scale) {
    return (uint8_t)((uint32_t)value * full_scale / max_value);
}

static uint16_t exp_rise_sample(uint16_t phase) {
    uint32_t x = (uint32_t)phase * 255u / (FPGA_SAMPLE_COUNT - 1u);
    return (uint16_t)((x * x * x) / (255u * 255u));
}

static uint16_t lorentz_sample(uint16_t phase) {
    int32_t x = (int32_t)phase - (int32_t)(FPGA_SAMPLE_COUNT / 2u);
    int32_t gamma = FPGA_SAMPLE_COUNT / 11;
    uint32_t g2 = (uint32_t)(gamma * gamma);
    uint32_t x2 = (uint32_t)(x * x);
    return (uint16_t)(255u * g2 / (x2 + g2));
}

static void build_waveform(uint8_t enabled, uint8_t wave, uint8_t duty_percent, uint8_t amplitude_tenths_v) {
    uint32_t rng = 0x13579BDFu;
    uint16_t split;
    uint8_t full_scale;

    if (!enabled) {
        sample_buffer_fill(0);
        return;
    }

    if (wave >= SIGGEN_WAVE_COUNT) {
        wave = SIGGEN_WAVE_SINE;
    }
    if (duty_percent < 1u) {
        duty_percent = 1u;
    } else if (duty_percent > 100u) {
        duty_percent = 100u;
    }

    full_scale = full_scale_for_amp(amplitude_tenths_v);
    split = (uint16_t)((uint32_t)FPGA_SAMPLE_COUNT * duty_percent / 100u);
    if (split < 1u) {
        split = 1u;
    } else if (split >= FPGA_SAMPLE_COUNT) {
        split = (uint16_t)(FPGA_SAMPLE_COUNT - 1u);
    }

    if (wave == SIGGEN_WAVE_DC) {
        sample_buffer_fill(full_scale);
        return;
    }
    if (wave == SIGGEN_WAVE_SQUARE) {
        if (duty_percent >= 100u) {
            sample_buffer_fill(full_scale);
            return;
        }
        uint16_t i = 0;
        for (; i < split; ++i) {
            sample_buffer[i] = full_scale;
        }
        for (; i < FPGA_SAMPLE_COUNT; ++i) {
            sample_buffer[i] = 0;
        }
        return;
    }

    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        uint16_t phase = (uint16_t)(i & (FPGA_SAMPLE_COUNT - 1u));
        uint16_t value;

        if (wave == SIGGEN_WAVE_SAW) {
            value = (uint16_t)((uint32_t)phase * 255u / (FPGA_SAMPLE_COUNT - 1u));
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_TRIANGLE) {
            if (phase < split) {
                value = (uint16_t)((uint32_t)phase * 255u / split);
            } else {
                value = (uint16_t)((uint32_t)(FPGA_SAMPLE_COUNT - phase) * 255u / (FPGA_SAMPLE_COUNT - split));
            }
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_HALF) {
            int16_t signed_sample = phase < (FPGA_SAMPLE_COUNT / 2u) ? sine_interp_sample(phase) : 0;
            value = signed_sample > 0 ? (uint16_t)signed_sample : 0u;
            sample_buffer[i] = scale_sample(value, 64u, full_scale);
        } else if (wave == SIGGEN_WAVE_FULL) {
            int16_t signed_sample = sine_interp_sample(phase);
            value = (uint16_t)(signed_sample < 0 ? -signed_sample : signed_sample);
            sample_buffer[i] = scale_sample(value, 64u, full_scale);
        } else if (wave == SIGGEN_WAVE_POS_STEP) {
            value = (uint16_t)((phase / (FPGA_SAMPLE_COUNT / 5u)) * 64u);
            if (value > 255u) {
                value = 255u;
            }
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_REV_STEP) {
            value = (uint16_t)(255u - (phase / (FPGA_SAMPLE_COUNT / 5u)) * 64u);
            if (value > 255u) {
                value = 0u;
            }
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_EXP_RISE) {
            sample_buffer[i] = scale_sample(exp_rise_sample(phase), 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_EXP_FALL) {
            sample_buffer[i] = scale_sample((uint16_t)(255u - exp_rise_sample(phase)), 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_NOISE) {
            rng = rng * 1664525u + 1013904223u;
            value = (uint16_t)((rng >> 24) & 0xFFu);
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_MULTI_AUDIO) {
            int16_t sample = (int16_t)(sine_interp_sample(phase) +
                                      sine_interp_sample((uint16_t)((phase * 3u) & (FPGA_SAMPLE_COUNT - 1u))) / 2 +
                                      sine_interp_sample((uint16_t)((phase * 5u) & (FPGA_SAMPLE_COUNT - 1u))) / 3);
            if (sample < -64) {
                sample = -64;
            } else if (sample > 64) {
                sample = 64;
            }
            sample_buffer[i] = scale_sample((uint16_t)(sample + 64), 128u, full_scale);
        } else if (wave == SIGGEN_WAVE_SINKER_PULSE) {
            if (phase < split) {
                value = (uint16_t)(255u - (uint32_t)phase * 255u / split);
            } else {
                value = 0u;
            }
            sample_buffer[i] = scale_sample(value, 255u, full_scale);
        } else if (wave == SIGGEN_WAVE_LORENTZ) {
            sample_buffer[i] = scale_sample(lorentz_sample(phase), 255u, full_scale);
        } else {
            int16_t signed_sample = sine_interp_sample(phase);
            value = (uint16_t)(signed_sample + 64);
            sample_buffer[i] = scale_sample(value, 128u, full_scale);
        }
    }
}

static uint32_t tuning_word_for(uint32_t freq_hz) {
    if (!freq_hz) {
        return 0;
    }
    if (freq_hz > 2000000u) {
        freq_hz = 2000000u;
    }
    return freq_hz * 43u - freq_hz / 20u;
}

void siggen_configure(uint8_t enabled, uint8_t wave, uint32_t freq_hz, uint8_t duty_percent, uint8_t amplitude_tenths_v) {
    uint8_t first_config;
    uint8_t timing_dirty;
    uint8_t buffer_dirty;

    enabled = enabled ? 1u : 0u;
    if (wave >= SIGGEN_WAVE_COUNT) {
        wave = SIGGEN_WAVE_SINE;
    }
    if (duty_percent < 1u) {
        duty_percent = 1u;
    } else if (duty_percent > 100u) {
        duty_percent = 100u;
    }
    if (amplitude_tenths_v < 1u) {
        amplitude_tenths_v = 1u;
    } else if (amplitude_tenths_v > 33u) {
        amplitude_tenths_v = 33u;
    }

    first_config = last_enabled == 0xFFu;
    if (enabled == last_enabled &&
        wave == last_wave &&
        freq_hz == last_freq_hz &&
        duty_percent == last_duty_percent &&
        amplitude_tenths_v == last_amplitude_tenths_v) {
        return;
    }

    fpga_init_once();

    timing_dirty = (uint8_t)(first_config ||
                             enabled != last_enabled ||
                             freq_hz != last_freq_hz);
    buffer_dirty = (uint8_t)(enabled &&
                             (first_config ||
                              !last_enabled ||
                              wave != last_wave ||
                              duty_percent != last_duty_percent ||
                              amplitude_tenths_v != last_amplitude_tenths_v));

    if (!enabled && (first_config || last_enabled)) {
        build_waveform(0, wave, duty_percent, amplitude_tenths_v);
        fpga_write_signal_buffer(sample_buffer, FPGA_SAMPLE_COUNT);
        delay_ms(SIGGEN_BUFFER_SETTLE_MS);
    }
    if (timing_dirty) {
        fpga_write_timing(tuning_word_for(enabled ? freq_hz : 0u), FPGA_SAMPLE_COUNT);
        delay_ms(SIGGEN_TIMING_SETTLE_MS);
    }
    if (buffer_dirty) {
        build_waveform(enabled, wave, duty_percent, amplitude_tenths_v);
        fpga_write_signal_buffer(sample_buffer, FPGA_SAMPLE_COUNT);
        delay_ms(SIGGEN_BUFFER_SETTLE_MS);
    }

    last_enabled = enabled;
    last_wave = wave;
    last_freq_hz = freq_hz;
    last_duty_percent = duty_percent;
    last_amplitude_tenths_v = amplitude_tenths_v;
}

uint8_t siggen_ready(void) {
    return fpga_ready();
}
