#pragma once

#include <stdint.h>

enum {
    SIGGEN_WAVE_SINE,
    SIGGEN_WAVE_SQUARE,
    SIGGEN_WAVE_SAW,
    SIGGEN_WAVE_HALF,
    SIGGEN_WAVE_FULL,
    SIGGEN_WAVE_POS_STEP,
    SIGGEN_WAVE_REV_STEP,
    SIGGEN_WAVE_EXP_RISE,
    SIGGEN_WAVE_EXP_FALL,
    SIGGEN_WAVE_DC,
    SIGGEN_WAVE_MULTI_AUDIO,
    SIGGEN_WAVE_SINKER_PULSE,
    SIGGEN_WAVE_LORENTZ,
    SIGGEN_WAVE_TRIANGLE,
    SIGGEN_WAVE_NOISE,
    SIGGEN_WAVE_COUNT,
};

void siggen_configure(uint8_t enabled, uint8_t wave, uint32_t freq_hz, uint8_t duty_percent, uint8_t amplitude_tenths_v);
uint8_t siggen_ready(void);
