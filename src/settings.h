#pragma once

#include <stdint.h>

enum {
    SETTINGS_START_MENU,
    SETTINGS_START_DMM,
    SETTINGS_START_SCOPE,
    SETTINGS_START_GEN,
    SETTINGS_START_COUNT,
    SETTINGS_LEVEL_COUNT = 5,
    SETTINGS_SCOPE_CHANNEL_COUNT = 2,
    SETTINGS_SCOPE_RANGE_COUNT = 9,
};

typedef struct {
    uint8_t dmm_mode;
    uint8_t beep_level;
    uint8_t brightness_level;
    uint8_t startup_screen;
    uint8_t last_screen;
    uint8_t sleep_enabled;
    uint16_t scope_bias[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
    uint16_t scope_bias_rate[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
} settings_state_t;

uint8_t settings_load(settings_state_t *settings);
void settings_note(const settings_state_t *settings);
uint8_t settings_load_dmm_mode(uint8_t *mode);
void settings_note_dmm_mode(uint8_t mode);
void settings_flush(void);
