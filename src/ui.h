#pragma once

#include <stdint.h>

void ui_init(void);
void ui_handle_keys(uint32_t events);
void ui_tick(uint32_t elapsed_ms);
void ui_set_load_sample(uint32_t active_ticks, uint32_t idle_ticks);
void ui_dmm_measurement_updated(void);
uint8_t ui_consume_beep_preview(void);
uint8_t ui_auto_sleep_due(void);
uint8_t ui_diode_beep_enabled(void);
uint8_t ui_live_beep_enabled(void);
void ui_set_live_wire_detected(uint8_t detected);
