#pragma once

#include <stdint.h>

void ui_init(void);
void ui_handle_keys(uint32_t events);
void ui_tick(uint32_t elapsed_ms);
void ui_set_load_sample(uint32_t active_ticks, uint32_t idle_ticks);
void ui_dmm_measurement_updated(void);
void ui_note_runtime_settings(void);
uint8_t ui_consume_beep_preview(void);
uint8_t ui_auto_sleep_due(void);
uint8_t ui_diode_beep_enabled(void);
uint8_t ui_live_beep_enabled(void);
void ui_set_live_wire_detected(uint8_t detected);
void ui_set_dmm_mode(uint8_t mode);
void ui_set_main_mode(uint8_t mode);
void ui_scope_set_running(uint8_t run);
void ui_scope_trigger_auto(void);
void ui_scope_step_timebase(int8_t dir);
void ui_scope_step_vdiv(uint8_t ch, int8_t dir);
void ui_request_screenshot(void);
void ui_inject_keys(uint32_t events);
void ui_gen_set_output(uint8_t run);
void ui_gen_set_frequency(uint32_t hz);
void ui_gen_set_wave(uint8_t wave);
void ui_gen_set_amplitude(uint8_t tenths_v);
void ui_gen_set_duty(uint8_t duty_percent);
uint8_t ui_gen_is_running(void);
uint32_t ui_gen_frequency(void);
uint8_t ui_gen_wave(void);
uint8_t ui_gen_amplitude(void);
