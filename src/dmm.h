#pragma once

#include <stdint.h>

// Fixed-point values are scaled in the reported unit, including its prefix.
enum { DMM_VALUE_SCALE = 10000 };

void dmm_init(void);
void dmm_pause(void);
void dmm_hw4_set_mode_gate(uint8_t active);
void dmm_reenter(uint8_t mode_index);
void dmm_set_mode(uint8_t mode_index);
uint8_t dmm_poll(void);
void dmm_tick(uint32_t elapsed_ms);
uint8_t dmm_has_reading(void);
uint8_t dmm_value_is_numeric(void);
int32_t dmm_value_milli_units(void);
int32_t dmm_value_fixed_units(void);
const char *dmm_value_text(void);
const char *dmm_unit_text(void);
const char *dmm_status_text(void);
const char *dmm_mode_name(void);
uint8_t dmm_reading_is_real(void);
uint8_t dmm_live_wire_active(void);
uint8_t dmm_diode_continuity_active(void);
void dmm_uart_irq_handler(void);
