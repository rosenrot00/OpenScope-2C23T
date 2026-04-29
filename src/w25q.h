#pragma once

#include <stdint.h>

void w25q_init(void);
uint32_t w25q_capacity_bytes(void);
uint16_t w25q_device_id(void);
uint8_t w25q_read(uint32_t addr, uint8_t *dst, uint16_t len);
uint8_t w25q_write_sector(uint32_t sector_addr, const uint8_t *src);
