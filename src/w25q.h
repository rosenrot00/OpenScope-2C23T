#pragma once

#include <stdint.h>

typedef enum w25q_write_status_t {
    W25Q_WRITE_IDLE,
    W25Q_WRITE_BUSY,
    W25Q_WRITE_DONE,
    W25Q_WRITE_ERROR,
} w25q_write_status_t;

void w25q_init(void);
uint32_t w25q_capacity_bytes(void);
uint16_t w25q_device_id(void);
// Reads fail while a write is active or a single status check reports WIP/SPI error.
// After a write ERROR, reads recover once the chip is ready; ERROR stays latched.
uint8_t w25q_read(uint32_t addr, uint8_t *dst, uint16_t len);
// Start/poll have one foreground owner. Each poll checks WIP at most once and
// starts at most one erase/page command; it never waits for WIP to clear.
// On success, keep src[4096] valid and unchanged until poll returns DONE/ERROR.
// A rejected concurrent start leaves the active job and its source untouched.
uint8_t w25q_write_sector_start(uint32_t sector_addr, const uint8_t *src);
w25q_write_status_t w25q_write_sector_poll(void);
uint8_t w25q_write_sector_busy(void);
// Blocking wrapper for explicit local operations; never takes over an active job.
uint8_t w25q_write_sector(uint32_t sector_addr, const uint8_t *src);
