#pragma once

#include <stdint.h>

enum {
    FW_UPDATE_STATE_IDLE,
    FW_UPDATE_STATE_STAGING,
    FW_UPDATE_STATE_READY,
    FW_UPDATE_STATE_ERROR,
    FW_UPDATE_STATE_APPLYING,
};

enum {
    FW_UPDATE_ERR_NONE,
    FW_UPDATE_ERR_RANGE,
    FW_UPDATE_ERR_VECTOR,
};

typedef struct {
    uint8_t state;
    uint8_t error;
    uint32_t sequence;
    uint32_t bytes;
    uint32_t expected_size;
    uint32_t base_lba;
} fw_update_status_t;

void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len);
void fw_update_note_file(uint32_t base_lba, uint32_t size);
uint8_t fw_update_request_apply(void);
void fw_update_clear(void);
void fw_update_service(void);
void fw_update_status(fw_update_status_t *status);
