#pragma once

#include <stdint.h>

typedef enum {
    BOOT_EXPORT_OK,
    BOOT_EXPORT_BUSY,
    BOOT_EXPORT_PROTECTED,
    BOOT_EXPORT_INVALID,
    BOOT_EXPORT_STORAGE_ERROR,
    BOOT_EXPORT_NO_SPACE,
    BOOT_EXPORT_VERIFY_ERROR,
} boot_export_result_t;

typedef void (*boot_export_progress_fn_t)(uint8_t percent);

void usb_msc_init(void);
void usb_msc_set_enabled(uint8_t enabled);
void usb_msc_poll(void);
uint8_t usb_msc_storage_begin(void);
void usb_msc_storage_end(void);
// Why a capture could not be saved. The screen used to show a bare error
// for every one of these, which left "the drive is full" -- the one the
// user can actually act on -- indistinguishable from a write failure.
typedef enum {
    SCREENSHOT_SAVE_OK,
    SCREENSHOT_SAVE_BUSY,        // the host is in the middle of a transfer
    SCREENSHOT_SAVE_NO_VOLUME,   // the FAT would not mount
    SCREENSHOT_SAVE_NO_SLOT,     // the root directory has no free entry
    SCREENSHOT_SAVE_FULL,        // no free cluster left on the drive
    SCREENSHOT_SAVE_ERROR,       // the write itself failed
} screenshot_save_t;

uint8_t usb_msc_store_screenshot(void);
screenshot_save_t usb_msc_save_screenshot(void);
uint8_t usb_msc_storage_free_kb(uint32_t *free_kb, uint32_t *total_kb);
boot_export_result_t usb_msc_export_bootloader(const char *version,
                                               boot_export_progress_fn_t progress);
void USB_LP_CAN1_RX0_IRQHandler(void);

// USB CDC Virtual COM Port additions
void usb_cdc_send_bytes(const uint8_t *data, uint16_t len);
void usb_cdc_send_dmm(const char *mode, const char *val, const char *unit, uint8_t valid);
uint8_t usb_cdc_is_configured(void);
