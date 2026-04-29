#pragma once

#include <stdint.h>

void usb_msc_init(void);
void usb_msc_set_enabled(uint8_t enabled);
void usb_msc_poll(void);
uint8_t usb_msc_store_screenshot(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
