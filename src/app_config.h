#pragma once

#ifndef APP_BASE_ADDR
#define APP_BASE_ADDR 0x08008000u
#endif

#ifndef APP_FLASH_SIZE_BYTES
#define APP_FLASH_SIZE_BYTES 0x00038000u
#endif

#ifndef FW_STAGE_BASE_ADDR
#define FW_STAGE_BASE_ADDR 0x08040000u
#endif

#ifndef HW_TARGET_HW40
#define HW_TARGET_HW40 0
#endif

#if HW_TARGET_HW40
#define HW_TARGET_NAME "HW4.0"
#else
#define HW_TARGET_NAME "<HW4.0"
#endif
