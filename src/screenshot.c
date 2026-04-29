#include "screenshot.h"

#include "display.h"

#include <stdint.h>

enum {
    SCREENSHOT_BMP_WIDTH = LCD_WIDTH,
    SCREENSHOT_BMP_HEIGHT = LCD_HEIGHT,
    SCREENSHOT_BMP_HEADER_SIZE = 66u,
    SCREENSHOT_BMP_ROW_BYTES = SCREENSHOT_BMP_WIDTH * 2u,
    SCREENSHOT_BMP_IMAGE_BYTES = SCREENSHOT_BMP_ROW_BYTES * SCREENSHOT_BMP_HEIGHT,
};

static uint8_t le32_byte(uint32_t value, uint8_t byte) {
    return (uint8_t)(value >> ((uint32_t)byte * 8u));
}

static uint8_t bmp_header_byte(uint16_t pos) {
    if (pos == 0u) {
        return 'B';
    }
    if (pos == 1u) {
        return 'M';
    }
    if (pos >= 2u && pos < 6u) {
        return le32_byte(SCREENSHOT_FILE_SIZE, (uint8_t)(pos - 2u));
    }
    if (pos >= 6u && pos < 10u) {
        return 0u;
    }
    if (pos >= 10u && pos < 14u) {
        return le32_byte(SCREENSHOT_BMP_HEADER_SIZE, (uint8_t)(pos - 10u));
    }

    pos = (uint16_t)(pos - 14u);
    if (pos < 4u) {
        return le32_byte(40u, (uint8_t)pos);
    }
    if (pos >= 4u && pos < 8u) {
        return le32_byte(SCREENSHOT_BMP_WIDTH, (uint8_t)(pos - 4u));
    }
    if (pos >= 8u && pos < 12u) {
        return le32_byte(SCREENSHOT_BMP_HEIGHT, (uint8_t)(pos - 8u));
    }
    if (pos == 12u) {
        return 1u;
    }
    if (pos == 14u) {
        return 16u;
    }
    if (pos >= 16u && pos < 20u) {
        return le32_byte(3u, (uint8_t)(pos - 16u));
    }
    if (pos >= 20u && pos < 24u) {
        return le32_byte(SCREENSHOT_BMP_IMAGE_BYTES, (uint8_t)(pos - 20u));
    }
    if (pos >= 24u && pos < 28u) {
        return le32_byte(2835u, (uint8_t)(pos - 24u));
    }
    if (pos >= 28u && pos < 32u) {
        return 0u;
    }
    pos = (uint16_t)(pos + 14u);
    if (pos >= 54u && pos < 58u) {
        return le32_byte(0x0000F800u, (uint8_t)(pos - 54u));
    }
    if (pos >= 58u && pos < 62u) {
        return le32_byte(0x000007E0u, (uint8_t)(pos - 58u));
    }
    if (pos >= 62u && pos < 66u) {
        return le32_byte(0x0000001Fu, (uint8_t)(pos - 62u));
    }
    return 0u;
}

static uint16_t bmp_rgb565_pixel(const uint16_t *fb, uint16_t row, uint16_t col) {
    uint16_t src_y = (uint16_t)(SCREENSHOT_BMP_HEIGHT - 1u - row);

    return fb[(uint32_t)src_y * LCD_WIDTH + col];
}

static uint8_t bmp_pixel_byte(const uint16_t *fb, uint16_t row, uint16_t byte_in_row) {
    uint16_t pixel = bmp_rgb565_pixel(fb, row, (uint16_t)(byte_in_row / 2u));

    return (byte_in_row & 1u) ? (uint8_t)(pixel >> 8) : (uint8_t)pixel;
}

static uint8_t bmp_byte(uint32_t offset, const uint16_t *fb) {
    if (offset < SCREENSHOT_BMP_HEADER_SIZE) {
        return bmp_header_byte((uint16_t)offset);
    }

    offset -= SCREENSHOT_BMP_HEADER_SIZE;
    return bmp_pixel_byte(fb,
                          (uint16_t)(offset / SCREENSHOT_BMP_ROW_BYTES),
                          (uint16_t)(offset % SCREENSHOT_BMP_ROW_BYTES));
}

uint8_t screenshot_render_current(uint32_t offset, uint8_t *dst, uint16_t len) {
    const uint16_t *fb = lcd_framebuffer();

    if (!dst || !fb) {
        return 0;
    }
    for (uint16_t i = 0; i < len; ++i) {
        uint32_t pos = offset + i;
        dst[i] = pos < SCREENSHOT_FILE_SIZE ? bmp_byte(pos, fb) : 0u;
    }
    return 1;
}
