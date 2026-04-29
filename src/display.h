#pragma once

#include <stdint.h>

#define LCD_WIDTH  320u
#define LCD_HEIGHT 240u
#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8u) << 8) | (((g) & 0xFCu) << 3) | (((b) & 0xF8u) >> 3))

typedef void (*lcd_render_fn_t)(void *ctx);

void lcd_init(void);
void lcd_display_on(void);
void lcd_render(lcd_render_fn_t render, void *ctx);
void lcd_fill(uint16_t color);
void lcd_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void lcd_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void lcd_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void lcd_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale);
void lcd_text_transparent(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint8_t scale);
void lcd_text_center(uint16_t x, uint16_t y, uint16_t w, const char *text, uint16_t fg, uint16_t bg, uint8_t scale);
uint16_t lcd_text_width(const char *text, uint8_t scale);
const uint16_t *lcd_framebuffer(void);
