#pragma once

#include <stdint.h>

#include "ui_fonts.h"

// Anti-aliased drawing on top of display.c. Like lcd_text(), every call takes
// the colour it is drawn over (bg) so edges can be blended without reading
// the framebuffer back; this keeps it working with the banded renderer too.

uint16_t gfx_blend(uint16_t fg, uint16_t bg, uint8_t alpha16);

uint16_t gfx_text_width(const lcd_font_t *font, const char *text);
// y is the top of the line box; returns the x just after the text.
int16_t gfx_text(int16_t x, int16_t y, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg);
void gfx_text_right(int16_t right, int16_t y, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg);
void gfx_text_center(int16_t x, int16_t y, uint16_t w, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg);

void gfx_round_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color, uint16_t bg);
// A line with round caps, `width` pixels wide, anti-aliased.
void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t width, uint16_t color, uint16_t bg);
