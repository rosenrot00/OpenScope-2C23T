#pragma once

#include <stdint.h>

// One anti-aliased glyph: 4-bit alpha, run-length encoded (see gfx.c).
typedef struct {
    uint16_t codepoint;
    uint8_t w;
    uint8_t h;
    int8_t x_off;   // from the pen position to the bitmap's left edge
    int8_t y_off;   // from the top of the line to the bitmap's top edge
    uint8_t advance;
    uint16_t offset; // into the font's data
} lcd_glyph_t;

typedef struct {
    const lcd_glyph_t *glyphs;
    const uint8_t *data;
    uint16_t count;
    uint8_t line_height;
    uint8_t ascent;
} lcd_font_t;

extern const lcd_font_t font_huge;   // 56 px, digits and sign only
extern const lcd_font_t font_large;  // 26 px, letters, digits, units
extern const lcd_font_t font_medium; // 13 px, full ASCII + unit symbols
extern const lcd_font_t font_small;  // 10 px, full ASCII + unit symbols
extern const lcd_font_t font_icons;  // Lucide icons, see ui_icons.h
