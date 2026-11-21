#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../src/gfx.c"

static uint16_t pixels[LCD_WIDTH * LCD_HEIGHT];
static uint16_t expected[LCD_WIDTH * LCD_HEIGHT];
static uint16_t *output = pixels;
static int16_t reference_x0;
static int16_t reference_x1 = LCD_WIDTH - 1;
static uint8_t collecting;
static unsigned marks;

uint8_t lcd_collecting(void) { return collecting; }
void lcd_mark_dirty(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    (void)x; (void)y;
    assert(collecting && w && h);
    ++marks;
}
void lcd_pixel(int16_t x, int16_t y, uint16_t color) {
    assert(!collecting);
    if (x >= 0 && y >= 0 && x < (int16_t)LCD_WIDTH && y < (int16_t)LCD_HEIGHT) {
        output[(uint32_t)y * LCD_WIDTH + x] = color;
    }
}
void lcd_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    assert(!collecting);
    for (uint32_t row = y; row < (uint32_t)y + h && row < LCD_HEIGHT; ++row) {
        for (uint32_t col = x; col < (uint32_t)x + w && col < LCD_WIDTH; ++col) {
            output[row * LCD_WIDTH + col] = color;
        }
    }
}

static void clear(void) {
    for (unsigned i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) pixels[i] = expected[i] = 0x0841;
    output = pixels;
}

static void reference_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x >= reference_x0 && x <= reference_x1) {
        output = expected;
        lcd_pixel(x, y, color);
        output = pixels;
    }
}

static uint8_t reference_coverage(uint16_t r, uint16_t i, uint16_t j) {
    int32_t c = (int32_t)r * 8;
    uint8_t hits = 0;
    for (uint8_t sy = 0; sy < 4; ++sy) {
        for (uint8_t sx = 0; sx < 4; ++sx) {
            int32_t dx = (int32_t)i * 8 + sx * 2 + 1 - c;
            int32_t dy = (int32_t)j * 8 + sy * 2 + 1 - c;
            hits += dx * dx + dy * dy <= c * c;
        }
    }
    return hits;
}

static void reference_round_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint16_t r, uint16_t color, uint16_t bg) {
    if (!w || !h) return;
    if (2u * r > w) r = w / 2u;
    if (2u * r > h) r = h / 2u;
    for (uint16_t row = 0; row < h; ++row) {
        for (uint16_t col = 0; col < w; ++col) {
            uint8_t cov = 16;
            uint16_t cx = col < r ? col : (uint16_t)(w - col - 1u);
            uint16_t cy = row < r ? row : (uint16_t)(h - row - 1u);
            if (cx < r && cy < r) cov = reference_coverage(r, cx, cy);
            if (cov) reference_pixel((int16_t)(x + col), (int16_t)(y + row), gfx_blend(color, bg, cov));
        }
    }
}

static void test_round_rects(void) {
    for (uint16_t r = 0; r <= 120u; ++r) {
        for (uint8_t clipped = 0; clipped < 2; ++clipped) {
            clear();
            reference_x0 = clipped ? 19 : 0;
            reference_x1 = clipped ? 151 : LCD_WIDTH - 1;
            gfx_clip(reference_x0, reference_x1);
            gfx_round_rect(5, 3, 260, 234, r, 0xAF35, 0x0841);
            reference_round_rect(5, 3, 260, 234, r, 0xAF35, 0x0841);
            assert(!memcmp(pixels, expected, sizeof(pixels)));
        }
    }
    for (uint16_t w = 0; w <= 20; ++w) {
        clear();
        gfx_clip_reset();
        reference_x0 = 0;
        reference_x1 = LCD_WIDTH - 1;
        gfx_round_rect(310, 234, w, 9, 40, 0xAF35, 0x0841);
        reference_round_rect(310, 234, w, 9, 40, 0xAF35, 0x0841);
        assert(!memcmp(pixels, expected, sizeof(pixels)));
    }
}

static void reference_glyph(int16_t x, int16_t y, const lcd_font_t *font,
                           const lcd_glyph_t *g, uint16_t fg, uint16_t bg) {
    const uint8_t *p = font->data + g->offset;
    uint32_t i = 0;
    uint32_t total = (uint32_t)g->w * g->h;
    while (i < total) {
        uint8_t code = *p++;
        uint8_t alpha = code & 0x80u ? (code >> 2) & 15u : (code & 0x40u ? 15u : 0u);
        uint8_t run = (code & 0x80u ? code & 3u : code & 63u) + 1u;
        assert(i + run <= total);
        uint16_t color = gfx_blend(fg, bg, alpha == 15u ? 16u : (uint8_t)(alpha + (alpha >> 3)));
        while (run--) {
            if (alpha) {
                reference_pixel((int16_t)(x + g->x_off + i % g->w),
                                (int16_t)(y + g->y_off + i / g->w), color);
            }
            ++i;
        }
    }
}

static void test_glyphs(void) {
    static const lcd_font_t *const fonts[] = {&font_huge, &font_large, &font_medium, &font_small, &font_icons};
    static const int16_t positions[][2] = {{8, 8}, {-4, -6}, {315, 230}};
    for (unsigned f = 0; f < sizeof(fonts) / sizeof(fonts[0]); ++f) {
        for (uint16_t g = 0; g < fonts[f]->count; ++g) {
            if (!fonts[f]->glyphs[g].w) continue;
            for (unsigned pos = 0; pos < sizeof(positions) / sizeof(positions[0]); ++pos) {
                clear();
                reference_x0 = pos == 0 ? 10 : 0;
                reference_x1 = pos == 0 ? 22 : LCD_WIDTH - 1;
                gfx_clip(reference_x0, reference_x1);
                draw_glyph(positions[pos][0], positions[pos][1], fonts[f], &fonts[f]->glyphs[g], 0xFFFF, 0x0841);
                reference_glyph(positions[pos][0], positions[pos][1], fonts[f], &fonts[f]->glyphs[g], 0xFFFF, 0x0841);
                assert(!memcmp(pixels, expected, sizeof(pixels)));
            }
        }
    }
    gfx_clip_reset();
}

static void test_collect(void) {
    clear();
    collecting = 1;
    marks = 0;
    gfx_text(3, 4, &font_medium, "123.4 V", 0xFFFF, 0x0841);
    gfx_round_rect(2, 3, 70, 20, 8, 0xFFFF, 0x0841);
    collecting = 0;
    assert(marks == 2);
    assert(!memcmp(pixels, expected, sizeof(pixels)));
}

int main(void) {
    test_round_rects();
    test_glyphs();
    test_collect();
    puts("GFX: rounded rectangles, every font glyph, clipping and collect pass match reference pixels");
    return 0;
}
