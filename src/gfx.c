#include "gfx.h"

#include "display.h"

uint16_t gfx_blend(uint16_t fg, uint16_t bg, uint8_t alpha16) {
    if (alpha16 >= 16u) {
        return fg;
    }
    if (!alpha16) {
        return bg;
    }
    uint32_t inv = 16u - alpha16;
    uint32_t r = (((fg >> 11) & 0x1Fu) * alpha16 + ((bg >> 11) & 0x1Fu) * inv + 8u) >> 4;
    uint32_t g = (((fg >> 5) & 0x3Fu) * alpha16 + ((bg >> 5) & 0x3Fu) * inv + 8u) >> 4;
    uint32_t b = ((fg & 0x1Fu) * alpha16 + (bg & 0x1Fu) * inv + 8u) >> 4;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

// Minimal UTF-8: ASCII plus the two-byte sequences the fonts carry
// (Ohm, micro, degree, Delta) and the three-byte em dash.
static uint16_t utf8_next(const char **text) {
    const uint8_t *s = (const uint8_t *)*text;
    uint16_t cp = s[0];
    if (cp < 0x80u) {
        *text += 1;
    } else if ((cp & 0xE0u) == 0xC0u && s[1]) {
        cp = (uint16_t)(((cp & 0x1Fu) << 6) | (s[1] & 0x3Fu));
        *text += 2;
    } else if ((cp & 0xF0u) == 0xE0u && s[1] && s[2]) {
        cp = (uint16_t)(((cp & 0x0Fu) << 12) | ((s[1] & 0x3Fu) << 6) | (s[2] & 0x3Fu));
        *text += 3;
    } else {
        cp = '?';
        *text += 1;
    }
    return cp;
}

static const lcd_glyph_t *find_glyph(const lcd_font_t *font, uint16_t cp) {
    // Glyphs are sorted by code point.
    uint16_t lo = 0;
    uint16_t hi = font->count;
    while (lo < hi) {
        uint16_t mid = (uint16_t)((lo + hi) / 2u);
        uint16_t c = font->glyphs[mid].codepoint;
        if (c == cp) {
            return &font->glyphs[mid];
        }
        if (c < cp) {
            lo = (uint16_t)(mid + 1u);
        } else {
            hi = mid;
        }
    }
    if (cp >= 'a' && cp <= 'z') {
        return find_glyph(font, (uint16_t)(cp - 32u));
    }
    return cp == '?' ? 0 : find_glyph(font, '?');
}

uint16_t gfx_text_width(const lcd_font_t *font, const char *text) {
    uint16_t w = 0;
    while (*text) {
        const lcd_glyph_t *g = find_glyph(font, utf8_next(&text));
        if (g) {
            w = (uint16_t)(w + g->advance);
        }
    }
    return w;
}

static void draw_glyph(int16_t x, int16_t y, const lcd_font_t *font, const lcd_glyph_t *g, uint16_t fg, uint16_t bg) {
    const uint8_t *p = font->data + g->offset;
    uint32_t total = (uint32_t)g->w * g->h;
    uint32_t i = 0;
    int16_t gx = (int16_t)(x + g->x_off);
    int16_t gy = (int16_t)(y + g->y_off);

    while (i < total) {
        uint8_t code = *p++;
        uint8_t alpha;
        uint8_t run;
        if (code & 0x80u) {
            alpha = (uint8_t)((code >> 2) & 0x0Fu);
            run = (uint8_t)((code & 0x03u) + 1u);
        } else {
            alpha = (code & 0x40u) ? 15u : 0u;
            run = (uint8_t)((code & 0x3Fu) + 1u);
        }
        if (!alpha) {
            i += run;
            continue;
        }
        uint16_t color = gfx_blend(fg, bg, alpha == 15u ? 16u : (uint8_t)(alpha + (alpha >> 3)));
        while (run--) {
            uint16_t col = (uint16_t)(i % g->w);
            uint16_t row = (uint16_t)(i / g->w);
            if (alpha == 15u && run) {
                // Opaque spans within one row go out as a single rect.
                uint16_t n = 1;
                while (run && col + n < g->w) {
                    ++n;
                    --run;
                }
                lcd_rect((uint16_t)(gx + col), (uint16_t)(gy + row), n, 1, color);
                i += n;
                continue;
            }
            lcd_pixel((int16_t)(gx + col), (int16_t)(gy + row), color);
            ++i;
        }
    }
}

int16_t gfx_text(int16_t x, int16_t y, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg) {
    if (lcd_collecting()) {
        uint16_t w = gfx_text_width(font, text);
        int16_t x0 = x > 2 ? (int16_t)(x - 2) : 0;
        lcd_mark_dirty((uint16_t)x0, (uint16_t)(y < 0 ? 0 : y), (uint16_t)(w + 4u + (x - x0)), font->line_height);
        return (int16_t)(x + w);
    }
    while (*text) {
        const lcd_glyph_t *g = find_glyph(font, utf8_next(&text));
        if (!g) {
            continue;
        }
        if (g->w) {
            draw_glyph(x, y, font, g, fg, bg);
        }
        x = (int16_t)(x + g->advance);
    }
    return x;
}

void gfx_text_right(int16_t right, int16_t y, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg) {
    gfx_text((int16_t)(right - (int16_t)gfx_text_width(font, text)), y, font, text, fg, bg);
}

void gfx_text_center(int16_t x, int16_t y, uint16_t w, const lcd_font_t *font, const char *text, uint16_t fg, uint16_t bg) {
    uint16_t tw = gfx_text_width(font, text);
    gfx_text((int16_t)(x + (tw < w ? (int16_t)((w - tw) / 2u) : 0)), y, font, text, fg, bg);
}

// Coverage of one corner pixel, in sixteenths, by 4x4 supersampling.
// Units are eighths of a pixel: sample k of pixel i sits at (4i + k) * 2 + 1,
// the quarter circle's centre and radius are both 8r.
static uint8_t corner_coverage(uint16_t r, uint16_t i, uint16_t j) {
    int32_t c = (int32_t)r * 8;
    int32_t rr = c * c;
    uint8_t hits = 0;
    for (uint8_t sy = 0; sy < 4u; ++sy) {
        int32_t dy = ((int32_t)j * 4 + sy) * 2 + 1 - c;
        for (uint8_t sx = 0; sx < 4u; ++sx) {
            int32_t dx = ((int32_t)i * 4 + sx) * 2 + 1 - c;
            if (dx * dx + dy * dy <= rr) {
                ++hits;
            }
        }
    }
    return hits;
}

void gfx_round_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color, uint16_t bg) {
    if (!w || !h) {
        return;
    }
    if (r * 2u > w) {
        r = (uint16_t)(w / 2u);
    }
    if (r * 2u > h) {
        r = (uint16_t)(h / 2u);
    }
    if (lcd_collecting()) {
        lcd_mark_dirty(x, y, w, h);
        return;
    }
    if (!r) {
        lcd_rect(x, y, w, h, color);
        return;
    }
    lcd_rect((uint16_t)(x + r), y, (uint16_t)(w - 2u * r), h, color);
    lcd_rect(x, (uint16_t)(y + r), r, (uint16_t)(h - 2u * r), color);
    lcd_rect((uint16_t)(x + w - r), (uint16_t)(y + r), r, (uint16_t)(h - 2u * r), color);
    for (uint16_t j = 0; j < r; ++j) {
        for (uint16_t i = 0; i < r; ++i) {
            uint8_t cov = corner_coverage(r, i, j);
            if (!cov) {
                continue;
            }
            uint16_t c = gfx_blend(color, bg, cov);
            int16_t left = (int16_t)(x + i);
            int16_t right = (int16_t)(x + w - 1u - i);
            int16_t top = (int16_t)(y + j);
            int16_t bottom = (int16_t)(y + h - 1u - j);
            lcd_pixel(left, top, c);
            lcd_pixel(right, top, c);
            lcd_pixel(left, bottom, c);
            lcd_pixel(right, bottom, c);
        }
    }
}

void gfx_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t width, uint16_t color, uint16_t bg) {
    int16_t pad = (int16_t)(width / 2u + 1u);
    int16_t bx0 = (int16_t)((x0 < x1 ? x0 : x1) - pad);
    int16_t by0 = (int16_t)((y0 < y1 ? y0 : y1) - pad);
    int16_t bx1 = (int16_t)((x0 > x1 ? x0 : x1) + pad);
    int16_t by1 = (int16_t)((y0 > y1 ? y0 : y1) + pad);

    if (bx0 < 0) bx0 = 0;
    if (by0 < 0) by0 = 0;
    if (lcd_collecting()) {
        lcd_mark_dirty((uint16_t)bx0, (uint16_t)by0, (uint16_t)(bx1 - bx0 + 1), (uint16_t)(by1 - by0 + 1));
        return;
    }

    // Everything in eighths of a pixel; endpoints sit on pixel centres.
    int32_t ax = x0 * 8 + 4;
    int32_t ay = y0 * 8 + 4;
    int32_t vx = (x1 - x0) * 8;
    int32_t vy = (y1 - y0) * 8;
    int64_t len2 = (int64_t)vx * vx + (int64_t)vy * vy;
    int64_t hw = (int64_t)width * 4; // half width in eighths
    int64_t hw2 = hw * hw;

    for (int16_t py = by0; py <= by1; ++py) {
        for (int16_t px = bx0; px <= bx1; ++px) {
            uint8_t hits = 0;
            for (uint8_t sy = 0; sy < 4u; ++sy) {
                for (uint8_t sx = 0; sx < 4u; ++sx) {
                    int32_t qx = px * 8 + sx * 2 + 1 - ax;
                    int32_t qy = py * 8 + sy * 2 + 1 - ay;
                    int64_t dot = (int64_t)qx * vx + (int64_t)qy * vy;
                    uint8_t inside;
                    if (len2 == 0 || dot <= 0) {
                        inside = (int64_t)qx * qx + (int64_t)qy * qy <= hw2;
                    } else if (dot >= len2) {
                        int64_t ex = qx - vx;
                        int64_t ey = qy - vy;
                        inside = ex * ex + ey * ey <= hw2;
                    } else {
                        // distance^2 = cross^2 / len2; compare without dividing
                        int64_t cross = (int64_t)qx * vy - (int64_t)qy * vx;
                        inside = cross * cross <= hw2 * len2;
                    }
                    hits = (uint8_t)(hits + inside);
                }
            }
            if (hits) {
                lcd_pixel(px, py, gfx_blend(color, bg, hits));
            }
        }
    }
}
