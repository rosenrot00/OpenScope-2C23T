#include "display.h"

#include "board.h"
#include "font.h"
#include "hw.h"

#include <stdint.h>

#define LCD_BAND_HEIGHT 16u
#define LCD_MAX_DIRTY_RECTS 24u
#define LCD_USE_FRAMEBUFFER 1u
#define LCD_FSMC_READ_TIMING  0x02020424u
#define LCD_FSMC_WRITE_TIMING 0x00000101u
#define DMA_CH1_CLEAR_FLAGS 0x0000000Fu
#define DMA_CH1_TC_FLAG     0x00000002u
#define DMA_CH1_TE_FLAG     0x00000008u
#define DMA_CCR_MEM2MEM     (1u << 14)
#define DMA_CCR_PL_HIGH     (3u << 12)
#define DMA_CCR_MSIZE_16    (1u << 10)
#define DMA_CCR_PSIZE_16    (1u << 8)
#define DMA_CCR_MINC        (1u << 7)
#define DMA_CCR_DIR         (1u << 4)
#define DMA_CCR_EN          1u

typedef enum {
    LCD_PASS_IDLE,
    LCD_PASS_COLLECT,
    LCD_PASS_RENDER,
} lcd_pass_t;

typedef struct {
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
} dirty_rect_t;

#if LCD_USE_FRAMEBUFFER
static uint16_t frame_buffer[LCD_WIDTH * LCD_HEIGHT];
static uint16_t *band_buffer = frame_buffer;
#else
static uint16_t band_buffers[2][LCD_WIDTH * LCD_BAND_HEIGHT];
static uint16_t *band_buffer = band_buffers[0];
#endif
static dirty_rect_t dirty_rects[LCD_MAX_DIRTY_RECTS];
static uint8_t dirty_count;
static lcd_pass_t lcd_pass;
static uint8_t lcd_dma_active;
static uint16_t band_x0;
static uint16_t band_y0;
static uint16_t band_x1;
static uint16_t band_y1;

static void dirty_reset(void) {
    dirty_count = 0;
}

static uint8_t rect_contains(const dirty_rect_t *outer, const dirty_rect_t *inner) {
    return outer->x0 <= inner->x0 && outer->y0 <= inner->y0 &&
           outer->x1 >= inner->x1 && outer->y1 >= inner->y1;
}

static void dirty_append(const dirty_rect_t *rect) {
    for (uint8_t i = 0; i < dirty_count; ++i) {
        if (rect_contains(&dirty_rects[i], rect)) {
            return;
        }
    }

    uint8_t i = 0;
    while (i < dirty_count) {
        if (rect_contains(rect, &dirty_rects[i])) {
            dirty_rects[i] = dirty_rects[(uint8_t)(dirty_count - 1u)];
            dirty_count--;
            continue;
        }
        i++;
    }

    if (dirty_count < LCD_MAX_DIRTY_RECTS) {
        dirty_rects[dirty_count++] = *rect;
        return;
    }

    dirty_rect_t *base = &dirty_rects[0];
    if (rect->x0 < base->x0) {
        base->x0 = rect->x0;
    }
    if (rect->y0 < base->y0) {
        base->y0 = rect->y0;
    }
    if (rect->x1 > base->x1) {
        base->x1 = rect->x1;
    }
    if (rect->y1 > base->y1) {
        base->y1 = rect->y1;
    }
}

static void dirty_mark(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!w || !h || x >= LCD_WIDTH || y >= LCD_HEIGHT) {
        return;
    }

    if ((uint32_t)x + w > LCD_WIDTH) {
        w = (uint16_t)(LCD_WIDTH - x);
    }
    if ((uint32_t)y + h > LCD_HEIGHT) {
        h = (uint16_t)(LCD_HEIGHT - y);
    }

    dirty_rect_t rect = {
        .x0 = x,
        .y0 = y,
        .x1 = (uint16_t)(x + w - 1u),
        .y1 = (uint16_t)(y + h - 1u),
    };
    dirty_append(&rect);
}

static void fsmc_lcd_gpio_init(void) {
    RCC_AHBENR |= 1u << 0; // DMA1

    gpio_config_mask(GPIOD_BASE, 0x08B0u, 0x9);
    gpio_config_mask(GPIOD_BASE, 0xC703u, 0x9);
    gpio_config_mask(GPIOE_BASE, 0xFF80u, 0x9);

    FSMC_BCR1 = 0x00005010u;
    FSMC_BTR1 = LCD_FSMC_READ_TIMING;
    FSMC_BWTR1 = LCD_FSMC_WRITE_TIMING;
    FSMC_WPCR1 = (FSMC_WPCR1 & 0xFFFF0000u) | (32u << 8) | 8u;
    FSMC_BCR1 |= 1u;
}

static void lcd_cmd(uint16_t cmd) {
    LCD_CMD = cmd;
}

static void lcd_data(uint16_t data) {
    LCD_DATA = data;
}

void lcd_init(void) {
    fsmc_lcd_gpio_init();

    lcd_cmd(0x28);
    lcd_cmd(0x11);
    delay_ms(60);
    lcd_cmd(0x36); lcd_data(0xA0);
    lcd_cmd(0x3A); lcd_data(0x05);
    lcd_cmd(0xB2); lcd_data(0x0C); lcd_data(0x0C); lcd_data(0x00); lcd_data(0x33); lcd_data(0x33);
    lcd_cmd(0xB7); lcd_data(0x35);
    lcd_cmd(0xBB); lcd_data(0x28);
    lcd_cmd(0xC0); lcd_data(0x2C);
    lcd_cmd(0xC2); lcd_data(0x01);
    lcd_cmd(0xC3); lcd_data(0x0B);
    lcd_cmd(0xC4); lcd_data(0x20);
    lcd_cmd(0xC6); lcd_data(0x0F);
    lcd_cmd(0xD0); lcd_data(0xA4); lcd_data(0xA1);
    lcd_cmd(0xE0);
    lcd_data(0xD0); lcd_data(0x01); lcd_data(0x08); lcd_data(0x0F);
    lcd_data(0x11); lcd_data(0x2A); lcd_data(0x36); lcd_data(0x55);
    lcd_data(0x44); lcd_data(0x3A); lcd_data(0x0B); lcd_data(0x06);
    lcd_data(0x11); lcd_data(0x20);
    lcd_cmd(0xE1);
    lcd_data(0xD0); lcd_data(0x02); lcd_data(0x07); lcd_data(0x0A);
    lcd_data(0x0B); lcd_data(0x18); lcd_data(0x34); lcd_data(0x43);
    lcd_data(0x4A); lcd_data(0x2B); lcd_data(0x1B); lcd_data(0x1C);
    lcd_data(0x22); lcd_data(0x1F);
    dirty_reset();
    lcd_pass = LCD_PASS_IDLE;
    lcd_dma_active = 0;
}

void lcd_display_on(void) {
    lcd_cmd(0x29);
    delay_ms(20);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_cmd(0x2A);
    lcd_data(x0 >> 8); lcd_data(x0);
    lcd_data(x1 >> 8); lcd_data(x1);
    lcd_cmd(0x2B);
    lcd_data(y0 >> 8); lcd_data(y0);
    lcd_data(y1 >> 8); lcd_data(y1);
    lcd_cmd(0x2C);
}

static void lcd_rect_direct(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (!w || !h || x >= LCD_WIDTH || y >= LCD_HEIGHT) {
        return;
    }
    if ((uint32_t)x + w > LCD_WIDTH) {
        w = (uint16_t)(LCD_WIDTH - x);
    }
    if ((uint32_t)y + h > LCD_HEIGHT) {
        h = (uint16_t)(LCD_HEIGHT - y);
    }

    lcd_set_window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));
    for (uint32_t i = 0; i < (uint32_t)w * h; ++i) {
        lcd_data(color);
    }
}

static void lcd_rect_band(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (!w || !h || x > band_x1 || y > band_y1) {
        return;
    }

    uint16_t x1 = (uint16_t)(x + w - 1u);
    uint16_t y1 = (uint16_t)(y + h - 1u);

    if (x < band_x0) {
        x = band_x0;
    }
    if (y < band_y0) {
        y = band_y0;
    }
    if (x1 > band_x1) {
        x1 = band_x1;
    }
    if (y1 > band_y1) {
        y1 = band_y1;
    }
    if (x > x1 || y > y1) {
        return;
    }

    for (uint16_t row = y; row <= y1; ++row) {
        uint32_t offset = (uint32_t)(row - band_y0) * LCD_WIDTH + x;
        for (uint16_t col = x; col <= x1; ++col) {
            band_buffer[offset++] = color;
        }
    }
}

static void lcd_plot_band(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= (int16_t)LCD_WIDTH || y >= (int16_t)LCD_HEIGHT) {
        return;
    }
    if ((uint16_t)x < band_x0 || (uint16_t)x > band_x1 ||
        (uint16_t)y < band_y0 || (uint16_t)y > band_y1) {
        return;
    }
    band_buffer[(uint32_t)((uint16_t)y - band_y0) * LCD_WIDTH + (uint16_t)x] = color;
}

static void lcd_plot_direct(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= (int16_t)LCD_WIDTH || y >= (int16_t)LCD_HEIGHT) {
        return;
    }
    lcd_set_window((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
    lcd_data(color);
}

void lcd_fill(uint16_t color) {
    if (lcd_pass == LCD_PASS_COLLECT) {
        dirty_mark(0, 0, LCD_WIDTH, LCD_HEIGHT);
        return;
    }
    if (lcd_pass == LCD_PASS_RENDER) {
        lcd_rect_band(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
        return;
    }
    lcd_rect_direct(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

void lcd_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (lcd_pass == LCD_PASS_COLLECT) {
        dirty_mark(x, y, w, h);
        return;
    }
    if (lcd_pass == LCD_PASS_RENDER) {
        lcd_rect_band(x, y, w, h, color);
        return;
    }
    lcd_rect_direct(x, y, w, h, color);
}

void lcd_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (w < 2u || h < 2u) {
        return;
    }
    lcd_rect(x, y, w, 1, color);
    lcd_rect(x, (uint16_t)(y + h - 1u), w, 1, color);
    lcd_rect(x, y, 1, h, color);
    lcd_rect((uint16_t)(x + w - 1u), y, 1, h, color);
}

static int16_t iabs16(int16_t v) {
    return v < 0 ? (int16_t)-v : v;
}

void lcd_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t min_x = x0 < x1 ? x0 : x1;
    int16_t max_x = x0 > x1 ? x0 : x1;
    int16_t min_y = y0 < y1 ? y0 : y1;
    int16_t max_y = y0 > y1 ? y0 : y1;

    if (lcd_pass == LCD_PASS_COLLECT) {
        if (max_x < 0 || max_y < 0 || min_x >= (int16_t)LCD_WIDTH || min_y >= (int16_t)LCD_HEIGHT) {
            return;
        }
        if (min_x < 0) {
            min_x = 0;
        }
        if (min_y < 0) {
            min_y = 0;
        }
        if (max_x >= (int16_t)LCD_WIDTH) {
            max_x = (int16_t)(LCD_WIDTH - 1u);
        }
        if (max_y >= (int16_t)LCD_HEIGHT) {
            max_y = (int16_t)(LCD_HEIGHT - 1u);
        }
        dirty_mark((uint16_t)min_x, (uint16_t)min_y,
                   (uint16_t)(max_x - min_x + 1), (uint16_t)(max_y - min_y + 1));
        return;
    }
    if (lcd_pass == LCD_PASS_RENDER &&
        (max_x < (int16_t)band_x0 || min_x > (int16_t)band_x1 ||
         max_y < (int16_t)band_y0 || min_y > (int16_t)band_y1)) {
        return;
    }

    int16_t dx = iabs16((int16_t)(x1 - x0));
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t dy = (int16_t)-iabs16((int16_t)(y1 - y0));
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);

    while (1) {
        if (lcd_pass == LCD_PASS_RENDER) {
            lcd_plot_band(x0, y0, color);
        } else {
            lcd_plot_direct(x0, y0, color);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int16_t e2 = (int16_t)(2 * err);
        if (e2 >= dy) {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + sx);
        }
        if (e2 <= dx) {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + sy);
        }
    }
}

#if !LCD_USE_FRAMEBUFFER
static void band_clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    for (uint16_t y = y0; y <= y1; ++y) {
        uint32_t offset = (uint32_t)(y - band_y0) * LCD_WIDTH + x0;
        for (uint16_t x = x0; x <= x1; ++x) {
            band_buffer[offset++] = 0;
        }
    }
}
#endif

static uint8_t lcd_dma_wait(void) {
    if (!lcd_dma_active) {
        return 1;
    }
    for (uint32_t timeout = 0; timeout < 200000u; ++timeout) {
        uint32_t isr = DMA_ISR;
        if (isr & DMA_CH1_TC_FLAG) {
            DMA_CCR1 = 0;
            DMA_IFCR = DMA_CH1_CLEAR_FLAGS;
            lcd_dma_active = 0;
            return 1;
        }
        if (isr & DMA_CH1_TE_FLAG) {
            break;
        }
    }

    DMA_CCR1 = 0;
    DMA_IFCR = DMA_CH1_CLEAR_FLAGS;
    lcd_dma_active = 0;
    return 0;
}

static uint8_t lcd_dma_start_words(const uint16_t *src, uint16_t words) {
    if (!words) {
        return 0;
    }

    DMA_CCR1 = 0;
    DMA_IFCR = DMA_CH1_CLEAR_FLAGS;
    DMA_CPAR1 = LCD_DATA_ADDR;
    DMA_CMAR1 = (uint32_t)(uintptr_t)src;
    DMA_CNDTR1 = words;
    DMA_CCR1 = DMA_CCR_MEM2MEM | DMA_CCR_PL_HIGH | DMA_CCR_MSIZE_16 |
               DMA_CCR_PSIZE_16 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
    lcd_dma_active = 1;
    return 1;
}

static uint8_t lcd_dma_write_words_sync(const uint16_t *src, uint16_t words) {
    return lcd_dma_start_words(src, words) && lcd_dma_wait();
}

#if LCD_USE_FRAMEBUFFER
static void frame_flush_words(const uint16_t *src, uint32_t words) {
    while (words) {
        uint16_t chunk = words > 65535u ? 65535u : (uint16_t)words;
        if (!lcd_dma_write_words_sync(src, chunk)) {
            return;
        }
        src += chunk;
        words -= chunk;
    }
}

static void frame_flush_rect(const dirty_rect_t *rect) {
    uint16_t width = (uint16_t)(rect->x1 - rect->x0 + 1u);
    uint16_t height = (uint16_t)(rect->y1 - rect->y0 + 1u);

    if (width == LCD_WIDTH) {
        lcd_set_window(rect->x0, rect->y0, rect->x1, rect->y1);
        frame_flush_words(&frame_buffer[(uint32_t)rect->y0 * LCD_WIDTH], (uint32_t)width * height);
        return;
    }

    lcd_set_window(rect->x0, rect->y0, rect->x1, rect->y1);
    for (uint16_t y = rect->y0; y <= rect->y1; ++y) {
        frame_flush_words(&frame_buffer[(uint32_t)y * LCD_WIDTH + rect->x0], width);
    }
}
#else
static const uint16_t *band_prepare_dma_words(uint16_t *buffer,
                                              uint16_t x0,
                                              uint16_t y0,
                                              uint16_t x1,
                                              uint16_t y1,
                                              uint16_t *words) {
    uint16_t width = (uint16_t)(x1 - x0 + 1u);
    uint16_t height = (uint16_t)(y1 - y0 + 1u);
    uint16_t first_row = (uint16_t)(y0 - band_y0);
    *words = (uint16_t)(width * height);

    if (width == LCD_WIDTH) {
        return &buffer[(uint32_t)first_row * LCD_WIDTH];
    }

    for (uint16_t row = 0; row < height; ++row) {
        uint32_t src = (uint32_t)(first_row + row) * LCD_WIDTH + x0;
        uint32_t dst = (uint32_t)row * width;
        for (uint16_t col = 0; col < width; ++col) {
            buffer[dst++] = buffer[src++];
        }
    }
    return buffer;
}

static uint8_t band_flush_start(uint16_t *buffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint16_t words = 0;
    const uint16_t *src = band_prepare_dma_words(buffer, x0, y0, x1, y1, &words);
    lcd_set_window(x0, y0, x1, y1);
    return lcd_dma_start_words(src, words);
}
#endif

void lcd_render(lcd_render_fn_t render, void *ctx) {
    if (!render) {
        return;
    }

    dirty_reset();
    lcd_pass = LCD_PASS_COLLECT;
    render(ctx);

    uint8_t count = dirty_count;
#if LCD_USE_FRAMEBUFFER
    band_buffer = frame_buffer;
    band_x0 = 0;
    band_y0 = 0;
    band_x1 = (uint16_t)(LCD_WIDTH - 1u);
    band_y1 = (uint16_t)(LCD_HEIGHT - 1u);
    lcd_pass = LCD_PASS_RENDER;
    render(ctx);
    lcd_pass = LCD_PASS_IDLE;

    for (uint8_t i = 0; i < count; ++i) {
        frame_flush_rect(&dirty_rects[i]);
    }
#else
    uint8_t next_buffer = 0;
    uint8_t pending_dma = 0;
    for (uint8_t i = 0; i < count; ++i) {
        const dirty_rect_t *rect = &dirty_rects[i];
        uint16_t y = rect->y0;
        while (y <= rect->y1) {
            band_buffer = band_buffers[next_buffer];
            band_x0 = rect->x0;
            band_y0 = y;
            band_x1 = rect->x1;
            band_y1 = (uint16_t)(y + LCD_BAND_HEIGHT - 1u);
            if (band_y1 > rect->y1) {
                band_y1 = rect->y1;
            }

            band_clear(band_x0, band_y0, band_x1, band_y1);
            lcd_pass = LCD_PASS_RENDER;
            render(ctx);
            if (pending_dma) {
                (void)lcd_dma_wait();
            }
            (void)band_flush_start(band_buffer, band_x0, band_y0, band_x1, band_y1);
            pending_dma = 1;
            next_buffer ^= 1u;

            if (band_y1 == rect->y1) {
                break;
            }
            y = (uint16_t)(band_y1 + 1u);
        }
    }

    if (pending_dma) {
        (void)lcd_dma_wait();
    }
    lcd_pass = LCD_PASS_IDLE;
#endif
    dirty_reset();
}

uint16_t lcd_text_width(const char *text, uint8_t scale) {
    uint16_t width = 0;
    while (*text++) {
        width = (uint16_t)(width + 6u * scale);
    }
    return width;
}

static void lcd_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    const uint8_t *glyph = font5x7_get(c);
    lcd_rect(x, y, (uint16_t)(6u * scale), (uint16_t)(8u * scale), bg);
    for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t col = 0; col < 5; ++col) {
            if (glyph[col] & (1u << row)) {
                lcd_rect((uint16_t)(x + col * scale), (uint16_t)(y + row * scale), scale, scale, fg);
            }
        }
    }
}

static void lcd_char_transparent(uint16_t x, uint16_t y, char c, uint16_t fg, uint8_t scale) {
    const uint8_t *glyph = font5x7_get(c);
    for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t col = 0; col < 5; ++col) {
            if (glyph[col] & (1u << row)) {
                lcd_rect((uint16_t)(x + col * scale), (uint16_t)(y + row * scale), scale, scale, fg);
            }
        }
    }
}

void lcd_text(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale) {
    while (*text) {
        lcd_char(x, y, *text++, fg, bg, scale);
        x = (uint16_t)(x + 6u * scale);
    }
}

void lcd_text_transparent(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint8_t scale) {
    while (*text) {
        lcd_char_transparent(x, y, *text++, fg, scale);
        x = (uint16_t)(x + 6u * scale);
    }
}

void lcd_text_center(uint16_t x, uint16_t y, uint16_t w, const char *text, uint16_t fg, uint16_t bg, uint8_t scale) {
    uint16_t tw = lcd_text_width(text, scale);
    uint16_t tx = tw >= w ? x : (uint16_t)(x + (w - tw) / 2u);
    lcd_text(tx, y, text, fg, bg, scale);
}

const uint16_t *lcd_framebuffer(void) {
#if LCD_USE_FRAMEBUFFER
    return frame_buffer;
#else
    return 0;
#endif
}
