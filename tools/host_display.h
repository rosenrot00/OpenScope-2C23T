#pragma once

// Reuse the real pixel/line/text code without touching the host's MMIO.
#define lcd_render lcd_render_hardware_unused
#include "../src/display.c"
#undef lcd_render

static void host_display_begin_render(void) {
    band_buffer = frame_buffer;
    band_x0 = band_y0 = 0;
    band_x1 = LCD_WIDTH - 1u;
    band_y1 = LCD_HEIGHT - 1u;
    lcd_pass = LCD_PASS_RENDER;
}

void lcd_render(lcd_render_fn_t render, void *ctx) {
    if (!render) return;
    lcd_pass_t previous = lcd_pass;
    dirty_reset();
    lcd_pass = LCD_PASS_COLLECT;
    render(ctx);
    host_display_begin_render();
    render(ctx);
    lcd_pass = previous;
}
