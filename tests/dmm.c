#include <assert.h>
#include <stdio.h>
#include <string.h>

// Test the real decoder without Cortex-M interrupt instructions or MMIO calls.
#define __asm__
#define volatile(...)
#define dmm_current_mode dmm_wire_mode
#include "../src/dmm.c"
#undef dmm_current_mode
#undef volatile
#undef __asm__
#include "../src/ui.c"
#include "../src/font.c"

static void make_frame(const char digits[4], unsigned dots, uint8_t prefix,
                       uint8_t flags_b, uint8_t flags_c, uint8_t frame[9]) {
    uint8_t segments[4];
    for (unsigned i = 0; i < 4; ++i) {
        const char *p = strchr(dmm_segment_chars, digits[i]);
        assert(p);
        segments[i] = dmm_segment_codes[p - dmm_segment_chars];
        if (dots & (1u << i)) segments[i] |= 0x10;
    }
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = segments[0] & 0xF0;
    frame[3] = (segments[1] & 0xF0) | (segments[0] & 0x0F);
    frame[4] = (segments[2] & 0xF0) | (segments[1] & 0x0F);
    frame[5] = (segments[3] & 0xF0) | (segments[2] & 0x0F);
    frame[6] = (segments[3] & 0x0F) | prefix;
    frame[7] = flags_b;
    frame[8] = flags_c;
}

static void setup(uint8_t mode) {
    dmm_wire_mode = ui.dmm_mode = mode;
    ui.mode = UI_MODE_DMM;
    dmm_valid = dmm_synthetic_valid = dmm_numeric_valid = dmm_baud_locked = 0;
    dmm_value_fixed = 0;
    dmm_hold_active = dmm_rel_active = 0;
    rx_pos = rx_ready = 0;
    strcpy(dmm_value, "----");
    strcpy(dmm_unit, "?");
}

static void reading(uint8_t mode, const char digits[4], unsigned dots,
                    uint8_t prefix, uint8_t flags_b, uint8_t flags_c,
                    const char *value, const char *unit) {
    uint8_t frame[9];
    setup(mode);
    make_frame(digits, dots, prefix, flags_b, flags_c, frame);
    assert(parse_frame(frame));
    assert(dmm_reading_is_real() && dmm_baud_locked);
    assert(strcmp(dmm_value_text(), value) == 0);
    assert(strcmp(dmm_unit_text(), unit) == 0);
}

static void test_stock_decoding(void) {
    // Stock digit renderers: 2.0.2 at 0x0800D1B4, 2.1.0 at 0x0800C52C.
    reading(0, "1234", 8, 0, 0, 2, "123.4", "V");
    reading(1, "1234", 8, 0, 0, 2, "123.4", "V");
    reading(2, "1234", 8, 0, 0, 2, "123.4", "V");
    reading(5, "0123", 8, 0x10, 0x10, 0, "12.3", "UF");
    reading(3, "1234", 0, 0, 0x20, 0, "1234.0", "OHM");
    reading(1, "1234", 0, 0, 0, 2, "1234", "V");
    reading(12, "1234", 2, 0x10, 0, 1, "1.234", "UA");
    assert(strcmp(dmm_live_display_value(), "1.2340") == 0);
    assert(strcmp(dmm_live_display_unit(), "uA") == 0);
    reading(10, "1234", 2, 0, 1, 1, "1.234", "MA");
    assert(strcmp(dmm_live_display_unit(), "mA") == 0);
    reading(12, "0499", 0, 0x10, 0, 1, "499", "UA");
    reading(10, "0123", 5, 0, 0, 1, "-1.23", "A");
    reading(12, "0123", 5, 0, 1, 1, "-1.23", "MA");
    reading(8, "0037", 1, 0, 0, 0x20, "-37", "DEG");
    reading(1, "3478", 9, 0, 0, 2, "-347.8", "V");
    reading(0, "0000", 8, 0, 0, 2, "0.0", "V");
    assert(strcmp(dmm_live_display_unit(), "V") == 0);
    reading(8, "0000", 0, 0, 0, 0x20, "0", "DEG");
    reading(8, "0042", 0, 0, 0, 0x10, "42", "DEG");

    uint8_t frame[9];
    for (uint8_t mode = 0; mode <= 2; ++mode) {
        setup(mode);
        make_frame("1234", 4, 0, 0, 2, frame);
        frame[2] |= 1; // Stock's AUTO AC/DC flag is not a negative sign.
        assert(parse_frame(frame));
        assert(strcmp(dmm_value_text(), "12.34") == 0);
    }

    static const uint8_t commands[] = {
        0x14, 0x0C, 0x0D, 0x0B, 0x0E, 0x0A, 0x13,
        0x12, 0x12, 0x10, 0x11, 0x15, 0x16,
    };
    assert(sizeof(commands) == sizeof(dmm_mode_command));
    assert(memcmp(commands, dmm_mode_command, sizeof(commands)) == 0);
}

static void test_all_numeric_digits(void) {
    static const struct {
        uint8_t mode, prefix, flags_b, flags_c;
        const char *unit;
    } kinds[] = {
        {0, 0, 0, 2, "V"}, {1, 0, 0, 2, "V"}, {2, 0, 1, 2, "MV"},
        {3, 0x40, 0x20, 0, "KOHM"}, {4, 0, 0, 2, "V"},
        {5, 0x20, 0x10, 0, "NF"}, {8, 0, 0, 0x20, "DEG"},
        {9, 0, 0, 1, "A"}, {10, 0, 1, 1, "MA"},
        {11, 0x10, 0, 1, "UA"}, {12, 0, 0, 1, "A"},
    };
    for (unsigned kind = 0; kind < sizeof(kinds) / sizeof(kinds[0]); ++kind) {
        setup(kinds[kind].mode);
        for (unsigned value = 0; value <= 9999; ++value) {
            char digits[5];
            snprintf(digits, sizeof(digits), "%04u", value);
            for (unsigned point = 0; point <= 3; ++point) {
                int32_t factor = point == 1 ? 10 : point == 2 ? 100 : point == 3 ? 1000 : DMM_VALUE_SCALE;
                for (unsigned negative = 0; negative <= 1; ++negative) {
                    uint8_t frame[9];
                    make_frame(digits, (point ? 1u << point : 0) | negative,
                               kinds[kind].prefix, kinds[kind].flags_b, kinds[kind].flags_c, frame);
                    parse_frame(frame);
                    assert(dmm_has_reading() && dmm_value_is_numeric());
                    assert(dmm_value_fixed_units() == (negative ? -(int32_t)value : (int32_t)value) * factor);
                    assert(strcmp(dmm_unit_text(), kinds[kind].unit) == 0);
                }
            }
        }
    }
}

static void test_status_and_rejected_frames(void) {
    reading(0, "AUTO", 0, 0, 0, 2, "AUTO", "V");
    assert(strcmp(dmm_live_display_unit(), "") == 0);
    for (unsigned point = 0; point <= 3; ++point) {
        reading(3, "00L0", point ? 1u << point : 0, 0, 0x24, 0, "OPEN", "MOHM");
        assert(!dmm_value_is_numeric());
        assert(strcmp(dmm_live_display_unit(), "") == 0);
        reading(4, "00L0", point ? 1u << point : 0, 0, 0, 2, "OPEN", "V");
        assert(!dmm_diode_continuity_active());
    }
    reading(6, "L1UE", 0, 0, 0, 0, "L1UE", "");
    assert(!dmm_live_wire_active());
    reading(6, "00L0", 0, 0, 0, 0, "00L0", "");
    assert(dmm_live_wire_active());
    reading(6, "L1UE", 0, 0, 0, 2, "L1UE", "V");
    assert(!dmm_live_wire_active());
    reading(6, "00L0", 0, 0, 0, 2, "00L0", "V");
    assert(dmm_live_wire_active());
    reading(6, "0000", 0, 0, 0, 0, "L1UE", "");
    assert(!dmm_live_wire_active());
    reading(8, "0037", 0, 0, 0, 0x20, "37", "DEG");
    uint8_t frame[9];
    make_frame("00L0", 0, 0, 0, 0x20, frame);
    assert(!parse_frame(frame));
    assert(strcmp(dmm_value_text(), "37") == 0);

    setup(3);
    make_frame("L1UE", 0, 0, 0, 0, frame);
    assert(!parse_frame(frame) && dmm_baud_locked && !dmm_has_reading());
    dmm_baud_locked = 0;
    make_frame("1234", 2, 0, 0, 2, frame);
    assert(!parse_frame(frame) && dmm_baud_locked); // Do not relabel stale volts as ohms.
    setup(8);
    make_frame("00L0", 0, 0, 0, 0x20, frame);
    assert(!parse_frame(frame) && dmm_baud_locked && !dmm_has_reading());
    setup(5);
    make_frame("1234", 2, 0, 0, 2, frame);
    assert(!parse_frame(frame) && dmm_baud_locked && !dmm_has_reading());
    dmm_started = dmm_powered = 1;
    dmm_settle_ms = dmm_retry_ms = 0;
    uint8_t baud = dmm_baud_index;
    dmm_tick(3000);
    assert(dmm_baud_index == baud && dmm_retry_ms == 0);
    dmm_started = dmm_powered = 0;
    setup(1);
    make_frame("1234", 2, 0, 0, 2, frame);
    frame[2] = 0x20;
    frame[3] &= 0xF0;
    assert(!parse_frame(frame) && !dmm_baud_locked);
    make_frame("1234", 6, 0, 0, 2, frame);
    assert(!parse_frame(frame) && !dmm_baud_locked);
    make_frame("1234", 2, 0, 0, 0, frame);
    assert(!parse_frame(frame) && !dmm_baud_locked);
}

static void test_relative_precision(void) {
    setup(12);
    dmm_valid = dmm_numeric_valid = 1;
    strcpy(dmm_unit, "MA");
    strcpy(dmm_value, "0.0001");
    assert(parse_fixed_units(dmm_value, &dmm_value_fixed));
    dmm_toggle_relative();
    assert(dmm_rel_active && strcmp(dmm_display_value(), "0.0000") == 0);
    strcpy(dmm_value, "0.0002");
    assert(parse_fixed_units(dmm_value, &dmm_value_fixed));
    assert(strcmp(dmm_display_value(), "0.0001") == 0);
    assert(strcmp(dmm_rel_ref_value, "0.0001") == 0);
    strcpy(dmm_value, "-0.0001");
    assert(parse_fixed_units(dmm_value, &dmm_value_fixed));
    assert(strcmp(dmm_display_value(), "-0.0002") == 0);
    strcpy(dmm_unit, "UA");
    assert(strcmp(dmm_display_value(), "RANGE") == 0);
    format_signed_fixed(-99999999, dmm_rel_value);
    assert(strcmp(dmm_rel_value, "-9999.9999") == 0);
    assert(parse_fixed_units("0.00015", &dmm_value_fixed) && dmm_value_fixed == 2);
    assert(parse_fixed_units("0.99995", &dmm_value_fixed) && dmm_value_fixed == DMM_VALUE_SCALE);
    assert(parse_fixed_units("-1.2345", &dmm_value_fixed));
    assert(dmm_value_milli_units() == -1235);
}

static void test_synthetic_readings(void) {
    for (uint8_t mode = 0; mode < sizeof(dmm_mode_command); ++mode) {
        setup(mode);
        dmm_apply_missing_uart_fallback();
        assert(dmm_has_reading() && !dmm_reading_is_real());
        assert(!dmm_diode_continuity_active() && !dmm_live_wire_active());
        assert(!dmm_baud_locked);
    }
    setup(12);
    dmm_apply_missing_uart_fallback();
    assert(strcmp(dmm_live_display_value(), "0.0000") == 0);
    setup(0);
    dmm_apply_missing_uart_fallback();
    assert(strcmp(dmm_live_display_value(), "AUTO") == 0);
    assert(strcmp(dmm_live_display_unit(), "") == 0);
}

static void test_diode_thresholds(void) {
    reading(4, "0010", 8, 0x40, 0x20, 0, "1.0", "KOHM");
    assert(!dmm_diode_continuity_active());
    reading(4, "0050", 2, 0x40, 0x20, 0, "0.050", "KOHM");
    assert(dmm_diode_continuity_active());
    reading(4, "0051", 2, 0x40, 0x20, 0, "0.051", "KOHM");
    assert(!dmm_diode_continuity_active());
    reading(4, "0001", 2, 0, 0x24, 0, "0.001", "MOHM");
    assert(!dmm_diode_continuity_active());
    reading(4, "5000", 4, 0, 1, 2, "50.00", "MV");
    assert(dmm_diode_continuity_active());
    reading(4, "0200", 2, 0, 0, 2, "0.200", "V");
    assert(!dmm_diode_continuity_active());
    reading(4, "0200", 3, 0, 0, 2, "-0.200", "V");
    assert(!dmm_diode_continuity_active());
    reading(4, "0050", 3, 0, 0, 2, "-0.050", "V");
    assert(dmm_diode_continuity_active());
    dmm_synthetic_valid = 1;
    assert(!dmm_diode_continuity_active());
}

static void receive(const uint8_t frame[9], unsigned count) {
    for (unsigned i = 0; i < count; ++i) dmm_rx_event(USART_STS_RXNE, frame[i]);
}

static void test_packet_boundaries(void) {
    uint8_t frame[9];
    setup(1);
    make_frame("1234", 8, 0, 0, 2, frame);
    for (unsigned cut = 1; cut < 9; ++cut) {
        rx_pos = rx_ready = 0;
        receive(frame, cut);
        assert(!rx_ready);
        dmm_rx_event(USART_STS_IDLE, 0);
        assert(rx_pos == 0);
        receive(frame, 9);
        assert(rx_ready && memcmp((const void *)rx_pending, frame, 9) == 0);
        dmm_rx_event(USART_STS_IDLE, 0);
        assert(rx_ready && rx_pos == 0);
    }
    static const uint32_t errors[] = {USART_STS_FE, USART_STS_NE, USART_STS_ORE};
    for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        rx_pos = rx_ready = 0;
        receive(frame, 4);
        dmm_rx_event(errors[i] | USART_STS_RXNE, frame[4]);
        assert(rx_pos == 0 && !rx_ready);
        receive(frame, 9);
        assert(rx_ready && memcmp((const void *)rx_pending, frame, 9) == 0);
    }
    rx_pos = rx_ready = 0;
    receive(frame, 4);
    dmm_rx_event(USART_STS_IDLE | USART_STS_RXNE, frame[0]);
    assert(rx_pos == 1);
    receive(frame + 1, 8);
    assert(rx_ready && memcmp((const void *)rx_pending, frame, 9) == 0);
}

// Case carries the meaning in SI prefixes, so the screen must not shout the
// units: "MV" reads as megavolt, "US" is not a unit at all, and "MOHM" was
// indistinguishable from a milliohm. The USB protocol keeps its own upper
// case spelling, which is why only the display text is translated.
static void test_display_units_are_si(void) {
    assert(strcmp(dmm_unit_si_text("MV"), "mV") == 0);
    assert(strcmp(dmm_unit_si_text("MA"), "mA") == 0);
    assert(strcmp(dmm_unit_si_text("UA"), "uA") == 0);
    assert(strcmp(dmm_unit_si_text("NF"), "nF") == 0);
    assert(strcmp(dmm_unit_si_text("UF"), "uF") == 0);
    assert(strcmp(dmm_unit_si_text("MF"), "mF") == 0);

    // milli and mega must no longer collide.
    assert(strcmp(dmm_unit_si_text("OHM"), "Ohm") == 0);
    assert(strcmp(dmm_unit_si_text("KOHM"), "kOhm") == 0);
    assert(strcmp(dmm_unit_si_text("MOHM"), "MOhm") == 0);
    assert(strcmp(dmm_unit_si_text("KOHM"), dmm_unit_si_text("MOHM")) != 0);

    // Unambiguous units pass through, and a null never crashes the display.
    assert(strcmp(dmm_unit_si_text("V"), "V") == 0);
    assert(strcmp(dmm_unit_si_text("A"), "A") == 0);
    // Temperature is shown as a degree sign plus C; the font carries the
    // degree sign on '~', which is why the unit is spelled that way.
    assert(strcmp(dmm_unit_si_text("DEG"), "~C") == 0);
    assert(strcmp(dmm_unit_si_text(NULL), "") == 0);

    // Every character these units are made of must have a glyph of its own.
    // font5x7_get() falls back to a blank for an unknown character, so a
    // missing glyph would quietly erase part of the unit rather than fail
    // loudly -- and that blank is exactly what the SI spelling buys back.
    static const char *const kUnits[] = {
        "MV", "MA", "UA", "NF", "UF", "MF", "OHM", "KOHM", "MOHM", "DEG",
    };
    const uint8_t *blank = font5x7_get(1);   // 1 has no glyph: the blank one
    for (unsigned u = 0; u < sizeof(kUnits) / sizeof(kUnits[0]); ++u) {
        for (const char *p = dmm_unit_si_text(kUnits[u]); *p; ++p) {
            assert(font5x7_get(*p) != blank);
        }
    }
}

// The scope ranges have the multimeter's problem and the same cure: one
// spelling for the host, which has parsed "500M" since before any of this,
// and one for the eye, which reads "500M" as five hundred megavolts.
// What draw_scope_channel_range() ends up occupying: the whole label at
// scale 2 when it fits, otherwise the number at scale 2 and the unit at
// scale 1 beside it.
static unsigned range_field_width(const char *value, const char *unit) {
    unsigned number = (4u + (unsigned)strlen(value)) * 12u;
    unsigned full = number + (unsigned)strlen(unit) * 12u;

    return full <= SCOPE_RANGE_FIELD_W ? full : number + (unsigned)strlen(unit) * 6u;
}

static void test_scope_ranges_are_readable(void) {
    const uint8_t *blank = font5x7_get(1);
    unsigned count = sizeof(scope_vdivs_short) / sizeof(scope_vdivs_short[0]);

    assert(count == SCOPE_VDIV_COUNT);
    assert(sizeof(scope_vdiv_value) / sizeof(scope_vdiv_value[0]) == count);
    assert(sizeof(scope_vdiv_unit) / sizeof(scope_vdiv_unit[0]) == count);
    assert(sizeof(scope_vdiv_x10_value) / sizeof(scope_vdiv_x10_value[0]) == count);
    assert(sizeof(scope_vdiv_x10_unit) / sizeof(scope_vdiv_x10_unit[0]) == count);

    for (unsigned i = 0; i < count; ++i) {
        const char *parts[] = {
            scope_vdiv_value[i], scope_vdiv_unit[i],
            scope_vdiv_x10_value[i], scope_vdiv_x10_unit[i],
        };
        for (unsigned p = 0; p < 4u; ++p) {
            for (const char *c = parts[p]; *c; ++c) {
                assert(font5x7_get(*c) != blank);
            }
        }
        // The bottom bar gives each channel SCOPE_RANGE_FIELD_W pixels
        // and the timebase starts immediately after, so a range that
        // overflows does not wrap -- it writes over the field beside it.
        // The bar draws the unit at full size when the pair fits and at
        // half size otherwise, so the narrower of the two has to fit.
        assert(range_field_width(scope_vdiv_value[i], scope_vdiv_unit[i]) <=
               SCOPE_RANGE_FIELD_W);
        assert(range_field_width(scope_vdiv_x10_value[i], scope_vdiv_x10_unit[i]) <=
               SCOPE_RANGE_FIELD_W);
    }

    // The version string is shown as written, lower case v and all.
    for (const char *c = FIRMWARE_VERSION_TEXT; *c; ++c) {
        assert(font5x7_get(*c) != blank);
    }
    assert(font5x7_get('v') != font5x7_get('V'));
}

int main(void) {
    test_stock_decoding();
    test_display_units_are_si();
    test_scope_ranges_are_readable();
    test_all_numeric_digits();
    test_status_and_rejected_frames();
    test_relative_precision();
    test_synthetic_readings();
    test_diode_thresholds();
    test_packet_boundaries();
    assert(strcmp(settings_value_text(SETTINGS_ROW_COUNT - 1u, NULL), EXPECTED_FW_VERSION) == 0);
    puts("DMM decoding, display, REL, continuity and UART framing tests passed");
    return 0;
}
