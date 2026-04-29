#include "dmm.h"

#include "board.h"
#include "hw.h"

#include <stdint.h>

enum {
    USART_STS_FE = 1u << 1,
    USART_STS_NE = 1u << 2,
    USART_STS_ORE = 1u << 3,
    USART_STS_RXNE = 1u << 5,
    USART_STS_TXE = 1u << 7,

    USART_CTRL1_RE = 1u << 2,
    USART_CTRL1_TE = 1u << 3,
    USART_CTRL1_RXNEIE = 1u << 5,
    USART_CTRL1_UE = 1u << 13,

    DMM_FRAME_LEN = 9,
    DMM_CMD_LEN = 10,
    DMM_FALLBACK_PCLK_COUNT = 6,
    DMM_BAUD_CANDIDATE_COUNT = DMM_FALLBACK_PCLK_COUNT + 1,
    DMM_BAUD_RETRY_MS = 600,
    DMM_MODE_SWITCH_SETTLE_MS = 120,
};

#ifndef DMM_UART_BAUD
#define DMM_UART_BAUD 9600u
#endif

static const uint8_t dmm_mode_command[] = {
    0x14, 0x0C, 0x0D, 0x0B, 0x0E, 0x0A, 0x13,
    0x12, 0x12, 0x10, 0x11, 0x15, 0x16,
};

static const uint8_t dmm_segment_codes[] = {
    0xEB, 0x0A, 0xAD, 0x8F, 0x4E, 0xC7, 0xE7, 0x8A,
    0xEF, 0xCF, 0xEE, 0x23, 0x65, 0x27, 0x61, 0xE5,
};

static const char dmm_segment_chars[] = "0123456789AUTOLE";

static const uint32_t dmm_fallback_pclk[DMM_FALLBACK_PCLK_COUNT] = {
    36000000u,
    72000000u,
    96000000u,
    108000000u,
    120000000u,
    144000000u,
};

static volatile uint8_t rx_pos;
static volatile uint8_t rx_capture[DMM_FRAME_LEN];
static volatile uint8_t rx_pending[DMM_FRAME_LEN];
static volatile uint8_t rx_ready;
static volatile uint16_t rx_byte_count;
static volatile uint16_t rx_sync_count;

static uint8_t dmm_started;
static uint8_t dmm_powered;
static uint8_t dmm_current_mode;
static uint8_t dmm_valid;
static uint8_t dmm_synthetic_valid;
static uint8_t dmm_numeric_valid;
static uint8_t dmm_baud_index;
static uint8_t dmm_baud_locked;
static uint16_t dmm_retry_ms;
static uint16_t dmm_settle_ms;
static uint16_t dmm_detected_brr;
static int32_t dmm_value_milli;
static char dmm_value[10] = "----";
static char dmm_unit[6] = "V";
static char dmm_status[9] = "B0 RX00";

static void text_copy(char *dst, const char *src, uint8_t max_len);
static void format_unit(uint8_t flags_a, uint8_t flags_b, uint8_t flags_c, char out[6]);

static uint32_t rcc_sysclk_hz(void) {
    static const uint8_t ahb_shift[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9,
    };
    static const uint8_t apb_shift[] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t cfgr = RCC_CFGR;
    uint32_t sysclk = 8000000u;
    uint32_t sws = (cfgr >> 2) & 3u;

    if (sws == 2u) {
        uint32_t mult_bits = (cfgr >> 18) & 0x0Fu;
        uint32_t ext_bits = (cfgr >> 29) & 0x03u;
        uint32_t mult = (ext_bits || mult_bits == 15u) ? (mult_bits + 16u * ext_bits + 1u) : (mult_bits + 2u);
        uint32_t base = 4000000u;
        if (cfgr & (1u << 16)) {
            base = 8000000u;
        }
        sysclk = base * mult;
    } else if (sws == 1u) {
        sysclk = 8000000u;
    }

    sysclk >>= ahb_shift[(cfgr >> 4) & 0x0Fu];
    sysclk >>= apb_shift[(cfgr >> 8) & 0x07u];
    return sysclk;
}

static uint16_t dmm_brr_for_pclk(uint32_t pclk) {
    uint32_t brr = (pclk + (DMM_UART_BAUD / 2u)) / DMM_UART_BAUD;
    if (!brr || brr > 60000u) {
        return (uint16_t)((72000000u + (DMM_UART_BAUD / 2u)) / DMM_UART_BAUD);
    }
    return (uint16_t)brr;
}

static uint16_t dmm_compute_brr(void) {
    return dmm_brr_for_pclk(rcc_sysclk_hz());
}

static uint16_t dmm_current_brr(void) {
    if (dmm_baud_index == 0u) {
        return dmm_detected_brr;
    }
    return dmm_brr_for_pclk(dmm_fallback_pclk[dmm_baud_index - 1u]);
}

static void dmm_format_status(void) {
    uint16_t rx = rx_byte_count % 100u;

    dmm_status[0] = 'B';
    dmm_status[1] = (char)('0' + dmm_baud_index);
    dmm_status[2] = ' ';
    dmm_status[3] = 'R';
    dmm_status[4] = 'X';
    dmm_status[5] = (char)('0' + rx / 10u);
    dmm_status[6] = (char)('0' + rx % 10u);
    dmm_status[7] = 0;
}

static void dmm_uart_apply_brr(uint16_t brr) {
    USART_CTRL1(USART3_BASE) = 0;
    USART_BAUDR(USART3_BASE) = brr;
    (void)USART_STS(USART3_BASE);
    (void)USART_DT(USART3_BASE);
    USART_CTRL1(USART3_BASE) = USART_CTRL1_UE | USART_CTRL1_RXNEIE | USART_CTRL1_TE | USART_CTRL1_RE;
}

static void dmm_uart_drain_rx(void) {
    for (uint8_t i = 0; i < 32u; ++i) {
        uint32_t status = USART_STS(USART3_BASE);
        if (!(status & (USART_STS_RXNE | USART_STS_ORE | USART_STS_NE | USART_STS_FE))) {
            break;
        }
        (void)USART_DT(USART3_BASE);
    }
}

static void dmm_rx_clear(uint8_t reset_counters) {
    __asm__ volatile("cpsid i" ::: "memory");
    dmm_uart_drain_rx();
    rx_pos = 0;
    rx_ready = 0;
    if (reset_counters) {
        rx_byte_count = 0;
        rx_sync_count = 0;
    }
    __asm__ volatile("cpsie i" ::: "memory");
}

static void dmm_rx_reset_counters(void) {
    dmm_rx_clear(1);
    dmm_retry_ms = 0;
    dmm_format_status();
}

static char decode_segment(uint8_t segment) {
    for (uint8_t i = 0; i < (uint8_t)sizeof(dmm_segment_codes); ++i) {
        if (dmm_segment_codes[i] == segment) {
            return dmm_segment_chars[i];
        }
    }
    return '?';
}

static uint8_t char_is_digit(char c) {
    return c >= '0' && c <= '9';
}

static uint8_t text_equal(const char *a, const char *b) {
    while (*a && *b) {
        if (*a++ != *b++) {
            return 0;
        }
    }
    return *a == *b;
}

static uint8_t parse_milli_units(const char *text, int32_t *value) {
    uint8_t negative = 0;
    uint8_t seen_dot = 0;
    uint8_t digits = 0;
    uint8_t frac_digits = 0;
    uint8_t round_up = 0;
    uint8_t round_checked = 0;
    int32_t whole = 0;
    int32_t frac = 0;

    if (*text == '-') {
        negative = 1;
        ++text;
    }

    while (*text) {
        if (*text >= '0' && *text <= '9') {
            uint8_t digit = (uint8_t)(*text - '0');
            ++digits;
            if (seen_dot) {
                if (frac_digits < 3u) {
                    frac = frac * 10 + digit;
                    ++frac_digits;
                } else if (!round_checked) {
                    round_up = digit >= 5u ? 1u : 0u;
                    round_checked = 1;
                }
            } else {
                whole = whole * 10 + digit;
            }
        } else if (*text == '.' && !seen_dot) {
            seen_dot = 1;
        } else {
            return 0;
        }
        ++text;
    }

    if (!digits) {
        return 0;
    }

    while (frac_digits < 3u) {
        frac *= 10;
        ++frac_digits;
    }
    if (round_up) {
        ++frac;
        if (frac >= 1000) {
            ++whole;
            frac = 0;
        }
    }

    *value = whole * 1000 + frac;
    if (negative) {
        *value = -*value;
    }
    return 1;
}

static void text_copy(char *dst, const char *src, uint8_t max_len) {
    uint8_t i = 0;
    if (!max_len) {
        return;
    }
    while (i < (uint8_t)(max_len - 1u) && src[i]) {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = 0;
}

static void default_unit_for_mode(uint8_t mode, char out[6]) {
    if (mode == 6u) {
        text_copy(out, "", 6);
    } else if (mode == 3u) {
        text_copy(out, "MOHM", 6);
    } else if (mode == 7u) {
        text_copy(out, "OHM", 6);
    } else if (mode == 5u) {
        text_copy(out, "NF", 6);
    } else if (mode == 8u) {
        text_copy(out, "DEG", 6);
    } else if (mode == 11u || mode == 12u) {
        text_copy(out, "MA", 6);
    } else if (mode == 9u || mode == 10u) {
        text_copy(out, "A", 6);
    } else {
        text_copy(out, "V", 6);
    }
}

static void default_value_for_mode(uint8_t mode, char out[10]) {
    if (mode == 0u) {
        text_copy(out, "AUTO", 10);
    } else if (mode == 3u) {
        text_copy(out, "00.L0", 10);
    } else if (mode == 4u) {
        text_copy(out, "0.0L0", 10);
    } else if (mode == 7u) {
        text_copy(out, "0.0", 10);
    } else if (mode == 5u) {
        text_copy(out, "0.000", 10);
    } else if (mode == 6u) {
        text_copy(out, "L1UE", 10);
    } else if (mode == 8u) {
        text_copy(out, "0000", 10);
    } else if (mode == 12u) {
        text_copy(out, "0000", 10);
    } else {
        text_copy(out, "0.000", 10);
    }
}

static uint8_t mode_uses_one_decimal_minimum(void) {
    return dmm_current_mode == 3u || dmm_current_mode == 4u || dmm_current_mode == 7u;
}

static uint8_t mode_is_high_current(void) {
    return dmm_current_mode == 9u || dmm_current_mode == 10u;
}

static uint8_t mode_is_low_current(void) {
    return dmm_current_mode == 11u || dmm_current_mode == 12u;
}

static void format_implicit_decimal_value(const char digits[5], uint8_t frac_digits, uint8_t negative, char out[10]) {
    char scaled[12];
    uint8_t pos = 0;
    uint8_t src = 0;
    uint8_t integer_digits;

    if (frac_digits > 4u) {
        frac_digits = 4u;
    }
    integer_digits = (uint8_t)(4u - frac_digits);

    if (negative) {
        scaled[pos++] = '-';
    }
    if (!integer_digits) {
        scaled[pos++] = '0';
    } else {
        for (; src < integer_digits && pos < (uint8_t)(sizeof(scaled) - 1u); ++src) {
            scaled[pos++] = digits[src];
        }
    }
    if (frac_digits && pos < (uint8_t)(sizeof(scaled) - 1u)) {
        scaled[pos++] = '.';
        for (; src < 4u && pos < (uint8_t)(sizeof(scaled) - 1u); ++src) {
            scaled[pos++] = digits[src];
        }
    }
    scaled[pos] = 0;

    pos = negative ? 1u : 0u;
    while (scaled[pos] == '0' && scaled[pos + 1u] && scaled[pos + 1u] != '.') {
        ++pos;
    }
    if (negative) {
        out[0] = '-';
        text_copy(&out[1], &scaled[pos], 9);
    } else {
        text_copy(out, &scaled[pos], 10);
    }
}

static void format_unit(uint8_t flags_a, uint8_t flags_b, uint8_t flags_c, char out[6]) {
    char kind = 0;
    char prefix = 0;

    if (flags_b & 0x20u) {
        kind = 'R';
    } else if (flags_b & 0x10u) {
        kind = 'F';
    } else if (flags_c & 0x02u) {
        kind = 'V';
    } else if (flags_c & 0x01u) {
        kind = 'A';
    } else if (flags_c & 0x20u) {
        kind = 'C';
    } else if (flags_c & 0x10u) {
        kind = 'D';
    }

    if (flags_a & 0x40u) {
        prefix = 'K';
    } else if (flags_a & 0x20u) {
        prefix = 'N';
    } else if (flags_a & 0x10u) {
        prefix = 'U';
    } else if (flags_b & 0x04u) {
        prefix = 'M';
    } else if (flags_b & 0x01u) {
        prefix = 'm';
    }

    if (kind == 'R') {
        if (prefix == 'K') {
            text_copy(out, "KOHM", 6);
        } else if (prefix == 'M') {
            text_copy(out, "MOHM", 6);
        } else {
            text_copy(out, "OHM", 6);
        }
    } else if (kind == 'F') {
        if (prefix == 'N') {
            text_copy(out, "NF", 6);
        } else if (prefix == 'U') {
            text_copy(out, "UF", 6);
        } else if (prefix == 'm' || prefix == 'M') {
            text_copy(out, "MF", 6);
        } else {
            text_copy(out, "F", 6);
        }
    } else if (kind == 'V') {
        if (prefix == 'm') {
            text_copy(out, "MV", 6);
        } else {
            text_copy(out, "V", 6);
        }
    } else if (kind == 'A') {
        if (mode_is_high_current()) {
            text_copy(out, "A", 6);
        } else if (mode_is_low_current()) {
            text_copy(out, "MA", 6);
        } else if (prefix == 'm') {
            text_copy(out, "MA", 6);
        } else if (prefix == 'U') {
            text_copy(out, "UA", 6);
        } else {
            text_copy(out, "A", 6);
        }
    } else if (kind == 'C') {
        text_copy(out, "DEG", 6);
    } else if (kind == 'D') {
        text_copy(out, "V", 6);
    } else {
        default_unit_for_mode(dmm_current_mode, out);
    }
}

static void normalize_current_reading(char value[10], char unit[6], int32_t *milli_units) {
    if (mode_is_high_current()) {
        (void)value;
        (void)milli_units;
        text_copy(unit, "A", 6);
    } else if (mode_is_low_current()) {
        (void)value;
        (void)milli_units;
        text_copy(unit, "MA", 6);
    }
}

static uint8_t text_has_char(const char *text, char needle) {
    while (*text) {
        if (*text++ == needle) {
            return 1;
        }
    }
    return 0;
}

static uint8_t text_alpha_count(const char *text) {
    uint8_t count = 0;

    while (*text) {
        if (!char_is_digit(*text) && *text != '.') {
            ++count;
        }
        ++text;
    }
    return count;
}

static void normalize_plain_numeric_text(char text[10]) {
    uint8_t start = 0;

    if (!text[0] || text_has_char(text, '.')) {
        return;
    }
    while (text[start] == '0' && text[start + 1u]) {
        ++start;
    }
    if (start) {
        text_copy(text, &text[start], 10);
    }
}

static void dmm_apply_missing_uart_fallback(void) {
    char wire_value[10];
    char wire_unit[6];
    char new_value[10];
    char new_unit[6];
    int32_t new_milli = 0;
    uint8_t new_numeric;

    default_value_for_mode(dmm_current_mode, wire_value);
    default_unit_for_mode(dmm_current_mode, wire_unit);

    text_copy(new_value, wire_value, sizeof(new_value));
    text_copy(new_unit, wire_unit, sizeof(new_unit));
    if (text_has_char(new_value, 'L')) {
        if (text_has_char(new_value, '.')) {
            text_copy(new_value, "OPEN", sizeof(new_value));
        } else if (text_alpha_count(new_value) == 1u) {
            text_copy(new_value, "0.0", sizeof(new_value));
        }
    } else if (mode_is_low_current() && !text_has_char(new_value, '.')) {
        char digits[5] = "0000";
        uint8_t idx = 0;

        for (uint8_t i = 0; new_value[i] && idx < 4u; ++i) {
            if (char_is_digit(new_value[i])) {
                digits[idx++] = new_value[i];
            }
        }
        format_implicit_decimal_value(digits, 4u, 0, new_value);
    } else {
        normalize_plain_numeric_text(new_value);
    }

    new_numeric = parse_milli_units(new_value, &new_milli);
    if (new_numeric) {
        normalize_current_reading(new_value, new_unit, &new_milli);
    }
    text_copy(dmm_value, new_value, sizeof(dmm_value));
    text_copy(dmm_unit, new_unit, sizeof(dmm_unit));
    dmm_numeric_valid = new_numeric;
    dmm_value_milli = new_milli;
    dmm_valid = 1;
    dmm_synthetic_valid = 1;
}

static uint8_t unit_is_voltage(const char *unit) {
    return text_equal(unit, "V") || text_equal(unit, "MV");
}

static uint8_t unit_is_resistance(const char *unit) {
    return text_equal(unit, "OHM") || text_equal(unit, "KOHM") || text_equal(unit, "MOHM");
}

static uint8_t unit_is_current(const char *unit) {
    return text_equal(unit, "A") || text_equal(unit, "MA") || text_equal(unit, "UA");
}

static uint8_t reading_uses_one_decimal_minimum(const char *unit) {
    return mode_uses_one_decimal_minimum() || unit_is_resistance(unit);
}

static uint8_t reading_implicit_fraction_digits(const char *unit, uint8_t digit_count) {
    if (unit_is_current(unit) && digit_count == 4u) {
        return 4u;
    }
    if (unit_is_resistance(unit)) {
        return digit_count > 0u ? 1u : 0u;
    }
    return dmm_current_mode == 2u && text_equal(unit, "V") && digit_count == 4u ? 1u : 0u;
}

static uint8_t unit_is_capacitance(const char *unit) {
    return text_equal(unit, "F") || text_equal(unit, "NF") ||
           text_equal(unit, "UF") || text_equal(unit, "MF");
}

static uint8_t dmm_reading_compatible(const char *value, const char *unit, uint8_t numeric) {
    if (dmm_current_mode == 0u) {
        return 1;
    }
    if (!numeric && text_equal(value, "OPEN")) {
        return 1;
    }
    if (!numeric && text_equal(value, "O.L")) {
        return dmm_current_mode == 3u || dmm_current_mode == 4u || dmm_current_mode == 5u;
    }

    if (dmm_current_mode == 1u || dmm_current_mode == 2u) {
        return unit_is_voltage(unit);
    }
    if (dmm_current_mode == 3u || dmm_current_mode == 4u || dmm_current_mode == 7u) {
        return 1;
    }
    if (dmm_current_mode == 5u) {
        return unit_is_capacitance(unit);
    }
    if (dmm_current_mode == 6u) {
        return text_equal(unit, "") || text_equal(value, "L1UE") || text_equal(value, "LIVE");
    }
    if (dmm_current_mode == 8u) {
        return text_equal(unit, "DEG");
    }
    if (mode_is_high_current()) {
        return text_equal(unit, "A");
    }
    if (mode_is_low_current()) {
        return text_equal(unit, "MA");
    }
    return 1;
}

static uint8_t format_value(const uint8_t frame[DMM_FRAME_LEN], const char *unit, char out[10]) {
    uint8_t segments[4];
    char digits[5];
    char raw[9];
    uint8_t len = 0;
    uint8_t has_alpha = 0;
    uint8_t alpha_count = 0;
    uint8_t has_l = 0;
    uint8_t has_dot = 0;
    uint8_t negative = frame[2] & 0x01u;
    uint8_t all_zero = 1;
    uint8_t implicit_frac_digits = 0;
    int8_t dot_index = -1;

    segments[0] = (uint8_t)((frame[2] & 0xF0u) | (frame[3] & 0x0Fu));
    segments[1] = (uint8_t)((frame[3] & 0xF0u) | (frame[4] & 0x0Fu));
    segments[2] = (uint8_t)((frame[4] & 0xF0u) | (frame[5] & 0x0Fu));
    segments[3] = (uint8_t)((frame[5] & 0xF0u) | (frame[6] & 0x0Fu));

    for (uint8_t i = 0; i < 3u; ++i) {
        if (segments[i] & 0x10u) {
            dot_index = (int8_t)i;
        }
    }
    has_dot = dot_index >= 0 ? 1u : 0u;

    for (uint8_t i = 0; i < 4u; ++i) {
        uint8_t masked = (uint8_t)(segments[i] & 0xEFu);
        char c = masked ? decode_segment(masked) : '0';
        if (c == '?') {
            return 0;
        }
        if (!char_is_digit(c)) {
            has_alpha = 1;
            ++alpha_count;
            if (c == 'L') {
                has_l = 1;
            }
        } else if (c != '0') {
            all_zero = 0;
        }
        digits[i] = c;
    }
    digits[4] = 0;

    if (dmm_current_mode == 8u && has_l) {
        return 0;
    }

    if (has_alpha) {
        if (has_l && has_dot) {
            text_copy(out, "OPEN", 10);
            return 1;
        }
        if (has_l && alpha_count == 1u) {
            text_copy(out, "0.0", 10);
            return 1;
        }
        text_copy(out, digits, 10);
        return 1;
    }

    if (!has_dot) {
        implicit_frac_digits = reading_implicit_fraction_digits(unit, 4u);
        if (implicit_frac_digits && mode_is_high_current() && digits[0] != '0') {
            implicit_frac_digits = 2u;
        }
        if (implicit_frac_digits) {
            format_implicit_decimal_value(digits, implicit_frac_digits, negative, out);
            return 1;
        }
    }

    if (all_zero) {
        negative = 0;
        if (dot_index < 0) {
            if (dmm_current_mode == 8u) {
                text_copy(out, "0", 10);
                return 1;
            }
            if (reading_uses_one_decimal_minimum(unit)) {
                text_copy(out, "0.0", 10);
                return 1;
            }
            default_value_for_mode(dmm_current_mode, out);
            return 1;
        }
    }

    for (uint8_t i = 0; i < 4u; ++i) {
        if ((int8_t)i == dot_index && len < (uint8_t)(sizeof(raw) - 1u)) {
            raw[len++] = '.';
            has_dot = 1;
        }
        if (len < (uint8_t)(sizeof(raw) - 1u)) {
            raw[len++] = digits[i];
        }
    }
    raw[len] = 0;

    uint8_t start = 0;
    while (raw[start] == '0' && raw[start + 1u] && raw[start + 1u] != '.') {
        ++start;
    }

    if (reading_uses_one_decimal_minimum(unit) && raw[start] == '0' && raw[start + 1u] == 0) {
        text_copy(out, "0.0", 10);
        return 1;
    }

    uint8_t out_len = 0;
    uint8_t remaining_digits = 0;
    uint8_t output_has_dot = has_dot;

    if (!has_dot) {
        for (uint8_t i = start; raw[i]; ++i) {
            if (char_is_digit(raw[i])) {
                ++remaining_digits;
            }
        }
    }

    if (negative && out_len < 9u) {
        out[out_len++] = '-';
    }
    if (has_dot && raw[start] == '.') {
        out[out_len++] = '0';
    }
    while (raw[start] && out_len < 9u) {
        if (char_is_digit(raw[start]) && remaining_digits) {
            --remaining_digits;
        }
        out[out_len++] = raw[start++];
    }
    if (reading_uses_one_decimal_minimum(unit) && !output_has_dot && out_len < 8u) {
        out[out_len++] = '.';
        out[out_len++] = '0';
    }
    out[out_len] = 0;
    return 1;
}

static uint8_t parse_frame(const uint8_t frame[DMM_FRAME_LEN]) {
    char new_value[10];
    char new_unit[6];
    int32_t new_milli = 0;
    uint8_t new_numeric;
    uint8_t changed;

    if (frame[0] != 0x5Au || frame[1] != 0xA5u) {
        return 0;
    }
    dmm_baud_locked = 1;

    format_unit(frame[6], frame[7], frame[8], new_unit);
    if (!format_value(frame, new_unit, new_value)) {
        return 0;
    }
    new_numeric = parse_milli_units(new_value, &new_milli);
    if (new_numeric) {
        normalize_current_reading(new_value, new_unit, &new_milli);
    }
    if (dmm_current_mode == 3u && !unit_is_resistance(new_unit)) {
        text_copy(new_unit, "OHM", sizeof(new_unit));
    } else if (dmm_current_mode == 4u && !unit_is_voltage(new_unit) && !unit_is_resistance(new_unit)) {
        text_copy(new_unit, "V", sizeof(new_unit));
    }
    if (!dmm_reading_compatible(new_value, new_unit, new_numeric)) {
        return 0;
    }

    changed = (uint8_t)(!dmm_valid || !text_equal(new_value, dmm_value) || !text_equal(new_unit, dmm_unit));
    text_copy(dmm_value, new_value, sizeof(dmm_value));
    text_copy(dmm_unit, new_unit, sizeof(dmm_unit));
    dmm_numeric_valid = new_numeric;
    dmm_value_milli = new_milli;
    dmm_valid = 1;
    dmm_synthetic_valid = 0;
    return changed;
}

static void rx_byte(uint8_t byte) {
    ++rx_byte_count;
    if (rx_pos == 0u) {
        if (byte == 0x5Au) {
            rx_capture[rx_pos++] = byte;
            ++rx_sync_count;
        }
        return;
    }

    if (rx_pos == 1u) {
        if (byte == 0xA5u) {
            rx_capture[rx_pos++] = byte;
        } else {
            rx_pos = byte == 0x5Au ? 1u : 0u;
            rx_capture[0] = byte;
        }
        return;
    }

    rx_capture[rx_pos++] = byte;
    if (rx_pos >= DMM_FRAME_LEN) {
        for (uint8_t i = 0; i < DMM_FRAME_LEN; ++i) {
            rx_pending[i] = rx_capture[i];
        }
        rx_ready = 1;
        rx_pos = 0;
    }
}

static uint8_t uart_wait_txe(void) {
    for (uint32_t timeout = 0; timeout < 200000u; ++timeout) {
        if (USART_STS(USART3_BASE) & USART_STS_TXE) {
            return 1;
        }
    }
    return 0;
}

void dmm_init(void) {
    RCC_APB2ENR |= (1u << 0) | (1u << 3) | (1u << 4);
    RCC_APB1ENR |= 1u << 18; // USART3

    gpio_config_mask(GPIOB_BASE, 1u << 10, 0xBu); // USART3 TX, AF push-pull
    gpio_config_mask(GPIOB_BASE, 1u << 11, 0x4u); // USART3 RX, floating input
    gpio_config_mask(GPIOC_BASE, 1u << 6, 0x1u);  // DMM IC enable
    gpio_set(GPIOC_BASE, 1u << 6);

    USART_CTRL2(USART3_BASE) = 0;
    USART_CTRL3(USART3_BASE) = 0;
    dmm_detected_brr = dmm_compute_brr();
    dmm_uart_apply_brr(dmm_current_brr());
    REG32(NVIC_ISER1) = 1u << 7; // IRQ39, USART3 global

    dmm_started = 1;
    dmm_powered = 1;
    dmm_set_mode(0);
}

void dmm_pause(void) {
    if (!dmm_started || !dmm_powered) {
        return;
    }

    USART_CTRL1(USART3_BASE) = 0;
    gpio_clear(GPIOC_BASE, 1u << 6);
    dmm_powered = 0;
    dmm_valid = 0;
    dmm_synthetic_valid = 0;
    dmm_numeric_valid = 0;
    dmm_rx_reset_counters();
}

static void dmm_resume_power(void) {
    if (!dmm_started || dmm_powered) {
        return;
    }

    gpio_set(GPIOC_BASE, 1u << 6);
    dmm_uart_apply_brr(dmm_current_brr());
    dmm_powered = 1;
}

static void dmm_send_mode_command(uint8_t mode_index) {
    uint8_t frame[DMM_CMD_LEN] = {0xAA, 0x55, 0x05, 0, 0, 0, 0, 0, 0, 0};
    uint16_t checksum = 0;

    if (mode_index >= (uint8_t)sizeof(dmm_mode_command)) {
        return;
    }
    if (!dmm_started || !dmm_powered) {
        return;
    }

    frame[3] = dmm_mode_command[mode_index];
    for (uint8_t i = 2; i < 8u; ++i) {
        checksum = (uint16_t)(checksum + frame[i]);
    }
    frame[8] = (uint8_t)(checksum >> 8);
    frame[9] = (uint8_t)checksum;

    for (uint8_t i = 0; i < DMM_CMD_LEN; ++i) {
        if (!uart_wait_txe()) {
            return;
        }
        USART_DT(USART3_BASE) = frame[i];
    }
}

void dmm_set_mode(uint8_t mode_index) {
    uint8_t previous_mode;

    if (mode_index >= (uint8_t)sizeof(dmm_mode_command)) {
        return;
    }
    if (!dmm_started) {
        return;
    }

    dmm_resume_power();
    previous_mode = dmm_current_mode;
    dmm_current_mode = mode_index;
    dmm_valid = 0;
    dmm_synthetic_valid = 0;
    dmm_numeric_valid = 0;
    dmm_value_milli = 0;
    default_value_for_mode(mode_index, dmm_value);
    default_unit_for_mode(mode_index, dmm_unit);
    dmm_rx_reset_counters();
    dmm_apply_missing_uart_fallback();
    dmm_settle_ms = DMM_MODE_SWITCH_SETTLE_MS;
    if (mode_index == 12u && previous_mode != 11u) {
        // Original UI reaches DC low current through AC low first; keep that low-range latch.
        dmm_send_mode_command(11u);
        delay_ms(30);
    }
    dmm_send_mode_command(mode_index);
}

uint8_t dmm_poll(void) {
    uint8_t frame[DMM_FRAME_LEN];

    if (!dmm_powered) {
        rx_ready = 0;
        return 0;
    }
    if (dmm_settle_ms) {
        dmm_rx_clear(0);
        return 0;
    }
    if (!rx_ready) {
        return 0;
    }

    __asm__ volatile("cpsid i" ::: "memory");
    for (uint8_t i = 0; i < DMM_FRAME_LEN; ++i) {
        frame[i] = rx_pending[i];
    }
    rx_ready = 0;
    __asm__ volatile("cpsie i" ::: "memory");

    return parse_frame(frame);
}

void dmm_tick(uint32_t elapsed_ms) {
    if (!dmm_started || !dmm_powered) {
        return;
    }
    if (dmm_settle_ms) {
        dmm_rx_clear(0);
        if (elapsed_ms >= dmm_settle_ms) {
            dmm_settle_ms = 0;
        } else {
            dmm_settle_ms = (uint16_t)(dmm_settle_ms - elapsed_ms);
        }
        dmm_format_status();
        return;
    }
    if (dmm_baud_locked) {
        dmm_format_status();
        return;
    }

    dmm_retry_ms = (uint16_t)(dmm_retry_ms + elapsed_ms);
    if (dmm_retry_ms < DMM_BAUD_RETRY_MS) {
        dmm_format_status();
        return;
    }

    dmm_retry_ms = 0;
    dmm_baud_index = (uint8_t)((dmm_baud_index + 1u) % DMM_BAUD_CANDIDATE_COUNT);
    dmm_uart_apply_brr(dmm_current_brr());
    dmm_send_mode_command(dmm_current_mode);
    dmm_format_status();
}

uint8_t dmm_has_reading(void) {
    return dmm_valid;
}

uint8_t dmm_value_is_numeric(void) {
    return dmm_numeric_valid;
}

int32_t dmm_value_milli_units(void) {
    return dmm_value_milli;
}

const char *dmm_value_text(void) {
    return dmm_value;
}

const char *dmm_unit_text(void) {
    return dmm_unit;
}

const char *dmm_status_text(void) {
    return dmm_valid && !dmm_synthetic_valid ? "LIVE" : dmm_status;
}

uint8_t dmm_reading_is_real(void) {
    return dmm_valid && !dmm_synthetic_valid;
}

uint8_t dmm_live_wire_active(void) {
    if (dmm_current_mode != 6u || !dmm_reading_is_real()) {
        return 0;
    }
    return !text_equal(dmm_value, "L1UE");
}

void dmm_uart_irq_handler(void) {
    uint32_t status = USART_STS(USART3_BASE);

    if (status & USART_STS_RXNE) {
        rx_byte((uint8_t)USART_DT(USART3_BASE));
    } else if (status & (USART_STS_ORE | USART_STS_NE | USART_STS_FE)) {
        (void)USART_DT(USART3_BASE);
    }
}

void USART3_IRQHandler(void) {
    dmm_uart_irq_handler();
}
