#include "board.h"

#include "app_config.h"
#include "hw.h"

#include <stdint.h>

typedef struct {
    uint32_t base;
    uint16_t mask;
    uint32_t key;
    uint8_t active_high;
} key_pin_t;

static const key_pin_t key_pins[] = {
    {GPIOB_BASE, 1u << 6, KEY_MOVE, 0},
    {GPIOB_BASE, 1u << 7, KEY_F2, 0},
    {GPIOC_BASE, 1u << 14, KEY_F3, 0},
    {GPIOE_BASE, 1u << 1, KEY_F4, 1},
    {GPIOB_BASE, 1u << 9, KEY_AUTO, 0},
    {GPIOE_BASE, 1u << 4, KEY_MENU, 0},
    {GPIOC_BASE, 1u << 15, KEY_LEFT, 0},
    {GPIOE_BASE, 1u << 5, KEY_RIGHT, 0},
    {GPIOE_BASE, 1u << 6, KEY_UP, 0},
    {GPIOA_BASE, 1u << 3, KEY_DOWN, 0},
    {GPIOC_BASE, 1u << 13, KEY_OK, 0},
    {GPIOB_BASE, 1u << 8, KEY_CH1, 0},
    {GPIOE_BASE, 1u << 2, KEY_CH2, 0},
    {GPIOE_BASE, 1u << 3, KEY_SAVE, 0},
    {GPIOD_BASE, 1u << 3, KEY_POWER, 0},
};

static uint16_t g_battery_mv;
static uint8_t g_battery_percent;
static uint8_t g_battery_charging;
static uint8_t g_battery_filter_valid;
static uint8_t g_backlight_on;
static uint8_t g_backlight_percent = 100;
static volatile uint8_t g_buzzer_on;
static uint8_t g_buzzer_volume = 50;
static volatile uint8_t g_dmm_beep_irq_armed;
static volatile uint8_t g_dmm_beep_irq_full;
static volatile uint8_t g_dmm_beep_edge_seen;
static volatile uint8_t g_power_off_requested;

void delay_ms(uint32_t ms) {
    while (ms--) {
        for (volatile uint32_t i = 0; i < 12000u; ++i) {
            __asm__ volatile("nop");
        }
    }
}

static void gpio_config_nibble(uint32_t base, uint8_t pin, uint8_t cfg) {
    volatile uint32_t *reg = pin < 8 ? &GPIO_CRL(base) : &GPIO_CRH(base);
    uint8_t shift = (uint8_t)((pin & 7u) * 4u);
    uint32_t value = *reg;
    value &= ~(0xFu << shift);
    value |= ((uint32_t)cfg & 0xFu) << shift;
    *reg = value;
}

void gpio_config_mask(uint32_t base, uint16_t mask, uint8_t cfg) {
    for (uint8_t pin = 0; pin < 16; ++pin) {
        if (mask & (1u << pin)) {
            gpio_config_nibble(base, pin, cfg);
        }
    }
}

void board_init(void) {
    REG32(SCB_VTOR) = APP_BASE_ADDR;

    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6);
    RCC_AHBENR |= (1u << 8);

    gpio_config_mask(GPIOB_BASE, 1u << 2, 0x1); // PB2 power hold
    gpio_set(GPIOB_BASE, 1u << 2);

    gpio_config_mask(GPIOB_BASE, 1u << 0, 0x8); // PB0 charger status, active low
    gpio_set(GPIOB_BASE, 1u << 0);

    gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1); // PA0 LCD backlight
    gpio_clear(GPIOA_BASE, 1u << 0);

    gpio_config_mask(GPIOA_BASE, 1u << 1, 0x1); // PA1 buzzer, quiet until PWM init
    gpio_clear(GPIOA_BASE, 1u << 1);

    gpio_config_mask(GPIOD_BASE, 1u << 3, 0x8); // PD3 power key, input pull-up
    gpio_set(GPIOD_BASE, 1u << 3);

    gpio_config_mask(GPIOD_BASE, 1u << 6, 0x1); // PD6 LCD reset
    gpio_set(GPIOD_BASE, 1u << 6);
}

void load_counter_init(void) {
    SYSTICK_CTRL = 0;
    SYSTICK_LOAD = 0x00FFFFFFu;
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = 5u; // processor clock, no interrupt
}

uint32_t load_counter_read(void) {
    return SYSTICK_VAL & 0x00FFFFFFu;
}

uint32_t load_counter_elapsed(uint32_t start, uint32_t end) {
    return (start - end) & 0x00FFFFFFu;
}

void power_key_exti_init(void) {
    g_power_off_requested = 0;
    AFIO_EXTICR1 = (AFIO_EXTICR1 & ~(0xFu << 12)) | (0x3u << 12); // EXTI3 = port D
    EXTI_PR = 1u << 3;
    EXTI_RTSR &= ~(1u << 3);
    EXTI_FTSR |= 1u << 3;
    EXTI_IMR |= 1u << 3;
    REG32(NVIC_ISER0) = 1u << 9; // EXTI3 IRQ
    __asm__ volatile("cpsie i");
}

void power_key_irq_handler(void) {
    EXTI_PR = 1u << 3;
    EXTI_IMR &= ~(1u << 3);
    g_power_off_requested = 1;
}

uint8_t board_power_off_requested(void) {
    return g_power_off_requested;
}

void board_power_off(void) {
    gpio_clear(GPIOB_BASE, 1u << 2);
    while (1) {
    }
}

void board_reboot(void) {
    // AIRCR ignores a reset request that does not carry the VECTKEY.
    // Rebooting is the only honest way to apply settings that have been
    // rewritten underneath a running interface: half the live state lives
    // in static variables that ui_init() seeds once and never re-reads.
    __asm__ volatile("dsb" ::: "memory");
    REG32(SCB_AIRCR) = 0x05FA0004u;
    __asm__ volatile("dsb" ::: "memory");
    while (1) {
    }
}

static uint16_t percent_to_timer_compare(uint8_t percent, uint16_t max_compare) {
    if (percent > 100u) {
        percent = 100u;
    }
    return (uint16_t)((uint32_t)max_compare * percent / 100u);
}

static void tmr5_update_counter(void) {
    if (g_backlight_on || g_buzzer_on) {
        TMR_CTRL1(TMR5_BASE) |= 1u;
    } else {
        TMR_CTRL1(TMR5_BASE) &= ~1u;
    }
}

void board_backlight_set(uint8_t on) {
    g_backlight_on = on ? 1u : 0u;
    if (on) {
        gpio_config_mask(GPIOA_BASE, 1u << 0, 0xBu); // TMR5 CH1, AF push-pull
        TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
        TMR_CCEN(TMR5_BASE) |= 1u; // CH1 enable
    } else {
        TMR_CCEN(TMR5_BASE) &= ~1u;
        gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1);
        gpio_clear(GPIOA_BASE, 1u << 0);
    }
    tmr5_update_counter();
}

void board_backlight_set_level(uint8_t percent) {
    if (percent > 100u) {
        percent = 100u;
    }
    g_backlight_percent = percent;
    TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
}

void board_buzzer_init(void) {
    RCC_APB1ENR |= 1u << 3; // TMR5

    gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1); // PA0 backlight stays off until display is ready
    gpio_clear(GPIOA_BASE, 1u << 0);
    gpio_config_mask(GPIOA_BASE, 1u << 1, 0xBu); // TMR5 CH2, AF push-pull
#if !HW_TARGET_HW40
    gpio_config_mask(GPIOC_BASE, 1u << 7, 0x8u); // DMM beep request, active low
    gpio_set(GPIOC_BASE, 1u << 7);
    AFIO_EXTICR2 = (AFIO_EXTICR2 & ~(0xFu << 12)) | (0x2u << 12); // EXTI7 = port C
    EXTI_PR = 1u << 7;
    EXTI_FTSR |= 1u << 7;
    EXTI_RTSR &= ~(1u << 7);
    EXTI_IMR |= 1u << 7;
    REG32(NVIC_ISER0) = 1u << 23; // EXTI9_5 IRQ
#else
    EXTI_IMR &= ~(1u << 7);
    EXTI_PR = 1u << 7;
#endif

    TMR_CTRL1(TMR5_BASE) = 0;
    TMR_CCEN(TMR5_BASE) = 0;
    TMR_PSC(TMR5_BASE) = 71u;   // 1 MHz timer tick when APB1 timer clock is 72 MHz
    TMR_PR(TMR5_BASE) = 999u;   // about 1 kHz
    TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(g_buzzer_volume, 500u);
    TMR_CCM1(TMR5_BASE) = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TMR_EG(TMR5_BASE) = 1u;
    TMR_CTRL1(TMR5_BASE) = 1u << 7; // auto-reload preload, counter off until needed
    g_backlight_on = 0;
    g_buzzer_on = 0;
}

static void board_buzzer_set_at(uint8_t on, uint8_t percent) {
    on = on ? 1u : 0u;
    if (percent > 100u) {
        percent = 100u;
    }
    if (!percent) {
        on = 0;
    }
    if (on == g_buzzer_on && (!on || TMR_C2DT(TMR5_BASE) == percent_to_timer_compare(percent, 500u))) {
        return;
    }
    g_buzzer_on = on;
    if (on) {
        TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(percent, 500u);
        TMR_CCEN(TMR5_BASE) |= 1u << 4; // CH2 enable
    } else {
        TMR_CCEN(TMR5_BASE) &= ~(1u << 4);
        gpio_config_mask(GPIOA_BASE, 1u << 1, 0x1);
        gpio_clear(GPIOA_BASE, 1u << 1);
        gpio_config_mask(GPIOA_BASE, 1u << 1, 0xBu);
    }
    tmr5_update_counter();
}

void board_buzzer_set(uint8_t on) {
    board_buzzer_set_at(on, g_buzzer_volume);
}

void board_buzzer_set_full(uint8_t on) {
    board_buzzer_set_at(on, 100);
}

void board_buzzer_set_percent(uint8_t on, uint8_t percent) {
    board_buzzer_set_at(on, percent);
}

void board_buzzer_set_volume(uint8_t percent) {
    if (percent > 100u) {
        percent = 100u;
    }
    g_buzzer_volume = percent;
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(g_buzzer_volume, 500u);
    if (!g_buzzer_volume && g_buzzer_on) {
        board_buzzer_set(0);
    }
}

#if !HW_TARGET_HW40
static void board_buzzer_start_from_irq(void) {
    uint8_t percent = g_dmm_beep_irq_full ? 100u : g_buzzer_volume;
    if (!percent) {
        return;
    }
    if (g_buzzer_on) {
        return;
    }
    g_buzzer_on = 1;
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(percent, 500u);
    TMR_CCEN(TMR5_BASE) |= 1u << 4;
    tmr5_update_counter();
}
#endif

uint8_t board_dmm_beep_active(void) {
#if HW_TARGET_HW40
    return 0;
#else
    return gpio_read(GPIOC_BASE, 1u << 7) ? 0u : 1u;
#endif
}

uint8_t board_dmm_beep_edge_seen(void) {
    uint8_t seen;
    __asm__ volatile("cpsid i" ::: "memory");
    seen = g_dmm_beep_edge_seen;
    g_dmm_beep_edge_seen = 0;
    __asm__ volatile("cpsie i" ::: "memory");
    return seen;
}

void board_dmm_beep_irq_arm(uint8_t enabled) {
    g_dmm_beep_irq_armed = enabled ? 1u : 0u;
    if (!enabled) {
        g_dmm_beep_irq_full = 0;
        g_dmm_beep_edge_seen = 0;
    }
}

void board_dmm_beep_irq_force_full(uint8_t enabled) {
    g_dmm_beep_irq_full = enabled ? 1u : 0u;
}

void EXTI9_5_IRQHandler(void) {
#if !HW_TARGET_HW40
    if (EXTI_PR & (1u << 7)) {
        EXTI_PR = 1u << 7;
        if (!gpio_read(GPIOC_BASE, 1u << 7)) {
            g_dmm_beep_edge_seen = 1;
            if (g_dmm_beep_irq_armed) {
                board_buzzer_start_from_irq();
            }
        }
    }
#else
    if (EXTI_PR & (1u << 7)) {
        EXTI_PR = 1u << 7;
    }
#endif
}

static uint8_t battery_percent_from_mv(uint16_t mv) {
    if (mv >= 4100u) {
        return 100;
    }
    if (mv >= 4000u) {
        return (uint8_t)(95u + (uint32_t)(mv - 4000u) * 5u / 100u);
    }
    if (mv >= 3900u) {
        return (uint8_t)(90u + (uint32_t)(mv - 3900u) * 5u / 100u);
    }
    if (mv >= 3800u) {
        return (uint8_t)(85u + (uint32_t)(mv - 3800u) * 5u / 100u);
    }
    if (mv >= 3700u) {
        return (uint8_t)(75u + (uint32_t)(mv - 3700u) * 10u / 100u);
    }
    if (mv >= 3600u) {
        return (uint8_t)(50u + (uint32_t)(mv - 3600u) * 25u / 100u);
    }
    if (mv >= 3500u) {
        return (uint8_t)(35u + (uint32_t)(mv - 3500u) * 15u / 100u);
    }
    if (mv >= 3400u) {
        return (uint8_t)(15u + (uint32_t)(mv - 3400u) * 20u / 100u);
    }
    if (mv >= 3300u) {
        return (uint8_t)(5u + (uint32_t)(mv - 3300u) * 10u / 100u);
    }
    if (mv >= 3200u) {
        return (uint8_t)(1u + (uint32_t)(mv - 3200u) * 4u / 100u);
    }
    return 0;
}

static uint16_t battery_adc_read_raw(void) {
    ADC_SR(ADC1_BASE) = 0;
    ADC_CR2(ADC1_BASE) |= 1u << 22; // SWSTART
    for (uint32_t timeout = 0; timeout < 100000u; ++timeout) {
        if (ADC_SR(ADC1_BASE) & (1u << 1)) {
            return (uint16_t)(ADC_DR(ADC1_BASE) & 0x0FFFu);
        }
    }
    return 0;
}

void battery_init(void) {
    gpio_config_mask(GPIOA_BASE, 1u << 2, 0x0); // PA2 ADC input

    RCC_APB2ENR |= 1u << 9; // ADC1 clock
    RCC_CFGR = (RCC_CFGR & ~(3u << 14)) | (2u << 14); // ADC clock = PCLK2 / 6

    ADC_CR1(ADC1_BASE) = 0;
    ADC_CR2(ADC1_BASE) = 0;
    ADC_SMPR2(ADC1_BASE) = (ADC_SMPR2(ADC1_BASE) & ~(7u << 6)) | (7u << 6);
    ADC_SQR1(ADC1_BASE) = 0;
    ADC_SQR3(ADC1_BASE) = 2u;

    ADC_CR2(ADC1_BASE) |= 1u; // ADON
    delay_ms(2);
    ADC_CR2(ADC1_BASE) |= 1u << 3; // reset calibration
    for (uint32_t timeout = 0; timeout < 100000u && (ADC_CR2(ADC1_BASE) & (1u << 3)); ++timeout) {
    }
    ADC_CR2(ADC1_BASE) |= 1u << 2; // calibration
    for (uint32_t timeout = 0; timeout < 100000u && (ADC_CR2(ADC1_BASE) & (1u << 2)); ++timeout) {
    }
    ADC_CR2(ADC1_BASE) |= (7u << 17) | (1u << 20) | 1u; // software trigger
    battery_update();
}

void battery_update(void) {
    uint32_t sum = 0;
    uint16_t min_raw = 0x0FFFu;
    uint16_t max_raw = 0;
    uint16_t raw;
    uint16_t raw_mv;

    for (uint8_t i = 0; i < 18; ++i) {
        raw = battery_adc_read_raw();
        sum += raw;
        if (raw < min_raw) {
            min_raw = raw;
        }
        if (raw > max_raw) {
            max_raw = raw;
        }
    }
    sum -= min_raw;
    sum -= max_raw;
    raw_mv = (uint16_t)((sum * 6600u + (4095u * 8u)) / (4095u * 16u));

    if (!g_battery_filter_valid) {
        g_battery_mv = raw_mv;
        g_battery_filter_valid = 1;
    } else {
        g_battery_mv = (uint16_t)(((uint32_t)g_battery_mv * 7u + raw_mv + 4u) / 8u);
    }
    g_battery_percent = battery_percent_from_mv(g_battery_mv);
    battery_update_charging_status();
}

void battery_update_charging_status(void) {
    g_battery_charging = gpio_read(GPIOB_BASE, 1u << 0) ? 0u : 1u;
}

uint16_t battery_millivolts(void) {
    return g_battery_mv;
}

uint8_t battery_percent(void) {
    return g_battery_percent;
}

uint8_t battery_is_charging(void) {
    return g_battery_charging;
}

void input_init(void) {
    for (uint32_t i = 0; i < sizeof(key_pins) / sizeof(key_pins[0]); ++i) {
        gpio_config_mask(key_pins[i].base, key_pins[i].mask, 0x8);
        if (key_pins[i].active_high) {
            gpio_clear(key_pins[i].base, key_pins[i].mask);
        } else {
            gpio_set(key_pins[i].base, key_pins[i].mask);
        }
    }
}

uint32_t input_read_keys(void) {
    uint32_t keys = 0;
    for (uint32_t i = 0; i < sizeof(key_pins) / sizeof(key_pins[0]); ++i) {
        uint8_t high = gpio_read(key_pins[i].base, key_pins[i].mask) ? 1u : 0u;
        uint8_t pressed = key_pins[i].active_high ? high : (uint8_t)!high;
        if (pressed) {
            keys |= key_pins[i].key;
        }
    }
    return keys;
}

static uint32_t input_debounced_keys(void) {
    enum {
        DEBOUNCE_STABLE_POLLS = 2,
    };
    static uint32_t stable_keys;
    static uint32_t candidate_keys;
    static uint8_t stable_count;
    uint32_t raw = input_read_keys();

    if (raw == candidate_keys) {
        if (stable_count < DEBOUNCE_STABLE_POLLS) {
            ++stable_count;
        }
    } else {
        candidate_keys = raw;
        stable_count = 0;
    }

    if (stable_count >= DEBOUNCE_STABLE_POLLS) {
        stable_keys = candidate_keys;
    }
    return stable_keys;
}

static uint32_t input_repeat_event(uint32_t now,
                                   uint32_t last,
                                   uint32_t key,
                                   uint16_t *hold_ms,
                                   uint16_t *repeat_ms) {
    enum {
        INPUT_POLL_MS = 20,
        REPEAT_START_MS = 360,
        REPEAT_FAST_MS = 900,
        REPEAT_SLOW_INTERVAL_MS = 120,
        REPEAT_FAST_INTERVAL_MS = 40,
    };
    uint16_t interval;

    if (!(now & key)) {
        *hold_ms = 0;
        *repeat_ms = 0;
        return 0;
    }
    if (!(last & key)) {
        *hold_ms = 0;
        *repeat_ms = 0;
        return 0;
    }

    if (*hold_ms < 2000u) {
        *hold_ms = (uint16_t)(*hold_ms + INPUT_POLL_MS);
    }
    if (*hold_ms < REPEAT_START_MS) {
        return 0;
    }

    interval = *hold_ms >= REPEAT_FAST_MS ? REPEAT_FAST_INTERVAL_MS : REPEAT_SLOW_INTERVAL_MS;
    *repeat_ms = (uint16_t)(*repeat_ms + INPUT_POLL_MS);
    if (*repeat_ms >= interval) {
        *repeat_ms = 0;
        return key | KEY_REPEAT;
    }
    return 0;
}

static uint32_t input_short_long_event(uint32_t now,
                                       uint32_t last,
                                       uint32_t key,
                                       uint32_t long_key,
                                       uint16_t *hold_ms,
                                       uint8_t *long_sent) {
    enum {
        INPUT_POLL_MS = 20,
        LONG_PRESS_MS = 700,
    };

    if (now & key) {
        if (!(last & key)) {
            *hold_ms = 0;
            *long_sent = 0;
            return 0;
        }
        if (!*long_sent) {
            if (*hold_ms < LONG_PRESS_MS) {
                *hold_ms = (uint16_t)(*hold_ms + INPUT_POLL_MS);
            }
            if (*hold_ms >= LONG_PRESS_MS) {
                *long_sent = 1;
                return long_key;
            }
        }
        return 0;
    }

    if (last & key) {
        uint32_t event = *long_sent ? 0u : key;
        *hold_ms = 0;
        *long_sent = 0;
        return event;
    }

    *hold_ms = 0;
    *long_sent = 0;
    return 0;
}

uint32_t input_pressed_events(void) {
    enum {
        SHORT_LONG_KEYS = KEY_MOVE | KEY_F2 | KEY_F3 | KEY_F4 | KEY_AUTO | KEY_SAVE | KEY_CH1,
        STARTUP_GUARD_POLLS = 10,
    };
    static uint32_t last_keys;
    static uint32_t startup_block_keys;
    static uint8_t startup_guard_polls = STARTUP_GUARD_POLLS;
    static uint8_t initialized;
    static uint16_t move_hold_ms;
    static uint16_t f2_hold_ms;
    static uint16_t f3_hold_ms;
    static uint16_t f4_hold_ms;
    static uint16_t auto_hold_ms;
    static uint16_t save_hold_ms;
    static uint16_t left_hold_ms;
    static uint16_t right_hold_ms;
    static uint16_t up_hold_ms;
    static uint16_t down_hold_ms;
    static uint16_t left_repeat_ms;
    static uint16_t right_repeat_ms;
    static uint16_t up_repeat_ms;
    static uint16_t down_repeat_ms;
    static uint8_t move_long_sent;
    static uint8_t f2_long_sent;
    static uint8_t f3_long_sent;
    static uint8_t f4_long_sent;
    static uint8_t auto_long_sent;
    static uint8_t save_long_sent;
    static uint16_t ch1_hold_ms;
    static uint8_t ch1_long_sent;
    uint32_t now = input_debounced_keys();
    uint32_t events;

    if (!initialized) {
        initialized = 1;
        startup_block_keys = 0;
        last_keys = now;
        return 0;
    }

    if (startup_guard_polls) {
        --startup_guard_polls;
        startup_block_keys |= now & SHORT_LONG_KEYS;
        last_keys = now;
        return 0;
    }

    startup_block_keys &= now;
    now &= ~startup_block_keys;
    last_keys &= ~startup_block_keys;
    events = now & ~last_keys;

    // Keys with long actions emit their short action on release, so a long press
    // cannot also run the short action first.
    events &= ~KEY_MOVE;
    events &= ~KEY_F2;
    events &= ~KEY_F3;
    events &= ~KEY_F4;
    events &= ~KEY_AUTO;
    events &= ~KEY_SAVE;
    events &= ~KEY_CH1;

    events |= input_short_long_event(now, last_keys, KEY_MOVE, KEY_MOVE_LONG, &move_hold_ms, &move_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F2, KEY_F2_LONG, &f2_hold_ms, &f2_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F3, KEY_F3_LONG, &f3_hold_ms, &f3_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F4, KEY_F4_LONG, &f4_hold_ms, &f4_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_AUTO, KEY_AUTO_LONG, &auto_hold_ms, &auto_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_SAVE, KEY_SAVE_LONG, &save_hold_ms, &save_long_sent);

    events |= input_repeat_event(now, last_keys, KEY_LEFT, &left_hold_ms, &left_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_RIGHT, &right_hold_ms, &right_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_UP, &up_hold_ms, &up_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_DOWN, &down_hold_ms, &down_repeat_ms);

    events |= input_short_long_event(now, last_keys, KEY_CH1, KEY_CH1_LONG, &ch1_hold_ms, &ch1_long_sent);

    last_keys = now;
    return events;
}
