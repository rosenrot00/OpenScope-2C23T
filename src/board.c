#include "board.h"

#include "app_config.h"
#include "hw.h"

#include <stdint.h>

typedef struct {
    uint32_t base;
    uint16_t mask;
    uint32_t key;
} key_pin_t;

static const key_pin_t key_pins[] = {
    {GPIOB_BASE, 1u << 6, KEY_MOVE},
    {GPIOB_BASE, 1u << 7, KEY_F2},
    {GPIOC_BASE, 1u << 14, KEY_F3},
    {GPIOE_BASE, 1u << 1, KEY_F4},
    {GPIOB_BASE, 1u << 9, KEY_AUTO},
    {GPIOE_BASE, 1u << 4, KEY_MENU},
    {GPIOC_BASE, 1u << 15, KEY_LEFT},
    {GPIOE_BASE, 1u << 5, KEY_RIGHT},
    {GPIOE_BASE, 1u << 6, KEY_UP},
    {GPIOA_BASE, 1u << 3, KEY_DOWN},
    {GPIOC_BASE, 1u << 13, KEY_OK},
    {GPIOB_BASE, 1u << 8, KEY_CH1},
    {GPIOE_BASE, 1u << 2, KEY_CH2},
    {GPIOE_BASE, 1u << 3, KEY_SAVE},
    {GPIOD_BASE, 1u << 3, KEY_POWER},
};

static uint16_t g_battery_mv;
static uint8_t g_battery_percent;
static uint8_t g_battery_charging;
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
    gpio_config_mask(GPIOC_BASE, 1u << 7, 0x8u); // DMM beep request, active low
    gpio_set(GPIOC_BASE, 1u << 7);
    AFIO_EXTICR2 = (AFIO_EXTICR2 & ~(0xFu << 12)) | (0x2u << 12); // EXTI7 = port C
    EXTI_PR = 1u << 7;
    EXTI_FTSR |= 1u << 7;
    EXTI_RTSR &= ~(1u << 7);
    EXTI_IMR |= 1u << 7;
    REG32(NVIC_ISER0) = 1u << 23; // EXTI9_5 IRQ

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

static void board_buzzer_start_from_irq(void) {
    uint8_t percent = g_dmm_beep_irq_full ? 100u : g_buzzer_volume;
    if (!percent) {
        return;
    }
    g_buzzer_on = 1;
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(percent, 500u);
    TMR_CCEN(TMR5_BASE) |= 1u << 4;
    tmr5_update_counter();
}

uint8_t board_dmm_beep_active(void) {
    return gpio_read(GPIOC_BASE, 1u << 7) ? 0u : 1u;
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
    if (EXTI_PR & (1u << 7)) {
        EXTI_PR = 1u << 7;
        if (!gpio_read(GPIOC_BASE, 1u << 7)) {
            g_dmm_beep_edge_seen = 1;
            if (g_dmm_beep_irq_armed) {
                board_buzzer_start_from_irq();
            }
        }
    }
}

static uint8_t battery_percent_from_mv(uint16_t mv) {
    if (mv > 4100u) {
        return 100;
    }
    if (mv > 4000u) {
        return (uint8_t)(95u + (uint32_t)(mv - 4000u) * 50u / 100u);
    }
    if (mv > 3900u) {
        return (uint8_t)(90u + (uint32_t)(mv - 3900u) * 50u / 100u);
    }
    if (mv > 3800u) {
        return (uint8_t)(85u + (uint32_t)(mv - 3800u) * 100u / 100u);
    }
    if (mv > 3700u) {
        return (uint8_t)(75u + (uint32_t)(mv - 3700u) * 100u / 100u);
    }
    if (mv > 3600u) {
        return (uint8_t)(50u + (uint32_t)(mv - 3600u) * 250u / 100u);
    }
    if (mv > 3500u) {
        return (uint8_t)(35u + (uint32_t)(mv - 3500u) * 150u / 100u);
    }
    if (mv > 3400u) {
        return (uint8_t)(15u + (uint32_t)(mv - 3400u) * 200u / 100u);
    }
    if (mv > 3300u) {
        return (uint8_t)(5u + (uint32_t)(mv - 3300u) * 100u / 100u);
    }
    if (mv > 3200u) {
        return (uint8_t)(1u + (uint32_t)(mv - 3200u) * 40u / 100u);
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
    for (uint8_t i = 0; i < 10; ++i) {
        sum += battery_adc_read_raw();
    }
    g_battery_mv = (uint16_t)((sum * 6600u + (4095u * 5u)) / (4095u * 10u));
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
        gpio_set(key_pins[i].base, key_pins[i].mask);
    }
}

uint32_t input_read_keys(void) {
    uint32_t keys = 0;
    for (uint32_t i = 0; i < sizeof(key_pins) / sizeof(key_pins[0]); ++i) {
        if (!gpio_read(key_pins[i].base, key_pins[i].mask)) {
            keys |= key_pins[i].key;
        }
    }
    return keys;
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

uint32_t input_pressed_events(void) {
    static uint32_t last_keys;
    static uint16_t move_hold_ms;
    static uint16_t f2_hold_ms;
    static uint16_t f3_hold_ms;
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
    static uint8_t auto_long_sent;
    static uint8_t save_long_sent;
    enum {
        INPUT_POLL_MS = 20,
        LONG_PRESS_MS = 700,
    };
    uint32_t now = input_read_keys();
    uint32_t events = now & ~last_keys;

    events &= ~KEY_F3; // F3/SAVE have distinct short/long actions; emit short on release.
    events &= ~KEY_SAVE;

    if (now & KEY_MOVE) {
        if (!(last_keys & KEY_MOVE)) {
            move_hold_ms = 0;
            move_long_sent = 0;
        } else if (!move_long_sent) {
            move_hold_ms = move_hold_ms < LONG_PRESS_MS ? (uint16_t)(move_hold_ms + INPUT_POLL_MS) : move_hold_ms;
            if (move_hold_ms >= LONG_PRESS_MS) {
                events |= KEY_MOVE_LONG;
                move_long_sent = 1;
            }
        }
    } else {
        move_hold_ms = 0;
        move_long_sent = 0;
    }

    if (now & KEY_F2) {
        if (!(last_keys & KEY_F2)) {
            f2_hold_ms = 0;
            f2_long_sent = 0;
        } else if (!f2_long_sent) {
            f2_hold_ms = f2_hold_ms < LONG_PRESS_MS ? (uint16_t)(f2_hold_ms + INPUT_POLL_MS) : f2_hold_ms;
            if (f2_hold_ms >= LONG_PRESS_MS) {
                events |= KEY_F2_LONG;
                f2_long_sent = 1;
            }
        }
    } else {
        f2_hold_ms = 0;
        f2_long_sent = 0;
    }

    if (now & KEY_F3) {
        if (!(last_keys & KEY_F3)) {
            f3_hold_ms = 0;
            f3_long_sent = 0;
        } else if (!f3_long_sent) {
            f3_hold_ms = f3_hold_ms < LONG_PRESS_MS ? (uint16_t)(f3_hold_ms + INPUT_POLL_MS) : f3_hold_ms;
            if (f3_hold_ms >= LONG_PRESS_MS) {
                events |= KEY_F3_LONG;
                f3_long_sent = 1;
            }
        }
    } else {
        if ((last_keys & KEY_F3) && !f3_long_sent) {
            events |= KEY_F3;
        }
        f3_hold_ms = 0;
        f3_long_sent = 0;
    }

    if (now & KEY_AUTO) {
        if (!(last_keys & KEY_AUTO)) {
            auto_hold_ms = 0;
            auto_long_sent = 0;
        } else if (!auto_long_sent) {
            auto_hold_ms = auto_hold_ms < LONG_PRESS_MS ? (uint16_t)(auto_hold_ms + INPUT_POLL_MS) : auto_hold_ms;
            if (auto_hold_ms >= LONG_PRESS_MS) {
                events |= KEY_AUTO_LONG;
                auto_long_sent = 1;
            }
        }
    } else {
        auto_hold_ms = 0;
        auto_long_sent = 0;
    }

    if (now & KEY_SAVE) {
        if (!(last_keys & KEY_SAVE)) {
            save_hold_ms = 0;
            save_long_sent = 0;
        } else if (!save_long_sent) {
            save_hold_ms = save_hold_ms < LONG_PRESS_MS ? (uint16_t)(save_hold_ms + INPUT_POLL_MS) : save_hold_ms;
            if (save_hold_ms >= LONG_PRESS_MS) {
                events |= KEY_SAVE_LONG;
                save_long_sent = 1;
            }
        }
    } else {
        if ((last_keys & KEY_SAVE) && !save_long_sent) {
            events |= KEY_SAVE;
        }
        save_hold_ms = 0;
        save_long_sent = 0;
    }

    events |= input_repeat_event(now, last_keys, KEY_LEFT, &left_hold_ms, &left_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_RIGHT, &right_hold_ms, &right_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_UP, &up_hold_ms, &up_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_DOWN, &down_hold_ms, &down_repeat_ms);

    last_keys = now;
    return events;
}
