#include "board.h"
#include "dmm.h"
#include "display.h"
#include "fw_update.h"
#include "settings.h"
#include "ui.h"
#include "usb_msc.h"

#include <stdint.h>

static uint16_t diode_beep_hold_ms;
static uint16_t live_beep_hold_ms;
static uint16_t live_beep_phase_ms;
static uint16_t ui_beep_ms;

enum {
    DIODE_BEEP_HOLD_MS = 30,
    KEY_BEEP_MS = 28,
    SETTINGS_BEEP_PREVIEW_MS = 90,
    LIVE_BEEP_HOLD_MS = 450,
    LIVE_BEEP_PERIOD_MS = 180,
    LIVE_BEEP_ON_MS = 45,
    LIVE_BEEP_VOLUME_PERCENT = 70,
    STARTUP_BEEP_MS = 55,
    STARTUP_INPUT_SETTLE_MS = 200,
};

static void shutdown_now(void) {
    board_dmm_beep_irq_arm(0);
    board_buzzer_set(0);
    settings_flush();
    board_power_off();
}

static void dmm_beep_service(uint8_t elapsed_ms) {
    uint8_t dmm_beep_seen;
    uint8_t buzzer_on = 0;
    uint8_t force_full = 0;
    uint8_t fixed_volume = 0;
    uint8_t diode_mode = ui_diode_beep_enabled();
    uint8_t live_mode = ui_live_beep_enabled();

    if (ui_beep_ms && !diode_mode && !live_mode) {
        board_dmm_beep_irq_arm(0);
        if (elapsed_ms >= ui_beep_ms) {
            ui_beep_ms = 0;
            board_buzzer_set(0);
        } else {
            ui_beep_ms = (uint16_t)(ui_beep_ms - elapsed_ms);
            board_buzzer_set(1);
        }
        return;
    }

    if (diode_mode) {
        ui_beep_ms = 0;
        board_dmm_beep_irq_force_full(1);
        board_dmm_beep_irq_arm(1);
        dmm_beep_seen = (uint8_t)(board_dmm_beep_active() || board_dmm_beep_edge_seen());
        if (dmm_beep_seen) {
            diode_beep_hold_ms = DIODE_BEEP_HOLD_MS;
        } else if (diode_beep_hold_ms > elapsed_ms) {
            diode_beep_hold_ms = (uint16_t)(diode_beep_hold_ms - elapsed_ms);
        } else {
            diode_beep_hold_ms = 0;
        }
        live_beep_hold_ms = 0;
        live_beep_phase_ms = 0;
        buzzer_on = diode_beep_hold_ms ? 1u : 0u;
        force_full = 1;
    } else if (live_mode) {
        ui_beep_ms = 0;
        board_dmm_beep_irq_force_full(0);
        board_dmm_beep_irq_arm(0);
        (void)board_dmm_beep_edge_seen();
        dmm_beep_seen = dmm_live_wire_active();
        if (dmm_beep_seen) {
            if (!live_beep_hold_ms) {
                live_beep_phase_ms = 0;
            }
            live_beep_hold_ms = LIVE_BEEP_HOLD_MS;
        } else if (live_beep_hold_ms > elapsed_ms) {
            live_beep_hold_ms = (uint16_t)(live_beep_hold_ms - elapsed_ms);
        } else {
            live_beep_hold_ms = 0;
        }

        if (live_beep_hold_ms) {
            live_beep_phase_ms = (uint16_t)(live_beep_phase_ms + elapsed_ms);
            if (live_beep_phase_ms >= LIVE_BEEP_PERIOD_MS) {
                live_beep_phase_ms = (uint16_t)(live_beep_phase_ms % LIVE_BEEP_PERIOD_MS);
            }
        } else {
            live_beep_phase_ms = 0;
        }
        diode_beep_hold_ms = 0;
        buzzer_on = (live_beep_hold_ms && live_beep_phase_ms < LIVE_BEEP_ON_MS) ? 1u : 0u;
        fixed_volume = LIVE_BEEP_VOLUME_PERCENT;
    } else {
        board_dmm_beep_irq_arm(0);
        diode_beep_hold_ms = 0;
        live_beep_hold_ms = 0;
        live_beep_phase_ms = 0;
        (void)board_dmm_beep_edge_seen();
    }

    ui_set_live_wire_detected((uint8_t)(live_mode && live_beep_hold_ms));

    if (force_full) {
        board_buzzer_set_full(buzzer_on);
    } else if (fixed_volume) {
        board_buzzer_set_percent(buzzer_on, fixed_volume);
    } else if (ui_beep_ms) {
        if (elapsed_ms >= ui_beep_ms) {
            ui_beep_ms = 0;
            board_buzzer_set(0);
        } else {
            ui_beep_ms = (uint16_t)(ui_beep_ms - elapsed_ms);
            board_buzzer_set(1);
        }
    } else {
        board_buzzer_set(buzzer_on);
    }
}

int main(void) {
    uint16_t input_settle_ms = STARTUP_INPUT_SETTLE_MS;

    board_init();
    battery_init();
    input_init();
    dmm_init();
    board_buzzer_init();

    lcd_init();
    usb_msc_init();
    ui_init();
    lcd_display_on();
    delay_ms(50);
    board_backlight_set(1);
    load_counter_init();
    ui_beep_ms = STARTUP_BEEP_MS;

    while (1) {
        uint32_t active_start = load_counter_read();
        if (dmm_poll()) {
            dmm_beep_service(0);
            ui_dmm_measurement_updated();
        }
        uint32_t events = input_pressed_events();
        if (events && !input_settle_ms) {
            if (events & KEY_POWER) {
                shutdown_now();
            }
            ui_handle_keys(events);
            if (events & KEY_REPEAT) {
                ui_beep_ms = 0;
            } else if (ui_consume_beep_preview()) {
                ui_beep_ms = SETTINGS_BEEP_PREVIEW_MS;
            } else {
                ui_beep_ms = KEY_BEEP_MS;
            }
            dmm_beep_service(0);
        } else {
            ui_tick(20);
        }
        if (ui_auto_sleep_due()) {
            shutdown_now();
        }
        fw_update_service();
        uint32_t active_ticks = load_counter_elapsed(active_start, load_counter_read());

        uint32_t idle_start = load_counter_read();
        for (uint8_t i = 0; i < 20u; ++i) {
            usb_msc_poll();
            delay_ms(1);
            dmm_beep_service(1);
        }
        dmm_tick(20);
        if (input_settle_ms > 20u) {
            input_settle_ms = (uint16_t)(input_settle_ms - 20u);
        } else {
            input_settle_ms = 0;
        }
        uint32_t idle_ticks = load_counter_elapsed(idle_start, load_counter_read());
        ui_set_load_sample(active_ticks, idle_ticks);
    }
}
