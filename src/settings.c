#include "settings.h"

#include "hw.h"

#include <stdint.h>

enum {
    SETTINGS_FLASH_BASE = 0x080FF800u,
    SETTINGS_FLASH_SIZE = 2048u,
    SETTINGS_RECORD_MAGIC = 0xC6000000u,
    SETTINGS_RECORD_MAGIC_MASK = 0xFF000000u,
    SETTINGS_SCOPE_CAL_LEGACY_MAGIC = 0xC8000000u,
    SETTINGS_SCOPE_CAL_MAGIC = 0xC7000000u,
    SETTINGS_SCOPE_CAL_MAGIC_MASK = 0xFF000000u,
    SETTINGS_LEGACY_MAGIC = 0xE1A60000u,
    SETTINGS_LEGACY_MAGIC_MASK = 0xFFFF0000u,
    SETTINGS_SCOPE_BIAS_DEFAULT = 1861u,
    SETTINGS_SCOPE_BIAS_RATE_DEFAULT = 505u,
    SETTINGS_SCOPE_CAL_WORDS = SETTINGS_SCOPE_CHANNEL_COUNT * SETTINGS_SCOPE_RANGE_COUNT * 2,
    SETTINGS_WRITE_BYTES = 4u * (1u + SETTINGS_SCOPE_CAL_WORDS),
    FLASH_BANK2_BASE = 0x08080000u,
    FLASH_STS_BSY = 1u << 0,
    FLASH_STS_PGERR = 1u << 2,
    FLASH_STS_WRPRTERR = 1u << 4,
    FLASH_STS_EOP = 1u << 5,
    FLASH_CTRL_PG = 1u << 0,
    FLASH_CTRL_PER = 1u << 1,
    FLASH_CTRL_STRT = 1u << 6,
    FLASH_CTRL_LOCK = 1u << 7,
};

static uint8_t settings_loaded;
static settings_state_t settings_cached;
static settings_state_t settings_pending;
static uint8_t settings_dirty;

static void settings_scope_cal_defaults(settings_state_t *settings) {
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            settings->scope_bias[ch][range] = SETTINGS_SCOPE_BIAS_DEFAULT;
            settings->scope_bias_rate[ch][range] = SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
        }
    }
}

static void settings_defaults(settings_state_t *settings) {
    settings->dmm_mode = 0;
    settings->beep_level = 3;
    settings->brightness_level = 4;
    settings->startup_screen = SETTINGS_START_DMM;
    settings->last_screen = 0;
    settings->sleep_enabled = 0;
    settings_scope_cal_defaults(settings);
}

static void settings_copy(settings_state_t *dst, const settings_state_t *src) {
    dst->dmm_mode = src->dmm_mode;
    dst->beep_level = src->beep_level;
    dst->brightness_level = src->brightness_level;
    dst->startup_screen = src->startup_screen;
    dst->last_screen = src->last_screen;
    dst->sleep_enabled = src->sleep_enabled;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            dst->scope_bias[ch][range] = src->scope_bias[ch][range];
            dst->scope_bias_rate[ch][range] = src->scope_bias_rate[ch][range];
        }
    }
}

static uint8_t settings_equal(const settings_state_t *a, const settings_state_t *b) {
    if (a->dmm_mode != b->dmm_mode ||
        a->beep_level != b->beep_level ||
        a->brightness_level != b->brightness_level ||
        a->startup_screen != b->startup_screen ||
        a->last_screen != b->last_screen ||
        a->sleep_enabled != b->sleep_enabled) {
        return 0;
    }
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            if (a->scope_bias[ch][range] != b->scope_bias[ch][range] ||
                a->scope_bias_rate[ch][range] != b->scope_bias_rate[ch][range]) {
                return 0;
            }
        }
    }
    return 1;
}

static void settings_clamp(settings_state_t *settings) {
    if (settings->beep_level >= SETTINGS_LEVEL_COUNT) {
        settings->beep_level = 3;
    }
    if (settings->brightness_level >= SETTINGS_LEVEL_COUNT) {
        settings->brightness_level = 4;
    }
    if (settings->startup_screen >= SETTINGS_START_COUNT) {
        settings->startup_screen = SETTINGS_START_DMM;
    }
    if (settings->last_screen > 2u) {
        settings->last_screen = 0;
    }
    settings->sleep_enabled = settings->sleep_enabled ? 1u : 0u;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            if (settings->scope_bias[ch][range] > 4095u) {
                settings->scope_bias[ch][range] = SETTINGS_SCOPE_BIAS_DEFAULT;
            }
            if (!settings->scope_bias_rate[ch][range] || settings->scope_bias_rate[ch][range] > 4095u) {
                settings->scope_bias_rate[ch][range] = SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
            }
        }
    }
}

static uint8_t settings_checksum(uint16_t payload) {
    return (uint8_t)((payload & 0xFFu) ^ (payload >> 8) ^ 0xA5u);
}

static uint16_t settings_payload(const settings_state_t *settings) {
    return (uint16_t)((settings->dmm_mode & 0x0Fu) |
                      ((settings->beep_level & 0x07u) << 4) |
                      ((settings->brightness_level & 0x07u) << 7) |
                      ((settings->startup_screen & 0x03u) << 10) |
                      ((settings->last_screen & 0x03u) << 12) |
                      ((settings->sleep_enabled & 0x01u) << 14));
}

static uint32_t settings_record_for(const settings_state_t *settings) {
    uint16_t payload = settings_payload(settings);
    return SETTINGS_RECORD_MAGIC | ((uint32_t)payload << 8) | settings_checksum(payload);
}

static uint8_t settings_scope_cal_legacy_checksum(uint16_t payload) {
    return (uint8_t)((payload & 0xFFu) ^ (payload >> 8) ^ 0x5Cu);
}

static uint8_t settings_scope_cal_checksum(uint32_t payload) {
    return (uint8_t)((payload ^ (payload >> 6) ^ (payload >> 12) ^ 0x15u) & 0x3Fu);
}

static uint32_t settings_scope_cal_record(uint8_t type, uint8_t ch, uint8_t range, uint16_t value) {
    uint32_t payload = ((uint32_t)(type & 1u) << 17) |
                       ((uint32_t)(ch & 1u) << 16) |
                       ((uint32_t)(range & 0x0Fu) << 12) |
                       (value & 0x0FFFu);
    return SETTINGS_SCOPE_CAL_MAGIC | (payload << 6) | settings_scope_cal_checksum(payload);
}

static uint8_t settings_scope_cal_record_valid(uint32_t record, settings_state_t *settings) {
    uint8_t type;
    uint8_t ch;
    uint8_t range;
    uint16_t value;

    if ((record & SETTINGS_SCOPE_CAL_MAGIC_MASK) == SETTINGS_SCOPE_CAL_LEGACY_MAGIC) {
        uint16_t payload = (uint16_t)(record >> 8);
        uint8_t checksum = (uint8_t)record;
        if (checksum != settings_scope_cal_legacy_checksum(payload)) {
            return 0;
        }
        type = (uint8_t)((payload >> 15) & 1u);
        ch = (uint8_t)((payload >> 14) & 1u);
        range = (uint8_t)((payload >> 12) & 3u);
        value = (uint16_t)(payload & 0x0FFFu);
    } else if ((record & SETTINGS_SCOPE_CAL_MAGIC_MASK) == SETTINGS_SCOPE_CAL_MAGIC) {
        uint32_t payload = (record >> 6) & 0x3FFFFu;
        uint8_t checksum = (uint8_t)(record & 0x3Fu);
        if (checksum != settings_scope_cal_checksum(payload)) {
            return 0;
        }
        type = (uint8_t)((payload >> 17) & 1u);
        ch = (uint8_t)((payload >> 16) & 1u);
        range = (uint8_t)((payload >> 12) & 0x0Fu);
        value = (uint16_t)(payload & 0x0FFFu);
    } else {
        return 0;
    }

    if (ch >= SETTINGS_SCOPE_CHANNEL_COUNT || range >= SETTINGS_SCOPE_RANGE_COUNT) {
        return 0;
    }
    if (type) {
        settings->scope_bias_rate[ch][range] = value ? value : SETTINGS_SCOPE_BIAS_RATE_DEFAULT;
    } else {
        settings->scope_bias[ch][range] = value;
    }
    return 1;
}

static uint8_t settings_record_valid(uint32_t record, settings_state_t *settings) {
    uint16_t payload = (uint16_t)(record >> 8);
    uint8_t checksum = (uint8_t)record;

    if ((record & SETTINGS_RECORD_MAGIC_MASK) != SETTINGS_RECORD_MAGIC) {
        return 0;
    }
    if (checksum != settings_checksum(payload)) {
        return 0;
    }

    settings_defaults(settings);
    settings->dmm_mode = (uint8_t)(payload & 0x0Fu);
    settings->beep_level = (uint8_t)((payload >> 4) & 0x07u);
    settings->brightness_level = (uint8_t)((payload >> 7) & 0x07u);
    settings->startup_screen = (uint8_t)((payload >> 10) & 0x03u);
    settings->last_screen = (uint8_t)((payload >> 12) & 0x03u);
    settings->sleep_enabled = (uint8_t)((payload >> 14) & 0x01u);
    settings_clamp(settings);
    return 1;
}

static uint8_t settings_legacy_record_valid(uint32_t record, uint8_t *mode) {
    uint8_t value = (uint8_t)(record >> 8);
    uint8_t checksum = (uint8_t)record;

    if ((record & SETTINGS_LEGACY_MAGIC_MASK) != SETTINGS_LEGACY_MAGIC) {
        return 0;
    }
    if (checksum != (uint8_t)(value ^ 0x5Au)) {
        return 0;
    }
    *mode = value;
    return 1;
}

uint8_t settings_load(settings_state_t *settings) {
    uint8_t found = 0;
    settings_state_t latest;

    settings_defaults(&latest);
    for (uint32_t addr = SETTINGS_FLASH_BASE; addr < SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE; addr += 4u) {
        uint32_t record = REG32(addr);
        settings_state_t value;
        uint8_t legacy_mode;

        if (record == 0xFFFFFFFFu) {
            break;
        }
        if (settings_record_valid(record, &value)) {
            settings_copy(&latest, &value);
            found = 1;
        } else if (settings_scope_cal_record_valid(record, &latest)) {
            found = 1;
        } else if (settings_legacy_record_valid(record, &legacy_mode)) {
            latest.dmm_mode = legacy_mode;
            found = 1;
        }
    }

    settings_clamp(&latest);
    settings_copy(&settings_cached, &latest);
    settings_copy(&settings_pending, &latest);
    settings_loaded = 1;
    settings_dirty = 0;
    if (settings) {
        settings_copy(settings, &latest);
    }
    return found;
}

static void settings_ensure_loaded(void) {
    if (!settings_loaded) {
        (void)settings_load(0);
    }
}

static volatile uint32_t *flash_sts_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x4Cu) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x0Cu);
}

static volatile uint32_t *flash_ctrl_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x50u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x10u);
}

static volatile uint32_t *flash_addr_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x54u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x14u);
}

static volatile uint32_t *flash_keyr_reg(uint32_t addr) {
    return addr >= FLASH_BANK2_BASE ?
        (volatile uint32_t *)(FLASH_R_BASE + 0x44u) :
        (volatile uint32_t *)(FLASH_R_BASE + 0x04u);
}

static void flash_wait(uint32_t addr) {
    volatile uint32_t *sts = flash_sts_reg(addr);

    while (*sts & FLASH_STS_BSY) {
    }
}

static void flash_unlock(uint32_t addr) {
    volatile uint32_t *ctrl = flash_ctrl_reg(addr);
    volatile uint32_t *keyr = flash_keyr_reg(addr);

    if (*ctrl & FLASH_CTRL_LOCK) {
        *keyr = 0x45670123u;
        *keyr = 0xCDEF89ABu;
    }
}

static void flash_lock(uint32_t addr) {
    *flash_ctrl_reg(addr) |= FLASH_CTRL_LOCK;
}

static void flash_clear_status(uint32_t addr) {
    *flash_sts_reg(addr) = FLASH_STS_EOP | FLASH_STS_PGERR | FLASH_STS_WRPRTERR;
}

static void flash_erase_settings_page(void) {
    volatile uint32_t *ctrl = flash_ctrl_reg(SETTINGS_FLASH_BASE);
    volatile uint32_t *addr_reg = flash_addr_reg(SETTINGS_FLASH_BASE);

    flash_wait(SETTINGS_FLASH_BASE);
    flash_clear_status(SETTINGS_FLASH_BASE);
    *ctrl |= FLASH_CTRL_PER;
    *addr_reg = SETTINGS_FLASH_BASE;
    *ctrl |= FLASH_CTRL_STRT;
    flash_wait(SETTINGS_FLASH_BASE);
    *ctrl &= ~FLASH_CTRL_PER;
    flash_clear_status(SETTINGS_FLASH_BASE);
}

static void flash_program_halfword(uint32_t addr, uint16_t value) {
    volatile uint32_t *ctrl = flash_ctrl_reg(addr);

    flash_wait(addr);
    flash_clear_status(addr);
    *ctrl |= FLASH_CTRL_PG;
    REG16(addr) = value;
    flash_wait(addr);
    *ctrl &= ~FLASH_CTRL_PG;
    flash_clear_status(addr);
}

static void flash_program_word(uint32_t addr, uint32_t value) {
    flash_program_halfword(addr, (uint16_t)value);
    flash_program_halfword(addr + 2u, (uint16_t)(value >> 16));
}

static uint32_t settings_first_free_addr(void) {
    for (uint32_t addr = SETTINGS_FLASH_BASE; addr < SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE; addr += 4u) {
        if (REG32(addr) == 0xFFFFFFFFu) {
            return addr;
        }
    }
    return 0;
}

static void settings_write(const settings_state_t *settings) {
    uint32_t addr;
    uint32_t record = settings_record_for(settings);

    __asm__ volatile("cpsid i" ::: "memory");
    flash_unlock(SETTINGS_FLASH_BASE);
    addr = settings_first_free_addr();
    if (!addr || addr + SETTINGS_WRITE_BYTES > SETTINGS_FLASH_BASE + SETTINGS_FLASH_SIZE) {
        flash_erase_settings_page();
        addr = SETTINGS_FLASH_BASE;
    }
    flash_program_word(addr, record);
    addr += 4u;
    for (uint8_t ch = 0; ch < SETTINGS_SCOPE_CHANNEL_COUNT; ++ch) {
        for (uint8_t range = 0; range < SETTINGS_SCOPE_RANGE_COUNT; ++range) {
            flash_program_word(addr, settings_scope_cal_record(0, ch, range, settings->scope_bias[ch][range]));
            addr += 4u;
            flash_program_word(addr, settings_scope_cal_record(1, ch, range, settings->scope_bias_rate[ch][range]));
            addr += 4u;
        }
    }
    flash_lock(SETTINGS_FLASH_BASE);
    __asm__ volatile("cpsie i" ::: "memory");

    settings_copy(&settings_cached, settings);
    settings_loaded = 1;
}

void settings_note(const settings_state_t *settings) {
    settings_ensure_loaded();
    settings_copy(&settings_pending, settings);
    settings_clamp(&settings_pending);
    settings_dirty = settings_equal(&settings_pending, &settings_cached) ? 0u : 1u;
}

uint8_t settings_load_dmm_mode(uint8_t *mode) {
    settings_state_t settings;
    uint8_t found = settings_load(&settings);
    if (mode) {
        *mode = settings.dmm_mode;
    }
    return found;
}

void settings_note_dmm_mode(uint8_t mode) {
    settings_ensure_loaded();
    settings_pending.dmm_mode = mode;
    settings_dirty = settings_equal(&settings_pending, &settings_cached) ? 0u : 1u;
}

void settings_flush(void) {
    settings_ensure_loaded();
    if (!settings_dirty) {
        return;
    }

    settings_write(&settings_pending);
    settings_dirty = 0;
}
