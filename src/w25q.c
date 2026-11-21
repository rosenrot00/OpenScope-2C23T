#include "w25q.h"

#include "board.h"
#include "hw.h"

#include <stdint.h>

enum {
    W25Q_CS_PIN = 1u << 12,
    SPI_STS_RXNE = 1u << 0,
    SPI_STS_TXE = 1u << 1,
    SPI_STS_BSY = 1u << 7,
    W25Q_SECTOR_SIZE = 4096,
    W25Q_PAGE_SIZE = 256,
    SPI_TIMEOUT = 60000u,
    // Cycle budgets remain conservative across the supported processor clocks.
    W25Q_ERASE_TIMEOUT_CYCLES = 288000000u,
    W25Q_PAGE_TIMEOUT_CYCLES = 28800000u,
};

#ifndef W25Q_SPI_BR
#define W25Q_SPI_BR 2u
#endif

static uint8_t w25q_ready;
static uint16_t w25q_id;
static uint32_t w25q_capacity;

enum W25qWritePhase {
    W25Q_WAIT_INITIAL,
    W25Q_WAIT_ERASE,
    W25Q_WAIT_PAGE,
};

static volatile w25q_write_status_t write_status;
static enum W25qWritePhase write_phase;
static const uint8_t *write_source;
static uint32_t write_address;
static uint32_t write_wait_start;
static uint16_t write_offset;

static void w25q_select(void) {
    gpio_clear(GPIOB_BASE, W25Q_CS_PIN);
}

static void w25q_deselect(void) {
    gpio_set(GPIOB_BASE, W25Q_CS_PIN);
}

static uint8_t spi2_transfer(uint8_t value, uint8_t *out) {
    uint32_t timeout = SPI_TIMEOUT;

    while (!(SPI_STS(SPI2_BASE) & SPI_STS_TXE)) {
        if (!--timeout) {
            return 0;
        }
    }
    SPI_DT(SPI2_BASE) = value;

    timeout = SPI_TIMEOUT;
    while (!(SPI_STS(SPI2_BASE) & SPI_STS_RXNE)) {
        if (!--timeout) {
            return 0;
        }
    }
    value = (uint8_t)SPI_DT(SPI2_BASE);

    timeout = SPI_TIMEOUT;
    while (SPI_STS(SPI2_BASE) & SPI_STS_BSY) {
        if (!--timeout) {
            return 0;
        }
    }
    if (out) {
        *out = value;
    }
    return 1;
}

static uint8_t spi2_write(uint8_t value) {
    return spi2_transfer(value, 0);
}

static uint8_t w25q_read_status(uint8_t *status) {
    w25q_select();
    if (!spi2_write(0x05u) || !spi2_transfer(0xFFu, status)) {
        w25q_deselect();
        return 0;
    }
    w25q_deselect();
    return 1;
}

static uint8_t w25q_write_enable(void) {
    w25q_select();
    if (!spi2_write(0x06u)) {
        w25q_deselect();
        return 0;
    }
    w25q_deselect();
    return 1;
}

static uint8_t w25q_erase_sector_start(uint32_t addr) {
    if (!w25q_write_enable()) {
        return 0;
    }
    w25q_select();
    if (!spi2_write(0x20u) ||
        !spi2_write((uint8_t)(addr >> 16)) ||
        !spi2_write((uint8_t)(addr >> 8)) ||
        !spi2_write((uint8_t)addr)) {
        w25q_deselect();
        return 0;
    }
    w25q_deselect();
    return 1;
}

static uint8_t w25q_page_program_start(uint32_t addr, const uint8_t *src) {
    if (!w25q_write_enable()) {
        return 0;
    }
    w25q_select();
    if (!spi2_write(0x02u) ||
        !spi2_write((uint8_t)(addr >> 16)) ||
        !spi2_write((uint8_t)(addr >> 8)) ||
        !spi2_write((uint8_t)addr)) {
        w25q_deselect();
        return 0;
    }
    for (uint16_t i = 0; i < W25Q_PAGE_SIZE; ++i) {
        if (!spi2_write(src[i])) {
            w25q_deselect();
            return 0;
        }
    }
    w25q_deselect();
    return 1;
}

static uint16_t w25q_read_id90(void) {
    uint8_t manufacturer = 0;
    uint8_t device = 0;

    w25q_select();
    if (!spi2_write(0x90u) ||
        !spi2_write(0x00u) ||
        !spi2_write(0x00u) ||
        !spi2_write(0x00u) ||
        !spi2_transfer(0xFFu, &manufacturer) ||
        !spi2_transfer(0xFFu, &device)) {
        w25q_deselect();
        return 0;
    }
    w25q_deselect();
    return (uint16_t)(((uint16_t)manufacturer << 8) | device);
}

void w25q_init(void) {
    if (w25q_ready) {
        return;
    }

    RCC_APB2ENR |= (1u << 0) | (1u << 3);
    RCC_APB1ENR |= 1u << 14; // SPI2

    gpio_config_mask(GPIOB_BASE, W25Q_CS_PIN, 0x1u);
    w25q_deselect();
    gpio_config_mask(GPIOB_BASE, (1u << 13) | (1u << 15), 0x9u);
    gpio_config_mask(GPIOB_BASE, 1u << 14, 0x4u);

    SPI_CTRL1(SPI2_BASE) = 0;
    SPI_CTRL2(SPI2_BASE) = 0;
    SPI_CTRL1(SPI2_BASE) = (1u << 9) | (1u << 8) | (1u << 2) |
                           (((uint32_t)W25Q_SPI_BR & 7u) << 3);
    SPI_CTRL1(SPI2_BASE) |= 1u << 6;

    w25q_id = w25q_read_id90();
    if (w25q_id == 0xEF15u) {
        w25q_capacity = 0x00400000u;
    } else if (w25q_id == 0xEF16u) {
        w25q_capacity = 0x00800000u;
    } else if (w25q_id == 0xEF17u) {
        w25q_capacity = 0x01000000u;
    } else {
        w25q_capacity = 0;
    }
    w25q_ready = 1;
}

uint32_t w25q_capacity_bytes(void) {
    return w25q_capacity;
}

uint16_t w25q_device_id(void) {
    return w25q_id;
}

uint8_t w25q_read(uint32_t addr, uint8_t *dst, uint16_t len) {
    uint8_t status;
    if (w25q_write_sector_busy() || !w25q_capacity || !dst || addr > w25q_capacity ||
        (uint32_t)len > w25q_capacity - addr) {
        return 0;
    }
    // A failed command can still have started a physical erase/program.
    if (!w25q_read_status(&status) || (status & 1u)) {
        return 0;
    }

    w25q_select();
    if (!spi2_write(0x03u) ||
        !spi2_write((uint8_t)(addr >> 16)) ||
        !spi2_write((uint8_t)(addr >> 8)) ||
        !spi2_write((uint8_t)addr)) {
        w25q_deselect();
        return 0;
    }
    for (uint16_t i = 0; i < len; ++i) {
        if (!spi2_transfer(0xFFu, &dst[i])) {
            w25q_deselect();
            return 0;
        }
    }
    w25q_deselect();
    return 1;
}

static w25q_write_status_t w25q_write_finish(w25q_write_status_t status) {
    write_source = 0;
    write_status = status;
    return status;
}

uint8_t w25q_write_sector_start(uint32_t sector_addr, const uint8_t *src) {
    if (w25q_write_sector_busy()) {
        return 0;
    }
    if (w25q_capacity < W25Q_SECTOR_SIZE || !src ||
        (sector_addr & (W25Q_SECTOR_SIZE - 1u)) != 0 ||
        sector_addr > w25q_capacity - W25Q_SECTOR_SIZE) {
        (void)w25q_write_finish(W25Q_WRITE_ERROR);
        return 0;
    }

    write_source = src;
    write_address = sector_addr;
    write_offset = 0;
    write_phase = W25Q_WAIT_INITIAL;
    write_wait_start = board_cycle_counter_read();
    write_status = W25Q_WRITE_BUSY;
    return 1;
}

w25q_write_status_t w25q_write_sector_poll(void) {
    uint8_t status;
    if (!w25q_write_sector_busy()) {
        return write_status;
    }
    if (!w25q_read_status(&status)) {
        return w25q_write_finish(W25Q_WRITE_ERROR);
    }
    if (status & 1u) {
        uint32_t budget = write_phase == W25Q_WAIT_PAGE ?
            W25Q_PAGE_TIMEOUT_CYCLES : W25Q_ERASE_TIMEOUT_CYCLES;
        if ((uint32_t)(board_cycle_counter_read() - write_wait_start) >= budget) {
            return w25q_write_finish(W25Q_WRITE_ERROR);
        }
        return W25Q_WRITE_BUSY;
    }

    if (write_phase == W25Q_WAIT_INITIAL) {
        if (!w25q_erase_sector_start(write_address)) {
            return w25q_write_finish(W25Q_WRITE_ERROR);
        }
        write_phase = W25Q_WAIT_ERASE;
        write_wait_start = board_cycle_counter_read();
        return W25Q_WRITE_BUSY;
    }

    while (write_offset < W25Q_SECTOR_SIZE) {
        uint16_t offset = write_offset;
        uint8_t non_empty = 0;
        for (uint16_t i = 0; i < W25Q_PAGE_SIZE; ++i) {
            if (write_source[offset + i] != 0xFFu) {
                non_empty = 1;
                break;
            }
        }
        write_offset = (uint16_t)(write_offset + W25Q_PAGE_SIZE);
        if (non_empty) {
            if (!w25q_page_program_start(write_address + offset, write_source + offset)) {
                return w25q_write_finish(W25Q_WRITE_ERROR);
            }
            write_phase = W25Q_WAIT_PAGE;
            write_wait_start = board_cycle_counter_read();
            return W25Q_WRITE_BUSY;
        }
    }
    return w25q_write_finish(W25Q_WRITE_DONE);
}

uint8_t w25q_write_sector_busy(void) {
    return write_status == W25Q_WRITE_BUSY;
}

uint8_t w25q_write_sector(uint32_t sector_addr, const uint8_t *src) {
    if (!w25q_write_sector_start(sector_addr, src)) {
        return 0;
    }
    while (w25q_write_sector_poll() == W25Q_WRITE_BUSY) {
    }
    return write_status == W25Q_WRITE_DONE;
}
