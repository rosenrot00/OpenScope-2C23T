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
    W25Q_WRITE_TIMEOUT = 5000000u,
};

#ifndef W25Q_SPI_BR
#define W25Q_SPI_BR 2u
#endif

static uint8_t w25q_ready;
static uint16_t w25q_id;
static uint32_t w25q_capacity;

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

static uint8_t w25q_wait_ready(void) {
    uint8_t status = 0;
    uint32_t timeout = W25Q_WRITE_TIMEOUT;

    do {
        if (!w25q_read_status(&status)) {
            return 0;
        }
        if (!(status & 1u)) {
            return 1;
        }
    } while (--timeout);
    return 0;
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

static uint8_t w25q_erase_sector(uint32_t addr) {
    if (!w25q_wait_ready() || !w25q_write_enable()) {
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
    return w25q_wait_ready();
}

static uint8_t w25q_page_program(uint32_t addr, const uint8_t *src, uint16_t len) {
    if (!w25q_wait_ready() || !w25q_write_enable()) {
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
    for (uint16_t i = 0; i < len; ++i) {
        if (!spi2_write(src[i])) {
            w25q_deselect();
            return 0;
        }
    }
    w25q_deselect();
    return w25q_wait_ready();
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
    if (!w25q_capacity || !dst || addr > w25q_capacity ||
        (uint32_t)len > w25q_capacity - addr) {
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

uint8_t w25q_write_sector(uint32_t sector_addr, const uint8_t *src) {
    if (!w25q_capacity || !src ||
        (sector_addr & (W25Q_SECTOR_SIZE - 1u)) != 0 ||
        sector_addr > w25q_capacity - W25Q_SECTOR_SIZE) {
        return 0;
    }
    if (!w25q_erase_sector(sector_addr)) {
        return 0;
    }
    for (uint16_t off = 0; off < W25Q_SECTOR_SIZE; off = (uint16_t)(off + W25Q_PAGE_SIZE)) {
        uint8_t non_empty = 0;
        for (uint16_t i = 0; i < W25Q_PAGE_SIZE; ++i) {
            if (src[off + i] != 0xFFu) {
                non_empty = 1;
                break;
            }
        }
        if (non_empty && !w25q_page_program(sector_addr + off, &src[off], W25Q_PAGE_SIZE)) {
            return 0;
        }
    }
    return 1;
}
