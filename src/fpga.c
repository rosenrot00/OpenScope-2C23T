#include "fpga.h"

#include "app_config.h"
#include "board.h"
#include "fpga_bitstream.h"
#include "hw.h"

#include <stdint.h>

enum {
    FPGA_LATCH_SETTLE_MS = 1,
    SPI_CAPTURE_TIMEOUT = 60000u,

    SPI_STS_RXNE = 1u << 0,
    SPI_STS_TXE = 1u << 1,
    SPI_STS_BSY = 1u << 7,
};

#ifndef FPGA_SPI_BR
#define FPGA_SPI_BR 2u
#endif

static uint8_t fpga_ready_flag;
static uint8_t fpga_loaded;

static void fpga_start_clock_output(void) {
    RCC_CFGR = (RCC_CFGR & ~(7u << 24)) | (4u << 24);
    RCC_CFGR2 = (RCC_CFGR2 & ~((1u << 16) | (15u << 28))) | (11u << 28);
}

#if HW_TARGET_HW40

enum {
    FPGA_HW4_BUS_MASK = 0x00FFu,
    FPGA_HW4_CLK_PIN = 1u << 3,   // PB3
    FPGA_HW4_MISO_PIN = 1u << 4,  // PB4, also capture-ready in runtime mode
    FPGA_HW4_MOSI_PIN = 1u << 5,  // PB5
    FPGA_HW4_SELECT_PIN = 1u << 15, // PA15
    FPGA_HW4_RESET_PIN = 1u << 8, // PC8
};

static uint32_t fpga_last_tuning_word;
static uint32_t fpga_last_span;

static void fpga_hw4_short_delay(void) {
    for (volatile uint32_t i = 0; i < 80u; ++i) {
        __asm__ volatile("nop");
    }
}

static void fpga_hw4_bus_input(void) {
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_BUS_MASK, 0x4u);
}

static void fpga_hw4_bus_output(void) {
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_BUS_MASK, 0x1u);
}

static void fpga_hw4_bus_write(uint8_t value) {
    GPIO_ODR(GPIOC_BASE) = (GPIO_ODR(GPIOC_BASE) & ~FPGA_HW4_BUS_MASK) | value;
}

static uint8_t fpga_hw4_bitbang_transfer_byte(uint8_t value) {
    uint8_t result = 0;

    for (uint8_t i = 0; i < 8u; ++i) {
        gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
        if (value & 0x80u) {
            gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
        } else {
            gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
        }
        gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
        result = (uint8_t)(result << 1);
        value = (uint8_t)(value << 1);
        if (gpio_read(GPIOB_BASE, FPGA_HW4_MISO_PIN)) {
            result |= 1u;
        }
    }
    return result;
}

static uint32_t fpga_hw4_read_register32(uint32_t addr) {
    uint8_t bytes[4];

    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 24));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 16));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 8));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)addr);
    for (uint8_t i = 0; i < 4u; ++i) {
        bytes[i] = fpga_hw4_bitbang_transfer_byte(0);
    }
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);

    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

static void fpga_hw4_write_command_word(uint16_t command) {
    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(command >> 8));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)command);
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
}

static void fpga_hw4_parallel_write_byte(uint8_t value) {
    fpga_hw4_bus_write(value);
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
}

static uint8_t fpga_hw4_parallel_read_byte(void) {
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    return (uint8_t)(GPIO_IDR(GPIOC_BASE) & FPGA_HW4_BUS_MASK);
}

static void fpga_hw4_write_timing_frame(uint32_t tuning_word, uint32_t span) {
    gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_output();

    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 24));
    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 16));
    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 8));
    fpga_hw4_parallel_write_byte((uint8_t)tuning_word);
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 24));
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 16));
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 8));
    fpga_hw4_parallel_write_byte((uint8_t)span);

    for (uint8_t i = 0; i < 8u; ++i) {
        fpga_hw4_parallel_write_byte(i);
    }

    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

static void fpga_hw4_begin(void) {
    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4);
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    gpio_config_mask(GPIOB_BASE, FPGA_HW4_CLK_PIN | FPGA_HW4_MOSI_PIN, 0x1u);
    gpio_config_mask(GPIOB_BASE, FPGA_HW4_MISO_PIN, 0x4u);
    gpio_config_mask(GPIOA_BASE, FPGA_HW4_SELECT_PIN, 0x1u);
    gpio_config_mask(GPIOA_BASE, 1u << 8, 0x9u);
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_RESET_PIN, 0x1u);
    fpga_hw4_bus_input();

    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN | FPGA_HW4_MOSI_PIN);
    fpga_start_clock_output();
}

void fpga_init_once(void) {
    uint32_t status;

    if (fpga_loaded) {
        return;
    }

    fpga_hw4_begin();
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOC_BASE, FPGA_HW4_RESET_PIN);
    delay_ms(10);
    gpio_set(GPIOC_BASE, FPGA_HW4_RESET_PIN);
    delay_ms(1);

    (void)fpga_hw4_read_register32(0x11000000u);
    (void)fpga_hw4_read_register32(0x13000000u);
    (void)fpga_hw4_read_register32(0x41000000u);
    fpga_hw4_write_command_word(0x1200u);
    fpga_hw4_write_command_word(0x1500u);
    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte(0x3Bu);
    for (uint32_t i = 0; i < fpga_bitstream_len; ++i) {
        (void)fpga_hw4_bitbang_transfer_byte(fpga_bitstream[i]);
    }
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);

    status = fpga_hw4_read_register32(0x41000000u);
    fpga_hw4_write_command_word(0x3A00u);
    delay_ms(100);
    fpga_ready_flag = status != 0xFFFFFFFFu ? 1u : 0u;
    fpga_loaded = 1u;
}

uint8_t fpga_ready(void) {
    return fpga_ready_flag;
}

void fpga_write_timing(uint32_t tuning_word, uint32_t span) {
    fpga_last_tuning_word = tuning_word;
    fpga_last_span = span;
    fpga_hw4_write_timing_frame(tuning_word, span);
}

void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_output();
    for (uint16_t i = 0; i < len; ++i) {
        fpga_hw4_parallel_write_byte(data[i]);
    }
    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

void fpga_capture_latch(void) {
    fpga_hw4_write_timing_frame(fpga_last_tuning_word, fpga_last_span);
}

uint8_t fpga_capture_ready(void) {
    return gpio_read(GPIOB_BASE, FPGA_HW4_MISO_PIN) ? 1u : 0u;
}

uint8_t fpga_capture_read(uint8_t *dst, uint16_t len) {
    fpga_hw4_bus_input();
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = fpga_hw4_parallel_read_byte();
    }
    return 1u;
}

uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    if (!sample) {
        return 0;
    }

    fpga_hw4_bus_input();
    sample[0] = fpga_hw4_parallel_read_byte();
    sample[1] = fpga_hw4_parallel_read_byte();
    fpga_capture_latch();
    return 1u;
}

#else

static void spi3_write_raw(uint8_t value) {
    while (!(SPI_STS(SPI3_BASE) & SPI_STS_TXE)) {
    }
    SPI_DT(SPI3_BASE) = value;
    while (SPI_STS(SPI3_BASE) & SPI_STS_BSY) {
    }
    (void)SPI_DT(SPI3_BASE);
}

static uint8_t spi3_transfer_raw_timeout(uint8_t value, uint8_t *out) {
    uint32_t timeout = SPI_CAPTURE_TIMEOUT;

    while (!(SPI_STS(SPI3_BASE) & SPI_STS_TXE)) {
        if (!--timeout) {
            return 0;
        }
    }
    SPI_DT(SPI3_BASE) = value;

    timeout = SPI_CAPTURE_TIMEOUT;
    while (SPI_STS(SPI3_BASE) & SPI_STS_BSY) {
        if (!--timeout) {
            return 0;
        }
    }

    timeout = SPI_CAPTURE_TIMEOUT;
    while (!(SPI_STS(SPI3_BASE) & SPI_STS_RXNE)) {
        if (!--timeout) {
            return 0;
        }
    }
    *out = (uint8_t)SPI_DT(SPI3_BASE);
    return 1;
}

static void spi3_write_framed(uint8_t value) {
    gpio_clear(GPIOA_BASE, 1u << 15);
    spi3_write_raw(value);
    gpio_set(GPIOA_BASE, 1u << 15);
}

static void fpga_spi_begin(void) {
    RCC_APB1ENR |= 1u << 15; // SPI3

    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    gpio_config_mask(GPIOC_BASE, (1u << 0) | (1u << 1) | (1u << 2) | (1u << 9) | (1u << 10), 0x1);
    gpio_config_mask(GPIOA_BASE, 1u << 15, 0x1);
    gpio_config_mask(GPIOB_BASE, (1u << 3) | (1u << 5), 0x9u);
    gpio_config_mask(GPIOB_BASE, 1u << 4, 0x4);
    gpio_config_mask(GPIOA_BASE, 1u << 8, 0x9u);
    gpio_config_mask(GPIOC_BASE, 1u << 3, 0x4);
    gpio_config_mask(GPIOC_BASE, 1u << 8, 0x8);
    gpio_set(GPIOC_BASE, 1u << 8);

    gpio_set(GPIOA_BASE, 1u << 15);
    gpio_clear(GPIOC_BASE, (1u << 0) | (1u << 1) | (1u << 2));
    fpga_start_clock_output();

    SPI_CTRL1(SPI3_BASE) = 0;
    SPI_CTRL2(SPI3_BASE) = 0;
    SPI_CTRL1(SPI3_BASE) = (1u << 9) | (1u << 8) | (1u << 2) |
                           (((uint32_t)FPGA_SPI_BR & 7u) << 3) |
                           (1u << 1) | (1u << 0);
    SPI_CTRL1(SPI3_BASE) |= 1u << 6;
}

static void fpga_write_strobe(void) {
    gpio_set(GPIOC_BASE, 1u << 2);
    spi3_write_framed(0);
    gpio_clear(GPIOC_BASE, 1u << 2);
}

void fpga_init_once(void) {
    if (fpga_loaded) {
        return;
    }

    fpga_spi_begin();

    gpio_clear(GPIOA_BASE, 1u << 15);
    gpio_clear(GPIOC_BASE, 1u << 9);
    delay_ms(10);
    gpio_set(GPIOC_BASE, 1u << 9);
    delay_ms(1);

    for (uint32_t i = 0; i < fpga_bitstream_len; ++i) {
        spi3_write_raw(fpga_bitstream[i]);
    }
    for (uint16_t i = 0; i < 200u; ++i) {
        spi3_write_raw(0);
    }
    fpga_ready_flag = gpio_read(GPIOC_BASE, 1u << 8) ? 1u : 0u;
    gpio_set(GPIOA_BASE, 1u << 15);

    gpio_clear(GPIOC_BASE, 1u << 2);
    gpio_set(GPIOC_BASE, 1u << 10);
    delay_ms(1);
    gpio_clear(GPIOC_BASE, 1u << 0);
    gpio_clear(GPIOC_BASE, 1u << 10);
    delay_ms(1);
    gpio_set(GPIOC_BASE, 1u << 10);
    delay_ms(1);

    if (gpio_read(GPIOC_BASE, 1u << 8)) {
        fpga_ready_flag = 1u;
    }
    fpga_loaded = 1u;
}

uint8_t fpga_ready(void) {
    return fpga_ready_flag;
}

void fpga_write_timing(uint32_t tuning_word, uint32_t span) {
    fpga_write_strobe();
    gpio_set(GPIOC_BASE, 1u << 0);
    gpio_clear(GPIOC_BASE, 1u << 1);

    spi3_write_framed((uint8_t)(tuning_word >> 24));
    spi3_write_framed((uint8_t)(tuning_word >> 16));
    spi3_write_framed((uint8_t)(tuning_word >> 8));
    spi3_write_framed((uint8_t)tuning_word);
    spi3_write_framed((uint8_t)(span >> 24));
    spi3_write_framed((uint8_t)(span >> 16));
    spi3_write_framed((uint8_t)(span >> 8));
    spi3_write_framed((uint8_t)span);

    gpio_clear(GPIOC_BASE, 1u << 0);
}

void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    fpga_write_strobe();
    gpio_set(GPIOC_BASE, (1u << 0) | (1u << 1));
    for (uint16_t i = 0; i < len; ++i) {
        spi3_write_framed(data[i]);
    }
    gpio_clear(GPIOC_BASE, 1u << 0);
}

void fpga_capture_latch(void) {
    gpio_set(GPIOC_BASE, 1u << 0);
    gpio_set(GPIOC_BASE, 1u << 2);
    delay_ms(FPGA_LATCH_SETTLE_MS);
    gpio_clear(GPIOC_BASE, 1u << 2);
    gpio_clear(GPIOC_BASE, 1u << 0);
}

uint8_t fpga_capture_ready(void) {
    return gpio_read(GPIOC_BASE, 1u << 3) ? 1u : 0u;
}

uint8_t fpga_capture_read(uint8_t *dst, uint16_t len) {
    gpio_clear(GPIOC_BASE, 1u << 0);
    fpga_write_strobe();
    (void)SPI_DT(SPI3_BASE);
    gpio_set(GPIOC_BASE, 1u << 1);

    for (uint16_t i = 0; i < len; ++i) {
        gpio_clear(GPIOA_BASE, 1u << 15);
        if (!spi3_transfer_raw_timeout(0, &dst[i])) {
            gpio_set(GPIOA_BASE, 1u << 15);
            gpio_clear(GPIOC_BASE, 1u << 1);
            return 0;
        }
        gpio_set(GPIOA_BASE, 1u << 15);
    }
    gpio_clear(GPIOC_BASE, 1u << 1);
    return 1;
}

uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    if (!sample) {
        return 0;
    }

    gpio_clear(GPIOC_BASE, 1u << 0);
    fpga_write_strobe();
    (void)SPI_DT(SPI3_BASE);
    gpio_set(GPIOC_BASE, 1u << 1);

    for (uint8_t i = 0; i < 2u; ++i) {
        gpio_clear(GPIOA_BASE, 1u << 15);
        if (!spi3_transfer_raw_timeout(0, &sample[i])) {
            gpio_set(GPIOA_BASE, 1u << 15);
            gpio_clear(GPIOC_BASE, 1u << 1);
            return 0;
        }
        gpio_set(GPIOA_BASE, 1u << 15);
    }

    fpga_capture_latch();
    return 1;
}

#endif
