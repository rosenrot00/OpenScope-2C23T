#include "../src/hw.h"
#include "../src/board.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

enum SpiFault { SPI_OK, SPI_NO_TXE, SPI_NO_RXNE, SPI_STUCK_BSY };
enum PendingOperation { OP_NONE, OP_ERASE, OP_PROGRAM };

static uint8_t flash[16u * 1024u * 1024u];
static uint8_t source[4096];
static uint8_t other_source[4096];
static uint8_t command_data[256];
static uint8_t pending_data[256];
static uint32_t registers[256];
static volatile uint32_t spi_data;
static volatile uint32_t spi_status;
static uint32_t cycles;
static uint32_t cycle_step;
static uint32_t command_address;
static uint32_t pending_address;
static unsigned command_length;
static unsigned program_length;
static unsigned pending_length;
static unsigned transfers;
static unsigned status_register_reads;
static unsigned status_checks;
static unsigned erase_starts;
static unsigned program_starts;
static unsigned pending_checks;
static unsigned fail_after;
static uint8_t selected;
static uint8_t tx_pending;
static uint8_t rx_pending;
static uint8_t command;
static uint8_t write_enabled;
static uint8_t initially_busy;
static uint8_t hold_busy;
static uint8_t manufacturer;
static uint8_t device;
static enum SpiFault fault;
static enum PendingOperation pending_operation;

static uint8_t simulated_status(void) {
    ++status_checks;
    if (initially_busy) {
        return 1;
    }
    if (pending_operation != OP_NONE) {
        if (hold_busy) {
            return 1;
        }
        if (pending_checks) {
            --pending_checks;
            return 1;
        }
        // Flash changes become durable only after the operation finishes.
        if (pending_operation == OP_ERASE) {
            memset(flash + pending_address, 0xFF, 4096);
        } else {
            for (unsigned i = 0; i < pending_length; ++i) {
                flash[pending_address + i] &= pending_data[i];
            }
        }
        pending_operation = OP_NONE;
    }
    return write_enabled ? 2u : 0u;
}

static uint8_t exchange(uint8_t value) {
    unsigned position = command_length++;
    assert(selected);
    ++transfers;
    if (!position) {
        command = value;
        return 0xFF;
    }
    if (command == 0x05) {
        assert(position == 1);
        return simulated_status();
    }
    if (position <= 3) {
        command_address = (command_address << 8) | value;
        return 0xFF;
    }
    if (command == 0x90) {
        return position == 4 ? manufacturer : device;
    }
    if (command == 0x03) {
        assert(command_address < sizeof(flash));
        assert(!initially_busy && pending_operation == OP_NONE);
        return flash[command_address++];
    }
    assert(command == 0x02 && program_length < sizeof(command_data));
    command_data[program_length++] = value;
    return 0xFF;
}

static volatile uint32_t *mock_register(uint32_t address) {
    if (address == SPI2_BASE + 8u) {
        uint8_t failing = fault != SPI_OK && transfers >= fail_after;
        ++status_register_reads;
        spi_status = failing && fault == SPI_NO_TXE ? 0u : 2u;
        if (tx_pending && !(failing && fault == SPI_NO_RXNE)) {
            spi_data = exchange((uint8_t)spi_data);
            tx_pending = 0;
            rx_pending = 1;
        }
        if (rx_pending) {
            spi_status |= 1u;
        }
        if (failing && fault == SPI_STUCK_BSY) {
            spi_status |= 0x80u;
        }
        return &spi_status;
    }
    if (address == SPI2_BASE + 12u) {
        if (rx_pending) {
            rx_pending = 0;
        } else {
            tx_pending = 1;
        }
        return &spi_data;
    }
    return &registers[(address / 4u) % 256u];
}

static void mock_gpio_clear(uint32_t base, uint32_t mask) {
    assert(base == GPIOB_BASE && mask == (1u << 12) && !selected);
    selected = 1;
    command = 0;
    command_length = program_length = 0;
    command_address = 0;
    tx_pending = rx_pending = 0;
}

static void mock_gpio_set(uint32_t base, uint32_t mask) {
    assert(base == GPIOB_BASE && mask == (1u << 12));
    if (!selected) {
        return;
    }
    selected = 0;
    tx_pending = rx_pending = 0;
    if (command == 0x06 && command_length == 1) {
        write_enabled = 1;
    } else if ((command == 0x20 && command_length == 4) ||
               (command == 0x02 && program_length)) {
        assert(write_enabled && !initially_busy && pending_operation == OP_NONE);
        pending_address = command_address;
        assert(pending_address < sizeof(flash));
        if (command == 0x20) {
            assert(!(pending_address % 4096u));
            pending_operation = OP_ERASE;
            ++erase_starts;
        } else {
            assert(!(pending_address % 256u));
            pending_operation = OP_PROGRAM;
            pending_length = program_length;
            memcpy(pending_data, command_data, program_length);
            ++program_starts;
        }
        write_enabled = 0;
        pending_checks = 2;
    }
}

#undef REG32
#define REG32(address) (*mock_register(address))
#define gpio_set mock_gpio_set
#define gpio_clear mock_gpio_clear
#include "../src/w25q.c"

void gpio_config_mask(uint32_t base, uint16_t mask, uint8_t cfg) {
    (void)base;
    (void)mask;
    (void)cfg;
}

uint32_t board_cycle_counter_read(void) {
    uint32_t value = cycles;
    cycles += cycle_step;
    return value;
}

static void setup(void) {
    memset(flash, 0x3C, sizeof(flash));
    memset(source, 0xFF, sizeof(source));
    memset(other_source, 0x55, sizeof(other_source));
    memset(registers, 0, sizeof(registers));
    cycles = cycle_step = 0;
    command_length = program_length = 0;
    transfers = status_register_reads = status_checks = 0;
    erase_starts = program_starts = 0;
    selected = tx_pending = rx_pending = write_enabled = 0;
    initially_busy = hold_busy = 0;
    manufacturer = 0xEF;
    device = 0x15;
    fault = SPI_OK;
    fail_after = 0;
    pending_operation = OP_NONE;
    w25q_ready = 0;
    w25q_capacity = 0;
    write_status = W25Q_WRITE_IDLE;
    write_source = 0;
    w25q_init();
    assert(w25q_capacity_bytes() == 4u * 1024u * 1024u);
    assert(w25q_device_id() == 0xEF15);
    assert(w25q_write_sector_poll() == W25Q_WRITE_IDLE);
    assert(!w25q_write_sector_busy() && !selected);
}

static w25q_write_status_t poll_checked(void) {
    unsigned before_checks = status_checks;
    unsigned before_commands = erase_starts + program_starts;
    unsigned before_transfers = transfers;
    unsigned before_status_reads = status_register_reads;
    w25q_write_status_t result = w25q_write_sector_poll();
    assert(status_checks - before_checks <= 1u);
    assert(erase_starts + program_starts - before_commands <= 1u);
    assert(transfers - before_transfers <= 2u + 1u + 4u + 256u);
    assert(status_register_reads - before_status_reads <= SPI_TIMEOUT + 3u * 263u);
    assert(!selected);
    assert(w25q_write_sector_busy() == (result == W25Q_WRITE_BUSY));
    if (result != W25Q_WRITE_BUSY) {
        assert(!write_source);
    }
    return result;
}

static void finish_success(uint32_t address) {
    unsigned polls = 0;
    while (poll_checked() == W25Q_WRITE_BUSY) {
        assert(++polls < 100u);
        assert(write_source == source);
    }
    assert(write_status == W25Q_WRITE_DONE && pending_operation == OP_NONE);
    assert(!memcmp(flash + address, source, sizeof(source)));
    unsigned before = transfers;
    assert(poll_checked() == W25Q_WRITE_DONE && transfers == before);
}

static void test_validation_and_capacity(void) {
    setup();
    assert(!w25q_write_sector_start(0, 0));
    assert(poll_checked() == W25Q_WRITE_ERROR);
    assert(!w25q_write_sector_start(1, source));
    assert(!w25q_write_sector_start(w25q_capacity, source));
    assert(!w25q_write_sector_start(UINT32_MAX, source));
    w25q_capacity = 2048;
    assert(!w25q_write_sector_start(0, source));
    w25q_capacity = 0;
    assert(!w25q_write_sector_start(0, source));
    for (unsigned i = 0; i < 3; ++i) {
        device = (uint8_t)(0x15u + i);
        w25q_ready = 0;
        w25q_init();
        assert(w25q_capacity == (0x00400000u << i));
    }
    device = 0;
    w25q_ready = 0;
    w25q_init();
    assert(!w25q_capacity);
}

static void test_ownership_and_durability(void) {
    setup();
    for (unsigned i = 0; i < sizeof(source); ++i) {
        source[i] = (uint8_t)(i * 17u);
    }
    unsigned before = transfers;
    assert(w25q_write_sector_start(4096, source));
    assert(transfers == before && write_source == source);
    assert(!w25q_write_sector_start(0, other_source));
    assert(!w25q_write_sector_start(1, 0));
    assert(!w25q_write_sector(0, other_source));
    uint8_t byte = 0xAA;
    assert(!w25q_read(0, &byte, 1) && byte == 0xAA && transfers == before);
    assert(poll_checked() == W25Q_WRITE_BUSY && pending_operation == OP_ERASE);
    assert(flash[4096] == 0x3C);
    assert(poll_checked() == W25Q_WRITE_BUSY && flash[4096] == 0x3C);
    assert(poll_checked() == W25Q_WRITE_BUSY && flash[4096] == 0x3C);
    assert(poll_checked() == W25Q_WRITE_BUSY && pending_operation == OP_PROGRAM);
    assert(flash[4096] == 0xFF);

    // A transport reset must leave this source owned while the job drains.
    for (unsigned i = 0; i < 2; ++i) {
        w25q_init();
        assert(!w25q_write_sector_start(0, other_source));
        assert(write_source == source && w25q_write_sector_busy());
        assert(poll_checked() == W25Q_WRITE_BUSY);
    }
    assert(flash[4096] == 0xFF);
    finish_success(4096);
    assert(erase_starts == 1 && program_starts == 16 && pending_length == 256);
    assert(flash[0] == 0x3C && flash[8192] == 0x3C);
    assert(w25q_read(4096, &byte, 1) && byte == source[0]);
    memset(source, 0xFF, sizeof(source));
    assert(w25q_write_sector_start(0, source));
    finish_success(0);
}

static void test_empty_pages_and_wrapper(void) {
    setup();
    assert(w25q_write_sector_start(0, source));
    finish_success(0);
    assert(erase_starts == 1 && program_starts == 0);
    source[0] = 0;
    source[4095] = 0x7E;
    uint32_t last_sector = w25q_capacity - sizeof(source);
    assert(w25q_write_sector_start(last_sector, source));
    finish_success(last_sector);
    assert(erase_starts == 2 && program_starts == 2);
    assert(w25q_write_sector(4096, source));
    assert(write_status == W25Q_WRITE_DONE && !w25q_write_sector_busy());
    assert(!memcmp(flash + 4096, source, sizeof(source)));
}

static void test_initial_busy_and_deadlines(void) {
    setup();
    initially_busy = 1;
    cycles = UINT32_MAX - 100u;
    assert(w25q_write_sector_start(0, source));
    uint32_t start = write_wait_start;
    for (unsigned i = 0; i < 100; ++i) {
        assert(poll_checked() == W25Q_WRITE_BUSY);
    }
    assert(!erase_starts && !program_starts);
    cycles = start + W25Q_ERASE_TIMEOUT_CYCLES - 1u;
    assert(poll_checked() == W25Q_WRITE_BUSY);
    cycles++;
    assert(poll_checked() == W25Q_WRITE_ERROR);
    uint8_t byte = 0xA5;
    unsigned before = transfers;
    assert(!w25q_read(0, &byte, 1) && byte == 0xA5);
    assert(transfers - before == 2u);
    initially_busy = 0;
    assert(w25q_read(0, &byte, 1) && byte == 0x3C);
    assert(poll_checked() == W25Q_WRITE_ERROR && !w25q_write_sector_busy());
    assert(w25q_write_sector_start(0, source));
    finish_success(0);

    setup();
    initially_busy = 1;
    assert(w25q_write_sector_start(0, source));
    assert(poll_checked() == W25Q_WRITE_BUSY);
    initially_busy = 0;
    assert(poll_checked() == W25Q_WRITE_BUSY && erase_starts == 1);
    hold_busy = 1;
    cycles = write_wait_start + W25Q_ERASE_TIMEOUT_CYCLES;
    assert(poll_checked() == W25Q_WRITE_ERROR);
    assert(pending_operation == OP_ERASE && !w25q_read(0, &byte, 1));
    hold_busy = 0;
    pending_checks = 0;
    assert(w25q_read(0, &byte, 1) && byte == 0xFF);
    assert(poll_checked() == W25Q_WRITE_ERROR);
    assert(w25q_write_sector_start(4096, other_source));
    assert(poll_checked() == W25Q_WRITE_BUSY && erase_starts == 2);

    setup();
    source[0] = 0x55;
    cycles = UINT32_MAX - 100u;
    assert(w25q_write_sector_start(0, source));
    do {
        assert(poll_checked() == W25Q_WRITE_BUSY);
    } while (!program_starts);
    hold_busy = 1;
    start = write_wait_start;
    cycles = start + W25Q_PAGE_TIMEOUT_CYCLES - 1u;
    assert(poll_checked() == W25Q_WRITE_BUSY && flash[0] == 0xFF);
    cycles++;
    assert(poll_checked() == W25Q_WRITE_ERROR && flash[0] == 0xFF);

    setup();
    initially_busy = 1;
    cycle_step = W25Q_ERASE_TIMEOUT_CYCLES;
    assert(!w25q_write_sector(0, source));
    assert(write_status == W25Q_WRITE_ERROR && !erase_starts);
}

static void test_spi_faults(void) {
    for (enum SpiFault injected = SPI_NO_TXE; injected <= SPI_STUCK_BSY; ++injected) {
        setup();
        assert(w25q_write_sector_start(0, source));
        fault = injected;
        fail_after = transfers;
        assert(poll_checked() == W25Q_WRITE_ERROR);
        assert(!erase_starts && !program_starts);
        unsigned before = transfers;
        assert(poll_checked() == W25Q_WRITE_ERROR && transfers == before);
    }
    // Fail each byte of the status/WREN/erase start, including a command
    // that may already have reached the chip when the SPI BSY wait fails.
    for (enum SpiFault injected = SPI_NO_TXE; injected <= SPI_STUCK_BSY; ++injected) {
        for (unsigned byte = 0; byte < 7; ++byte) {
            setup();
            assert(w25q_write_sector_start(0, source));
            fail_after = transfers + byte;
            fault = injected;
            assert(poll_checked() == W25Q_WRITE_ERROR);
        }
        for (unsigned byte = 0; byte < 263; ++byte) {
            setup();
            source[0] = 0;
            assert(w25q_write_sector_start(0, source));
            assert(poll_checked() == W25Q_WRITE_BUSY);
            pending_checks = 0;
            fail_after = transfers + byte;
            fault = injected;
            assert(poll_checked() == W25Q_WRITE_ERROR);
            uint8_t value = 0xA5;
            if (pending_operation != OP_NONE) {
                fault = SPI_OK;
                hold_busy = 1;
                assert(!w25q_read(0, &value, 1) && value == 0xA5);
            }
        }
    }
}

static void test_read_validation_and_faults(void) {
    setup();
    uint8_t byte = 0xA5;
    unsigned before = transfers;
    assert(!w25q_read(0, 0, 1));
    assert(!w25q_read(w25q_capacity, &byte, 1));
    assert(!w25q_read(UINT32_MAX, &byte, 1));
    assert(transfers == before);
    assert(w25q_read(w25q_capacity - 1u, &byte, 1) && byte == 0x3C);
    for (enum SpiFault injected = SPI_NO_TXE; injected <= SPI_STUCK_BSY; ++injected) {
        // The initial status read, address and data bytes all propagate SPI failure.
        for (unsigned i = 0; i < 7; ++i) {
            setup();
            fail_after = transfers + i;
            fault = injected;
            assert(!w25q_read(0, &byte, 1) && !selected);
        }
    }
}

int main(void) {
    test_validation_and_capacity();
    test_ownership_and_durability();
    test_empty_pages_and_wrapper();
    test_initial_busy_and_deadlines();
    test_spi_faults();
    test_read_validation_and_faults();
    puts("W25Q: bounded polls, deferred durability, ownership, deadlines/wrap, empty pages and SPI faults passed");
    return 0;
}
