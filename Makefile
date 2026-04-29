PROJECT := f2c23t_hello
VERSION ?= v2026.04.1
BUILD_ROOT ?= build
BUILD ?= $(BUILD_ROOT)
DIST ?= dist
APP_BASE ?= 0x08008000
APP_FLASH_SIZE ?= 0x00038000
FW_STAGE_BASE ?= 0x08040000
HW_TARGET ?= lt-hw4
HW_TARGET_HW40 ?= 0
DMM_UART_BAUD ?= 9600
FPGA_SPI_BR ?= 2
SCOPE_HW_CAPTURE ?= 1
SCOPE_UI_SAFE_STUB ?= 0
SCOPE_ANALOG_CONFIG ?= 1
SCOPE_ATTENUATOR_CONFIG ?= 1

CLANG ?= clang
RUST_LLD := $(shell find $(HOME)/.rustup/toolchains -path '*/bin/rust-lld' | head -1)
LLD_DIR := $(BUILD)/lld

CFLAGS := \
	-target armv7em-none-eabi \
	-mcpu=cortex-m4 \
	-mthumb \
	-ffreestanding \
	-fno-builtin \
	-fdata-sections \
	-ffunction-sections \
	-fno-unwind-tables \
	-fno-asynchronous-unwind-tables \
	-Os \
	-Wall \
	-Wextra \
	-Werror \
	-I src \
	-DAPP_BASE_ADDR=$(APP_BASE)u \
	-DAPP_FLASH_SIZE_BYTES=$(APP_FLASH_SIZE)u \
	-DFW_STAGE_BASE_ADDR=$(FW_STAGE_BASE)u \
	-DHW_TARGET_HW40=$(HW_TARGET_HW40) \
	-DDMM_UART_BAUD=$(DMM_UART_BAUD) \
	-DFPGA_SPI_BR=$(FPGA_SPI_BR) \
	-DSCOPE_HW_CAPTURE=$(SCOPE_HW_CAPTURE) \
	-DSCOPE_UI_SAFE_STUB=$(SCOPE_UI_SAFE_STUB) \
	-DSCOPE_ANALOG_CONFIG=$(SCOPE_ANALOG_CONFIG) \
	-DSCOPE_ATTENUATOR_CONFIG=$(SCOPE_ATTENUATOR_CONFIG)

LDFLAGS := \
	-target armv7em-none-eabi \
	-mcpu=cortex-m4 \
	-mthumb \
	-nostdlib \
	-fuse-ld=lld \
	-B$(LLD_DIR) \
	-Wl,-T,$(BUILD)/linker.ld \
	-Wl,--gc-sections \
	-Wl,-Map,$(BUILD)/$(PROJECT).map

SRCS := src/startup.c src/board.c src/display.c src/font.c src/dmm.c src/settings.c src/fpga.c src/scope.c src/siggen.c src/fw_update.c src/screenshot.c src/w25q.c src/usb_msc.c src/ui.c src/main.c
ifeq ($(HW_TARGET_HW40),1)
SRCS += src/fpga_bitstream_hw4.c
else
SRCS += src/fpga_bitstream.c
endif
OBJS := $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all clean clean-dist release release-lt-hw4 release-hw4

all: $(BUILD)/$(PROJECT).bin

$(LLD_DIR)/ld.lld:
	@test -n "$(RUST_LLD)" || (echo "rust-lld not found; install an ARM linker or Rust toolchain with rust-lld" >&2; exit 1)
	@mkdir -p $(LLD_DIR)
	@ln -sf "$(RUST_LLD)" $(LLD_DIR)/ld.lld

$(BUILD)/linker.ld: linker.ld
	@mkdir -p $(BUILD)
	sed \
		-e 's/ORIGIN = 0x08008000/ORIGIN = $(APP_BASE)/' \
		-e 's/LENGTH = 224K/LENGTH = $(APP_FLASH_SIZE)/' \
		$< > $@

$(BUILD)/%.o: src/%.c
	@mkdir -p $(BUILD)
	$(CLANG) $(CFLAGS) -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJS) $(BUILD)/linker.ld $(LLD_DIR)/ld.lld
	$(CLANG) $(LDFLAGS) $(OBJS) -o $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf tools/elf2bin.py
	python3 tools/elf2bin.py $< $@ $(APP_BASE)

clean:
	rm -rf $(BUILD_ROOT)

clean-dist:
	rm -rf $(DIST)

release: clean-dist
	$(MAKE) release-lt-hw4
	$(MAKE) release-hw4

release-lt-hw4:
	$(MAKE) BUILD=$(BUILD_ROOT)/lt-hw4 APP_BASE=0x08008000 HW_TARGET=lt-hw4 HW_TARGET_HW40=0 all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/lt-hw4/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-08008000.bin

release-hw4:
	$(MAKE) BUILD=$(BUILD_ROOT)/hw4.0 APP_BASE=0x08007000 HW_TARGET=hw4.0 HW_TARGET_HW40=1 all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/hw4.0/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-HW4.0-08007000.bin
