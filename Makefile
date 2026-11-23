PROJECT := f2c23t_hello
VERSION ?= v2026.11.8
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
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=softfp \
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
	-DSCOPE_ATTENUATOR_CONFIG=$(SCOPE_ATTENUATOR_CONFIG) \
	-DFIRMWARE_VERSION_TEXT='"$(VERSION)"'

LDFLAGS := \
	-target armv7em-none-eabi \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=softfp \
	-mthumb \
	-nostdlib \
	-fuse-ld=lld \
	-B$(LLD_DIR) \
	-Wl,-T,$(BUILD)/linker.ld \
	-Wl,--gc-sections \
	-Wl,-Map,$(BUILD)/$(PROJECT).map

SRCS := src/startup.c src/board.c src/display.c src/font.c src/dmm.c src/settings.c src/fpga.c src/scope.c src/siggen.c src/fw_update.c src/screenshot.c src/w25q.c src/usb_msc.c src/bootloader.c src/ui.c src/main.c src/fft.c src/arb_csv.c src/gfx.c src/ui_fonts.c
ifeq ($(HW_TARGET_HW40),1)
SRCS += src/fpga_bitstream_hw4.c
else
SRCS += src/fpga_bitstream.c
endif
OBJS := $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))

HOST_TEST_FLAGS := -std=c11 -O2 -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fsanitize=address,undefined
ifeq ($(shell uname -s),Darwin)
HOST_TEST_LINK := -Wl,-dead_strip
else
HOST_TEST_LINK := -Wl,--gc-sections -lm
endif
HOST_TEST_BINS := $(BUILD_ROOT)/tests/bootloader-old $(BUILD_ROOT)/tests/bootloader-hw4 \
                  $(BUILD_ROOT)/tests/bootloader_export-old $(BUILD_ROOT)/tests/bootloader_export-hw4 \
                  $(BUILD_ROOT)/tests/dmm-old $(BUILD_ROOT)/tests/dmm-hw4 \
                  $(BUILD_ROOT)/tests/scope_window-old $(BUILD_ROOT)/tests/scope_window-hw4 \
                  $(BUILD_ROOT)/tests/scope_measurements-old $(BUILD_ROOT)/tests/scope_measurements-hw4 \
                  $(BUILD_ROOT)/tests/fft $(BUILD_ROOT)/tests/settings \
                  $(BUILD_ROOT)/tests/generator-old $(BUILD_ROOT)/tests/generator-hw4 \
                  $(BUILD_ROOT)/tests/fpga_timing-old $(BUILD_ROOT)/tests/fpga_timing-hw4 \
                  $(BUILD_ROOT)/tests/ui_render-old $(BUILD_ROOT)/tests/ui_render-hw4 \
                  $(BUILD_ROOT)/tests/gfx \
                  $(BUILD_ROOT)/tests/fw_update-old $(BUILD_ROOT)/tests/fw_update-hw4 \
                  $(BUILD_ROOT)/tests/w25q-old $(BUILD_ROOT)/tests/w25q-hw4

.PHONY: all clean clean-dist release release-lt-hw4 release-hw4 test preview screenshots

all: $(BUILD)/$(PROJECT).bin

test: $(HOST_TEST_BINS)
	$(BUILD_ROOT)/tests/bootloader-old
	$(BUILD_ROOT)/tests/bootloader-hw4
	$(BUILD_ROOT)/tests/bootloader_export-old
	$(BUILD_ROOT)/tests/bootloader_export-hw4
	$(BUILD_ROOT)/tests/dmm-old
	$(BUILD_ROOT)/tests/dmm-hw4
	$(BUILD_ROOT)/tests/scope_window-old
	$(BUILD_ROOT)/tests/scope_window-hw4
	$(BUILD_ROOT)/tests/scope_measurements-old
	$(BUILD_ROOT)/tests/scope_measurements-hw4
	$(BUILD_ROOT)/tests/fft
	$(BUILD_ROOT)/tests/settings
	$(BUILD_ROOT)/tests/generator-old
	$(BUILD_ROOT)/tests/generator-hw4
	$(BUILD_ROOT)/tests/fpga_timing-old
	$(BUILD_ROOT)/tests/fpga_timing-hw4
	$(BUILD_ROOT)/tests/fw_update-old
	$(BUILD_ROOT)/tests/fw_update-hw4
	$(BUILD_ROOT)/tests/ui_render-old
	$(BUILD_ROOT)/tests/ui_render-hw4
	$(BUILD_ROOT)/tests/gfx
	$(BUILD_ROOT)/tests/w25q-old
	$(BUILD_ROOT)/tests/w25q-hw4

$(BUILD_ROOT)/tests/bootloader-old $(BUILD_ROOT)/tests/bootloader-hw4: tests/bootloader.c src/bootloader.c src/bootloader.h src/app_config.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) -DAPP_BASE_ADDR=$(if $(filter %-hw4,$@),0x08007000u,0x08008000u) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/bootloader_export-old $(BUILD_ROOT)/tests/bootloader_export-hw4: tests/bootloader_export.c src/usb_msc.c src/usb_msc.h src/bootloader.h src/w25q.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) -DAPP_BASE_ADDR=$(if $(filter %-hw4,$@),0x08007000u,0x08008000u) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/generator-old $(BUILD_ROOT)/tests/generator-hw4: tests/generator.c src/ui.c src/siggen.c src/siggen.h src/fft.c src/settings.h src/fpga.h src/gfx.c src/ui_fonts.c
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) $< src/siggen.c src/fft.c src/gfx.c src/ui_fonts.c $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/fpga_timing-old $(BUILD_ROOT)/tests/fpga_timing-hw4: tests/fpga_timing.c src/fpga.c src/fpga.h src/hw.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/fw_update-old $(BUILD_ROOT)/tests/fw_update-hw4: tests/fw_update.c src/fw_update.c src/fw_update.h src/app_config.h src/hw.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) $(if $(filter %-hw4,$@),-DAPP_BASE_ADDR=0x08007000u -DAPP_FLASH_SIZE_BYTES=0x00079000u -DFW_STAGE_BASE_ADDR=0x08080000u,) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/dmm-old $(BUILD_ROOT)/tests/dmm-hw4: tests/dmm.c src/dmm.c src/dmm.h src/ui.c src/settings.h src/hw.h Makefile
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) -DEXPECTED_FW_VERSION='"$(VERSION)"' $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/%-old: tests/%.c src/ui.c src/settings.h src/fft.h src/gfx.c src/ui_fonts.c
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=0 $< src/gfx.c src/ui_fonts.c $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/%-hw4: tests/%.c src/ui.c src/settings.h src/fft.h src/gfx.c src/ui_fonts.c
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=1 $< src/gfx.c src/ui_fonts.c $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/fft: tests/fft.c src/fft.c src/fft.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/settings: tests/settings.c src/settings.c src/settings.h src/hw.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) $< $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/ui_render-old $(BUILD_ROOT)/tests/ui_render-hw4: tests/ui_render.c tools/host_display.h src/ui.c src/dmm.c src/display.c src/font.c src/gfx.c src/ui_fonts.c src/fft.c
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) $< src/gfx.c src/ui_fonts.c $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/gfx: tests/gfx.c src/gfx.c src/gfx.h src/ui_fonts.c src/ui_fonts.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) $< src/ui_fonts.c $(HOST_TEST_LINK) -o $@

$(BUILD_ROOT)/tests/w25q-old $(BUILD_ROOT)/tests/w25q-hw4: tests/w25q.c src/w25q.c src/w25q.h src/board.h src/hw.h
	@mkdir -p $(dir $@)
	$(CLANG) $(HOST_TEST_FLAGS) -DHW_TARGET_HW40=$(if $(filter %-hw4,$@),1,0) $< $(HOST_TEST_LINK) -o $@

# Render every screen on the host with the firmware's own drawing code:
# build/preview/*.ppm are the exact 320x240 framebuffers.
preview: tools/screen_preview.c tools/host_display.h src/ui.c src/dmm.c src/display.c src/gfx.c src/ui_fonts.c
	@mkdir -p $(BUILD_ROOT)/preview
	$(CLANG) -std=c11 -O1 -Wall -Wextra -ffunction-sections -fdata-sections -DHW_TARGET_HW40=1 -DSCOPE_UI_SAFE_STUB=0 -DSCREEN_PREVIEW_DIR='"$(BUILD_ROOT)/preview"' $< $(HOST_TEST_LINK) -o $(BUILD_ROOT)/preview/run
	$(BUILD_ROOT)/preview/run

# README screenshots: the preview frames at 2x, pixel-exact (needs Pillow).
screenshots: preview tools/ppm_to_png.py
	rm -f docs/screenshots/*.png
	python3 tools/ppm_to_png.py $(BUILD_ROOT)/preview docs/screenshots

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

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf tools/elf2bin.py tools/check_layout.py
	python3 tools/elf2bin.py $< $@ $(APP_BASE)
	python3 tools/check_layout.py $@ $(APP_BASE) $(APP_FLASH_SIZE) $(FW_STAGE_BASE)

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
	$(MAKE) BUILD=$(BUILD_ROOT)/hw4.0 APP_BASE=0x08007000 APP_FLASH_SIZE=0x00079000 FW_STAGE_BASE=0x08080000 HW_TARGET=hw4.0 HW_TARGET_HW40=1 all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/hw4.0/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-HW4.0-08007000.bin
