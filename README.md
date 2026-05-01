# F2C23T Custom Firmware

Custom firmware for the F2C23T handheld multimeter, oscilloscope, and signal generator.

The goal of this repository is to provide an open, improved firmware base for the device with a cleaner UI, better controls, USB storage support, screenshots, in-app firmware updates, and separate builds for the known hardware revisions.

This firmware is still under active development. Use it at your own risk.

## Screenshots

| | | | |
|---|---|---|---|
| <img src="https://github.com/user-attachments/assets/17989306-4eee-4da2-8fed-e7fec0660791" width="320"/> | <img src="https://github.com/user-attachments/assets/371b7de3-4b3e-49d5-ad8f-349b1c0e128d" width="320"/> | <img src="https://github.com/user-attachments/assets/eeacebf9-699b-4f2b-977f-c0a2f7650e51" width="320"/> | <img src="https://github.com/user-attachments/assets/47e9052f-8960-4eac-9fb4-16ef131bd813" width="320"/> |
## Features

- Multimeter UI with mode selection, hold/relative modes, battery status, and USB charging indication.
- Oscilloscope UI with channel controls, trigger menu, move/cursor/measurement menus, rolling display, and improved display layout.
- Signal generator UI with waveform preview, frequency, duty cycle, amplitude controls, and multiple waveform types.
- USB mass-storage mode while the device is running.
- Firmware update by copying a matching `F2C23T*.bin` file to the exposed USB storage.
- Screenshot capture to the device storage.
- Settings menu for brightness, beep volume, sleep behavior, and startup screen.
- Separate release binaries for old and newer hardware revisions.

## Installation

Download the correct `.bin` file from the release assets:

- Use `F2C23T-v2026.04.1-08008000.bin` if your device did **not** already have firmware `2.1.0` installed.
- Use `F2C23T-v2026.04.1-HW4.0-08007000.bin` if your device already came with, or was already running, firmware `2.1.0`.

To flash the firmware with the built-in bootloader:

1. Turn the device off.
2. Hold the `MENU` button.
3. While holding `MENU`, press the power button.
4. Connect the device to your computer by USB.
5. A USB drive should appear.
6. Copy the selected `.bin` file to that drive.
7. Wait until the copy has finished and the device has flashed the firmware.
8. Restart the device.

The bootloader is not modified by this firmware. If the application firmware does not boot, you should still be able to enter bootloader mode again with `MENU` + power and flash another firmware file. In normal use this means there is no permanent brick risk from flashing the application firmware.

## Hardware Versions

There are at least two hardware variants:

- `<HW4.0`: older hardware, app start at `0x08008000`, old FPGA transport.
- `HW4.0`: newer hardware, app start at `0x08007000`, newer FPGA bitstream and GPIOC parallel FPGA transport.

The older hardware build is the primary tested target at the moment. The newer HW4.0 build exists, but still needs real-device testing to confirm that oscilloscope, signal generator, multimeter, storage, and firmware update behavior all work correctly.

## Build

The default build targets devices older than HW4.0:

```sh
make
```

Release builds create both hardware targets:

```sh
make release
```

Output files:

- `dist/F2C23T-v2026.04.1-08008000.bin`
- `dist/F2C23T-v2026.04.1-HW4.0-08007000.bin`

## Project Status

The firmware already covers the core workflows, but there are certainly still things that can be improved, cleaned up, optimized, or made more accurate. Contributions, testing feedback, and hardware-specific findings are welcome.

