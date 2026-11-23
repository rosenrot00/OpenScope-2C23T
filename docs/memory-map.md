# Internal Flash Layout

The AT32F403A has 1 MiB of internal flash. The application shares it with the
factory bootloader, a staging area used by the in-app updater, and the settings
page.

| Region | `<HW4.0` | `HW4.0` |
| --- | --- | --- |
| Bootloader | `0x08000000`–`0x08008000` | `0x08000000`–`0x08007000` |
| Application slot | `0x08008000`–`0x08040000` (224 KiB) | `0x08007000`–`0x08080000` (484 KiB) |
| Update staging | `0x08040000`–`0x08078000` | `0x08080000`–`0x080F9000` |
| Spare | `0x08078000`–`0x080FF800` | `0x080F9000`–`0x080FF800` |
| Settings page | `0x080FF800`–`0x08100000` | `0x080FF800`–`0x08100000` |

`APP_BASE`, `APP_FLASH_SIZE` and `FW_STAGE_BASE` in the Makefile set this per
target; the build passes them to the code as `APP_BASE_ADDR`,
`APP_FLASH_SIZE_BYTES` and `FW_STAGE_BASE_ADDR`. The staging area is always the
same size as the application slot, because it has to hold a complete image.

## The flash is two banks

Bank 1 covers `0x08000000`–`0x08080000`, bank 2 covers
`0x08080000`–`0x08100000`, and each is programmed through its own register set.
Bank 1 uses the registers at `FLASH_R_BASE`; bank 2 uses a second set 0x40 bytes
further on:

| Register | Bank 1 | Bank 2 |
| --- | --- | --- |
| `UNLOCK` | `FLASH_R_BASE + 0x04` | `FLASH_R_BASE + 0x44` |
| `STS` | `FLASH_R_BASE + 0x0C` | `FLASH_R_BASE + 0x4C` |
| `CTRL` | `FLASH_R_BASE + 0x10` | `FLASH_R_BASE + 0x50` |
| `ADDR` | `FLASH_R_BASE + 0x14` | `FLASH_R_BASE + 0x54` |

**An erase or a program aimed at a bank 2 address through the bank 1 registers
reports success and does nothing.** `EOP` is set, no error flag is raised, and
only reading the target back shows that the flash never changed. Code that
writes internal flash therefore has to pick the register set from the address --
`FLASH_KEYR_OF()` and friends in `src/fw_update.c`, `flash_*_reg()` in
`src/settings.c` -- and read back what it wrote.

Keeping the staging area inside a single bank is the other half of that:
`tools/check_layout.py` fails the build when it straddles `0x08080000`, so a
future layout change cannot bring back an update that stages half an image and
reports success.

## Build-time checks

`tools/check_layout.py` runs as part of the `.bin` rule. It refuses an image
larger than its slot, an application slot that runs into the staging area,
either region covering the settings page or leaving the 1 MiB of flash, and a
staging area that straddles the bank boundary. It also reports how full the slot
is and warns from 90% on:

```
layout: f2c23t_hello.bin 269018 B / 495616 B app slot (54.3%)
layout: app 0x8007000..0x8080000  stage 0x8080000..0x80f9000  bank 2
```

## RAM cost of staging

Staging records what has already reached flash, so that a host rewriting a
sector -- or a filesystem driver writing the directory entry twice -- does not
try to program the same halfword twice, which the flash would reject. One bit
per halfword costs `APP_FLASH_SIZE / 16` bytes of RAM: 30 976 bytes for the
484 KiB slot, against roughly 25 KiB of RAM left over by the application. One
bit per 512-byte sector plus a single sector held in RAM costs 728 bytes, which
is what makes the larger slot possible at all. `tests/fw_update.c` asserts the
budget so it cannot creep back up.

A sector is held in RAM until every one of its bytes has arrived. It counts
as staged only after a single full comparison with flash, including words
that should have been left erased. Rewriting a page also verifies its other
previously staged sectors as they are restored.
