#!/usr/bin/env python3
"""Check a built image against the flash layout it was built for.

The in-app updater stages the new image in a second flash region before it
overwrites the application, so three things have to hold at build time:

  * the image fits the application slot;
  * the application slot does not run into the staging area;
  * the staging area does not run into the settings page at 0x080FF800, and
    stays inside the 1 MiB flash.

It also fails when the staging area straddles 0x08080000. The flash is two
independently controlled banks, and a write aimed at a bank 2 address through
the bank 1 registers reports success while doing nothing: an image larger than
the part of the staging area that sits in bank 1 was silently truncated, then
flashed, and the device came up black. Keeping the staging area inside one bank
is what makes that impossible rather than merely unlikely.
"""

import sys
from pathlib import Path

FLASH_BASE = 0x08000000
FLASH_END = 0x08100000
BANK2_BASE = 0x08080000
SETTINGS_BASE = 0x080FF800
WARN_RATIO = 0.90


def fail(message: str) -> int:
    print(f"layout: {message}", file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: check_layout.py image.bin app_base app_size stage_base",
            file=sys.stderr,
        )
        return 2

    image = Path(sys.argv[1])
    app_base = int(sys.argv[2], 0)
    app_size = int(sys.argv[3], 0)
    stage_base = int(sys.argv[4], 0)
    app_end = app_base + app_size
    stage_end = stage_base + app_size
    size = image.stat().st_size

    if app_base < FLASH_BASE or app_end > FLASH_END:
        return fail(f"application slot {app_base:#x}..{app_end:#x} is outside flash")
    if stage_base < FLASH_BASE or stage_end > FLASH_END:
        return fail(f"staging area {stage_base:#x}..{stage_end:#x} is outside flash")
    if app_base < stage_end and stage_base < app_end:
        return fail(
            f"application {app_base:#x}..{app_end:#x} overlaps "
            f"staging {stage_base:#x}..{stage_end:#x}"
        )
    if stage_base < SETTINGS_BASE < stage_end:
        return fail(
            f"staging {stage_base:#x}..{stage_end:#x} covers the settings page "
            f"at {SETTINGS_BASE:#x}"
        )
    if app_base < SETTINGS_BASE < app_end:
        return fail(
            f"application {app_base:#x}..{app_end:#x} covers the settings page "
            f"at {SETTINGS_BASE:#x}"
        )
    if stage_base < BANK2_BASE < stage_end:
        return fail(
            f"staging {stage_base:#x}..{stage_end:#x} straddles the bank "
            f"boundary at {BANK2_BASE:#x}; an image larger than "
            f"{BANK2_BASE - stage_base} bytes would stage into the other bank"
        )
    if size > app_size:
        return fail(
            f"{image.name} is {size} bytes, {size - app_size} over the "
            f"{app_size} byte application slot"
        )

    used = size / app_size
    note = "  <-- getting full" if used >= WARN_RATIO else ""
    print(
        f"layout: {image.name} {size} B / {app_size} B app slot "
        f"({used * 100:.1f}%){note}"
    )
    print(
        f"layout: app {app_base:#x}..{app_end:#x}  stage "
        f"{stage_base:#x}..{stage_end:#x}  "
        f"bank {'2' if stage_base >= BANK2_BASE else '1'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
