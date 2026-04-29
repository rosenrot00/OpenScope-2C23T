#!/usr/bin/env python3
import struct
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print("usage: elf2bin.py input.elf output.bin [flash_base]", file=sys.stderr)
        return 2

    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    flash_base = int(sys.argv[3], 0) if len(sys.argv) == 4 else 0x08008000
    data = src.read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise SystemExit("expected 32-bit little-endian ELF")

    e_phoff = struct.unpack_from("<I", data, 28)[0]
    e_phentsize = struct.unpack_from("<H", data, 42)[0]
    e_phnum = struct.unpack_from("<H", data, 44)[0]

    segments = []
    for idx in range(e_phnum):
        off = e_phoff + idx * e_phentsize
        p_type, p_offset, _p_vaddr, p_paddr, p_filesz, _p_memsz, _p_flags, _p_align = struct.unpack_from(
            "<IIIIIIII", data, off
        )
        if p_type != 1 or p_filesz == 0:
            continue
        if not (flash_base <= p_paddr < 0x08100000):
            continue
        segments.append((p_paddr, p_offset, p_filesz))

    if not segments:
        raise SystemExit("no flash sections found")

    base = min(addr for addr, _off, _size in segments)
    end = max(addr + size for addr, _off, size in segments)
    out = bytearray([0xFF]) * (end - base)

    for addr, off, size in segments:
        pos = addr - base
        out[pos : pos + size] = data[off : off + size]

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
