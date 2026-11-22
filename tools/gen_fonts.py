#!/usr/bin/env python3
"""Generate the anti-aliased UI fonts in src/ui_fonts.c.

The glyphs are rendered from Inter (SIL Open Font License 1.1, see
tools/fonts/Inter-LICENSE.txt) and stored as 4-bit alpha, run-length encoded:
  00nnnnnn  n+1 transparent pixels (1..64)
  01nnnnnn  n+1 opaque pixels (1..64)
  1aaaarr   r+1 pixels of alpha a (1..14), r in 0..3
Digits get one shared advance (tabular figures) so live readings do not jitter sideways as they change.

Icons come from Lucide (ISC licence, tools/icons/LUCIDE-LICENSE.txt),
rasterised by tools/render_icons.js into tools/icons/*.png, and are stored
as one more font (font_icons) on private-use code points U+E000 onwards, so
the text renderer draws them too. src/ui_icons.h names them.

Usage:
    node tools/render_icons.js /path/to/lucide-static/icons tools/icons
    python3 tools/gen_fonts.py /path/to/Inter/extras/ttf tools/icons src/ui_icons.h > src/ui_fonts.c
"""

import os
import sys

from PIL import Image, ImageFont

ICONS = ["check_xl", "zap_xl", "alert_sm", "beep_sm", "pause_sm", "plug_sm", "charge_sm", "usb_sm",
         "multimeter_md", "scope_md", "generator_md", "settings_md", "chevron_l_sm", "chevron_r_sm",
         "playpause_sm", "move_sm", "beep_md", "display_md", "start_md", "sleep_md", "export_md",
         "reset_md", "info_md", "storage_sm", "ok_md", "error_md",
         "edge_rise_sm", "edge_fall_sm", "scope_sm", "generator_sm",
         "key_dc", "key_ac", "key_diode_cont", "key_cap"]

ASCII = "".join(chr(c) for c in range(32, 127))
EXTRA = "Ωµ°Δ"  # Ohm, micro, degree, Delta

# name, file, pixel size, characters
FONTS = [
    ("font_huge", "Inter-SemiBold.ttf", 56, "0123456789.-+ OL"),
    ("font_large", "Inter-SemiBold.ttf", 26,
     " 0123456789.-+%" "ABCDEFGHIJKLMNOPQRSTUVWXYZ" "abcdefghijklmnopqrstuvwxyz" + EXTRA),
    ("font_medium", "Inter-SemiBold.ttf", 13, ASCII + EXTRA + "—"),
    ("font_small", "Inter-SemiBold.ttf", 10, ASCII + EXTRA + "—"),
]


def rle(alpha):
    out = bytearray()
    i = 0
    while i < len(alpha):
        a = alpha[i]
        limit = 64 if a in (0, 15) else 4
        n = 1
        while i + n < len(alpha) and alpha[i + n] == a and n < limit:
            n += 1
        if a == 0:
            out.append(n - 1)
        elif a == 15:
            out.append(0x40 | (n - 1))
        else:
            out.append(0x80 | (a << 2) | (n - 1))
        i += n
    return out


def build(name, path, size, chars):
    font = ImageFont.truetype(path, size)
    ascent, descent = font.getmetrics()
    digit_adv = max(round(font.getlength(d)) for d in "0123456789")
    glyphs = []
    data = bytearray()
    for ch in sorted(set(chars), key=ord):
        adv = round(font.getlength(ch))
        shift = 0
        if ch.isdigit() or (name == "font_huge" and ch == " "):
            shift = (digit_adv - adv) // 2
            adv = digit_adv
        if ch == " ":
            glyphs.append((ord(ch), 0, 0, 0, 0, adv, len(data)))
            continue
        mask, (ox, oy) = font.getmask2(ch, mode="L")
        w, h = mask.size
        px = [mask.getpixel((x, y)) for y in range(h) for x in range(w)]
        # Trim fully transparent rows/columns.
        rows = [y for y in range(h) if any(px[y * w + x] for x in range(w))]
        cols = [x for x in range(w) if any(px[y * w + x] for y in range(h))]
        if not rows:
            glyphs.append((ord(ch), 0, 0, 0, 0, adv, len(data)))
            continue
        y0, y1, x0, x1 = rows[0], rows[-1], cols[0], cols[-1]
        a4 = [(px[y * w + x] + 8) // 17 for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
        off = len(data)
        data += rle(a4)
        glyphs.append((ord(ch), x1 - x0 + 1, y1 - y0 + 1, ox + x0 + shift, oy + y0, adv, off))
    assert len(data) < 65536
    return ascent, descent, glyphs, data


def build_icons(icon_dir):
    glyphs = []
    data = bytearray()
    line = 0
    for i, name in enumerate(ICONS):
        im = Image.open(os.path.join(icon_dir, name + ".png")).convert("RGBA")
        w, h = im.size
        line = max(line, h)
        alpha = [im.getpixel((x, y))[3] for y in range(h) for x in range(w)]
        rows = [y for y in range(h) if any(alpha[y * w + x] for x in range(w))]
        cols = [x for x in range(w) if any(alpha[y * w + x] for y in range(h))]
        y0, y1, x0, x1 = rows[0], rows[-1], cols[0], cols[-1]
        a4 = [(alpha[y * w + x] + 8) // 17 for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
        off = len(data)
        data += rle(a4)
        glyphs.append((0xE000 + i, x1 - x0 + 1, y1 - y0 + 1, x0, y0, w, off))
    return line, glyphs, data


def emit(name, glyphs, data, line_height, ascent):
    print(f"static const uint8_t {name}_data[{len(data)}] = {{")
    for i in range(0, len(data), 20):
        print("    " + ", ".join(f"0x{b:02X}" for b in data[i:i + 20]) + ",")
    print("};\n")
    print(f"static const lcd_glyph_t {name}_glyphs[{len(glyphs)}] = {{")
    for cp, w, h, xo, yo, adv, off in glyphs:
        print(f"    {{0x{cp:04X}, {w}, {h}, {xo}, {yo}, {adv}, {off}}},")
    print("};\n")
    print(f"const lcd_font_t {name} = {{{name}_glyphs, {name}_data, {len(glyphs)}, "
          f"{line_height}, {ascent}}};\n")
    sys.stderr.write(f"{name}: {len(glyphs)} glyphs, {len(data) + len(glyphs) * 10} bytes\n")
    return len(data) + len(glyphs) * 10


def main():
    root = sys.argv[1]
    total = 0
    print("// Generated by tools/gen_fonts.py from Inter (SIL OFL 1.1). Do not edit.")
    print('#include "ui_fonts.h"\n')
    for name, file, size, chars in FONTS:
        ascent, descent, glyphs, data = build(name, os.path.join(root, file), size, chars)
        total += emit(name, glyphs, data, ascent + descent, ascent)
    line, glyphs, data = build_icons(sys.argv[2])
    total += emit("font_icons", glyphs, data, line, line)
    with open(sys.argv[3], "w") as h:
        h.write("#pragma once\n\n// Generated by tools/gen_fonts.py. Icons are drawn as text with font_icons.\n\n")
        for i, name in enumerate(ICONS):
            cp = 0xE000 + i
            utf8 = "".join(f"\\x{b:02X}" for b in chr(cp).encode("utf-8"))
            h.write(f'#define ICON_{name.upper()} "{utf8}"\n')
    sys.stderr.write(f"font bytes: {total}\n")


if __name__ == "__main__":
    main()
