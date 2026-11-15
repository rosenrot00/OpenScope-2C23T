#!/usr/bin/env python3
"""Convert the preview framebuffers (build/preview/*.ppm) to README images.

Each 320x240 frame is scaled 2x with nearest-neighbour sampling, so every
device pixel becomes a crisp 2x2 block: what you see is exactly what the
LCD shows.

Usage: python3 tools/ppm_to_png.py build/preview docs/screenshots
"""

import glob
import os
import sys

from PIL import Image


def main():
    src, dst = sys.argv[1], sys.argv[2]
    os.makedirs(dst, exist_ok=True)
    for path in sorted(glob.glob(os.path.join(src, "*.ppm"))):
        name = os.path.splitext(os.path.basename(path))[0] + ".png"
        im = Image.open(path).convert("RGB")
        im.resize((im.width * 2, im.height * 2), Image.NEAREST).save(os.path.join(dst, name), optimize=True)


if __name__ == "__main__":
    main()
