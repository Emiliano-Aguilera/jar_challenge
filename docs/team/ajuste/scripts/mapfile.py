"""Load a ROS map (.yaml + .pgm, trinary) into a numpy array. Shared by the other scripts."""
import os
import re

import numpy as np


def load_map(yaml_path):
    """Return (occupied[j, i] bool array, resolution, origin_x, origin_y). Row j=0 is the bottom (y = origin_y)."""
    text = open(yaml_path).read()
    image = re.search(r'image:\s*(\S+)', text).group(1)
    res = float(re.search(r'resolution:\s*(\S+)', text).group(1))
    origin = [float(v) for v in re.search(r'origin:\s*\[([^\]]+)\]', text).group(1).split(',')]
    raw = open(os.path.join(os.path.dirname(yaml_path), image), 'rb').read()
    fields, i = [], 0
    while len(fields) < 4:                       # P5 header: magic, width, height, maxval
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        start = i
        while not raw[i:i + 1].isspace():
            i += 1
        fields.append(raw[start:i])
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(raw[i + 1:i + 1 + width * height], dtype=np.uint8).reshape(height, width)
    occupied = (pixels < 128)[::-1]              # black = wall; flip so row 0 is the bottom
    return occupied, res, origin[0], origin[1]


def distance_field(occupied, res, max_m=0.5):
    """Distance (m) from each cell center to the nearest occupied cell center, capped at max_m.

    Uses scipy when installed; otherwise a plain numpy search over a disc of max_m.
    """
    try:
        from scipy import ndimage
        return np.minimum(ndimage.distance_transform_edt(~occupied) * res, max_m)
    except ImportError:
        pass
    h, w = occupied.shape
    r = int(max_m / res)
    best = np.full((h, w), max_m)
    padded = np.zeros((h + 2 * r, w + 2 * r), bool)
    padded[r:r + h, r:r + w] = occupied
    for dj in range(-r, r + 1):
        for di in range(-r, r + 1):
            d = res * (di * di + dj * dj) ** 0.5
            if d < max_m:
                hit = padded[r + dj:r + dj + h, r + di:r + di + w]
                best = np.where(hit & (d < best), d, best)
    return best
