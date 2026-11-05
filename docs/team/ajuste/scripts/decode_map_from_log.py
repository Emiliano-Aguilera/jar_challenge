"""Rebuild the /map of a run from a log that printed it as zlib + base64 parts.

Grupo_de_Lie's mission node logs the map it receives:
  [mission]: map: 174x383 @ 0.0199... m, origin (-0.553, -0.572), 3 parts
  [mission]: map[1/3]: <base64> ...
The decoded bytes are the OccupancyGrid data (0 free, 100 wall), row 0 at the bottom.

Usage: python3 decode_map_from_log.py LOG OUT_PREFIX   (writes OUT_PREFIX.pgm and OUT_PREFIX.yaml)
"""
import base64
import os
import re
import sys
import zlib

log, out = sys.argv[1], sys.argv[2]
header, parts = None, {}
for line in open(log, errors='replace'):
    m = re.search(r'map: (\d+)x(\d+) @ (\S+) m, origin \((\S+), (\S+)\), (\d+) parts', line)
    if m:
        header = m.groups()
    m = re.search(r'map\[(\d+)/(\d+)\]: (\S+)', line)
    if m:
        parts[int(m.group(1))] = m.group(3)
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
width, height, res, ox, oy, count = header
width, height, count = int(width), int(height), int(count)
data = zlib.decompress(base64.b64decode(''.join(parts[i] for i in range(1, count + 1))))
assert len(data) == width * height, 'decoded size does not match the header'
rows = [bytes(0 if data[j * width + i] >= 50 else 254 for i in range(width)) for j in range(height - 1, -1, -1)]
open(out + '.pgm', 'wb').write(b'P5\n%d %d\n255\n' % (width, height) + b''.join(rows))
open(out + '.yaml', 'w').write(
    f'image: {os.path.basename(out)}.pgm\nmode: trinary\nresolution: {round(float(res), 4)}\n'
    f'origin: [{ox.rstrip(",")}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print(f'{width}x{height} @ {round(float(res), 4)} m, origin ({ox} {oy}) -> {out}.pgm/.yaml')
