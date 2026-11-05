"""Where the robot's rectangle touched the /map walls, from the AMCL poses in our log.txt.

Reads the `POSE x y yaw` lines that registro.py writes, puts the chassis rectangle (0.119 m
ahead, 0.181 m behind, 0.100 m to each side of base_footprint) on the map and measures the
clearance to the nearest wall cell. Clearance < 0 = the rectangle overlaps a wall cell.
AMCL is ~3-5 cm off, so read single poses with care; long runs of overlap are real contact.
For each new contact it prints the last mission/behavior command before it.

Usage: python3 contacts_on_map.py MAP.yaml LOG.txt [LOG2.txt ...]
"""
import bisect
import math
import re
import sys

import numpy as np

sys.path.insert(0, __import__('os').path.dirname(__file__))
from mapfile import distance_field, load_map  # noqa: E402

occ, res, ox, oy = load_map(sys.argv[1])
dist = distance_field(occ, res)
FRONT, BACK, HALF = 0.119, 0.181, 0.100
edge = ([(x, y) for x in np.linspace(-BACK, FRONT, 16) for y in (-HALF, HALF)]
        + [(x, y) for y in np.linspace(-HALF, HALF, 10) for x in (-BACK, FRONT)])


def clearance(x, y, yaw):
    c, s, best = math.cos(yaw), math.sin(yaw), 9.9
    for px, py in edge:
        i, j = int((x + c * px - s * py - ox) / res), int((y + s * px + c * py - oy) / res)
        if not (0 <= i < occ.shape[1] and 0 <= j < occ.shape[0]):
            return -1.0
        best = min(best, -res / 2 if occ[j, i] else dist[j, i] - res / 2)
    return best


COMMANDS = ('Giro en el lugar', 'Viaje al punto', 'Vista extra', 'Voy a mirar', 'Running spin',
            'Running backup', 'Running wait', 'Vuelta')
for log in sys.argv[2:]:
    poses, cmds = [], []
    for line in open(log, errors='replace'):
        m = re.match(r'^\S+\s+(\S+) POSE (\S+) (\S+) (\S+)', line)
        if m and m.group(1) != '-':
            poses.append(tuple(map(float, m.groups())))
            continue
        m = re.match(r'^\S+\s+(\S+) \w+\s+\[(?:mision|behavior_server)\] (.*)$', line.rstrip())
        if m and m.group(1) != '-' and m.group(2).startswith(COMMANDS):
            cmds.append((float(m.group(1)), m.group(2)[:50]))
    cl = [(t, x, y, clearance(x, y, yaw)) for t, x, y, yaw in poses]
    v = np.array([c[3] for c in cl])
    print(f'== {log}: {len(cl)} poses, median clearance {np.median(v):.3f} m, '
          f'overlap {np.mean(v < 0) * 100:.0f} % of poses, < 2 cm {np.mean(v < 0.02) * 100:.0f} %')
    times = [c[0] for c in cmds]
    free = True
    for t, x, y, c in cl:
        if c < 0 and free:
            k = bisect.bisect_right(times, t) - 1
            last = f'{t - cmds[k][0]:.1f} s after "{cmds[k][1]}"' if k >= 0 else ''
            print(f'   contact t={t:6.1f} at ({x:.2f}, {y:.2f}) {last}')
        free = c >= 0.02
