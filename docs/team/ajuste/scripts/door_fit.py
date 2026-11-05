"""Rough model: can MPPI's footprint cross a doorway in the local costmap, and at which headings?

The local costmap has only LiDAR hits (no static map). Each hit marks its 5 cm cell as lethal;
with observation_persistence the hits of the last K scans are all marked. A heading passes if,
at some lateral offset, a straight crossing keeps the footprint outline off lethal cells (what
MPPI's CostCritic checks with consider_footprint). It ignores MPPI's random sampling, latency
and the motors, so it only says whether the geometry leaves room, and how much.

Usage: python3 door_fit.py [N]   (N random grid offsets/noise draws per case, default 20)
"""

import numpy as np, sys
rng = np.random.default_rng(1)
FRONT, BACK, HALF = 0.119, 0.181, 0.100      # chassis from navegacion.launch.py
T = 0.06                                      # wall thickness (tars_qualy_foto)

def hits(w, sensor, ang_step, sigma):
    """LiDAR hits on two walls y in [-T/2, T/2], x<0 and x>w, from a sensor below."""
    sx, sy = sensor
    out = []
    for a in np.arange(0, np.pi, np.radians(ang_step)) + rng.uniform(0, np.radians(ang_step)):
        dx, dy = np.cos(a), np.sin(a)
        best = None
        # front faces y=-T/2
        t = (-T/2 - sy) / dy
        x = sx + t*dx
        if t > 0 and (x <= 0 or x >= w): best = t
        # jamb end faces x=0 (seen from x>0 side) and x=w (seen from x<w side)
        for xf in (0.0, w):
            if abs(dx) > 1e-9:
                t2 = (xf - sx) / dx
                y = sy + t2*dy
                if t2 > 0 and -T/2 <= y <= T/2 and (best is None or t2 < best): best = t2
        if best is None or best > 3.0: continue
        r = best + rng.normal(0, sigma)
        out.append((sx + r*dx, sy + r*dy))
    return np.array(out)

def outline(pad, n=60):
    f, b, h = FRONT + pad, BACK + pad, HALF + pad
    c = [(f, h), (f, -h), (-b, -h), (-b, h)]
    pts = []
    for (x0, y0), (x1, y1) in zip(c, c[1:] + c[:1]):
        s = np.linspace(0, 1, n, endpoint=False)
        pts.append(np.stack([x0 + s*(x1-x0), y0 + s*(y1-y0)], 1))
    return np.concatenate(pts)

def feasible(w, res, pad, scans, sigma, ang_step=1.0):
    ox, oy = rng.uniform(0, res, 2)                  # grid alignment vs the walls
    pts = np.concatenate([hits(w, (w/2 + rng.normal(0, 0.01), -0.45 + 0.012*k), ang_step, sigma)
                          for k in range(scans)])
    cells = set(map(tuple, np.floor((pts - (ox, oy)) / res).astype(int)))
    lethal = np.zeros((400, 400), bool)
    for i, j in cells:
        if 0 <= i+200 < 400 and 0 <= j+200 < 400: lethal[i+200, j+200] = True
    base = outline(pad)
    ok = []
    for deg in range(-40, 41, 2):
        th = np.radians(90 + deg)                    # long axis along +y = straight through
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        fp = base @ R.T
        good = False
        for off in np.arange(-0.05, 0.051, 0.01):
            free = True
            for y in np.arange(-0.30, 0.301, 0.02):
                p = fp + (w/2 + off, y)
                ij = np.floor((p - (ox, oy)) / res).astype(int) + 200
                if lethal[ij[:, 0], ij[:, 1]].any():
                    free = False; break
            if free: good = True; break
        ok.append(good)
    return sum(ok) * 2                                # degrees of heading that pass

configs = [  # (name, cell size m, local footprint_padding m, scans kept)
    ('run 2: 5 cm cells, padding 0.02, persistence 1 s', 0.05, 0.02, 7),
    ('5 cm cells, padding 0.02, persistence 0.3 s',     0.05, 0.02, 2),
    ('5 cm cells, padding 0.02, no persistence',        0.05, 0.02, 1),
    ('5 cm cells, padding 0.03, persistence 0.3 s',     0.05, 0.03, 2),
    ('5 cm cells, padding 0.01, persistence 1 s',       0.05, 0.01, 7),
    ('2.5 cm cells, padding 0.02, persistence 1 s',     0.025, 0.02, 7),
]
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
for sigma in (0.01, 0.02):
    for w in (0.38, 0.40, 0.44, 0.46):
        print(f'--- doorway {w:.2f} m, LiDAR noise {sigma * 100:.0f} cm')
        for name, res, pad, k in configs:
            r = [feasible(w, res, pad, k, sigma) for _ in range(N)]
            print(f'  {name:48s} headings that fit: median {np.median(r):3.0f} deg in total '
                  f'(fits in {np.mean(np.array(r) > 0) * 100:3.0f} % of draws)', flush=True)
