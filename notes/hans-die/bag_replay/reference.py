"""Reference pose for every scan: scan-to-map fit around an IMU+odom prediction.

usage: reference.py MAP.pgm DESKEW(+1|-1|0) OUT.npz
"""
import sys
import numpy as np
from scipy.optimize import minimize
from common import load_map, load_data, odom_at, imu_yaw_at, ray_angles_base, wrap, LASER_X, LASER_Y

M = load_map(sys.argv[1])
DESKEW = int(sys.argv[2])
OUT = sys.argv[3]
D = load_data()
CAP = 0.12           # robust cap (m): box faces and points outside the map count as outliers
ODOM_SCALE = 0.80    # measured real/odom ratio, only used for the prediction


def field(x, y):
    """bilinear distance to the nearest wall; outside the map = CAP."""
    u = (x - M['ox']) / M['res'] - 0.5
    v = (y - M['oy']) / M['res'] - 0.5
    i0, j0 = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - i0, v - j0
    ok = (i0 >= 0) & (j0 >= 0) & (i0 < M['w'] - 1) & (j0 < M['h'] - 1)
    i0c, j0c = np.clip(i0, 0, M['w'] - 2), np.clip(j0, 0, M['h'] - 2)
    d = M['dist']
    val = (d[j0c, i0c] * (1 - fu) * (1 - fv) + d[j0c, i0c + 1] * fu * (1 - fv)
           + d[j0c + 1, i0c] * (1 - fu) * fv + d[j0c + 1, i0c + 1] * fu * fv)
    return np.where(ok, val, CAP)


st = D['scan_t'][:, 0]
R = D['scan_r']
odom, imu = D['odom'], D['imu']
ang = ray_angles_base()
tinc = 0.00012453735689632595
n = len(st)
od = odom_at(odom, st)
iy = imu_yaw_at(imu, st)


def base_points(k):
    r = R[k].astype(float)
    ok = np.isfinite(r) & (r > 0.10) & (r < 12.0)
    idx = np.nonzero(ok)[0]
    a, rr = ang[idx], r[idx]
    px = LASER_X + rr * np.cos(a)
    py = LASER_Y + rr * np.sin(a)
    if DESKEW:
        ti = st[k] + DESKEW * idx * tinc if DESKEW > 0 else st[k] + (idx - 1080) * tinc
        dth = imu_yaw_at(imu, ti) - iy[k]
        o0 = od[k]
        oi = odom_at(odom, ti)
        dxo, dyo = (oi[:, 0] - o0[0]) * ODOM_SCALE, (oi[:, 1] - o0[1]) * ODOM_SCALE
        c, s = np.cos(-o0[2]), np.sin(-o0[2])
        bx, by = c * dxo - s * dyo, s * dxo + c * dyo           # translation in base(stamp)
        cx, sx = np.cos(dth), np.sin(dth)
        px, py = bx + cx * px - sx * py, by + sx * px + cx * py
    return px, py


def cost(p, px, py):
    c, s = np.cos(p[2]), np.sin(p[2])
    d = field(p[0] + c * px - s * py, p[1] + s * px + c * py)
    return np.mean(np.minimum(d, CAP) ** 2)


def grid_search(center, px, py, dxy, dth, sxy, sth):
    sub = slice(None, None, 3)
    best = (np.inf, center)
    for th in center[2] + np.arange(-dth, dth + 1e-9, sth):
        c, s = np.cos(th), np.sin(th)
        wx, wy = c * px[sub] - s * py[sub], s * px[sub] + c * py[sub]
        for x in center[0] + np.arange(-dxy, dxy + 1e-9, sxy):
            ys = center[1] + np.arange(-dxy, dxy + 1e-9, sxy)
            d = field(x + wx[None, :], ys[:, None] + wy[None, :])
            cs = np.mean(np.minimum(d, CAP) ** 2, 1)
            j = np.argmin(cs)
            if cs[j] < best[0]:
                best = (cs[j], np.array([x, ys[j], th]))
    return best[1]


def refine(p0, px, py):
    r = minimize(cost, p0, args=(px, py), method='Nelder-Mead',
                 options=dict(xatol=1e-4, fatol=1e-9, initial_simplex=p0 + np.array(
                     [[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]])))
    return r.x


# global start: robot still for the first 15 s
px, py = base_points(0)
best = (np.inf, None)
START = sys.argv[4] if len(sys.argv) > 4 else 'global'
if START == 'origin':     # hand placement near (0, 0, 0)
    for th in np.arange(-0.5, 0.51, 0.05):
        p = grid_search(np.array([0.0, 0.0, th]), px, py, 0.3, 0.0, 0.025, 1)
        c = cost(p, px, py)
        if c < best[0]:
            best = (c, p)
else:
    for th in np.arange(-np.pi, np.pi, 0.05):
        p = grid_search(np.array([0.6, 0.35, th]), px, py, 0.9, 0.0, 0.05, 1)
        c = cost(p, px, py)
        if c < best[0]:
            best = (c, p)
p = refine(grid_search(best[1], px, py, 0.08, 0.06, 0.01, 0.01), px, py)
print('start pose', p, 'cost', cost(p, px, py))

ref = np.zeros((n, 3))
fit = np.zeros((n, 2))
ref[0] = p
for k in range(n):
    px, py = base_points(k)
    if k > 0:
        prev = ref[k - 1]
        dth = iy[k] - iy[k - 1]
        do = od[k] - od[k - 1]
        c, s = np.cos(-od[k - 1, 2]), np.sin(-od[k - 1, 2])
        bx, by = (c * do[0] - s * do[1]) * ODOM_SCALE, (s * do[0] + c * do[1]) * ODOM_SCALE
        c, s = np.cos(prev[2]), np.sin(prev[2])
        pred = prev + np.array([c * bx - s * by, s * bx + c * by, dth])
        g = grid_search(pred, px, py, 0.10, 0.6, 0.025, 0.02)
        g = grid_search(g, px, py, 0.02, 0.03, 0.005, 0.005)
        ref[k] = refine(g, px, py)
    c, s = np.cos(ref[k, 2]), np.sin(ref[k, 2])
    d = field(ref[k, 0] + c * px - s * py, ref[k, 1] + s * px + c * py)
    fit[k] = [np.mean(d < 0.05), np.sqrt(np.mean(np.minimum(d, CAP) ** 2))]

np.savez(OUT, ref=ref, fit=fit, cost=np.array([0]), imu_yaw=iy, odom=od, t=st)
print('mean rms %.5f' % np.mean(fit[:, 1]))
print('inlier<5cm median %.3f  p10 %.3f   rms median %.4f  p90 %.4f' % (
    np.median(fit[:, 0]), np.percentile(fit[:, 0], 10), np.median(fit[:, 1]), np.percentile(fit[:, 1], 90)))
h_imu = iy - iy[0] + ref[0, 2]
dh = wrap(ref[:, 2] - h_imu)
print('scan-fit heading minus IMU-anchored heading: median %.3f  p90|.| %.3f  max|.| %.3f rad' % (
    np.median(dh), np.percentile(np.abs(dh), 90), np.abs(dh).max()))
