"""Per-turn analysis: odom scale vs real rate, and AMCL heading error added by each turn.

usage: turns.py REF.npz RUNS.npz
"""
import sys
import numpy as np
from common import load_data, odom_at, imu_yaw_at, wrap

D = load_data()
ref = np.load(sys.argv[1])['ref']
runs = np.load(sys.argv[2])
names = list(runs['names'])
ests = runs['ests']
st = D['scan_t'][:, 0]
t = st - st[0]
od = odom_at(D['odom'], st)
# real yaw from the IMU orientation at 10 Hz (finer than per scan)
imu = D['imu']
ti = imu[:, 0]
iyaw = np.unwrap(imu[:, 2])
oyaw = odom_at(D['odom'], ti)[:, 2]
rate = np.gradient(iyaw, ti)
orate = np.gradient(oyaw, ti)
turning = np.abs(orate) > 0.15
# turns: runs of turning samples, merged across gaps < 0.3 s, total >= 0.3 rad odom
segs = []
k = 0
while k < len(ti):
    if turning[k]:
        j = k
        while j + 1 < len(ti) and (turning[j + 1] or (j + 3 < len(ti) and turning[j + 2:j + 4].any())):
            j += 1
        segs.append((k, j))
        k = j + 1
    else:
        k += 1
rows = []
for a, b in segs:
    a0, b1 = max(a - 1, 0), min(b + 1, len(ti) - 1)
    dodo = oyaw[b1] - oyaw[a0]
    dreal = iyaw[b1] - iyaw[a0]
    if abs(dodo) < 0.3:
        continue
    peak = np.abs(rate[a:b + 1]).max()
    # scans just before and just after the turn
    ka = np.searchsorted(st, ti[a0]) - 1
    kb = np.searchsorted(st, ti[b1]) + 1
    if ka < 0 or kb >= len(st):
        continue
    e = {}
    for vi, n in enumerate(names):
        eh = np.degrees(np.abs(wrap(ests[vi, :, :, 2] - ref[None, :, 2])))
        e[n] = (np.median(eh[:, ka]), np.median(eh[:, kb]))
    rows.append(dict(t=t[ka], odo=dodo, real=dreal, ratio=dreal / dodo, peak=peak, err=e))

print(f'{len(rows)} turns')
print(' t(s)  odom(rad) real(rad) ratio  peak real rate  | current: head err before -> after (deg)')
for r in rows:
    b, a = r['err'][names[0]]
    print(f"{r['t']:5.1f} {r['odo']:8.2f} {r['real']:8.2f} {r['ratio']:6.3f} {r['peak']:8.2f}        | {b:5.1f} -> {a:5.1f}")
ratio = np.array([r['ratio'] for r in rows])
peak = np.array([r['peak'] for r in rows])
mag = np.abs([r['real'] for r in rows])
slow = peak < 1.0
print('ratio slow (<1.0 rad/s): median %.3f n=%d   fast: median %.3f n=%d   corr(ratio, peak) %.2f' % (
    np.median(ratio[slow]), slow.sum(), np.median(ratio[~slow]), (~slow).sum(), np.corrcoef(ratio, peak)[0, 1]))
for n in names:
    after = np.array([r['err'][n][1] for r in rows])
    grow = np.array([r['err'][n][1] - r['err'][n][0] for r in rows])
    print(f'{n:40s} after-turn head err median slow {np.median(after[slow]):5.1f} fast {np.median(after[~slow]):5.1f} | '
          f'growth per rad turned: slow {np.median(grow[slow] / mag[slow]):5.1f} fast {np.median(grow[~slow] / mag[~slow]):5.1f} deg/rad')
np.save(sys.argv[2].replace('_runs.npz', '_turns.npy'), rows, allow_pickle=True)
