"""Run all AMCL variants over several seeds, compare against the reference.

usage: evaluate.py MAP.pgm REF.npz SEEDS OUT_PREFIX
"""
import sys, json
import numpy as np
from multiprocessing import Pool
import amcl
from common import load_data, wrap, odom_at, imu_yaw_at

PGM, REF, SEEDS, OUT = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
BOX = 1.5

VARIANTS = [
    ('current (a1 0.2)', {}, {}),
    ('alpha1 0.3', dict(alpha1=0.3), {}),
    ('alpha1 0.5', dict(alpha1=0.5), {}),
    ('alpha1 0.8', dict(alpha1=0.8), {}),
    ('no-motion x3 after turn', {}, dict(nomotion_after_turn=3)),
    ('no-motion x5 after turn', {}, dict(nomotion_after_turn=5)),
    ('no-motion x10 after turn', {}, dict(nomotion_after_turn=10)),
    ('alpha1 0.3 + no-motion x5', dict(alpha1=0.3), dict(nomotion_after_turn=5)),
    ('alpha1 0.5 + no-motion x5', dict(alpha1=0.5), dict(nomotion_after_turn=5)),
    ('alpha1 0.5 + no-motion x10', dict(alpha1=0.5), dict(nomotion_after_turn=10)),
    ('alpha1 0.8 + no-motion x10', dict(alpha1=0.8), dict(nomotion_after_turn=10)),
    ('extra: update_min_a 0.05', dict(update_min_a=0.05), {}),
    ('extra: alpha1 0.5 + update_min_a 0.05', dict(alpha1=0.5, update_min_a=0.05), {}),
    ('ceiling: odom x0.8 (not deployable)', {}, dict(odom_scale=0.8)),
]

D = load_data()
R = np.load(REF)
ref = R['ref']
st = D['scan_t'][:, 0]
t = st - st[0]
od = odom_at(D['odom'], st)
iy = imu_yaw_at(D['imu'], st)

# still scans that follow a turn (>= 0.3 rad of odom rotation since the previous stop)
dod = np.diff(od, axis=0, prepend=od[:1])
still = (np.hypot(dod[:, 0], dod[:, 1]) < 0.003) & (np.abs(dod[:, 2]) < 0.005)
post_turn = np.zeros(len(st), bool)
turned, armed = 0.0, False
for k in range(1, len(st)):
    turned += abs(dod[k, 2])
    if not still[k]:
        armed = False
        continue
    if not still[k - 1] and turned >= 0.3:
        armed = True
    if not still[k - 1]:
        turned = 0.0
    post_turn[k] = armed
post_turn &= t > 16          # skip the initial stop
# the last still scan of each post-turn stop: when the robot would look and report
stop_end = post_turn & ~np.roll(post_turn, -1)


def errors(est):
    eh = wrap(est[:, 2] - ref[:, 2])
    ep = np.hypot(est[:, 0] - ref[:, 0], est[:, 1] - ref[:, 1])
    bx = est[:, 0] + BOX * np.cos(est[:, 2]) - (ref[:, 0] + BOX * np.cos(ref[:, 2]))
    by = est[:, 1] + BOX * np.sin(est[:, 2]) - (ref[:, 1] + BOX * np.sin(ref[:, 2]))
    return eh, ep, np.hypot(bx, by)


def job(arg):
    vi, seed = arg
    name, pp, kw = VARIANTS[vi]
    P = dict(amcl.TEAM, **pp)
    F = amcl.Field(PGM, P['laser_likelihood_max_dist'])
    est, upd, npart = amcl.run(P, F, D, seed, **kw)
    return vi, seed, est, upd, npart


if __name__ == '__main__':
    with Pool(8) as pool:
        res = pool.map(job, [(v, s) for v in range(len(VARIANTS)) for s in range(SEEDS)])
    ests = np.full((len(VARIANTS), SEEDS, len(st), 3), np.nan)
    upds = np.zeros((len(VARIANTS), SEEDS, len(st)), int)
    for vi, s, est, upd, npart in res:
        ests[vi, s], upds[vi, s] = est, upd
    np.savez(OUT + '_runs.npz', ests=ests, upds=upds, names=np.array([v[0] for v in VARIANTS]),
             post_turn=post_turn, stop_end=stop_end)
    valid = t > 1.0
    rows = []
    for vi, (name, _, _) in enumerate(VARIANTS):
        eh, ep, eb = [], [], []
        for s in range(SEEDS):
            a, b, c = errors(ests[vi, s])
            eh.append(np.abs(a)); ep.append(b); eb.append(c)
        eh, ep, eb = np.degrees(np.array(eh)), np.array(ep), np.array(eb)
        allm = np.broadcast_to(valid, eh.shape)
        pt = np.broadcast_to(stop_end, eh.shape)
        r = dict(name=name,
                 h_med=np.median(eh[allm]), h_p90=np.percentile(eh[allm], 90), h_max=eh[allm].max(),
                 p_med=np.median(ep[allm]), p_p90=np.percentile(ep[allm], 90), p_max=ep[allm].max(),
                 hs_med=np.median(eh[pt]), hs_p90=np.percentile(eh[pt], 90), hs_max=eh[pt].max(),
                 ps_med=np.median(ep[pt]), ps_p90=np.percentile(ep[pt], 90),
                 b_med=np.median(eb[allm]), b_p90=np.percentile(eb[allm], 90), b_max=eb[allm].max(),
                 bs_med=np.median(eb[pt]), bs_p90=np.percentile(eb[pt], 90),
                 bs_over=np.mean(eb[pt] > 0.25) * 100,
                 # worst seed: max heading error over the run, median across seeds
                 h_max_seedmed=np.median(eh[:, valid].max(1)),
                 forced=np.mean((upds[vi] == 2).sum(1)), regular=np.mean((upds[vi] == 1).sum(1)))
        rows.append(r)
        print(f"{name:40s} head med {r['h_med']:5.1f} p90 {r['h_p90']:5.1f} max {r['h_max']:5.1f} | "
              f"pos med {r['p_med']:.3f} p90 {r['p_p90']:.3f} | stop: head p90 {r['hs_p90']:5.1f} "
              f"box med {r['bs_med']:.3f} p90 {r['bs_p90']:.3f} >0.25 {r['bs_over']:4.1f}% | "
              f"upd {r['regular']:.0f}+{r['forced']:.0f}", flush=True)
    json.dump(rows, open(OUT + '_table.json', 'w'), indent=1, default=float)
    print('post-turn still scans', post_turn.sum(), 'stops', stop_end.sum())
