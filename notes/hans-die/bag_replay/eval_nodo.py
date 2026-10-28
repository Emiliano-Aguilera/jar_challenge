"""Replay the week-10 bag with the localization node's real heading-fix logic (CorreccionGiro).

usage: eval_nodo.py MAP.pgm REF.npz SEEDS   (run from this folder, after extract.py and reference.py)
Forced updates follow the node: a no-motion request every 0.2 s tick while
pedir_actualizacion() is true; AMCL runs it on the next scan.
"""
import sys, json
import numpy as np
from multiprocessing import Pool
from pathlib import Path
PGM, REF, SEEDS = sys.argv[1], sys.argv[2], int(sys.argv[3])
AQUI = Path(__file__).resolve().parent
sys.path[:0] = [str(AQUI), str(AQUI.parents[2] / 'src' / 'equipo_jar')]
import amcl
from common import load_data, wrap, odom_at
from equipo_jar.correccion_giro import CorreccionGiro

D = load_data(); ref = np.load(REF)['ref']
st = D['scan_t'][:, 0]; t = st - st[0]; od = odom_at(D['odom'], st)
O = D['odom']
dod = np.diff(od, axis=0, prepend=od[:1])
still = (np.hypot(dod[:, 0], dod[:, 1]) < 0.003) & (np.abs(dod[:, 2]) < 0.005)
post_turn = np.zeros(len(st), bool); turned, armed = 0.0, False
for k in range(1, len(st)):
    turned += abs(dod[k, 2])
    if not still[k]: armed = False; continue
    if not still[k - 1] and turned >= 0.3: armed = True
    if not still[k - 1]: turned = 0.0
    post_turn[k] = armed
post_turn &= t > 16
stop_end = post_turn & ~np.roll(post_turn, -1)

VARIANTS = [('today a1 0.2, off', {}, 0), ('node: 3 updates', {}, 3), ('node: 5 updates', {}, 5), ('node: 10 updates', {}, 10),
            ('node: a1 0.5 + 5 updates', dict(alpha1=0.5), 5), ('node: a1 0.8 + 10 updates', dict(alpha1=0.8), 10)]

def run(P, F, seed, n_upd):
    rng = np.random.default_rng(seed)
    c = CorreccionGiro({'actualizaciones_giro': n_upd, 'giro_minimo_rad': 0.3, 'girando_rads': 0.3, 'quieto_lineal_ms': 0.02,
                        'quieto_angular_rads': 0.05, 'max_correccion_s': 3.0})
    a = amcl.Amcl(P, F, rng); a.init(P['init'], P['init_sxy'], P['init_syaw'])
    n = len(st); est = np.full((n, 3), np.nan); upd = np.zeros(n, int)
    rb = [amcl.beams(D['scan_r'][k], P) for k in range(n)]
    a.update(*rb[0]); force = P['init_nomotion']; pf = od[0].copy(); m2o = None
    oi = 0; ultimo_pedido = -1e9; pedido = False; doubt = np.zeros(n, bool)
    for k in range(n):
        # odom messages (10 Hz, header stamp col 1, twist cols 5-7) up to this scan
        while oi < len(O) and O[oi, 1] <= st[k]:
            r = O[oi]; c.con_odom(r[1], r[4], r[5], r[6], r[7]); oi += 1
            # node tick every 0.2 s: one request while it asks
            if c.pedir_actualizacion() and r[1] - ultimo_pedido >= 0.2:
                pedido = True; ultimo_pedido = r[1]
        if k > 0:
            delta = od[k] - pf; delta[2] = wrap(delta[2])
            moved = max(abs(delta[0]), abs(delta[1])) > P['update_min_d'] or abs(delta[2]) > P['update_min_a']
            if moved or force > 0 or pedido:
                a.motion(delta, pf[2]); a.update(*rb[k]); upd[k] = 1 if moved else 2
                if force > 0: force -= 1
                pedido = False; pf = od[k].copy()
                if force == 0: c.con_actualizacion_amcl(st[k])
        doubt[k] = c.en_duda
        if upd[k] or k == 0:
            mp, _, _ = amcl.cluster_pose(a.x); th = mp[2] - od[k, 2]; cs, sn = np.cos(th), np.sin(th)
            m2o = np.array([mp[0] - (cs * od[k, 0] - sn * od[k, 1]), mp[1] - (sn * od[k, 0] + cs * od[k, 1]), th])
        cs, sn = np.cos(m2o[2]), np.sin(m2o[2])
        est[k] = [m2o[0] + cs * od[k, 0] - sn * od[k, 1], m2o[1] + sn * od[k, 0] + cs * od[k, 1], m2o[2] + od[k, 2]]
    return est, upd, doubt, c.correcciones, c.vencidas

def job(arg):
    vi, seed = arg; name, pp, nu = VARIANTS[vi]; P = dict(amcl.TEAM, **pp)
    return (vi, seed) + run(P, amcl.Field(PGM, P['laser_likelihood_max_dist']), seed, nu)

if __name__ == '__main__':
    with Pool(8) as pool:
        res = pool.map(job, [(v, s) for v in range(len(VARIANTS)) for s in range(SEEDS)])
    print('stops after turns', stop_end.sum())
    for vi, (name, _, _) in enumerate(VARIANTS):
        eh, eb, dsec, fixes, venc, flag_at_stop, ehf, ebf = [], [], [], [], [], [], [], []
        for (v, s, est, upd, doubt, nc, nv) in res:
            if v != vi: continue
            h = np.degrees(np.abs(wrap(est[:, 2] - ref[:, 2])))
            bx = est[:, 0] + 1.5 * np.cos(est[:, 2]) - (ref[:, 0] + 1.5 * np.cos(ref[:, 2]))
            by = est[:, 1] + 1.5 * np.sin(est[:, 2]) - (ref[:, 1] + 1.5 * np.sin(ref[:, 2]))
            eh.append(h[stop_end]); eb.append(np.hypot(bx, by)[stop_end])
            dsec.append(np.sum(np.diff(st)[doubt[1:]])); fixes.append(nc); venc.append(nv)
            flag_at_stop.append(np.mean(~doubt[stop_end]) * 100)
            fin = np.where(doubt[:-1] & ~doubt[1:])[0] + 1      # scans where the flag turns true
            ehf.append(h[fin]); ebf.append(np.hypot(bx, by)[fin])
        eh, eb = np.concatenate(eh), np.concatenate(eb)
        ehf, ebf = np.concatenate(ehf), np.concatenate(ebf)
        if len(ehf):
            print(f"{'':28s} when the flag turns true: heading med {np.median(ehf):5.1f} p90 {np.percentile(ehf, 90):5.1f} | box >0.25 m {np.mean(ebf > 0.25) * 100:5.1f}%")
        print(f"{name:28s} heading at stops med {np.median(eh):5.1f} p90 {np.percentile(eh, 90):5.1f} | "
              f"box >0.25 m {np.mean(eb > 0.25) * 100:5.1f}% | flag false {np.mean(dsec):5.1f} s of {st[-1]-st[0]:.0f} s, "
              f"fixes {np.mean(fixes):.0f} (timed out {np.mean(venc):.0f}) | flag true at stop end {np.mean(flag_at_stop):.0f}%")
