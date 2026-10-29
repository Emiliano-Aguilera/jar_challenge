"""Replay the week-10 bag with AMCL on odom_fused, built by the node's own class (RumboImu).

usage: eval_rumbo_imu.py MAP.pgm REF.npz SEEDS   (run from this folder, after extract.py and reference.py)
/odom and /imu/data go to RumboImu in bag arrival order; AMCL then runs on odom_fused -> base.
Variants with "PR34 fix 5" also need correccion_giro.py (branch localizacion/correccion-giros).
Other heading params: env CFG (list of (name, params)) and VAR (list of (label, name, updates)).
"""
import os
import sys
import numpy as np
from multiprocessing import Pool
from pathlib import Path
PGM, REF, SEEDS = sys.argv[1], sys.argv[2], int(sys.argv[3])
AQUI = Path(__file__).resolve().parent
sys.path[:0] = [str(AQUI), str(AQUI.parents[2] / 'src' / 'equipo_jar')]
import amcl
from common import load_data, wrap, odom_at
from equipo_jar.rumbo_imu import RumboImu
try:
    from equipo_jar.correccion_giro import CorreccionGiro
except ImportError:                 # without PR #34: only variants with 0 updates
    CorreccionGiro = None

D = load_data(); ref = np.load(REF)['ref']
st = D['scan_t'][:, 0]; O = D['odom']; I = D['imu']
BASE = {'fuente_rumbo': 'giroscopo', 'signo_giroscopo': 0, 'imu_vieja_s': 0.5,
        'signo_wz_minimo_rads': 0.2, 'signo_giro_minimo_rad': 0.5, 'signo_correlacion_minima': 0.7}


def fused_by_node(p):
    """Feed /odom and /imu/data in arrival order; odom_fused -> base at each /odom stamp."""
    r = RumboImu(dict(BASE, **p))
    ev = [(O[k, 1], 0, k) for k in range(len(O))] + [(I[k, 1], 1, k) for k in range(len(I))]
    ev.sort()
    tf, lag, signo_t = {}, [], None
    for tb, kind, k in ev:
        if kind == 0:
            out = r.con_odom(O[k, 0], O[k, 2], O[k, 3], O[k, 4], O[k, 7])
        else:
            out = r.con_imu(I[k, 0], I[k, 3], I[k, 2])
        for t, x, y, yaw in out:
            tf[t] = (x, y, yaw); lag.append(tb - t)
        if signo_t is None and r.signo:
            signo_t = (tb - O[0, 1], r.signo)
    o2 = O.copy()
    for k in range(len(O)):
        x, y, yaw = tf[O[k, 0]]
        c, s = np.cos(yaw), np.sin(yaw)
        o2[k, 2] = x + c * O[k, 2] - s * O[k, 3]
        o2[k, 3] = y + s * O[k, 2] + c * O[k, 3]
        o2[k, 4] = wrap(yaw + O[k, 4])
    o2[:, 0] = O[:, 0]
    return o2, np.array(lag), signo_t, r.pasos_ruedas


ODOMS, INFO = {'wheels': O}, {}
CFG = eval(os.environ.get('CFG', "[('node gyro', {}), ('node orientation', {'fuente_rumbo': 'orientacion'}), ('node off', {'fuente_rumbo': 'ruedas'})]"))
for name, p in CFG:
    ODOMS[name], lag, signo_t, nr = fused_by_node(p)
    INFO[name] = (lag, signo_t, nr)

od0 = odom_at(O, st)
dod = np.diff(od0, axis=0, prepend=od0[:1])
still = (np.hypot(dod[:, 0], dod[:, 1]) < 0.003) & (np.abs(dod[:, 2]) < 0.005)
post_turn = np.zeros(len(st), bool); turned, armed = 0.0, False
t = st - st[0]
for k in range(1, len(st)):
    turned += abs(dod[k, 2])
    if not still[k]: armed = False; continue
    if not still[k - 1] and turned >= 0.3: armed = True
    if not still[k - 1]: turned = 0.0
    post_turn[k] = armed
post_turn &= t > 16
stop_end = post_turn & ~np.roll(post_turn, -1)

VARIANTS = eval(os.environ.get('VAR', "[('wheels (today)', 'wheels', 0), ('wheels + PR34 fix 5', 'wheels', 5), ('node off (identity)', 'node off', 0), ('node gyro', 'node gyro', 0), ('node gyro + PR34 fix 5', 'node gyro', 5), ('node orientation', 'node orientation', 0)]"))


def run(P, F, seed, Ox, n_upd):
    # Same loop as the prototype (eval_imu.py); odom stamps here are header stamps.
    rng = np.random.default_rng(seed); od = odom_at(Ox, st)
    c = CorreccionGiro if n_upd else None
    c = c and c({'actualizaciones_giro': n_upd, 'giro_minimo_rad': 0.3, 'girando_rads': 0.3,
                        'quieto_lineal_ms': 0.02, 'quieto_angular_rads': 0.05, 'max_correccion_s': 3.0})
    a = amcl.Amcl(P, F, rng); a.init(P['init'], P['init_sxy'], P['init_syaw'])
    n = len(st); est = np.full((n, 3), np.nan)
    rb = [amcl.beams(D['scan_r'][k], P) for k in range(n)]
    a.update(*rb[0]); force = P['init_nomotion']; pf = od[0].copy(); m2o = None; oi = 0; last = -1e9; ped = False
    for k in range(n):
        while oi < len(O) and O[oi, 0] <= st[k]:
            r = O[oi]; oi += 1
            if c is None: continue
            c.con_odom(r[0], r[4], r[5], r[6], r[7])
            if c.pedir_actualizacion() and r[0] - last >= 0.2: ped = True; last = r[0]
        upd = False
        if k > 0:
            dl = od[k] - pf; dl[2] = wrap(dl[2])
            moved = max(abs(dl[0]), abs(dl[1])) > P['update_min_d'] or abs(dl[2]) > P['update_min_a']
            if moved or force > 0 or ped:
                a.motion(dl, pf[2]); a.update(*rb[k]); upd = True; ped = False; pf = od[k].copy()
                if force > 0: force -= 1
                if force == 0 and c is not None: c.con_actualizacion_amcl(st[k])
        if upd or k == 0:
            mp, _, _ = amcl.cluster_pose(a.x); th = mp[2] - od[k, 2]; cs, sn = np.cos(th), np.sin(th)
            m2o = np.array([mp[0] - (cs * od[k, 0] - sn * od[k, 1]), mp[1] - (sn * od[k, 0] + cs * od[k, 1]), th])
        cs, sn = np.cos(m2o[2]), np.sin(m2o[2])
        est[k] = [m2o[0] + cs * od[k, 0] - sn * od[k, 1], m2o[1] + sn * od[k, 0] + cs * od[k, 1], m2o[2] + od[k, 2]]
    return est


def job(arg):
    vi, seed = arg; name, oname, nu = VARIANTS[vi]; P = dict(amcl.TEAM)
    return vi, run(P, amcl.Field(PGM, P['laser_likelihood_max_dist']), seed, ODOMS[oname], nu)


if __name__ == '__main__':
    for name, (lag, signo_t, nr) in INFO.items():
        print(f'{name}: TF lag after /odom arrival p50 {np.median(lag) * 1e3:.1f} ms p99 {np.percentile(lag, 99) * 1e3:.1f} ms '
              f'max {lag.max() * 1e3:.1f} ms | sign {signo_t} | turning steps on wheels {nr}')
    # Heading drift of odom_fused vs the reference over the run (no AMCL).
    of = odom_at(ODOMS[CFG[0][0]], st)
    print('odom_fused heading - ref heading: first', np.degrees(wrap(of[20, 2] - ref[20, 2])).round(1),
          'last', np.degrees(wrap(of[-1, 2] - ref[-1, 2])).round(1))
    print('stops after turns', stop_end.sum())
    todas = [(v, s) for v in range(len(VARIANTS)) for s in range(SEEDS)]
    todas = [(v, s) for v, s in todas if VARIANTS[v][2] == 0 or CorreccionGiro]
    with Pool(8) as pool:
        res = pool.map(job, todas)
    valid = t > 1.0
    for vi, (name, *_rest) in enumerate(VARIANTS):
        hs, bs, ha, pa = [], [], [], []
        for v, est in res:
            if v != vi: continue
            h = np.degrees(np.abs(wrap(est[:, 2] - ref[:, 2])))
            bx = est[:, 0] + 1.5 * np.cos(est[:, 2]) - (ref[:, 0] + 1.5 * np.cos(ref[:, 2]))
            by = est[:, 1] + 1.5 * np.sin(est[:, 2]) - (ref[:, 1] + 1.5 * np.sin(ref[:, 2]))
            hs.append(h[stop_end]); bs.append(np.hypot(bx, by)[stop_end]); ha.append(h[valid])
            pa.append(np.hypot(est[valid, 0] - ref[valid, 0], est[valid, 1] - ref[valid, 1]))
        if not hs: continue
        hs, bs, ha, pa = map(np.concatenate, (hs, bs, ha, pa))
        print(f"{name:26s} stop end: heading med {np.median(hs):4.1f} p90 {np.percentile(hs, 90):4.1f} max {hs.max():4.1f}, "
              f"box>0.25 {np.mean(bs > 0.25) * 100:4.1f}% box med {np.median(bs):.2f} | "
              f"all: heading med {np.median(ha):4.1f} p90 {np.percentile(ha, 90):4.1f} max {ha.max():4.1f}, "
              f"pos med {np.median(pa):.3f} p90 {np.percentile(pa, 90):.3f}")
