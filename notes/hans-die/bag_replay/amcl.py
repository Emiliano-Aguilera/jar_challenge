"""Offline replay of Nav2 Humble AMCL (OmniMotionModel + likelihood_field, pf.c resampling)."""
import numpy as np
from common import load_map, load_data, odom_at, ray_angles_base, wrap, LASER_X, LASER_Y

TEAM = dict(alpha1=0.2, alpha2=0.2, alpha3=0.2, alpha4=0.01, alpha5=0.2,
            laser_min_range=0.10, laser_max_range=12.0, max_beams=120,
            laser_likelihood_max_dist=2.0, sigma_hit=0.1, z_hit=0.5, z_rand=0.5,
            min_particles=500, max_particles=3000, pf_err=0.05, pf_z=0.99,
            update_min_d=0.05, update_min_a=0.1,
            init=(0.0, 0.0, 0.0), init_sxy=0.20, init_syaw=0.30, init_nomotion=10)
KD = np.array([0.5, 0.5, np.deg2rad(10)])


class Field:
    """map_t of amcl: occ_dist per cell (brushfire capped at max_occ_dist), lookup as MAP_GXWX."""

    def __init__(self, pgm, max_dist):
        m = load_map(pgm)
        self.d = np.minimum(m['dist'], max_dist)
        self.ox, self.oy, self.res, self.w, self.h = m['ox'], m['oy'], m['res'], m['w'], m['h']
        self.max = max_dist

    def __call__(self, x, y):
        # nav2 map.h: floor((x - origin_x)/scale + 0.5) + size_x/2 with origin_x moved to the
        # map centre -> floor((x - ox)/res + 0.5): half a cell off, as in the real node
        i = np.floor((x - self.ox) / self.res + 0.5).astype(np.int64)
        j = np.floor((y - self.oy) / self.res + 0.5).astype(np.int64)
        ok = (i >= 0) & (j >= 0) & (i < self.w) & (j < self.h)
        return np.where(ok, self.d[np.clip(j, 0, self.h - 1), np.clip(i, 0, self.w - 1)], self.max)


def resample_limit(k, P):
    k = np.asarray(k, float)
    b = 2 / (9 * np.maximum(k - 1, 1e-9))
    x = 1 - b + np.sqrt(b) * P['pf_z']
    n = np.ceil((k - 1) / (2 * P['pf_err']) * x ** 3)
    n = np.clip(n, P['min_particles'], P['max_particles'])
    return np.where(k <= 1, P['max_particles'], n)


def cluster_pose(pose):
    """pf_cluster_stats + getMaxWeightHyps: kd leaves, 26-connected, max-weight cluster mean."""
    keys = np.floor(pose / KD).astype(np.int64)
    uk, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    lut = {tuple(k): i for i, k in enumerate(uk)}
    label = -np.ones(len(uk), int)
    nl = 0
    offs = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    for i in range(len(uk)):
        if label[i] >= 0:
            continue
        label[i] = nl
        stack = [i]
        while stack:
            j = stack.pop()
            kj = uk[j]
            for o in offs:
                q = lut.get((kj[0] + o[0], kj[1] + o[1], kj[2] + o[2]))
                if q is not None and label[q] < 0:
                    label[q] = nl
                    stack.append(q)
        nl += 1
    lab = label[inv]
    cnt = np.bincount(lab, minlength=nl)
    best = np.argmax(cnt)
    sel = lab == best
    p = pose[sel]
    return np.array([p[:, 0].mean(), p[:, 1].mean(),
                     np.arctan2(np.sin(p[:, 2]).mean(), np.cos(p[:, 2]).mean())]), len(uk), nl


class Amcl:
    def __init__(self, P, field, rng):
        self.P, self.f, self.rng = P, field, rng
        self.denom = 2 * P['sigma_hit'] ** 2
        self.zr = P['z_rand'] / P['laser_max_range']

    def init(self, mean, sxy, syaw):
        n = self.P['max_particles']
        self.x = np.column_stack([mean[0] + self.rng.normal(0, sxy, n), mean[1] + self.rng.normal(0, sxy, n),
                                  mean[2] + self.rng.normal(0, syaw, n)])

    def motion(self, delta, old_yaw):
        P, n = self.P, len(self.x)
        dt = np.hypot(delta[0], delta[1])
        dr = delta[2]
        s_tr = np.sqrt(P['alpha3'] * dt ** 2 + P['alpha4'] * dr ** 2)
        s_rot = np.sqrt(P['alpha1'] * dr ** 2 + P['alpha2'] * dt ** 2)
        s_st = np.sqrt(P['alpha4'] * dr ** 2 + P['alpha5'] * dt ** 2)
        bear = wrap(np.arctan2(delta[1], delta[0]) - old_yaw) + self.x[:, 2]
        cb, sb = np.cos(bear), np.sin(bear)
        tr = dt + self.rng.normal(0, 1, n) * s_tr
        rot = dr + self.rng.normal(0, 1, n) * s_rot
        sf = self.rng.normal(0, 1, n) * s_st
        self.x[:, 0] += tr * cb + sf * sb
        self.x[:, 1] += tr * sb - sf * cb
        self.x[:, 2] += rot

    def sensor(self, rng_, bear):
        x = self.x
        c, s = np.cos(x[:, 2]), np.sin(x[:, 2])
        lx = x[:, 0] + c * LASER_X - s * LASER_Y
        ly = x[:, 1] + s * LASER_X + c * LASER_Y
        a = x[:, 2:3] + bear[None, :]
        hx = lx[:, None] + rng_[None, :] * np.cos(a)
        hy = ly[:, None] + rng_[None, :] * np.sin(a)
        z = self.f(hx, hy)
        pz = self.P['z_hit'] * np.exp(-z * z / self.denom) + self.zr
        w = 1.0 + np.sum(pz ** 3, 1)
        return w / w.sum()

    def resample(self, w):
        P = self.P
        c = np.concatenate([[0], np.cumsum(w)])
        mx = P['max_particles']
        idx = np.searchsorted(c, self.rng.random(mx), side='right') - 1
        idx = np.clip(idx, 0, len(w) - 1)
        new = self.x[idx]
        keys = np.floor(new / KD).astype(np.int64)
        kv = keys[:, 0] * 1_000_003 + keys[:, 1] * 1_009 + keys[:, 2]
        _, first = np.unique(kv, return_index=True)
        isfirst = np.zeros(mx, bool)
        isfirst[first] = True
        leaves = np.cumsum(isfirst)
        count = np.arange(1, mx + 1)
        stop = np.nonzero(count > resample_limit(leaves, P))[0]
        n = stop[0] + 1 if len(stop) else mx
        self.x = new[:n].copy()

    def update(self, rng_, bear):
        self.resample(self.sensor(rng_, bear))


def beams(r, P):
    """laser data as amcl builds it: short -> max, then every step-th ray; nan/max skipped."""
    n = len(r)
    step = max(1, (n - 1) // (P['max_beams'] - 1))
    idx = np.arange(0, n, step)
    rr = r[idx].astype(float)
    rr = np.where(rr <= max(0.05, P['laser_min_range']), P['laser_max_range'], rr)
    ok = np.isfinite(rr) & (rr < P['laser_max_range'])
    return rr[ok], ray_angles_base()[idx][ok]


def run(P, field, D, seed, nomotion_after_turn=0, odom_scale=1.0, still_turn_min=0.3):
    """Replay; returns per-scan estimated pose (nan before the filter has a pose) and flags.

    nomotion_after_turn: N forced updates (request_nomotion_update) on the N scans that
        follow the first still scan after >= still_turn_min rad of odom rotation.
    odom_scale: multiply odom deltas (0.8 = oracle, not deployable).
    """
    rng = np.random.default_rng(seed)
    st = D['scan_t'][:, 0]
    od = odom_at(D['odom'], st)
    if odom_scale != 1.0:
        # re-integrate the body-frame steps of /odom (10 Hz) with both scaled
        o = D['odom']
        th = np.unwrap(o[:, 4])
        dx, dy, dth = np.diff(o[:, 2]), np.diff(o[:, 3]), np.diff(th)
        bx = np.cos(th[:-1]) * dx + np.sin(th[:-1]) * dy
        by = -np.sin(th[:-1]) * dx + np.cos(th[:-1]) * dy
        nth = np.concatenate([[0], np.cumsum(dth * odom_scale)])
        nx = np.concatenate([[0], np.cumsum(odom_scale * (np.cos(nth[:-1]) * bx - np.sin(nth[:-1]) * by))])
        ny = np.concatenate([[0], np.cumsum(odom_scale * (np.sin(nth[:-1]) * bx + np.cos(nth[:-1]) * by))])
        o2 = o.copy()
        o2[:, 2], o2[:, 3], o2[:, 4] = nx, ny, wrap(nth)
        od = odom_at(o2, st)
    a = Amcl(P, field, rng)
    a.init(P['init'], P['init_sxy'], P['init_syaw'])
    n = len(st)
    est = np.full((n, 3), np.nan)
    upd = np.zeros(n, int)       # 1 = regular update, 2 = forced (no-motion) update
    npart = np.zeros(n, int)
    pf_odom = od[0].copy()
    force = P['init_nomotion']
    rb = [beams(D['scan_r'][k], P) for k in range(n)]
    a.update(*rb[0])
    upd[0] = 2
    turned = 0.0
    prev_still = True
    # map->odom correction, as amcl publishes it
    m2o = None
    for k in range(n):
        if k > 0:
            dodom = od[k] - od[k - 1]
            still = np.hypot(dodom[0], dodom[1]) < 0.003 and abs(dodom[2]) < 0.005
            turned += abs(dodom[2])
            if nomotion_after_turn and still and not prev_still and turned >= still_turn_min:
                force = max(force, nomotion_after_turn)
                turned = 0.0
            if still and turned < still_turn_min and not prev_still:
                turned = 0.0
            prev_still = still
            delta = od[k] - pf_odom
            delta[2] = wrap(delta[2])
            moved = abs(delta[0]) > P['update_min_d'] or abs(delta[1]) > P['update_min_d'] \
                or abs(delta[2]) > P['update_min_a']
            if moved or force > 0:
                a.motion(delta, pf_odom[2])
                a.update(*rb[k])
                upd[k] = 1 if moved else 2
                if force > 0:
                    force -= 1
                pf_odom = od[k].copy()
        if upd[k]:
            mp, _, _ = cluster_pose(a.x)
            # map->odom so that map pose = m2o ∘ odom(k)
            th = mp[2] - od[k, 2]
            c, s = np.cos(th), np.sin(th)
            m2o = np.array([mp[0] - (c * od[k, 0] - s * od[k, 1]), mp[1] - (s * od[k, 0] + c * od[k, 1]), th])
        npart[k] = len(a.x)
        if m2o is not None:
            c, s = np.cos(m2o[2]), np.sin(m2o[2])
            est[k] = [m2o[0] + c * od[k, 0] - s * od[k, 1], m2o[1] + s * od[k, 0] + c * od[k, 1],
                      m2o[2] + od[k, 2]]
    return est, upd, npart
