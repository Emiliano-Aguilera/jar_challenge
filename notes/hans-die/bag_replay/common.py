import numpy as np
from scipy.ndimage import distance_transform_edt

WORK = __file__.rsplit('/', 1)[0]
LASER_X, LASER_Y, LASER_YAW = 0.0435, 0.0001, np.pi


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_map(pgm, origin=(-0.31, -0.265), res=0.05):
    d = open(pgm, 'rb').read()
    parts = d.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3][-w * h:], np.uint8).reshape(h, w)[::-1]  # row 0 = min y
    occ = img < 100            # trinary: 0 occupied, 254 free, 205 unknown
    # distance (m) from each cell centre to the nearest occupied cell centre
    dist = distance_transform_edt(~occ) * res
    return dict(occ=occ, dist=dist, ox=origin[0], oy=origin[1], res=res, w=w, h=h)


def load_data():
    d = np.load(f'{WORK}/data.npz')
    return {k: d[k] for k in d.files}


def odom_at(odom, t):
    """odom pose (x, y, unwrapped yaw) interpolated at times t (header stamps)."""
    ot = odom[:, 0]
    th = np.unwrap(odom[:, 4])
    return np.stack([np.interp(t, ot, odom[:, 2]), np.interp(t, ot, odom[:, 3]),
                     np.interp(t, ot, th)], -1)


def imu_yaw_at(imu, t):
    return np.interp(t, imu[:, 0], np.unwrap(imu[:, 2]))


def ray_angles_base(n=1080, amin=-np.pi, ainc=0.005823156330734491):
    return wrap(LASER_YAW + amin + ainc * np.arange(n))
