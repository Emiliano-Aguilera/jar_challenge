"""Evaluate /jar/observaciones and /jar/candidatas against the practice world's boxes.

    python3 eval_replay.py <map_name> <bag_with_truth.mcap> [<bag_with_outputs.mcap>] [label]

If the second bag is omitted the outputs are read from the first one (the run as recorded).
Ground truth (/ground_truth/odom) and /map come from the first bag. Prints, per red box, the
observation error, and the candidates that mission would report with its two rules:
  normal round: observaciones >= 5 and worst std <= 0.10
  final round:  observaciones >= 1 and worst std <= 0.30 (leftover reports, right before /done)
A candidate farther than 0.5 m from every red box is a FALSE one. Also lists observations
that are far from every red box (drift removed with ground truth).
"""
import math
import sys
from bisect import bisect_left

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WORLDS = {
    'maze_1_6x5': {'red': [(-0.1698, -2.0228), (2.0734, -2.0864)], 'blue': [(2.0594, -1.0174), (3.9811, 0.9197)]},
    'maze_3_6x6': {'red': [(0.0013, 0.9937), (2.9552, -2.0360), (-2.0847, -2.0202)], 'blue': [(0.9563, -1.9086), (-0.0847, -2.9586)]},
    'maze_4_metal_6x6': {'red': [(2.8712, 0.0424), (-0.9851, 0.9609)], 'blue': [(2.9382, -2.0381), (-1.0404, -2.0497)]},
    'laberinto_1': {'red': [(-1.2468, -0.1580), (-1.9000, 1.4500), (2.3000, 1.5040)], 'blue': [(-2.5470, -1.5286)]},
    'maze_2_6x5': {'red': [(1.0308, -1.8959), (2.9884, -0.9984), (-0.9077, 0.9429)], 'blue': [(3.0479, -1.9360), (-0.9657, -2.0783)]},
    'laberinto_simple': {'red': [(0.0, 2.0), (3.5, 4.0)], 'blue': [(3.5, -0.5)]},
}


def read(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, types[topic])))
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Track:
    def __init__(self, msgs):
        self.t = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in msgs]
        self.p = [(m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)) for _, m in msgs]

    def at(self, t):
        i = bisect_left(self.t, t)
        i = min(max(i, 0), len(self.t) - 1)
        if i > 0 and abs(self.t[i - 1] - t) < abs(self.t[i] - t):
            i -= 1
        return self.p[i]


def nearest(p, boxes):
    return min(math.hypot(p[0] - b[0], p[1] - b[1]) for b in boxes) if boxes else float('inf')


def worst_std(cov4):
    c = np.array(cov4).reshape(2, 2)
    return math.sqrt(max(np.linalg.eigvalsh(c)))


def main():
    world = WORLDS[sys.argv[1]]
    truth_bag = sys.argv[2]
    out_bag = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith('-') else truth_bag
    label = sys.argv[4] if len(sys.argv) > 4 else ('as recorded' if out_bag == truth_bag else 'replay')
    red, blue = world['red'], world['blue']

    truth_data = read(truth_bag, ['/ground_truth/odom', '/tf'])
    # A replay records its outputs as /replay/*; a run as recorded has /jar/*.
    probe = rosbag2_py.SequentialReader()
    probe.open(rosbag2_py.StorageOptions(uri=out_bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name for t in probe.get_all_topics_and_types()}
    prefix = '/replay' if '/replay/observaciones' in names else '/jar'
    raw = read(out_bag, [f'{prefix}/observaciones', f'{prefix}/candidatas'])
    outputs = {'/jar/observaciones': raw[f'{prefix}/observaciones'], '/jar/candidatas': raw[f'{prefix}/candidatas']}
    truth = Track(truth_data['/ground_truth/odom'])
    # AMCL map->odom from the recorded /tf, to measure the localization error at each time.
    amcl = []
    for t_rx, tfm in truth_data['/tf']:
        for tr in tfm.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                amcl.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                             tr.transform.translation.x, tr.transform.translation.y))

    obs = outputs['/jar/observaciones']
    print(f"\n==== {sys.argv[1]} | {label} | observations {len(obs)}, candidate messages {len(outputs['/jar/candidatas'])}")
    if obs:
        errs, far = [], []
        for t_rx, m in obs:
            z = (m.pose.pose.position.x, m.pose.pose.position.y)
            e = nearest(z, red)
            errs.append(e)
            if e > 0.5:
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                far.append((t, z, e, nearest(z, blue)))
        errs = np.array(errs)
        print(f"  error to nearest red box (as published, with AMCL): median {np.median(errs):.3f}  p90 {np.percentile(errs, 90):.3f}  max {errs.max():.3f} m")
        print(f"  observations within 0.25 m: {(errs <= 0.25).mean() * 100:.0f} %  within 0.50 m: {(errs <= 0.5).mean() * 100:.0f} %")
        print(f"  observations farther than 0.5 m from every red box: {len(far)}")
        for t, z, e, eb in far[:12]:
            print(f"     t={t:.1f} ({z[0]:.2f}, {z[1]:.2f}) red {e:.2f} m, blue {eb:.2f} m")
        if len(far) > 12:
            print(f"     ... and {len(far) - 12} more")

    cands = outputs['/jar/candidatas']
    if cands:
        _, last = cands[-1]
        print("  final candidates:")
        for c in last.candidatas:
            z = (c.centro.x, c.centro.y)
            e = nearest(z, red)
            s = worst_std(c.covarianza_xy)
            tag = 'FALSE' if e > 0.5 else 'ok'
            normal = c.observaciones >= 5 and s <= 0.10
            final = c.observaciones >= 1 and s <= 0.30
            print(f"    id {c.id}: ({z[0]:.2f}, {z[1]:.2f}) n={c.observaciones} std={s:.3f} red {e:.2f} m [{tag}] "
                  f"reportable normal={normal} final={final}")
        # Any FALSE candidate that ever became reportable by the normal rule, and when.
        seen = {}
        for t_rx, msg in cands:
            for c in msg.candidatas:
                z = (c.centro.x, c.centro.y)
                if nearest(z, red) > 0.5 and c.observaciones >= 5 and worst_std(c.covarianza_xy) <= 0.10 and c.id not in seen:
                    seen[c.id] = (t_rx, z, c.observaciones)
        if seen:
            print("  FALSE candidates that passed mission's normal rule at some point:")
            for i, (t, z, n) in seen.items():
                print(f"    id {i} at t={t:.1f}: ({z[0]:.2f}, {z[1]:.2f}) n={n}")
        else:
            print("  no FALSE candidate ever passed mission's normal rule")


if __name__ == '__main__':
    main()
