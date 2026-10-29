"""Dump /scan, /odom, /imu/data, /tf_static, /cmd_vel from the bag to an npz."""
import sys
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag, out = Path(sys.argv[1]), sys.argv[2]


def yaw(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


scan_t, scan_r, odom, imu, cmd, tfs = [], [], [], [], [], []
meta = {}
with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
    conns = [c for c in r.connections if c.topic in
             ('/scan', '/odom', '/imu/data', '/tf_static', '/cmd_vel', '/tf')]
    for c, t, raw in r.messages(connections=conns):
        m = r.deserialize(raw, c.msgtype)
        tb = t * 1e-9
        if c.topic == '/scan':
            scan_t.append((st(m.header), tb))
            scan_r.append(np.asarray(m.ranges, np.float32))
            if not meta:
                meta = dict(frame=m.header.frame_id, amin=m.angle_min, amax=m.angle_max,
                            ainc=m.angle_increment, tinc=m.time_increment, stime=m.scan_time,
                            rmin=m.range_min, rmax=m.range_max)
        elif c.topic == '/odom':
            p, v = m.pose.pose, m.twist.twist
            odom.append((st(m.header), tb, p.position.x, p.position.y, yaw(p.orientation),
                         v.linear.x, v.linear.y, v.angular.z))
            meta.setdefault('odom_frames', (m.header.frame_id, m.child_frame_id))
        elif c.topic == '/imu/data':
            imu.append((st(m.header), tb, yaw(m.orientation), m.angular_velocity.z))
        elif c.topic == '/cmd_vel':
            cmd.append((tb, m.linear.x, m.linear.y, m.angular.z))
        elif c.topic == '/tf_static':
            for tr in m.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                tfs.append(f'{tr.header.frame_id} -> {tr.child_frame_id}: '
                           f'{p.x:.4f} {p.y:.4f} {p.z:.4f} yaw {yaw(q):.4f}')
        elif c.topic == '/tf' and 'tfdyn' not in meta:
            meta['tfdyn'] = [(tr.header.frame_id, tr.child_frame_id) for tr in m.transforms]

print(meta)
print('\n'.join(sorted(set(tfs))))
np.savez(out, scan_t=np.array(scan_t), scan_r=np.array(scan_r), odom=np.array(odom),
         imu=np.array(imu), cmd=np.array(cmd), meta=np.array(repr(meta)), tfs=np.array(tfs))
print(len(scan_t), len(odom), len(imu), len(cmd))
