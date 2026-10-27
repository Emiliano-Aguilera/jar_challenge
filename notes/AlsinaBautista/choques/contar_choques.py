#!/usr/bin/env python3
"""Count the robot's collisions in the simulator.

The practice judge counts no collisions (nobody calls /juez/colision), but in the
competition every touch costs -50. This tool puts the robot's simulator collision
shapes at its real pose (/ground_truth/odom) and checks them against every wall
and box of the .world. Each contact counts once, however long it lasts.

Not team code: it reads /ground_truth/odom, which src/ must never use, so it
lives outside src/.

Live, next to the simulator and the judge (counts from /map on):
    python3 contar_choques.py --world maze_1_6x5 --output choques.json
Offline, from poses in JSON lines ({"t": s, "x": m, "y": m, "yaw": rad}):
    python3 contar_choques.py --world path/to/x.world --models path/to/models --poses poses.jsonl
Check one pose (nearest obstacles and their clearance):
    python3 contar_choques.py --world maze_1_6x5 --at 0 0 0

Only the standard library, so the tests run without ROS.
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

# Robot collision shapes in the simulator, measured from base_footprint.
# Source: AIRclub-UdeSA/yahboom_rosmaster @ jar2026-v1, rosmaster_x3_base.urdf.xacro
# and mecanum_wheel.urdf.xacro. The chassis is a box; each wheel collides as a sphere.
CHASSIS_FRONT_M = 0.119        # base_length 0.300, shifted back by (0.238 - 0.300) / 2
CHASSIS_REAR_M = 0.181
CHASSIS_HALF_WIDTH_M = 0.0693  # (total_width 0.1994 - 2 * wheel_width 0.0304) / 2
CHASSIS_BOTTOM_M = 0.073       # height of the chassis box above the floor
CHASSIS_TOP_M = 0.270
WHEEL_X_M = 0.08               # wheel_xoff
WHEEL_Y_M = 0.0845             # wheel_separation 0.169 / 2
WHEEL_RADIUS_M = 0.0325        # sphere radius; its center is one radius above the floor

CHASSIS = [(CHASSIS_FRONT_M, CHASSIS_HALF_WIDTH_M), (-CHASSIS_REAR_M, CHASSIS_HALF_WIDTH_M),
           (-CHASSIS_REAR_M, -CHASSIS_HALF_WIDTH_M), (CHASSIS_FRONT_M, -CHASSIS_HALF_WIDTH_M)]
WHEELS = [(sx * WHEEL_X_M, sy * WHEEL_Y_M) for sx in (1, -1) for sy in (1, -1)]
# Farthest point of the robot from base_footprint: used to skip far obstacles quickly.
ROBOT_REACH_M = max(max(math.hypot(x, y) for x, y in CHASSIS),
                    max(math.hypot(x, y) for x, y in WHEELS) + WHEEL_RADIUS_M)

FLOOR_Z_M = 0.005              # an obstacle lower than this is flat on the floor
BOX_PREFIX = 'cuadrado_'       # colored boxes (victims and distractors) in the practice worlds


# ---------------------------------------------------------------- 3D poses (SDF)

def rotation(roll, pitch, yaw):
    """Rotation matrix of an SDF pose: roll about x, then pitch about y, then yaw about z."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class Transform:
    """A 3D pose: rotation r (3x3) and translation t."""

    def __init__(self, r=IDENTITY, t=(0.0, 0.0, 0.0)):
        self.r, self.t = r, t

    def rotate(self, p):
        return tuple(sum(self.r[i][k] * p[k] for k in range(3)) for i in range(3))

    def apply(self, p):
        q = self.rotate(p)
        return (q[0] + self.t[0], q[1] + self.t[1], q[2] + self.t[2])

    def __mul__(self, other):
        r = tuple(tuple(sum(self.r[i][k] * other.r[k][j] for k in range(3)) for j in range(3))
                  for i in range(3))
        return Transform(r, self.apply(other.t))


def parse_pose(element):
    """<pose> element (or None) -> Transform. Angles in radians unless degrees="true"."""
    if element is None or not (element.text or '').strip():
        return Transform()
    if element.get('relative_to'):
        raise ValueError(f'pose relative_to="{element.get("relative_to")}" is not supported')
    v = [float(x) for x in element.text.split()]
    if len(v) == 3:
        v += [0.0, 0.0, 0.0]
    if len(v) != 6:
        raise ValueError(f'a pose needs 6 numbers: "{element.text.strip()}"')
    if element.get('degrees') == 'true':
        v[3:] = [math.radians(a) for a in v[3:]]
    return Transform(rotation(*v[3:]), tuple(v[:3]))


# ---------------------------------------------------------------- the world

@dataclass
class Obstacle:
    name: str        # model/link path in the world
    kind: str        # 'box' (a colored box) or 'wall' (anything else)
    polygon: list    # floor projection, convex, counter-clockwise [(x, y), ...]
    z_min: float
    z_max: float
    cx: float = 0.0  # bounding circle, for quick rejection
    cy: float = 0.0
    radius: float = 0.0


def convex_hull(points):
    """Counter-clockwise convex hull of 2D points (monotone chain)."""
    pts = sorted(set((round(x, 9), round(y, 9)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def make_obstacle(name, kind, points3d):
    polygon = convex_hull([(x, y) for x, y, _ in points3d])
    zs = [z for _, _, z in points3d]
    cx = sum(x for x, _ in polygon) / len(polygon)
    cy = sum(y for _, y in polygon) / len(polygon)
    radius = max(math.hypot(x - cx, y - cy) for x, y in polygon)
    return Obstacle(name, kind, polygon, min(zs), max(zs), cx, cy, radius)


def geometry_points(geometry):
    """Points of a collision geometry in its own frame, 'plane', or None if unsupported."""
    if geometry is None:
        return None
    box = geometry.find('box')
    if box is not None:
        sx, sy, sz = (float(v) / 2 for v in box.findtext('size').split())
        return [(x, y, z) for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        r = float(cylinder.findtext('radius'))
        h = float(cylinder.findtext('length')) / 2
        ring = [(r * math.cos(a), r * math.sin(a)) for a in (2 * math.pi * k / 16 for k in range(16))]
        return [(x, y, z) for x, y in ring for z in (-h, h)]
    sphere = geometry.find('sphere')
    if sphere is not None:
        r = float(sphere.findtext('radius'))
        ring = [(r * math.cos(a), r * math.sin(a)) for a in (2 * math.pi * k / 16 for k in range(16))]
        return [(x, y, 0.0) for x, y in ring] + [(0.0, 0.0, -r), (0.0, 0.0, r)]
    if geometry.find('plane') is not None:
        return 'plane'
    return None


class WorldLoader:
    """Reads the collision shapes of an SDF world, resolving model:// includes."""

    def __init__(self, model_dirs):
        self.model_dirs = [Path(d) for d in model_dirs]
        self.cache = {}
        self.obstacles, self.skipped = [], []

    def load(self, world_path):
        world = ET.parse(world_path).getroot().find('world')
        if world is None:
            raise ValueError(f'{world_path}: no <world> element')
        self._children(world, Transform(), '', 0)
        return self.obstacles

    def _model_file(self, uri):
        if not uri.startswith('model://'):
            return None
        name = uri[len('model://'):].strip('/').split('/')[0]
        if name in self.cache:
            return self.cache[name]
        found = None
        for d in self.model_dirs:
            folder = d / name
            # model.config can list one SDF file per version: the last one is the newest.
            files = ['model.sdf']
            if (folder / 'model.config').exists():
                listed = [e.text.strip() for e in ET.parse(folder / 'model.config').getroot().iter('sdf')
                          if e.text and e.text.strip()]
                files = listed[-1:] + files
            existing = [folder / f for f in files if (folder / f).exists()]
            if existing:
                # A model with no <model> (a light such as `sun`) has nothing to collide with.
                found = ET.parse(existing[0]).getroot().find('model')
                found = found if found is not None else ET.Element('model')
                break
        self.cache[name] = found
        return found

    def _children(self, element, tf, prefix, depth):
        if depth > 20:
            raise ValueError('models nested too deep')
        for model in element.findall('model'):
            name = f'{prefix}{model.get("name")}'
            self._model(model, tf * parse_pose(model.find('pose')), name, depth + 1)
        for include in element.findall('include'):
            uri = (include.findtext('uri') or '').strip()
            model = self._model_file(uri)
            if model is None:
                self.skipped.append(f'{prefix}{uri}: model not found')
                continue
            pose = include.find('pose')
            pose = pose if pose is not None else model.find('pose')
            name = f'{prefix}{(include.findtext("name") or model.get("name") or uri).strip()}'
            self._model(model, tf * parse_pose(pose), name, depth + 1)

    def _model(self, model, tf, name, depth):
        for link in model.findall('link'):
            link_name = link.get('name', '')
            tf_link = tf * parse_pose(link.find('pose'))
            for collision in link.findall('collision'):
                label = f'{name}/{link_name}'
                points = geometry_points(collision.find('geometry'))
                if points == 'plane':
                    continue
                if points is None:
                    self.skipped.append(f'{label}: unsupported geometry')
                    continue
                tf_col = tf_link * parse_pose(collision.find('pose'))
                kind = 'box' if link_name.startswith(BOX_PREFIX) else 'wall'
                self.obstacles.append(make_obstacle(label, kind, [tf_col.apply(p) for p in points]))
        self._children(model, tf, f'{name}/', depth)


# ---------------------------------------------------------------- 2D distances

def _point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    s = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy)


def _edges(polygon):
    if len(polygon) == 1:
        return [(polygon[0], polygon[0])]
    return [(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon))]


def _segments_cross(a, b, c, d):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return (side(a, b, c) * side(a, b, d) < 0) and (side(c, d, a) * side(c, d, b) < 0)


def _sat_overlap(p, q):
    """Smallest overlap of two convex polygons along their edge normals, or None if apart."""
    smallest = math.inf
    for polygon in (p, q):
        for a, b in _edges(polygon):
            nx, ny = a[1] - b[1], b[0] - a[0]
            norm = math.hypot(nx, ny)
            if norm == 0:
                continue
            nx, ny = nx / norm, ny / norm
            pp = [x * nx + y * ny for x, y in p]
            qq = [x * nx + y * ny for x, y in q]
            overlap = min(max(pp), max(qq)) - max(min(pp), min(qq))
            if overlap < 0:
                return None
            smallest = min(smallest, overlap)
    return smallest if smallest != math.inf else 0.0


def polygon_distance(p, q):
    """Signed distance between two convex polygons: gap if apart, minus the overlap if not."""
    overlap = _sat_overlap(p, q)
    if overlap is not None:
        return -overlap
    best = math.inf
    for a, b in _edges(p):
        for c, d in _edges(q):
            if _segments_cross(a, b, c, d):
                return 0.0
            best = min(best, _point_segment(a, c, d), _point_segment(b, c, d),
                       _point_segment(c, a, b), _point_segment(d, a, b))
    return best


def point_polygon_distance(point, polygon):
    """Signed distance from a point to a convex counter-clockwise polygon (negative inside)."""
    edge = min(_point_segment(point, a, b) for a, b in _edges(polygon))
    if len(polygon) >= 3 and all(
            (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]) >= 0
            for a, b in _edges(polygon)):
        return -edge
    return edge


def robot_parts(x, y, yaw):
    """Chassis polygon and wheel centers in the world, for a robot at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)

    def place(px, py):
        return (x + c * px - s * py, y + s * px + c * py)
    return [place(*p) for p in CHASSIS], [place(*w) for w in WHEELS]


def clearance(x, y, yaw, obstacle):
    """Gap between the robot and an obstacle, m. Zero or negative means contact."""
    chassis, wheels = robot_parts(x, y, yaw)
    best = math.inf
    if obstacle.z_max > CHASSIS_BOTTOM_M and obstacle.z_min < CHASSIS_TOP_M:
        best = polygon_distance(chassis, obstacle.polygon)
    if obstacle.z_min < 2 * WHEEL_RADIUS_M:
        for w in wheels:
            best = min(best, point_polygon_distance(w, obstacle.polygon) - WHEEL_RADIUS_M)
    return best


# ---------------------------------------------------------------- counting

class CollisionCounter:
    """Turns a stream of poses into collisions (contact episodes) and close calls.

    A collision starts when any part of the robot gets within `tolerance` of an obstacle
    and ends after `gap_s` without contact, so pushing a wall for 20 s counts once. A close
    call is an approach within `near_margin` that never touched anything.
    """

    def __init__(self, obstacles, tolerance=0.005, near_margin=0.03, gap_s=0.5):
        self.obstacles = [o for o in obstacles if o.z_max > FLOOR_Z_M and o.z_min < CHASSIS_TOP_M]
        self.tolerance, self.near_margin, self.gap_s = tolerance, near_margin, gap_s
        self.collisions, self.close_calls = [], []
        self._contact = self._near = None
        self.samples = 0
        self.t_first = self.t_last = None
        self.min_clearance = math.inf

    def nearby(self, x, y, yaw):
        """(clearance, obstacle) for every obstacle within reach, nearest first."""
        reach = ROBOT_REACH_M + max(self.near_margin, self.tolerance)
        found = []
        for o in self.obstacles:
            if math.hypot(o.cx - x, o.cy - y) - o.radius > reach:
                continue
            found.append((clearance(x, y, yaw, o), o))
        return sorted(found, key=lambda item: item[0])

    def update(self, t, x, y, yaw):
        """Add one pose (time in s). Returns the new collision if this pose starts one."""
        self.samples += 1
        self.t_first = t if self.t_first is None else self.t_first
        self.t_last = t
        self._close_stale(t)
        near = [(d, o) for d, o in self.nearby(x, y, yaw) if d <= self.near_margin]
        if not near:
            return None
        d_min = near[0][0]
        self.min_clearance = min(self.min_clearance, d_min)
        touching = [o for d, o in near if d <= self.tolerance]
        started = None
        if touching:
            if self._contact is None:
                self._contact = {'start': t, 'end': t, 'x': x, 'y': y, 'yaw': yaw,
                                 'objects': [], 'kinds': [], 'min_clearance_m': d_min}
                self.collisions.append(self._contact)
                started = self._contact
            event = self._contact
            event['end'] = t
            event['min_clearance_m'] = min(event['min_clearance_m'], d_min)
            for o in touching:
                if o.name not in event['objects']:
                    event['objects'].append(o.name)
                if o.kind not in event['kinds']:
                    event['kinds'].append(o.kind)
        if self._near is None:
            self._near = {'start': t, 'end': t, 'x': x, 'y': y, 'yaw': yaw, 'objects': [],
                          'min_clearance_m': d_min, 'touched': False}
        self._near['end'] = t
        if d_min < self._near['min_clearance_m']:
            self._near.update(x=x, y=y, yaw=yaw, min_clearance_m=d_min)
        if near[0][1].name not in self._near['objects']:
            self._near['objects'].append(near[0][1].name)
        self._near['touched'] = self._near['touched'] or bool(touching)
        return started

    def _close_stale(self, t):
        if self._contact is not None and t - self._contact['end'] > self.gap_s:
            self._contact = None
        if self._near is not None and t - self._near['end'] > self.gap_s:
            self._finish_near()

    def _finish_near(self):
        if not self._near['touched']:
            self.close_calls.append(self._near)
        self._near = None

    def finish(self):
        """Close the open episodes. Call it once, at the end."""
        self._contact = None
        if self._near is not None:
            self._finish_near()

    def summary(self, t0=None):
        t0 = self.t_first if t0 is None else t0

        def event(e):
            out = {'t_s': round(e['start'] - t0, 2), 'duration_s': round(e['end'] - e['start'], 2),
                   'x': round(e['x'], 3), 'y': round(e['y'], 3), 'yaw': round(e['yaw'], 3),
                   'objects': e['objects'], 'min_clearance_m': round(e['min_clearance_m'], 4)}
            if 'kinds' in e:
                out['kinds'] = e['kinds']
            return out

        return {
            'collisions': len(self.collisions),
            'wall_collisions': sum('wall' in e['kinds'] for e in self.collisions),
            'box_collisions': sum('box' in e['kinds'] for e in self.collisions),
            'close_calls': len(self.close_calls),
            'min_clearance_m': None if self.min_clearance == math.inf else round(self.min_clearance, 4),
            'samples': self.samples,
            'duration_s': None if self.t_first is None else round(self.t_last - t0, 2),
            'tolerance_m': self.tolerance, 'near_margin_m': self.near_margin, 'gap_s': self.gap_s,
            'events': [event(e) for e in self.collisions],
            'close_call_events': [event(e) for e in self.close_calls],
        }


# ---------------------------------------------------------------- command line

def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def resolve_world(world, model_dirs):
    """A .world path, or a practice world name looked up in the simulator package."""
    path = Path(world)
    dirs = [Path(d) for d in model_dirs]
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('yahboom_rosmaster_gazebo'))
        for name in (f'{world}_victimas.world', f'{world}.world', world):
            if (share / 'worlds' / name).exists():
                path = share / 'worlds' / name
                break
        else:
            raise FileNotFoundError(f'world not found: {world}')
    dirs.append(path.parent.parent / 'models')
    for variable in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        dirs += [Path(d) for d in os.environ.get(variable, '').split(':') if d]
    return path, [d for d in dirs if d.is_dir()]


def write_summary(summary, output):
    text = json.dumps(summary, indent=1, ensure_ascii=False)
    if output:
        Path(output).write_text(text + '\n', encoding='utf-8')
    else:
        print(text)


def run_offline(counter, poses_file):
    with open(poses_file, encoding='utf-8') as f:
        for line in f:
            if not line.strip():
                continue
            p = json.loads(line)
            yaw = p['yaw'] if 'yaw' in p else yaw_from_quaternion(0, 0, p['qz'], p['qw'])
            counter.update(float(p['t']), float(p['x']), float(p['y']), float(yaw))
    counter.finish()
    return counter.summary()


def run_live(counter, args, world_name):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('contar_choques')
    log = node.get_logger()
    state = {'map': args.all_poses, 't0': None}

    def on_map(_msg):
        if not state['map']:
            log.info('/map received: counting from now')
        state['map'] = True

    def on_odom(msg):
        if not state['map']:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if state['t0'] is None:
            state['t0'] = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        new = counter.update(t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if new is not None:
            log.warning(f'collision {len(counter.collisions)} at t={t - state["t0"]:.1f} s '
                        f'({p.x:.2f}, {p.y:.2f}): {", ".join(new["objects"])}')

    def save():
        summary = counter.summary(state['t0'])
        summary['world'] = world_name
        if args.output:
            write_summary(summary, args.output)
        return summary

    node.create_subscription(Odometry, '/ground_truth/odom', on_odom, qos_profile_sensor_data)
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', on_map, map_qos)
    node.create_timer(5.0, save)       # a killed process still leaves a recent file
    log.info(f'{len(counter.obstacles)} obstacles loaded; waiting for /map and /ground_truth/odom')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        counter.finish()
        summary = save()
        if not args.output:
            write_summary(summary, None)
        print(f'collisions: {summary["collisions"]} (walls {summary["wall_collisions"]}, '
              f'boxes {summary["box_collisions"]}), close calls: {summary["close_calls"]}',
              file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--world', required=True, help='.world path or practice world name')
    ap.add_argument('--models', action='append', default=[], help='extra model folder (repeatable)')
    ap.add_argument('--poses', help='offline: JSON lines with t, x, y, yaw')
    ap.add_argument('--at', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                    help='print the nearest obstacles at one pose and exit')
    ap.add_argument('--output', help='write the JSON summary here (default: print it)')
    ap.add_argument('--tolerance', type=float, default=0.005, help='contact if the gap is at most this, m')
    ap.add_argument('--near-margin', type=float, default=0.03, help='close call if the gap is at most this, m')
    ap.add_argument('--gap', type=float, default=0.5, help='s without contact that end a collision')
    ap.add_argument('--all-poses', action='store_true', help='live: count before /map too')
    args = ap.parse_args(argv)

    world_path, model_dirs = resolve_world(args.world, args.models)
    loader = WorldLoader(model_dirs)
    obstacles = loader.load(world_path)
    for problem in loader.skipped:
        print(f'warning: skipped {problem}', file=sys.stderr)
    counter = CollisionCounter(obstacles, args.tolerance, args.near_margin, args.gap)

    if args.at:
        x, y, yaw = args.at
        print(f'{world_path.name}: {sum(o.kind == "wall" for o in counter.obstacles)} wall shapes, '
              f'{sum(o.kind == "box" for o in counter.obstacles)} boxes')
        nearest = sorted(((clearance(x, y, yaw, o), o) for o in counter.obstacles),
                         key=lambda item: item[0])
        for d, o in nearest[:5]:
            print(f'  {d:+.3f} m  {o.kind:4}  {o.name}')
        return 0
    if args.poses:
        summary = run_offline(counter, args.poses)
        summary['world'] = world_path.name
        write_summary(summary, args.output)
        return 0
    run_live(counter, args, world_path.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
