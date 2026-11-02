"""Offline checks for recorded navigation evidence; no ROS dependencies."""
import ast
import json
import math


def evidence_json(value, **options):
    """Keep invalid measurements explicit without emitting nonstandard JSON."""
    def clean(item):
        if isinstance(item, float) and not math.isfinite(item):
            return {'nonfinite': str(item)}
        if isinstance(item, dict):
            return {k: clean(v) for k, v in item.items()}
        if isinstance(item, (list, tuple)):
            return [clean(v) for v in item]
        return item
    return json.dumps(clean(value), allow_nan=False, **options)


def rectangle(padding=0.0):
    return [(x, y) for x, y in ((.119 + padding, .1 + padding),
                                (.119 + padding, -.1 - padding),
                                (-.181 - padding, -.1 - padding),
                                (-.181 - padding, .1 + padding))]


def same_polygon(points, expected, tolerance=0.001):
    """Match vertices and cyclic adjacency, allowing either winding and start."""
    try:
        if len(points) != len(expected) or any(
                len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            return False
        for sequence in (expected, expected[::-1]):
            for start in range(len(sequence)):
                if all(math.dist(p, sequence[(start + i) % len(sequence)]) <= tolerance
                       for i, p in enumerate(points)):
                    return True
    except (TypeError, ValueError):
        pass
    return False


def check_costmap(snapshot):
    params = snapshot.get('parameters', {})
    published = snapshot.get('published', {})
    missing, errors = [], []
    for name in ('footprint', 'footprint_padding', 'robot_base_frame'):
        if params.get(name) is None:
            missing.append(name)
    if params.get('footprint') is not None:
        try:
            raw = ast.literal_eval(params['footprint'])
        except (ValueError, SyntaxError, TypeError):
            raw = []
        if not same_polygon(raw, rectangle()):
            errors.append('raw footprint differs from the chassis')
    padding = params.get('footprint_padding')
    if padding is not None and (not isinstance(padding, (int, float))
                                or not math.isclose(padding, .02, abs_tol=1e-9)):
        errors.append('padding differs from 0.02 m')
    if params.get('robot_base_frame') not in (None, 'base_footprint'):
        errors.append('base frame differs from base_footprint')
    if 'points_base' not in published:
        missing.append('published polygon transformed at its stamp')
    elif not same_polygon(published['points_base'], rectangle(.02)):
        errors.append('published padded footprint differs from expected rectangle')
    for name in ('age_s', 'source_age_s'):
        age = published.get(name)
        if not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= 2.0:
            missing.append(f'fresh footprint ({name} <= 2 seconds)')
    return {'status': 'mismatch' if errors else 'missing' if missing else 'confirmed',
            'errors': errors, 'missing': missing,
            'bounds_m': [-.201, .139, -.12, .12],
            'turn_radius_m': math.hypot(.201, .12)}


def points_in_base(points, transform):
    """Inverse planar base pose in the polygon frame, at the polygon stamp."""
    x, y, yaw = transform
    co, si = math.cos(yaw), math.sin(yaw)
    return [((px - x) * co + (py - y) * si,
             -(px - x) * si + (py - y) * co) for px, py in points]


def trial_summary(target, status, localized, truth, recoveries):
    def error(pose):
        if pose is None:
            return None
        return {'xy_m': math.hypot(pose[0] - target[0], pose[1] - target[1]),
                'yaw_rad': abs(math.atan2(math.sin(pose[2] - target[2]),
                                          math.cos(pose[2] - target[2])))}
    return {'navigation_status': {4: 'succeeded', 5: 'canceled', 6: 'aborted',
                                  -1: 'rejected'}.get(status, 'unknown'),
            'localized_error': error(localized), 'true_error': error(truth),
            'recoveries': recoveries, 'collision_free': None}


def check_consumers(parameters):
    controller = parameters.get('controller_server', {})
    behavior = parameters.get('behavior_server', {})
    expected = {
        'controller_server': {
            'FollowPath.plugin': 'nav2_mppi_controller::MPPIController',
            'FollowPath.CostCritic.enabled': True,
            'FollowPath.CostCritic.consider_footprint': True},
        'behavior_server': {
            'costmap_topic': 'local_costmap/costmap_raw',
            'footprint_topic': 'local_costmap/published_footprint',
            'robot_base_frame': 'base_footprint'},
    }
    missing, errors = [], []
    for node, values in expected.items():
        for name, value in values.items():
            actual = parameters.get(node, {}).get(name)
            if actual is None:
                missing.append(f'{node}.{name}')
            elif actual != value:
                errors.append(f'{node}.{name}: expected {value!r}, received {actual!r}')
    critics = controller.get('FollowPath.critics')
    if critics is None:
        missing.append('FollowPath.critics')
    elif 'CostCritic' not in critics:
        errors.append('CostCritic is not active')
    return {'status': 'mismatch' if errors else 'missing' if missing else 'confirmed',
            'missing': missing, 'errors': errors}
