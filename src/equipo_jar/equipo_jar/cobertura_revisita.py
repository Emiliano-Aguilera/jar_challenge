"""Safe revisit hints from actual observations, not a new box-report policy.

A visible box face can have a map-occluded centre. A recent measured view is
therefore useful when a centre-ray inspection search fails. Returning a hint
does not establish that navigation or a new image will succeed.
"""
from dataclasses import dataclass
import math

from equipo_jar.mapa import Grid, Pose, check_cancelled


@dataclass(frozen=True)
class ObservedView:
    """Observer pose and target from the same observation stamp/map revision."""
    pose: Pose
    target_x: float
    target_y: float
    stamp_s: float
    sigma_m: float
    map_revision: int

    def __post_init__(self):
        if not isinstance(self.pose, Pose):
            raise ValueError('observer must be a finite map Pose')
        if not all(math.isfinite(v) for v in (self.target_x, self.target_y, self.stamp_s, self.sigma_m)):
            raise ValueError('observation values must be finite')
        if self.stamp_s < 0 or self.sigma_m <= 0:
            raise ValueError('observation time must be nonnegative and sigma positive')
        if type(self.map_revision) is not int or self.map_revision < 0:
            raise ValueError('map revision must be a nonnegative integer')


@dataclass(frozen=True)
class RevisitHint:
    """A goal near measured evidence; adjustment is not measured visibility."""
    pose: Pose
    evidence: ObservedView

    @property
    def adjustment_m(self):
        return math.hypot(self.pose.x - self.evidence.pose.x, self.pose.y - self.evidence.pose.y)


def safe_observer(grid, pose, radius, threshold):
    """Disk against full occupied squares at the actual continuous XY.

    Testing only the centre of pose's grid cell can overstate clearance by
    half a cell. Unknown cells and touching a wall/map edge remain blocked.
    """
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('clearance must be finite and nonnegative')
    if type(threshold) is not int or not 0 <= threshold < 100:
        raise ValueError('invalid occupancy threshold')
    cell = grid.cell(pose.x, pose.y)
    if cell is None:
        return False
    dx, dy = pose.x - grid.origin.x, pose.y - grid.origin.y
    x = (grid.cos * dx + grid.sin * dy) / grid.resolution
    y = (-grid.sin * dx + grid.cos * dy) / grid.resolution
    clearance = radius / grid.resolution
    if min(x, y, grid.width - x, grid.height - y) <= clearance + 1e-12:
        return False
    span = math.ceil(clearance) + 1
    ci, cj = cell
    for j in range(max(0, cj - span), min(grid.height, cj + span + 1)):
        for i in range(max(0, ci - span), min(grid.width, ci + span + 1)):
            if grid.free((i, j), threshold):
                continue
            distance = math.hypot(max(i - x, x - i - 1, 0), max(j - y, y - j - 1, 0))
            if distance <= clearance or math.isclose(distance, clearance, abs_tol=1e-12):
                return False
    return True


def revisit_views(grid, robot, target_x, target_y, observations, *, now_s,
                  map_revision, max_age_s, association_m, min_shift_m,
                  transit_radius, view_clearance, threshold, max_adjustment_m=0.0, cancel=None):
    """Rank recent measured views that remain safe and connected.

    All stamps/now must use one ROS clock. The caller must obtain observer TF
    at the observation stamp and invalidate map revisions on map changes.
    This is repeat observation, not a promise of a new triangulation baseline.
    It must not mark a box inspected, grant coverage or publish a report.
    """
    values = (target_x, target_y, now_s, max_age_s, association_m, min_shift_m,
              transit_radius, view_clearance, max_adjustment_m)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('revisit arguments must be finite')
    if (now_s < 0 or max_age_s <= 0 or association_m <= 0
            or min(min_shift_m, transit_radius, view_clearance, max_adjustment_m) < 0):
        raise ValueError('invalid revisit limits')
    if type(map_revision) is not int or map_revision < 0:
        raise ValueError('invalid current map revision')
    if not isinstance(grid, Grid) or not isinstance(robot, Pose):
        raise ValueError('a map Grid and robot Pose are required')
    # A finer map must not disable exact measured views with the default budget.
    # Keep any correction within both the configured limit and one map cell.
    adjustment_limit = min(max_adjustment_m, grid.resolution)
    check_cancelled(cancel)
    reachable = grid.reachable_from(robot.x, robot.y, transit_radius, threshold, cancel)
    clearance = max(transit_radius, view_clearance)
    result = []
    for view in observations:
        check_cancelled(cancel)
        if not isinstance(view, ObservedView):
            raise ValueError('expected stamped ObservedView records')
        age = now_s - view.stamp_s
        if view.map_revision != map_revision or not 0 <= age <= max_age_s:
            continue
        if math.hypot(view.target_x - target_x, view.target_y - target_y) > association_m:
            continue
        cell = grid.cell(view.pose.x, view.pose.y)
        if cell not in reachable:
            continue
        goals = []
        if safe_observer(grid, view.pose, clearance, threshold):
            goals.append(view.pose)
        elif adjustment_limit > 0:
            span = math.ceil(adjustment_limit / grid.resolution) + 1
            ci, cj = cell
            for j in range(max(0, cj - span), min(grid.height, cj + span + 1)):
                check_cancelled(cancel)
                for i in range(max(0, ci - span), min(grid.width, ci + span + 1)):
                    if (i, j) not in reachable:
                        continue
                    pose = Pose(*grid.center((i, j)), view.pose.yaw)
                    if math.hypot(pose.x - view.pose.x, pose.y - view.pose.y) > adjustment_limit:
                        continue
                    if safe_observer(grid, pose, clearance, threshold):
                        goals.append(pose)
        for pose in goals:
            if math.hypot(pose.x - robot.x, pose.y - robot.y) >= min_shift_m:
                result.append(RevisitHint(pose, view))
    # Prefer the smallest correction, then sensor uncertainty and recency.
    # Neither the evidence nor the corrected goal guarantees a future image.
    ranked = sorted(result, key=lambda v: (v.adjustment_m, v.evidence.sigma_m,
                                           now_s - v.evidence.stamp_s,
                                           math.hypot(v.pose.x - robot.x, v.pose.y - robot.y)))
    unique = {}
    for view in ranked:
        unique.setdefault(view.pose, view)
    return tuple(unique.values())
