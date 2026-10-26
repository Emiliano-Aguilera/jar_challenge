"""Map-based viewpoint planning, independent of ROS.

Visibility is a planar prediction, not proof that the camera processed an image.
Only the route below is predicted; execution and failure feedback need a team
agreement before we can maintain the remaining route during a run.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import CancelledError
from dataclasses import dataclass, replace
import heapq
import math


def check_cancelled(cancel):
    if cancel is not None and cancel.is_set():
        raise CancelledError('coverage calculation cancelled')


@dataclass(frozen=True)
class Settings:
    robot_radius: float = 0.18
    safety_margin: float = 0.03
    candidate_spacing: float = 0.45
    target_spacing: float = 0.15
    view_range: float = 2.0
    fov_degrees: float = 61.0
    heading_count: int = 8
    travel_speed: float = 0.25
    turn_speed: float = 0.7
    observation_time: float = 1.0
    max_views: int = 80
    max_positions: int = 256
    max_targets: int = 2048
    free_threshold: int = 25
    order_check_interval_s: float = 60.0
    order_start_delay_s: float = 8.0
    order_multiview: bool = True
    order_baseline_m: float = 0.30
    order_block_candidates: int = 4
    view_clearance: float = 0.24
    view_preferred_clearance: float = 0.30

    def __post_init__(self):
        positive = ('robot_radius', 'candidate_spacing', 'target_spacing',
                    'view_range', 'travel_speed', 'turn_speed', 'observation_time',
                    'order_baseline_m')
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError('safety_margin must be finite and nonnegative')
        if not math.isfinite(self.fov_degrees) or not 0 < self.fov_degrees < 180:
            raise ValueError('fov_degrees must be between 0 and 180')
        for name in ('heading_count', 'max_views', 'max_positions', 'max_targets'):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        if type(self.free_threshold) is not int or not 0 <= self.free_threshold < 100:
            raise ValueError('free_threshold must be an integer in [0, 100)')
        if (not math.isfinite(self.order_check_interval_s)
                or self.order_check_interval_s < self.observation_time):
            raise ValueError('order_check_interval_s must be finite and at least observation_time')
        if not math.isfinite(self.order_start_delay_s) or self.order_start_delay_s < 0:
            raise ValueError('order_start_delay_s must be finite and nonnegative')
        if type(self.order_multiview) is not bool:
            raise ValueError('order_multiview must be a boolean')
        if type(self.order_block_candidates) is not int or not 0 <= self.order_block_candidates <= 16:
            raise ValueError('order_block_candidates must be an integer in [0, 16]')
        for name in ('view_clearance', 'view_preferred_clearance'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('pose must be finite')


@dataclass(frozen=True)
class Camera:
    """Camera translation and forward-axis yaw in the robot frame, from TF."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('camera transform must be finite')


@dataclass(frozen=True)
class Plan:
    poses: tuple[Pose, ...]
    target_count: int
    predicted_count: int
    reachable_count: int
    candidate_count: int
    metrics: Metrics | None = None
    diagnostics: Diagnostics | None = None
    unpolished_metrics: Metrics | None = None


@dataclass(frozen=True)
class Metrics:
    distance: float
    turning: float
    duration: float
    return_distance: float
    mean_first_view_time: float
    visible_from_candidates: int
    coverage_curve: tuple[tuple[float, int], ...]


@dataclass(frozen=True)
class Diagnostics:
    targets: tuple
    covered: tuple
    outside_candidates: tuple
    deferred: tuple
    positions: tuple
    legs: tuple
    added_positions: int


class Grid:
    def __init__(self, width, height, resolution, data, origin=Pose(0.0, 0.0, 0.0)):
        if type(width) is not int or type(height) is not int or min(width, height) < 1:
            raise ValueError('map dimensions must be positive integers')
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('map resolution must be finite and positive')
        if len(data) != width * height:
            raise ValueError('map data length does not match dimensions')
        if any(type(v) is not int or not -1 <= v <= 100 for v in data):
            raise ValueError('map cells must be integers in [-1, 100]')
        self.width, self.height, self.resolution = width, height, resolution
        self.data, self.origin = tuple(data), origin
        self.cos, self.sin = math.cos(origin.yaw), math.sin(origin.yaw)
        self._free_cache = None

    def cell(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        dx, dy = x - self.origin.x, y - self.origin.y
        i = math.floor((self.cos * dx + self.sin * dy) / self.resolution)
        j = math.floor((-self.sin * dx + self.cos * dy) / self.resolution)
        return (i, j) if 0 <= i < self.width and 0 <= j < self.height else None

    def center(self, cell):
        i, j = cell
        x, y = (i + 0.5) * self.resolution, (j + 0.5) * self.resolution
        return (self.origin.x + self.cos * x - self.sin * y,
                self.origin.y + self.sin * x + self.cos * y)

    def free(self, cell, threshold):
        i, j = cell
        return (0 <= i < self.width and 0 <= j < self.height
                and 0 <= self.data[j * self.width + i] <= threshold)

    def _free_mask(self, threshold, cancel=None):
        """One immutable occupancy lookup per map/threshold, shared by geometry.

        Keep only the latest threshold. A local snapshot and atomic tuple
        replacement let concurrent readers use different thresholds safely.
        """
        cached = self._free_cache
        if cached is not None and cached[0] == threshold:
            return cached[1]
        mask = bytearray()
        for first in range(0, len(self.data), self.width):
            check_cancelled(cancel)
            mask.extend(0 <= value <= threshold
                        for value in self.data[first:first + self.width])
        result = bytes(mask)
        self._free_cache = threshold, result
        return result

    def safe_cells(self, radius, threshold, cancel=None):
        """Conservative circular footprint against full cell squares and map edges."""
        check_cancelled(cancel)
        clear = self._free_mask(threshold, cancel)
        width = self.width
        clearance = radius / self.resolution
        span = math.ceil(clearance + 0.5)
        offsets = []
        for dj in range(-span, span + 1):
            for di in range(-span, span + 1):
                distance = math.hypot(max(abs(di) - 0.5, 0), max(abs(dj) - 0.5, 0))
                if distance <= clearance or math.isclose(distance, clearance, abs_tol=1e-12):
                    offsets.append((di, dj))
        kernel_span = max(abs(di) for di, _ in offsets)
        safe = set()
        for j in range(self.height):
            check_cancelled(cancel)
            for i in range(self.width):
                if min(i + 0.5, j + 0.5, self.width - i - 0.5,
                       self.height - j - 0.5) <= clearance + 1e-12:
                    continue
                # isclose can include a square just beyond the exact radius.
                if min(i, j, width - i - 1, self.height - j - 1) < kernel_span:
                    continue
                if all(clear[(j + dj) * width + i + di] for di, dj in offsets):
                    safe.add((i, j))
        return safe

    def visible(self, start, end, threshold):
        """Supercover ray: a wall touched at a grid corner blocks the ray too."""
        first, last = self.cell(*start), self.cell(*end)
        if first is None or last is None:
            return False
        clear = self._free_mask(threshold)
        width, height = self.width, self.height
        i, j = first
        if not clear[j * width + i]:
            return False
        end_i, end_j = last
        dxw, dyw = end[0] - start[0], end[1] - start[1]
        dx = (self.cos * dxw + self.sin * dyw) / self.resolution
        dy = (-self.sin * dxw + self.cos * dyw) / self.resolution
        sxw, syw = start[0] - self.origin.x, start[1] - self.origin.y
        sx = (self.cos * sxw + self.sin * syw) / self.resolution
        sy = (-self.sin * sxw + self.cos * syw) / self.resolution
        step_i, step_j = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        ti = ((i + (dx > 0) - sx) / dx) if dx else math.inf
        tj = ((j + (dy > 0) - sy) / dy) if dy else math.inf
        delta_i = abs(1 / dx) if dx else math.inf
        delta_j = abs(1 / dy) if dy else math.inf
        while i != end_i or j != end_j:
            if math.isclose(ti, tj, rel_tol=1e-12, abs_tol=1e-12):
                ni, nj = i + step_i, j + step_j
                if not (0 <= ni < width and 0 <= nj < height
                        and clear[j * width + ni] and clear[nj * width + i]):
                    return False
                i, j = i + step_i, j + step_j
                ti, tj = ti + delta_i, tj + delta_j
            elif ti < tj:
                i, ti = i + step_i, ti + delta_i
            else:
                j, tj = j + step_j, tj + delta_j
            if not (0 <= i < width and 0 <= j < height and clear[j * width + i]):
                return False
        return True


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def reachable_cells(safe, start):
    if start not in safe:
        raise ValueError('robot pose is outside the safe free space; check TF or clearance')
    reached, pending = {start}, deque([start])
    while pending:
        i, j = pending.popleft()
        for nxt in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
            if nxt in safe and nxt not in reached:
                reached.add(nxt)
                pending.append(nxt)
    return reached


def distances(cells, start, resolution, cancel=None):
    """Dijkstra distances, with no diagonal cuts through corners."""
    best, pending = {start: 0.0}, [(0.0, start)]
    while pending:
        check_cancelled(cancel)
        distance, current = heapq.heappop(pending)
        if distance != best[current]:
            continue
        i, j = current
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                nxt = (i + di, j + dj)
                if (not (di or dj) or nxt not in cells or
                        (di and dj and ((i + di, j) not in cells or (i, j + dj) not in cells))):
                    continue
                value = distance + resolution * math.hypot(di, dj)
                if value < best.get(nxt, math.inf):
                    best[nxt] = value
                    heapq.heappush(pending, (value, nxt))
    return best


def sampled_cells(cells, spacing, resolution, limit):
    """One representative per spatial bin; increase spacing if the map is large."""
    stride = max(1, math.ceil(spacing / resolution))
    ordered = sorted(cells, key=lambda c: (c[1], c[0]))
    while True:
        bins = {}
        for i, j in ordered:
            key = i // stride, j // stride
            center_i, center_j = (key[0] + 0.5) * stride, (key[1] + 0.5) * stride
            score = (i + 0.5 - center_i) ** 2 + (j + 0.5 - center_j) ** 2
            if key not in bins or score < bins[key][0]:
                bins[key] = score, (i, j)
        if len(bins) <= limit:
            return [entry[1] for entry in bins.values()]
        stride += max(1, stride // 4)


def visibility_mask(grid, cell, yaw, centers, settings, camera):
    x, y = grid.center(cell)
    co, si = math.cos(yaw), math.sin(yaw)
    optical = (x + co * camera.x - si * camera.y, y + si * camera.x + co * camera.y)
    mask = 0
    radius_squared = settings.view_range ** 2
    half_fov = math.radians(settings.fov_degrees / 2)
    for index, (tx, ty) in enumerate(centers):
        dx, dy = tx - optical[0], ty - optical[1]
        if dx * dx + dy * dy > radius_squared:
            continue
        if abs(angle_difference(math.atan2(dy, dx), yaw + camera.yaw)) > half_fov:
            continue
        if grid.visible(optical, (tx, ty), settings.free_threshold):
            mask |= 1 << index
    return mask


def _heading_geometry(grid, settings, camera):
    """Prepare camera terms once; retain the original arithmetic order."""
    result = []
    for heading in range(settings.heading_count):
        yaw = grid.origin.yaw + heading * math.tau / settings.heading_count
        co, si = math.cos(yaw), math.sin(yaw)
        result.append((co * camera.x, si * camera.y, si * camera.x,
                       co * camera.y, yaw + camera.yaw))
    return tuple(result)


def _has_visible_heading(grid, cell, center, geometry, radius_squared, half_fov, threshold):
    """Single-target witness without rebuilding camera transforms per heading."""
    x, y = grid.center(cell)
    tx, ty = center
    for ax, bx, ay, by, looking in geometry:
        optical = x + ax - bx, y + ay + by
        dx, dy = tx - optical[0], ty - optical[1]
        if dx * dx + dy * dy > radius_squared:
            continue
        if abs(angle_difference(math.atan2(dy, dx), looking)) > half_fov:
            continue
        if grid.visible(optical, center, threshold):
            return True
    return False


class TravelCosts:
    """Keep distances only between candidate centers, not full cached grid fields."""
    def __init__(self, cells, positions, resolution, cancel=None):
        self.cells, self.positions, self.resolution = cells, set(positions), resolution
        self.cache = {}
        self.turn_cache = {}
        self.cancel = cancel

    def between(self, first, last):
        check_cancelled(self.cancel)
        if first == last:
            return 0.0
        if first not in self.cache:
            field = distances(self.cells, first, self.resolution, self.cancel)
            self.cache[first] = {cell: field[cell] for cell in self.positions}
        return self.cache[first][last]

    def turn_between(self, first, last):
        """Reuse exact angular costs across trials of the same view set."""
        key = first, last
        if key not in self.turn_cache:
            self.turn_cache[key] = abs(angle_difference(first, last))
        return self.turn_cache[key]


def route_metrics(choices, start_cell, start_yaw, costs, settings, possible):
    distance = turning = elapsed = weighted_time = 0.0
    mask, cell, yaw = 0, start_cell, start_yaw
    curve = [(0.0, 0)]
    turn_cost = getattr(costs, 'turn_between', None)
    for nxt, pose, visible in choices:
        leg = costs.between(cell, nxt)
        turn = (turn_cost(yaw, pose.yaw) if turn_cost is not None
                else abs(angle_difference(yaw, pose.yaw)))
        distance += leg
        turning += turn
        elapsed += leg / settings.travel_speed + turn / settings.turn_speed + settings.observation_time
        gain = (visible & ~mask).bit_count()
        weighted_time += elapsed * gain
        mask |= visible
        curve.append((elapsed, mask.bit_count()))
        cell, yaw = nxt, pose.yaw
    return Metrics(distance, turning, elapsed, costs.between(cell, start_cell),
                   weighted_time / mask.bit_count() if mask else 0.0,
                   possible.bit_count(), tuple(curve))


def improve_order(choices, start_cell, start_yaw, costs, settings, possible, cancel=None):
    """Keep every baseline view's predicted clock deadline during new ordering."""
    baseline = _polish_order(choices, start_cell, start_yaw, costs, settings, possible, cancel)
    if len(baseline) < 2:
        return baseline
    if settings.order_multiview:
        baseline = _regional_multiview_order(baseline, start_cell, start_yaw,
                                             costs, settings, possible, cancel)
    return _block_order(baseline, start_cell, start_yaw, costs, settings, possible, cancel)


def _polish_order(choices, start_cell, start_yaw, costs, settings, possible, cancel=None):
    """Bounded 2-opt and relocation sweeps, retaining every selected view."""
    choices = list(choices)
    metrics = route_metrics(choices, start_cell, start_yaw, costs, settings, possible)
    for first in range(len(choices) - 1):
        for last in range(first + 1, len(choices)):
            check_cancelled(cancel)
            candidate = choices[:first] + list(reversed(choices[first:last + 1])) + choices[last + 1:]
            trial = route_metrics(candidate, start_cell, start_yaw, costs, settings, possible)
            if (trial.duration < metrics.duration - 1e-9
                    and trial.mean_first_view_time <= metrics.mean_first_view_time + 1e-9):
                choices, metrics = candidate, trial
    # Keep every view: equal sampled coverage does not prove equal visibility
    # between samples, so removing a seemingly redundant pose would be unsafe.
    choices = relocate_order(choices, start_cell, start_yaw, costs, settings, possible,
                             settings.order_check_interval_s, settings.order_start_delay_s, cancel)
    # Later moves can unlock improvements for views visited early in the first
    # sweep. One cleanup sweep keeps the work bounded and repeats every guard.
    return relocate_order(choices, start_cell, start_yaw, costs, settings, possible,
                          settings.order_check_interval_s, settings.order_start_delay_s, cancel)


def _multiview_seed(choices, start_cell, start_yaw, costs, settings, cancel=None):
    """Favor scarce views and a second separated center, retaining every pose.

    This is a geometric ordering heuristic, not detection/confirmation feedback.
    Multiple headings at one center never count as a second position.
    """
    check_cancelled(cancel)
    positions = tuple(dict.fromkeys(cell for cell, _, _ in choices))
    unions = dict.fromkeys(positions, 0)
    for cell, _, mask in choices:
        check_cancelled(cancel)
        unions[cell] |= mask
    counts = {}
    for mask in unions.values():
        check_cancelled(cancel)
        while mask:
            bit = mask & -mask
            counts[bit] = counts.get(bit, 0) + 1
            mask ^= bit
    rarity = {}
    for bit, count in counts.items():
        rarity[count] = rarity.get(count, 0) | bit
    resolution = getattr(costs, 'resolution', 1.0)
    distant = {cell: [other for other in positions
                     if math.dist(cell, other) * resolution >= settings.order_baseline_m]
               for cell in positions}

    def weight(mask):
        return sum((mask & group).bit_count() / count for count, group in rarity.items())

    pending, result = list(choices), []
    seen = dict.fromkeys(positions, 0)
    covered = paired = 0
    current, yaw = start_cell, start_yaw
    while pending:
        check_cancelled(cancel)
        far = {}
        for cell in positions:
            mask = 0
            for other in distant[cell]:
                mask |= seen[other]
            far[cell] = mask
        best = None
        for index, (cell, pose, mask) in enumerate(pending):
            check_cancelled(cancel)
            duration = (costs.between(current, cell) / settings.travel_speed
                        + abs(angle_difference(yaw, pose.yaw)) / settings.turn_speed
                        + settings.observation_time)
            gain = weight(mask & ~covered) + weight(mask & far[cell] & ~paired)
            rank = gain / duration, -duration, -index
            if best is None or rank > best[0]:
                best = rank, index
        _, index = best
        cell, pose, mask = pending.pop(index)
        result.append((cell, pose, mask))
        paired |= mask & far[cell]
        covered |= mask
        seen[cell] |= mask
        current, yaw = cell, pose.yaw
    return result


def _within_order_costs(candidate, original, settings):
    return (candidate.duration <= original.duration + 1e-9
            and candidate.mean_first_view_time <= original.mean_first_view_time + 1e-9
            and candidate.duration + candidate.return_distance / settings.travel_speed
                <= original.duration + original.return_distance / settings.travel_speed + 1e-9)


def _regional_multiview_order(choices, start_cell, start_yaw, costs, settings, possible, cancel=None):
    """Restart within existing clock buckets; do not postpone their original views."""
    check_cancelled(cancel)
    current = list(choices)
    metrics = route_metrics(current, start_cell, start_yaw, costs, settings, possible)
    guards = _view_clock_checks(current, metrics.coverage_curve, settings.order_check_interval_s,
                                settings.order_start_delay_s)
    groups = []
    for index, (elapsed, _) in enumerate(metrics.coverage_curve[1:]):
        bucket = math.ceil((elapsed + settings.order_start_delay_s - 1e-9) / settings.order_check_interval_s)
        if groups and groups[-1][0] == bucket:
            groups[-1][2] = index + 1
        else:
            groups.append([bucket, index, index + 1])
    for _, first, last in groups:
        check_cancelled(cancel)
        if last - first < 2:
            continue
        previous = current[first - 1] if first else (start_cell, Pose(0.0, 0.0, start_yaw), 0)
        local = replace(settings, order_start_delay_s=(settings.order_start_delay_s
                                                       + metrics.coverage_curve[first][0]))
        seeded = _multiview_seed(current[first:last], previous[0], previous[1].yaw, costs, local, cancel)
        ordered = _polish_order(seeded, previous[0], previous[1].yaw, costs, local, possible, cancel)
        trial = current[:first] + ordered + current[last:]
        candidate = route_metrics(trial, start_cell, start_yaw, costs, settings, possible)
        if (not _within_order_costs(candidate, metrics, settings)
                or (candidate.duration >= metrics.duration - 1e-9
                    and candidate.mean_first_view_time >= metrics.mean_first_view_time - 1e-9)):
            continue
        if _meets_view_clock(trial, candidate.coverage_curve, guards):
            current, metrics = trial, candidate
    return current


def _block_order(choices, start_cell, start_yaw, costs, settings, possible, cancel=None):
    """Two bounded short-segment sweeps, shortlisting by exact edge-cost deltas.

    Rank a few insertions per segment instead of rebuilding every full route.
    Every accepted move retains all views and repeats all cost/clock guards.
    """
    check_cancelled(cancel)
    if not settings.order_block_candidates or len(choices) < 3:
        return list(choices)
    choices = list(choices)
    ids = list(range(len(choices)))
    nodes = [(cell, pose.yaw) for cell, pose, _ in choices] + [(start_cell, start_yaw)]
    matrix = []
    for cell, yaw in nodes:
        check_cancelled(cancel)
        matrix.append([costs.between(cell, nxt) / settings.travel_speed
                       + abs(angle_difference(yaw, heading)) / settings.turn_speed
                       for nxt, heading in nodes])

    metrics = route_metrics(choices, start_cell, start_yaw, costs, settings, possible)
    for _ in range(2):
        shortlist = _block_proposals(ids, matrix, settings.order_block_candidates, cancel)
        guards = _view_clock_checks([choices[i] for i in ids], metrics.coverage_curve,
                                   settings.order_check_interval_s, settings.order_start_delay_s)
        for _, first, size, destination, reverse in sorted(shortlist):
            check_cancelled(cancel)
            block, rest = ids[first:first + size], ids[:first] + ids[first + size:]
            trial = rest[:destination] + (block[::-1] if reverse else block) + rest[destination:]
            candidate = route_metrics([choices[i] for i in trial], start_cell, start_yaw, costs, settings, possible)
            if (candidate.duration < metrics.duration - 1e-9
                    and _within_order_costs(candidate, metrics, settings)
                    and _meets_view_clock([choices[i] for i in trial], candidate.coverage_curve, guards)):
                ids, metrics = trial, candidate
                break
        else:
            break
    return [choices[i] for i in ids]


def _block_proposals(ids, matrix, limit, cancel=None):
    """Rank segment insertions from their changed edges, including reversal costs."""
    def edge(first, last):
        return matrix[first][last] if last is not None else 0.0

    shortlist = []
    for size in (2, 3):
        for first in range(len(ids) - size + 1):
            check_cancelled(cancel)
            block, rest = ids[first:first + size], ids[:first] + ids[first + size:]
            before = ids[first - 1] if first else len(ids)
            after = ids[first + size] if first + size < len(ids) else None
            removal = -edge(before, block[0]) - edge(block[-1], after) + edge(before, after)
            inside = sum(edge(a, b) for a, b in zip(block, block[1:]))
            proposals = []
            for reverse in (False, True):
                moved = block[::-1] if reverse else block
                change = removal + sum(edge(a, b) for a, b in zip(moved, moved[1:])) - inside
                for destination in range(len(rest) + 1):
                    previous = rest[destination - 1] if destination else len(ids)
                    following = rest[destination] if destination < len(rest) else None
                    delta = change + edge(previous, moved[0]) + edge(moved[-1], following) - edge(previous, following)
                    if delta < -1e-9:
                        proposals.append((delta, first, size, destination, reverse))
            shortlist.extend(heapq.nsmallest(limit, proposals))
    return shortlist


def _view_clock_checks(choices, curve, interval, start_delay):
    """Index every original view occurrence, including views with zero sampled gain."""
    identities, checks = {}, []
    mask = 0
    for index, (entry, (seconds, _)) in enumerate(zip(choices, curve[1:])):
        bit = 1 << index
        identities.setdefault(entry, []).append(bit)
        mask |= bit
        clock = math.ceil((seconds + start_delay - 1e-9) / interval) * interval - start_delay
        if checks and abs(checks[-1][0] - clock) <= 1e-9:
            checks[-1] = clock, mask
        else:
            checks.append((clock, mask))
    return identities, checks


def _meets_view_clock(choices, curve, guards):
    """A greater coverage count cannot compensate for a postponed original view."""
    identities, checks = guards
    occurrences = {}
    index = mask = 0
    for seconds, required in checks:
        while index < len(choices) and curve[index + 1][0] <= seconds + 1e-9:
            entry = choices[index]
            occurrence = occurrences.get(entry, 0)
            try:
                mask |= identities[entry][occurrence]
            except (KeyError, IndexError):
                return False
            occurrences[entry] = occurrence + 1
            index += 1
        if required & ~mask:
            return False
    return True


def _order_checkpoints(curve, interval, start_delay):
    """Only the first clock-grid check after each coverage increase can fail.

    At most one check per view is needed, even for a very small interval.
    Times in the curve are relative to route start; the grid includes the
    estimated initial delay. This is a prediction, not mission's actual clock.
    """
    result = []
    previous = 0
    for seconds, count in curve:
        if count == previous:
            continue
        previous = count
        check = math.ceil((seconds + start_delay - 1e-9) / interval) * interval - start_delay
        if result and abs(check - result[-1][0]) <= 1e-9:
            result[-1] = check, count
        else:
            result.append((check, count))
    return result


def _meets_order_checks(curve, checks):
    index = 0
    for seconds, required in checks:
        while index + 1 < len(curve) and curve[index + 1][0] <= seconds + 1e-9:
            index += 1
        if curve[index][1] < required:
            return False
    return True


def relocate_order(choices, start_cell, start_yaw, costs, settings, possible,
                   interval, start_delay, cancel=None):
    """One bounded relocation sweep, preserving all views and sampled checks.

    Accept only lower mean first-view time (or equal mean and shorter search),
    with no increase in search duration or search-plus-return estimate. Do not
    reduce sampled visibility on the configured predicted clock grid. This
    does not guarantee dense visibility between target samples or clock checks.
    """
    if not math.isfinite(interval) or interval < settings.observation_time:
        raise ValueError('order interval must be finite and at least observation_time')
    if not math.isfinite(start_delay) or start_delay < 0:
        raise ValueError('order start delay must be finite and nonnegative')
    check_cancelled(cancel)
    current = list(choices)
    metrics = route_metrics(current, start_cell, start_yaw, costs, settings, possible)
    for entry in tuple(choices):
        check_cancelled(cancel)
        first = current.index(entry)
        rest = current[:first] + current[first + 1:]
        checks = _order_checkpoints(metrics.coverage_curve, interval, start_delay)
        best, best_metrics = current, metrics
        for last in range(len(current)):
            check_cancelled(cancel)
            if last == first:
                continue
            trial = rest[:last] + [entry] + rest[last:]
            candidate = route_metrics(trial, start_cell, start_yaw, costs, settings, possible)
            if candidate.duration > metrics.duration + 1e-9:
                continue
            if (candidate.duration + candidate.return_distance / settings.travel_speed
                    > metrics.duration + metrics.return_distance / settings.travel_speed + 1e-9):
                continue
            earlier = candidate.mean_first_view_time < best_metrics.mean_first_view_time - 1e-9
            equal = abs(candidate.mean_first_view_time - best_metrics.mean_first_view_time) <= 1e-9
            if not (earlier or (equal and candidate.duration < best_metrics.duration - 1e-9)):
                continue
            if not _meets_order_checks(candidate.coverage_curve, checks):
                continue
            best, best_metrics = trial, candidate
        current, metrics = best, best_metrics
    return current


def shortest_leg(cells, start, goal, resolution):
    """Reconstruct a shortest grid leg for diagnostic plots only."""
    if start == goal:
        return (start,)
    best, parents = {start: 0.0}, {}
    pending = [(resolution * math.dist(start, goal), 0.0, start)]
    while pending:
        _, distance, current = heapq.heappop(pending)
        if distance != best[current]:
            continue
        if current == goal:
            result = [goal]
            while result[-1] != start:
                result.append(parents[result[-1]])
            return tuple(reversed(result))
        i, j = current
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                nxt = i + di, j + dj
                if (not (di or dj) or nxt not in cells or
                        (di and dj and ((i + di, j) not in cells or (i, j + dj) not in cells))):
                    continue
                value = distance + resolution * math.hypot(di, dj)
                if value < best.get(nxt, math.inf):
                    best[nxt], parents[nxt] = value, current
                    heapq.heappush(pending, (value + resolution * math.dist(nxt, goal), value, nxt))
    raise ValueError('no grid path between observation goals')


def observation_cells(grid, reachable, settings, cancel=None, *, preferred=False):
    """Reachable centers with room for observation, separate from transit.

    The minimum envelope must cover the robot at every requested heading.
    Prefer the larger envelope where it can supply the needed visibility.
    Neither certifies map accuracy, localization or unmapped obstacles.
    Setting both observation clearances to zero restores the transit mask.
    """
    transit_clearance = settings.robot_radius + settings.safety_margin
    clearance = max(settings.view_clearance,
                    settings.view_preferred_clearance if preferred else 0.0)
    if clearance <= transit_clearance:
        return reachable
    return reachable & grid.safe_cells(clearance, settings.free_threshold, cancel)


def make_plan(grid, start, settings=Settings(), camera=Camera(), *,
              refine=True, improve=True, diagnostics=False, cancel=None):
    """Greedily cover sampled free cells, balancing new visibility and travel time.

    Output poses are observation goals, NOT a collision-free trajectory between
    goals. Navigation must plan and execute each leg, with dynamic obstacles.
    """
    check_cancelled(cancel)
    safe = grid.safe_cells(settings.robot_radius + settings.safety_margin,
                           settings.free_threshold, cancel)
    start_cell = grid.cell(start.x, start.y)
    reachable = reachable_cells(safe, start_cell)
    observation = observation_cells(grid, reachable, settings, cancel)
    preferred = observation_cells(grid, observation, settings, cancel, preferred=True)
    positions = sampled_cells(preferred, settings.candidate_spacing, grid.resolution,
                              settings.max_positions)
    if start_cell in preferred and start_cell not in positions:
        positions = [start_cell] + positions[:settings.max_positions - 1]
    free = {(i, j) for j in range(grid.height) for i in range(grid.width)
            if grid.free((i, j), settings.free_threshold)}
    targets = sampled_cells(free, settings.target_spacing, grid.resolution,
                            settings.max_targets)
    centers = [grid.center(cell) for cell in targets]
    options, possible = [], 0

    def add_position(cell):
        nonlocal possible
        check_cancelled(cancel)
        x, y = grid.center(cell)
        for heading in range(settings.heading_count):
            yaw = grid.origin.yaw + heading * math.tau / settings.heading_count
            mask = visibility_mask(grid, cell, yaw, centers, settings, camera)
            if mask:
                options.append((cell, Pose(x, y, yaw), mask))
                possible |= mask

    for cell in positions:
        add_position(cell)
    base_options = tuple(options)
    added = 0
    if refine:
        # Search fine safe cells only for targets missed by coarse sampling.
        # Exhaust preferred centers before using a minimum-clearance witness.
        # A target in another raw free-space component cannot have a wall-free
        # line of sight from this component in the conservative planar model.
        existing = set(positions)
        raw_component = reachable_cells(free, start_cell)
        radius_squared = ((settings.view_range + math.hypot(camera.x, camera.y)) / grid.resolution) ** 2
        geometry = _heading_geometry(grid, settings, camera)
        view_radius_squared = settings.view_range ** 2
        half_fov = math.radians(settings.fov_degrees / 2)
        for index, center in enumerate(centers):
            check_cancelled(cancel)
            if len(positions) >= settings.max_positions:
                break
            if possible & (1 << index) or targets[index] not in raw_component:
                continue
            ti, tj = targets[index]
            nearby = sorted((cell for cell in observation if cell not in existing
                             and (cell[0] - ti) ** 2 + (cell[1] - tj) ** 2 <= radius_squared),
                            key=lambda cell: (cell not in preferred,
                                              (cell[0] - ti) ** 2 + (cell[1] - tj) ** 2,
                                              cell[1], cell[0]))
            for cell in nearby:
                check_cancelled(cancel)
                witness = _has_visible_heading(
                    grid, cell, center, geometry, view_radius_squared,
                    half_fov, settings.free_threshold)
                if witness:
                    existing.add(cell)
                    add_position(cell)
                    positions.append(cell)
                    added += 1
                    break
    costs = TravelCosts(reachable, positions + [start_cell], grid.resolution, cancel)
    result, predicted = [], 0
    current_cell, current_yaw = start_cell, start.yaw
    def select_views(available, budget):
        nonlocal predicted, current_cell, current_yaw
        for _ in range(budget):
            chosen, best_score = None, 0.0
            for cell, pose, mask in available:
                gain = (mask & ~predicted).bit_count()
                if not gain:
                    continue
                duration = (costs.between(current_cell, cell) / settings.travel_speed
                            + abs(angle_difference(pose.yaw, current_yaw)) / settings.turn_speed
                            + settings.observation_time)
                score = gain / duration
                if score > best_score:
                    chosen, best_score = (cell, pose, mask), score
            if chosen is None:
                break
            cell, pose, mask = chosen
            result.append(chosen)
            predicted |= mask
            current_yaw, current_cell = pose.yaw, cell

    # Preserve the initial plan's complete view set, including visibility
    # between target samples, then fill gaps within the remaining view budget.
    select_views(base_options, settings.max_views)
    select_views(options, settings.max_views - len(result))
    unpolished = route_metrics(result, start_cell, start.yaw, costs, settings, possible)
    if improve:
        result = improve_order(result, start_cell, start.yaw, costs, settings, possible, cancel)
    metrics = route_metrics(result, start_cell, start.yaw, costs, settings, possible)
    details = None
    if diagnostics:
        covered = tuple(cell for index, cell in enumerate(targets) if predicted & (1 << index))
        outside = tuple(cell for index, cell in enumerate(targets) if not possible & (1 << index))
        deferred = tuple(cell for index, cell in enumerate(targets)
                         if possible & (1 << index) and not predicted & (1 << index))
        legs, current = [], start_cell
        for cell, _, _ in result:
            check_cancelled(cancel)
            legs.append(shortest_leg(reachable, current, cell, grid.resolution))
            current = cell
        details = Diagnostics(tuple(targets), covered, outside, deferred,
                              tuple(positions), tuple(legs), added)
    return Plan(tuple(entry[1] for entry in result), len(targets), predicted.bit_count(),
                len(reachable), len(options), metrics, details, unpolished)
