"""Test view removal with dense temporal and viewpoint-diversity guards.

No production changes. Grid visibility is not evidence of processed images.
"""
import argparse
import json
import math
from pathlib import Path
import time

from equipo_jar.cobertura_plan import Camera, Pose, Settings, TravelCosts, reachable_cells
from evaluar_cobertura import read_grid
from metricas_cobertura import at_budget, route_trace, visibility_masks


def evidence(trace, cells, masks, resolution, baseline):
    """Cumulative seen cells and cells seen from sufficiently separated centers."""
    seen = {}
    diverse = 0
    events = []
    for event in trace.events:
        cell, mask = cells[event.index], masks[event.index]
        new = mask & ~seen.get(cell, 0)
        if new:
            for other, previous in seen.items():
                if math.dist(cell, other) * resolution >= baseline:
                    diverse |= new & previous
            seen[cell] = seen.get(cell, 0) | mask
        events.append((event.elapsed, event.mask, diverse))
    return events


def preserves_evidence(trial, reference, diverse):
    index = 0
    union = pairs = 0
    for seconds, required, required_pairs in reference:
        while index < len(trial) and trial[index][0] <= seconds + 1e-9:
            _, union, pairs = trial[index]
            index += 1
        if required & ~union or (diverse and required_pairs & ~pairs):
            return False
    return True


def prune(grid, start, poses, masks, costs, settings, startup, baseline, mode):
    cells = [grid.cell(p.x, p.y) for p in poses]
    indices = list(range(len(poses)))
    def trace(order):
        return route_trace(grid, start, poses, masks, order, costs, settings, startup)
    current = trace(indices)
    reference = evidence(current, cells, masks, grid.resolution, baseline)
    removed = []
    for index in reversed(range(len(poses))):
        if mode == 'position':
            other = 0
            for candidate in indices:
                if candidate != index and cells[candidate] == cells[index]:
                    other |= masks[candidate]
            if masks[index] & ~other:
                continue
        order = [candidate for candidate in indices if candidate != index]
        trial = trace(order)
        old_return = current.events[-1].return_distance if current.events else 0
        new_return = trial.events[-1].return_distance if trial.events else 0
        if (trial.duration + new_return / settings.travel_speed
                > current.duration + old_return / settings.travel_speed + 1e-9):
            continue
        if not preserves_evidence(evidence(trial, cells, masks, grid.resolution, baseline),
                                  reference, mode != 'union'):
            continue
        indices, current = order, trial
        removed.append(index)
    return indices, current, removed


def describe(trace, budgets, target_mask, settings):
    return dict(duration=trace.duration, distance=trace.distance, turning=trace.turning,
                goals=len(trace.events), visible=trace.events[-1].mask.bit_count() if trace.events else 0,
                budgets={str(seconds): at_budget(trace, seconds, target_mask=target_mask,
                                                 travel_speed=settings.travel_speed)
                         for seconds in budgets})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--simulator', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--baseline', type=float, default=.30)
    parser.add_argument('--box-feasibility', action='store_true',
                        help='exclude hypothetical box centers overlapping the robot (evaluation model)')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('runtime output must stay under build/')
    saved = json.loads(args.reference.read_text())
    settings, camera = Settings(**saved['settings']), Camera(**saved['camera'])
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in saved['cases']:
        if args.cases and case['case'] not in args.cases:
            continue
        print(f"Preparing {case['case']}", flush=True)
        grid = read_grid(args.simulator / 'maps' / f"{case['map']}.yaml")
        poses = [Pose(**p) for p in case['poses']]
        start = Pose(**case['start'])
        free = [(i, j) for j in range(grid.height) for i in range(grid.width)
                if grid.free((i, j), settings.free_threshold)]
        masks = visibility_masks(grid, poses, free, settings, camera)
        if args.box_feasibility:
            centers = [grid.center(cell) for cell in free]
            filtered = []
            for pose, visible in zip(poses, masks):
                co, si = math.cos(pose.yaw), math.sin(pose.yaw)
                valid = 0
                while visible:
                    bit = visible & -visible
                    tx, ty = centers[bit.bit_length() - 1]
                    dx, dy = tx - pose.x, ty - pose.y
                    x, y = co * dx + si * dy, -si * dx + co * dy
                    # Chassis from the current simulator/Nav2 contract; box
                    # inner disk is necessary for every 0.25 m square heading.
                    distance = math.hypot(max(x - .119, -.181 - x, 0), max(abs(y) - .10, 0))
                    if distance >= .125:
                        valid |= bit
                    visible ^= bit
                filtered.append(valid)
            masks = tuple(filtered)
        safe = grid.safe_cells(settings.robot_radius + settings.safety_margin, settings.free_threshold)
        reachable = reachable_cells(safe, grid.cell(start.x, start.y))
        costs = TravelCosts(reachable, [grid.cell(start.x, start.y)] +
                            [grid.cell(p.x, p.y) for p in poses], grid.resolution)
        startup = case['intended']['startup_delay']
        target_mask = (1 << len(free)) - 1
        before = route_trace(grid, start, poses, masks, range(len(poses)), costs, settings, startup)
        row = dict(case=case['case'], reference=describe(before, saved['budgets'], target_mask, settings),
                   variants={})
        for mode in ('position', 'diverse', 'union'):
            beginning = time.perf_counter()
            order, result, removed = prune(grid, start, poses, masks, costs, settings,
                                            startup, args.baseline, mode)
            after = describe(result, saved['budgets'], target_mask, settings)
            after.update(indices=order, removed=removed, compute_seconds=time.perf_counter() - beginning)
            assert after['visible'] == row['reference']['visible']
            row['variants'][mode] = after
            print(f"{case['case']} {mode}: {len(poses)}->{len(order)} goals; "
                  f"{before.duration:.2f}->{result.duration:.2f}s", flush=True)
        rows.append(row)
        (args.output / 'experiment.json').write_text(json.dumps(dict(cases=rows,
            baseline=args.baseline, limits='Dense grid model; no real observation or collision proof.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
