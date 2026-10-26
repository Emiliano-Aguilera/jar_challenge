"""Compare fixed-view orders that consider hard-to-see areas and second views."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time

from equipo_jar.cobertura_plan import (
    Camera, Pose, Settings, TravelCosts, angle_difference, improve_order,
    reachable_cells, route_metrics, sampled_cells, _meets_order_checks, _order_checkpoints)
from evaluar_cobertura import read_grid
from experimentar_orden import summary
from experimentar_seleccion import evidence
from metricas_cobertura import route_trace, visibility_masks


def greedy(choices, start_cell, start_yaw, costs, settings, baseline, mode):
    positions = tuple(dict.fromkeys(cell for cell, _, _ in choices))
    union_by_cell = {cell: 0 for cell in positions}
    for cell, _, mask in choices:
        union_by_cell[cell] |= mask
    counts = {}
    for mask in union_by_cell.values():
        while mask:
            bit = mask & -mask
            counts[bit] = counts.get(bit, 0) + 1
            mask ^= bit
    rarity = {}
    for bit, count in counts.items():
        rarity[count] = rarity.get(count, 0) | bit
    distant = {cell: [other for other in positions
                      if math.dist(cell, other) * costs.resolution >= baseline]
               for cell in positions}
    def gain(mask):
        return (sum((mask & group).bit_count() / count for count, group in rarity.items())
                if mode.startswith('rare') else mask.bit_count())
    pending, result = list(choices), []
    seen = {cell: 0 for cell in positions}
    covered = paired = 0
    current, yaw = start_cell, start_yaw
    while pending:
        far = {}
        for cell in positions:
            value = 0
            for other in distant[cell]:
                value |= seen[other]
            far[cell] = value
        best = None
        for index, (cell, pose, mask) in enumerate(pending):
            duration = (costs.between(current, cell) / settings.travel_speed
                        + abs(angle_difference(yaw, pose.yaw)) / settings.turn_speed
                        + settings.observation_time)
            value = gain(mask & ~covered)
            if 'pair' in mode:
                value += gain(mask & far[cell] & ~paired)
            rank = (value / duration, -duration, -index)
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


def pair_summary(trace, cells, masks, resolution, baseline):
    events = evidence(trace, cells, masks, resolution, baseline)
    result = {}
    for seconds in (60.0, 120.0, 300.0, 480.0):
        value = 0
        for elapsed, _, paired in events:
            if elapsed <= seconds:
                value = paired
        result[str(seconds)] = value.bit_count()
    return result


def repair_clock(ordered, reference, start, costs, settings, moves=2):
    """Try two deadline-rescue relocations within the original cost limits."""
    original = route_metrics(reference, start[0], start[1], costs, settings, 0)
    checks = _order_checkpoints(original.coverage_curve, settings.order_check_interval_s,
                               settings.order_start_delay_s)
    def deficits(metrics):
        index, values = 0, []
        for seconds, count in checks:
            while index + 1 < len(metrics.coverage_curve) and metrics.coverage_curve[index + 1][0] <= seconds + 1e-9:
                index += 1
            values.append(max(0, count - metrics.coverage_curve[index][1]))
        return values
    current = list(ordered)
    metrics = route_metrics(current, start[0], start[1], costs, settings, 0)
    if (metrics.duration > original.duration + 1e-9
            or metrics.mean_first_view_time > original.mean_first_view_time + 1e-9
            or metrics.duration + metrics.return_distance / settings.travel_speed
                > original.duration + original.return_distance / settings.travel_speed + 1e-9):
        return current
    for _ in range(moves):
        missing = deficits(metrics)
        if not any(missing):
            break
        bad = next(i for i, value in enumerate(missing) if value)
        deadline = checks[bad][0]
        first_late = next((i for i, (seconds, _) in enumerate(metrics.coverage_curve[1:])
                           if seconds > deadline + 1e-9), len(current))
        rank = sum(missing), metrics.duration, metrics.mean_first_view_time
        best, best_metrics = current, metrics
        for source in range(first_late, len(current)):
            entry = current[source]
            rest = current[:source] + current[source + 1:]
            for destination in range(first_late + 1):
                trial = rest[:destination] + [entry] + rest[destination:]
                candidate = route_metrics(trial, start[0], start[1], costs, settings, 0)
                if (candidate.duration > original.duration + 1e-9
                        or candidate.mean_first_view_time > original.mean_first_view_time + 1e-9
                        or candidate.duration + candidate.return_distance / settings.travel_speed
                            > original.duration + original.return_distance / settings.travel_speed + 1e-9):
                    continue
                candidate_missing = deficits(candidate)
                if any(candidate_missing[:bad]):
                    continue
                candidate_rank = sum(candidate_missing), candidate.duration, candidate.mean_first_view_time
                if candidate_rank < rank:
                    rank, best, best_metrics = candidate_rank, trial, candidate
        current, metrics = best, best_metrics
    return current


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--simulator', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--baseline', type=float, default=.30)
    parser.add_argument('--modes', nargs='+', default=['first', 'rare', 'pair', 'rare-pair'])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('runtime output must stay under build/')
    args.output.mkdir(parents=True, exist_ok=True)
    saved = json.loads(args.reference.read_text())
    settings = replace(Settings(**saved['settings']), order_multiview=False, order_block_candidates=0)
    camera = Camera(**saved['camera'])
    rows = []
    for case in saved['cases']:
        if args.cases and case['case'] not in args.cases:
            continue
        print(f"Preparing {case['case']}", flush=True)
        grid = read_grid(args.simulator / 'maps' / f"{case['map']}.yaml")
        start = Pose(**case['start'])
        poses = [Pose(**p) for p in case['poses']]
        free = {(i, j) for j in range(grid.height) for i in range(grid.width)
                if grid.free((i, j), settings.free_threshold)}
        dense = tuple(sorted(free, key=lambda c: (c[1], c[0])))
        targets = sampled_cells(free, settings.target_spacing, grid.resolution, settings.max_targets)
        masks = visibility_masks(grid, poses, targets, settings, camera)
        dense_masks = visibility_masks(grid, poses, dense, settings, camera)
        choices = [(grid.cell(p.x, p.y), p, mask) for p, mask in zip(poses, masks)]
        dense_by_pose = dict(zip(poses, dense_masks))
        safe = grid.safe_cells(settings.robot_radius + settings.safety_margin, settings.free_threshold)
        reachable = reachable_cells(safe, grid.cell(start.x, start.y))
        costs = TravelCosts(reachable, [grid.cell(start.x, start.y)] + [c[0] for c in choices], grid.resolution)
        startup, budgets = case['intended']['startup_delay'], saved['budgets']
        target_mask = (1 << len(dense)) - 1
        original = [(c, p, dense_by_pose[p]) for c, p, _ in choices]
        original_trace = route_trace(grid, start, poses, dense_masks, range(len(poses)), costs, settings, startup)
        sample_reference = route_metrics(choices, grid.cell(start.x, start.y), start.yaw, costs, settings, 0)
        row = dict(case=case['case'], reference=summary(
            grid, start, original, costs, settings, startup, budgets, target_mask),
            reference_pairs=pair_summary(original_trace, [c[0] for c in choices],
                                         dense_masks, grid.resolution, args.baseline), variants={})
        for mode in args.modes:
            beginning = time.perf_counter()
            ordered = greedy(choices, grid.cell(start.x, start.y), start.yaw,
                              costs, settings, args.baseline, mode)
            for polish in (False, True, 'repair'):
                result = (improve_order(ordered, grid.cell(start.x, start.y), start.yaw,
                                         costs, settings, 0) if polish else ordered)
                if polish == 'repair':
                    result = repair_clock(result, choices, (grid.cell(start.x, start.y), start.yaw),
                                          costs, settings)
                assert sorted((p.x, p.y, p.yaw) for _, p, _ in result) == sorted(
                    (p.x, p.y, p.yaw) for p in poses)
                indices = [poses.index(p) for _, p, _ in result]
                dense_result = [(c, p, dense_by_pose[p]) for c, p, _ in result]
                after = summary(grid, start, dense_result, costs, settings, startup, budgets, target_mask)
                trial = route_trace(grid, start, poses, dense_masks, indices, costs, settings, startup)
                after.update(indices=indices, compute_seconds=time.perf_counter()-beginning,
                    pairs=pair_summary(trial, [c[0] for c in choices], dense_masks,
                                       grid.resolution, args.baseline))
                sample_trial = route_metrics(result, grid.cell(start.x, start.y), start.yaw, costs, settings, 0)
                reference_metrics = row['reference']['metrics']
                guards = dict(
                    duration=sample_trial.duration <= sample_reference.duration + 1e-9,
                    return_duration=(sample_trial.duration + sample_trial.return_distance / settings.travel_speed
                                     <= sample_reference.duration + sample_reference.return_distance / settings.travel_speed + 1e-9),
                    mean=sample_trial.mean_first_view_time <= sample_reference.mean_first_view_time + 1e-9,
                    clock=_meets_order_checks(sample_trial.coverage_curve, _order_checkpoints(
                        sample_reference.coverage_curve, settings.order_check_interval_s, startup)))
                after['guards'] = guards
                after['eligible'] = all(guards.values())
                after['dense_clock'] = _meets_order_checks(after['metrics']['coverage_curve'],
                    _order_checkpoints(reference_metrics['coverage_curve'], settings.order_check_interval_s, startup))
                key = mode + ('-repaired' if polish == 'repair' else '-polished' if polish else '')
                row['variants'][key] = after
                print(f"{case['case']} {key}: {after['metrics']['duration']:.2f}s; "
                      f"dense300 {after['budgets']['300.0']['visible']}; "
                      f"pairs300 {after['pairs']['300.0']}", flush=True)
        rows.append(row)
        (args.output / 'experiment.json').write_text(json.dumps(dict(cases=rows,
            baseline=args.baseline, limits='Same saved views; second-view model, not detector confirmation.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
