"""Compare guarded multi-view route moves on saved plans; no runtime changes."""
import argparse
import heapq
import json
from pathlib import Path
import time

from equipo_jar.cobertura_plan import (
    Camera, Pose, Settings, TravelCosts, _meets_order_checks, _order_checkpoints,
    reachable_cells, route_metrics, sampled_cells, angle_difference, _within_order_costs)
from evaluar_cobertura import read_grid
from experimentar_orden import summary
from metricas_cobertura import visibility_masks


def reorder(choices, start_cell, start_yaw, costs, settings, mode):
    if mode == 'shortlist':
        return shortlist(choices, start_cell, start_yaw, costs, settings)
    current = list(choices)
    metrics = route_metrics(current, start_cell, start_yaw, costs, settings, 0)
    accepted = attempts = 0
    for size in (2, 3):
        for first in range(len(current) - size + 1):
            best, best_metrics = current, metrics
            checks = _order_checkpoints(metrics.coverage_curve, settings.order_check_interval_s,
                                        settings.order_start_delay_s)
            block = current[first:first + size]
            rest = current[:first] + current[first + size:]
            for ordered in (block, list(reversed(block))):
                for last in range(len(rest) + 1):
                    trial = rest[:last] + ordered + rest[last:]
                    attempts += 1
                    candidate = route_metrics(trial, start_cell, start_yaw, costs, settings, 0)
                    if candidate.duration >= best_metrics.duration - 1e-9:
                        continue
                    if (candidate.duration + candidate.return_distance / settings.travel_speed
                            > metrics.duration + metrics.return_distance / settings.travel_speed + 1e-9):
                        continue
                    if (mode == 'guarded'
                            and candidate.mean_first_view_time > metrics.mean_first_view_time + 1e-9):
                        continue
                    if not _meets_order_checks(candidate.coverage_curve, checks):
                        continue
                    best, best_metrics = trial, candidate
            if best is not current:
                current, metrics = best, best_metrics
                accepted += 1
    return current, dict(accepted=accepted, attempts=attempts)


def shortlist(choices, start_cell, start_yaw, costs, settings, limit=4):
    """Exact edge deltas shortlist block moves before evaluating complete profiles."""
    choices = list(choices)
    ids = list(range(len(choices)))
    nodes = [(cell, pose.yaw) for cell, pose, _ in choices] + [(start_cell, start_yaw)]
    matrix = [[costs.between(a, b) / settings.travel_speed
               + abs(angle_difference(ay, by)) / settings.turn_speed
               for b, by in nodes] for a, ay in nodes]
    def edge(a, b):
        return matrix[a][b] if b is not None else 0.0
    accepted = attempts = 0
    metrics = route_metrics(choices, start_cell, start_yaw, costs, settings, 0)
    for _ in range(2):
        proposals = {}
        for size in (2, 3):
            for first in range(len(ids) - size + 1):
                block, rest = ids[first:first + size], ids[:first] + ids[first + size:]
                previous = ids[first - 1] if first else len(choices)
                following = ids[first + size] if first + size < len(ids) else None
                removal = -edge(previous, block[0]) - edge(block[-1], following) + edge(previous, following)
                inside = sum(edge(a, b) for a, b in zip(block, block[1:]))
                for reverse in (False, True):
                    moved = block[::-1] if reverse else block
                    change = removal + sum(edge(a, b) for a, b in zip(moved, moved[1:])) - inside
                    for destination in range(len(rest) + 1):
                        before = rest[destination - 1] if destination else len(choices)
                        after = rest[destination] if destination < len(rest) else None
                        delta = change + edge(before, moved[0]) + edge(moved[-1], after) - edge(before, after)
                        if delta < -1e-9:
                            proposals.setdefault((first, size), []).append((delta, first, size, destination, reverse))
        checks = _order_checkpoints(metrics.coverage_curve, settings.order_check_interval_s,
                                   settings.order_start_delay_s)
        best, best_metrics = ids, metrics
        shortlisted = [proposal for group in proposals.values() for proposal in heapq.nsmallest(limit, group)]
        for delta, first, size, destination, reverse in sorted(shortlisted):
            attempts += 1
            block, rest = ids[first:first + size], ids[:first] + ids[first + size:]
            trial = rest[:destination] + (block[::-1] if reverse else block) + rest[destination:]
            candidate = route_metrics([choices[i] for i in trial], start_cell, start_yaw, costs, settings, 0)
            assert abs(candidate.duration - metrics.duration - delta) < 1e-8
            if (candidate.duration < best_metrics.duration - 1e-9
                    and _within_order_costs(candidate, metrics, settings)
                    and _meets_order_checks(candidate.coverage_curve, checks)):
                best, best_metrics = trial, candidate
                break
        if best is ids:
            break
        ids, metrics = best, best_metrics
        accepted += 1
    return [choices[i] for i in ids], dict(accepted=accepted, attempts=attempts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--simulator', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cases', nargs='+')
    parser.add_argument('--modes', nargs='+', default=['guarded', 'clock'])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('runtime output must stay under build/')
    args.output.mkdir(parents=True, exist_ok=True)
    saved = json.loads(args.reference.read_text())
    settings, camera = Settings(**saved['settings']), Camera(**saved['camera'])
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
        dense_by_pose = {p: mask for p, mask in zip(poses, dense_masks)}
        safe = grid.safe_cells(settings.robot_radius + settings.safety_margin, settings.free_threshold)
        reachable = reachable_cells(safe, grid.cell(start.x, start.y))
        costs = TravelCosts(reachable, [grid.cell(start.x, start.y)] + [c[0] for c in choices], grid.resolution)
        startup, budgets = case['intended']['startup_delay'], saved['budgets']
        all_dense = (1 << len(dense)) - 1
        original = [(cell, p, dense_by_pose[p]) for cell, p, _ in choices]
        row = dict(case=case['case'], reference=summary(
            grid, start, original, costs, settings, startup, budgets, all_dense), variants={})
        for mode in args.modes:
            beginning = time.perf_counter()
            result, counts = reorder(choices, grid.cell(start.x, start.y), start.yaw,
                                     costs, settings, mode)
            elapsed = time.perf_counter() - beginning
            assert sorted((p.x, p.y, p.yaw) for _, p, _ in result) == sorted(
                (p.x, p.y, p.yaw) for p in poses)
            after = summary(grid, start, [(c, p, dense_by_pose[p]) for c, p, _ in result],
                            costs, settings, startup, budgets, all_dense)
            after.update(counts, indices=[poses.index(p) for _, p, _ in result], compute_seconds=elapsed)
            row['variants'][mode] = after
            print(f"{case['case']} {mode}: {counts['accepted']} moves, "
                  f"{row['reference']['metrics']['duration']:.2f}->{after['metrics']['duration']:.2f}s; "
                  f"CPU {elapsed:.3f}s", flush=True)
        rows.append(row)
        (args.output / 'experiment.json').write_text(json.dumps(dict(cases=rows,
            limits='Fixed poses; predicted costs, not navigation or detection results.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
