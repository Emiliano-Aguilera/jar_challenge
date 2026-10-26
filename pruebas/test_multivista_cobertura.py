"""Guard coverage restarts against lost views, timing regressions and false baselines."""
from collections import Counter
from concurrent.futures import CancelledError
from dataclasses import replace
import math
import random
from threading import Event

import pytest

from equipo_jar.cobertura_plan import (
    Pose, Settings, _block_order, _block_proposals, _meets_order_checks, _multiview_seed, _order_checkpoints,
    _meets_view_clock, _view_clock_checks, improve_order, route_metrics)


class Costs:
    resolution = 1.0

    def between(self, first, last):
        return math.dist(first, last) * self.resolution


def view(x, y, mask, yaw=0.0):
    return (x, y), Pose(float(x), float(y), yaw), mask


def config(**values):
    return replace(Settings(), travel_speed=1.0, turn_speed=1.0,
                   observation_time=1.0, **values)


@pytest.mark.parametrize('value', [0.0, -0.1, math.inf, math.nan])
def test_baseline_requires_a_finite_positive_distance(value):
    with pytest.raises(ValueError):
        Settings(order_baseline_m=value)


@pytest.mark.parametrize('value', [0, 1, 'false', None])
def test_restart_switch_requires_a_boolean(value):
    with pytest.raises(ValueError):
        Settings(order_multiview=value)


@pytest.mark.parametrize('value', [-1, 17, 1.5, True])
def test_shortlist_work_limit_is_a_bounded_integer(value):
    with pytest.raises(ValueError):
        Settings(order_block_candidates=value)


def test_rotation_at_one_center_is_not_a_second_viewpoint():
    tasks = [view(0, 0, 1), view(0, 0, 1, .1), view(1, 0, 1)]
    result = _multiview_seed(tasks, (0, 0), 0.0, Costs(), config())
    assert result[:2] == [tasks[0], tasks[2]]
    assert Counter(result) == Counter(tasks)


def test_baseline_uses_metres_even_when_positions_are_grid_indices():
    costs = Costs()
    costs.resolution = .1
    tasks = [view(0, 0, 1), view(1, 0, 1), view(3, 0, 1)]
    result = _multiview_seed(tasks, (0, 0), 0.0, costs, config())
    assert result[:2] == [tasks[0], tasks[2]]


def test_equal_total_coverage_cannot_compensate_for_a_postponed_region():
    tasks = [view(1, 0, 1), view(2, 0, 2)]
    curve = ((0.0, 0), (1.0, 1), (2.0, 2))
    # Both orders claim the same counts, but they cover different regions first.
    assert _meets_order_checks(curve, _order_checkpoints(curve, 1.0, 0.0))
    assert not _meets_view_clock(tasks[::-1], curve, _view_clock_checks(tasks, curve, 1.0, 0.0))
    assert _meets_view_clock(tasks, curve, _view_clock_checks(tasks, curve, 1.0, 0.0))


def test_clock_guard_keeps_zero_gain_and_repeated_view_occurrences():
    first, last = view(1, 0, 1), view(2, 0, 1)
    tasks = [first, first, last]
    curve = ((0.0, 0), (1.0, 1), (2.0, 1), (3.0, 1))
    guards = _view_clock_checks(tasks, curve, 1.0, 0.0)
    assert _meets_view_clock(tasks, curve, guards)
    assert not _meets_view_clock([first, last, first], curve, guards)


def test_cancelled_restart_never_returns_a_new_plan():
    cancel = Event()
    cancel.set()
    with pytest.raises(CancelledError):
        _multiview_seed([view(0, 0, 1)], (0, 0), 0.0, Costs(), config(), cancel)


def test_generated_restart_keeps_all_views_and_every_baseline_clock_check():
    rng = random.Random(41690)
    positions = [(x, y) for x in range(1, 10) for y in range(1, 10)]
    improved = 0
    for _ in range(40):
        tasks = [view(x, y, rng.randrange(1, 1 << 14), rng.uniform(-math.pi, math.pi))
                 for x, y in rng.sample(positions, 9)]
        original = list(tasks)
        settings = config(order_block_candidates=0, order_check_interval_s=rng.uniform(3.0, 60.0),
                          order_start_delay_s=rng.uniform(0.0, 8.0))
        baseline = improve_order(tasks, (0, 0), 0.0, Costs(), replace(
            settings, order_multiview=False, order_block_candidates=0), 16383)
        result = improve_order(tasks, (0, 0), 0.0, Costs(), settings, 16383)
        before = route_metrics(baseline, (0, 0), 0.0, Costs(), settings, 16383)
        after = route_metrics(result, (0, 0), 0.0, Costs(), settings, 16383)
        assert tasks == original
        assert sorted(result) == sorted(tasks)
        assert result == improve_order(tasks, (0, 0), 0.0, Costs(), settings, 16383)
        assert after.duration <= before.duration + 1e-9
        assert after.mean_first_view_time <= before.mean_first_view_time + 1e-9
        assert after.duration + after.return_distance <= before.duration + before.return_distance + 1e-9
        assert _meets_order_checks(after.coverage_curve, _order_checkpoints(
            before.coverage_curve, settings.order_check_interval_s, settings.order_start_delay_s))
        for check in range(1, math.ceil((before.duration + settings.order_start_delay_s) / settings.order_check_interval_s) + 1):
            seconds = check * settings.order_check_interval_s - settings.order_start_delay_s
            required = Counter(entry for entry, (finished, _) in zip(baseline, before.coverage_curve[1:])
                               if finished <= seconds + 1e-9)
            actual = Counter(entry for entry, (finished, _) in zip(result, after.coverage_curve[1:])
                             if finished <= seconds + 1e-9)
            assert not required - actual
        improved += after.duration < before.duration - 1e-9
    assert improved > 0, 'exercise accepted alternatives as well as fallbacks'


def test_segment_edge_deltas_match_independent_full_route_costs():
    rng = random.Random(8854)
    for count in range(3, 10):
        ids = rng.sample(range(count), count)
        matrix = [[rng.randrange(1, 100) for _ in range(count + 1)] for _ in range(count + 1)]
        def duration(sequence):
            return matrix[count][sequence[0]] + sum(matrix[a][b] for a, b in zip(sequence, sequence[1:]))
        before, expected = duration(ids), []
        for size in (2, 3):
            for first in range(count - size + 1):
                rest = ids[:first] + ids[first + size:]
                group = []
                for reverse in (False, True):
                    block = ids[first:first + size]
                    if reverse:
                        block = block[::-1]
                    for destination in range(len(rest) + 1):
                        trial = rest[:destination] + block + rest[destination:]
                        delta = duration(trial) - before
                        if delta < 0:
                            group.append((delta, first, size, destination, reverse))
                expected.extend(sorted(group)[:4])
        assert sorted(_block_proposals(ids, matrix, 4)) == sorted(expected)


def test_segment_cleanup_preserves_guards_and_all_headings_on_generated_tasks():
    rng = random.Random(77122)
    improved = 0
    for _ in range(30):
        tasks = [view(rng.randrange(10), rng.randrange(10), rng.randrange(1, 1024), rng.uniform(-math.pi, math.pi))
                 for _ in range(9)]
        settings = config(order_check_interval_s=rng.uniform(3.0, 20.0), order_start_delay_s=2.0)
        result = _block_order(tasks, (0, 0), 0.0, Costs(), settings, 1023)
        before = route_metrics(tasks, (0, 0), 0.0, Costs(), settings, 1023)
        after = route_metrics(result, (0, 0), 0.0, Costs(), settings, 1023)
        assert Counter(result) == Counter(tasks)
        assert after.duration <= before.duration + 1e-9
        assert after.mean_first_view_time <= before.mean_first_view_time + 1e-9
        assert after.duration + after.return_distance <= before.duration + before.return_distance + 1e-9
        assert _meets_order_checks(after.coverage_curve, _order_checkpoints(before.coverage_curve,
            settings.order_check_interval_s, settings.order_start_delay_s))
        for check in range(1, math.ceil((before.duration + settings.order_start_delay_s) / settings.order_check_interval_s) + 1):
            seconds = check * settings.order_check_interval_s - settings.order_start_delay_s
            required = Counter(entry for entry, (finished, _) in zip(tasks, before.coverage_curve[1:])
                               if finished <= seconds + 1e-9)
            actual = Counter(entry for entry, (finished, _) in zip(result, after.coverage_curve[1:])
                             if finished <= seconds + 1e-9)
            assert not required - actual
        assert _block_order(tasks, (0, 0), 0.0, Costs(), replace(settings, order_block_candidates=0), 1023) == tasks
        improved += after.duration < before.duration - 1e-9
    assert improved > 0
