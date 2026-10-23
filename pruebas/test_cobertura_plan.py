"""Geometry and planning checks on generated maps, independent of ROS."""
from dataclasses import replace
import math

import pytest

from equipo_jar.cobertura_plan import (
    Camera, Grid, Pose, Settings, TravelCosts, distances, improve_order, make_plan,
    reachable_cells, route_metrics, sampled_cells, shortest_leg)


def room(width=12, height=12, resolution=0.1, blocked=(), origin=Pose(0.0, 0.0, 0.0)):
    obstacles = set(blocked)
    data = [100 if (i, j) in obstacles or i in (0, width - 1) or j in (0, height - 1)
            else 0 for j in range(height) for i in range(width)]
    return Grid(width, height, resolution, data, origin)


def test_rotated_origin_round_trip_and_row_order():
    grid = Grid(3, 2, 0.2, [0, 100, 0, -1, 0, 0], Pose(-2.0, -3.0, math.pi / 3))
    for j in range(2):
        for i in range(3):
            assert grid.cell(*grid.center((i, j))) == (i, j)
    assert not grid.free((1, 0), 25)
    assert not grid.free((0, 1), 25)
    assert grid.free((1, 1), 25)


def test_outside_negative_edge_is_not_truncated_into_map():
    grid = Grid(2, 2, 1.0, [0] * 4)
    assert grid.cell(-0.001, 0.5) is None
    assert grid.cell(0.5, -0.001) is None
    assert grid.cell(2.0, 0.5) is None
    assert grid.cell(math.nan, 0.5) is None


@pytest.mark.parametrize('width,height,resolution,data', [
    (0, 2, 0.1, []), (2, 2, 0.0, [0] * 4), (2, 2, math.nan, [0] * 4),
    (2, 2, 0.1, [0]), (2, 2, 0.1, [0, 0, 0, 101]),
    (2, 2, 0.1, [0, 0, 0, -2]),
])
def test_invalid_maps(width, height, resolution, data):
    with pytest.raises(ValueError):
        Grid(width, height, resolution, data)


@pytest.mark.parametrize('settings', [
    {'robot_radius': 0.0}, {'safety_margin': -0.1}, {'fov_degrees': 180.0},
    {'heading_count': 1.5}, {'max_views': 0}, {'travel_speed': math.inf},
    {'free_threshold': 100}, {'target_spacing': math.nan},
])
def test_invalid_parameters(settings):
    with pytest.raises(ValueError):
        Settings(**settings)


def test_inflation_uses_whole_wall_cells_and_unknown_is_blocked():
    grid = room(width=16, height=16, blocked=[(6, 6)])
    safe = grid.safe_cells(0.15, 25)
    assert (6, 6) not in safe
    assert (7, 6) not in safe
    assert (7, 7) not in safe
    assert (8, 6) not in safe  # touches the wall's square at 0.15 m
    assert (9, 6) in safe
    unknown = Grid(5, 5, 1.0, [(-1 if k == 12 else 0) for k in range(25)])
    assert (2, 2) not in unknown.safe_cells(0.1, 25)
    assert not unknown.visible((1.5, 2.5), (3.5, 2.5), 25)


def test_inflation_does_not_turn_walls_into_visibility_obstacles():
    grid = room()
    near_wall = (1, 5)
    assert near_wall not in grid.safe_cells(0.21, 25)
    assert grid.visible(grid.center((5, 5)), grid.center(near_wall), 25)


def test_rays_do_not_see_through_wall_or_corner():
    wall = room(blocked=[(6, j) for j in range(1, 11)])
    assert not wall.visible(wall.center((3, 5)), wall.center((9, 5)), 25)
    assert wall.visible(wall.center((3, 5)), wall.center((5, 5)), 25)
    corner = room(blocked=[(5, 4)])
    assert not corner.visible(corner.center((4, 4)), corner.center((5, 5)), 25)
    assert not corner.visible(corner.center((5, 5)), corner.center((4, 4)), 25)


def test_dijkstra_does_not_cross_blocked_diagonal():
    cells = {(0, 0), (1, 1), (0, 2), (1, 2)}
    assert distances(cells, (0, 0), 0.1) == {(0, 0): 0.0}
    connected = {(0, 0), (1, 0), (1, 1)}
    assert distances(connected, (0, 0), 0.1)[(1, 1)] == pytest.approx(0.2)


def test_disconnected_rooms_never_receive_goals():
    grid = room(width=20, blocked=[(10, j) for j in range(1, 11)])
    settings = replace(Settings(), robot_radius=0.1, safety_margin=0.01)
    plan = make_plan(grid, Pose(*grid.center((4, 5)), 0.0), settings)
    assert plan.poses
    assert all(grid.cell(p.x, p.y)[0] < 10 for p in plan.poses)
    assert 0 < plan.predicted_count < plan.target_count
    assert plan == make_plan(grid, Pose(*grid.center((4, 5)), 0.0), settings)


def test_unsafe_start_is_rejected_instead_of_snapped_through_wall():
    grid = room()
    with pytest.raises(ValueError, match='safe free space'):
        make_plan(grid, Pose(*grid.center((0, 0)), 0.0))


def test_route_contains_safe_finite_oriented_goals_and_honors_limits():
    grid = room(width=22, height=18, origin=Pose(-1.7, -2.4, 0.4))
    settings = replace(Settings(), max_views=4, max_positions=12, max_targets=30)
    plan = make_plan(grid, Pose(*grid.center((6, 6)), 1.0), settings)
    safe = grid.safe_cells(settings.robot_radius + settings.safety_margin, 25)
    assert 0 < len(plan.poses) <= 4
    assert plan.target_count <= 30
    assert plan.candidate_count <= 12 * settings.heading_count
    assert len(set(plan.poses)) == len(plan.poses)
    assert all(grid.cell(p.x, p.y) in safe for p in plan.poses)
    assert all(math.isfinite(p.yaw) for p in plan.poses)


def test_full_room_plan_reaches_all_sampled_targets():
    grid = room(width=18, height=18)
    plan = make_plan(grid, Pose(*grid.center((5, 5)), 0.0))
    assert plan.predicted_count == plan.target_count
    assert len({round(p.yaw, 3) for p in plan.poses}) > 1


def test_camera_forward_axis_changes_goal_orientations():
    grid = room(width=18, height=18)
    settings = replace(Settings(), max_views=1)
    start = Pose(*grid.center((5, 5)), 0.0)
    forward = make_plan(grid, start, settings)
    backward = make_plan(grid, start, settings, Camera(yaw=math.pi))
    assert forward.poses != backward.poses


def test_sampling_preserves_cells_in_narrow_bins_and_has_a_budget():
    cells = {(i, 4) for i in range(100)}
    samples = sampled_cells(cells, 0.45, 0.05, 10)
    assert 0 < len(samples) <= 10
    assert all(cell in cells for cell in samples)
    assert samples == sampled_cells(reversed(sorted(cells)), 0.45, 0.05, 10)


def test_reachability_keeps_only_start_component():
    assert reachable_cells({(0, 0), (1, 0), (8, 0)}, (0, 0)) == {(0, 0), (1, 0)}


def test_route_polish_preserves_coverage_and_improves_both_time_measures():
    grid = room(width=40, height=40)
    start = Pose(*grid.center((6, 6)), 0.0)
    raw = make_plan(grid, start, refine=False, improve=False)
    polished = make_plan(grid, start, refine=False, improve=True, diagnostics=True)
    assert polished.predicted_count == raw.predicted_count
    assert set(polished.poses) == set(raw.poses)
    assert polished.metrics.duration <= raw.metrics.duration + 1e-9
    assert polished.metrics.mean_first_view_time <= raw.metrics.mean_first_view_time + 1e-9
    assert set(polished.diagnostics.covered) <= set(polished.diagnostics.targets)
    curve = polished.metrics.coverage_curve
    assert all(t1 <= t2 and c1 <= c2 for (t1, c1), (t2, c2) in zip(curve, curve[1:]))
    assert curve[-1] == (polished.metrics.duration, polished.predicted_count)


def test_refinement_keeps_initial_observations_and_respects_view_budget():
    grid = room(width=35, height=30, blocked=[(15, j) for j in range(1, 17)])
    start = Pose(*grid.center((5, 5)), 0.0)
    settings = replace(Settings(), candidate_spacing=1.2, view_range=0.6, max_views=80)
    base = make_plan(grid, start, settings, refine=False, improve=False)
    improved = make_plan(grid, start, settings, diagnostics=True)
    assert set(base.poses) <= set(improved.poses)
    assert improved.predicted_count > base.predicted_count
    assert improved.diagnostics.added_positions > 0
    assert len(improved.poses) <= settings.max_views


def test_order_improvement_shortens_a_crossing_route():
    cells = {(i, j) for j in range(5) for i in range(5)}
    sequence = [(1, 0), (3, 3), (3, 0), (1, 3)]
    choices = [(cell, Pose(float(cell[0]), float(cell[1]), 0.0), 1 << index)
               for index, cell in enumerate(sequence)]
    costs = TravelCosts(cells, sequence + [(0, 0)], 1.0)
    settings = Settings()
    before = route_metrics(choices, (0, 0), 0.0, costs, settings, 15)
    improved = improve_order(choices, (0, 0), 0.0, costs, settings, 15)
    after = route_metrics(improved, (0, 0), 0.0, costs, settings, 15)
    assert {entry[1] for entry in choices} == {entry[1] for entry in improved}
    assert after.duration < before.duration
    assert after.mean_first_view_time <= before.mean_first_view_time


def test_planner_can_be_cancelled_while_preparing_geometry():
    from concurrent.futures import CancelledError

    class CancelDuringWork:
        checks = 0

        def is_set(self):
            self.checks += 1
            return self.checks >= 10

    cancel = CancelDuringWork()
    with pytest.raises(CancelledError):
        make_plan(room(width=40, height=40), Pose(0.65, 0.65, 0.0), cancel=cancel)
    assert cancel.checks == 10


def test_diagnostics_partition_missing_targets():
    grid = room(width=30, height=20, blocked=[(15, j) for j in range(1, 19)])
    plan = make_plan(grid, Pose(*grid.center((5, 5)), 0.0),
                     replace(Settings(), max_views=1), diagnostics=True)
    d = plan.diagnostics
    assert set(d.covered).isdisjoint(d.outside_candidates)
    assert set(d.covered).isdisjoint(d.deferred)
    assert set(d.deferred).isdisjoint(d.outside_candidates)
    assert set(d.covered) | set(d.outside_candidates) | set(d.deferred) == set(d.targets)
    assert len(d.covered) == plan.predicted_count
    assert len(d.covered) + len(d.deferred) == plan.metrics.visible_from_candidates
    assert d.outside_candidates and d.deferred


def test_diagnostic_paths_match_geodesic_distance_and_clearance():
    grid = room(width=20, height=24, blocked=[(10, j) for j in range(1, 15)])
    safe = grid.safe_cells(0.21, 25)
    first, last = (5, 5), (14, 5)
    leg = shortest_leg(safe, first, last, grid.resolution)
    assert leg[0] == first and leg[-1] == last
    assert all(cell in safe for cell in leg)
    length = sum(math.dist(a, b) * grid.resolution for a, b in zip(leg, leg[1:]))
    assert length == pytest.approx(distances(safe, first, grid.resolution)[last])
    for a, b in zip(leg, leg[1:]):
        if a[0] != b[0] and a[1] != b[1]:
            assert (a[0], b[1]) in safe and (b[0], a[1]) in safe


def test_line_of_sight_matches_independent_segment_square_intersection():
    import random
    rng = random.Random(719)
    size = 9
    walls = {(i, j) for j in range(size) for i in range(size) if rng.random() < 0.15}
    grid = Grid(size, size, 1.0, [100 if (i, j) in walls else 0
                                for j in range(size) for i in range(size)])

    def touches_square(start, end, cell):
        lower, upper = 0.0, 1.0
        for axis in range(2):
            delta = end[axis] - start[axis]
            if delta == 0:
                if not cell[axis] <= start[axis] <= cell[axis] + 1:
                    return False
            else:
                a = (cell[axis] - start[axis]) / delta
                b = (cell[axis] + 1 - start[axis]) / delta
                lower, upper = max(lower, min(a, b)), min(upper, max(a, b))
                if lower > upper + 1e-12:
                    return False
        return True

    for _ in range(500):
        start = grid.center((rng.randrange(size), rng.randrange(size)))
        end = grid.center((rng.randrange(size), rng.randrange(size)))
        expected = not any(touches_square(start, end, cell) for cell in walls)
        assert grid.visible(start, end, 25) == expected, (start, end)


@pytest.mark.parametrize('yaw', [0.4, -0.7, math.pi / 2, math.pi])
def test_rotating_the_map_preserves_visibility(yaw):
    plain = room(blocked=[(6, j) for j in range(3, 9)])
    rotated = room(blocked=[(6, j) for j in range(3, 9)], origin=Pose(-2.3, -3.8, yaw))
    cells = [(3, 3), (8, 8), (5, 5), (7, 4), (3, 9)]
    for a in cells:
        for b in cells:
            assert rotated.visible(rotated.center(a), rotated.center(b), 25) == plain.visible(
                plain.center(a), plain.center(b), 25)
