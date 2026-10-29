"""Revisit measured views without trusting stale/unsafe/other-map evidence."""
from concurrent.futures import CancelledError
from dataclasses import replace
import math
from threading import Event

import pytest

from equipo_jar.mapa import Grid, Pose
from equipo_jar.cobertura_revisita import ObservedView, RevisitHint, revisit_views, safe_observer


def call(grid, robot, views, **overrides):
    limits = dict(now_s=10.0, map_revision=3, max_age_s=5.0, association_m=0.3,
                  min_shift_m=0.05, transit_radius=0.15, view_clearance=0.25, threshold=25)
    limits.update(overrides)
    return revisit_views(grid, robot, 2.5, 1.5, views, **limits)


def example():
    return ObservedView(Pose(1.55, 1.55, -0.2), 2.5, 1.5, 9.0, 0.07, 3)


def test_recent_measured_view_preserves_observed_yaw_even_if_centre_ray_is_occluded():
    values = [0] * 1200
    for j in range(30):
        values[j * 40 + 20] = 100
    grid = Grid(40, 30, 0.1, values)
    view = example()
    assert not grid.visible((view.pose.x, view.pose.y), (view.target_x, view.target_y), 25)
    assert call(grid, Pose(1.55, 1.35, 0.0), (view,)) == (RevisitHint(view.pose, view),)


@pytest.mark.parametrize('change', [dict(stamp_s=4.9), dict(stamp_s=10.1),
                                   dict(map_revision=2), dict(target_x=3.0)])
def test_stale_future_other_map_and_unrelated_targets_are_rejected(change):
    grid = Grid(40, 30, 0.1, [0] * 1200)
    assert call(grid, Pose(1.55, 1.35, 0), (replace(example(), **change),)) == ()


def test_clear_but_disconnected_pocket_is_not_a_revisit_goal():
    values = [0] * 1200
    for j in range(30):
        values[j * 40 + 20] = 100
    grid = Grid(40, 30, 0.1, values)
    assert call(grid, Pose(2.55, 1.35, 0), (example(),)) == ()


def test_subcell_observer_does_not_borrow_clearance_from_the_cell_centre():
    values = [0] * 25
    values[2 * 5 + 2] = 100
    grid = Grid(5, 5, 1.0, values)
    assert (1, 2) in grid.safe_cells(0.25, 25)
    assert safe_observer(grid, Pose(1.5, 2.5, 0), 0.25, 25)
    assert not safe_observer(grid, Pose(1.8, 2.5, 0), 0.25, 25)


def test_unknown_and_map_edge_are_never_safe_evidence():
    values = [0] * 25
    values[12] = -1
    grid = Grid(5, 5, 1.0, values)
    assert not safe_observer(grid, Pose(2.5, 2.5, 0), 0.25, 25)
    assert not safe_observer(grid, Pose(0.2, 2.5, 0), 0.25, 25)


def test_old_40cm_shift_can_reject_a_valid_short_revisit_without_lowering_clearance():
    grid = Grid(40, 30, 0.1, [0] * 1200)
    view, robot = example(), Pose(1.55, 1.35, 0)
    assert call(grid, robot, (view,), min_shift_m=0.4) == ()
    assert call(grid, robot, (view,), min_shift_m=0.05) == (RevisitHint(view.pose, view),)


def test_duplicate_evidence_does_not_make_duplicate_goals():
    grid = Grid(40, 30, 0.1, [0] * 1200)
    view = example()
    assert call(grid, Pose(1.55, 1.35, 0), (view, view)) == (RevisitHint(view.pose, view),)


def test_duplicate_pose_keeps_the_better_measurement_regardless_of_input_order():
    grid = Grid(40, 30, 0.1, [0] * 1200)
    best = example()
    weak = replace(best, sigma_m=0.15)
    robot = Pose(1.55, 1.35, 0)
    assert call(grid, robot, (weak, best)) == call(grid, robot, (best, weak)) == (RevisitHint(best.pose, best),)


def test_unsafe_measurement_can_offer_a_nearby_safe_goal_without_reducing_clearance():
    values = [0] * (80 * 60)
    for j in range(60):
        values[j * 80 + 40] = 100
    grid = Grid(80, 60, 0.05, values)
    view = replace(example(), pose=Pose(1.755, 1.525, -0.2))
    robot = Pose(1.55, 1.35, 0)
    assert not safe_observer(grid, view.pose, 0.25, 25)
    assert call(grid, robot, (view,)) == ()
    hints = call(grid, robot, (view,), max_adjustment_m=0.05)
    assert hints
    assert all(h.evidence == view and 0 < h.adjustment_m <= 0.05 for h in hints)
    assert all(h.pose.yaw == view.pose.yaw and safe_observer(grid, h.pose, 0.25, 25) for h in hints)
    assert not call(grid, robot, (view,), max_adjustment_m=0.01)


def test_adjustment_never_escapes_the_robots_connected_component():
    values = [0] * (20 * 20)
    for j in range(20):
        values[j * 20 + 10] = 100
    grid = Grid(20, 20, 0.05, values)
    view = replace(example(), pose=Pose(0.56, 0.5, 0))
    assert call(grid, Pose(0.4, 0.5, 0), (view,), transit_radius=0.0,
                view_clearance=0.0, max_adjustment_m=0.05) == ()


def test_adjustment_budget_cannot_turn_into_a_large_search():
    values = [0] * 1200
    for j in range(30):
        values[j * 40 + 20] = 100
    grid = Grid(40, 30, 0.1, values)
    view = replace(example(), pose=Pose(1.9, 1.55, -0.2))
    assert not call(grid, Pose(1.55, 1.35, 0), (view,), max_adjustment_m=1.0)


def test_finer_map_retains_exact_safe_views_with_the_default_adjustment_budget():
    grid = Grid(160, 120, 0.025, [0] * 19200)
    view = example()
    assert call(grid, Pose(1.55, 1.35, 0), (view,), max_adjustment_m=0.05) == (RevisitHint(view.pose, view),)


@pytest.mark.parametrize('origin_yaw', [0.0, 0.37, -math.pi / 2])
def test_continuous_stand_matches_existing_disk_model_at_cell_centres(origin_yaw):
    values = [0] * 120
    values[46] = 100
    values[72] = -1
    grid = Grid(12, 10, 0.1, values, Pose(-1.2, 0.7, origin_yaw))
    for radius in (0.0, 0.02, 0.05, 0.15, 0.24, 0.30):
        safe = grid.safe_cells(radius, 25)
        for j in range(grid.height):
            for i in range(grid.width):
                pose = Pose(*grid.center((i, j)), 0)
                assert safe_observer(grid, pose, radius, 25) == ((i, j) in safe)


def test_cancelled_revisit_search_stops():
    signal = Event()
    signal.set()
    with pytest.raises(CancelledError):
        call(Grid(40, 30, 0.1, [0] * 1200), Pose(1.55, 1.35, 0), (example(),), cancel=signal)


@pytest.mark.parametrize('change', [dict(stamp_s=math.nan), dict(sigma_m=0.0), dict(map_revision=-1)])
def test_invalid_observation_metadata_is_rejected(change):
    with pytest.raises(ValueError):
        replace(example(), **change)
