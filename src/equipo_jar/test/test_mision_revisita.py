"""Observed-view fallback: acquire new evidence without changing report policy."""
from types import MethodType, SimpleNamespace as NS

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import pytest

from equipo_jar.cobertura_revisita import ObservedView
from equipo_jar.mapa import Grid, Pose
from equipo_jar.mision import Mision
import equipo_jar.mision as module


class Logger:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def candidate(count=2, identity=7):
    return NS(id=identity, centro=NS(x=2.5, y=1.5), observaciones=count)


def mission(monkeypatch):
    monkeypatch.setattr(module, 'viewpoint_for', lambda *args: None)
    node = NS()
    node.p = dict(visit_transit_radius_m=0.15, visit_free_threshold=25,
                  visit_distance_m=0.8, visit_clearance_m=0.25, visit_angles=12,
                  visit_min_shift_m=0.4, visit_box_margin_m=0.25, visit_max_attempts=3,
                  revisit_enabled=True, revisit_max_age_s=10.0, revisit_min_shift_m=0.05,
                  revisit_wait_s=2.0, revisit_max_records=128,
                  revisit_adjustment_m=0.05,
                  min_report_distance_m=0.4, min_observations=5, settle_s=0.0,
                  return_time_s=480.0)
    node.clock_s = 10.0
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(node.clock_s * 1e9)))
    node.seconds_since_t0 = lambda: node.clock_s
    node.get_logger = lambda: Logger()
    node.grid = Grid(40, 30, 0.1, [0] * 1200)
    node.mapa = object()
    node.current_pose = lambda: (1.55, 1.35, 0.0)
    node.observation_map_revision = 3
    node.observed_views = [ObservedView(Pose(1.55, 1.55, -0.2), 2.5, 1.5, 9.0, 0.07, 3)]
    node.candidatas = [candidate()]
    node.next_doubtful = lambda: node.candidatas[0]
    node.failed_inspections = {}
    node.inspected_points = []
    node.revisit_pending = None
    node.sent = []
    node.make_pose = lambda x, y, yaw: (x, y, yaw)
    node.go_to = lambda pose: node.sent.append(pose) or True
    node.nav_state = 'idle'
    node.current_point = None
    node.visited = []
    node.settle_until = None
    node.plan = None
    node.spins = []
    node.spin_if_idle = lambda: node.spins.append(True)
    node.inspect_doubtful = MethodType(Mision.inspect_doubtful, node)
    node.finish_revisit = MethodType(Mision.finish_revisit, node)
    return node


def sent_revisit(monkeypatch):
    node = mission(monkeypatch)
    assert node.inspect_doubtful()
    node.inspect_doubtful = lambda: False
    return node


def test_failed_ring_search_uses_safe_measured_pose_without_marking_it_inspected(monkeypatch):
    node = sent_revisit(monkeypatch)
    assert node.sent == [(1.55, 1.55, -0.2)]
    assert node.inspected_points == []
    assert node.revisit_pending['before'] == 2
    assert node.failed_inspections == {}


def test_original_ring_success_keeps_its_existing_behavior(monkeypatch):
    node = mission(monkeypatch)
    monkeypatch.setattr(module, 'viewpoint_for', lambda *args: (1.5, 2.0, 0.5))
    assert node.inspect_doubtful()
    assert node.sent == [(1.5, 2.0, 0.5)]
    assert node.inspected_points == [(2.5, 1.5)]
    assert node.revisit_pending is None


@pytest.mark.parametrize('parameter,value', [('revisit_enabled', False), ('revisit_wait_s', -1.0),
                                             ('revisit_wait_s', 6.0), ('revisit_max_age_s', 0.0)])
def test_disabled_or_invalid_fallback_cannot_send_a_goal(monkeypatch, parameter, value):
    node = mission(monkeypatch)
    node.p[parameter] = value
    assert not node.inspect_doubtful()
    assert node.sent == []
    assert node.inspected_points == []


def test_fallback_can_move_an_unsafe_measurement_to_a_nearby_safe_goal(monkeypatch):
    node = mission(monkeypatch)
    values = [0] * (80 * 60)
    for j in range(60):
        values[j * 80 + 40] = 100
    node.grid = Grid(80, 60, 0.05, values)
    node.observed_views = [ObservedView(Pose(1.755, 1.525, -0.2), 2.5, 1.5, 9.0, 0.07, 3)]
    assert node.inspect_doubtful()
    x, y, yaw = node.sent[0]
    assert x < 1.755 and abs(x - 1.755) <= 0.05
    assert y == pytest.approx(1.525) and yaw == -0.2
    assert node.inspected_points == []


def test_wait_starts_after_arrival_not_when_the_goal_is_sent(monkeypatch):
    node = sent_revisit(monkeypatch)
    node.clock_s = 13.0
    node.nav_state = 'arrived'
    Mision.explore(node)
    assert node.settle_until == 15.0
    assert node.spins == []
    assert node.inspected_points == []
    node.clock_s = 14.9
    Mision.explore(node)
    assert node.spins == []
    node.candidatas[0].observaciones = 5
    node.clock_s = 15.0
    Mision.explore(node)
    assert node.inspected_points == [(2.5, 1.5)]
    assert node.revisit_pending is None
    assert node.spins == [True]


def test_wait_parameter_applies_live_and_cannot_create_an_unbounded_pause(monkeypatch):
    node = sent_revisit(monkeypatch)
    node.nav_state = 'arrived'
    Mision.explore(node)
    node.clock_s = 10.5
    node.p['revisit_wait_s'] = 3.0
    Mision.explore(node)
    assert node.settle_until == 13.0
    node.p['revisit_wait_s'] = 100.0
    Mision.explore(node)
    assert node.revisit_pending is None
    assert node.inspected_points == []
    assert node.spins == [True]


@pytest.mark.parametrize('count', [2, 3])
def test_time_or_a_partial_extra_observation_is_not_confirmation(monkeypatch, count):
    node = sent_revisit(monkeypatch)
    node.nav_state = 'arrived'
    Mision.explore(node)
    node.candidatas[0].observaciones = count
    node.clock_s = 12.0
    Mision.explore(node)
    assert node.inspected_points == []
    assert node.failed_inspections == {7: (count, 1)}


def test_failed_navigation_does_not_wait_or_grant_inspection(monkeypatch):
    node = sent_revisit(monkeypatch)
    node.nav_state = 'failed'
    node.candidatas[0].observaciones = 5
    Mision.explore(node)
    assert node.revisit_pending is None
    assert node.inspected_points == []
    assert node.settle_until is None
    assert node.failed_inspections == {7: (5, 1)}


def test_candidate_merge_can_supply_new_evidence_but_another_map_cannot(monkeypatch):
    node = sent_revisit(monkeypatch)
    node.revisit_pending['arrived'] = True
    node.candidatas = [candidate(count=5, identity=8)]
    node.finish_revisit(True)
    assert node.inspected_points == [(2.5, 1.5)]
    node = sent_revisit(monkeypatch)
    node.revisit_pending['arrived'] = True
    node.candidatas[0].observaciones = 5
    node.observation_map_revision += 1
    node.finish_revisit(True)
    assert node.inspected_points == []


def test_report_count_and_deadline_preempt_the_wait_but_empty_plan_does_not(monkeypatch):
    node = sent_revisit(monkeypatch)
    node.reported_points = []
    node.cantidad_victimas = 2
    node.next_doubtful = lambda: None
    node.plan = NS(poses=[])
    node.current_box_zones = lambda: []
    node.p.update(same_point_distance_m=0.02, same_point_angle_rad=0.1)
    assert Mision.reason_to_return(node) is None
    node.clock_s = 480.0
    assert Mision.reason_to_return(node) == 'se termina el tiempo'
    node.reported_points = [(1, 1), (2, 2)]
    assert Mision.reason_to_return(node) == 'todas reportadas'


def observation():
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 9
    msg.pose.pose.position.x = 2.5
    msg.pose.pose.position.y = 1.5
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
    return msg


def test_observer_tf_uses_the_measurement_stamp_and_memory_is_bounded(monkeypatch):
    node = mission(monkeypatch)
    seen = []
    tr = TransformStamped()
    tr.transform.translation.x = 1.55
    tr.transform.translation.y = 1.55
    tr.transform.rotation.w = 1.0
    def lookup(parent, child, stamp):
        seen.append(stamp.nanoseconds)
        return tr
    node.tf_buffer = NS(lookup_transform=lookup)
    node.p['revisit_max_records'] = 1
    for _ in range(3):
        Mision.al_recibir_observacion(node, observation())
    assert seen == [9_000_000_000] * 3
    assert len(node.observed_views) == 1
    assert node.observed_views[0].map_revision == 3
    msg = observation()
    msg.header.stamp.sec = 0
    Mision.al_recibir_observacion(node, msg)
    assert len(seen) == 3


def test_map_change_invalidates_evidence_but_repeated_identical_map_does_not(monkeypatch):
    node = mission(monkeypatch)
    node.mapa = None
    node.grid = None
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 4
    msg.info.resolution = 1.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 16
    Mision.al_recibir_mapa(node, msg)
    revision = node.observation_map_revision
    node.observed_views.append(NS(evidence=True))
    Mision.al_recibir_mapa(node, msg)
    assert node.observation_map_revision == revision
    assert len(node.observed_views) == 1
    msg.data[0] = 100
    Mision.al_recibir_mapa(node, msg)
    assert node.observation_map_revision == revision + 1
    assert node.observed_views == []
