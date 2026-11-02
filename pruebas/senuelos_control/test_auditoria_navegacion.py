"""Catch false confirmations of geometry and unreachable-view evidence."""
import math
import json

import pytest

import auditoria_navegacion as audit


RAW = [[.119, .1], [.119, -.1], [-.181, -.1], [-.181, .1]]
PADDED = [[.139, .12], [.139, -.12], [-.201, -.12], [-.201, .12]]


def snapshot():
    return {'parameters': {'footprint': str(RAW), 'footprint_padding': .02,
                           'robot_base_frame': 'base_footprint'},
            'published': {'points_base': PADDED, 'age_s': .1, 'source_age_s': .1}}


def test_confirms_asymmetric_rectangle_in_either_vertex_order():
    data = snapshot()
    data['published']['points_base'] = PADDED[::-1]
    result = audit.check_costmap(data)
    assert result['status'] == 'confirmed'
    assert result['bounds_m'] == pytest.approx([-.201, .139, -.12, .12])
    assert result['turn_radius_m'] == pytest.approx(.234096133, abs=1e-8)


@pytest.mark.parametrize('points', [
    [[.17, .12], [.17, -.12], [-.17, -.12], [-.17, .12]],
    RAW,
    [[.139, 0], [0, -.12], [-.201, 0], [0, .12]],
    [[.139, .12]] * 4,
])
def test_rejects_recentered_unpadded_or_nonrectangular_publication(points):
    data = snapshot()
    data['published']['points_base'] = points
    assert audit.check_costmap(data)['status'] == 'mismatch'


@pytest.mark.parametrize('mutation', ['missing', 'stale', 'old_stamp', 'nan', 'padding', 'raw'])
def test_missing_stale_or_wrong_parameters_cannot_confirm(mutation):
    data = snapshot()
    if mutation == 'missing':
        data.pop('published')
    elif mutation == 'stale':
        data['published']['age_s'] = 5.0
    elif mutation == 'old_stamp':
        data['published']['source_age_s'] = 20.0
    elif mutation == 'nan':
        data['published']['points_base'] = [[float('nan'), 0]] * 4
    elif mutation == 'padding':
        data['parameters']['footprint_padding'] = .0
    else:
        data['parameters']['footprint'] = '[]'
    assert audit.check_costmap(data)['status'] != 'confirmed'


def test_polygon_is_transformed_from_its_own_stamped_frame():
    # A 90-degree rotation and translation of the hand-derived padded rectangle.
    world = [[1.88, 3.139], [2.12, 3.139], [2.12, 2.799], [1.88, 2.799]]
    for actual, expected in zip(audit.points_in_base(world, (2, 3, math.pi / 2)), PADDED):
        assert actual == pytest.approx(expected)


def test_summary_does_not_turn_action_success_into_collision_free_proof():
    result = audit.trial_summary([1, 2, math.pi], 4, (1.06, 2.08, -math.pi + .2),
                                 (1.3, 2, math.pi), 2)
    assert result['navigation_status'] == 'succeeded'
    assert result['localized_error']['xy_m'] == pytest.approx(.1)
    assert result['localized_error']['yaw_rad'] == pytest.approx(.2)
    assert result['true_error']['xy_m'] == pytest.approx(.3)
    assert result['recoveries'] == 2
    assert result['collision_free'] is None


def test_missing_truth_and_unknown_result_remain_unknown():
    result = audit.trial_summary([0, 0, 0], None, None, None, None)
    assert result['navigation_status'] == 'unknown'
    assert result['true_error'] is None
    assert result['localized_error'] is None
    assert result['recoveries'] is None


def test_invalid_sensor_values_are_preserved_as_invalid_json_evidence():
    payload = {'pose': [float('nan'), float('inf'), -float('inf')], 'padding': .02}
    decoded = json.loads(audit.evidence_json(payload))
    assert decoded['pose'] == [{'nonfinite': 'nan'}, {'nonfinite': 'inf'}, {'nonfinite': '-inf'}]
    assert decoded['padding'] == .02


def test_mppi_and_behavior_wiring_are_checked_separately_from_geometry():
    params = {
        'controller_server': {'FollowPath.plugin': 'nav2_mppi_controller::MPPIController',
                              'FollowPath.CostCritic.enabled': True,
                              'FollowPath.CostCritic.consider_footprint': True,
                              'FollowPath.critics': ['CostCritic']},
        'behavior_server': {'costmap_topic': 'local_costmap/costmap_raw',
                            'footprint_topic': 'local_costmap/published_footprint',
                            'robot_base_frame': 'base_footprint'},
    }
    assert audit.check_consumers(params)['status'] == 'confirmed'
    params['behavior_server']['footprint_topic'] = 'other/published_footprint'
    assert audit.check_consumers(params)['status'] == 'mismatch'
    params.pop('controller_server')
    assert audit.check_consumers(params)['status'] != 'confirmed'
