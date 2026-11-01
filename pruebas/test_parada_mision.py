"""Terminal navigation failures must close the velocity path before /done."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from std_msgs.msg import Bool

from test_historial_mision import mission
from test_cancelar_inspeccion_reportada import Goal


class PendingNavigation:
    def __init__(self):
        self.response = Future()
        self.calls = 0

    def server_is_ready(self):
        return True

    def send_goal_async(self, *args, **kwargs):
        self.calls += 1
        return self.response


@pytest.mark.parametrize('state', ['exploring', 'returning'])
def test_lost_acceptance_stops_within_five_seconds(mission, monkeypatch, state):
    node, clock, _ = mission
    nav = PendingNavigation()
    monkeypatch.setattr(node, 'navegacion', nav)
    node.state = state
    assert node.go_to(node.make_pose(1.0, 1.0, 0.0))
    clock.seconds += 5.1
    node.step_logic()
    assert node.state == 'stopping'
    assert nav.calls == 1
    late = Goal()
    nav.response.set_result(late)
    assert late.cancel_calls == 1
    assert node.state == 'stopping'
    assert node.nav_state != 'moving'
    assert not node.go_to(node.make_pose(2.0, 1.0, 0.0))


def test_final_deadline_never_publishes_done_without_stop_confirmation(mission, monkeypatch):
    node, clock, _ = mission
    done = []
    monkeypatch.setattr(node, 'pub_done', SimpleNamespace(publish=done.append))
    node.nav_state = 'moving'
    node.current_goal = Goal()
    node.change_state('stopping')
    clock.seconds = 5.0 + node.p['done_limit_s'] + 1.0
    node.step_logic()
    node.step_logic()
    assert node.state == 'stopping'
    assert done == []


def test_confirmed_velocity_inhibit_can_finish_despite_hung_nav2(mission, monkeypatch):
    node, clock, _ = mission
    done, stop = [], []
    monkeypatch.setattr(node, 'pub_done', SimpleNamespace(publish=done.append))
    monkeypatch.setattr(node, 'pub_stop', SimpleNamespace(publish=stop.append))
    node.current_goal = Goal()
    node.nav_state = 'moving'
    node.change_state('stopping')
    assert stop and stop[-1].data
    node.step_logic()
    assert node.state == 'stopping'
    clock.seconds += node.p['stop_wait_s'] + 0.1
    node.on_stop_confirmed(Bool(data=True))
    node.step_logic()
    node.step_logic()
    assert node.state == 'finished'
    assert len(done) == 1
    node.step_logic()
    assert len(done) == 1


def test_cancel_exception_cannot_prevent_velocity_inhibit(mission, monkeypatch):
    node, _, _ = mission
    stop = []
    monkeypatch.setattr(node, 'pub_stop', SimpleNamespace(publish=stop.append))
    goal = Goal()
    goal.cancel_error = RuntimeError('Nav2 unavailable')
    node.current_goal = goal
    node.nav_state = 'moving'
    node.change_state('stopping')
    node.step_logic()
    node.step_logic()
    assert stop[-1].data
    assert goal.cancel_calls == 1
    assert node.state == 'stopping'


def test_acceptance_just_before_timeout_keeps_normal_navigation(mission, monkeypatch):
    node, clock, _ = mission
    nav = PendingNavigation()
    monkeypatch.setattr(node, 'navegacion', nav)
    assert node.go_to(node.make_pose(1.0, 1.0, 0.0))
    clock.seconds += 4.9
    goal = Goal()
    nav.response.set_result(goal)
    node.step_logic()
    assert node.state == 'exploring'
    assert node.nav_state == 'moving'
    assert goal.cancel_calls == 0


def test_synchronous_send_failure_closes_navigation_output(mission, monkeypatch):
    node, _, _ = mission
    nav = PendingNavigation()
    def unavailable(*args, **kwargs):
        raise RuntimeError('action transport unavailable')
    nav.send_goal_async = unavailable
    monkeypatch.setattr(node, 'navegacion', nav)
    assert not node.go_to(node.make_pose(1.0, 1.0, 0.0))
    assert node.state == 'stopping'


def test_stale_or_revoked_ack_cannot_publish_done(mission, monkeypatch):
    node, clock, _ = mission
    done = []
    monkeypatch.setattr(node, 'pub_done', SimpleNamespace(publish=done.append))
    node.change_state('stopping')
    node.on_stop_confirmed(Bool(data=True))
    clock.seconds += 2.0
    node.step_logic()
    assert node.state == 'stopping'
    node.on_stop_confirmed(Bool(data=True))
    node.step_logic()
    assert node.state == 'finishing'
    node.on_stop_confirmed(Bool(data=False))
    node.step_logic()
    assert node.state == 'stopping'
    assert done == []


def test_asynchronous_acceptance_error_also_closes_output(mission, monkeypatch):
    node, _, _ = mission
    nav = PendingNavigation()
    monkeypatch.setattr(node, 'navegacion', nav)
    assert node.go_to(node.make_pose(1.0, 1.0, 0.0))
    nav.response.set_exception(RuntimeError('acceptance response lost'))
    assert node.state == 'stopping'
    assert not node.go_to(node.make_pose(2.0, 1.0, 0.0))


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -1.0, 0.0, True, 60.0])
def test_invalid_acceptance_timeout_stops_instead_of_waiting_forever(mission, monkeypatch, invalid):
    node, _, _ = mission
    node.p['goal_accept_timeout_s'] = invalid
    monkeypatch.setattr(node, 'navegacion', PendingNavigation())
    node.go_to(node.make_pose(1.0, 1.0, 0.0))
    node.step_logic()
    assert node.state == 'stopping'


def test_accepted_goal_with_unavailable_result_transport_stops(mission, monkeypatch):
    node, _, _ = mission
    nav = PendingNavigation()
    monkeypatch.setattr(node, 'navegacion', nav)
    node.go_to(node.make_pose(1.0, 1.0, 0.0))
    goal = Goal()
    def unavailable():
        raise RuntimeError('result transport unavailable')
    goal.get_result_async = unavailable
    nav.response.set_result(goal)
    assert node.state == 'stopping'
    assert not node.go_to(node.make_pose(2.0, 1.0, 0.0))
