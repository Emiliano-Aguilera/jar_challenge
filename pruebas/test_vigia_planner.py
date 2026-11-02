"""Planner watch: stale-path detection and the planner_server restart sequence."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

from equipo_jar.localizacion import Arranque
from equipo_jar.vigia_planner import VigiaPlanner

P = {'vigia_planner_distancia_m': 0.5, 'vigia_planner_veces': 3,
     'vigia_planner_espera_s': 30.0}


def test_paths_from_the_robot_never_restart():
    vigia = VigiaPlanner(dict(P))
    for t in range(20):
        assert vigia.con_plan(float(t), (1.0, 1.0), (1.3, 1.2)) is None
    assert vigia.reinicios == 0


def test_three_stale_paths_in_a_row_restart_once_per_wait():
    vigia = VigiaPlanner(dict(P))
    assert vigia.con_plan(0.0, (-1.82, -2.72), (-1.0, -2.0)) is None
    assert vigia.con_plan(1.0, (-1.82, -2.72), (-0.9, -2.0)) is None
    assert vigia.con_plan(2.0, (-1.82, -2.72), (-0.8, -2.0)) == pytest.approx(1.249, abs=0.01)
    for t in range(3, 32):                      # still stale, but within the wait
        assert vigia.con_plan(float(t), (-1.82, -2.72), (1.0, 1.0)) is None
    assert vigia.con_plan(33.0, (-1.82, -2.72), (1.0, 1.0)) is not None
    assert vigia.reinicios == 2


def test_one_good_path_resets_the_count():
    vigia = VigiaPlanner(dict(P))
    vigia.con_plan(0.0, (0.0, 0.0), (1.0, 0.0))
    vigia.con_plan(1.0, (0.0, 0.0), (1.0, 0.0))
    vigia.con_plan(2.0, (1.0, 0.0), (1.0, 0.0))
    assert vigia.con_plan(3.0, (0.0, 0.0), (1.0, 0.0)) is None
    assert vigia.seguidos == 1


@pytest.mark.parametrize('limite', [0.0, -1.0])
def test_off(limite):
    vigia = VigiaPlanner(dict(P, vigia_planner_distancia_m=limite))
    for t in range(10):
        assert vigia.con_plan(float(t), (0.0, 0.0), (5.0, 5.0)) is None


def test_nan_pose_is_ignored():
    vigia = VigiaPlanner(dict(P))
    for t in range(5):
        assert vigia.con_plan(float(t), (0.0, 0.0), (float('nan'), 0.0)) is None


class FakeClient:
    """Lifecycle service client of one fake node: answers at once."""

    def __init__(self, node, kind):
        self.node, self.kind = node, kind

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        if self.kind is GetState:
            response = GetState.Response()
            response.current_state.id = self.node.state
            response.current_state.label = str(self.node.state)
        else:
            self.node.transitions.append(request.transition.id)
            self.node.state = {
                Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
                Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
                Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
                Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
            }[request.transition.id]
            response = ChangeState.Response(success=True)
        self.node.pending.append((future, response))
        return future

    def remove_pending_request(self, future):
        pass


class FakeNav2:
    def __init__(self):
        self.state = State.PRIMARY_STATE_ACTIVE
        self.transitions = []
        self.pending = []
        self.logs = []

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, warning=self.logs.append)

    def create_client(self, kind, name):
        return FakeClient(self, kind)

    def run(self, limit=50):
        """Deliver answers one at a time, as the executor would."""
        for _ in range(limit):
            if not self.pending:
                return
            future, response = self.pending.pop(0)
            future.set_result(response)


def test_restart_goes_down_to_unconfigured_and_back_up():
    nav2 = FakeNav2()
    arranque = Arranque(nav2, ['planner_server'], 5.0)
    arranque.paso()
    nav2.run()
    assert arranque.faltan == [] and nav2.transitions == []
    assert arranque.reiniciar('planner_server')
    nav2.run()
    assert nav2.transitions == [Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP,
                                Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
    assert nav2.state == State.PRIMARY_STATE_ACTIVE
    assert arranque.faltan == [] and not arranque.bajando


def test_no_restart_while_start_up_is_running_or_for_unknown_nodes():
    nav2 = FakeNav2()
    nav2.state = State.PRIMARY_STATE_UNCONFIGURED
    arranque = Arranque(nav2, ['planner_server'], 5.0)
    assert not arranque.reiniciar('planner_server')     # still starting
    arranque.paso()
    nav2.run()
    assert not arranque.reiniciar('controller_server')
    assert arranque.reiniciar('planner_server')
