"""Nav2 start-up and the check after it: a respawned node is brought back to active."""
from types import SimpleNamespace

import pytest

lifecycle = pytest.importorskip('lifecycle_msgs')
from lifecycle_msgs.msg import State, Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402

from equipo_jar.localizacion import Arranque  # noqa: E402


class FakeFuture:
    def __init__(self):
        self.value = None
        self.callbacks = []

    def done(self):
        return self.value is not None

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def finish(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


class FakeClient:
    def __init__(self):
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append((request, future))
        return future

    def remove_pending_request(self, future):
        pass


class FakeNode:
    def __init__(self):
        self.clients = {}
        self.warnings = []

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None,
                               warning=lambda msg, **k: self.warnings.append(msg))

    def create_client(self, kind, name):
        self.clients[name] = FakeClient()
        return self.clients[name]


def answer(client, state_id):
    _, future = client.calls[-1]
    future.finish(GetState.Response(current_state=State(id=state_id, label=str(state_id))))


def bring_up(get, change):
    """Answer the usual unconfigured -> configure -> inactive -> activate -> active."""
    answer(get, State.PRIMARY_STATE_UNCONFIGURED)
    change.calls[-1][1].finish(ChangeState.Response(success=True))
    answer(get, State.PRIMARY_STATE_INACTIVE)
    change.calls[-1][1].finish(ChangeState.Response(success=True))
    answer(get, State.PRIMARY_STATE_ACTIVE)


def started(revisar_s=0.5):
    node = FakeNode()
    arranque = Arranque(node, ['planner_server'], 5.0, revisar_s)
    get = node.clients['/planner_server/get_state']
    change = node.clients['/planner_server/change_state']
    arranque.tick()
    bring_up(get, change)
    assert arranque.arrancado and not arranque.faltan
    return node, arranque, get, change


def test_start_up_configures_and_activates():
    _, _, _, change = started()
    assert [r.transition.id for r, _ in change.calls] == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_a_respawned_node_is_brought_back_to_active():
    node, arranque, get, change = started()
    arranque.t_revision -= 1.0
    arranque.tick()                      # periodic check asks the state
    answer(get, State.PRIMARY_STATE_UNCONFIGURED)
    arranque.tick()                      # found down: start-up again
    assert arranque.faltan == ['planner_server']
    assert arranque.recuperados == 1 and node.warnings
    bring_up(get, change)
    assert not arranque.faltan
    assert [r.transition.id for r, _ in change.calls][-2:] == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_an_active_node_is_left_alone():
    _, arranque, get, change = started()
    calls = len(change.calls)
    for _ in range(5):
        arranque.t_revision -= 1.0
        arranque.tick()
        answer(get, State.PRIMARY_STATE_ACTIVE)
        arranque.tick()
    assert not arranque.faltan and len(change.calls) == calls


def test_a_dead_process_is_skipped_until_it_answers():
    _, arranque, get, _ = started()
    get.ready = False
    calls = len(get.calls)
    arranque.t_revision -= 1.0
    arranque.tick()
    assert len(get.calls) == calls and arranque.revision is None


def test_the_check_can_be_turned_off():
    _, arranque, get, _ = started(revisar_s=0.0)
    calls = len(get.calls)
    arranque.t_revision -= 10.0
    arranque.tick()
    assert len(get.calls) == calls
