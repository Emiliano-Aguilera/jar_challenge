import math

import pytest

from equipo_jar.mapa import Grid
from equipo_jar.vuelta import nearest_cell, return_leg, steps_to_start, way_back, way_back_m

RES = 0.05
# Valores de params.yaml (visit_transit_radius_m, visit_free_threshold, return_snap_m).
RADIUS = 0.15
THRESHOLD = 25
SNAP = 0.3


def u_map(closed=False):
    """Mapa de 4 x 2 m en forma de U, origen (0, 0).

    Bordes ocupados. Una pared en y de 0.95 a 1.05 va desde x = 0 hasta x = 3.0, así que
    los dos pasillos (abajo y arriba) se unen solo por la derecha. Desde (0.5, 1.5) la
    largada (0.5, 0.5) está a 1 m en línea recta pero a ~7 m por el camino.
    `closed`: la pared llega hasta el borde derecho y los pasillos quedan separados.
    """
    width, height = 80, 40
    data = [0] * (width * height)
    for j in range(height):
        for i in range(width):
            edge = i in (0, width - 1) or j in (0, height - 1)
            wall = j in (19, 20) and (i < 60 or closed)
            if edge or wall:
                data[j * width + i] = 100
    return Grid(width, height, RES, data)


START = (0.5, 0.5)
ROBOT = (0.5, 1.5)


@pytest.fixture(scope='module')
def grid():
    return u_map()


@pytest.fixture(scope='module')
def steps(grid):
    return steps_to_start(grid, *START, RADIUS, THRESHOLD, SNAP)


def steps_at(grid, steps, x, y):
    return steps[nearest_cell(steps, grid.cell(x, y), 6)]


# --- way_back_m: largo de la vuelta en /map ---

def test_the_way_back_goes_around_the_wall(grid, steps):
    # 1 m en línea recta, pero hay que ir hasta la abertura y volver: unos 7 m.
    length = way_back_m(grid, steps, *ROBOT, SNAP)
    assert 6.0 < length < 8.0


def test_the_way_back_is_zero_at_the_start(grid, steps):
    assert way_back_m(grid, steps, *START, SNAP) == 0.0


def test_no_way_back_without_a_connection():
    closed = u_map(closed=True)
    steps = steps_to_start(closed, *START, RADIUS, THRESHOLD, SNAP)
    assert way_back_m(closed, steps, *ROBOT, SNAP) is None
    assert return_leg(closed, steps, *ROBOT, 2.0, SNAP) is None


def test_start_close_to_a_wall_still_works(grid):
    # Largada a 0.10 m del borde, más cerca que el radio: se toma la celda segura más cercana.
    steps = steps_to_start(grid, 0.5, 0.15, RADIUS, THRESHOLD, SNAP)
    assert steps
    assert way_back_m(grid, steps, *ROBOT, SNAP) is not None


def test_start_inside_a_wall_gives_nothing(grid):
    # Más lejos que return_snap_m de cualquier celda segura: no inventa una largada.
    assert steps_to_start(grid, -2.0, -2.0, RADIUS, THRESHOLD, SNAP) == {}


# --- way_back: celdas hasta la largada ---

def test_way_back_ends_at_the_start_one_step_at_a_time(grid, steps):
    here = nearest_cell(steps, grid.cell(*ROBOT), 6)
    path = way_back(steps, here)
    assert steps[path[-1]] == 0
    for a, b in zip(path, path[1:]):
        assert steps[b] == steps[a] - 1
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


# --- return_leg: el próximo tramo ---

def test_leg_is_leg_m_along_the_way_back(grid, steps):
    x, y, _ = return_leg(grid, steps, *ROBOT, 2.0, SNAP)
    before = steps_at(grid, steps, *ROBOT)
    after = steps_at(grid, steps, x, y)
    assert (before - after) * RES == pytest.approx(2.0, abs=RES)


def test_leg_follows_the_corridor_not_the_straight_line(grid, steps):
    # El tramo sigue por el pasillo de arriba (y > 1.05) hacia la abertura (x crece),
    # no atraviesa la pared hacia la largada.
    x, y, _ = return_leg(grid, steps, *ROBOT, 2.0, SNAP)
    assert y > 1.05
    assert x > ROBOT[0] + 1.0


def test_leg_is_never_longer_than_leg_m(grid, steps):
    # Lo que se mueve en /map por el camino nunca es menos que la línea recta.
    for leg_m in (1.0, 2.0, 5.0):
        x, y, _ = return_leg(grid, steps, *ROBOT, leg_m, SNAP)
        assert math.hypot(x - ROBOT[0], y - ROBOT[1]) <= leg_m + RES


def test_leg_faces_along_the_way_back(grid, steps):
    # En el pasillo de arriba, yendo hacia la abertura: mira hacia +x.
    _, _, yaw = return_leg(grid, steps, *ROBOT, 1.0, SNAP)
    assert abs(yaw) < 0.3


def test_no_leg_when_the_start_is_close(grid, steps):
    # Con el resto del camino más corto que un tramo, misión va directo a la largada.
    assert return_leg(grid, steps, 0.9, 0.5, 2.0, SNAP) is None
    assert return_leg(grid, steps, *ROBOT, 10.0, SNAP) is None


def test_no_leg_shorter_than_a_cell(grid, steps):
    assert return_leg(grid, steps, *ROBOT, 0.0, SNAP) is None


def test_blocked_end_shortens_the_leg(grid, steps):
    free = return_leg(grid, steps, *ROBOT, 2.0, SNAP)
    # Caja (u obstáculo del costmap) justo donde terminaba el tramo.
    box = lambda x, y: math.hypot(x - free[0], y - free[1]) < 0.3
    x, y, _ = return_leg(grid, steps, *ROBOT, 2.0, SNAP, box)
    assert not box(x, y)
    assert steps_at(grid, steps, x, y) > steps_at(grid, steps, *free[:2])


def test_everything_blocked_gives_no_leg(grid, steps):
    assert return_leg(grid, steps, *ROBOT, 2.0, SNAP, lambda x, y: True) is None


def test_legs_one_after_another_reach_the_start(grid, steps):
    # Como en misión: tramo, el próximo desde donde llegó, y el último trecho directo.
    x, y = ROBOT
    legs = 0
    while True:
        leg = return_leg(grid, steps, x, y, 2.0, SNAP)
        if leg is None:
            break
        assert steps_at(grid, steps, *leg[:2]) < steps_at(grid, steps, x, y)
        x, y = leg[:2]
        legs += 1
        assert legs < 10
    assert legs == 3
    assert way_back_m(grid, steps, x, y, SNAP) <= 2.0
