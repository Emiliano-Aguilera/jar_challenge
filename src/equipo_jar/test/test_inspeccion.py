import math
from types import SimpleNamespace

import pytest

from equipo_jar.inspeccion import pick_doubtful, viewpoint_for
from equipo_jar.mapa import Grid

RES = 0.05

# Valores de params.yaml (visit_distance_m, visit_clearance_m, visit_angles,
# visit_min_shift_m, visit_box_margin_m).
VIEW = dict(distance=0.8, clearance=0.25, n_angles=12, min_shift=0.4, box_margin=0.25)


def make_grid(walls=()):
    """OccupancyGrid falso de 4 x 4 m, origen (0, 0). `walls`: celdas (i, j) ocupadas."""
    width = height = 80
    data = [0] * (width * height)
    for i, j in walls:
        data[j * width + i] = 100
    info = SimpleNamespace(
        resolution=RES, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))
    return SimpleNamespace(info=info, data=data)


def faces(view, cx, cy):
    """¿El yaw del lugar elegido mira hacia la caja?"""
    x, y, yaw = view
    return yaw == pytest.approx(math.atan2(cy - y, cx - x))


# Pared vertical en x de 2.00 a 2.05 (columna 40), de punta a punta.
WALL = [(40, j) for j in range(80)]
# La misma pared con una puerta de 1 m arriba (y de 3 a 4 m): se puede pasar al otro lado.
WALL_WITH_DOOR = [(40, j) for j in range(60)]

# Valores de params.yaml (visit_transit_radius_m, visit_free_threshold).
TRANSIT_RADIUS = 0.15
FREE_THRESHOLD = 25


def reach(grid, robot_x, robot_y):
    """Grid de mapa.py y celdas alcanzables desde el robot, como los arma misión."""
    cell_grid = Grid(grid.info.width, grid.info.height, RES, list(grid.data))
    reachable = cell_grid.reachable_from(robot_x, robot_y, TRANSIT_RADIUS, FREE_THRESHOLD)
    return dict(cell_grid=cell_grid, reachable=reachable)


def every_cell(grid):
    """Sin filtro de camino: todas las celdas cuentan como alcanzables."""
    cell_grid = Grid(grid.info.width, grid.info.height, RES, list(grid.data))
    cells = frozenset((i, j) for i in range(grid.info.width) for j in range(grid.info.height))
    return dict(cell_grid=cell_grid, reachable=cells)


# --- viewpoint_for ---

def test_free_map_picks_the_spot_on_the_robot_side():
    # Caja en (2, 2), robot a la izquierda: el punto del círculo más cercano es (1.2, 2).
    grid = make_grid()
    x, y, yaw = viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **VIEW, **reach(grid, 0.5, 2.0))
    assert (x, y) == (pytest.approx(1.2), pytest.approx(2.0))
    assert yaw == pytest.approx(0.0)


def test_skips_the_spot_where_the_robot_already_is():
    # Mirarla desde donde ya está no le da una vista nueva a fusión.
    grid = make_grid()
    view = viewpoint_for(grid, 2.0, 2.0, 1.2, 2.0, **VIEW, **reach(grid, 1.2, 2.0))
    assert math.hypot(view[0] - 1.2, view[1] - 2.0) >= VIEW['min_shift']
    assert faces(view, 2.0, 2.0)


def test_does_not_look_through_a_wall():
    # Caja a la derecha de la pared, robot a la izquierda: el lugar tiene que quedar a la
    # derecha. La pared tiene puerta, así que el robot puede llegar.
    grid = make_grid(WALL_WITH_DOOR)
    view = viewpoint_for(grid, 2.6, 2.0, 0.5, 2.0, **VIEW, **reach(grid, 0.5, 2.0))
    assert view[0] > 2.05 + VIEW['clearance']
    assert faces(view, 2.6, 2.0)


def test_box_near_the_map_edge_gets_a_spot_inside():
    grid = make_grid()
    view = viewpoint_for(grid, 2.0, 0.3, 2.0, 3.0, **VIEW, **reach(grid, 2.0, 3.0))
    assert view[1] > 0.3
    assert faces(view, 2.0, 0.3)


def test_no_spot_returns_none():
    # Tan cerca de la pared que el robot no cabe en ningún punto del círculo.
    grid = make_grid(WALL)
    assert viewpoint_for(grid, 2.1, 2.0, 0.5, 2.0, **{**VIEW, 'distance': 0.1},
                         **reach(grid, 0.5, 2.0)) is None


def test_box_drawn_in_the_map_is_not_taken_as_a_wall():
    # Si el /map dibuja la caja (0.25 x 0.25 m alrededor de (2, 2)), box_margin hace que
    # la línea termine antes de la caja. Sin ese margen, la caja tapa todas las líneas.
    box = [(i, j) for i in range(37, 43) for j in range(37, 43)]
    grid = make_grid(box)
    assert viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **VIEW, **reach(grid, 0.5, 2.0)) is not None
    assert viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **{**VIEW, 'box_margin': 0.0},
                         **reach(grid, 0.5, 2.0)) is None


# Cuarto cerrado de x 0.85 a 1.60 m e y 1.65 a 2.40 m (columnas 17-31, filas 33-47), con
# una ranura de 10 cm (filas 39-40) en la pared de la derecha. Por la ranura se ve la caja
# en (2, 2), pero el robot (0.42 m de ancho con margen) no pasa. Es el caso del bolsillo
# de laberinto_1.
POCKET = ([(i, j) for i in range(17, 32) for j in (33, 47)]
          + [(i, j) for i in (17, 31) for j in range(33, 48) if not (i == 31 and j in (39, 40))])


def test_skips_the_nearest_spot_when_it_cannot_be_reached():
    grid = make_grid(POCKET)
    # Sin filtro de camino gana (1.2, 2), adentro del cuarto: el más cercano al robot.
    x, y, _ = viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **VIEW, **every_cell(grid))
    assert (x, y) == (pytest.approx(1.2), pytest.approx(2.0))
    # Con el filtro elige uno más lejos, afuera del cuarto.
    view = viewpoint_for(grid, 2.0, 2.0, 0.5, 2.0, **VIEW, **reach(grid, 0.5, 2.0))
    x, y, _ = view
    assert not (0.85 <= x <= 1.60 and 1.65 <= y <= 2.40)
    assert math.hypot(x - 0.5, y - 2.0) > 0.7
    assert faces(view, 2.0, 2.0)


def test_returns_none_when_no_spot_can_be_reached():
    # Pared de punta a punta: la caja solo se ve desde la derecha y el robot está a la izquierda.
    grid = make_grid(WALL)
    assert viewpoint_for(grid, 2.6, 2.0, 0.5, 2.0, **VIEW, **every_cell(grid)) is not None
    assert viewpoint_for(grid, 2.6, 2.0, 0.5, 2.0, **VIEW, **reach(grid, 0.5, 2.0)) is None


def test_robot_inside_a_wall_raises():
    # Misión atrapa este error y deja la inspección para más tarde, sin contarla como intento.
    grid = make_grid(WALL)
    with pytest.raises(ValueError):
        reach(grid, 2.02, 2.0)


# --- pick_doubtful ---

MIN_OBS = 2
DIST_MIN = 0.4
MAX_ATTEMPTS = 3


def cand(id_, x, y, observaciones=3):
    """Candidata falsa con los mismos campos que equipo_jar_msgs/Candidata."""
    return SimpleNamespace(id=id_, centro=SimpleNamespace(x=x, y=y), observaciones=observaciones)


def pick(cands, reported_ids=None, reported_points=None, inspected_points=None, failed=None):
    # El robot está en (0, 0).
    return pick_doubtful(cands, reported_ids or set(), reported_points or [],
                         inspected_points or [], MIN_OBS, DIST_MIN, 0.0, 0.0,
                         failed or {}, MAX_ATTEMPTS)


TWO = [cand(1, 1.0, 0.0), cand(2, 3.0, 0.0)]


def test_picks_the_nearest():
    assert pick(TWO).id == 1


def test_skips_a_reported_id():
    assert pick(TWO, reported_ids={1}).id == 2


def test_skips_the_same_box_reported_with_another_id():
    assert pick(TWO, reported_points=[(1.1, 0.1)]).id == 2


def test_skips_one_already_visited():
    assert pick(TWO, inspected_points=[(0.9, 0.0)]).id == 2


def test_skips_one_with_few_observations():
    assert pick([cand(1, 1.0, 0.0, observaciones=1), cand(2, 3.0, 0.0)]).id == 2


def test_skips_a_non_finite_center():
    assert pick([cand(1, float('nan'), 0.0), cand(2, 3.0, 0.0)]).id == 2


def test_nothing_left_returns_none():
    assert pick(TWO, reported_ids={1, 2}) is None
    assert pick([]) is None


def test_does_not_modify_the_lists_it_gets():
    reported_ids, reported_points, inspected_points = {9}, [(5.0, 5.0)], [(6.0, 6.0)]
    pick(TWO, reported_ids, reported_points, inspected_points)
    assert reported_ids == {9}
    assert reported_points == [(5.0, 5.0)]
    assert inspected_points == [(6.0, 6.0)]


# --- Intentos fallidos (sin lugar para mirarla) ---
# `failed` guarda, por id, (observaciones en el último intento, cantidad de intentos).
# Una candidata fallida vuelve solo si tiene observaciones nuevas y le quedan intentos.

def test_skips_a_failed_one_without_new_observations():
    # Mismas 3 observaciones que en el intento: viewpoint_for daría lo mismo.
    assert pick(TWO, failed={1: (3, 1)}).id == 2


def test_retries_a_failed_one_with_new_observations():
    cands = [cand(1, 1.0, 0.0, observaciones=4), cand(2, 3.0, 0.0)]
    assert pick(cands, failed={1: (3, 1)}).id == 1


def test_skips_a_failed_one_at_max_attempts():
    cands = [cand(1, 1.0, 0.0, observaciones=10), cand(2, 3.0, 0.0)]
    assert pick(cands, failed={1: (3, MAX_ATTEMPTS)}).id == 2


def test_a_failed_one_does_not_block_the_return():
    # Sin evidencia nueva no queda ninguna dudosa: la vuelta por "no quedan puntos
    # ni dudosas" puede salir.
    assert pick([cand(1, 1.0, 0.0)], failed={1: (3, 1)}) is None


def test_failed_is_by_id_not_by_position():
    # Otra candidata en el mismo lugar, con otro id, no queda salteada.
    assert pick([cand(5, 1.0, 0.0)], failed={1: (3, 1)}).id == 5


def test_does_not_modify_failed():
    failed = {1: (3, 1)}
    pick(TWO, failed=failed)
    assert failed == {1: (3, 1)}
