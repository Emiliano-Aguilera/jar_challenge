import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from equipo_jar.exploracion import (
    angle_difference, first_pending, already_visited, yaw_from_quaternion, goal_position,
    box_zones, near_box)

DIST = 0.30
ANG = 0.5


def quaternion(yaw):
    """Cuaternión de un giro `yaw` sobre el eje vertical."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def pose(x, y, yaw):
    """PoseStamped falso: mismos campos que usa first_pending."""
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y), orientation=quaternion(yaw)))


# --- yaw_from_quaternion ---

def test_yaw_no_rotation():
    assert yaw_from_quaternion(quaternion(0.0)) == pytest.approx(0.0)


def test_yaw_quarter_turn():
    assert yaw_from_quaternion(quaternion(math.pi / 2)) == pytest.approx(math.pi / 2)


def test_yaw_negative():
    assert yaw_from_quaternion(quaternion(-1.0)) == pytest.approx(-1.0)


# --- angle_difference ---

def test_difference_simple():
    assert angle_difference(0.1, 0.3) == pytest.approx(0.2)


def test_difference_does_not_depend_on_order():
    assert angle_difference(0.3, 0.1) == pytest.approx(0.2)


def test_difference_wraps_around():
    # 179° y -179° están a 2°, no a 358°.
    assert angle_difference(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(2))


def test_difference_full_turn_is_zero():
    assert angle_difference(0.0, 2 * math.pi) == pytest.approx(0.0, abs=1e-9)


# --- already_visited ---

def test_no_visited():
    assert not already_visited(1.0, 1.0, 0.0, [], DIST, ANG)


def test_same_point_far_from_origin():
    assert already_visited(3.0, 3.0, 0.0, [(3.0, 3.0, 0.0)], DIST, ANG)


def test_near_origin_not_visited():
    assert not already_visited(0.1, 0.1, 0.0, [(5.0, 5.0, 0.0)], DIST, ANG)


def test_almost_equal_counts_as_visited():
    assert already_visited(1.1, 1.0, 0.1, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_same_place_facing_other_way_is_not_visited():
    assert not already_visited(1.0, 1.0, math.pi / 2, [(1.0, 1.0, 0.0)], DIST, ANG)


def test_same_heading_elsewhere_is_not_visited():
    assert not already_visited(2.0, 1.0, 0.0, [(1.0, 1.0, 0.0)], DIST, ANG)


# --- first_pending ---

def test_empty_plan():
    assert first_pending([], [], DIST, ANG, []) is None


def test_no_visited_returns_first():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    assert first_pending(poses, [], DIST, ANG, []) is poses[0]


def test_skips_visited_and_keeps_order():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(3.0, 0.5, 0.0)]
    assert first_pending(poses, [(1.0, 0.5, 0.0)], DIST, ANG, []) is poses[1]


def test_all_visited():
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0)]
    visited = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert first_pending(poses, visited, DIST, ANG, []) is None


def test_same_place_other_heading_is_still_pending():
    # El plan de la prueba del paso 8: los dos últimos puntos están en el mismo lugar.
    poses = [pose(1.0, 0.5, 0.0), pose(2.0, 0.5, 0.0), pose(2.0, 0.5, math.pi / 2)]
    visited = [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)]
    assert first_pending(poses, visited, DIST, ANG, []) is poses[2]


# --- Identidad de una vista del plan de cobertura ---
# Cobertura genera las vistas sobre una grilla fija (centros de celda y múltiplos de
# 360°/heading_count), así que la misma vista sale siempre con las mismas coordenadas.
# Misión las compara con tolerancias que solo filtran redondeo (params.yaml).

PLAN_DIST = 0.02
PLAN_ANG = 0.1
MAP_RESOLUTION = 0.05   # m por celda en los mapas de práctica


def test_coverage_views_25_cm_apart_are_different():
    # Ejemplo de Marco (laberinto_1_start_0): vistas 31 y 34, misma dirección, a 0.25 m.
    yaw = 4.71238898038469
    poses = [pose(-2.675, 2.125, yaw), pose(-2.925, 2.125, yaw)]
    visited = [(-2.675, 2.125, yaw_from_quaternion(quaternion(yaw)))]
    assert first_pending(poses, visited, PLAN_DIST, PLAN_ANG, []) is poses[1]


def test_coverage_headings_45_degrees_apart_are_different():
    poses = [pose(1.025, 0.475, 0.0), pose(1.025, 0.475, math.pi / 4)]
    visited = [(1.025, 0.475, 0.0)]
    assert first_pending(poses, visited, PLAN_DIST, PLAN_ANG, []) is poses[1]


def test_same_view_in_a_new_plan_is_recognized():
    # El mismo punto, publicado otra vez en un plan nuevo y leído desde el cuaternión.
    yaw = 3 * math.pi / 4
    visited = [(1.025, 0.475, yaw_from_quaternion(quaternion(yaw)))]
    assert already_visited(1.025, 0.475, yaw_from_quaternion(quaternion(yaw)),
                           visited, PLAN_DIST, PLAN_ANG)


def test_params_respect_the_coverage_resolution():
    # Regla acordada con cobertura: las tolerancias de misión tienen que ser menores que
    # media celda y que media separación entre direcciones. Si cobertura cambia su grilla
    # o la cantidad de direcciones, este test avisa.
    yaml = pytest.importorskip('yaml')
    path = Path(__file__).resolve().parents[1] / 'params' / 'params.yaml'
    params = yaml.safe_load(path.read_text())
    mision = params['mision']['ros__parameters']
    heading_count = params['cobertura']['ros__parameters']['heading_count']
    assert mision['same_point_distance_m'] < MAP_RESOLUTION / 2
    assert mision['same_point_angle_rad'] < math.pi / heading_count


# --- goal_position: girar en el lugar ---
# Nav2 da por llegado un objetivo a menos de xy_goal_tolerance (0.10 m) y en el giro se
# puede correr un poco. Si misión volviera a pedir el x,y del plan para el giro siguiente,
# Nav2 tendría que acercarse otra vez (laberinto_1: 23 s trabado, notas de Hans).

TURN_RADIUS = 0.20


def test_without_tf_goes_to_the_plan_point():
    assert goal_position(1.0, 2.0, None, TURN_RADIUS) == (1.0, 2.0, False)


def test_robot_near_the_plan_point_turns_in_place():
    # Caso de Hans: el objetivo (-2.17, 2.43) se dio por llegado con el robot a 0.14 m.
    robot = (-2.03, 2.43, 0.5)
    assert goal_position(-2.17, 2.43, robot, TURN_RADIUS) == (-2.03, 2.43, True)


def test_robot_far_from_the_plan_point_travels():
    robot = (1.5, 2.0, 0.0)
    assert goal_position(1.0, 2.0, robot, TURN_RADIUS) == (1.0, 2.0, False)


def test_robot_exactly_at_the_radius_travels():
    # La comparación es estricta (<): justo en el borde, viaja.
    robot = (0.2, 0.0, 0.0)
    assert goal_position(0.0, 0.0, robot, TURN_RADIUS) == (0.0, 0.0, False)


def test_turn_radius_covers_nav2_arrivals():
    # turn_in_place_m tiene que ser mayor que la tolerancia de llegada de Nav2: si no, un
    # robot que Nav2 dio por llegado puede quedar afuera del radio y tener que acercarse
    # otra vez. Si navegación cambia su tolerancia, este test avisa.
    yaml = pytest.importorskip('yaml')
    params_dir = Path(__file__).resolve().parents[1] / 'params'
    params = yaml.safe_load((params_dir / 'params.yaml').read_text())
    mision = params['mision']['ros__parameters']
    nav = yaml.safe_load((params_dir / 'navegacion.yaml').read_text())
    checker = nav['controller_server']['ros__parameters']['general_goal_checker']
    assert mision['turn_in_place_m'] > checker['xy_goal_tolerance']


# --- Zonas alrededor de las cajas ---
# Las cajas no están en /map: un punto del plan puede caer pegado a una. Misión saltea
# los puntos a menos de box_clearance_m + sigma_factor * dispersión de una candidata.

CLEARANCE = 0.42


def box_cand(id_, x, y, observaciones=3, sigma=0.15):
    """Candidata falsa con los campos que usa box_zones (covarianza redonda de `sigma`)."""
    var = sigma ** 2
    return SimpleNamespace(id=id_, centro=SimpleNamespace(x=x, y=y),
                           observaciones=observaciones, covarianza_xy=[var, 0.0, 0.0, var])


def test_zone_radius_adds_the_uncertainty():
    zones = box_zones([box_cand(1, 1.0, 2.0, sigma=0.15)], 2, CLEARANCE, 1.0)
    assert len(zones) == 1
    x, y, radius = zones[0]
    assert (x, y) == (1.0, 2.0)
    assert radius == pytest.approx(CLEARANCE + 0.15)


def test_zone_factor_zero_ignores_the_uncertainty():
    radius = box_zones([box_cand(1, 0.0, 0.0, sigma=0.15)], 2, CLEARANCE, 0.0)[0][2]
    assert radius == pytest.approx(CLEARANCE)


def test_candidate_with_few_observations_makes_no_zone():
    # Una detección suelta puede ser falsa: no tiene que borrar zonas del plan.
    assert box_zones([box_cand(1, 0.0, 0.0, observaciones=1)], 2, CLEARANCE, 1.0) == []


def test_non_finite_center_makes_no_zone():
    assert box_zones([box_cand(1, float('nan'), 0.0)], 2, CLEARANCE, 1.0) == []


def test_near_box():
    zones = [(0.0, 0.0, 0.5)]
    assert near_box(0.3, 0.0, zones)
    assert not near_box(0.6, 0.0, zones)
    assert not near_box(0.5, 0.0, zones)   # justo en el borde: la comparación es estricta
    assert not near_box(0.3, 0.0, [])


def test_first_pending_skips_a_point_next_to_a_box():
    # Caso de Hans en laberinto_simple: vista (0.144, 1.969) con la caja roja en (0.0, 2.0).
    poses = [pose(0.144, 1.969, 0.0), pose(1.0, 1.0, 0.0)]
    zones = box_zones([box_cand(1, 0.0, 2.0, sigma=0.10)], 2, CLEARANCE, 1.0)
    assert first_pending(poses, [], DIST, ANG, zones) is poses[1]


def test_a_skipped_point_is_still_pending():
    # Saltear no es visitar: si fusión corre la caja, el punto vuelve a estar disponible.
    poses = [pose(0.144, 1.969, 0.0)]
    near = box_zones([box_cand(1, 0.0, 2.0)], 2, CLEARANCE, 1.0)
    moved = box_zones([box_cand(1, 0.0, 3.0)], 2, CLEARANCE, 1.0)
    assert first_pending(poses, [], DIST, ANG, near) is None
    assert first_pending(poses, [], DIST, ANG, moved) is poses[0]


def launch_constants(path):
    """Constantes numéricas sueltas de un archivo .py, leídas sin importarlo."""
    values = {}
    for node in ast.parse(path.read_text()).body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Constant)):
            values[node.targets[0].id] = node.value.value
    return values


def test_box_clearance_covers_the_turning_robot_and_the_box():
    # box_clearance_m = radio de la esquina más lejana del robot (al llegar a un objetivo
    # gira en el lugar y barre ese círculo) + media diagonal de la caja (cualquier
    # orientación). Si navegación cambia la forma o el margen del robot, o percepción el
    # tamaño de la caja, este test avisa.
    yaml = pytest.importorskip('yaml')
    base = Path(__file__).resolve().parents[1]
    chassis = launch_constants(base / 'launch' / 'navegacion.launch.py')
    nav = yaml.safe_load((base / 'params' / 'navegacion.yaml').read_text())
    padding = max(nav[c][c]['ros__parameters']['footprint_padding']
                  for c in ('local_costmap', 'global_costmap'))
    params = yaml.safe_load((base / 'params' / 'params.yaml').read_text())
    box_side = params['deteccion']['ros__parameters']['box_side_m']
    corner = math.hypot(max(chassis['ADELANTE'], chassis['ATRAS']) + padding,
                        chassis['MEDIO_ANCHO'] + padding)
    needed = corner + box_side * math.sqrt(2) / 2
    assert params['mision']['ros__parameters']['box_clearance_m'] >= needed
