"""Tests for contar_choques.py. No ROS needed:
    python3 -m pytest -q test_contar_choques.py
"""
import json
import math

import pytest

import contar_choques as cc

# A panel as the practice mazes define it (plywood_panel/model.sdf): a 0.98 x 0.6 x 0.02 box,
# stood up with roll and yaw, with its origin at one corner.
PANEL_MODEL = """<?xml version="1.0"?>
<sdf version="1.5"><model name="panel"><static>true</static>
  <link name="main"><pose>0 0 0 0 0 0</pose>
    <collision name="collision"><pose>0.0 -0.5 0.3 -1.571 0 -1.571</pose>
      <geometry><box><size>0.98 0.6 0.02</size></box></geometry></collision>
  </link></model></sdf>
"""


def world(body):
    return f'<?xml version="1.0"?><sdf version="1.6"><world name="w">{body}</world></sdf>'


def wall(name, x, y, sx, sy, link='wall'):
    """A static model with one box link standing on the floor."""
    return (f'<model name="{name}"><static>1</static><link name="{link}">'
            f'<pose>{x} {y} 0.25 0 0 0</pose><collision name="c"><geometry><box>'
            f'<size>{sx} {sy} 0.5</size></box></geometry></collision></link></model>')


def load(tmp_path, body, models=None):
    path = tmp_path / 'test.world'
    path.write_text(world(body))
    dirs = []
    if models:
        for name, text in models.items():
            folder = tmp_path / 'models' / name
            folder.mkdir(parents=True)
            (folder / 'model.sdf').write_text(text)
        dirs.append(tmp_path / 'models')
    loader = cc.WorldLoader(dirs)
    return loader.load(path), loader


def bounds(obstacle):
    xs = [x for x, _ in obstacle.polygon]
    ys = [y for _, y in obstacle.polygon]
    return min(xs), max(xs), min(ys), max(ys)


def drive(counter, poses, dt=0.05, t=0.0):
    """Feed (x, y, yaw) poses at a fixed rate; returns the time after the last one."""
    for x, y, yaw in poses:
        counter.update(t, x, y, yaw)
        t += dt
    return t


# --- reading the world ---

def test_panel_pose_matches_the_maze_definition(tmp_path):
    # Roll and yaw stand the panel up: a 2 cm thick wall along -y from its corner, 0.6 m tall.
    obstacles, _ = load(tmp_path, '<include><uri>model://panel</uri><pose>0 0 0 0 0 0</pose></include>',
                        {'panel': PANEL_MODEL})
    (panel,) = obstacles
    x0, x1, y0, y1 = bounds(panel)
    assert (x0, x1) == (pytest.approx(-0.01, abs=1e-3), pytest.approx(0.01, abs=1e-3))
    assert (y0, y1) == (pytest.approx(-0.99, abs=1e-3), pytest.approx(-0.01, abs=1e-3))
    assert (panel.z_min, panel.z_max) == (pytest.approx(0.0, abs=1e-3), pytest.approx(0.6, abs=1e-3))
    assert panel.kind == 'wall'


def test_include_pose_rotates_the_model(tmp_path):
    # Same panel included at (1, 0) turned 90 degrees, as the mazes lay their x-axis walls.
    obstacles, _ = load(tmp_path, '<include><name>p</name><uri>model://panel</uri>'
                                  '<pose>1 0 0 0 0 1.571</pose></include>', {'panel': PANEL_MODEL})
    x0, x1, y0, y1 = bounds(obstacles[0])
    assert (x0, x1) == (pytest.approx(1.01, abs=2e-3), pytest.approx(1.99, abs=2e-3))
    assert (y0, y1) == (pytest.approx(-0.01, abs=2e-3), pytest.approx(0.01, abs=2e-3))
    assert obstacles[0].name == 'p/main'


def test_nested_include_composes_poses(tmp_path):
    maze = ('<?xml version="1.0"?><sdf version="1.5"><model name="maze">'
            '<include><name>a</name><uri>model://panel</uri><pose>0 0 0 0 0 1.571</pose></include>'
            '</model></sdf>')
    obstacles, _ = load(tmp_path, '<include><uri>model://maze</uri><pose>-1.5 -3.5 0 0 0 0</pose></include>',
                        {'panel': PANEL_MODEL, 'maze': maze})
    x0, x1, y0, _ = bounds(obstacles[0])
    assert (x0, x1, y0) == (pytest.approx(-1.49, abs=2e-3), pytest.approx(-0.51, abs=2e-3),
                            pytest.approx(-3.51, abs=2e-3))
    assert obstacles[0].name == 'maze/a/main'


def test_boxes_floors_and_missing_models(tmp_path):
    body = (wall('marcas_color', 2, 0, 0.25, 0.25, link='cuadrado_rojo_1')
            + '<model name="floor"><link name="l"><collision name="c"><geometry><plane>'
              '<normal>0 0 1</normal><size>8 8</size></plane></geometry></collision></link></model>'
            + '<include><uri>model://nowhere</uri></include>')
    obstacles, loader = load(tmp_path, body)
    assert [o.kind for o in obstacles] == ['box']          # the plane is the floor: ignored
    assert any('nowhere' in s for s in loader.skipped)     # never silently dropped


# --- geometry ---

def test_polygon_distance_apart_touching_and_overlapping():
    square = [(0, 0), (1, 0), (1, 1), (0, 1)]
    moved = [(x + 1.5, y) for x, y in square]
    assert cc.polygon_distance(square, moved) == pytest.approx(0.5)
    assert cc.polygon_distance(square, [(x + 1.0, y) for x, y in square]) == pytest.approx(0.0)
    assert cc.polygon_distance(square, [(x + 0.8, y) for x, y in square]) == pytest.approx(-0.2)


def test_a_wheel_sticks_out_past_the_chassis(tmp_path):
    # Wall face at y = 0.15, robot driving along x at y = 0.035: the chassis side is 4.6 cm
    # away, but the left wheels (spheres) reach y = 0.152 and touch it.
    obstacles, _ = load(tmp_path, wall('w', 0, 0.2, 4, 0.1))
    gap = cc.clearance(0.0, 0.035, 0.0, obstacles[0])
    assert gap == pytest.approx(0.15 - 0.035 - cc.WHEEL_Y_M - cc.WHEEL_RADIUS_M, abs=1e-6)
    assert gap < 0


def test_the_rear_is_longer_than_the_front(tmp_path):
    obstacles, _ = load(tmp_path, wall('w', 1.0, 0, 0.1, 2))     # face at x = 0.95
    front = cc.clearance(0.5, 0.0, 0.0, obstacles[0])
    rear = cc.clearance(0.5, 0.0, math.pi, obstacles[0])
    assert front == pytest.approx(0.95 - 0.5 - cc.CHASSIS_FRONT_M)
    assert rear == pytest.approx(0.95 - 0.5 - cc.CHASSIS_REAR_M)


# --- counting ---

def approach_and_push(stop_x):
    """Drive along x towards the wall, then stay at stop_x for 3 s."""
    poses = [(x / 100, 0.0, 0.0) for x in range(0, int(stop_x * 100))]
    return poses + [(stop_x, 0.0, 0.0)] * 60


def test_pushing_a_wall_for_3_s_is_one_collision(tmp_path):
    obstacles, _ = load(tmp_path, wall('w', 1.0, 0, 0.1, 2))     # face at x = 0.95
    counter = cc.CollisionCounter(obstacles)
    drive(counter, approach_and_push(0.95 - cc.CHASSIS_FRONT_M + 0.002))
    counter.finish()
    s = counter.summary()
    assert (s['collisions'], s['wall_collisions'], s['box_collisions']) == (1, 1, 0)
    assert s['events'][0]['duration_s'] >= 2.9
    assert s['close_calls'] == 0


def test_backing_off_and_hitting_again_is_two_collisions(tmp_path):
    obstacles, _ = load(tmp_path, wall('w', 1.0, 0, 0.1, 2))
    counter = cc.CollisionCounter(obstacles, gap_s=0.5)
    touch = 0.95 - cc.CHASSIS_FRONT_M
    t = drive(counter, [(touch, 0, 0)] * 10)
    t = drive(counter, [(touch - 0.2, 0, 0)] * 20, t=t)            # 1 s away
    drive(counter, [(touch, 0, 0)] * 10, t=t)
    counter.finish()
    assert counter.summary()['collisions'] == 2


def test_a_short_bounce_is_still_one_collision(tmp_path):
    obstacles, _ = load(tmp_path, wall('w', 1.0, 0, 0.1, 2))
    counter = cc.CollisionCounter(obstacles, gap_s=0.5)
    touch = 0.95 - cc.CHASSIS_FRONT_M
    t = drive(counter, [(touch, 0, 0)] * 10)
    t = drive(counter, [(touch - 0.05, 0, 0)] * 4, t=t)             # 0.2 s away
    drive(counter, [(touch, 0, 0)] * 10, t=t)
    counter.finish()
    assert counter.summary()['collisions'] == 1


def test_passing_2_cm_from_a_box_is_a_close_call(tmp_path):
    obstacles, _ = load(tmp_path, wall('marcas_color', 1.0, 0, 0.25, 0.25, link='cuadrado_azul_1'))
    counter = cc.CollisionCounter(obstacles)
    side = 0.125 + cc.WHEEL_Y_M + cc.WHEEL_RADIUS_M + 0.02           # wheels 2 cm from the box
    drive(counter, [(x / 100, side, 0.0) for x in range(0, 200)])
    counter.finish()
    s = counter.summary()
    assert s['collisions'] == 0
    assert s['close_calls'] == 1
    assert s['close_call_events'][0]['min_clearance_m'] == pytest.approx(0.02, abs=1e-3)


def test_hitting_a_box_is_reported_as_a_box(tmp_path):
    obstacles, _ = load(tmp_path, wall('marcas_color', 1.0, 0, 0.25, 0.25, link='cuadrado_rojo_1'))
    counter = cc.CollisionCounter(obstacles)
    drive(counter, approach_and_push(1.0 - 0.125 - cc.CHASSIS_FRONT_M))
    counter.finish()
    s = counter.summary()
    assert (s['collisions'], s['box_collisions']) == (1, 1)
    assert s['events'][0]['objects'] == ['marcas_color/cuadrado_rojo_1']


def test_offline_mode_reads_json_lines(tmp_path):
    path = tmp_path / 'test.world'
    path.write_text(world(wall('w', 1.0, 0, 0.1, 2)))
    poses = tmp_path / 'poses.jsonl'
    with poses.open('w') as f:
        for i, (x, y, yaw) in enumerate(approach_and_push(0.95 - cc.CHASSIS_FRONT_M)):
            f.write(json.dumps({'t': 100 + i * 0.05, 'x': x, 'y': y, 'yaw': yaw}) + '\n')
    out = tmp_path / 'out.json'
    assert cc.main(['--world', str(path), '--poses', str(poses), '--output', str(out)]) == 0
    s = json.loads(out.read_text())
    assert s['collisions'] == 1
    assert s['events'][0]['t_s'] == pytest.approx(0.83 / 0.01 * 0.05, abs=0.06)
