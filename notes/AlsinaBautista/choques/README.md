# Collision counter (simulator only)

The practice judge counts no collisions: nobody calls `/juez/colision`. In the
competition a person counts every touch, at −50 each. `contar_choques.py` measures
them in the simulator.

## What it does

- Reads the robot's real pose from `/ground_truth/odom`.
- Places the robot's simulator collision shapes there: the chassis box (0.119 m
  front, 0.181 m rear, 0.069 m to each side) and the four wheels, which collide as
  spheres (r 0.0325 m, so they stick out to 0.117 m on each side). Source: the
  simulator's URDF at `jar2026-v1`.
- Checks them against every wall and box of the `.world`, includes resolved.
- A **collision** is a contact (gap ≤ 5 mm) and lasts until 0.5 s without
  contact, so pushing a wall for 20 s counts once. A **close call** is an
  approach within 3 cm that never touched.
- Counts from `/map` on, like the judge.

It reads `/ground_truth/odom`, which team code must never use, so it lives
outside `src/`. Standard library only, so the tests need no ROS.

## Use

```bash
# Live, on the simulator machine, next to the judge (same world name):
python3 contar_choques.py --world maze_1_6x5 --output choques.json
# Offline, from poses in JSON lines ({"t": s, "x": m, "y": m, "yaw": rad}):
python3 contar_choques.py --world path/x.world --models path/models --poses poses.jsonl
# The nearest obstacles at one pose:
python3 contar_choques.py --world maze_1_6x5 --at 0 0 0
```

Live mode logs each collision when it starts, writes the JSON every 5 s and
once more on Ctrl+C. Main fields: `collisions`, `wall_collisions`,
`box_collisions`, `close_calls`, and `events` (time since `/map`, position,
duration, objects touched).

## Limits

- Simulator shapes, not the real robot. The real wheels are narrower than the
  simulator's spheres.
- The practice boxes are static: a push is a contact, the box does not move.
- Only box, cylinder and sphere collisions. Meshes are skipped with a warning
  (the practice worlds have none).

## Checked

- Tests: `python3 -m pytest -q test_contar_choques.py` (13 tests).
- The walls read from the 6 practice worlds match their maps: 99–100 % of the
  occupied map cells are within 10 cm of a wall, and no wall is missing. In
  `laberinto_1` and `laberinto_simple`, the world's walls are up to 7.5 cm
  thicker or shifted compared to what the map draws.
- At the start pose (0, 0) nothing touches. The nearest wall is 0.31 m away in
  the mazes.
