# Replaying a server recording through deteccion + fusion

Offline test bench for perception: the same recording (MCAP from a `simular.yml` batch with
`grabar=true`, or a live session) is replayed through the code of this checkout and, optionally,
through a baseline checkout, and both outputs are scored against the practice world's boxes.
Needs Docker with the `jar-dev` image (ROS 2 Humble + rosbag2 mcap), the bag in a folder of its
own, and `colcon build` done in each checkout.

```bash
REPLAY_DIR=~/jar_replay BASELINE=~/jar_baseline \
  notes/SantosBunge/replay/compare.sh maze_3_6x6 ~/bags/maze_3 m3           # baseline + new
notes/SantosBunge/replay/compare.sh maze_3_6x6 ~/bags/maze_3 m3_abl "cloud_check=true max_range_m=99.0"  # new only, params changed
```

- `replay_bag.sh`: plays only the sensors, TF and the map; the nodes run on the bag's clock and
  publish on `/replay/*` (the bag's own `/jar/*` topics are latched and would otherwise leak in).
- `eval_replay.py`: per red box, error of every observation; final candidates with the two
  mission rules (normal: n >= 5 and std <= 0.10; final round: n >= 1 and std <= 0.30); a
  candidate farther than 0.5 m from every red box is FALSE. Ground truth is used offline only.
- A params YAML wins over `-p` on the command line in Humble: the ablations write a whole file.
- `ros2 run` leaves its child alive after `kill -9`: the script kills the node processes by name.
