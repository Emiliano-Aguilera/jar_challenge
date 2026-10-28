# Interfaces between modules

What the modules pass to each other: enough for each module to be built
separately from day one. The official interface with the robot and the judge
is in the rules (§7.1), in [`docs/QOS.md`](../QOS.md) and in the root
`CLAUDE.md`.

## Summary

| From | To | What | Topic or action | Type |
| --- | --- | --- | --- | --- |
| Localization | All | Robot pose in the map | TF `map → odom` | `tf2` |
| Localization | Mission, Detection | Pose trusted (start-up done, no turn waiting for a fix) | `/jar/localizacion_lista` | `std_msgs/Bool` |
| Coverage | Mission | Pending viewpoints, in order | `/jar/plan_cobertura` | `nav_msgs/Path` |
| Mission | Navigation | Go to a pose | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Detection | Fusion | What it sees, frame by frame | defined by the perception owners | — |
| Fusion | Mission | Victim candidates | `/jar/candidatas` | `equipo_jar_msgs/Candidatas` (ours) |
| Mission | Judge | Reports and end | `/victimas`, `/done` | official |
| Navigation | Robot | Velocity | `/cmd_vel` | official |
| Navigation | Mission | Obstacles seen by the LiDAR (boxes, decoys) on top of `/map` | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` |

Anything else a module needs from another is added when it's needed, through
the change process at the end.

## Rules for everyone

- **Internal topics use the `/jar/` prefix.** The CI auditor fails any team
  node that uses a non-official topic the simulator also uses; the prefix
  avoids that.
- **QoS:** sensors (`/scan`, `/cam_1/*`) with `qos_profile_sensor_data`.
  Internal topics *reliable*, plus *transient local* with depth 1 for the ones
  that hold a state (plan, candidates), so a node that starts late gets the
  last message.
- **Frames:** every position in the maze is in `map`.
- **One publisher per official output:** navigation publishes `/cmd_vel`;
  mission publishes `/victimas` and `/done` and is the only client of
  navigation.
- **Parameters** go in `params/params.yaml` under each node that uses them.
  Names can't contain `victim`, `coord`, `spawn` or `posicion_`.
- **Tunable parameters apply live:** read them through a callback registered
  with `add_on_set_parameters_callback`, not only at startup. Why: in a live
  simulation session you change a value from the viewer and see the effect at
  once, without restarting the node or the run.

## Localization → all

**TF `map → odom`.** With it any node transforms what it sees into `map`.

**`/jar/localizacion_lista`** (`std_msgs/Bool`, reliable + transient local, on
change and once per second):

- `false` until AMCL has settled at the start (no-motion updates done), and
  after each turn of at least `giro_minimo_rad` (0.3 rad) until AMCL has used
  `actualizaciones_giro` (5) scans taken after the turn. At most
  `max_correccion_s` (3 s) after the turn ends.
- Why: on the real robot `/odom` counts turns ~25 % too much, so right after a
  turn the heading can be 10-20° off (a box seen 1.5 m away lands ~0.4 m off).
  Week-10 bag replay: when the flag turns true again the heading error is ~1°.
- Mission: fix the start pose the first time it is `true`; after a trip, wait
  while it is `false` (with a max wait) before the next goal or a report.
- Detection: skip frames taken while it is `false`.
- `actualizaciones_giro: 0` turns the fix off (the flag then only covers the
  start). The simulator has no odometry error, but the fix also works there.

## Coverage → Mission

**`/jar/plan_cobertura`** (`nav_msgs/Path`, frame `map`): the viewpoints still
pending, in visiting order. Each pose has an orientation, because the camera
only sees 61°. Each message replaces the previous one.

## Mission → Navigation

**Nav2's `NavigateToPose` action** (`bt_navigator`): a goal pose in `map`, with
orientation. Mission can cancel it.

- Default tree: `behavior_trees/navegar.xml` (Humble's, arrives within 0.10 m
  and 0.25 rad).
- Trip back to the start: set the goal's `behavior_tree` to the installed
  `share/equipo_jar/behavior_trees/vuelta_precisa.xml`
  (`get_package_share_directory('equipo_jar')`). It arrives within 0.05 m, any
  heading (`precise_goal_checker`). The return bonus needs the 4 wheels inside
  a 0.25 m circle.

## Navigation → Mission

**`/global_costmap/costmap`** (`nav_msgs/OccupancyGrid`, frame `map`, reliable
+ transient local, full grid at 1 Hz): `/map` plus what the LiDAR marks
(boxes, decoys), inflated. Values are Nav2's 0-255 costs scaled to the grid:
100 = lethal (obstacle), 99 = inscribed (the robot's center there would touch
it), 1-98 = inflation only, 0 = free, -1 = unknown. Use it to skip goals that
fall on an obstacle missing from `/map` (value >= 99).

## Fusion → Mission

Fusion delivers **candidates**: one per box it thinks it has seen. Mission
decides when to report each one.

**`/jar/candidatas`** (`equipo_jar_msgs/Candidatas`): the full list every time
something changes, and at least once per second.

```text
# Candidatas.msg
std_msgs/Header header        # frame_id: map
Candidata[] candidatas

# Candidata.msg
uint32 id                     # stable while it exists; if two merge, one remains
geometry_msgs/Point centro    # estimated box center, in map
float64[4] covarianza_xy      # 2x2 matrix by rows, in m²
uint32 observaciones          # how many detections back it
builtin_interfaces/Time ultima_vez
```

These live in our own message package, `src/equipo_jar_msgs`.

## Mission → Judge

- **`/victimas`** (`PointStamped`, frame `map`): one message per report, built
  with the template's `armar_reporte()`. N in total.
- **`/done`** (`std_msgs/Empty`): once, at the end.

## Testing before the other modules exist

`pruebas.launch.py` starts `competencia.launch.py` plus a static identity TF
`map → odom`, so every module works in simulation before localization does.
It **never** goes in `competencia.launch.py`. Test files with coordinates go
outside `src/` (e.g. `pruebas/`) so they don't trigger the CI's hardcoding
warnings.

## Changing this interface

Changes to this file or to `equipo_jar_msgs` go through a PR to `development`,
approved by the owners of the affected modules. See [`FLOW.md`](FLOW.md).
