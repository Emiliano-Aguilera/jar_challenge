# Navigation access diagnosis

This review implements Marco's evidence request. It does not change navigation,
coverage, margins or competition interfaces. The movement comparison remains
pending until both cases run on the official simulator. ROS transport tests use
synthetic inputs; they do not demonstrate that Nav2 can enter a room.

Problem case: `tars_borde_ventanas`, upper-right room, from
[run 41323263886](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/41323263886).
Recover the actual requested pose and incoming trajectory from that run before
choosing the controlled target. Keep test coordinates in runtime output under
`build/`, never in `src/` or production parameters.

## 1. Footprint and effective configuration

`navegacion.launch.py` builds the chassis from forward 0.119 m, rear 0.181 m
and half width 0.100 m. Both costmaps request 0.020 m padding. The intended
effective bounds in `base_footprint` are:

| Axis | Minimum | Maximum |
| --- | --- | --- |
| x | -0.201 m | +0.139 m |
| y | -0.120 m | +0.120 m |

Size: 0.340 x 0.240 m. Center: x = -0.031 m. Farthest corner from the rotation
origin: 0.234096 m. A disk of that radius covers all in-place headings at the
nominal position; it does not include localization or stopping error.

The launch passes the footprint through process-wide parameters, including the
embedded costmap nodes. YAML alone cannot prove it was loaded. The diagnostic:

1. Reads all declared effective parameters from both costmap nodes, controller,
   planner, behaviors and navigator, before and after the probe.
2. Records both `published_footprint` topics, transforms each polygon back to
   `base_footprint` **at its message stamp**, and checks all four corners and
   cyclic adjacency. A centered rectangle, unpadded rectangle or a polygon with
   matching bounds but wrong corners must fail.
3. Separately checks that MPPI's active CostCritic considers the footprint and
   that the behavior server points to the local raw costmap and footprint.
4. Saves the installed navigator tree and its hash, plus installed Nav2 package
   versions. Compare these with the source checkout; do not infer a deployed
   binary's behavior from a different distribution's documentation.

`confirmed` means those configuration/publication checks passed, not that an
instrumented collision-checker call was observed or a movement was safe.
Missing, old or mismatched evidence cannot become a confirmation. No tool
automatically corrects parameters. Record any discrepancy before changing it.

## 2. What a costmap connection proves

NavFn searches position cells and costs in the global grid; its search has no
heading state. Inflation reflects the footprint's inscribed radius (here
0.120 m), and the planner rejects obstacle/inscribed cells. It does not test the
oriented rectangle or its swept area along every turn. Dijkstra and A* share
that limitation. `allow_unknown: false` rejects unknown transit. Planner
`tolerance: 0.05` permits a nearby reachable endpoint instead of the exact goal.
The path and actual endpoint must therefore be recorded.

[NavFn Humble source](https://api.nav2.org/nav2-humble/html/navfn__planner_8cpp_source.html).

MPPI predicts local x/y/yaw trajectories with the Omni motion model. The
configured horizon is 40 x 0.05 = 2 s. With `consider_footprint: true`, CostCritic
uses the local costmap footprint at predicted headings when the center cost
indicates possible contact. Inflation supports that shortcut. Colliding samples
receive a large finite cost; if all sampled trajectories collide, the critic
signals failure. This is not a global reachability proof or a mathematical hard
constraint on a continuous swept volume. The footprint checker rasterizes the
polygon edges against the grid; sensor, grid and prediction sampling matter.
Do not treat `trajectory_point_step: 2` in YAML as effective without checking
the installed implementation: the Humble source linked here does not read it
in CostCritic. A parameter appearing in a dump alone does not prove it is used.

[CostCritic](https://api.nav2.org/nav2-humble/html/cost__critic_8cpp_source.html),
[footprint checker](https://api.nav2.org/nav2-humble/html/footprint__collision__checker_8cpp_source.html).

## 3. Approach, arrival and recovery

The current default configuration has these distinct mechanisms:

| Mechanism | What it can attempt / check |
| --- | --- |
| NavFn | Finds a position path. Default `use_final_approach_orientation: false` retains the requested final heading; it does not constrain the approach to that heading. |
| MPPI tracking | Omni translation and rotation. PathAngleCritic favors alignment with the path, with forward/reverse allowed. PathAlignCritic does not use path orientations. These are preferences, not a guaranteed maneuver. |
| Final approach | GoalCritic and GoalAngleCritic become relevant within 0.4 m; the robot can rotate while still approaching. There is no separate RotationShim in this configuration. |
| General goal checker | `stateful: true`, XY 0.10 m and yaw 0.25 rad. After entering the XY window it latches that check until reset, then tests heading. It does not command rotation, check obstacles or require zero velocity. A final pose can drift from the latched position. |
| Controller failure | Failure tolerance 0.3 s; PoseProgressChecker requires 0.3 m translation or 0.5 rad rotation within 5 s. Controller failure can trigger costmap clearing and retries. |
| Default tree | Replans at 1 Hz; contextual clearing/retry, then global recovery with up to six retries. RoundRobin: clear local/global maps, Spin +1.57 rad, wait 5 s, reverse 0.30 m at 0.15 m/s. Recoveries are alternatives, not one compulsory sequence per goal. |
| Spin | Uses the local footprint/costmap at sampled future headings, with a 1 s lookahead at 10 Hz. It may begin and later stop when an upcoming collision is found. Collision, missing TF or expiration can fail the behavior; that need not immediately abort the overall goal. |
| BackUp | Checks the reverse movement with the behavior collision checker. Space for turning does not imply room to reverse. |
| Mission | In production, a goal has a 90 s budget, supplementary goals at most 30 s; 20 s without sufficient motion also cancels, as do 10 recoveries outside return. A newly occupied goal can also be canceled. Unknown acceptance has a 5 s budget and triggers terminal stop. These outer limits can interrupt the tree before its retries finish. |

The controlled client deliberately runs without mission to isolate Nav2. Its
wall-time timeout is explicit and recorded; it is not a measurement of mission's
production timeout. Compare mission logs separately before attributing an abort
to Nav2. Clearing a costmap does not remove a physical obstacle.

[Goal checker](https://api.nav2.org/nav2-humble/html/simple__goal__checker_8cpp_source.html),
[controller server](https://api.nav2.org/nav2-humble/html/controller__server_8cpp_source.html),
[Spin](https://api.nav2.org/nav2-humble/html/spin_8cpp_source.html).

## 4. Controlled test procedure

Run on the official Linux simulator, with its normal robot/sensors/judge already
started. This macOS checkout has no official simulator. Do not run a second
mission or send simultaneous goals. Start navigation alone in terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 launch equipo_jar navegacion.launch.py use_sim_time:=true
```

Use the real scenario's ROS domain if it differs. `navegacion.launch.py` includes
localization and the permanent output guard. The normal competition launch also
starts mission, so it is unsuitable for this controlled client. Do not change
the shared server workflows or start a live session without the user's request;
follow `docs/team/SIMULATION.md` when arranging execution there.

Terminal 2, read-only configuration audit (safe alongside normal mission):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
python3 pruebas/diagnosticar_navegacion.py \
  --output build/verificacion/acceso/audit-01 --duration 5 \
  --ros-args -p use_sim_time:=true
```

For movement, choose actual map coordinates from the scenario. First choose a
wide doorway and a destination with enough space for the whole intended turn.
Set X/Y/YAW to that target; a single test invocation sends exactly one goal:

```bash
read -r X Y YAW
python3 pruebas/diagnosticar_navegacion.py \
  --case wide --goal "$X" "$Y" "$YAW" --timeout 120 \
  --truth-topic /ground_truth/odom --truth-frame-is-map \
  --output build/verificacion/acceso/wide-01 \
  --ros-args -p use_sim_time:=true
```

`--truth-frame-is-map` declares an alignment: use it only when the simulator's
truth frame equals `map` (the official simulator's `world` frame does). The
original header is retained. Without that declaration, a non-map truth frame is
recorded but not used for final error. This test-only topic never enters `src/`.

Repeat from a documented initial pose on `tars_borde_ventanas`, upper-right room,
with the recovered goal and `--case problem`, using a new output directory. Keep
the chassis, padding, map resolution, localization, speeds and controller/BT
settings identical between runs. Record doorway geometry and initial pose so a
different approach is not mistaken for a controller change. Success in the wide
case alone does not prove that the difficult entry is feasible.

The linked run's log identifies two upper-right corner goals at approximately
(3.53, 3.53), with yaw +2.35619 and -2.35619 rad. Both were canceled by mission
after 10 recoveries; the latter includes a Spin stopped by predicted collision.
Earlier targets in this area also failed because their cells became occupied
or because mission detected no motion. Thus there is more than one failure
mechanism in the recording. The log's goal coordinates are rounded; recover the
exact action/plan poses and incoming headings from the recording for replay.
The run used code `35450e5` and world `35ac43e`, not this branch's current code.
Its summary reports 400 points, zero collisions and RTF 0.976. These historical
results are not the requested paired controlled test.

Each output contains:

- `events.jsonl`: source messages and ROS/wall times for maps, raw costmaps,
  footprints, global plans, localized TF, optional true odometry, actual output
  velocities, Nav2 logs, feedback/recovery counts and action status/result.
- `summary.json`: effective parameters before/after, footprint checks, installed
  tree/version evidence, requested goal, action outcome and final localized/true
  position and wrapped yaw errors when fresh evidence exists.

Exit 0 means the audit passed and, if requested, Nav2 returned success. It never
means collision-free certification. `collision_free` stays unknown: inspect the
simulator's collision report, true swept path and recording. Record final pose
even when Nav2 says success, especially because XY checking is stateful.
Unknown recoveries remain unknown rather than being silently changed to zero.

On timeout, interrupt or unknown acceptance **after dispatch**, the client sends
the permanent `/jar/parar` inhibit and requests cancellation if a handle arrives.
Inspect stop-confirmation events. Restart navigation before another trial; the
inhibit cannot be undone. It does not publish direct velocities or `/done`.
Before dispatch, incomplete geometry or a detected mission prevents the goal.
Output directories must be new, so previous evidence cannot be overwritten.

## Coverage acceptance conditions

Keep today's margins, including observation minimum 0.24 m and preferred 0.30 m.
They cover nominal turning geometry at a view center but do not establish
arrival-error robustness or feasible transit (the circular transit model uses
0.18 + 0.03 m, smaller than the padded rear-corner radius).

Before treating a proposed view as executable, require:

1. A current, known map/costmap and valid localized pose; include obstacles absent
   from `/map`. A connected component is only a preliminary filter.
2. A collision-free arrival envelope for the padded rectangle at the viewing
   direction, accounting for allowed position/yaw error and localization error.
   A free exact endpoint alone is insufficient.
3. A feasible approach from the incoming posture and all intermediate footprint
   positions/headings, including the final correction. Check the return/escape
   maneuver too when the mission relies on it.
4. Compatibility with the actual controller and recoveries. A specifically
   validated approach cannot be called guaranteed while Nav2 may choose another
   orientation or attempt an incompatible Spin/BackUp. Runtime checks must be
   able to stop/fail, and mission must skip/replan instead of retrying forever.

Full-turn clearance at every view is a conservative option, not a universal
physical requirement. A particular orientation/approach can admit more views,
but needs validation and execution that respects it. The current NavFn path plus
goal quaternion does not supply that contract. This branch documents that choice;
it does not silently relax coverage's checks or replace the planner.

## Evidence status

| Requested result | Current evidence |
| --- | --- |
| Source footprint | Matches requested dimensions and padding; unchanged |
| Effective footprint on actual scenario | Pending capture; synthetic transport tests are not that capture |
| Wide entry and turning destination | Pending official simulator run |
| Upper-right room comparison | Pending exact goal extraction and controlled run |
| Coverage conditions | Defined above; behavior changes require the measured diagnosis |
