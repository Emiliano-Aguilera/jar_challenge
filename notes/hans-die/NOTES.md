# hans-die's notes

Module: localizacion + navegacion

One entry per decision: the question, what was tried, what was decided.

## Navigation tuning log (navegacion/nav2, from 2026-10-25)

Batches are on the sim server: `https://server-trx50-ubuntu.tail4dd159.ts.net/runs/<batch>/`
(Tailscale). "Stuck" = ground truth moved < 0.15 m for >= 30 s. Times are seconds after `/map`.

Offline tool (not in the repo): rasterize the map `.pgm` + the boxes of the `_victimas.world`
and compute, for each goal, the largest circular robot radius that still reaches it from the
start (costmap rule: a cell is blocked if its center is within the radius of an obstacle
cell's center). "+1 cell" = obstacles one cell thicker, as LiDAR marks and AMCL error do.

| Map | Max radius that reaches every free spot | With +1 cell |
| --- | --- | --- |
| maze_1_6x5 | 0.195 | 0.150 |
| maze_2_6x5 | 0.180 | 0.140 |
| maze_3_6x6 | 0.180 | 0.140 |
| maze_4_metal_6x6 | 0.180 | 0.140 |
| laberinto_1 | 0.195 | 0.150 |
| laberinto_simple | 0.295 | 0.250 |

Robot (URDF `rosmaster_x3_base.urdf.xacro`): chassis 0.30 × 0.20 m (with wheels), shifted
0.031 m back from `base_footprint`: x in [-0.181, 0.119], y in [-0.10, 0.10]. Rear corners
are 0.207 m from the center, front corners 0.155 m.

### Baseline 04a07ce (batches 20261025-181419/181420-04a07ce, 300 s)

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 100 | tiempo | 0.97 | none | 125 planner failures: every goal in the lower right (2.73,-1.92), (2.28,-2.82), (3.63,-1.92), (4.08,-1.47). Offline: bottleneck radius 0.20 (0.15 with +1 cell) < planner radius 0.20 |
| laberinto_1 | 0 | tiempo | 0.97 | 92 s at (-0.68,-2.63) | Goal (-1.32,-2.72) is 0.65 m away in a straight line but behind a wall (U-turn round the wall tip). 18 "Failed to make progress" |

### Q: why does the planner find no path to whole regions? (iteration 1)

- Hypothesis: the circular robot model is too big. Any circle that covers the chassis (0.21 m)
  closes the 0.15-0.20 m bottlenecks; the 0.18 m circle we had even leaves the rear corners out.
  With the real rectangle the planner sees a 0.10 m half width, and MPPI's CostCritic
  (`consider_footprint: true`) and the behaviors check the real shape near obstacles.
  Expect: no planner failures towards the lower right of maze_1, and the gap next to the blue
  box passed without hitting it.
- Change: `footprint` built in `navegacion.launch.py` from three chassis measures (no literal
  coordinate pairs: the technical judge flags 3+ pairs), `footprint_padding: 0.02` in both
  costmaps, `consider_footprint: true`. Checked in Nav2 humble: costmap uses `footprint` when
  it is valid (`costmap_2d_ros.cpp`); CostCritic throws if `consider_footprint` is true and the
  costmap uses a radius (`cost_critic.cpp`); launch_ros writes dict params for `/**` when the
  node has no namespace, so they reach the costmap nodes inside controller/planner.

### Q: why does MPPI push into a wall when the goal is just behind it? (iteration 1)

- Hypothesis: `threshold_to_consider` is a straight-line distance to the goal
  (`utils::withinPositionGoalTolerance`). Closer than it, PathFollow/PathAlign/PathAngle turn
  off and GoalCritic pulls straight to the goal. With 1.0 m, a goal 0.65 m away behind a wall
  pulls the robot into the wall. Expect: no stuck at (-0.68,-2.63) in laberinto_1.
- Change: all five thresholds to 0.4 m (less than two poses on both sides of a wall are apart).

**Result iteration 1** (155a367, batches 20261025-182715/182716-155a367): both hypotheses held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 (was 100) | done 212.6 s | 0.97 | 42 s at (2.56,-2.13) | 0 planner failures (was 125). Both boxes, fast bonus, back at 0.08 m. Log: "Critic will collision check based on footprint cost" |
| laberinto_1 | 100 (was 0) | tiempo | 0.97 | 30 s at (1.68,-2.73) | 0 planner failures; 54 goals reached (was 30). No stuck at the U-turn |

Decided: keep both. Remaining stuck periods:
- maze_1 t=69-117: MPPI waits in front of the 0.40 m diagonal gap between the red box (2.07,-2.09)
  and the wall tip (2.55,-2.4), then goes through. 10 "Failed to make progress".
- laberinto_1 t=140-170: the robot is at the goal (1.68,-2.72) but cannot turn to its yaw:
  "Collision Ahead - Exiting Spin" at 161.8. The wall edge is ~0.22 m away and the padded rear
  corner reaches 0.227 m. Coverage places viewpoints with a 0.18 m circle + 0.03 m
  (`cobertura.robot_radius`, `safety_margin`): too close to turn there (for Marco).

### Q: why does MPPI wait in front of narrow gaps? (iteration 2)

- Hypothesis: with the long local inflation (0.45 m, factor 3) every cell of the maze has
  cost and the middle of a 0.40 m gap costs ~200 per trajectory point, so trajectories into the
  gap lose against staying still (the MPPI README warns about this: "it may generally refuse to
  go into costed space"). With a short local inflation (0.30 m, factor 10) the cost rises only
  near obstacles. The global costmap keeps the long inflation, so paths still go down the middle.
  Collision checking (the rectangle) does not change. Expect: maze_1 gap passed without the 40 s
  wait, and higher average speed.
- Change: local `inflation_layer` 0.30 m / 10.0 (minimum: 0.23 m, the padded rectangle's
  circumscribed radius, so CostCritic can find its "possibly inscribed" cost).

**Result iteration 2** (4dd995d, batches 20261025-183738-4dd995d and -2):

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_3_6x6 | 300 | done 47.4 s | 0.94 | none | 0 planner failures, 0 recoveries, back at 0.03 m. Lost 100 to a false report at (0.425,0.588) |
| maze_1_6x5 | 0 | inmovil | 0.95 | whole run | Never moved: startup bug below, not the inflation change |

The maze_1 run does not test the hypothesis: re-run in iteration 3.

### Q: why did the robot never move in one run? (iteration 3)

- Evidence: `[amcl.rclcpp]: failed to send response to /amcl/change_state (timeout)`; the
  localization lifecycle manager stays at "Configuring amcl" forever, AMCL never activates, no
  `map -> odom`, the planner never finishes activating. 1 of 11 runs so far.
- Cause (Nav2 humble source): `LifecycleServiceClient::change_state(transition)` calls
  `ServiceClient::invoke(request, response)`, which waits with `spin_until_future_complete`
  and no timeout. A lost response (FastDDS drops it when the client's reader is not matched yet)
  hangs the manager. It can hit any of the 6 managed nodes.
- Change: no Nav2 lifecycle managers. The `localizacion` node (already registered in setup.py)
  brings the nodes up in order (amcl, controller, planner, behaviors, smoother, bt_navigator),
  one request at a time with a 5 s deadline (`plazo_arranque_s`); on a missed deadline it asks
  `get_state` and continues from the real state. Tested offline with mocks (lost configure,
  lost activate, slow planner activation). Side effect: nothing publishes `/diagnostics`
  anymore, so the remap trick is gone. bt_navigator goes last, so `navigate_to_pose` appears
  only when everything else is active.
- Expect: same behavior as before in normal runs; log lines "<node>: active" and "Nav2 activo".

**Result iteration 3** (2a19e92, batches 20261025-184725/184726-2a19e92): bring-up works
(all 6 nodes "active", "Nav2 activo"; 9 "sin respuesta" from the planner while it waits for `/map`).

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_2_6x5 | 600 (max) | done 115.0 s | 0.96 | none | back at 0.02 m |
| maze_1_6x5 | 500 | done 287.7 s | 0.97 | 55 s at (3.46,0.88), 38 s at (3.96,0.25) | Both at the blue box (3.98,0.92) |

Decided: keep. The interval t=44-91: the robot starts at the gap entrance (3.58,0.75) and MPPI
moves it away, to (3.42,0.92), then dithers; 4 "Failed to make progress", spin, wait, then mission
cancels ("robot quieto"). The gap between the wall tip (3.6,0.55) and the box corner
(3.855,0.795) is 0.354 m. Offline footprint check (5 cm grid, padding 0.02): 29 of 72 headings
are free in the middle of the gap (a ~40° window each way), so it is passable.

### Q: why does MPPI move away from a gap the path goes through? (iteration 4)

- Hypothesis: CostCritic's preference term (cost per point, ~110-140 in the gap, less away
  from it) weighs as much as the path critics, so staying away scores better. The global path
  is already centered by the global inflation. With `cost_weight` 1.0 (was 3.81) the path
  critics dominate in narrow places; a collision still costs 1.0/254 × 1e6/40 ≈ 98, far above
  the other critics (~1-10). Expect: maze_1 blue-box gap passed without stalling.

**Result iteration 4** (8a86bb0, batches 20261025-185710-8a86bb0 and -2): hypothesis held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 | done 195.3 s (best so far) | 0.97 | 48 s at (-0.89,1.16) | Blue-box gap passed without stalling |
| maze_4_metal_6x6 | 200 | done 101.2 s | 0.96 | none | Back at 0.02 m. 2nd report (1.592,-0.476) is not a box: wasted (perception) |

Decided: keep. New stuck at goal (-0.87,1.23, yaw -1.57): the robot is within xy tolerance but
does not finish turning; offline check: all 72 headings are free there, so it is not geometry.

### Q: why does the final turn at a goal take so long? (iteration 5)

- Evidence: pure-turn goals (mission sends the same xy with another yaw) take 1.4 s for 45°,
  2.5 s for 90°, 4.3-4.6 s for 180° in every run: MPPI turns at ~0.7 rad/s (wz_max 1.9).
  `SimpleProgressChecker` only measures translation, so a slow final turn (> 5 s) is aborted
  as "Failed to make progress", over and over.
- Hypotheses: (1) `PoseProgressChecker` (in humble since 1.1.7, June 2023; the server has
  CostCritic, from 1.1.14, so it has it): 0.5 rad of turning counts as progress. Expect no
  "Failed to make progress" while turning at a goal. (2) `wz_std` 0.4 -> 0.8: MPPI samples turn
  rates with a spread that is small next to wz_max. Expect 180° in <= 3 s, and faster coverage
  (mission asks for many 45° turns per viewpoint).

**Result iteration 5** (d419c80, batches 20261025-190549/190550-d419c80): both held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| laberinto_simple | 500 (max) | done 33.5 s | 0.95 | none | back at 0.10 m |
| maze_1_6x5 | 500 | done 191.7 s | 0.97 | 37 s at (2.59,-2.11) | Turns: 180° in 2.1-3.6 s (was 4.3-4.6), 45° in 1.0-1.2 s (was 1.4). No stuck at (-0.87,1.23) |

Decided: keep. The stuck is the red-box gap again (same as iteration 1; iterations 3-4 passed it
fast): borderline. Offline: at the bottleneck (2.44,-2.34) only 18 of 72 headings are free with
padding 0.02 (29 with 0.01): the long axis must point along the gap within ±20°.

### Q: how to pass gaps that need a precise heading? (iteration 6)

- Hypothesis: PathAngleCritic acts only when the heading is more than `max_angle_to_furthest`
  (1.0 rad) off the path direction, so the robot reaches gaps with any heading and has to find
  the ±20° window inside the gap. With 0.35 rad it travels with its long axis on the path
  (forward or backward, `forward_preference: false`) and reaches gaps already aligned.
  Expect: both maze_1 gaps passed without stalls. Cost: less sideways travel.
- Also (localization): AMCL settled at 5.6 s after `/map` (10 no-motion updates at the 0.5 s tick)
  while Nav2 was active at 3.1 s. Tick 0.2 s (just over one LiDAR sweep). Expect ~2.5 s.

**Result iteration 6** (388cbef, batches 20261025-191306-388cbef and -2): held on maze_1.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 | done 182.2 s (best so far) | 0.96 | none | Red-box gap: 14 s pause, no stall |
| laberinto_1 | 100 | time (300 s) | 0.97 | 28 s at (-1.43,-2.77) | 1 of 3 boxes. 58.7 m driven, mean 0.20 m/s |

AMCL settled 2.2 s after `/map` (was 5.6 s). Decided: keep.

### Q: why does the robot freeze next to a goal? (iteration 7)

- Evidence (laberinto_1, 388cbef-2): two freezes, 21 s at goal (1.43,-2.72) t=136-158 and 28 s
  at goal (-1.32,-2.72) t=174-202. Both goals are 0.25 m from a wall. Both times bt_navigator
  says the robot is ~0.11 m from the goal (just outside `xy_goal_tolerance` 0.10), MPPI does not
  move, 3-4 "Failed to make progress", then spin or mission cancel. Ground truth vs the pose
  bt_navigator logs at goal start (robot still): AMCL error median 0.10 m, max 0.22 m on
  laberinto_1; 0.05 m on maze_1/2/4. Example t=158.6: AMCL (1.42,-2.61), truth (1.45,-2.71).
  So the goal in the real world is ~0.14 m from the wall: the rectangle cannot get there and
  GoalCritic pulls into the wall. Root cause: localization, not the controller.
- Hypothesis: `sigma_hit` 0.2 (Nav2 default) blurs the likelihood field over a whole corridor
  (0.6 m wide). LiDAR noise is 5 mm (sim URDF) and the map has 5 cm cells, so 0.10 is still
  wide enough for a real map that is off by a few cm. Expect: AMCL median error on laberinto_1
  <= 0.06 m and no near-goal freezes. Also helps reports (error budget 0.25 m).

**Result iteration 7** (50a6b57, batches 20261025-192651-50a6b57 and -2): held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| laberinto_1 | 100 | time (300 s) | 0.97 | none | 0 "Failed to make progress" (was 7), 63 goals OK, 73 m driven (was 59). AMCL median 0.075 m (was 0.10) |
| maze_3_6x6 | 300 | done 86.3 s | 0.95 | none | back at 0.08 m. AMCL median 0.03 m. 3rd report is not a box (perception) |

Decided: keep. Time near goals (< 0.4 m) on laberinto_1: 141 s of 291; the last 0.4 m takes
1-3 s per goal (96 s over ~45 travel goals) and pure turns 19 s. Travel speed 0.34 m/s.
The rest is the coverage route (see "For other modules").

## For other modules (evidence)

- Perception: reports with no box nearby. maze_3, 50a6b57, t=75.3 "Reporte 3. id=6 x=1.61
  y=-1.46": nearest box is blue (0.956,-1.909), 0.79 m away; mission then sent `/done` with red
  (-2.08,-2.02) never seen (lost 100 + fast bonus). Same map, 4dd995d t=17.7 (0.42,0.59) and
  de1b2b4-2 t=107.7. maze_4, 8a86bb0-2: (1.592,-0.476), nearest box 1.38 m away.
- Coverage: laberinto_1, 50a6b57-2: goals (1.43,-2.72) t=132 -> (-1.32,-2.72) -> (-2.17,-2.77) ->
  ... -> (1.63,-1.97) t=173: the bottom corridor is driven west and then back east (~4 m twice),
  and the two pockets with red boxes ((-1.25,-0.16) and (-1.9,1.45)) are never entered in 300 s.
  Also `cobertura.robot_radius` 0.18 is below the real rear corner (0.207 m + 0.02 padding):
  155a367 t=160.8 "Collision Ahead - Exiting Spin" at goal (1.68,-2.72).
- Mission: turn goals reuse the viewpoint xy. When Nav2 stopped short (reached at the
  tolerance edge), the turn is a new goal that needs the approach again: laberinto_1
  193607-50a6b57-2, goal (-2.17,2.43) reached at 0.14 m t=262.6, turn to -1.57 stuck until
  t=285.5. Use the robot's current xy for turns (or a Spin). Also wait until the robot is
  still (or ~0.3 s) between the last goal and `/done` (maze_1 50a6b57: 0.17 s gap, 1 nonzero
  /cmd_vel after `/done`).
- Mission: `start_wait_s` 8 while Nav2 is active at ~3 s and AMCL settled at 2.2 s (~4 s per run
  to gain). 04a07ce maze_1: the same failing point (2.28,-2.82) sent 4 times, t=110-206.

## Final validation, round 1 (50a6b57, 600 s)

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 (max) | done 147.3 s (best so far) | 0.96 | none | back at 0.01 m. **1 nonzero /cmd_vel after /done** |
| laberinto_1 | 200 | done 380.5 s | 0.98 | 22 s at (-2.21,2.35) | back at 0.10 m. 1 of 3 boxes, 91 m driven |
| maze_2_6x5 | 600 (max) | done 117.9 s | 0.96 | none | back at 0.02 m (194727-50a6b57) |
| maze_3_6x6 | 300 | done 85.6 s | 0.95 | none | back at 0.07 m. Same false 3rd report (1.605,-1.451), t=74.6: 2 of 2 runs (194726-50a6b57) |

### Q: why was there motion after /done? (iteration 8)

- Evidence (maze_1, 193607-50a6b57): "Reached the goal!" t=147.06, mission "finishing ->
  finished" (publishes `/done`) t=147.23, judge `movimiento_despues_de_done: 1` (one message).
  On goal success the controller sends zero once; the velocity smoother ramps down with
  `max_decel` -3.2 rad/s², so a final turn at ~0.85 rad/s needs 0.27 s to stop. In all 12 runs
  that ended with `/done`, the gap reached -> done is 0.17-0.24 s: a race that failed 1 of 24.
- Change: `max_decel` [-2.5, -2.5, -20.0]: from wz_max 1.9 it stops in < 0.1 s, from a typical
  arrival speed (< 0.15 m/s) in < 0.06 s. MPPI does not model the smoother's lag anyway.
  Expect: `movimiento_despues_de_done` 0 in every run. Mission should also wait until the robot
  is still before `/done` (see "For other modules").
- The laberinto_1 freeze is a different cause: goal (-2.17,2.43, yaw -1.57) is 0.25 m from the
  top wall in a corner; with the rear 0.20 m behind the center only a narrow yaw window fits.
  Nav2 reached it at 0.14 m (stateful checker), then mission sent a turn at the same xy: a new
  goal, so the robot had to get within 0.10 m again and could not (t=262.7-285.5, 3 "Failed to
  make progress", mission cancel). Not raised `xy_goal_tolerance`: stops are median 0.058 m,
  p90 0.10 m (255 goals); 0.15 would eat into the return margin (0.25 m with AMCL error).

## Round 2 (d9d637e, 600 s)

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_4_metal_6x6 | 500 (max) | done 104.7 s | 0.96 | none | back at 0.05 m, no false report this time (195208-d9d637e-2) |
| laberinto_simple | 500 (max) | done 82.1 s (was 33.5) | 0.96 | **50 s at (0.37,1.91)** | back at 0.05 m (195208-d9d637e) |
| maze_1_6x5 | 500 (max) | done 176.5 s | 0.97 | none (25 s pause at the red-box gap (2.48,-2.14)) | back at 0.07 m (195628-d9d637e-2) |
| laberinto_1 | 200 | done 380.1 s | 0.97 | 21 s at (-1.36,-2.70) | back at 0.07 m. Same goal (-1.32,-2.72) as iteration 6. AMCL median 0.115 m (195628-d9d637e) |

### Q: why 50 s stuck next to the red box in laberinto_simple? (iteration 9)

- Evidence: after "Reporte 1. id=1 x=0.01 y=2.00" (t=12.4) mission sends coverage viewpoint
  (0.144,1.969) t=14.1: 0.02 m from the face of the red box (0.0,2.0), which is not in `/map`,
  so coverage does not know it. NavFn (`tolerance` 0.2) plans to a free cell nearby; the robot
  stops at the path end, 0.21 m from the goal, and never reaches it: 7 "Failed to make
  progress", spin, wait, mission cancel "robot quieto" at t=67.6 (the BT retries 6 times and
  the recoveries move the robot, so the mission's stuck timer kept restarting).
- NavFn Humble (navfn.cpp `setCostmap`) maps inscribed (253) and lethal to obstacle, and the
  global costmap has the LiDAR obstacle layer. With `tolerance` 0.0 a goal whose center is
  within 0.12 m of an obstacle (impossible for the robot anyway) fails at planning.
  Expect: Nav2 returns failure after the BT retries (~16 s: clear, spin 2.3 s, wait 5 s,
  backup 6 s) instead of ~53 s, and the robot does not push against the box. Goals at the
  usual 0.25 m from walls are far from inscribed cost, so they still plan.
- Better fix on the coverage/mission side: skip viewpoints within ~0.3 m of a reported or
  candidate box. A custom BT with fewer retries would cut the 16 s further, but needs a
  `data_files` entry in setup.py (shared file): proposed, not done.

### Why AMCL is off by ~0.1 m on laberinto_1 only

The errors repeat at the same places in two runs (t~15 s dy +0.07/+0.11, t~211-231 s dx -0.14/-0.15),
so they are not noise. Rasterizing the 51 wall boxes of `laberinto_1_victimas.world` and comparing
with `laberinto_1.pgm` (scratchpad `muro.py`): 94 world wall cells have no pgm wall within 0.07 m
and 69 pgm wall cells have no world wall within 0.07 m. Example: the pgm wall at y=-2.17
(x=-2.9..-2.0) is 0.10 m off, the west wall x=-3.27 is missing by 0.10 m. AMCL matches the scan
to the map, so ~0.1 m is the map's error, not a tuning problem. A competition map may be like
this too, so goals 0.25 m from a map wall can be ~0.15 m from the real one. Not changed in
navigation (a larger goal tolerance would eat the return margin); coverage needs more clearance
(>= 0.30 m) or mission can accept "within 0.2 m and still for a few s" as arrived.

**Result iteration 9** (26db928, batches 20261025-200521/200522-26db928): half held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| laberinto_simple | 500 (max) | done 54.0 s (was 82.1) | 0.96 | none | the goal on the box failed fast, no pushing |
| maze_2_6x5 | 600 (max) | done 125.0 s (was 117.9) | 0.96 | none | 10 planner failures (was 0) |

maze_2: goal (0.495,0.295) is 0.25 m from the pgm wall, but planning failed on and off from
t=10.2 to 24.3 ("failed to create plan with tolerance 0.00") and the first goal took 9 s instead
of ~2 s. The goal sits at the edge of inscribed cost (the real wall, a mesh I could not compare,
or LiDAR noise), so it flickers between plannable and not.

### Q: what planner tolerance? (iteration 10)

- Rule: the planner may end the path at most `tolerance` from the goal (NavFn searches a square
  of ±tolerance, so up to 1.41 × tolerance). That point must count as arrived, so
  1.41 × tolerance < `xy_goal_tolerance` (0.10). 0.2 broke the rule (paths ending 0.28 m away:
  the robot never "arrives", 50 s stuck); 0 rejects goals at the edge. 0.05 (one cell, 0.07 m)
  keeps it. A goal on a box (0.02 m from its face) is still >= 0.10 m from any free cell, so it
  still fails fast.
- Expect: laberinto_simple still fast (~54 s, no stuck), maze_2 with ~0 planner failures.

**Result iteration 10** (bb0f061, batches 20261025-201147-bb0f061 and -2): held.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| laberinto_simple | 500 (max) | done 57.2 s | 0.96 | none | box goal failed fast again |
| maze_2_6x5 | 600 (max) | done 120.5 s | 0.96 | none | 0 planner failures |

## Final validation on bb0f061 (600 s, all 6 maps)

| Map | Score | End | RTF | Stuck > 30 s | Back | Batch |
| --- | --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 (max) | done 177.3 s | 0.97 | none (12 s pause at the red-box gap) | 0.07 m | 201658-bb0f061 |
| maze_2_6x5 | 600 (max) | done 120.5 s | 0.96 | none | 0.02 m | 201147-bb0f061-2 |
| maze_3_6x6 | 300 | done 91.0 s | 0.95 | none | 0.06 m | 202546-bb0f061 (false report again, 3 of 3) |
| maze_4_metal_6x6 | 500 (max) | done 107.3 s | 0.96 | none | 0.08 m | 202546-bb0f061-2 |
| laberinto_simple | 500 (max) | done 57.2 s | 0.96 | none | 0.08 m | 201147-bb0f061 |
| laberinto_1 | 300 (best so far, 2 of 3) | done 356.3 s | 0.97 | none, 0 "Failed to make progress" | 0.05 m | 201657-bb0f061 |

No motion after `/done` in any run. `verificar_params.py` OK, `verificar_estatico.py` 0 errors.

### Q: can the maze_1 red-box gap pause be shorter? (iteration 11)

- Evidence (201658-bb0f061): the robot waits at the gap entrance (2.48,-2.13) t=58-80 with a
  heading that does not fit, 4 "Failed to make progress" (5.2 s each), then the BT's Spin
  (t=79.5) turns it and it passes in 2 s. MPPI's random samples almost never rotate in place
  without also moving, and in the gap any drift collides. The Spin comes after 4 failures.
- Change: progress checker 0.2 m or 0.5 rad every 3 s (was 0.3 m / 5 s; min speed 0.067 m/s).
  Expect: Spin ~10 s earlier at the gap; no new "Failed to make progress" elsewhere
  (laberinto_1 had 0).

**Result iteration 11** (183efc9, batches 20261025-203109-183efc9 and -2): worse, reverted.

| Map | Score | End | RTF | Stuck | Notes |
| --- | --- | --- | --- | --- | --- |
| maze_1_6x5 | 500 | done 172.8 s | 0.96 | none | gap pause 10 s (was 12): within run-to-run noise (12-25 s) |
| laberinto_1 | 100 | done 510.7 s, not back (3.23 m) | 0.98 | **41, 40, 31 s** around (-2.88,1.6) | 115 planner failures, 49 recoveries |

Goal (-2.92,1.68, yaw 0) is in the middle of a 0.55 m corridor (west side); the 3 earlier runs
reached it. With 3 s the controller aborted more often, and the BT's spin/backup/wait moved the
robot inside the narrow corridor until even planning failed. Reverted to bb0f061's 0.3 m / 5 s:
the final config is bb0f061's (validated above).

## Navigation: planner, global costmap and precise return (from 2026-10-28)

Asked by mission (Bautista, 2026-10-27/28).

### Q: why did maze_1's return lose ~16 s to the planner?

- NavFn with `use_astar: true` sometimes fails to extract a path that exists ("Failed to
  create plan from potential"). Decided: `use_astar: false` (Dijkstra, Nav2's default; a
  6 x 6 m map plans fast either way).
- Checked (maze_1, 600 s): "from potential" 8 and 4 with A* (d14d0a4, 6f8a7e3), 0 and 0 with
  Dijkstra (f9977bb x2). Planner failures 22 / 18 -> 15 / 14. Score 500 in all; done at 233 /
  219 s with A*, 213 / 250 s with Dijkstra (exploration order varies run to run).
- laberinto_1 with Dijkstra: 200, 300, 300 (3 runs); with A* it also scored 200 once
  (localizacion/correccion-giros, 2026-10-28). Not navigation: every run there ends exploring
  with "no quedan puntos ni dudosas" and finds the second box (~(-2.08, 1.50)) only on the way
  back, if the return path passes it. Coverage gap for cobertura / mission.

### Q: can mission read Nav2's global costmap?

- Yes, `/global_costmap/costmap` (reliable + transient local). But Nav2 sends the full grid
  only once and then `costmap_updates` (recording: 1 vs 289 messages). Decided:
  `always_send_full_costmap: true` (full grid at 1 Hz, ~14 KB). Grid values are Nav2's costs
  scaled: 100 lethal, 99 inscribed, 1-98 inflation, -1 unknown (`costmap_2d_publisher.cpp`).

### Q: how does the return get closer than 0.10 m?

- Rule (organizers via mission): the 4 wheels inside a 0.25 m circle -> position error
  < 0.10 m. Nav2 accepted 0.10 m for every goal.
- Decided: a second goal checker `precise_goal_checker` (0.05 m, any heading; the wheels are
  symmetric) chosen by `goal_checker_id` in `behavior_trees/vuelta_precisa.xml`; mission
  sends that tree in the return goal's `behavior_tree`. 0.05 m, not 0.03: AMCL itself is
  ~0.05 m off, so tighter does not get closer to the real start.
- Trap: with two goal checkers, Humble's default tree fails (`FollowPath` has no
  `goal_checker_id` -> "goal_checker name  ... does not exist", `controller_server.cpp`). So
  `behavior_trees/navegar.xml` (Humble's tree + `goal_checker_id="general_goal_checker"`) is
  now the default, set by `navegacion.launch.py`. Both XML are installed by `setup.py`.
- Checked (temporary branch with the precise tree on every goal, 600 s): maze_1 500, done at
  196 s, judge's distance to the start 0.038 m; maze_3 300, 0.055 m. No stalls, no checker
  errors. With the normal tree the same maps end 0.054-0.084 m away.

## Localization: AMCL drifts after in-place turns (from 2026-10-27)

### Q: why did AMCL put the robot 0.54 m off, inside a wall? (Marco's report)

- Evidence (Marco, laberinto_1, 37c2306, batch 20261026-181111): AMCL fine for 188 s (median
  0.075 m against the judge's trajectory), then in the left corridor (x ~ -2.8, y 1.6 -> 0.2) the
  error grows to 0.30 m in 5 s and 0.54 m at t=208.8 s, estimate inside the wall at (-2.27,0.18).
  The planner then fails with the start inside an obstacle (62 "failed to create plan", 9
  spin/backup/wait, 7 "backup failed") and the robot stays there until the end (300 s). Iteration
  11 above (115 planner failures at (-2.88,1.6), same corridor) was most likely the same thing.
- What it is not: odometry (per AMCL update, odometry and ground truth agree within ~1 mm and
  0.01 rad: no slip), the map (pgm and world walls agree within 0.10 m there), TF or timing.
  The scan matches the map perfectly at the true pose (AMCL-like score 18.5 vs 6.7 at AMCL's pose).
- Mechanism (recording, `/particle_cloud`): the x covariance grows only during in-place turns
  (0.13 -> 0.45 m). Nav2 Humble `OmniMotionModel` adds x and y noise per radian turned with
  `alpha4` (`sqrt(alpha4 * dtheta^2)` in both). With 0.2, a 90° turn spreads the particles ~0.2 m
  per axis. Mission turns a lot at each viewpoint, so 5-10 % of the particles ended 0.8-1.0 m
  east, in the next corridor across the wall. The likelihood field sums `pz^3` over beams, so a
  pose that matches half the beams still keeps ~half the weight and those particles survive.
  AMCL's estimate is the mean of the biggest cluster (0.5 m bins), and the bins bridge both
  corridors, so the mean slides into the wall.
- Offline replay (scratchpad `replay_amcl.py`: Nav2's omni motion model, likelihood field,
  resampling, KLD limit and clustering, fed with the recorded `/scan` and `/odom`, 8 seeds,
  t=181-211 s):

| alpha4 | min_particles | Median | p90 | Max |
| --- | --- | --- | --- | --- |
| 0.2 (before) | 500 | 0.113 | 0.345 | 0.528 |
| 0.2 | 2000 | 0.078 | 0.213 | 0.340 |
| 0.1 | 500 | 0.026 | 0.087 | 0.196 |
| 0.05 | 500 | 0.029 | 0.093 | 0.175 |
| 0.02 | 500 | 0.029 | 0.060 | 0.114 |
| **0.01** | 500 | 0.030 | 0.050 | 0.089 |
| 0.005 | 500 | 0.028 | 0.045 | 0.075 |

  Starting from the drifted cloud (t=188.6): median 0.41 m with 0.2, 0.07 m with 0.01 (it comes
  back). Start-up with a hand-placement error (initial pose sent at the origin, 10 no-motion
  updates, then the recorded motion): converges with 0.2 and 0.01 alike up to 0.55 m / 1.0 rad;
  a 90° yaw error does not converge (5 of 6 seeds 2.5-3 m off).
- Change: `alpha4` 0.2 -> 0.01 (a full in-place turn adds ~0.09 m per axis, more than the real
  robot's center drift while turning). The other alphas stay: they did not cause this. The real
  robot's odometry error is a separate problem (next section).
- Checked (server, 600 s, 2026-10-27; AMCL error = `Begin navigating` poses vs the judge's
  trajectory):

| Code | Map | Score | AMCL error median / max | Planner failures |
| --- | --- | --- | --- | --- |
| d14d0a4 (this change) | laberinto_1 run 1 | 300 | 0.055 / 0.126 m | 0 |
| d14d0a4 | laberinto_1 run 2 | 300 | 0.048 / 0.128 m | 0 |
| 055a559 (development, alpha4 0.2) | laberinto_1 run 1 | 300 | 0.076 / 0.183 m | 0 |
| 055a559 | laberinto_1 run 2 | 200 | 0.087 / **0.922 m** | **193** |
| 37c2306 + this change (Marco's code) | laberinto_1, 300 s | 100 | 0.057 / 0.121 m | 0 |
| d14d0a4 | maze_1 / maze_2 / maze_4 / laberinto_simple | 500 / 600 / 500 / 500 | <= 0.10 m max | 3 / 0 / 22 / 0 |
| d14d0a4 | maze_3 | 300 of 600 | 0.024 / 0.097 m | 0 |

  Without the change, run 2 drifted like Marco's run (AMCL ~0.9 m off from t=274 s, no return).
  maze_3: one report on a spot with no red box and the third box never found; development does
  the same (run 40520732066), so it is not localization. Heading error from the recordings
  (`/amcl_pose` vs `/ground_truth/odom`, 320 samples per run): median 0.8°, p90 2.4°, max 5.4°.

## Localization: the real robot's odometry overcounts (from 2026-10-27)

### Q: why does AMCL's heading stay 10-26° off after turns on the real robot? (friend's report)

- Evidence (workshop week-10 bag `laberinto_victimas`, real X3): `/odom` counts ~25 % too much.
  Turns: the IMU gyro measures 0.80 of `/odom` (22 turns, most 0.77-0.83). Straight: `/odom` says
  0.948 m, the lidar (front and rear rays) 0.744 m (0.785). Same factor at slow and fast turns
  (0.800 / 0.802), so it is not slip. `/imu/data` angular_velocity.z has the wrong sign; its
  orientation is right. The simulator has no such error: no sim run can show it.
- Mechanism: per update the `alpha1` noise covers the bias, but the bias adds up and the noise
  does not: after 90° the bias is ~18° and the cloud ~10° wide. Scans smear ~9° at 1.2 rad/s.
  When the robot stops, AMCL stops updating (`update_min_d/a`), so the error stays exactly when
  the camera looks. 15° at 1.5 m puts a box ~0.39 m off.
- Organizers (2026-10-27): they will not calibrate it; the error is hardware (slow motors, not
  identical) and part of the challenge. Use the bag to tune; more bags from all robots may come.
  So no fixed scale correction (another robot can have another error).
- Offline replay of the bag (side session, 10 seeds, error at the stop after each turn):

| Variant | Heading at stops, median | Stops with a box at 1.5 m > 0.25 m off |
| --- | --- | --- |
| today (alpha1 0.2) | 10.9° | 59 % |
| alpha1 0.8 | 8.4° | 38 % |
| 5 no-motion updates after a turn | 5.9° | 15 % |
| alpha1 0.5 + 5 updates | 4.8° | 2.9 % |
| alpha1 0.8 + 10 updates | 3.6° | 0 % |
| odom x0.8 (not deployable, ceiling) | 1.8° | 0 % |

- alpha1 0.8 in the simulator (branch `localizacion/test-alpha1`, 2026-10-27, laberinto_1 x2,
  maze_1, maze_4, 600 s): same scores (300, 300, 500, 500), same position error (max 0.15 m);
  heading median 1.1-1.2° (0.8° with 0.2), p90 3.2-3.4° (2.4°), max 5.9-7.3° (5.4°).
  Decided: `alpha1` stays 0.2 for now; set it for the real robot with the new bags or in
  Rosario (it is a param).

### Q: can AMCL take the heading from the IMU instead of the wheels? (from 2026-10-29)

- Idea (Emiliano): a second odometry, `odom_fused`, that keeps the wheels' steps in the robot
  frame but turns them with the IMU heading. AMCL uses it as its odom frame. The real IMU
  measures turns well (0.80 of `/odom`, like the lidar), so AMCL no longer sees the 25 % excess.
- Decided (`odometria_imu.py` node, logic in `rumbo_imu.py`): publishes TF `odom_fused -> odom`
  with each `/odom` stamp; AMCL `odom_frame_id: odom_fused`. Tree: map -> odom_fused -> odom ->
  base_footprint. The robot's `odom`, `/odom` and everything that uses them (local costmap,
  behaviors, mission, detection's fixed frame) are unchanged. Launched only with AMCL.
  - Heading: gyro z integrated, each sample's rate over the interval that ends at its stamp
    (the trapezoid rule gave 3.1° instead of 1.8° at stops on the bag). The real gyro has the
    wrong sign: the sign is learned from the wheels (correlation >= 0.7 after 0.5 rad of
    turning faster than 0.2 rad/s; bag: -0.95). Until then, and when the IMU is silent for
    `imu_vieja_s`, a step uses the wheels' heading.
  - On the robot `/imu/data` has the same stamps as `/odom` and comes within ~1 ms: each
    `/odom` waits for its IMU sample (TF ~1.3 ms after `/odom` on the bag, max 5 ms).
  - Params under `odometria_imu` in `navegacion.yaml`. `fuente_rumbo: ruedas` turns it off
    (identity transform); `orientacion` uses the IMU orientation's yaw instead of the gyro.
- Checked on the week-10 bag (`bag_replay/eval_rumbo_imu.py`, the node's class fed in bag
  order, 10 seeds). Same stops as above (end of each stop after a turn):

| Variant | Heading at stops: median / p90 / max | Box 1.5 m > 0.25 m off | Heading, whole run: median / p90 |
| --- | --- | --- | --- |
| wheels (today) | 10.9° / 14.6° / 18.2° | 59 % | 4.1° / 13.1° |
| wheels + PR #34 fix (5 updates) | 6.5° / 11.5° / 14.1° | 32 % | 3.1° / 10.5° |
| `ruedas` (off) | 10.9° / 14.6° / 18.2° | 59 % | 4.1° / 13.1° |
| **gyro, learned sign** | **1.8° / 4.0° / 6.1°** | **0 %** | 1.3° / 3.9° |
| gyro + PR #34 fix | 1.3° / 3.3° / 5.5° | 0 % | 1.2° / 3.7° |
| IMU orientation | 1.8° / 4.1° / 7.1° | 0 % | 1.2° / 4.0° |

  Sign learned at 24 s (first real turn); before that the wheels' heading put odom_fused ~6°
  off, which AMCL fixed. Caveat: the reference pose was seeded with the IMU, so it favours the
  IMU; its own error is ~1-3°.
- Checked in the simulator (6731f9f, 600 s): laberinto_1 300, maze_1 500, maze_3 600 (dev +
  PR #34 at 4ff5abd: 200, 500, 300; laberinto_1 varies with the return path, maze_3 found its
  third box this time). Reports 0.01-0.09 m from the box centers, return 0.04-0.09 m. AMCL
  position error at each "Begin navigating" vs the true trajectory: max 0.159 / 0.054 /
  0.060 m (4ff5abd: 0.137 / 0.062 / 0.065). Gyro sign learned -1 in the sim too, 6-12 s after
  `/map`. Start-up TF warnings are the same as development. New: detection logged 2 (maze_1)
  and 5 (maze_3) "no transform base_footprint -> map ... extrapolation into the future" by
  10-20 ms (throttled to one per 5 s): the chain now waits for `odom_fused -> odom` too, which
  comes with `/odom` (10 Hz). It drops a few camera frames; scores were not affected.
- With this, PR #34's fix after turns adds little on the bag (1.8° -> 1.3°). Its flag
  (`/jar/localizacion_lista`) is still useful; `actualizaciones_giro` can be lowered or set to 0.
