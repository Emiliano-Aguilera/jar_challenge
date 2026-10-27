# AlsinaBautista's notes

Module: mision

One entry per decision: the question, what was tried, what was decided.

Code: `src/equipo_jar/equipo_jar/mision.py` (the node), `confirmacion.py` (what to
report), `exploracion.py` (where to go). Tunables: the `mision` section of
`src/equipo_jar/params/params.yaml`.

## When is a candidate good enough to report?

- Tried: the spread per axis (square root of the covariance diagonal). It misses
  tilted uncertainty ellipses: a candidate can look precise in x and in y and still
  be far off along a diagonal.
- Decided: report when `observaciones >= min_observaciones` (5) and the spread in
  the worst direction (square root of the largest eigenvalue of `covarianza_xy`) is
  `<= max_desvio_m` (0.10 m). The most precise candidates go first. Never more than
  N reports.

## How do we avoid reporting the same box twice?

- Tried: remembering only the reported ids. Fusion can merge two candidates or
  create one again with a new id, so the same box can come back with another id.
- Decided: skip a candidate if its id was reported, or if it is within
  `min_report_distance_m` (0.40 m) of a reported point or of another candidate
  picked in the same round. Candidates with a NaN or inf center are skipped:
  `armar_reporte` would raise and kill the node.

## What should fusion send?

- Decided (mission side, matches `docs/team/INTERFACES.md`): every candidate fusion
  thinks is a box, doubtful ones too, with an honest covariance and count. Mission
  is the only filter: it knows the time and the reports left, and can go and look
  at doubtful ones.
- Confirmed with fusion (SantosBunge, merged in `development`): it sends exactly
  that. While a box has been seen only from robot poses less than 0.3 m apart, its
  spread stays at 0.15 m or more, above `max_deviation_m`, so mission reports a box
  only after seeing it from two places. Checked by feeding fusion's code into
  `select_to_report`.

## How does mission go through the coverage plan?

- Tried: always going to the first pose of `/jar/plan_cobertura`. If coverage is
  slow to drop a reached point, the robot goes back to it again and again.
- Decided: mission keeps its own list of finished points, across plans, and takes
  the first plan pose that is not in it. A point whose goal failed also counts as
  finished (no endless retries). This works whether or not coverage drops reached
  points.
- Tried: "same point" = within 0.30 m and 0.5 rad. marcorita measured the 18
  practice cases: in 4 of them mission skipped 1 to 7 distinct coverage views
  (e.g. two views 0.25 m apart facing the same way).
- Decided (agreed with marcorita): a view's identity is its exact plan coordinates.
  Coverage puts views on a fixed grid (cell centers, multiples of
  360°/`heading_count`), so the same view always comes with the same numbers, even
  in a new plan. `same_point_distance_m` (0.02 m) and `same_point_angle_rad`
  (0.1 rad) only absorb rounding. Rule: both must stay below half a map cell and
  below π/`heading_count`; `test_params_respect_the_coverage_resolution` checks it.
  Mission compares plan coordinates, not where the robot arrived, so navigation
  accuracy does not matter here. Whether two close views are worth doing is
  coverage's call.

## How does mission talk to Nav2?

- Decided: action `navigate_to_pose`, one goal at a time, with async callbacks.
  Never wait inside a callback or a timer: the node has one thread and would freeze.
  `nav_state`: idle → sending → moving → arrived / failed. Any result other than
  SUCCEEDED counts as a failure. To change goals, mission cancels the current one
  and waits for its result before sending the next, so a late result can't
  overwrite the state of the new goal.

## How is the run organised?

- Decided: a state machine run by a 10 Hz timer:
  waiting_for_map → exploring → returning → stopping → finishing → finished. Every state
  change is logged. After /map, every tick also saves the start pose (if missing) and checks
  reports. Nothing happens before /map, and nothing after /done.

## Where is the start pose?

- Tried: reading it when /map arrives. Localization needs the map before it
  publishes `map → odom`, so at that moment the pose usually doesn't exist yet.
- Tried: keeping the first `map → base_footprint` found after /map. With AMCL that
  is its first guess, before it corrects the hand placement (hans-die's
  `localizacion` node settles it in about 5 s).
- Decided: until the first goal the robot has not moved, so the latest pose is the
  best estimate of the start. Mission updates `start_pose` on every tick and freezes
  it when it sends the first goal (`go_to`). No goal goes out during the first
  `start_wait_s` (4 s) after /map, so AMCL can settle; reports are not delayed.
- Measured (laberinto_1, 5 runs on 2026-10-26, after /map): AMCL settled at
  2.1-2.2 s, Nav2 active at 2.7-3.2 s, coverage plan at 4.7-5.3 s. With 8 s the
  first goal went out at 8.2 s, ~3 s after everything was ready. With 4 s it went
  out at 4.9 s, right after the plan (laberinto_1 120 s check), start pose
  (-0.05, 0.04), no "not ready" warnings. On the real robot AMCL may need longer: it is
  a parameter, so it can be raised after the delivery cutoff.

## When does the robot go back?

- Decided: at the first of: all N reported; no pending plan points and no goal in
  progress; `return_time_s` (480 s) after /map. Having no plan yet is not a reason.
  Mission cancels the current goal, waits for the cancel, then sends the start pose.
  When that goal ends, reached or not, the run ends (see below). No start pose (no
  TF) → it ends right away.

## What happens to the reports left at the end?

- Unused reports are worth 0. A report on a doubtful candidate can only add points:
  far from every box it scores 0, near a box already scored it changes nothing
  (each box keeps its best report), and the fast bonus is already lost if fewer
  than N were reported.
- Decided: right before /done, one last round with looser thresholds:
  `min_observations_final` (1) and `max_deviation_final_m` (0.30 m). It uses the
  same function as the normal round (`check_reports` takes the thresholds), so it
  never goes over N and never repeats a box. It runs at the end and not when the
  return starts, because the camera keeps looking on the way back.

## How does the run always end?

- Problem: /done only went out when the return goal ended. If Nav2 hangs, the run
  reaches 10 min without /done and without the final round.
- Decided: at `finish_time_s` (570 s) after /map, from exploring or returning, the
  run ends (see below).
- Tried: cancelling the current goal and sending /done on the next tick (0.1 s).
  Nav2 needs a moment to process the cancel, and the velocity smoother slows down
  gradually (with hans-die's values, ~0.33 s from 0.5 m/s and ~0.63 s from
  2 rad/s), so the robot could still move after /done. A goal still in `sending`
  could also be accepted and start moving after /done.
- Decided: every way of ending the run (time out, return finished, no start pose)
  goes to state `stopping`. There mission cancels the goal in progress (once),
  waits until no goal is `sending` or `moving`, then waits `stop_wait_s` (1 s) more
  so the smoother stops the robot. If Nav2 never answers, it ends anyway at
  `done_limit_s` (590 s), so /done goes out before 600 s. A goal accepted after the
  run ended is cancelled as soon as it is accepted.
- Ending = final round of reports, state `finishing`, then /done on the next tick.
  /victimas and /done are different topics and ROS does not guarantee the order
  between them; one tick (0.1 s) lets the reports arrive first.
- Rule: `done_limit_s` stays between `finish_time_s` and 600 s. If navigation
  changes its speeds or decelerations, check that `stop_wait_s` still covers the
  braking.

## How does mission avoid standing still?

- Problem: the judge ends the run after 60 s without moving or turning. With no
  plan from coverage, mission had nothing to send and the robot waited still.
- Decided: after `idle_spin_s` (10 s) with nothing to send, mission sends a goal at
  the robot's own position, turned `idle_spin_rad` (90°). Nav2 turns in place, and
  each turn also sweeps the camera (61° wide) for boxes. Turns are not plan points:
  they never go into the list of finished points.
- Needs from navigation (hans-die): a goal at the current position with another
  heading must make Nav2 turn in place.

## Who picks where to look at a doubtful candidate?

- Options: coverage (mission asks it through a service, a new `.srv` in
  `equipo_jar_msgs`) or mission itself, from `/map`.
- Decided (agreed with marcorita): mission. Coverage's main job, the plan, is the
  one still missing, and a service adds an interface change and one more thing that
  can fail. The map functions (`cell_value`, `is_free`, `line_is_clear`) go in
  `src/equipo_jar/equipo_jar/mapa.py` so coverage can use them too. If coverage
  writes similar ones, we merge them into one file.

## How does mission look at doubtful candidates?

- Problem: a box seen from one place stays at 0.15 m spread in fusion, above
  `max_deviation_m`. Seen once in passing, it is only reported in the final round,
  and the fast bonus is lost.
- Decided: every time the robot is free (a trip ended), before the next plan point
  or a turn, mission picks the nearest unreported candidate with at least
  `visit_min_observations` (2) that it has not visited yet (`inspeccion.py`,
  `pick_doubtful`). It goes to look at it from a free spot on a circle of
  `visit_distance_m` (0.8 m) around the box, facing it (`viewpoint_for`):
  - `visit_angles` (12) spots are tried; the robot must fit (`visit_clearance_m`,
    0.25 m from walls, unknown cells and the map edge);
  - no wall between the spot and the box, checked up to `visit_box_margin_m`
    (0.25 m) before its center, in case the map draws the boxes;
  - at least `visit_min_shift_m` (0.4 m) from the robot, so fusion gets a new view;
  - of the valid spots, the nearest to the robot.
- Each candidate is visited once, even if the trip fails: visited ones are
  remembered by position, like reports. Visits are not plan points.
- No spot is not a visit (marcorita's INSPECTION_REPLAY.md, laberinto_1). Mission
  keeps it apart, by candidate id: the observations it had and the number of tries.
  It tries again only when the candidate has more observations (a new center can
  give a spot; the same center gives the same answer) and fewer than
  `visit_max_attempts` (3) tries. Until then it is skipped, so it does not hide
  other candidates or block the return for "no plan points left".
- Not covered yet: a failed trip still counts as a visit, and with no spot the
  robot does not turn to face the candidate from where it is.
- Seen (laberinto_1, 300 s, 3 runs on e0f64a4): in run 1 candidate id 2 at
  (-2.07, 1.53) with 2 observations was the real box at (-1.90, 1.45), 0.19 m off
  (a report there scores 100). No spot, and no new observations came, so no retry;
  coverage went on in the same tick. The box is in the small upper-left pocket.
- marcorita's replay of that case (same simulator map): with 12 angles, rings of
  0.8, 0.6 and 0.5 m all give no spot. Outside points fail line of sight (the
  pocket walls), inside points fail the 0.25 m clearance or are walls: smaller
  rings do not help. The robot cannot enter the pocket (the only opening is one
  cell, ~5 cm; the robot needs >= 0.24 m). The box is visible from the left
  corridor only through that slot, in a 172-175 deg window at 0.8 m, and even
  there the real box is barely visible. With 72 angles `viewpoint_for` would
  pick 285 deg, inside the pocket and unreachable, because it picks the nearest
  spot without checking a route.
- Decided: do not chase the slot (a 3-4 deg window with a 0.19 m center error).
  Such a candidate is still reported by the final round (>= 1 observation,
  <= 0.30 m spread) when the run ends. Before adding angles or rings to
  `viewpoint_for`, it must discard spots not reachable from the robot; coverage
  already computes reachable areas, so that function goes to `mapa.py` for both.
- Mission does not go back for "no plan points left" while a doubtful candidate is
  still waiting for its visit.
- To check in the simulator: the robot leaves as soon as it arrives. If fusion
  needs more time to confirm, add a short wait at the spot.

## What if mission hits an unexpected error?

- Problem: an error in a callback goes up to `rclpy.spin`, `correr()` only catches
  Ctrl+C, and the node closes. Nobody restarts it: no more reports, goals or /done.
- Decided: `step`, `on_goal_response` and `on_goal_result` catch any error, log it
  with its traceback (at most every 5 s) and carry on. If a Nav2 callback fails,
  the goal counts as failed, so mission never waits forever for it.

## What if navigation gets stuck?

- Problem: while a goal is in progress mission only waits. If Nav2 gets stuck with
  a goal, mission waits forever and the judge ends the run after 60 s still.
- Decided: while a goal is moving, mission cancels it if it takes more than
  `goal_timeout_s` (90 s), or if the robot has not moved `stuck_distance_m` (5 cm)
  nor turned `stuck_angle_rad` (0.2 rad) for `stuck_time_s` (20 s). Movement is
  read from `odom → base_footprint`, like the judge reads /odom: AMCL corrections
  in `map` would look like movement. Without odom data the robot is not called
  stuck. After cancelling, mission waits for the goal's result (failed) and goes on
  as with any failed goal; a stuck return ends the run.
- Not covered: Nav2 never answering a goal (`sending` forever). Nav2 is down then,
  and no new goal would help.

## How does mission turn at a viewpoint?

- Problem: coverage publishes several poses at one viewpoint: same x,y, another
  heading. Mission sent each one with the plan's x,y. Nav2 calls a goal reached
  within `xy_goal_tolerance` (0.10 m) and can drift while it turns (0.14 m in
  laberinto_1), so the next turn was a new goal that needed the approach again:
  23 s stuck in a corner (hans-die's NOTES, "For other modules").
- Decided: if the robot is within `turn_in_place_m` (0.20 m) of the next plan
  point, mission sends the robot's own x,y with the plan's heading, so Nav2 only
  turns. The plan point, not the sent pose, goes into the list of finished
  points, so the 0.02 m / 0.1 rad identity still works. Without TF it sends the
  plan point.
- Why 0.20 m: Nav2 stops are median 0.058 m, p90 0.10 m, worst seen 0.14 m. A
  view from up to 0.20 m away is almost the same view.
- Side effect: a different plan point closer than 0.20 m is also only turned
  to, not driven to.
- Rule: `turn_in_place_m` stays above Nav2's `xy_goal_tolerance`
  (`test_turn_radius_covers_nav2_arrivals` checks it).
- Not chosen: Nav2's `Spin` action. It is a second navigation action, so an
  interface change with hans-die.
- Checked (laberinto_1, 300 s, batch 20261026-191613-5aa61f6, commit 5aa61f6):
  28 turns in place, all succeeded, 1.3 s each on average (max 3.7 s). 3 of them
  started 0.10-0.17 m from the plan point, beyond `xy_goal_tolerance`: before,
  each would have needed the approach again. No "Failed to make progress", no
  cancels, 70 of 70 finished goals succeeded. Score 100, same as development: the
  other two boxes are in areas the route never enters.

## How does mission keep goals away from boxes?

- Problem: boxes are not in `/map`, so a coverage view can fall on or next to
  one. In laberinto_simple mission sent the view (0.144, 1.969) with the red box
  at (0.0, 2.0): 50 s pushing it (hans-die's NOTES). Each collision is -50 (§8).
- Decided: when mission picks the next plan point, it skips points closer to a
  candidate than `box_clearance_m` + `box_clearance_sigmas` x its spread
  (`worst_deviation`). Only candidates with at least `box_min_observations` (2)
  count: a false single detection must not erase plan areas. A skipped point is
  not visited: it is checked again every time, so it comes back if fusion moves
  the candidate. If only skipped points remain, `first_pending` gives None and
  the robot goes back ("no plan points left"). Turns in place come from plan
  points, so they are skipped too.
- Why 0.42 m: at a goal the robot reaches the position and then turns, sweeping
  the circle of its farthest corner (0.181 + 0.02 back, 0.100 + 0.02 side: 0.234
  m). Plus half the box diagonal for any box orientation (0.25 x 0.707 = 0.177
  m): 0.411 m. A fixed 0.30 m rule (marcorita's warning) misses the corners.
  `test_box_clearance_covers_the_turning_robot_and_the_box` recomputes it from
  navigation's footprint and padding and perception's `box_side_m`.
- Checked (laberinto_simple, 120 s): after the first report at (0.03, 2.02),
  mission logged "Salteo el punto del plan (0.14,1.97)", the view 0.12 m from
  that box center where hans-die's run pushed the box for 50 s. The robot went
  on to the next view, reported both boxes and sent `/done` at 32 s.
- Open: 0.42 m + 1 sigma is the worst case (turning in place, box corner toward
  the robot, plus the spread). Facing the box without turning needs 0.26 m;
  turning with a box face toward the robot, 0.36 m. Not measured yet how many
  useful views it skips (laberinto_1 and maze_1 ran before the skip log). If it
  skips views at 0.42-0.55 m and loses coverage, first lower
  `box_clearance_sigmas` to 0.5, then `box_clearance_m` to 0.36.
- Problem: the rule only runs when mission picks the next point. A box found
  during the trip (often while approaching the view that sees it) did not stop
  the trip, so the robot could still drive into it.
- Decided: every tick, while a plan goal is `moving`, mission checks the plan
  point with the same `near_box` rule (`check_goal_near_box`). If it is now in a
  box zone, mission cancels the trip and sets `current_point` to None, so the
  point is not marked finished: it stays pending and comes back if fusion moves
  the candidate. It checks the plan point, not the sent pose (for a turn in
  place the sent pose is the robot's own x,y). Not checked: inspections, idle
  turns and the return (no plan point), a trip `check_stuck` already cancelled.
- Checked (Docker, `nav_falso.py`, 2026-10-27): plan A (1, 0) and B (0, 1.5),
  robot at the origin. A candidate at (1.2, 0) with 2 observations during the
  trip to A: mission cancelled it, skipped A and went to B. After the candidate
  was removed, it went back to A when B ended.
- Not covered: obstacles that are not red candidates (distractors, decoys);
  those need Nav2's costmap, an interface change with hans-die. Inspection
  spots near another candidate.

## How does mission skip spots it cannot reach?

- Problem: a free spot with a clear view of the box can be in a closed pocket
  (laberinto_1, see above). `viewpoint_for` picked the nearest spot without
  checking a route.
- Decided: use marcorita's `Grid.reachable_from` from `mapa.py` (#23,
  `notes/marcorita/MAP_REACHABILITY.md`). Mission builds `self.grid` once per
  `/map`. In `inspect_doubtful` it computes the reachable cells once, then
  `viewpoint_for` drops spots whose cell is not in that set, after the clearance
  and sight checks. Parameters: `visit_transit_radius_m` (0.15, see below)
  and `visit_free_threshold` (25, an integer).
- If the robot's own cell is not safe (`ValueError`: too close to a wall, or a
  bad pose), mission skips the inspection for now and does not count it as a
  try: the cause is the robot's position, not the candidate. The error text is
  logged, so a bad parameter (a float threshold also raises `ValueError`) shows.
- Tests (`test_inspeccion.py`): an unreachable nearest spot inside a pocket
  with a 10 cm slot is skipped for a farther one; no reachable spot gives None;
  a robot inside a wall raises.
- Checked (workshop 10 real-robot bag, 2026-10-27): no crash, same score as
  `development` (150/200). But the start pose (0, 0) is 0.165 m from a wall
  there, so with 0.21 every inspection at the start was skipped ("robot pose is
  outside the safe free space"). With 0.15 that cell is safe.
- Decided: radius 0.15 m. Coverage's 0.21 is the circle around the whole robot
  (for turning) and left the workshop start cell unsafe. Nav2 plans with a
  0.12 m circle (half width + 0.02 padding), so 0.15 is a bit stricter and never
  sends a goal Nav2 cannot plan. In a 0.55 m corridor the safe band grows from
  ~15 cm to ~25 cm. Tests pass with 0.15. Pending marcorita's OK.

## Next

- Run it in the simulator once Nav2, localization and coverage are in `development`.

## Testing without the other modules

- `notes/AlsinaBautista/nav_falso.py`: fake `navigate_to_pose` server. It accepts
  any goal, waits, answers succeeded (or aborted with `-p fallar:=true`) and
  accepts cancels. Fake `/map`, plan, candidates and TF with `ros2 topic pub` and
  `ros2 run tf2_ros static_transform_publisher`.
- Unit tests (no ROS needed) in `src/equipo_jar/test/`: `test_confirmacion.py`,
  `test_exploracion.py`, `test_mapa.py` and `test_inspeccion.py`.
