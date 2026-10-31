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
  it when it sends the first goal (`go_to`). Reports also wait for the start (since
  2026-10-29; before, they were checked from /map on).
- First version: no goal during the first `start_wait_s` (4 s) after /map. That was
  measured in the simulator; the real robot can take longer or shorter.
- Decided (2026-10-29): mission starts on the first `true` of
  `/jar/localizacion_lista` (hans-die, #34: false until AMCL has done its start-up
  no-motion updates). `start_ready` (`exploracion.py`); `on_localization_ready` keeps
  only the first `true` (later `false` after turns is ignored). Max wait
  `localization_wait_max_s` (10 s) in case the flag never comes: the robot stands
  still until then and 60 s still ends the run. Starting at the max wait logs a
  warning once. `start_wait_s` was removed (marcorita's `pruebas/evaluar_presupuesto.py`
  read it).
- Checked (2026-10-29, maze_1 and maze_3, 300 s, batch 20261029-192059-2c554b7): the flag
  turned `true` 2.3-2.4 s after /map; the first goal went out at 3.2-3.6 s, when the
  coverage plan and Nav2's `bt_navigator` were also ready (before: 4 s fixed). No max-wait
  warning. 500 and 600, 0 collisions, judge's distance to the start 0.031 / 0.051 m. Done
  at 194 / 170 s (182 / 153 s on the previous branch): exploration varies run to run;
  maze_1's trip to (0.03,-2.82) took 41 s with 8 Nav2 recoveries (37 s with 7 before).
- Checked with a hand-placement offset (2026-10-29, maze_1, 300 s, `largada=0.15,-0.1,0.2`,
  batch 20261029-193656-2c554b7): AMCL's pose at 0.7 s was (0.03,-0.02, yaw 0.01), still the
  guess at the origin, 0.14 m off. The flag turned `true` at 2.3 s; the start pose was fixed
  at 2.8 s at (0.12,-0.07, yaw 0.17), ~0.04 m and 0.03 rad from the real start. Return:
  mission 0.035 m, judge 0.03 m from the real start; 500, 0 collisions, done at 177 s.
  Fixing the start with the first estimate would have put the return ~0.14 m off, outside
  the 0.10 m the 4-wheel rule leaves.
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
- Failed trip (2026-10-29): if Nav2 does not reach the spot, the candidate is taken out
  of the visited ones and the failure counts as a try, with the observations it has
  then (`forget_failed_inspection`, `inspeccion.py`). Same rule as "no spot": tried again
  only with more observations and fewer than `visit_max_attempts` tries. Before, a failed
  trip counted as a visit and the candidate was never looked at again.
- Costmap (2026-10-29): `viewpoint_for` gets `blocked(x, y)` and skips spots occupied in
  Nav2's costmap while it chooses, so a free spot can win. Before, mission checked only the
  chosen spot and counted a try if it was occupied.
- Not covered yet: with no spot the robot does not turn to face the candidate from where
  it is. (A trip that arrived without new observations counted as a visit: fixed on
  2026-10-31, see below.)
- Checked (2026-10-29, branch `mision/inspecciones-y-vuelta`, 9aa5032): laberinto_1 600 s:
  300, 2 of 3 boxes (run 40978622059). The pocket box (-1.92,1.47) had no spot 3 times and
  was reported by the final round after the return; the third box was never seen
  (coverage). No failed inspection trip happened, so the new retry did not act. maze_1
  300 s: 500, done at 208 s (run 40985063777, recorded). An earlier maze_1 run of the same
  commit (40978622059) scored 0: from 25 s Nav2's controller dropped every plan
  ("Resulting plan has 0 poses in it", 274 times) near (3.0,0.3) and every trip failed;
  development alone scored 500 right after (40985060046), so it is intermittent and in
  navigation, not in this branch (sent to hans-die).
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
- Seen (Codex, custom world Ventanas, 2026-10-31): a candidate with 2 observations was
  inspected; Nav2 arrived, it still had 2, and it counted as inspected. Later it had 5
  with 0.15 m (fusion's one-spot floor, `single_view_sigma_m`) and was reported only by
  the final round at 360 s: the report scored, but the run lost the +200 bonus. With
  `settle_s` 0 the robot left the spot in the same tick it arrived.
- Decided (2026-10-31): arriving is not enough. Mission stays at the spot until the
  candidate is reported or `inspection_wait_s` (2.0 s) passes (`look_at_inspection`;
  `inspection_outcome` in `inspeccion.py`). Reports are checked every tick of the wait.
  "No plan points left" does not start the return while an inspection trip or its wait
  is on (`current_inspection`): when the inspection was the last thing left, the arrival
  came in the same tick and mission went back before waiting (seen in `mundo_falso.py`;
  `development` did the same). Still doubtful after
  the wait: `unhelpful_inspection` takes it out of the inspected ones and counts one
  attempt, without asking for new observations: the robot is at this spot, so
  `viewpoint_for` picks another one >= 0.4 m away, which also lifts fusion's one-spot
  floor (`min_baseline_m` 0.3). Gone (fusion merged or dropped it): taken out, no more
  wait. `visit_max_attempts` (3) counts both kinds of attempt; a failed trip after an
  unhelpful one again needs new observations.
- Cost: only for doubtful candidates (1 in Ventanas): 2 s per view plus the trip.
- Decided (2026-10-31): before going back for "no plan points nor doubtful candidates",
  mission waits until that has held for `final_look_s` (1.0 s; coverage plans
  `observation_time` 1.0 s per view). In `mundo_falso.py` the box was seen only from the
  last plan point, and mission went back 0.1 s after arriving, before fusion sent the
  candidate: it was reported only by the final round. Costs 1 s once per run.
- Checked (`mundo_falso.py`, first view hidden, 2026-10-31): 1st view gave nothing, 2 s,
  2nd view 0.42 m away, reported at 19 s. Same world with `development`: the robot left at
  once, never went back, and the box was reported by the final round at `/done`.

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
- Problem (2026-10-28): the motion test never fired in 23 server runs. Nav2's own
  recoveries (spin, backup) move the robot and reset the stuck timer, so a goal on an
  obstacle that is not in `/map` took 27-44 s to fail.
- Decided: mission reads `number_of_recoveries` from Nav2's feedback (it counts each
  costmap clear, spin, wait and backup in the trip) and cancels an exploration trip
  when it reaches `max_recoveries` (10, an integer). Not for the trip back. Each
  trip's count is logged with its status, to tune the limit.
- Checked (maze_1, 300 s, batch 20261028-100454-363bade, RTF 0.96): the goal next to
  the blue box was cancelled at 21 s with 10 recoveries (27-28 s before at the same
  RTF). The trip through the narrow gap next to the red box reached 7 and went on, as
  it should. The return reached 10 and was not cut. Score 500, back, done at 209 s
  (220-233 s before). Healthy trips: 0 recoveries.
- Why not lower: the gap trip needs 7 before it passes; 8 would save ~5 s on blocked
  goals but leaves no margin. The real fix for blocked goals is checking them against
  Nav2's costmap (see Next).

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

## How does mission let localization correct after a stop?

- Problem (hans-die, real-robot bag from the workshop): the wheels count turns
  ~25 % too much, so after a turn the robot believes it faces 10-20 deg past its
  real heading. AMCL fixes it with the LiDAR only while the robot moves. If the
  camera sees a box right after the stop, the box lands off in the map (one at
  0.30 m, plus a false box). The simulator's wheels are exact, so simulated runs
  do not show it.
- hans-die's side: localization detects a turn followed by a stop and asks AMCL
  for an update without motion. The heading is fixed in ~1 s (offline on the
  bag: 11 deg -> 3.6 deg at stops).
- Decided (mission side): after every trip that ends (arrived or failed: Nav2's
  recoveries also turn), mission waits `settle_s` before the next goal
  (`explore()`, `settle_until`). Reports are still checked during the wait, and
  a reason to go back ends it: the trip back starts without the wait (AMCL
  corrects while moving). `settle_s` 0 = no wait (simulator); ~1 s planned for
  the real robot.
- Cost: per trip, not per viewpoint. Turns in place are separate trips: ~70
  trips in a 300 s laberinto_1 run, so 1 s adds ~70 s against the 300 s bonus.
  Tune it in Rosario (0.5 s may be enough). Write it as a float (`1.0`): an
  integer makes the node fail at start.
- Not covered: observations taken right after the stop, before AMCL corrects,
  still reach fusion. Dropping them needs perception to know when localization
  is correcting (an interface change; asked hans-die).
- The same wait gives fusion more time at inspection spots (see "How does
  mission look at doubtful candidates?").
- Checked (Docker, `nav_falso.py`, 3 s trips, `settle_s` 2.0, 2026-10-27):
  2.17 s between the end of trip A and the start of trip B (0.15 s without the
  wait). The trip back started 0.19 s after the last trip, as designed.
- Update (2026-10-29): AMCL now takes the heading from the IMU (hans-die, #44). On the
  bag the heading at stops went 10.9° -> 1.8°, and localization's fix after turns
  (#34) only adds 1.8° -> 1.3°. Mission does not wait for `/jar/localizacion_lista`
  after trips (it would cost up to 21-34 s per run in the sim); `settle_s` stays 0.

## How does mission check the return?

- Problem: when the trip back ended (arrived or failed), mission sent `/done` at once.
  A failed return ended the run far from the start, and the final error was not logged.
- Rule (organizers, 2026-10-28): a tape mark where the robot starts, radius 0.25 m;
  the bonus counts if the 4 wheels are inside the circle at `/done`. Wheels (simulator
  URDF, `mecanum_wheel.urdf.xacro`): centres at x +-0.080 m, y +-0.0845 m from
  `base_footprint`, radius 0.0325 m, width 0.0304 m. Farthest wheel point 0.150 m, so
  the position error (localization + arrival) must stay under 0.10 m (contact points
  only, 0.116 m, would leave 0.134 m).
- Decided: after the trip back, state `checking`. Mission waits until the robot is
  still (moved < `return_still_m`, 0.01 m, in `odom` for `return_still_s`, 2.0 s), then
  compares its position in `map` with the start pose. `return_verdict` (`exploracion.py`)
  decides: within `return_tolerance_m` (0.05) -> `stopping`; otherwise, while fewer than
  `return_max_tries` (3) trips back were sent, it sends the start pose again; else it
  stops anyway. `finish_time_s` still ends it.
- The trip back uses navigation's precise tree (2026-10-28, #33): `go_to` sends
  `behavior_trees/vuelta_precisa.xml` in the goal's `behavior_tree`
  (`precise_goal_checker`: 0.05 m, any heading). Other goals send an empty string and
  use the default tree (0.10 m, 0.25 rad).
- Why only the position: the wheels are symmetric around `base_footprint`, so the
  heading does not change the bonus, and the precise checker never turns the robot to
  fix it: a heading check would spend the retries for nothing.
  `return_yaw_tolerance_rad` was removed.
- Why 0.05: the precise checker accepts that much. Smaller would retry trips Nav2
  already calls reached, without moving the robot; 0.10 or more leaves no room for the
  localization error (`test_return_tolerance_fits_the_precise_goal_checker_and_the_rule`).
- Log: "Regreso N: a X m de la largada". Compare with the judge's
  `distancia_a_largada` (true pose, simulator) to see the localization error.
- Checked with the first version (0.10 m and 0.25 rad, default tree; 300 s, batches
  20261028-104330-02aff82 and -2, RTF 0.96-0.97): both returns passed on the first try,
  the check took 0.5-0.7 s. maze_1: mission measured 0.081 m, the judge (true pose)
  0.045 m; maze_3: 0.083 m and 0.082 m. Nav2 stops as soon as it is inside its 0.10 m
  tolerance, so the error is mostly that tolerance, not localization. maze_3's 0.082 m
  leaves 1.8 cm under the 4-wheel rule (0.10 m): that is why the precise tree was
  needed. Scores 500 and 600, done at 204 s and 159 s, no collisions or close calls.
- `return_still_s` 0.5 -> 2.0 s (2026-10-29, hans-die's suggestion after #48): localization
  fine-tunes AMCL with a LiDAR match 0.3-2.5 s after the robot stops; at 0.5 s mission
  measured AMCL before it. Costs ~1.5 s per try.
- `return_check_max_s` (10 s, 2026-10-29): if mission cannot measure in time (no pose, or the
  robot never stays still), it stops and sends `/done` anyway. Without it the robot could
  stand still for 60 s, which ends the run without `/done` and loses the +100.
- Seen (maze_1, run 40985063777): first try measured 0.060 m (> 0.05), the retry 0.022 m;
  judge 0.031 m.
- Checked with the precise tree (2026-10-29, runs of `mision/objetivos-ocupados`, 300 s):
  both returns passed on the first try; mission measured 0.034 m (maze_1) and 0.045 m
  (maze_3); +100 in both.
- Problem (Codex, custom world Callejones, 2026-10-31): 5 of 5 at 143 s, then the trip
  back failed 3 times and the run ended 4.86 m from the start. NavFn (Humble) stops
  building a path after 4 x the costmap's longer side in cells, half a cell per step
  (`max_cycles` in `navfn_planner.cpp`): ~14 m on a 6.9 m map, and the way back was
  ~28 m. It is in Nav2's code, not a parameter, so the same goal fails again. Navigation
  can change the planner (asked hans-die: Smac 2D); mission's part, either way:
- Decided (2026-10-31): the trip back in legs (`vuelta.py`). Once, mission counts the
  steps from the start to every cell where the robot fits on `/map` (breadth-first, along
  x and y, with `visit_transit_radius_m` and `visit_free_threshold`; the start snaps to
  the nearest such cell within `return_snap_m`, 0.3 m). It goes by legs when that way
  back is longer than `return_direct_max_m` (10 m; 0 = only after a failed direct try),
  or after a direct try fails. Each leg ends `return_leg_m` (5 m) along the way, facing
  along it, shorter if it would end on a costmap obstacle or near a box. Legs are not
  measured and do not use up `return_max_tries`. When the start is closer than one leg,
  the usual direct trip (precise tree) and check follow. At most `return_max_legs` (10),
  and none after `return_max_failed_legs` (2) failed legs in a row (the next one would
  leave from the same spot to the same goal); after that, or on any error, straight to
  the start as before.
- Why 10 and 5: steps along x and y are never shorter than the real path, so 10 m on
  `/map` stays under NavFn's ~14 m, and a 5 m leg leaves room for Nav2's detours. If
  navigation moves to Smac, `return_direct_max_m` 0 keeps the legs only as a fallback
  (params only).
- Checked (`mundo_falso.py`: way back 14.8 m, fake planner limit 8 m, 2026-10-31): 2 legs
  and the direct trip: 0.00 m, `/done` at 38 s. With `return_direct_max_m` 0: one failed
  direct try, the same legs, 0.00 m. Same world with `development`: 3 failed tries,
  `/done` 3.44 m away. Steps: 0.15 s once on a 6 x 6 m map; < 1 ms per leg.

## How does mission skip goals on obstacles that are not in `/map`?

- Problem: blue boxes and decoys are not drawn in `/map`. A plan point on top of one was
  sent anyway and Nav2 spent time until it failed: maze_1 (4.08,0.78) failed in 3 of 3
  runs, 27-44 s each.
- Decided (2026-10-29): mission reads Nav2's global costmap, `/global_costmap/costmap`
  (`OccupancyGrid`, reliable + transient local, full grid at 1 Hz since #33; values 0-100:
  100 obstacle, 99 the robot's center there would touch, 1-98 margin only, 0 free, -1
  unknown). `goal_blocked` (`exploracion.py`): a point is blocked if its cell is >=
  `costmap_blocked_value` (99). Unknown or outside the grid is not blocked: Nav2 decides.
- Before sending: a blocked plan point is marked visited and skipped. Inspection spots are
  checked inside `viewpoint_for` (since 2026-10-29; before, only the chosen spot, and a
  blocked one counted as a failed try). Turns in place are not checked: the robot stays
  where it is.
- While moving (`check_goal_blocked`, like `check_goal_near_box`): if the plan point
  becomes blocked, mission cancels the trip; the next `explore()` sees it blocked and
  skips it. Not for turns in place, inspection trips or the trip back.
- Checked (2026-10-29, 300 s, 1 run per map): maze_1 500 of 500, done at 182 s (before:
  196-250 s); the trip to (4.08,0.78) was cancelled 1 s after it started and the point
  skipped. maze_3 600 of 600, done at 153 s, 3 of 3 reports (before: always 300); the
  trip to (-0.30,-3.15) was cancelled with the robot already within 0.20 m, so it
  turned in place there and saw the third box. One run per map: part of the maze_3
  gain may be run-to-run variation.

## Next (2026-10-28)

Done (mission): cancel exploration trips at `max_recoveries` Nav2 recoveries (see
"What if navigation gets stuck?"); precise return (see "How does mission check the
return?"); goals on unmapped obstacles (see "How does mission skip goals on obstacles
that are not in `/map`?"); start on localization's flag (see "Where is the start
pose?").

Pending, in this order:
1. Long trips back: done on 2026-10-31, in legs (see "How does mission check the
   return?"). (The A* failures to extract existing paths were fixed in #33: Dijkstra.)
2. Tests: unmapped obstacles other than the blue boxes (a decoy blocking a corridor, one
   near the start): the simulator cannot add them yet (asked Emiliano-Aguilera). Shifted
   and turned starts: done (`largada`, see "Where is the start pose?" and hans-die's #48).
3. Not done, on purpose (Codex audit, 2026-10-29): a failed coverage trip still counts as
   visited. Retrying it would repeat trips like maze_1's (0.03,-2.82) gap, 30-40 s each
   (hans-die's notes), with no evidence it adds points. A goal whose acceptance never
   comes (state `sending`) has no timeout: only seen in a synthetic test; `finish_time_s`
   still ends the run.

For other modules:
- Perception: the point cloud subscription is always created (`deteccion.py`;
  `use_cloud_fallback` is true by default). §7.1 warns it can add latency. Create it only
  when the fallback is on, and decide if it stays on.
- Navigation: speed limits are per axis (0.5 m/s in x and y), so a diagonal can reach
  0.71 m/s. Measure sqrt(vx^2 + vy^2) on `/cmd_vel`.
- Coverage/perception: a box seen through a gap may be unreachable (laberinto_1 V2).
  Check how walls with gaps look in `/map`.

Before the cutoff (Tue 3/11 9:00): every wait, tolerance, max tries and threshold as a
named parameter in `params/`.

## Testing without the other modules

- `notes/AlsinaBautista/nav_falso.py`: fake `navigate_to_pose` server. It accepts
  any goal, waits, answers succeeded (or aborted with `-p fallar:=true`) and
  accepts cancels. Fake `/map`, plan, candidates and TF with `ros2 topic pub` and
  `ros2 run tf2_ros static_transform_publisher`.
- `notes/AlsinaBautista/mundo_falso.py` (2026-10-31): fake map (an S, way back ~15 m),
  TF, plan, fusion and Nav2 in one node. Nav2 moves the robot to each goal and fails
  paths longer than `limit_m` (8 m, like NavFn); fusion gives nothing at the first spot
  where the robot stands near the box. Prints a RESULT line at `/done`. Run it, then
  mission with `-p cantidad_victimas:=1` (commands in its docstring).
- Unit tests (no ROS needed) in `src/equipo_jar/test/`: `test_confirmacion.py`,
  `test_exploracion.py`, `test_mapa.py`, `test_inspeccion.py` and `test_vuelta.py`.
