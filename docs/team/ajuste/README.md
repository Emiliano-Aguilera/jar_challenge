# Qualification tuning, 2026-11-04

Read this before you change `src/equipo_jar/params/` for the final. It records which params changed
between the two qualification runs, why, what the real robot did with them, and what is proposed for
the final (Thu 2026-11-05, one run, new layout, new N, no changes between runs).

Agents: treat the numbers here as measured on the real robot unless a line says "model", "offline"
or "simulator". Re-run the scripts in `scripts/` instead of trusting a number you need to rely on.

## Short version

| | Run 1 (13:19) | Run 2 (18:51) |
| --- | --- | --- |
| Commit | `265bfec` (frozen at 9:00) | `cec2c37` (same code, only `params/` changed) |
| Score | 250: 2 victims + return − 1 collision | 300: 3 victims, no penalties loaded |
| Victims (report time) | V2 72 s, V1 448 s | V2 57 s, V4 119 s, V3 147 s |
| End | `/done` at 448 s, back at the start | stopped by the organizers at ~521 s ("breaking the maze"), no `/done` |
| Upper third of the arena | never planned | explored, both victims there found |
| Rectangle on a `/map` wall | 1 % of poses, 2 contacts | 16 % of poses, 16 contacts |

- **What worked:** coverage reaching the upper third (2 more victims), Hans's cone params (no more
  167 s stuck behind a cone), fast confirmations, shorter coverage route.
- **What failed:** the robot touched and pushed walls, mostly in the upper third, and at the end got
  pinned against a wall for ~90 s while Nav2's recoveries kept turning it.
- **Main causes (see "Why the robot touched walls"):** ~0.4 s command delay at up to 0.5 m/s;
  Spin/BackUp recoveries turning or backing into walls; ghost obstacles in the local costmap after
  turns that triggered those recoveries; view points only ~3 cm from walls when turning in place.
- **Proposal for the final:** branch `mision/params-final` (`c160808`, on top of `cec2c37`). Keep
  everything that worked; slow down to 0.35 m/s, local persistence 0.3 s, slower Spin that looks
  further ahead, views 3 cm further from walls. Table below.

## The real qualification map

The `/map` the judge published in the qualification: 174 × 383 cells, 0.02 m, origin
(−0.553, −0.572). Grupo_de_Lie's mission node prints the map it receives (zlib + base64), and the
organizers share every team's logs with all teams, so `scripts/decode_map_from_log.py` can rebuild
it from their run-1 log (`salida_del_equipo/pr14_c1_1540.txt` in `Grupo_de_Lie_corrida1_2026-11-04.zip`).

**It is not in git:** `tools/verificar_estructura.py` rejects map files anywhere in the repo (§10),
so CI would fail. Rebuild it into `build/` (ignored by git) or ask Bautista for his copy:

```bash
python3 docs/team/ajuste/scripts/decode_map_from_log.py PATH/TO/pr14_c1_1540.txt build/mapa_qualy/qualy_2026-11-04
```

Check that it is the right map: our coverage planner on it with the run-1 params gives 58 views and
327 of 1056 samples unreachable. Our run-1 log says 57 and 326.

- Boxes and cones are **not** in `/map`. Only walls.
- The opening to the upper third measures ~0.45 m in `/map`: the upper third is reachable for
  coverage only with a transit clearance of 0.209 m or less.
- Rule of thumb for coverage on a 0.02 m map: transit clearance `t` (`robot_radius + safety_margin`)
  opens passages from about `2t + 0.04` m. 0.21 → 0.46 m, 0.18 → 0.38 m, 0.16 → 0.34 m.

## Run 1: what went wrong (log analysis)

1. **Upper third never planned.** Coverage transit was 0.18 + 0.03 = 0.21 m, which closes the
   0.45 m opening. 326 of 1056 samples unreachable; the two missing victims were up there.
2. **167 s stuck behind a cone** in the bottom corridor (210–377 s). Three plan points were behind it;
   Nav2 thought it could pass beside the thin cone. Each trip ran until mission's 10-recovery cutoff
   (54, 37 and 66 s). No trip with 4 or more recoveries reached its goal; the ones that arrived had ≤ 3.
3. **V1 confirmed late.** Seen 14 times, always from one spot (0.1, 2.36). Fusion keeps the
   uncertainty at ≥ 0.15 m while a box is seen from one spot (`single_view_sigma_m`), and mission
   needs ≤ 0.10 m. The inspection spot (0.78, 3.56) had a wall right next to the box and saw nothing.
   V1 was reported only in the final relaxed round, at 448 s (6.8 cm off).
4. **Same spots visited several times.** 6 spots got 2–3 separate trips. Real trips cost ~4 s fixed +
   2.8 s per straight-line meter (fit on 27 trips); the plan charged distance / 0.25 m/s + 1 s, so it
   split the headings of one spot into separate visits.
5. **One collision with a cone** near (0.5, 0.75) around 25 s (MPPI "Optimizer fail" there).

## Changes for run 2 (commit `cec2c37`, PR #16)

| Param | Run 1 → Run 2 | Owner | Why | What run 2 showed |
| --- | --- | --- | --- | --- |
| `observation_persistence` (both costmaps) | — → 1.0 | Hans | Thin cones flickered out of the costmaps between scans | No cone stall. But see ghost obstacles below (local costmap) |
| `mision.max_recoveries` | 10 → 4 | Hans / mision | Good trips used ≤ 3 | Stuck trips cut fast (25 cuts); several trips still arrived after 3 recoveries |
| `mision.goal_timeout_s` | 90 → 60 | Hans / mision | Good trips took ≤ 15 s | OK |
| `cobertura.robot_radius` | 0.18 → 0.15 (transit 0.18) | Marco | Open the 0.45 m opening | Upper third explored: +2 victims by 147 s |
| `cobertura.travel_speed` | 0.25 → 0.15 | Marco | Plan underpriced each trip | Offline on the real map: route 46 m instead of 62 m |
| `cobertura.max_views` | 80 → 120 | Santos | Whole arena needs ~80 views | 87 views planned |
| `fusion.single_view_sigma_m` | 0.15 → 0.09 | Santos | V1 waited 300 s for a second viewpoint | 3 reports by 147 s, all within 10 cm |
| `deteccion.sync_queue_size` | 60 → 120 | Santos | Other teams' logs: scans up to 2.8 s late on the venue Wi-Fi | OK |
| `mision.visit_box_margin_m` | 0.25 → 0.15 | mision | The V1 inspection spot looked through a wall | No inspection needed in run 2 |
| `mision.extra_search_enabled` (+ 5 `extra_*`) | false → true | mision | Use spare time on unseen floor | Started at 448 s, found 0 views, went back. Harmless |
| `mision.extra_transit_radius_m` | 0.21 → 0.18 | mision | Same transit as coverage | — |

Not changed on purpose: `local_costmap.footprint_padding` (stays 0.02; 0.01 was considered for the
doorway and dropped as untested), `mision.visit_min_observations` (stays 2).

How the coverage transit was chosen, on the real map (offline, `scripts/coverage_on_map.py`; the
0.18–0.14 rows use run 2's `travel_speed` and `max_views`; real-time estimate = run-1 trip fit, at 0.5 m/s):

| Transit | Views | Visible samples | Upper victims in view | Real-time estimate |
| --- | --- | --- | --- | --- |
| 0.21 (run 1) | 58 | 729 / 1056 | none | 295 s |
| 0.18 (run 2) | 81 | 905 / 1056 | both, ~125 s | 400 s |
| 0.16 | 81 | 918 / 1056 | both | 378 s |
| 0.14 | 87 | 918 / 1056 | both | 413 s |

0.16 only added 13 samples, in pockets behind 0.36–0.40 m passages, so 0.18 was used.

## Run 2: what the robot did

- 0–147 s: lower area, crossed into the upper third, reported V2, V4 and V3.
- 147–448 s: the plan went down, up and down again (see "Known issues"), with 25 trips cut by
  `max_recoveries` and many wall contacts.
- 448 s: plan finished, extra search found nothing, mission started the trip back.
- 431–521 s: pinned near (0.24, 4.81), next to the upper-left passage. The global planner refused
  every path ("Starting point in lethal space"), Spin and BackUp stopped with "Collision Ahead", and
  the trip back is not capped by `max_recoveries` (two return goals with 21 and 22 recoveries). The
  robot kept turning against the walls (its heading went through every direction) until the
  organizers stopped it.
- Localization was fine all run: LiDAR-to-map score median 0.85–0.98 per 30 s window.

Counts from our `log.txt` (repeated lines are collapsed as `[+N same]`, so these are lower bounds):

| Log line | Run 1 | Run 2 |
| --- | --- | --- |
| MPPI `Optimizer fail to compute path` | 12 | 42 |
| Smac `Starting point in lethal space` | 0 | 18 |
| Spin/BackUp `Collision Ahead` | 5 | 38 |
| `Running spin` / `Running backup` | 5 / 4 | 25 / 22 |
| `Failed to make progress` | 18 | 1 |
| Trips cancelled by mission | 3 | 25 |

## Why the robot touched walls

Method: `scripts/contacts_on_map.py` puts the chassis rectangle (0.119 m ahead, 0.181 m behind,
0.100 m to each side) on the real map at every AMCL pose of our log and measures the clearance to the
walls. AMCL is ~3–5 cm off, so single poses can be wrong; 16 % vs 1 % of poses is not noise.

| | Run 1 | Run 2 |
| --- | --- | --- |
| Poses with the rectangle on a wall cell | 1 % | 16 % |
| Poses with < 2 cm to a wall | 2 % | 22 % |
| Median clearance | 0.148 m | 0.090 m |
| Contacts (entering overlap) | 2 | 16 |

12 of the 16 run-2 contacts were more than 0.3 m from anywhere run 1 went (the upper third and the
passage into it); the other 4 were in the middle of the arena, where run 1 also passed. Corridors
there are ~0.4–0.7 m wide. Causes, in decreasing order of how sure we are:

1. **Recoveries turn or back into walls.** 12 of 16 contacts began within 3 s after a Spin, BackUp
   or Wait; in 10 of them a Spin (5) or a BackUp (5) had just stopped with "Collision Ahead". With
   ~0.4 s of delay the robot keeps moving after the stop. Spin turns at up to 1.0 rad/s and only
   checks 1.0 s ahead (`behavior_server.simulate_ahead_time`). The rear corners are 0.207 m from the
   center, so a turn sweeps them into a wall that the straight rectangle cleared.
2. **Command delay at speed.** The real robot answers ~0.4 s late (measured in run 1). The maximum
   commanded speed in the 3 s before each contact was 0.11–0.44 m/s; at 0.4 m/s that is 16 cm past
   what MPPI planned. MPPI does not slow down near walls: mean commanded speed 0.13 m/s with < 6 cm
   of clearance, 0.14 m/s with > 15 cm. MPPI's `CostCritic` only forbids the rectangle touching a
   lethal cell, and its weight is low (1.0), so it does not push away from walls.
3. **Ghost obstacles after turns (local costmap).** The local costmap is in the `odom` frame and the
   real odometry is off in turns. With `observation_persistence: 1.0`, 1 s of old scans is redrawn
   with the wrong heading. Run 2 had 3.5× the MPPI failures of run 1, and 32 of 42 came right after
   more than 20° of turning (76 %, against 58 % of the run's time); in run 1, 5 of 12 (42 %). Each
   failure starts a recovery near a wall (cause 1). This is the least certain of the four: the bags
   have no `/scan`, so the ghosts cannot be seen directly.
4. **View points close to walls.** Coverage puts views ≥ 0.24 m from walls (`view_clearance`), and
   mission turns in place at each view. Turning sweeps 0.207 m, so ~3 cm is left before AMCL error.
   Offline on the real map, run 2's plan had 11 views closer than 0.27 m to a wall (run 1: 7 of 57).

Once touching, the robot cannot get out by itself: Smac 2D treats the start as blocked when the robot
center is within 0.12 m (padded half-width) of an obstacle, so there is no path, and both recoveries
refuse to move.

## What another team did (Grupo_de_Lie, run 1, 15:39)

500 points: 4 victims + return, no collisions, `/done` at 503 s. Their own stack, not Nav2. From the
params block of their log: `v_max: 0.35`, `a_max: 0.35`, `w_max: 1.0`, `reaccion_s: 0.4` (they model
the 0.4 s delay), `margen_seguridad: 0.04`, `huella_margen: 0.05`. They also struggled at the opening
to the upper third: from 95 s to 193 s they tried and backed off several times, and crossed
diagonally near x ≈ 1.4.

## Proposal for the final

Branch `mision/params-final` (`c160808`, parent `cec2c37`): only `params/`. Passes
`tools/verificar_params.py`, `verificar_estructura.py` and `verificar_estatico.py`.
Not yet run in the simulator or on the robot.

| Param | Run 2 → Final | Owner | Against | Cost / risk |
| --- | --- | --- | --- | --- |
| `limites.lineal_max_ms` | 0.5 → 0.35 | Hans | Cause 2 | Slower trips. Lie's value |
| `limites.reversa_max_ms` | 0.35 → 0.25 | Hans | Cause 2 | — |
| `local_costmap...observation_persistence` | 1.0 → 0.3 | Hans | Cause 3 | A cone missed for > 2 scans can flicker again. Global keeps 1.0, so the planner still routes around cones |
| `behavior_server.max_rotational_vel` | 1.0 → 0.6 | Hans | Cause 1 | Spin takes ~2.6 s instead of ~1.6 s. Must stay above `min_rotational_vel` (0.5) |
| `behavior_server.simulate_ahead_time` | 1.0 → 2.0 | Hans | Cause 1 | Spin/BackUp refuse more often (they stop instead of moving) |
| `cobertura.view_clearance` | 0.24 → 0.27 | Marco | Cause 4 | Offline on the real map: 73 views (none closer than 0.27 m to a wall), 893/1056 visible (−1 %), real-time estimate ~355 s (−40 s), both upper victims still in view |

Keep from run 2: everything in the run-2 table above.

Considered and not proposed:
- `local_costmap.footprint_padding` 0.02 → 0.03: door model (`scripts/door_fit.py`) says a 0.44 m
  opening would fit ~30° of headings instead of ~42–57°; the qualification opening was ~0.45 m.
- Coverage transit back to 0.21: the upper third was worth 2 victims. The new layout may have similar
  openings (the organizers reuse the panels).
- MPPI `CostCritic.cost_weight` 1.0 → ~2–3 (keep away from walls): the most direct fix for cause 2,
  but it changes how MPPI enters narrow openings (why Hans set it low). Only with a simulator check.
- Local costmap with 2.5 cm cells: more room in doorways, but 4× the cells on a laptop that already
  misses its 20 Hz.

The door model (`scripts/door_fit.py`, rough): 0.3 s persistence gives the same or more room than
1.0 s in doorways (0.44 m: ~42° → ~57° of headings that fit).

## Testing before the final

The simulator has no command delay and no heading error in turns, so it will not reproduce the
contacts (0 collisions in every qualification-map run we checked). Use it only to check that nothing
regresses: doorways still crossed, plan and reports as before, return OK.

```bash
gh workflow run simular.yml -R Emiliano-Aguilera/jar_challenge \
  -f rama=mision/params-final -f mapas=tars_qualy_real,conos_pista,maze_3_6x6 -f duracion=600
```

## Facts about our stack learned here

- Nav2's `number_of_recoveries` also counts every costmap clear (`ClearEntireCostmap` increments it,
  nav2_behavior_tree Humble), and our tree clears on every failed path or follow. So
  `max_recoveries: 4` is not "4 maneuvers"; Spin, Wait and BackUp still ran in trips cut at 4.
- Smac 2D: a cell with cost ≥ INSCRIBED (within the padded half-width, 0.12 m, of an obstacle) is
  blocked, the start included.
- Local costmap: LiDAR only (no `/map`), `odom` frame, 5 cm cells. Global: `/map` + LiDAR, `map`
  frame, takes the `/map` resolution (0.02 m on site).
- `max_recoveries` does not apply to the trip back to the start.
- BackUp speed (0.15 m/s) and Spin angle (1.57 rad) are in `behavior_trees/navegar.xml`, not in
  `params/`, so they cannot change after the delivery cutoff.

## Known issues that need code (not for the final)

- The coverage order went lower → upper → lower → upper → lower → upper (offline on the real map, also
  with `travel_speed` 0.25 and with order checks off). Each crossing of a narrow opening costs time
  and risk.
- No way out when pinned against a wall (no path, recoveries refuse), and the trip back has no
  recovery cap.
- Nav2 plans through gaps from ~0.25–0.30 m (robot center > 0.12 m from both sides) even when
  coverage treats them as closed.

## Open questions

- Will collisions in the final be counted per contact, and will the organizers stop a run that pushes
  walls? In run 2 none were loaded and the run was stopped.
- LiDAR minimum range on the real robot (`sllidar_node`): if it is ~0.15 m, a wall touching the
  side is invisible right after a costmap clear, and recoveries cannot see it. Not checked.

## Files

| Path | What |
| --- | --- |
| `runs/run1_log.txt`, `runs/run2_log.txt` | Our `log.txt` from both runs (shared with all teams by the rules) |
| `runs/run*_result.json` | Organizers' judge result per run |
| `runs/run*_odom_2hz.csv`, `runs/run*_cmd_vel.csv` | Organizers' `/odom` (2 Hz) and `/cmd_vel` from the bags |
| `scripts/decode_map_from_log.py` | Rebuild a `/map` printed in a log (the qualification map, see above) |
| `scripts/coverage_on_map.py` | Our coverage planner offline on a map, per params variant |
| `scripts/contacts_on_map.py` | Rectangle-vs-wall contacts along the AMCL track of a log |
| `scripts/door_fit.py` | Rough doorway model for MPPI's rectangle in the local costmap |

From the repo root, after rebuilding the map (needs `numpy` and `pyyaml`; `scipy` only makes it faster):

```bash
python3 docs/team/ajuste/scripts/contacts_on_map.py build/mapa_qualy/qualy_2026-11-04.yaml \
  docs/team/ajuste/runs/run1_log.txt docs/team/ajuste/runs/run2_log.txt
python3 docs/team/ajuste/scripts/coverage_on_map.py build/mapa_qualy/qualy_2026-11-04.yaml \
  src/equipo_jar/params/params.yaml '{"final": {"robot_radius": 0.15, "max_views": 120,
  "travel_speed": 0.15, "view_clearance": 0.27}}' --start -0.03 0.05 -0.03 --point 0.08 6.71 --point 2.77 5.57
```
