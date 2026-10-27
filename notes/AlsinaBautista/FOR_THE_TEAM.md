# For the team, from mision

Things found while testing mission that belong to other modules or to everyone.
Each item says who it is for. Answer Bautista, or write the decision in your own
`NOTES.md`. Mission's own decisions and evidence are in [`NOTES.md`](NOTES.md).

## 2026-10-26

### Done in mission (in `development`)

- #17: a turn goal uses the robot's current x,y when it is within 0.20 m
  (`turn_in_place_m`) of the plan point, so Nav2 only turns. laberinto_1: 28
  turns per run, all succeeded, no "Failed to make progress".
- #18: a candidate with no spot to look from no longer counts as inspected.
  Mission tries again only when it has more observations, up to 3 tries
  (`visit_max_attempts`).
- #19: first goal after 4 s instead of 8 s (`start_wait_s`). Measured after
  `/map`: AMCL settled at ~2.2 s, Nav2 active at ~3 s, coverage plan at ~5 s.
- #20: mission skips plan points closer than 0.42 m + 1 sigma to a candidate
  with at least 2 observations (`box_*` parameters). Skipped points stay
  pending; if only skipped points remain, mission goes back. laberinto_simple:
  the view 0.12 m from the first box was skipped, `/done` at 32 s (before: 50 s
  pushing the box).

### For hans-die (navigation, localization)

1. **AMCL jumps in laberinto_1, always in the same place.** Left corridor near
   (-2.8, 0.2), during a turn in place, the pose moves ~0.3 m east, e.g.
   (-2.74, 0.20) -> (-2.48, 0.27) -> (-2.27, 0.49). After that the planner
   fails for every goal ("failed to generate a valid path") and the recoveries
   end in "Collision Ahead". 50-105 s lost until the end of the run. Seen in 2
   of 4 laberinto_1 runs: batch 20261026-205954-e0f64a4 run r03 (from t=195 s)
   and the laberinto_1 run of branch `mision/lejos-de-cajas` at a269a75 (from
   t=244 s). marcorita saw a 0.54 m AMCL error against ground truth in another
   run. Maybe related to the ~0.1 m map error you found in laberinto_1.
2. **Unreachable goal in maze_1.** Branch `mision/lejos-de-cajas` at a269a75,
   t=164-192 s: no path to the plan point (4.08, 0.78), and the behavior tree
   takes 28 s to give up (spin, wait, backup, spin). The next goals planned
   fine, so it is that point: probably an obstacle that is not in `/map` and
   not red, so mission cannot see it. Question: should Nav2 fail sooner when
   there is no path, or should mission check the global costmap before sending
   a goal? The second one is an interface change.

### For marcorita (coverage)

- Mission now skips plan points near candidate boxes (see #20 above). Your
  visibility predictions near a found box may be higher than what mission
  executes. The function is `box_zones` in `exploracion.py`.
- `first_pending` takes a new `zones` argument. `pruebas/metricas_cobertura.py`
  passes `[]` (agreed with you).
- Your INSPECTION_REPLAY case is covered by #18. Pocket replay: mission will not
  chase the 3-4 deg slot; that box is still reported by the final round.
- Pending: move your reachable-area function to `mapa.py`, so
  `viewpoint_for` can drop spots with no route before it gets more angles.

### For perception (Emiliano-Aguilera, SantosBunge)

- **False reports cost the most points we have seen.** maze_3, 2 of 2 runs in
  hans-die's NOTES: a report at (1.61, -1.45) with no red box near (the nearest
  is blue, 0.79 m away). It passed the report threshold (5 observations,
  <= 0.10 m), mission took it as the third victim and sent `/done`: 100 points
  plus the 200 bonus lost. There were also reports with no box near in maze_4.
- The competition is worse: N=5, and distractors are yellow, green and neutral;
  practice maps only have blue. Please check what is detected there, and test
  the red thresholds against yellow and orange.

### For Emiliano-Aguilera (simulation server)

- Could `simular.yml` take a start offset (`spawn_x`, `spawn_y`, `spawn_yaw`)?
  The simulator launch already accepts it, and in the competition the robot is
  placed by hand with no stated tolerance or yaw.
- Could the server run worlds with decoys (obstacles not in `/map`) or
  distractors of other colors? They cannot be in our repo (CI rejects `.world`),
  but they could live on the server.

### For everyone

1. **Organizer questions.** The 3 in `docs/team/ORGANIZER_QUESTIONS.md` are
   still not sent. Proposed additions: Does the competition `/map` include the
   boxes? Is the qualification also on a new layout? Can we use the physical
   robot before 2026-11-04?
2. **Overfitting to 6 maps.** All tuning uses the same practice maps. Proposal:
   keep maze_2_6x5 and maze_4_metal_6x6 for a final check only, without looking
   at them while tuning.
3. **Run every test before pushing.** A mission change broke the coverage suite
   because I only ran mission's tests. This runs all of `test/`, as CI does:
   `docker run --rm -v "$PWD":/repo -w /repo/src/equipo_jar equipo-jar python3 -m pytest test -q`
4. Minor: the `ExternalShutdownException` traceback at shutdown comes from
   `correr()` in `comun.py`, in every Python node. It does not change the score.
