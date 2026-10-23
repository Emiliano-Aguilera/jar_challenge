# Coverage results

## 2026-10-23: local observation-order refinement

Branch: `cobertura/orden-observacion`, from development `10f97ca` (merged PR #10).
Prepared for publication at Marco's request; GitHub holds the PR/merge state.
Every previously selected pose and orientation is
retained. One relocation sweep follows existing 2-opt, with sampled temporal
checks and nonincreasing estimated search, search-plus-return and sampled mean
first-view time. Marco approved the two coverage-only YAML parameters.

All 18 integrated cases match the isolated chosen prototype, retain final
dense/sample visibility, and mission selects every view. Dense/sample visibility
at 60/120/300/480 s never decreases in these tests. Search decreases in 17 cases;
one is unchanged. Median estimated saving is 11.43 s.

Nominal-start estimates below exclude startup and include the configured 1 s
observation cost per view. They are not physical navigation measurements.

| Map | Previous search | New search | Estimated saving |
| --- | --- | --- | --- |
| laberinto_1 | 540.52 s | 482.14 s | 58.38 s |
| laberinto_simple | 340.78 s | 336.29 s | 4.49 s |
| maze_1_6x5 | 415.84 s | 387.10 s | 28.75 s |
| maze_2_6x5 | 375.27 s | 365.50 s | 9.78 s |
| maze_3_6x6 | 437.75 s | 435.51 s | 2.24 s |
| maze_4_metal_6x6 | 471.18 s | 460.85 s | 10.33 s |

Paired CPU medians (warmup + three alternating repetitions) against `10f97ca`:
difficult map 5.586 -> 6.006 s; simple labyrinth 5.294 -> 5.507 s; first maze
2.225 -> 2.221 s. The last change is noise-sized. Runtime costs increased in
the larger maps; the largest single planning sample in the 18-case evaluation
was 6.395 s, below the configured 8 s startup estimate on this machine. This
does not guarantee a deadline on other hardware or under other workloads.

Rejected variants are recorded: mean-only ordering can regress early coverage;
strict full-curve dominance yields little travel reduction; unaligned periodic
checks can miss the intended wall-clock budgets. The selected sampled-clock
guard still does not guarantee dense visibility between samples/checks. Dense
mean first-view time increases slightly in one offset case, despite no loss at
the reported budgets. No physical navigation, detector or score improvement
has been demonstrated.

Evidence under ignored `build/verificacion/cobertura/`:
`orden-integrado/budget.*`, `orden-integrado/comparison.*`,
`orden-rendimiento/timing.*`, `orden-comparacion/`, `orden-intervalos/`,
`orden-fase/`. Final project and simulator gates are recorded in `STATUS.md`.

Final project gate passed: both packages build, 118 outer project checks
including the wrapper running 97 coverage/evaluation cases, zero static errors
or hardcoding warnings, four team nodes with zero runtime audit problems.
The repeated simulator handoff passed with 50 views and the first three mission
goals checked in order; all six parents and team children closed cleanly.
That fixture waited for TF readiness before starting the judge. The first
attempt failed by inactivity after a roughly 54 s simulator TF delay and is
retained. No physical travel/detection or competition startup success is claimed.

Gate logs: `orden-verificador.log`, `integracion-orden-20261023/` (failed first
attempt), `integracion-orden-listo-20261023/` (passing communication fixture).

## 2026-10-23: local budget evaluation and computation optimization

Branch: `cobertura/evaluacion-presupuesto`; integrated development `d375427`.
Reference measurements use `a8b9530`. See `STATUS.md` for the publication gate.

- Evaluated six maps at three starts with intended and mission-selected routes.
  All 18 cases passed. Dense visibility, raw-connected denominators and budgets
  at 60/120/300/480 s are saved in JSON/CSV/Markdown.
- Before mission PR #9, the selector skipped useful views in four cases. Dense visibility
  losses are 65, 40 and 55 cells in the three difficult-map starts, and 8 cells in
  the first offset of maze_3. PR #9 resolves this: all 18 saved plans now select
  every intended point in order. Details: `MISSION_COMPATIBILITY.md`.
- In the six nominal cases, an exhaustive safe-center/heading search found no
  missed positional witnesses beyond those already considered by the planner.
  Difficult-map residuals: 182 raw-disconnected samples; another 205 have no
  reachable witness, of which 78 are visible from another safe component and 127
  have no witness among any safe centers under the present model. The simple
  labyrinth has 79 raw-disconnected samples and 32 with no safe-center witness.
  The remaining four maps' missing sampled targets are raw-disconnected.
- Prepared camera/heading terms reduce repeated work during fine recovery.
  All 18 before/after comparisons retain exactly the same poses, mission-selected
  goals, dense/sample visibility, travel and turning. Startup-normalized event
  times match too. The check is `presupuesto-paridad.json`.
- Full official verifier passed after integrating development `d375427`:
  both packages build, 118 outer colcon entries pass (including 77 internal coverage/evaluator cases),
  static checks have zero errors/hardcoding warnings and the startup audit finds
  four team nodes with zero problems. All team parameters are unchanged.

Paired CPU comparison, excluding diagnostics, with one warmup and three
alternating repetitions per variant/map:

| Map | Reference median | Current median | Time reduction |
| --- | --- | --- | --- |
| laberinto_1 | 7.515 s | 5.606 s | 25.4% |
| laberinto_simple | 6.003 s | 5.003 s | 16.7% |
| maze_1_6x5 | 1.936 s | 1.886 s | 2.6% |

The small maze difference should not be treated as a reliable speed gain. These
are measurements on this machine, not hardware-independent or statistical
guarantees. All runs assert identical poses, target counts and route metrics.
No new physical navigation or victim detection was measured.
The simulator handoff was repeated after integrating PR #9: 50 viewpoints,
first three mission goals match the published Path in order, and all five
process parents exit 0. Navigation was fake; map-to-odom used the test identity.

Current artifacts under ignored `build/verificacion/cobertura/`:

- `presupuesto-base/budget.json` and `.md`;
- `presupuesto-optimizado/budget.json`, `.csv`, `.md`, `profile.txt`;
- `presupuesto-paridad.json`;
- `rendimiento/timing.json` and `.md`;
- `referencia/cobertura_plan.py`: saved trusted reference from development.
- `presupuesto-verificador.log`: complete build/tests/startup verification.
- `publicacion-verificador.log`: repeated complete gate after integrating PR #9.
- `integracion-presupuesto-20261023/`: repeated simulator handoff and process logs.

## 2026-10-22: first integration baseline

Branch: `cobertura/base-integrable`, based on development `c01ccdc`. Prepared for
publication at Marco's request. This includes the planner and ROS adapter; it is
independent of the shared-map change in PR #6.

- The full official local verifier (`--template`) passed: both packages build,
  113 colcon entries pass, static checks have zero errors/hardcoding warnings,
  and the runtime map-waiting audit finds four team nodes and zero problems.
- Colcon's coverage entry runs the 54 geometry/ROS adapter checks in a subprocess.
  Its outer count is one, so the 113 count is not the inner coverage case count.
- Parsed the merged YAML and checked that all non-coverage node settings are
  identical to development. Only coverage's 17 parameters were added.
- A local simulator probe used the official practice map and real simulated
  camera TF, `pruebas.launch.py`'s identity map-to-odom, and Bautista's fake Nav2
  action server. Coverage published 50 oriented goals. Mission selected three
  distinct points verified against that published Path and sent them to navigation.
- Capture, team launch, fake navigation, judge and simulator parents exited 0;
  every team launch child also logged clean shutdown.
- This proves plan generation and message/action handoff. It does not prove
  physical navigation, AMCL accuracy, processed observations or tournament score.
- Current outputs are ignored under `build/verificacion/`: the full
  `cobertura-verificador.log` and `cobertura/integracion-20261022/` with plan,
  plot, probe result and process logs.
- Verification used ROS Humble, system NumPy 1.21.5, Python user packages disabled
  and the repo's pinned OpenCV installed into an isolated temporary directory.
  No shared dependency versions were changed.

## 2026-10-17: planner evaluation

The measurements below were taken on the original local coverage branch before
the current team integration. They remain offline planner evidence, not results
of the complete autonomous stack.

## What changed

- Diagnose covered, deferred and outside-candidate target samples.
- Measure grid distance, rotation, estimated search time, return distance
  and mean first-view time. Produce full-grid visibility and route plots.
- Recover views on a finer position grid. Retain every initial observation.
- Improve the visit order with one bounded 2-opt sweep that preserves the
  view set and improves both time measures for that set.
- Reject results after pose corrections; cancel old work on map/config
  changes and shutdown. Coverage still publishes only its agreed Path.

## Evidence

- 54 coverage checks pass through `colcon test`, plus 3 existing report checks.
  Colcon shows the coverage suite as one wrapper check, so its outer count is 4.
- `bash tools/verificar.sh --template`: all checks passed; zero audit problems
  and zero static hardcoding warnings.
- Six practice maps, three starts per map: 18 accepted comparisons.
  The improved plan retains all baseline poses in every case. Full-grid
  visibility was checked at the default start on each map and never decreased.
- Final local simulator probe: 50 oriented goals, real simulated camera TF,
  map/plan transport works, all launch children shut down cleanly.

Default-start comparison (same parameters and camera model):

| Map | Sampled visibility before → after | Estimated search seconds before → after |
| --- | --- | --- |
| laberinto_1 | 1099 → 1112 / 1499 | 522.3 → 540.5 |
| laberinto_simple | 1474 → 1474 / 1585 | 345.3 → 340.8 |
| maze_1_6x5 | 1345 → 1345 / 1383 | 430.0 → 415.8 |
| maze_2_6x5 | 1343 → 1343 / 1376 | 383.4 → 375.3 |
| maze_3_6x6 | 1602 → 1602 / 1673 | 452.1 → 437.8 |
| maze_4_metal_6x6 | 1601 → 1601 / 1675 | 491.6 → 471.2 |

The difficult map gains coverage at a travel cost: 74 additional full-grid
cells at the default start. The report deliberately keeps this tradeoff visible.
The first maze's 38 pending sampled cells are disconnected in the raw map.
Other remaining gaps need separate analysis; they are not all called impossible.

Computation took about 2–8.1 seconds per improved case on this local WSL setup,
including diagnostic path reconstruction. One run per variant and start,
alternating variant order; these are not statistical runtime guarantees.

## Limits that matter

- Visibility is a horizontal geometric prediction. It does not prove a box
  was detected or an image was processed. Range and dwell need calibration.
- Grid paths and time estimates omit acceleration, localization delays,
  unexpected obstacles and detector workload. They are not Nav2 trajectories.
- One complete search estimate is 572.8 s plus 8.4 m to return. At 0.25 m/s,
  search plus return exceeds 600 s. Current mission enforces return at 480 s and
  finish at 570 s; these long plans need evaluation within that search allowance.
- Lowering the safety margin in an offline diagnostic did not solve the
  difficult map's gaps. Production settings remain unchanged.
- The simulator probe used the existing test identity map-to-odom transform.
  Real localization, navigation, mission progress and perception are still
  separate integration work. See INTEGRATION.md for the required agreements.

## Reproduce and inspect

Commands: `pruebas/README.md`.

Runtime outputs (ignored by Git):

- `build/verificacion/cobertura/evaluacion-final/comparison.md`, `.csv`, `.json`;
- one diagnostic PNG per map in that directory;
- `clearance-sensitivity.json`: offline diagnostic, no settings applied;
- `build/verificacion/cobertura/final-verifier.log`;
- `build/verificacion/cobertura/smoke-final/`: final capture and process logs.
