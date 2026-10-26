# Coverage results

## 2026-10-26: guarded route restart and short-segment cleanup

Branch: `cobertura/recorrido-laberinto`, from development `3adfaeb` (PR #15).
Marco authorized addressing Hans's HIGH wall-turn stalls and slow difficult-map
route. He subsequently authorized publication and two recorded 300-s server
comparisons in laberinto_1, one per version. Actual dispatch/results remain to
be verified; no server improvement is claimed here. Merge requires green CI
and no pending comparison/review issues.
Marco approved the three coverage-only YAML entries: order_multiview,
order_baseline_m and order_block_candidates. Their defaults are true, 0.30 m
and 4 candidates per short segment; changes apply live.

The planner still selects exactly the same views. One alternative ordering
favors cells visible from fewer distinct positions and a second position at
least the configured baseline away. Repeated headings at the same position
do not count as another baseline. The alternative receives the existing
polish, then at most two deadline-rescue relocations. It is rejected unless
search duration, search-plus-return, sampled mean first-view time and all
predicted clock checks are no worse than the original polished route.
Two short-segment sweeps follow: exact changed-edge costs shortlist candidate
insertions before evaluating their full coverage profiles. Independent
full-route cost tests cover reversal, endpoint, start and asymmetric edges.
All accepted moves repeat the same guards; all views remain in the route.

Compared with the merged PR #15 baseline on six maps / three starts:

| Case | Before, including 8 s startup | Candidate | Saved |
| --- | --- | --- | --- |
| Difficult nominal | 485.06 s | 477.62 s | 7.44 s |
| Difficult offset 1 | 474.88 s | 474.88 s | 0 s |
| Difficult offset 2 | 517.89 s | 402.27 s | 115.62 s |

Nine of 18 routes shorten; nine are unchanged. Pose/heading multisets, map and
target counts, final dense/sample visibility and mission selection are exact.
The independent comparison checks every 60-s predicted clock-grid point, plus
the saved 60/120/300/480-s budgets: no dense or sampled visibility loss.
These are planar predictions, not camera-processing or victim-confirmation
evidence. Retaining all views also preserves final geometric baseline diversity.

Planning adds work. One warmup plus three alternating timed repetitions,
each requiring exact saved-plan parity:

| Nominal map | Old ordering median | Candidate median | Added calculation |
| --- | --- | --- | --- |
| Difficult | 4.900 s | 6.397 s | 1.497 s |
| Simple | 4.703 s | 5.307 s | 0.604 s |
| Maze 1 | 2.150 s | 2.472 s | 0.322 s |

Largest single planning sample in the integrated evaluation: 6.534 s.
These local timings are not a robot/track-station startup guarantee.
The simple redundant-view pruning and brute-force block prototypes were
retained but not adopted; the useful cleanup shortlists edges instead.

Wall checks use real local AMCL/Nav2 and the actual published Path. Windows
around all three reported problem locations completed 13/13 actions after an
8-s simulated warmup. The integrated first-zone repeat completed another 5/5
actions with all process-parent exit codes 0. Earlier cardinal-spin probes
used arbitrary headings and cannot judge the actual selected route. Their
ros2-run coverage wrappers needed TERM on cleanup; the final fixture starts
the installed executable directly. Coverage now accepts expected ROS context
shutdown after shared cleanup and still propagates unexpected errors.

Software-rendered RTF was roughly 0.29-0.39. Action success in these short
windows does not certify collision-free navigation, all initial poses or the
full route. The full-team local smoke completed 180 wall seconds, about 65.5
simulated seconds (RTF 0.367): 79 published goals, 18 successful Nav2 actions,
zero failed actions/protocol violations and all process parents exiting 0.
No candidate or report was recorded; score was 0. This is integration evidence,
not a score gain. The fixture prepares sensor TF before the judge and does not
certify production startup. The nominal route is still long; no fast-bonus or
score gain is claimed. Dynamic box-goal exclusion remains the separate mission
agreement with Bautista. No interface or mission/navigation code was changed.

Evidence under ignored build/verificacion/cobertura/: recorrido-completo/,
recorrido-comparacion/, recorrido-rendimiento/, recorrido-verificador.log,
confirmacion-reparada/, bloques-diversos/ and the three pared-plan* folders.
Reproduce offline comparison using pruebas/comparar_planes_cobertura.py.

## 2026-10-26: faster calculations, identical plans

Branch: `cobertura/calculo-eficiente`, from development `96134c0` (PR #14).
Marco asked for independent, measurable coverage improvements while waiting
for the mission agreement. No shared parameters, interfaces or other modules
were changed. No shared-server job was started.

The baseline profile recorded 6.7 million free-cell queries and 2.7 million
angular differences. Geometry now reuses one immutable free-cell mask for the
latest occupancy threshold, with a local snapshot and atomic cache replacement.
Inflation retains the original boundary/isclose behavior. Route trials reuse
exact angular distances within each plan. Map data stays immutable; a new map
gets a new Grid. Interrupted mask construction publishes no partial cache.

Paired elapsed planning time, one warmup and three alternating repetitions:

| Map | PR #14 median | Current median | Time reduction |
| --- | --- | --- | --- |
| Difficult labyrinth | 6.768 s | 5.493 s | 18.8% |
| Simple labyrinth | 6.831 s | 5.390 s | 21.1% |
| Maze 1 | 3.135 s | 2.418 s | 22.9% |

All runs assert identical poses, target counts and complete route metrics.
The integrated 18-case comparison exactly retains poses/order, counts,
predicted route times, dense/sample visibility, all budget snapshots and
mission selection. Largest single planning sample there: 5.834 s. These are
local elapsed timings, not a hardware guarantee or a reduction in driving time.

Verification:

- 103 pure cases, including independent disk clearance, alternating thresholds,
  parallel readers and cancellation/retry during mask construction.
- Differential comparison against PR #14: 20,250 generated rays and 1,575
  inflated masks match, including translated/rotated maps and boundary cases.
- Full `tools/verificar.sh --template`: 118 outer tests, including 120 internal
  coverage cases; static checks and runtime interfaces pass, no pre-map motion
  or reports. The new ROS test changes free_threshold live and observes a
  replacement plan reaching the newly free component.
- One local Maze 4 smoke, real AMCL/Nav2, 90 wall seconds: 60 published views,
  three successful navigation actions, zero failed actions/protocol violations,
  one report / 100 points, all process parents exit 0. Sensors and the coverage
  policy were ready before the judge started. It does not validate production
  startup. Collisions and RTF were not measured; no score gain is claimed.

Rejected route experiments: whole-position grouping saves at most 0.586 s in
1/18 cases. Pair grouping with existing time/mean guards saves at most 2.361 s
in 2/18 cases. Relaxing mean improves only five cases by small amounts. The
grouping code stays an offline experiment and does not change runtime routes.

Evidence under ignored `build/verificacion/cobertura/`: `calculo-referencia/`,
`calculo-rendimiento/timing.*`, `calculo-integrado/budget.*`,
`calculo-diferencial.log`, `calculo-verificador.log`,
`nav-calculo-20261026/candidate/`, `agrupacion-prototipos/` and
`agrupacion-pares/`. Environment-specific helpers are outside the repository
in `../auditoria-2026-10-26/`. Marco authorized publishing this verified
candidate and merging after green checks with no pending issues. GitHub holds
the actual PR/merge state.

## 2026-10-26: bounded route cleanup after the geometry correction

The current candidate adds one cleanup relocation sweep after the existing
sweep. It revisits choices that later moves can unlock, with the same sampled
clock, mean, search and search-plus-return guards. No poses/headings are removed
or changed. Cancellation is now checked inside the 2-opt search too.

All 18 integrated routes exactly match the saved prototype's permutations.
Seven searches shorten, eleven are unchanged; the largest predicted saving is
50.96 s (Maze 3, offset 2). Final dense/sample visibility and all view/count
sets are identical to the geometry-only candidate. Dense/sample visibility at
60/120/300/480 s never decreases relative to that candidate. This does not
restore the old geometry's lost views or prove detection on a real robot.
Dense mean first-view time regresses by about 0.00072 s in one case, reflecting
the sampled nature of the runtime guard.

The difficult offset-2 route falls from 524.7 to 517.9 estimated seconds, still
71.4 s longer than the old transit-only policy. Its nominal-start route is now
485.1 s; Maze 4 nominal is 451.7 s. The table in the geometry section below
records the earlier geometry-only stage, before this cleanup.

Largest single complete planning sample in the integrated 18 cases: 6.336 s.
Paired CPU medians (one warmup and three alternating repetitions) against the
geometry-only candidate:

| Map | One sweep | With cleanup |
| --- | --- | --- |
| Difficult labyrinth | 5.224 s | 5.651 s |
| Simple labyrinth | 5.444 s | 5.468 s |
| Maze 1 | 2.178 s | 2.397 s |

These are local CPU timings, not driving savings or a hardware guarantee.
The earlier real-navigation smoke below tested the geometry-only stage; the
new order has exact offline integration checks, not a new physical benchmark.
Pure suites pass 99 cases, including a regression task and cancellation during
2-opt. Final official verifier passes on the combined candidate: 118 outer
tests, including 115 internal coverage cases; zero static errors/hardcoding
warnings and runtime interface problems, with no pre-map motion/reports.

Additional evidence under ignored `build/verificacion/cobertura/`:
`espacio-orden-extra/experiment.json`, `espacio-orden-integrado/budget.*`,
`espacio-orden-integrado/comparison.json`, `espacio-orden-rendimiento/timing.*`
and the trusted `espacio-orden-referencia/` snapshot.

## 2026-10-26: observation clearance, local candidate

Branch: `cobertura/espacio-observacion`, based on development `71ca3f1`.
This is a geometry correction with measured visibility/time tradeoffs, not a
demonstrated score improvement. Marco authorized publishing the verified
candidate; GitHub holds the PR/merge state.

Transit still uses radius+margin (0.21 m). Coarse observation candidates prefer
0.30 m clearance. Fine recovery exhausts preferred centers before adding a
tighter witness; all observations require at least 0.24 m, never less than
transit clearance. That minimum encloses the current padded Nav2 rear corner
(about 0.2341 m) in the known map. Arrival tolerance, localization, wall errors
and unmapped boxes remain outside this nominal guarantee.

The initial strict-0.30 m candidate was rejected: it lost 318 dense cells and
56 samples in the difficult nominal case. The two-level policy reduces those
losses to 26 cells / 5 samples. All 18 cases preserve transit reachability and
mission selects every goal, with no sampled targets visible-but-deferred.
Every new goal meets the minimum. For example, the difficult nominal case
changes from 39 goals below the new minimum to zero.

The table includes the estimated 8 s startup. It predicts all goals succeed;
it does not measure driving or detection. Dense changes count floor-cell
centers, not victims.

| Nominal case | Goals before → after | Estimated route s | Final dense change |
| --- | --- | --- | --- |
| Difficult labyrinth | 65 → 77 | 490.1 → 488.5 | -26 |
| Simple labyrinth | 52 → 52 | 344.3 → 346.1 | -10 |
| Maze 1 | 50 → 51 | 395.1 → 416.0 | +6 |
| Maze 2 | 52 → 52 | 373.5 → 363.8 | -7 |
| Maze 3 | 60 → 61 | 443.5 → 463.4 | +11 |
| Maze 4 | 62 → 60 | 468.9 → 455.7 | -9 |

The difficult offset-2 case regresses from 446.5 to 524.7 s and loses 301 dense
cells at 300 s; final loss is 31 cells. This requires route/stops work before
claiming an efficiency improvement. The largest current planning sample is
5.748 s; this is one sample per case, not a paired CPU benchmark.

Verification: 16 generated geometry cases, 97 pure coverage/evaluation cases,
113 internal coverage cases including ROS, and 118 outer project tests in the
official verifier. Static checks and runtime interfaces pass; no motion or
reports before /map. Marco approved only the two new coverage YAML entries;
no other module parameters or interfaces changed.

Real AMCL/Nav2 local smoke, one 180-wall-second run per variant on Maze 4:

| Measurement | Previous policy | Candidate |
| --- | --- | --- |
| Published views | 62 | 60 |
| Successful navigation actions | 17 | 14 |
| Failed navigation actions | 0 | 0 |
| Reports / practice score | 1 / 100 | 1 / 100 |
| Protocol violations | 0 | 0 |
| Process parent exit codes | all 0 | all 0 |

Actions include mission's inspections, so action count is not coverage gain.
An action was still running at the imposed cutoff. Collisions and RTF were not
measured. These short runs do not prove fewer wall stalls, better score, full
coverage, the 300 s bonus or performance on unseen maps/physical hardware.

The first official-runner attempt published /map before team startup on WSL,
invalidating the pre-map A/B parameter setup. Preserve that failure. The paired
geometry fixture instead waits for fresh odometry/camera TF and the selected
policy before starting the judge, with real AMCL/Nav2. It does not validate or
change production startup; tools/ and shared launch files were not edited.
No shared-server jobs were launched.

Evidence in ignored `build/verificacion/cobertura/`: `espacio-observacion/`
(rejected), `espacio-preferido/budget.*`, `espacio-preferido/comparison.json`,
`espacio-verificador.log`, and `nav-espacio-20261026/` (both probes and the
`official-startup-failure/` evidence). Helpers for this local WSL environment
are outside the repository in `../auditoria-2026-10-26/`.

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
