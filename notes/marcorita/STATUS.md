# Coverage work status

## Current task: shared reachability (2026-10-27)

- Branch `cobertura/mapa-alcanzable`, based on development `d7e8057` (PRs
  #19-21 merged). The first_pending compatibility fix is already merged in #20.
- Reused the clean map checkout. The PR #16 checkout and its uncommitted notes
  remain untouched; PR #16 remains a separate draft with behaviour pending.
- Shared the existing conservative Grid/Pose/connectivity code through mapa.py,
  with backward-compatible coverage imports and a ROS-free OccupancyGrid adapter.
  New `reachable_from` filters a radius-safe connected component; no snapping
  of an unsafe robot start, no diagonal corner cuts, unknown cells blocked.
- Focused geometry/planning: 88 tests passed. Full verifier passes, including
  137 outer tests, build/static checks and map-wait/interface audit (zero issues).
- Differential check against development d7e8057: all 18 generated plans
  match exactly, including diagnostics and metrics; all 18 safe components
  also match. Recorded pocket map: exterior ring points at 174/175 degrees
  remain connected; the interior point at 285 degrees is rejected. This is
  static geometry with rounded log inputs, not a new inspection run.
  Evidence: ignored `build/verificacion/cobertura/mapa-compartido/`.
- API, integration steps and model limits: `MAP_REACHABILITY.md`.
  Mission still needs to adopt the helper. Nonempty candidate-box exclusion
  zones and actual observation execution remain separate validation work.
- Marco authorized commit/push/PR on 2026-10-27; merge only after all checks pass
  and no pending issues, as previously requested. No new manual server run.
  GitHub holds the publication status. Previous milestones below are historical.

## Previous milestone: calculation efficiency

Updated: 2026-10-26. Branch: `cobertura/calculo-eficiente`.
Base: development `96134c0` (merged PR #14).

## Current task

Marco asked for measurable improvements within coverage while waiting for
Bautista. Keep mission, navigation and shared interfaces unchanged.

- Baseline: the 18 final PR #14 routes in espacio-orden-integrado/budget.json.
- Difficult cases revisit 10-11 positions for different headings; test whether
  grouping those headings reduces travel while keeping every original view.
- Grouping prototypes rejected: whole-position grouping saves at most 0.586 s;
  pair grouping with existing guards saves at most 2.361 s, in only 2/18 cases.
- The current change accelerates geometry using a one-threshold free-cell mask
  and reuses exact angular costs during route trials. Same formulas and outputs.
- All 18 integrated cases exactly retain poses/order, counts, route estimates,
  dense/sample visibility, budget snapshots and mission selection.
- Paired CPU medians: difficult 6.768->5.493 s, simple 6.831->5.390 s,
  first maze 3.135->2.418 s (about 19-23% less elapsed calculation time).
  One warmup and three alternating repetitions; local hardware only.
- Pure suites: 103 passed. Differential geometry matches 20,250 rays and 1,575
  inflated masks. Full verifier passes 118 outer tests including 120 internal
  coverage cases, with a live ROS threshold-change check and clean interfaces.
- Local 90 s AMCL/Nav2 smoke passes: one report / 100 points, three successful
  nav actions, zero failed actions/protocol violations, all parent exits 0.
  Prepared sensor TF; production startup, collisions and score gains unproved.
- No new shared parameters, interfaces or server jobs. Marco authorized
  commit/push/PR and merge after successful checks without pending issues.
  GitHub holds the publication/merge state.
- Prototypes: build/verificacion/cobertura/agrupacion-prototipos/.
  Partial grouping: agrupacion-pares/. Current performance/route evidence:
  calculo-rendimiento/, calculo-integrado/ and calculo-referencia/.

## Previous milestone: observation clearance and cleanup (merged PR #14)

Fix observation stand-off without applying the same larger clearance to transit.
Marco authorized starting with geometry, then route/stops. Source changes are
verified. Marco authorized commit, push and a PR to the team's development,
with merge after successful checks and no pending issues. GitHub holds the
publication/merge state; this file records the tested candidate.

The final local candidate also adds one bounded cleanup relocation sweep.
All 18 routes exactly match the experiment, keep the geometry candidate's
entire view set/final visibility, and lose no dense/sample coverage at the
reported budgets relative to it. Seven routes shorten (maximum 50.96 s).
The difficult offset-2 estimate is now 517.9 s, still 71.4 s above the old
transit-only policy. Largest planning sample: 6.336 s. Pure suites: 99 passed;
final full verifier: 118 outer tests, including 115 internal coverage cases,
all passed. See RESULTS.md for paired CPU cost and limits.

- Added minimum observation clearance 0.24 m and preferred clearance 0.30 m.
  Preferred centers are exhausted before fine recovery uses a tighter witness.
  Effective stand-off is never smaller than transit radius+margin. Marco
  authorized only these two additions under coverage in shared params.yaml.
- Candidate sampling, fine recovery and optional start observations obey it.
  Travel/connectivity still use the previous transit approximation.
- Generated two-room/corridor test proves distant safe observation positions
  remain reachable through transit cells that are unsuitable for stopping.
- Empty observation sets emit no unsafe goals. Translated/rotated maps and
  cancellation are covered. Sixteen geometry cases plus existing pure suites:
  97 passed. Full verifier: 118 outer tests, including 113 internal coverage
  cases; no static errors/hardcoding warnings or runtime interface problems.
- Rejected strict-0.30 m experiment: difficult nominal case lost 318 dense
  cells and 56 sampled targets. The two-level policy reduces these losses to
  26 dense cells and 5 targets. All 18 transit counts and mission selections
  remain unchanged; other maps have small dense visibility gains/losses.
- Safety changes the viewpoints and has costs: difficult offset 2 grows from
  446.5 to 524.7 estimated seconds and loses 301 dense cells at 300 s, although
  final loss is 31 cells. It is not a universal time/visibility improvement.
- Actual AMCL/Nav2 smoke finished locally: one 180-wall-second Maze 4 run per
  policy, before the cleanup sweep. Both report one victim / 100 points, with zero protocol violations
  and zero failed navigation actions. All process parents exit 0. The candidate
  completes 14 actions versus 17; this does not prove better coverage or score.
  Collisions and RTF were not measured. The official local runner
  exposed /map before the team started, invalidating pre-map policy setup.
  The geometry fixture waits for robot sensor TF and parameters before starting
  the judge. This isolates geometry and does not validate production startup.
- Evidence: build/verificacion/cobertura/espacio-observacion/ (rejected strict
  policy), espacio-preferido/ (current), espacio-verificador.log and
  nav-espacio-20261026/. Next: address route/stops and validate the changed wall
  goals in a targeted/full run before claiming fewer stalls. Candidate-box
  exclusion still needs agreement with Bautista. No server job was launched.

## Previous milestone: observation order (merged PR #11)

The sections below record the earlier ordering advance from development `10f97ca`.

Improve early predicted visibility and estimated route cost. Marco authorized
local work on selection and ordering, with comparisons and updated notes.
The previous advance is merged as PR #10; its evidence is retained below.

### Implemented locally

- Created a fresh coverage task branch from current development.
- First isolate ordering while keeping every selected pose/orientation.
- Compared nine ordering policies, including mean-only, complete curve dominance,
  fixed budgets and periodic checks with/without startup alignment.
- Mean-only and unaligned policies can reduce early dense visibility. Full-curve
  dominance yields little travel reduction. Chose a bounded relocation sweep
  preserving sampled coverage on periodic predicted-clock checks.
- Runtime keeps every baseline view, requires nonincreasing estimated search,
  search-plus-return and sampled mean first-view time, and checks cancellation.
- Marco explicitly authorized the two new coverage-only YAML parameters:
  `order_check_interval_s: 60.0`, `order_start_delay_s: 8.0`. They apply live
  through the existing adapter. The delay is an estimate, not mission's clock.
- Added ordering checks and included them in the existing colcon wrapper.

### Current verification

- All 18 integrated cases match the chosen isolated prototype exactly.
- Every pose/orientation, target count and final dense/sample visibility is
  retained. Mission selects every point. Search and search-plus-return estimates
  and sampled mean first-view time never increase.
- Dense and sampled visibility at 60/120/300/480 s never decrease in these cases.
- Median estimated search saving: 11.43 s. Difficult nominal case:
  540.52 -> 482.14 s, excluding startup. A different case is unchanged.
- Largest single measured complete planning time: 6.395 s. Paired CPU medians
  are 5.586 -> 6.006 s (difficult map), 5.294 -> 5.507 s (simple labyrinth),
  and 2.225 -> 2.221 s (first maze; noise-sized difference). One warmup and three
  alternating repetitions per variant/map; no hardware-independent guarantee.
- Full official verifier passed after final wiring: both packages build;
  118 outer project tests pass, including the wrapper running 97 internal
  coverage/evaluation cases. Static checks have zero errors/hardcoding warnings;
  the runtime audit has four team nodes and zero problems. No premotion/reports.
- Confirmed only the two approved coverage parameters differ from development.
  Invalid values are rejected and the ROS test checks both parameters apply live.
- Repeated simulator handoff passed after TF readiness: 50 published views,
  first three mission goals match the Path in order, all six process parents
  exit 0 and all team launch children close cleanly. Navigation was fake and
  map-to-odom used the test identity. Physical navigation/perception are untested.
- First handoff attempt failed: the simulator exposed the map before connected
  robot TF, which arrived about 54 s later; the stationary fake-navigation robot
  reached the judge's 60 s inactivity limit. Logs remain in
  `integracion-orden-20261023/`. The repeated communication fixture waits for
  fresh map/base/camera TF before starting the judge; this does not fix or
  validate production startup/localization.
- No physical navigation/detection measurements. Dense mean first-view time has
  a small regression in one case despite the sampled guard; this is an explicit
  sampling limit, not a claim of improvement at every time or on unseen maps.

Evidence under `build/verificacion/cobertura/`: `orden-comparacion/`,
`orden-intervalos/`, `orden-fase/`, `orden-integrado/budget.*` and
`orden-integrado/comparison.*`. The comparison excludes startup from search
duration and uses every free cell center for dense visibility. Early experiment
files' fraction denominator was the union of planned views; their absolute cell
counts remain valid. The final integrated report uses all free cells.
Full project gate: `orden-verificador.log`. Paired CPU evidence:
`orden-rendimiento/timing.json` and `.md`.
Passing simulator evidence: `integracion-orden-listo-20261023/probe.json` and logs.

### Next iteration

This local ordering advance is complete. Next, compare alternative view
selection/orientations as a separate measured change; retain the current dense
checks and do not prune poses using sampled visibility alone. Calibrate actual
view usefulness, arrival accuracy and startup delays with navigation/perception.

Marco authorized publishing this advance and merging once all checks pass,
with no conflicts or requested changes. This is the verified publication
candidate; GitHub holds the current PR/merge state.

## Previous milestone: evaluation and computation optimization (PR #10)

The remaining sections record the verified publication candidate for PR #10,
based on development `d375427`. That PR is now merged at `10f97ca`.
Its reference measurements used the earlier `a8b9530` planner.

### Completed

- Created the task branch from current development in the clean coverage worktree.
- Preserved the original local checkout and the merged delivery branches.
- Reviewed the current planner, evaluator, mission selector and existing evidence.
- Added budget/visibility accounting, the actual mission selector, dense target
  masks and exhaustive missing-target witness search. The 21 generated accounting
  checks pass; the existing coverage wrapper now includes them.
- Added the budget evaluation CLI with JSON/Markdown output, explicit model limits,
  separate runtime measurements and optional profiling.
- Difficult-map pilot before mission PR #9 passed. Mission selected 59/65 goals and lost 65 dense
  predicted cells. The exhaustive oracle classifies all 205 raw-connected missing
  samples as no witness under the current reachable-center/camera model.
- Profiling identifies repeated single-target visibility preparation during fine
  recovery as the dominant cost. Prepared camera/heading terms now avoid repeating
  those transforms per target; the original arithmetic order is preserved.
- The 61 pure-planner/evaluator tests pass, including 600 parity comparisons and
  field-of-view/range boundaries for the optimized witness check.
- The 18-case reference evaluation finished before PR #9. Useful goals were skipped in four
  cases: three difficult-map starts and one maze_3 offset. Other cases lose no
  dense predicted cells to the mission selector.

### Verification completed

- All 18 before/after cases have identical poses, mission-selected goals,
  dense/sample visibility, distance, turning and startup-normalized event times.
- Paired measurements (warmup + three alternating repetitions) give medians:
  difficult map 7.515 -> 5.606 s; simple labyrinth 6.003 -> 5.003 s; first maze
  1.936 -> 1.886 s. The small maze difference is within the range where runtime
  noise matters. No hardware-independent or statistical guarantee is claimed.
- Full official verifier passed after integrating development `d375427`:
  two packages built; 118 outer colcon checks,
  including the wrapper running 77 internal coverage/evaluation cases; zero
  static errors or hardcoding warnings; four team nodes and zero runtime audit
  problems. All team parameters match development.
- No new full navigation or victim-detection run was performed. The production
  change preserves the generated routes and all tested visibility semantics.
- Simulator handoff repeated on the combined revision: 50 published viewpoints,
  first three mission goals match the Path in order, and capture/team/fake
  navigation/judge/simulator parents exit 0. Navigation was fake and localization
  used the existing test identity transform; no physical travel is demonstrated.

### Current local state

Evaluation and computation optimization are implemented and verified locally.
Development `d375427` was incorporated with a fast-forward, preserving all local
coverage changes. The contribution changes only coverage code/tests, evaluation
scripts and marcorita's notes. It adds no changes to mission, shared parameters,
interfaces or navigation relative to that development revision.

### Findings requiring coordination

Resolved in reviewed development `d375427` (mission PR #9): identity tolerances
are now 0.02 m / 0.1 rad. Replaying all 18 saved routes selects every intended
view in order; all 22 merged selector tests pass. Mission compares requested
poses, so this does not require navigation to arrive within 2 cm. See
`MISSION_COMPATIBILITY.md` for evidence and remaining model limits. No mission
file or shared parameter was edited locally. The correction was first tested in
an exported snapshot, then integrated and checked through the full verifier.

Pilot evidence: `build/verificacion/cobertura/presupuesto-piloto/budget.md`,
`budget.json` and `profile.txt`. Profile timings include instrumentation overhead;
they are not the planner's uninstrumented latency.

Final evidence: `build/verificacion/cobertura/presupuesto-optimizado/budget.md`
and `.json`/`.csv`; `presupuesto-paridad.json`; `rendimiento/timing.md`/`.json`;
`presupuesto-verificador.log`. `RESULTS.md` summarizes measurements and limitations.
Publication gate: `publicacion-verificador.log` against development `d375427`.
Simulator handoff: `integracion-presupuesto-20261023/probe.json` and process logs.

### Next decisions

- Choose an algorithm change only after identifying its measured failure case.
- Compare early visibility, final visibility, travel/turning and compute cost.
- Keep movement, reports and deadlines with their current owners. Interface
  proposals need agreement before implementation.
- Mission goal identity is verified for all 18 saved practice plans after PR #9.
  Recheck tolerances if map resolution or heading spacing changes. Current
  development is integrated; physical navigation and perception still need an
  end-to-end run.
- Validate reachable footprint/components with Hans: 78 difficult-map samples
  have witnesses only in another safe component. Another 127 have no safe-center
  witness under this camera/range/heading model. Do not lower clearance blindly.
- Calibrate range and observation evidence with perception before optimizing
  predicted floor visibility as if it were confirmed detection.

### Evidence and limits

The merged baseline passed 54 internal coverage cases, official CI and a local
simulator handoff with fake navigation. The local follow-up has 77 internal cases
and the verification above. Neither validates physical travel, AMCL or actual
victim detection. A safe-center witness search is limited to its explicit model.
