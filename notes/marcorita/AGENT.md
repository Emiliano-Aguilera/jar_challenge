# Context for marcorita's agent

Module: cobertura. Follow root AGENTS.md, CLAUDE.md, docs/team/FLOW.md and
INTERFACES.md. Read NOTES.md for decisions and STATUS.md for current state.

## Current task: protected ordering, PR #16 held in draft

- Branch cobertura/recorrido-laberinto, from development 3adfaeb (PR #15).
  Latest fetch still has that development commit. Marco authorized coverage
  improvements, publication to the group's fork, and exactly two recorded
  300-s server runs. Both runs have been consumed; ask before another run.
  Merge only after green CI and no pending behavioral/review issues.
- Published first candidate 755c891 has four green CI checks, but PR #16 is
  DRAFT. It must not merge: baseline 79ec03c scored 100 (RTF .973, GPU 0),
  candidate 755c891 scored 0 (RTF .748, GPU 1). Same image, 300 s, no protocol
  violations. Unequal RTF/hardware and changed early order confound causality.
- Tailscale is installed (authorized official 1.104.2), logged in and connected.
  Full logs and bounded recording ranges were read successfully. Do not ask
  Marco to connect again. Never download multi-GB bags in full.
- The old global restart retained counts but delayed specific map regions:
  nominal at 60 s misses 373 baseline dense cells while adding 388 others.
  Discard the old 115.62-s headline as an accepted improvement. Old evidence
  is frozen in recorrido-completo/ and recorrido-rendimiento/.
- Correction: restart only inside original predicted-clock buckets; guard
  every baseline view occurrence at each configured clock cut, including
  zero sampled-gain views and repeated equal poses. Two bounded short-segment
  sweeps repeat those guards. Preserve all selected views and geometry.
  Existing merged polish stays unchanged. No mission/Nav2/interface edits.
- Corrected 18-case comparison: 5 shorter / 13 unchanged, maximum 11.034 s.
  Difficult offset 2 saves 10.584 s; nominal unchanged. Independent Counter
  checks preserve every completed view at each 60-s cut on intended and
  actual mission-selected sequences. Full verifier passes 118 outer tests,
  static checks, interfaces and map-wait audit.
- Paired CPU medians: difficult 5.687->5.686 s; simple 5.441->5.577 s;
  maze 1 2.468->2.473 s. One warmup, three alternating timed repetitions,
  exact saved-plan parity. Largest single integrated planning sample 5.732 s.
  These are local timings, not a robot startup guarantee.
- Evidence: recorrido-vistas-protegidas/, vistas-protegidas-comparacion/,
  vistas-protegidas-rendimiento/ and vistas-protegidas-verificador.log under
  ignored build/verificacion/cobertura/. Local 300-wall-second smoke reaches
  only 82.2 simulated seconds (RTF .273), with 79 views, 12 successful and two
  failed navigation actions, no candidates/reports/protocol violations. Score
  0; behavioral validation is not passed. See LOCAL_NAVIGATION.md. Mission
  and the timeline recorder also throw RuntimeError during SIGINT cleanup;
  do not describe all children as clean. No corrected server run yet.
- Marco approved only three new coverage YAML entries: order_multiview=true,
  order_baseline_m=0.30, order_block_candidates=4, applied live. No other
  shared parameter, source, message or launch permission is implied.
- Recorded candidate had 3 observations / sigma .15 m; reporting needs 5 /
  sigma <= .10 m. Latest delivered update had 2 at inspection attempt.
  Mission logged "Sin lugar para mirarla", appended the center to inspected
  points and continued. No retry/report followed. Replay finds no viewpoint
  at .8/1.2/1.6/2.0 m with 12 or 72 angles, nor a saved-plan or exhaustive
  reachable .25-clear grid alternative under current planar rays. Do not
  sell a larger radius as a fix, lower confidence or call the center occupied.
  See INSPECTION_REPLAY.md for Bautista's handoff. No mission change made.
- A recorded startup AMCL sample plus native map/static TF reproduces all 79
  baseline poses and order exactly. Corrected order keeps the first 37 views
  and the critical first-detection transition, saves 6.810 predicted seconds,
  and loses no view at any 60-s clock cut. This is not camera execution.
  Assuming exactly zero pose previously failed to reproduce the plan.
  Evidence: replay-plan-nativo.json. Practice truth stays out of src.
- Earlier actual-Path wall windows: 13/13 plus first-zone 5/5 successes, all
  parents exit 0. Short windows do not prove collisions, full mission, score
  or production startup. Coverage accepts expected ROS shutdown after shared
  cleanup; other errors still propagate.

### Previous milestone: calculation efficiency (merged PR #15)

- Previous task: `cobertura/calculo-eficiente`, from development `96134c0`
  (merged PR #14). Marco asked for measurable coverage improvements while
  waiting for Bautista. Work locally in coverage; do not edit mission/Nav2 or
  add shared interfaces. No shared-server run is authorized for this task.
- Tested consolidating separated views at the same cell into one visit, retaining
  every pose/heading. Baseline `espacio-orden-integrado/budget.json` has 10-11
  repeated visits in the difficult cases. `pruebas/experimentar_agrupacion.py`
  compares predicted-time, mean and strict sampled-curve guards. Whole grouping
  saves at most 0.586 s; guarded pair grouping at most 2.361 s in only 2/18
  cases. Rejected as insufficient benefit; no grouping added to runtime.
- Current runtime change reuses immutable free-cell masks (latest threshold
  only) in ray casting/inflation, and exact angular costs within a plan. Mask
  replacement uses an atomic tuple and local snapshots; cancellation is checked
  during construction. Edge checks retain the original isclose behavior.
- Paired CPU medians: 6.768->5.493 s, 6.831->5.390 s, 3.135->2.418 s.
  All 18 cases retain poses, order, counts, visibility, route estimates and
  mission selection exactly. Pure suites pass 103 cases; final verifier passes
  118 outer tests including 120 internal coverage cases. Differential geometry
  matches 20,250 rays / 1,575 masks; local 90 s Nav2 smoke passes with clean
  parent exits and no protocol violations. No route-algorithm/parameter or
  interface change. Do not prune views by sampled gain.
- Current prototype output: ignored build/verificacion/cobertura/agrupacion-prototipos/.
  Publication and shared-file/server permissions must follow the existing rules.
  Marco explicitly authorized publishing this verified calculation optimization
  and merging after green checks with no pending issues. GitHub holds its state.
  Performance evidence: calculo-rendimiento/, calculo-integrado/ and the trusted
  calculo-referencia/ snapshot under ignored build/verificacion/cobertura/.

### Previous milestone: observation clearance (merged PR #14)

- Current task (2026-10-26): `cobertura/espacio-observacion`, from development
  `71ca3f1` (navigation PR #13 and mission PR #12). Marco authorized correcting
  the observation geometry first, then route/stops. Keep this change focused.
- Current combined candidate adds a second bounded relocation sweep, preserving
  the geometry candidate's complete view set. All 18 routes match the prototype;
  7 shorten, max 50.96 s, no dense/sample losses at reported budgets relative to
  geometry-only. Largest planning sample 6.336 s. Pure suites: 99 cases pass;
  final verifier passes 118 outer tests, including 115 internal coverage cases.
  Paired CPU medians and the remaining 71.4 s
  difficult offset-2 regression versus old geometry are in RESULTS.md.
- Separate transit radius+margin from `view_clearance` for observation centers.
  Use 0.24 m minimum and prefer 0.30 m; fine recovery exhausts preferred
  centers before adding a tighter witness. The first strict-0.30 m experiment
  lost 318 dense cells on the difficult nominal case and was rejected.
  Coarse candidates, fine recovery and forced start views must obey it. This
  intentionally changes viewpoints; measure visibility loss and connectivity,
  not exact route parity. The existing transit approximation is retained.
- Sixteen geometry cases plus existing pure suites pass (97 pure cases).
  The full verifier passes 118 outer tests (113 internal coverage cases).
  Both 18-case evaluations finished. All transit counts and mission selection
  are preserved; visibility/time tradeoffs remain, especially difficult-map
  offset 2. Marco explicitly approved the two coverage-only YAML entries on
  2026-10-26 and they are added. Marco authorized commit/push/PR for the combined
  candidate, and merge after green checks with no pending issues. GitHub holds
  the actual publication state. Future unrelated publication/server jobs still
  need authorization.
- Real AMCL/Nav2 smoke finished: one 180-wall-second Maze 4 run per policy,
  both 100 points / one report, no protocol violations or failed nav actions;
  all parent processes exit 0. Candidate completes 14 actions vs 17. Neither
  collisions nor RTF measured; no score/efficiency improvement proved. Read
  RESULTS.md. The original official
  runner published the map before the team started on this WSL; preserve the
  failed attempt. The geometry fixture starts the judge after sensor TF and
  policy readiness, so it does not certify production startup. Do not dispatch
  shared-server jobs without authorization.

### Previous observation-order milestone (merged PR #11)

- Current work: `cobertura/orden-observacion` from development `10f97ca`.
  The previous evaluation advance was merged as PR #10; the worktree is updated.
  Marco authorized improving viewpoint selection/ordering locally and keeping
  comparisons and notes. Read `STATUS.md` for the current experiment.
- Fixed-view relocation is now wired after the existing 2-opt sweep. Every view
  is retained, so final dense visibility is preserved. Check the predicted
  clock grid, sampled mean first-view time, search and search-plus-return costs.
  `pruebas/experimentar_orden.py` records rejected variants using saved routes.
  Runtime outputs stay in build/. The 18 integrated cases match the selected
  prototype and have no visibility loss at the reported budgets.
- Marco explicitly approved adding `order_check_interval_s` and
  `order_start_delay_s` only under coverage in shared params.yaml. Defaults are
  60.0 and 8.0; no other module settings changed. These are forecast settings,
  not commands to mission. Paired timing and the full verifier pass; largest
  single planning sample is 6.395 s. Project tests: 118 outer entries including
  the wrapper running 97 coverage/evaluation cases. Fake-navigation simulator
  handoff passes after TF readiness: 50 views, three ordered goals, clean exits.
  The failed first probe exposed delayed simulator TF and is retained. No
  physical travel or detection was measured. Marco authorized publishing this
  advance and merging after all checks pass with no conflicts/change requests.
- Budget diagnostics and the prepared camera/heading optimization are verified
  locally. The 18-case comparison preserves routes exactly; paired runtimes and
  the complete verifier pass. Read `MISSION_COMPATIBILITY.md` before changing
  point identity: PR #9 in development `d375427` fixes all four diagnosed cases.
  Replaying all 18 saved routes selects every point; 22 merged selector tests pass.
  Development is now integrated. The full verifier passes with 118 project tests,
  including the wrapper running 77 coverage/evaluation checks.

- Pure planner: `src/equipo_jar/equipo_jar/cobertura_plan.py`.
- ROS adapter: existing `src/equipo_jar/equipo_jar/cobertura.py`.
- Generated test geometry: `pruebas/`, outside `src/`.
- `src/equipo_jar/test/test_cobertura.py` runs the coverage suites through
  colcon without placing geometry in `src/`. The normal verifier covers them.
- Offline evaluator: `pruebas/evaluar_cobertura.py`. Read practice maps from
  the simulator install; write results only under ignored `build/`.
- Fine candidate recovery retains every baseline observation. The bounded
  2-opt sweep preserves those poses and accepts only shorter estimated
  searches with no increase in mean first-view time for that fixed view set.
  Do not prune views using sampled coverage alone: full-grid visibility can regress.
- Planning is cancellable. Reject results after map/config changes or a
  substantial pose/camera correction during planning. Check cancellation in
  long loops so shutdown does not wait for the full calculation.
- Read `/map` and TF; publish `/jar/plan_cobertura` as a latched Path.
- Only propose oriented observation goals. Never drive, report or end a run.
- Mission in development keeps its own finished-goal list, using position and
  heading, and skips failed goals too. An initial Path can be integrated now.
  No actual-observation feedback is agreed: a reached goal does not prove an
  image was processed. Do not invent a topic. Navigation and detector evaluation
  remain open; mission handles candidate inspections and its real deadline.
- `INTEGRATION.md` is a proposal for the owners, not an agreed contract.
  `RESULTS.md` records local evidence and limits. One complete route plus
  return already exceeds the nominal 10-minute budget in the estimate;
  mission must manage the real clock and shorten exploration when needed.
- Coordinate conversion supports translated and rotated OccupancyGrid origins.
- Travel uses inflated obstacles; sight uses the original occupancy map.
- New params must reject invalid values and apply live on ROS 2 Humble.
- Shared files need user approval. Marco authorized publishing this first
  coverage baseline and its coverage-only parameters after local verification.
  Work on a short `cobertura/` task branch from `development`. New unrelated
  commits, pushes, PRs and server runs still need user authorization.
