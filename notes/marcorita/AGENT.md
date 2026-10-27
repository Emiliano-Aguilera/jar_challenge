# Context for marcorita's agent

Module: cobertura. Follow root `AGENTS.md`, `CLAUDE.md`, `docs/team/FLOW.md`
and `docs/team/INTERFACES.md`. Read `notes/marcorita/NOTES.md` for decisions.

## Local planner

### Current task: unblock mission's inspection tests (2026-10-27)

Marco requested publishing/merging our rotated-map test fix to unblock PR #26,
then explicitly preferred Bautista's final API without a compatibility layer.
Branch cobertura/test-inspeccion-alcanzable is based on his pending branch
mision/visitas-alcanzables (9b9fc1b); send its PR to that branch, not development.
This keeps the test fix with the function signature change. Bautista remains
responsible for merging his overall mission PR #26 into development.
The test directly passes real rotated Grid/reachable cells; same assertions.
See INSPECTION_ROTATED_TEST.md. No production or mission parameter edits.
Publish and merge after green checks with no pending issues. Run local ROS
tests on domain 177; Marco has a manual Gazebo/RViz session on domain 77.
The shared helper is already merged in PR #23 (055a559). PR #16 stays separate.

### Previous task: shared map reachability (2026-10-27)

Branch cobertura/mapa-alcanzable from origin/development d7e8057. Marco asked
to continue after Bautista's change. PR #20 is merged (f42cdd8), including the
fifth first_pending argument [] in offline metrics, with four green checks.
Fresh snapshot metrics/exploration checks passed 56 tests. PR #19 reduced
mission start_wait_s from 8 to 4; PR #21 records team integration requests.

Grid, Pose, planar_yaw, cancellation and reachable_cells now live in mapa.py;
old coverage import paths remain aliases. Grid.from_occupancy_grid copies a
validated map-frame message; reachable_from returns a frozen connected disk-safe
component, with explicit radius and occupancy threshold. See MAP_REACHABILITY.md.
Mission source/parameters are untouched. Bautista still needs to use the helper
in viewpoint_for; do not claim runtime inspection is fixed by this branch.
Focused geometry/planning: 88 passed. Full verifier: 137 outer tests passed,
build/static/interface/map-wait checks green. All 18 plans/diagnostics/metrics
and safe components exactly match development. Recorded pocket-map check
keeps exterior 174/175-degree points and rejects disconnected 285 degrees;
static diagnostic only, not a new run. See STATUS and ignored mapa-compartido/.
Marco authorized publication of this verified helper on 2026-10-27, with the
standing instruction to merge after all checks pass and no pending issues.
No new manual server job is authorized. GitHub holds the publication status.

The prior coverage checkout remains on cobertura/recorrido-laberinto (PR #16
DRAFT), with its pending local notes preserved. Do not merge that PR simply
because this helper passes. The old task descriptions below are historical.

- Current task: `cobertura/calculo-eficiente`, from development `96134c0`
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
