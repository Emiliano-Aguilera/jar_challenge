# Coverage decisions

## 2026-10-26: faster calculations with identical output

Whole-position grouping saved only 0.586 s in 1/18 cases. Guarded pair grouping
saved at most 2.361 s in 2/18; relaxing sampled mean gives only small gains too.
Keep these as rejected prototypes. Renamed the local branch calculo-eficiente.
The current profile has 6.7 million free-cell queries and 2.7 million angular
differences. Reuse an immutable free mask for the latest threshold and exact
angular distances within each TravelCosts object. Retain all arithmetic and
corner/boundary rules; isclose may include a kernel cell beyond exact radius.
Use local snapshots and atomic cache replacement for different thresholds.

All 18 PR #14 route cases match exactly after integration. Alternating paired
CPU medians: difficult 6.768->5.493 s, simple 6.831->5.390 s, first maze
3.135->2.418 s. Pure suites pass 103 cases. Full project verifier passes 118
outer tests (120 internal coverage cases), including a live ROS threshold
change. Differential checks match 20,250 rays and 1,575 masks. The 90 s local
Nav2 smoke exits cleanly, with zero failed actions or protocol violations.
This reduces computation, not estimated driving
time or demonstrated score. Shared files/other modules are unchanged.

## 2026-10-26: consolidate repeated-position visits

Marco asked to continue measurable coverage work independently of the pending
mission agreement. Created cobertura/visitas-agrupadas from merged PR #14,
development 96134c0. The difficult saved routes have 10-11 repeated visits to
positions already used at another heading. Prototype whole-position grouping
with original/reversed heading order and insertion only at visit boundaries,
so a move never splits an already continuous visit. Keep all poses/headings.
Accept only shorter estimated search with no increase in search-plus-return or
sampled mean; compare periodic and strict sampled-curve guards. Check dense
budgets separately. Production is unchanged until results support a policy.

## 2026-10-26: bounded cleanup sweep

A second relocation sweep preserves every geometry-candidate pose and heading.
It shortens 7/18 predicted routes, up to 50.96 s, without reducing dense/sample
visibility at 60/120/300/480 s relative to that candidate. Keep the same guards
and a fixed bounded pass rather than iterating until convergence. All integrated
permutations match the prototype. CPU medians rise by 0.427/0.024/0.219 s on
the three measured maps; largest complete planning sample is 6.336 s.
The difficult offset-2 geometry tradeoff remains: 517.9 versus 446.5 s under
the old transit-only policy. No physical score improvement has been proved.

## 2026-10-26: minimum and preferred observation clearance

The strict 0.30 m observation experiment lost 318 dense cells and 56 sampled
targets in the difficult nominal case. Retain the separated transit model,
but use a 0.24 m minimum and prefer 0.30 m. Fine recovery exhausts preferred
centers before adding a minimum-clearance witness for a missed target.
The padded Nav2 rear corner is sqrt((0.181+0.02)^2+(0.10+0.02)^2), about
0.2341 m; 0.24 m encloses it in the known map. This is not protection against
map error, pose error, arrival tolerance or boxes missing from the map.

The full 18-case evaluation retains transit connectivity and all mission
goals. Difficult nominal final loss falls to 26 dense cells / 5 samples;
offset 2 still has a 78.2 s route regression and worse early visibility.
Do not call this an across-the-board efficiency gain. Marco approved the two
coverage-only YAML entries. Full verifier passes; actual-navigation probes
and route/stops evaluation remain separate work.

## 2026-10-26: separate observation stand-off from transit

- User asked to start the four coverage corrections identified by Hans, beginning
  with viewpoint geometry and then route/stops. Created
  `cobertura/espacio-observacion` from current development `71ca3f1`.
- Current transit mask uses 0.21 m effective radius; applying 0.30 m globally
  disconnects 211 already-clear observation centers in the difficult map.
  Navigation uses a rectangular footprint and heading-dependent clearance.
- Added separate observation-center eligibility, default proposal 0.30 m, with
  the original transit mask retained. This is conservative in the known map;
  it does not validate real map error, localization, boxes or narrow transit.
- Sampling, fine recovery and the initial pose all obey the new stop rule.
  Eleven generated geometry cases plus existing pure suites pass (92 total).
- The 18-case dense evaluation is running; shared YAML remains unchanged until
  the minimal proposed addition is verified and approved. No publication yet.

## 2026-10-23: fixed-view observation ordering

- Question: can the same selected views observe useful space sooner with less
  estimated travel, while retaining final dense visibility?
- Created `cobertura/orden-observacion` from development `10f97ca` after PR #10
  merged. Preserved the other worktrees and the clean publication branch.
- Compared nine relocation policies on the 18 saved practice cases. Mean-only
  optimization and unaligned periodic checks regress some early dense budgets;
  strict full-curve dominance saves little. These variants remain experiments.
- Chose one bounded relocation sweep after existing 2-opt. Retain every pose
  and heading. Require nonincreasing search, search-plus-return and sampled mean
  first-view time; preserve sampled visibility on a periodic predicted-clock grid.
- Marco approved only two coverage-section additions to shared params.yaml:
  order_check_interval_s=60.0 and order_start_delay_s=8.0. The latter estimates
  startup; it neither changes nor measures mission's actual clock. Recheck the
  estimate if mission's startup wait or observed planning/localization delay changes.
- Integrated 18-case evaluation matches the isolated prototype exactly, retains
  all views/final visibility, and mission selects every goal. No dense/sample
  visibility loss at 60/120/300/480 s in these cases. Median estimated saving
  is 11.43 s; difficult nominal search is 540.52 -> 482.14 s, excluding startup.
- Sampling remains approximate: dense mean first-view time slightly increases
  in one case, despite improvements in sampled objectives. No universal claim
  is made about visibility between samples/checks or actual victim detection.
- Evidence: build/verificacion/cobertura/orden-integrado/comparison.json and .md.
  Paired CPU medians: difficult map 5.586 -> 6.006 s; simple labyrinth
  5.294 -> 5.507 s; first maze 2.225 -> 2.221 s (noise-sized). Both versions
  retain the same view set and deterministic output across all repetitions.
- Full final project verifier passed: 118 outer entries, including the wrapper
  running 97 coverage/evaluation cases; zero static/hardcoding warnings and zero
  runtime audit problems. Only the approved coverage YAML additions differ.
- First simulator handoff failed: map preceded connected robot TF by about 54 s;
  judge ended by inactivity at 60 s. This is retained under
  build/verificacion/cobertura/integracion-orden-20261023/.
- Repeated the communication fixture after waiting for fresh map/base/camera TF
  before starting the judge. Passed: 50 published views, three mission goals
  checked in order, six process parents exit 0 and team children close cleanly.
  Evidence: integracion-orden-listo-20261023/probe.json. This fixture preparation
  does not fix or validate actual competition startup, navigation or detection.
- Ordering work is complete. Marco authorized publication, then confirmed merge
  once checks pass with no issues. Publish only this verified advance to the
  team fork's development branch, preserving the failed-probe evidence/limits.
  Alternative view selection/orientations are the next separate experiment.

## 2026-10-23: prepare the budget evaluation advance for publication

- Marco authorized publishing this advance, then merging after all checks pass
  if there are no conflicts or requested changes.
- Fast-forwarded the task branch to development `d375427`; all pending coverage
  changes were preserved. No contribution changes to shared parameters or mission.
- Repeated the official verifier: two packages built, 118 project tests passed,
  static checks and map-waiting runtime audit passed. The coverage wrapper runs
  77 internal cases. Parameter values match development.
- Repeated simulator handoff: 50 viewpoints, first three mission goals match
  the Path in order, all process parents exit 0. Navigation was fake and
  localization used the test identity; physical performance remains unmeasured.
- Publish the computation optimization, evaluator, tests and documentation as
  one task PR to the team fork's development branch. Keep runtime artifacts ignored.

## 2026-10-23: budget evaluation and witness preparation

Question: what can mission actually execute, where are the model's gaps, and
which part of planning is expensive?

- Created `cobertura/evaluacion-presupuesto` from development `a8b9530`, reusing
  the clean coverage checkout. Marco requested careful local improvements and
  a live progress record; `STATUS.md` tracks this task. Nothing new is published.
- Added budget metrics at 60/120/300/480 s, dense and raw-connected denominators,
  estimated startup/travel/turn/dwell, and mission's actual point selector.
- Added exhaustive witness searches for outside targets. Nominal difficult map:
  182 samples are raw-disconnected; 205 have no reachable safe-center witness.
  Of the latter, 78 have a witness in another safe component and 127 have none
  among all safe centers under the current camera/range/heading model. There is
  no additional position-sampling gap in the six nominal cases at these settings.
- Mission skips useful views in four of 18 cases, losing 65/40/55/8 dense cells.
  See `MISSION_COMPATIBILITY.md`. No mission code, tolerances or ROS interface
  was changed. Distinct-goal semantics need coordination with Bautista.
- Profiling identified repeated single-target camera transforms in fine recovery.
  Prepared heading/camera terms now reuse those calculations, preserving the
  arithmetic order, range/FOV checks and ray semantics.
- Compared 18 cases before/after: identical poses, selected goals, dense/sample
  visibility, travel, turning and normalized event times. Added 600 generated
  parity comparisons plus exact FOV/range-boundary checks.
- Alternating paired runtime checks (one warmup, three repetitions per variant):
  difficult map median 7.515 -> 5.606 s; simple labyrinth 6.003 -> 5.003 s;
  first maze 1.936 -> 1.886 s. Outputs and route metrics match on every run.
  These are local CPU measurements, not speed gains in actual robot travel.
- Kept robot radius, margin, detector assumptions and all team parameters
  unchanged. More samples alone cannot repair the model-limited targets.

## 2026-10-22: prepare the first shared baseline

- Marco authorized a commit, push and PR of the working coverage baseline so
  teammates can integrate it. Earlier entries below describe the local-only
  phase and are historical.
- Started `cobertura/base-integrable` from current development `c01ccdc` in a
  separate worktree. The original local work is preserved in its checkout.
- Copied the tested planner, ROS adapter and tests. Added only coverage's 17
  parameters; detection, fusion and mission settings remain those of development.
- Updated the integration context: mission already remembers finished goals,
  including failures, and handles doubtful-candidate inspection. No new ROS
  message or progress topic is needed to try the initial route.
- Mission's 480 s return and 570 s finish policy remains authoritative. Range,
  dwell, localization and Nav2 performance need measured joint runs; this is not
  a claim of an optimal or tournament-winning algorithm.
- Shared map geometry work remains separate. This baseline does not depend on
  PR #6 and can be reviewed independently.
- Current verification passed the full official verifier: build, 113 outer
  colcon checks (including the 54-case coverage wrapper), static checks and
  map-waiting interface audit. Other nodes' YAML parameters were checked equal.
- Simulator integration produced 50 views and mission consumed three distinct
  published goals with Bautista's fake navigation. All processes and team
  children shut down cleanly. This is message/action integration, not real travel.

## 2026-10-17: local baseline

Question: what can coverage build before navigation and mission execute goals?

- Reviewed: current `development`, `CLAUDE.md`, team flow and interfaces.
- Decided: generate an initial ordered `nav_msgs/Path` on
  `/jar/plan_cobertura`. Do not send navigation actions or publish velocities,
  reports or `/done`. Use the existing launch and entry point.
- Tried: a pure Python planner on generated rooms and corridors. Candidate
  centers use a circular footprint plus margin; unknown cells and map edges
  block travel. Visibility uses the original map, the camera field of view
  and range. Rays cannot cross walls or cut wall corners.
- Decided: choose views by new predicted visible cells per estimated second
  (grid travel distance, heading change, observation time). The route is a
  baseline, not a proof of optimality or a time guarantee. Nav2 still plans
  each leg and handles objects absent from the map.
- Camera translation and forward axis come from TF. The field of view and
  useful range are parameters; useful range needs perception experiments.
  The planar model omits camera pitch, vertical field of view, object size,
  dynamic occlusion and actual detector performance.
- Only the start's connected safe component receives goals. Sampled target
  cells may be near walls where the robot itself cannot fit. Targets in
  closed, unreachable rooms remain in the predicted-coverage denominator.
- Limit positions, sampled targets and output views; enlarge the sampling
  bins for large maps. Small regions may be missed by sampling; this is not
  a guarantee that every possible box is observable.
- Replan on map content or committed parameter changes. Do not constantly
  replace mission's route when the robot moves. A worker keeps the ROS node
  responsive; ignore results from old map/parameter generations.
- Keep generated geometry and tests in `pruebas/`, outside `src/`, as the
  team interface guide requires.

## Pending agreements before execution integration

The current interface sends coverage to mission but defines no feedback.
No additional topic or message has been introduced.

- Mission (AlsinaBautista): agree how to identify the active goal and report
  reached, observed, failed, cancelled and timeout outcomes. Until then the
  published route is an initial proposal, not a live list with completed
  goals removed. Mission must not restart that route on every replacement.
- Localization/navigation (hans-die): confirm `base_footprint`, TF freshness,
  robot radius, safety margin, tolerances and treatment of blocked goals.
  This baseline does not consume a dynamic costmap or pose uncertainty.
- Perception: confirm camera frame, useful range/FOV and evidence that a view
  was processed. Predicted visibility is never recorded as an observation.
- Mission: an empty route or a partial predicted-coverage value must not be
  interpreted as all boxes found or a reason to publish `/done`.

## Local measurements

- Pure geometry/planning suite: 27 checks passed, including rotated origins,
  negative coordinates, malformed maps, inflation boundaries, unknown cells,
  thin walls, corner occlusion, disconnected components and camera direction.
- A boundary check found floating-point rounding at exactly the safety
  distance; touching cells are now conservatively blocked.
- Generated 120 x 120 grids, 0.05 m/cell, on the local WSL environment:
  open room 3.54 s / 43 views; corridors 2.88 s / 48 views. Both predicted
  visibility for 1600/1600 sampled cells. These are synthetic planner timings,
  not navigation runs, detection results or competition coverage guarantees.

- ROS adapter suite: 12 checks passed; 39 coverage checks in total. Confirmed
  map/TF gating, stale dynamic TF rejection, late subscribers, transactional
  parameter rejection, live replanning, old-result rejection and recovery
  after localization moves an unsafe start into free space.
- `bash tools/verificar.sh --template`: all checks passed, including build,
  3 existing report checks, no static warnings and no interface violations.
- Local simulator smoke test: existing `pruebas.launch.py`, official practice
  map and real simulated camera TF. Published 50 oriented goals; predicted
  visibility 1345/1383 sampled cells (38 remain outside this candidate plan).
  This tested message/TF integration only: no navigation, detector, real AMCL
  or competition score was tested. The identity `map → odom` is a test aid.
- Captured goal plot and runtime poses under ignored
  `build/verificacion/cobertura/`. The four parent processes exited zero;
  a later log review found child interruptions during cleanup (see below).
- The user requested local verification before sharing parameter changes.
  All 17 proposed values loaded through ROS's YAML parser and passed the
  official parameter-value checks. Add only the coverage section locally;
  keep all work uncommitted and unpushed until the user asks.

## 2026-10-17: diagnose gaps and improve the route

Question: does the initial plan generalize, where does it miss cells, and
can we reduce travel cost without losing useful observations?

- Added metrics for grid travel, heading changes, observation time, return
  distance, mean first-view time and the cumulative predicted-coverage curve.
- Added diagnostics that separate targets outside the candidate model from
  visible targets deferred by the view budget. Check the complete free-cell
  grid too; a sampled percentage can hide small gaps.
- Tried dropping sampled-redundant views. A full-grid check found a visibility
  regression between samples. Rejected that approach. Keep every baseline
  pose, append useful fine-grid candidates, then reorder the same view set.
- Fine search checks nearby safe cells for missed targets and skips raw
  disconnected components. It stays within max_positions and max_views.
- One bounded 2-opt sweep accepts improvements only when estimated search
  duration decreases and mean first-view time does not increase for that
  fixed view set. No claim of a globally optimal route.
- Compared all six practice maps at three starts each. All 18 starts were
  accepted; no baseline pose was removed. Full-grid comparison at each map's
  default start confirmed no visibility loss. See RESULTS.md and ignored
  build/verificacion/cobertura/evaluacion-final/ for the full report.
- The original 38 pending samples in maze_1_6x5 are all disconnected from
  the start even before footprint inflation. This classifies those samples;
  it is not a claim that every other free cell is visible in reality.
- The difficult map gained 13 sampled targets and 74 full-grid cells at the
  default start. Additional observations can increase total travel; the
  report shows that tradeoff rather than calling all variants faster.

## Robustness and verification

- A regression test failed on the original adapter: a result computed before
  a pose correction was published. It now passes for both translation and
  heading corrections. Compare start snapshots before publishing; larger
  changes than half the spatial/angular sampling step trigger a new plan.
- Cancel calculations on map/config updates and node destruction. Long loops
  cooperate with cancellation. Tests cover rejection of old results, cancel
  signals, and destruction while a worker is active.
- The original probe sent SIGINT to the entire launch process group. Launch
  forwarded it again, interrupting child cleanup twice. The corrected probe
  signals launch's parent, waits, and uses group termination only as a timeout
  fallback. The final simulator probe and all team children closed cleanly.
  No shared launcher code or other module was changed.
- 54 coverage checks pass automatically through the colcon test wrapper,
  together with the 3 existing report checks. The official template verifier
  passed build, static checks, map waiting and interface audit (zero problems).
- The final local simulator probe received the real simulated map/camera TF
  and published 50 views. It exercised the existing test identity map-to-odom
  transform; it did not test AMCL, Nav2 execution or detector performance.

## Decisions left for integration

- Keep radius and margin unchanged. On the worst connected-gap case, changing
  the margin from 0.03 to 0.015 m did not improve coverage; removing it gained
  just one sampled target. This offline sensitivity is not permission to
  reduce physical clearance. Agree footprint behavior with navigation.
- At one start the full search proxy is 572.8 s plus about 8.4 m to return.
  At the configured 0.25 m/s that exceeds 600 s even before unmodelled delays.
  Mission must stop the search early when needed and owns the real deadline.
- Do not turn predicted views into observed state or introduce an execution
  feedback topic until both owners approve the agreement in INTEGRATION.md.
- Useful range, observation dwell and target geometry still need perception
  experiments. Do not optimize away meaningful gaps using unconfirmed box sizes.
## Mission identity review (2026-10-23)

- Question: does Bautista's merged PR #9 resolve the distinct-view loss?
- Reviewed development `d375427` / mission commit `7282d33`. Only identity
  defaults/YAML values, selector tests and mission notes changed.
- Replayed the 18 saved plans using the merged selector and 0.02 m / 0.1 rad
  parameters. Every intended view is selected in order; previous four cases
  lose no predicted visibility to identity grouping. All 22 selector tests pass.
- Correction to our earlier discussion: mission stores requested goal poses,
  not measured arrival poses. These values do not require 2 cm Nav2 accuracy.
- Limits: current cell centers/headings; recheck finer grids or heading spacing.
  Actual observation and failed-goal handling remain execution questions.
- Evidence: `build/verificacion/cobertura/revision-mision-d375427/result.json`.
  Fetched origin and tested exported files. No merge, commit or push locally;
  pending coverage work and local parameters remain intact.
