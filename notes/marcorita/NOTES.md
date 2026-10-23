# Coverage decisions

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
