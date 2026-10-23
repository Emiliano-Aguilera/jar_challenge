# Coverage work status

Updated: 2026-10-23. Branch: `cobertura/evaluacion-presupuesto`.
Base: development `d375427`, including mission's distinct-view correction.
The reference measurements used the earlier `a8b9530` planner.

## Current task

Build an offline evaluation of visibility within a time allowance, including
mission's actual point selector. Use the evidence to choose planner changes.
Marco requested careful improvements and an updated progress record, then
authorized publication and merge after all checks pass. This document records
the verified publication candidate; GitHub holds the current PR/merge state.

## Completed

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

## Verification completed

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

## Current local state

Evaluation and computation optimization are implemented and verified locally.
Development `d375427` was incorporated with a fast-forward, preserving all local
coverage changes. The contribution changes only coverage code/tests, evaluation
scripts and marcorita's notes. It adds no changes to mission, shared parameters,
interfaces or navigation relative to that development revision.

## Findings requiring coordination

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

## Next decisions

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

## Evidence and limits

The merged baseline passed 54 internal coverage cases, official CI and a local
simulator handoff with fake navigation. The local follow-up has 77 internal cases
and the verification above. Neither validates physical travel, AMCL or actual
victim detection. A safe-center witness search is limited to its explicit model.
