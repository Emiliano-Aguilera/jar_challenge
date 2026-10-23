# Context for marcorita's agent

Module: cobertura. Follow root `AGENTS.md`, `CLAUDE.md`, `docs/team/FLOW.md`
and `docs/team/INTERFACES.md`. Read `notes/marcorita/NOTES.md` for decisions.

## Local planner

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
