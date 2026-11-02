# Exact ordering calculation before delivery — 2026-11-02

Branch cobertura/orden-calculo-final starts at development a162afe, including
the merged near-wall start recovery. Recover only the previously unpublished
ordering CPU optimization; preserve earlier work in the older checkout.

## Change

Most proposed order changes fail the existing duration condition. Compute that
same duration first and omit full visibility/coverage-curve calculations only
for proposals already rejected by that condition. Every remaining proposal still
passes the original metrics, visibility, mean-time, return and clock guards.

The directed-edge cache belongs to one ordering call and its fixed selected
view set. Increment arithmetic and elapsed accumulation keep the exact original
order, including floating-point comparisons. No sum regrouping, symmetry shortcut,
pose/heading change, pruning, parameter, dependency or cross-call cache.
Cancellation remains at each trial boundary and in travel-cost calculations.

## Acceptance work

- Directed costs, optional cached/custom turn costs, repeated views, empty
  prefixes, shuffled paths and angles around +/-pi must match authoritative
  route_metrics exactly. A full-order regression bypasses only the new early
  rejection to verify equal selection and fewer complete metric calculations.
- Compare complete serialized Plans against a162afe with fresh grids on every
  call: 18 normal map/start cases, three near-wall headings and the organizer
  mini map's hypothetical origin. One warmup and three alternating paired
  repetitions per case. No concurrent heavy local checks during measurement.
- Record wall and process CPU samples. At least 5% reduction in the sum of case
  wall medians, with no individual median regression exceeding 5%, is the
  predeclared local benefit gate. These are practical gates, not statistical
  confidence guarantees or a robot-wide accumulated planning time.
- After the benchmark, run the full repository verifier on the same source.
  Earlier 15.24% evidence belongs to the old baseline, not this new port.

## Final local results

All 22 complete Plans remain exactly equal in every warmup/measured call,
including start_anchor, unpolished/final metrics, views, order and target counts.
The sum of case medians is 87.256 -> 73.344 s (15.94% lower); process CPU
medians fall 15.84%. This sum covers different independent inputs, not one
robot's startup time. Twenty-one cases improve (4.85–22.28%).

| Representative case | Reference median s | Candidate median s |
| --- | ---: | ---: |
| Difficult map, start 0 | 5.996 | 4.720 |
| Difficult map, start 1 | 5.910 | 4.593 |
| Maze 3, normal start | 3.173 | 2.525 |
| Near-wall start, yaw 1.571 | 5.373 | 4.339 |
| Near-wall start, yaw 3.142 | 5.352 | 4.458 |

The original mini-map triplet is slightly slower: 1.471 -> 1.520 s (+49 ms).
A separate balanced follow-up, after all heavy checks ended, uses two warmup
pairs and six measured pairs: 1.2442 -> 1.2470 s (+2.8 ms, +0.23%). Plans are
still identical. Treat this short case as effectively neutral, not a proven
speedup. The follow-up does not replace the original 22-case aggregate.

132 focused tests passed. Full tools/verificar.sh --template exits 0: build,
262 package tests, structure/params/static/dependencies, 19 audited nodes,
zero interface problems and zero pre-map motion/reports/protocol violations.
The runner terminates its fixture processes during cleanup, as usual.

Evidence: ignored build/verificacion/cobertura/orden-final/comparacion.json,
mini-followup.json and verificacion.json. Final source hashes are unchanged:
cobertura_plan.py 88570f72da47988cc2967fdebadecc81dc322262ddb3b4c8d70b10f12ba7b228;
test_orden_cobertura.py cdeaf5199671940d4f34ddfbdf53eaeaa0db1c31fb1d9327efb00674dfd601e8.
These are local compute measurements, not detection/score or hardware timing
guarantees. Faster publication can alter asynchronous mission timing; remote CI
and runtime integration remain publication gates.

The existing pruebas/comparar_rendimiento.py can reproduce nominal-map pairs
against a saved a162afe planner. The full local 22-case/fresh-grid harness and
the balanced mini follow-up are in the adjacent audit folder outside the repo;
their receipts retain input/settings/camera/source hashes and individual times.

Marco requires closing today and authorized publication and merge of this
verified optimization after green CI with no pending observations. The shared
server is available if needed; no manual run has been dispatched for this task.
Existing #16/#36 drafts and narrow-passage work remain separate. Only the pure
planner, its ordering tests and this note belong to the candidate; pending access
diagnostics/personal notes are preserved locally. CI supplies runtime smoke tests.
