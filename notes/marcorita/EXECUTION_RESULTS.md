# Execution audit results, 2026-10-28

Reference: development `93bdd59`, after the mission integration. Branch
`cobertura/auditoria-robusta` publishes only the evaluator/auditor/tests/notes;
heading-optimization scripts and their tests remain local.
The branch subsequently incorporated `b271132` (mission #31). Geometry and
successful-goal proxy inputs are unchanged; real runs can cancel after 10
recoveries, excluding the return. The record audit preserves optional counts.
Then incorporated `90a4fca` (#32, return checks/retries). These do not change
the successful-search command model; return cost remains a proxy, not proof
of safe return, retries or the full-wheel bonus.

## Ready tooling

- Full package suite: 155 outer tests passed on latest `90a4fca`, including the nested coverage/map
  suites. New tests cover command substitution, fixed box exclusions, actual
  settle timing, failure/cancellation accounting, pose/clock provenance,
  heading-error sensitivity. The earlier local run also included unpublished
  heading-optimization guard tests; the PR wrapper excludes that separate suite.
- Earlier runs passed 147 outer tests on 93bdd59/b271132. The latest base adds
  return-policy tests. `compatibilidad-merges.json` verifies unchanged selector
  helper ASTs, coverage planner and the modeled mission configuration fields.
- Structure and parameter checkers passed. Static audit: zero errors and zero
  hardcoding-review warnings; ordinary team-topic notices remain unchanged.
- External-record CLI smoke passed on a generated map: one successful, one
  failed and one canceled trip; 9 s duration retained, only success credited.
  These were explicit synthetic commanded records, not a real robot run.
- Normal budget CLI completed all 18 cases. Every plan pose/order exactly
  matches the saved `calculo-integrado` reference; runtime algorithm is unchanged.

## What the new command proxy reveals

58 commanded views change position under mission's current in-place substitution,
across the 18 cases. The identity-only proxy misses this effect.

| Case | Final nominal cells lost | Added elsewhere |
| --- | --- | --- |
| difficult start 0 | 19 | 0 |
| difficult start 1 | 43 | 0 |
| difficult start 2 | 30 | 6 |

The other 15 cases lose no final nominal cells under exact commanded arrivals.
This reproduces the earlier diagnostic in reusable tooling. It is not a count
of missed boxes; actual arrival error, images, inspections and transit views
can change the outcome.

No-box and fixed synthetic-box scenarios were also evaluated on all 18 saved
plans with -10/0/+10 degree heading samples. Fixed zones are identical across
variants in each case, not detections recomputed from a modified trajectory.
The synthetic box is a diagnostic stress case, not practice victim truth.

Time values are proxies, not measured run durations. Startup treats the saved
wall-CPU planning sample as a hypothetical 1x startup delay, overlapping the
mission wait. Do not equate that with simulated /clock elapsed time or robot
hardware latency. External records must use one actual clock and /map origin.

## Separate unpublished heading experiments

Adding 12 headings alone reduced yaw-envelope visibility in all three initial
nominal cases. Planning with a narrower FOV sometimes improved the envelope
but could lose nominal cells or increase duration. None was activated.

The complete heading-only search uses up to 320 trials, changing no positions
or visit order. It preserves all reference nominal and sampled-envelope cells
at 60/120/300/480 s and completion, in no-box and fixed-box scenarios, with no
estimated duration increase. It also checks 5/15-degree envelope guards and
replays the final route through the actual mission selector/command helper.

| Initial case | Heading changes | Dense cells retained under sampled +/-10-degree error, no boxes | Nominal visibility |
| --- | --- | --- | --- |
| difficult | 6 | 8009 -> 8079 | 8650 -> 8656 |
| simple | 2 | 11931 -> 11936 | 12688 -> 12688 |
| first maze | 1 | 10129 -> 10138 | 10834 -> 10836 |

The fixed-box scenarios also retain all reference regions and improve the
envelopes. Clock guards are about cell identities, not just total counts.
These three cases do not establish performance over all maps or real noise.

Complete search took about 15-18 s per case in one local sample with other
local work active. This is not a paired CPU or robot benchmark. The bounded
16-trial search took about 6-8 s and accepted zero changes in those three cases.
It is rejected as a useful runtime improvement. The complete search remains
an offline prototype; no unverified search delay was added to robot startup.

## Evidence

Ignored `build/verificacion/cobertura/auditoria-robusta/`:

- `presupuesto-integrado/budget.{json,csv,md}`: 18 dense command-proxy cases.
- `baseline/execution.{json,md}`: 18 no-box plus 18 synthetic fixed-box scenarios.
- `quick/execution.{json,md}`: initial heading-count/FOV comparisons.
- `orientaciones/headings.json`: rejected 16-trial bounded experiment.
- `orientaciones-completas/headings.json`: complete heading-only experiment.
- `cli-smoke/`: generated map, explicit record input and result (not source data).

The next production decision needs current recorded execution and the Workshop
10 real-robot bag, with camera/pose timing, not just a higher proxy percentage.
No new manual simulation or server run was made for this audit. Marco authorized
publication of the small audit PR and merge after successful checks on 2026-10-28.
