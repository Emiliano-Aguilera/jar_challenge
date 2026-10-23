# Coverage goals grouped by mission

## Resolution checked against mission PR #9

Development `d375427` merges `7282d33`. Mission now uses 0.02 m and 0.1 rad
to recognize a previously finished plan pose. This is an approximate coordinate
identity, not a new goal ID or a navigation arrival tolerance. Mission records
the requested pose, not the robot's measured arrival pose.

Replaying all 18 saved plans with the merged selector and parameters selects
every intended index in order. The four losses below disappear; the difficult
nominal case selects 65/65 instead of 59/65. All 22 merged selector tests pass.
The identical selected sequence preserves the complete plan's predicted
visibility; this replay does not demonstrate successful navigation or detection.

The values distinguish the current 0.05 m cell centers and eight headings.
Recheck them for finer map resolutions, more headings, changed map geometry or
view generation. The merged parameter test assumes 0.05 m; it does not inspect
an incoming map's resolution. Failed goals still count as finished, and image
processing is not acknowledged. Those execution concerns remain separate.

Evidence: `build/verificacion/cobertura/revision-mision-d375427/result.json`.
The merge was first reviewed from an exported snapshot, then incorporated into
the coverage task branch with a fast-forward. The combined full verifier passes:
118 project tests, static checks and the map-waiting runtime audit.

## Original diagnosis before PR #9

2026-10-23. Measured locally against development `a8b9530` and its unchanged
`exploracion.first_pending` function. No mission code or tolerances were edited.

## Finding

Mission identifies a finished point by distance < 0.30 m AND heading difference
< 0.5 rad. Some distinct coverage viewpoints fall inside that tolerance. Skipping
them can remove predicted visibility, even though the complete published plan
contains those views.

| Case | Published goals | Selected goals | Lost dense cells | Lost sampled targets |
| --- | --- | --- | --- | --- |
| laberinto_1_start_0 | 65 | 59 | 65 | 10 |
| laberinto_1_start_1 | 69 | 62 | 40 | 9 |
| laberinto_1_start_2 | 65 | 60 | 55 | 9 |
| maze_3_6x6_start_1 | 61 | 60 | 8 | 2 |

The other 14 cases lose no dense cells to selection. These are deterministic
geometric predictions assuming successful goals, not missing-victim counts.

## Concrete example for Bautista

In `laberinto_1_start_0`, zero-based plan indices 31 and 34 both face
4.71238898038469 rad. Index 31 is at x=-2.675, y=2.125; index 34 is at
x=-2.925, y=2.125. They are 0.25 m apart, so reaching index 31 causes mission to
skip index 34. The planner treats those as different sensor positions.

The evaluator calls mission's actual selector with PoseStamped-shaped objects;
it does not implement a separate approximation of the identity rule. Dense
visibility is recomputed for both routes and all 18 practice cases.

## Original proposed discussion

Prefer distinguishing goals within a particular plan by their identity/index,
while defining what happens to completed work when a new Path replaces that plan.
That avoids using position tolerance as the identity of an intended sensor view.
Agree this with mission before implementing changes or a new message/topic.

The review of PR #9 clarifies that these identity tolerances compare requested
plan poses. Navigation arrival accuracy does not constrain the identity test.
Whether the robot can realize a useful change of viewpoint remains a separate
execution question; identity parameters do not adjust Nav2's arrival tolerance.

Other execution concerns still apply: failed goals count as finished, actual
image processing is not acknowledged, and candidate inspection may interrupt the
coverage route. Those require execution evidence before adaptive replanning.

## Evidence

- `build/verificacion/cobertura/presupuesto-base/budget.json` and `.md`.
- `build/verificacion/cobertura/presupuesto-optimizado/budget.json`, `.csv`, `.md`.
- `build/verificacion/cobertura/presupuesto-paridad.json`: all 18 plans and
  selected routes are unchanged by the computation optimization.
