# Coverage ordering: three authorized server runs

2026-10-26. PR #16 remains DRAFT. All four CI checks pass on corrected source
37c2306, but behavioral validation has open issues. No merge, score gain or
faster real execution is claimed. No fourth manual run is authorized.

All three runs use laberinto_1, 300 wall seconds, recording, the same simulator
image jarsim/sim:d607026-d7b7baf-r2 and have no protocol violations.

| Version | Commit | GPU / slot | RTF | Points | Report at | Nav success / failure |
| --- | --- | --- | --- | --- | --- | --- |
| Reference | 79ec03c | 0 / 0 | .973 | 100 | 74.83 s | 70 / 0 |
| Rejected global restart | 755c891 | 1 / 0 | .748 | 0 | none | 59 / 0 |
| Corrected view deadlines | 37c2306 | 0 / 0 | .972 | 100 | 161.92 s | 44 / 4 |

Run links: [reference](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40383675385),
[rejected](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40385012550),
[corrected](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40402162637).
The corrected batch metadata confirms 37c23063cdb761b2c47cd6289c458f7dc464ef35.
It recovers the report, 2.58 cm from its practice target (reference 5.51 cm),
but is 87.09 s later. One run per version does not establish reliability.
Equal GPU/image/RTF improves comparability, not deterministic execution.
The practice score omits unregistered collisions; zero protocol violations
does not certify collision-free motion.

## Evidence needed by mission

Actual recorded coverage Paths have equal view multisets and first differ at
index 37. The reported box is observed four times at 73.137-73.984 s, close to
the reference window of five observations at 73.648-74.772 s. Corrected fusion
still has sigma .15 m. At 84.30 s mission finds no inspection viewpoint and
marks that candidate inspected. A fifth observation at 161.809 s reduces
sigma to .0486 m, enabling the report at 161.92 s.

Reaching/ordering intended views does not acknowledge processed camera data.
Bautista needs the bounded-retry and evidence/second-view discussion in
INSPECTION_REPLAY.md. A larger ring or arbitrary fixed dwell is not a validated
solution. Do not lower the report threshold to make the test pass.

## Evidence needed by localization/navigation

The first failed goal is (-2.825,.225, heading 0), after earlier goals down
the left corridor. Subsequent failures concern nearby requested poses.
Recorded AMCL at sim stamp 247.420 s is (-2.265303,.177562). Simulator truth
at 247.421 s is (-2.735006,-.081186), a .536257 m translation error. The
estimated position has occupancy 100; actual position has occupancy 0 in the
recorded map. Practice world/map frames are aligned; both planar poses refer
to the robot base. This is a concrete case for Hans to inspect AMCL, TF and
costmaps. It does not prove the route has no influence on localization.

Truth was read only by an external diagnostic; no localization, navigation,
mission, map helper, shared interface or runtime parameter change was applied.
Keep the improved static planner experimental until these execution issues
are resolved or its behavioral regression is otherwise ruled out.

## Saved evidence

Ignored build/verificacion/cobertura/server-recorrido/ holds each run's
resumen.json, metadata, judge result and equipo.log; comparison-corrected.json
checks fixed commits, image, map, duration and recordings before summarizing.
corrected/messages.jsonl contains the map, static TF, Path, 15 observations
and report. It includes an earlier separate one-observation candidate.
corrected/inspection-window.jsonl, report-window.jsonl, first-nav-failure.jsonl
and localization-diagnostic.json retain the narrow diagnostic windows.
The multi-GB recording was read in bounded ranges, not downloaded whole.
