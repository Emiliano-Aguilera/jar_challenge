# Recorded inspection failure for mission

2026-10-26. No mission, navigation, map helper or interface change applied.
This is a failure case for Bautista to review, not a request to relax reporting.

## Follow-up in corrected server run

The authorized third run, 40402162637 / commit 37c2306, recovers a 100-point
report at 161.92 wall seconds. Its reported candidate (id 2) first receives
four observations at 73.137-73.984 s, with sigma .15 m. At 84.30 s mission
again logs "Sin lugar para mirarla" and marks the candidate inspected.
The fifth observation arrives at 161.809 s, reduces sigma to .0486 m and
enables reporting. This shows that the inspection mark suppresses another
inspection attempt; it does not block a later automatic report if evidence
arrives independently. A separate one-observation candidate appears at 53.875 s.

The actual coverage Path keeps the same view multiset and first 37 views as
the reference. Its early reported-box observation window is near the old
73.648-74.772 s window, but provides four observations instead of five. Thus
the intended view sequence alone does not guarantee processed evidence or
the report threshold. Do not prescribe a fixed short dwell as a proven fix:
mission must consider processed frames, confidence and a useful second view.
Full comparison and remaining navigation failures: SERVER_COMPARISON.md.
Decoded inspection/report windows are saved under server-recorrido/corrected/
in ignored build/verificacion/cobertura/.

## Recorded sequence

The first coverage candidate was tested in server batch
20261026-161508-755c891 ([run](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40385012550)).
It detected the same candidate three times, at about 85.675, 86.070 and
86.463 wall seconds after map publication. Fusion kept sigma .15 m. Reporting
requires five observations and sigma <= .10 m; those conditions were not met.

At wall timestamp 1793042304.544276305, mission logged:

```text
[mision]: Candidata id: 1. Sin lugar para mirarla
```

The latest delivered candidate then still had two observations. Its center
was (2.3427337944470494, 1.488008206923545), covariance diagonal .0225 m².
The nearby recorded AMCL pose was approximately (0.426244, 1.376978).
The three-observation update arrived later, around 87.73 wall seconds after map.
Both the center cell and the west margin endpoint used by the selector are
free in the recorded OccupancyGrid. Do not diagnose this as an occupied center.

In the no-view branch of mission's inspection attempt, it appends this center
to inspected_points and returns false. Coverage continues; this attempt is
then remembered as an inspection despite no inspection goal being executed.
No further detections or report were recorded in the 300-s run.

## Local replay

The diagnostic calls the current inspection viewpoint function using the
recorded map, the latest candidate delivered before that log, current mission
parameters and the nearby recorded robot pose. It uses no practice-box truth.

- Current .8 m ring / 12 angles: no point.
- .8, 1.2, 1.6 and 2.0 m rings with 12 or 72 angles: no point.
- All published coverage poses: no point under the same ray/clearance checks.
- Exhaustive reachable-component grid search, clearance .25 m, within 2 m
  and at least .4 m from the robot: no point under those checks.

The geometry model therefore rejects this case beyond sparse ring sampling.
It does not prove that the physical camera cannot obtain another observation.
Actual camera faces, an uncertain fused center and a conservative planar ray
are different models. A larger inspection radius alone is not a demonstrated fix.

Local evidence is in ignored build/verificacion/cobertura/server-recorrido/:
candidate/equipo.log, candidate/candidate-window.jsonl,
candidate/inspection-replay.json, and baseline/messages.jsonl for the map.
The server recording/logs remain accessible through the batch viewer over
Tailscale. The multi-GB recording was read in bounded ranges, not downloaded whole.

## Agreement needed with mission

Distinguish a failed attempt from a completed inspection and decide when a
candidate becomes eligible for a bounded retry, for example after new evidence
or another useful robot position. Avoid retrying every tick or rotating forever.
Also agree how mission waits for processed observations and requests another
baseline when confidence is still insufficient. That execution policy is not
part of the current Path interface and has not been implemented in coverage.

Coverage's corrected ordering keeps the existing views and their predicted
clock deadlines. It changes no mission parameters or topic types. It cannot
guarantee that a camera frame is processed during transit or that fusion reaches
the report threshold. The different GPUs/RTFs of the two runs and their changed
early route mean this single pair does not isolate the cause of the score loss.
