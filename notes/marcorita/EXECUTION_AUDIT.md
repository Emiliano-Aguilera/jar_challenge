# Coverage execution audit

Branch `cobertura/auditoria-robusta`, based on development `93bdd59`.
Updated through `90a4fca` after mission PRs #31 and #32: selection/command APIs
and modeled waits are unchanged; explicit records can include optional recovery
counts. Marco authorized publication of this small audit PR on 2026-10-28.
Runtime coverage geometry/parameters and other modules are unchanged.

## What changed

- `metricas_cobertura.mission_indices` accepts optional fixed candidate zones;
  the existing no-box case stays the default. This does not replay dynamic detections.
- `ejecucion_cobertura.predict_goals` calls mission's actual selector and
  goal-position substitution. It records shifted commands and distinguishes
  box exclusions from goal-identity exclusions. It uses the current startup
  and settle parameters; planner observation_time is not an actual dwell.
- `evaluar_presupuesto.py` now adds `mission_execution` to JSON/CSV and uses
  that commanded-position/time proxy in its human report. Historical intended
  and identity-only mission fields stay available for old comparisons.
- `audit_records` consumes explicit successful/failed/canceled goal records.
  Failed/canceled trips keep their elapsed time but receive no observation.
  It flags invalid localization cells and computes visibility at time budgets.
- `auditar_registro_cobertura.py` requires one stated clock, /map time origin,
  map frame, pose provenance, explicit results and camera transform. It does
  not silently turn planned poses or missing statuses into measured successes.

All visibility remains a planar cell-centre prediction. Successful navigation
is not an image acknowledgement. Commanded arrivals are not measured arrival
poses. In-transit observations, inspection detours, dynamic cancellations and
closed-loop detection effects need recorded data; fixed zones are only stress
scenarios. Current scripts make those limits explicit.

## Noise sensitivity

The record auditor intersects visibility over explicit position/yaw samples.
The default yaw samples are -10/0/+10 degrees. These are sensitivity tests,
not calibrated hardware noise or a guarantee over intermediate errors.
Separate local heading-optimization experiments remain unpublished; this PR
does not change viewpoints or activate an optimization search.

## Reproduce

Use an environment with ROS Humble for the adapter tests; geometry scripts
themselves do not import ROS. Keep local test domain separate from a live run.

```bash
export PYTHONPATH="$PWD/src/equipo_jar:$PWD/pruebas:$PYTHONPATH"
python3 -m pytest pruebas/test_ejecucion_cobertura.py pruebas/test_metricas_cobertura.py -q

SIM=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)
python3 pruebas/evaluar_presupuesto.py --simulator "$SIM" \
  --output build/verificacion/cobertura/auditoria-robusta/presupuesto-integrado --diagnose none
```

Keep generated practice plans under ignored build/. Do not put maps, worlds
or hardcoded practice routes into src/ or delivery parameters.

External goal-record input is an object with frame_id `map`, time_origin `/map`,
clock `simulated` or `wall`, pose_source `localized`, `commanded` or `predicted`,
a nonempty source label, camera `{x,y,yaw}`, and a goals array. Every goal must
contain index, elapsed seconds, pose `{x,y,yaw}`, and status `succeeded`, `failed`
or `canceled`. Optional recoveries is a nonnegative integer; absent counts
remain unknown. Do not mix clocks or pose sources in one file. A recovery
count does not itself determine success or cancellation; use the actual status.

```bash
python3 pruebas/auditar_registro_cobertura.py --records <goals.json> \
  --map "$SIM/maps/<practice-map>.yaml" \
  --output build/verificacion/cobertura/auditoria-robusta/recorded.json --dense
```

## Remaining validation

Use current recorded mission data and the Workshop 10 real-robot bag before
claiming a detection or score improvement. No new manual server run is
authorized for this task; three prior authorized runs were consumed. PR #16
remains a separate draft with unresolved execution evidence. Publication of
this new branch has not been authorized. See ORGANIZER_TIPS.md for team actions.
