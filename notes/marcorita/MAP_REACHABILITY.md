# Shared reachability for inspection

Branch: `cobertura/mapa-alcanzable`, based on development `d7e8057`.
Publication authorized on 2026-10-27; mission adoption remains a separate step.

## Problem and scope

A free inspection position with a clear view of a candidate can still be in a
disconnected pocket. Mission must discard unreachable positions **before**
choosing the closest one. More ring angles alone can select a disconnected
point that is closer in Euclidean distance.

The existing coverage map model now lives in `equipo_jar.mapa`. Coverage uses
the same implementation through its old imports. Its selected positions,
orientations and ordering must stay unchanged in this branch. PR #16 and its
unresolved server behaviour remain separate.

No changes to `inspeccion.py`, `mision.py`, shared parameters, topics or messages.
Legacy `is_free`, `cell_value` and `line_is_clear` retain their behaviour.

## API for Bautista

```python
from equipo_jar.mapa import Grid

grid = Grid.from_occupancy_grid(map_message)
reached = grid.reachable_from(
    robot_x, robot_y, radius=transit_radius, threshold=free_threshold)

# Inside the existing inspection candidate loop, before picking the best:
if grid.cell(vx, vy) not in reached:
    continue
```

- Input is the actual `/map` message, with `header.frame_id == 'map'` and a
  finite normalized planar origin quaternion. No ROS imports are needed here.
- `robot_x`, `robot_y`, candidate positions and the radius are in metres.
  `grid.cell(x, y)` returns an integer cell pair or None outside the map.
- `radius` is a **transit** disk including its safety margin. Coverage currently
  uses `robot_radius + safety_margin` (0.21 m). This is not the 0.25 m inspection
  stop clearance or the 0.42 m plus uncertainty box exclusion distance.
  Mission should use an agreed parameter, not a hardcoded practice value.
- `threshold` is explicit; known cells with `0 <= value <= threshold` are free.
  Coverage uses 25. Unknown values and map edges block transit. The legacy
  point helpers reject values >= 50, so ambiguous cells can pass the old stop
  check and still fail this stricter connectivity check.
- The result is a frozenset of connected safe cells. It uses the existing
  conservative circular inflation against full cell squares, followed by
  four-neighbour connectivity. Diagonal corner contacts do not join rooms.
- An outside, occupied or insufficiently clear robot cell raises ValueError.
  Handle that explicitly and skip/defer the inspection; do not silently fall
  back to unfiltered candidates or snap the pose across a wall.
- Optional `cancel` is an Event-like object; cancellation raises CancelledError
  during inflation and connectivity, without returning a partial component.

Build a Grid once per map snapshot; it copies the map data and origin. Compute
the component once per inspection search, outside the angle loop. If caching
components later, invalidate them for map/radius/threshold changes and ensure
the current robot cell belongs to that component.

Keep the existing candidate stop-clearance and line-of-sight checks, then filter
by connected-cell membership, then choose the closest remaining point. Do not
mark an inspection as completed just because no candidate survives. Increasing
the number of angles is a separate decision; this branch does not change it.

## Limits

This is a static-map prefilter using cell centres and a circular transit model.
It does not prove that Nav2 can execute the goal, that the measured robot pose
is correct, or that the camera has observed the box. It does not include boxes,
distractors or decoys absent from `/map`. Keep Nav2's live obstacle avoidance
and mission's box filtering. Different disc and rectangular footprints may
produce different reachable regions.

The recorded pocket replay remains a static diagnostic: its map comes from the
recorded corrected coverage run, while Bautista's rounded centre/robot pose
come from his unrecorded run. Do not claim an exact replay of his run or a new
successful inspection.

## Verification

- Focused geometry/planning suites: 88 tests passed, including generated narrow
  doorways, disconnected rooms, rotated origins, unknown cells, bad starts,
  cancellation and old coverage imports.
- Full `tools/verificar.sh --template`: green; 137 outer tests, successful
  build/static checks and map-wait/interface audit with zero violations.
- Differential comparison: all 18 plans, including diagnostics/metrics and
  connected components, match development exactly. In the recorded pocket map,
  exterior 174/175-degree ring points remain connected and the interior
  285-degree point is rejected. Results are kept in ignored
  `build/verificacion/cobertura/mapa-compartido/` and recorded in STATUS.
- No new simulator/server run; no camera, score or real navigation claim.

## Next integration check

Bautista can connect the filter in his own inspection change after this helper
is shared. Test an unreachable nearest point plus a farther reachable point,
an invalid start, and all candidates rejected. Separately measure how mission's
new candidate-box zones affect coverage; the `[]` metrics compatibility fix
does not evaluate nonempty zones or the actual turn-in-place goal offsets.
