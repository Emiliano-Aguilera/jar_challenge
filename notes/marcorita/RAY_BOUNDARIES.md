# Visibility at cell boundaries

Local branch cobertura/rayos-bordes, based on development 53ecc6b. Marco
authorized fixing/verifying the ray logic locally, then requested publication
and merge of this verified fix after green CI and no pending issues. That
authorization is specific to this branch. The separate mission PR #36 is not
changed by this task and remains reserved for Bautista.

## Problem and behavior

Grid.visible formerly stopped when its integer cell matched floor(endpoint).
When an endpoint lies on a cell edge or corner, that floor cell can be touched
without being entered. A diagonal crossing could pass it and keep walking
outside the map, reporting a blocked ray in an entirely empty grid. Reversing
the same segment could change the answer. Rays along an occupied grid edge
could also fail to inspect the other strip.

The corrected model is a closed segment against closed occupied/unknown cell
squares. It checks both endpoints against all touched cells, then walks only
the cells entered by the open segment. At an exact endpoint boundary the final
entered cell is chosen toward the segment interior, rather than blindly using
floor(endpoint). A line along a grid edge checks both strips. Corner crossings
still check side cells. Nonfinite/outside endpoints and contacts with the map
edge are blocked. Numeric snapping is limited to 1e-12 cell units to absorb
planar-transform roundoff, not a tunable geometric wall margin.

An earlier implementation checked the segment parameter at each crossing;
it passed geometry tests but added planning CPU. It was replaced by the
interior-cell termination described above. The earlier comparison is retained
under ignored planes-prototipo.json; final results use planes.json and source
hashes. Do not quote prototype timings as final performance.

## Scope and impact on mission

- The Grid.visible signature and import paths remain unchanged.
- Grid.cell, inflation, connectivity and the legacy is_free/line_is_clear
  helpers are unchanged. No shared parameter or message change is needed.
- The current runtime callers are the two coverage visibility functions in
  cobertura_plan.py. Mission's viewpoint_for still calls line_is_clear; this
  task does not change its inspection search, waits or report policy.
- Future surface-view experiments benefit from reliable endpoint handling,
  but this does not make their 2D visibility model a detector. The recorded
  pocket's two positive observations remain unexplained by that model.
- This is not a Nav2 collision fix or a demonstrated score improvement.

## Verification

- Focused geometry: 64 tests passed, including the new ray suite and a
  near-corner numeric regression found during final review. The last guard
  prevents a tied crossing from stepping past the closed endpoint.
- Independent segment/square slab oracle: 1,920 generated ray pairs on 12
  obstacle/unknown maps, checked in both directions under four origins/yaws
  (15,360 visibility assertions). All match.
- Exhaustive half-cell endpoints on three small obstacle maps: 7,203 directed
  pairs under two origin/resolution combinations (14,406 assertions). All match.
- Fixed regressions cover empty-map endpoint direction, zero-length rays,
  occupied/unknown endpoint touches, both strips of grid-aligned rays, genuine
  near-boundary free rays, threshold cache changes and invalid endpoints.
- The new suite runs through the existing colcon map-test wrapper; geometry
  fixtures stay in pruebas/, outside delivered runtime source.
- Final 18-case comparison: all serialized plans are identical to
  development. Rechecking both sets of routes with the corrected dense model
  loses/gains no cells. Dense counts also match the old visibility model in
  these cases. Source hash matches the saved run; --verify-saved rechecks it.
- Three alternating CPU pairs on the final code, with a fresh
  Grid cache for every call and exact Plan equality on each repetition:

  | Map | Old median s | New median s | Change |
  | --- | --- | --- | --- |
  | laberinto_1 | 6.081 | 5.963 | -1.95% |
  | laberinto_simple | 5.209 | 5.551 | +6.57% |
  | maze_1_6x5 | 2.474 | 2.303 | -6.94% |

  These are descriptive samples on one machine, not statistical significance
  or robot timing. This is a correctness fix, not a speed optimization. The
  maximum median cost in this final sample is about 0.342 s. The earlier pass
  had costs up to 0.402 s; it is retained in tiempos-antes-esquina-numerica.json.
  Variability prevents claiming a speedup. The separate CPU optimization has
  deliberately not been mixed in.
- Final complete verifier passed: structure, params, static checks, build,
  all 213 package tests, interface audit (17 team nodes, zero problems), and
  map-wait checks (no motion/reports before the map, zero violations).
  The final comparison/timing/verifier pipeline exited 0. Evidence stays under
  ignored build/verificacion/cobertura/rayos/. No server run was requested.

The comparisons use the six installed practice maps and three starts per map.
They compare complete serialized plans and recheck both old/new plans under
the corrected dense visibility model. Any loss caused by removing an old false
visibility claim must be distinguished from losing a valid view in a new plan.
Timing is only a local CPU measurement, not robot latency or competition score.
