# Supplementary views (2026-11-01)

Marco authorized implementing the pure coverage provider and geometry tests.
Branch `cobertura/repaso-oclusiones` starts at development `a785697`.
No mission, shared parameters, initial planner, launch, tools or workflows are
changed. Marco subsequently authorized commit/push and a draft PR to development
for Bautista's integration review. No merge or manual server run is authorized.
PR #36 and #16 remain separate drafts.

The requested provider is `cobertura_repaso.propose_extra_views`. Mission owns
history, freshness, time reserve and actions. The attached Bautista design is a
proposal, not execution authority over mission. History records are structural
protocols here, not a runtime import from the not-yet-existing mission module.
His frozen records with `pose/at_s` and `pose/outcome` fit without conversion.
Two optional keyword arguments expose mission's existing equivalent-view
tolerances (defaults 0.02 m / 0.10 rad); pass current values at integration.

Initial goals remain unchanged. Extra goals are independent alternatives ranked
by estimated new visibility / travel-and-look cost, not an extra route to execute
blindly. Execute one, dwell and replan. Unknown/outside optical data remains
unresolved. Pending targets can remain even when no reachable useful view exists.
Static safety is projected onto navigation cells; inflation is never inflated
again. Physical occupancy projection uses whole-cell overlap across origins and
resolutions, not center-only resampling. Clearance uses the physical grid.

Before integration Bautista must supply fresh, same-map snapshots and actual
pose history, never the consumed-goal ledger. A failed requested goal grants no
visibility. Camera opportunities are not image acknowledgements. Recomputing
with current occluders can restore a shadow to pending; it cannot reconstruct
past geometry if an obstacle disappears from the costmap. This limitation needs
explicit handling/acceptance with mission before runtime enablement.

## Contract for mission (#58)

Reviewed draft #58 at `9aae681`. Its actual frozen history records produce
exactly the same provider output as the fixture records. Import concrete
records from `historial_vistas`; the provider's Protocol names are annotations,
not replacement constructors. No change to his record classes is needed.

- `map_grid` is raw `/map`; `live_grid` is the full map-frame Nav2 costmap
  scaled to -1..100 (unknown, 99 inscribed, 100 lethal), not raw 0..255 costs.
- Robot/start/observations/exclusions are in map metres/radians. History time is
  seconds since the same `/map` epoch. Mission must discard incompatible history
  after map replacement or a pose-history correction; freshness is his check.
- `failed_shift_m` excludes failed/rejected/cancelled requested positions for
  every yaw. Arrived attempts alone grant no credit; measured history does.
- Pass existing `same_point_distance_m` / `same_point_angle_rad` as optional
  keywords if mission's live values differ from 0.02 m / 0.10 rad.
- `excluded` is `(x,y,radius)` per candidate. The exact radius boundary is
  excluded. Recheck current zones and live endpoint before dispatch.
- `ExtraView.gain` is a count of newly predicted visible sampled cells. Each
  alternative is scored independently; execute one, dwell and replan. Zero
  proposals with nonzero pending_count means unresolved, not fully observed.
- `limit` bounds alternatives, also capped by Settings.max_views. Coarse and
  fine stages share Settings.max_positions; there is no exhaustive guarantee.
- Construct Settings explicitly from mission's live parameters. In particular
  extra_transit_radius_m is a TOTAL radius: use robot_radius equal to that value
  and safety_margin=0, or split 0.18+0.03; do not accidentally make 0.21+0.03.
  Pass range/FOV/headings/spacing/counts/clearances/look time and named speed
  estimates; the provider never reads another ROS node's parameter state.
- Invalid/unsafe inputs raise ValueError; cancellation raises CancelledError.
  Catch at the worker boundary and preserve bounded return. No fabricated snap.
- Both full-grid physical projection and the static safety projection are
  conservative over cell areas, which can reject tight passages more than a
  center lookup would. This changes no Nav2 inflation/footprint parameter.

## Local evidence

Source SHA256: `557dd8b37a300b1ffb83fb24defe05415c37caf76c2d9171773aab27f2f7b207`.
58 geometry tests pass, including a polygon-clipping oracle independent of SAT,
different resolutions/rotations, unknown/partial grids, failed/arrived attempts,
candidate safety zones, continuous observation poses and cancellation. Fine
recovery finds useful views when the sole coarse position is already observed.
Closed-room geometry returns no goals and preserves pending samples.

`pruebas/medir_repaso.py` records three fresh-grid repetitions per case in
ignored `build/verificacion/cobertura/repaso/cpu.json`:

| Generated case | Median seconds |
| --- | --- |
| Room with cylinder | 0.612 |
| No actual history | 1.155 |
| 1200 distinct actual poses | 0.986 |
| Live grid at 3 cm | 0.929 |
| Shifted/rotated live grid | 0.831 |
| Closed room | 0.864 |

All 18 measured calls finish in 0.585..1.216 s on local WSL. This is not a
worst-case guarantee for arbitrary maps or the competition PC. In a generated
ideal-reached-pose loop, the first proposed view sees two previously shadowed
samples. There is no camera/detector, Nav2 motion or new report in that test.

Full verifier reports dependencies/structure/params/static/build, 246 outer
package tests, 17 audited nodes with zero problems, and map waiting with zero
pre-map motion/reports/violations. System OpenCV 4.5.4 is used; no pip package or
new dependency. The final explicit subprocess receipt confirms exit code 0 and
the same source hash (`verificador-resultado.json`, `verificador-cierre.log`).
The first driver's nonzero status despite its green log remains preserved as
preliminary evidence, superseded by this independently captured final result.

Integration and the recorded wide-door/narrow-door/easy-map comparisons are
still required before runtime enablement. The default mission flag stays off.
Publication is authorized only as a draft PR; keep it open for owner review.
No manual simulation or merge is authorized here.
