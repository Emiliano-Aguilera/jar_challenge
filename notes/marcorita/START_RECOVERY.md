# Initial coverage near a wall — 2026-11-02

## Scope

Marco requested option A after Hans reported a known-free starting cell outside
the 0.21 m transit mask and a failure latch that retried only after a cell change.
Branch cobertura/arranque-cercano starts at origin/development 60ddd13. Earlier
uncommitted access diagnostics/notes are preserved and are not part of this fix.
Marco subsequently authorized publishing this branch and a draft PR for review.
Do not merge or start new manual server runs. Mission/navigation are unchanged.

## Behavior

- Preserve the original plan path for a start cell already in the safe mask.
- For a known-free start, choose the nearest visible safe cell center within
  start_max_shift_m (0.20 m). Closed center-line visibility rejects wall/unknown
  crossings. Reject occupied, unknown, outside, too-distant and unconnected
  local starts; 0 disables the fallback. Shared mapa.py reachability stays strict.
- This is a planning anchor, not a corrected localization or a commanded escape.
  TF, mission's actual start and Nav2's physical starting pose are unchanged.
  The center-line check is NOT a whole-body swept-path proof. Nav2 still checks
  and executes the approach with its footprint/current costmaps and can fail.
- All observation/transit margins remain unchanged. Existing safe-start plans
  retain their views, order, coverage, metrics and diagnostic legs exactly.
- On recovery, select a first view at least start_min_goal_distance_m (0.25 m)
  from actual TF and preserve that condition after ordering. Keep this value
  above mission's turn_in_place_m (currently 0.20 m). A Nav2 final heading does
  not force translation before rotation; this only avoids mission replacing
  the requested position with the current position. No suitable first view
  means planning failure, not an empty successful plan or an unsafe spin.
- Keep all selected views when restoring the first-goal condition after order
  refinement. Include the straight local connector estimate in outbound/return
  distance, duration and first-visibility timing; diagnostic legs begin at the
  safe anchor. These are estimates, not an executed itinerary.
- Retry failures with current TF after retry_interval_s (2 s, monotonic wall
  clock), even in the same cell. A new map/configuration/cell can retry earlier.
  One worker at a time; stale-generation failures do not poison the new state.
- Recheck the actual start/connector/first-goal distance before publishing a
  recovery plan: a small TF correction can invalidate it below normal tolerances.
- Three tunables are declared and live-adjustable, with YAML entries only under
  cobertura. No nav, mission, shared map API, dependencies or workflow changes.

## Evidence and remaining validation

- Final focused run: 112 passed. Full tools/verificar.sh --template exits 0:
  257 package tests, build/structure/params/static/dependencies green, 18 audited
  team nodes with zero interface problems, zero pre-map motion/reports and no
  protocol violations. The final receipt verifies unchanged source hashes.
- Differential comparison against development: all 18 existing safe-start
  plans, including metrics and diagnostics, are identical. Only optional new
  Plan.start_anchor metadata is absent in the old record (None in the new).
- Organizer mini map, hypothetical TF at (0,0,0): old planner raises the reported
  safe-space error; new anchor is (0.112,0.004), shift about 0.1121 m. It produces
  18 views, predicts 334/532 sampled targets and retains 0.24 m goal clearance.
  First view is about 0.421 m from the hypothetical actual pose. No victim truth,
  bag localization, camera processing, driving or score is inferred from this.
- Ignored evidence: build/verificacion/cobertura/arranque/comparacion.json,
  focalizadas-final.log, verificador-final.log and verificacion-final.json.
- The first full verifier is preliminary because an additional adapter guard
  was added during it. Use the later explicit exit-code/hash receipt for closure.
- Need a recorded near-wall start comparison to validate physical departure,
  navigation success and time. No new server job is authorized. This patch
  removes the coverage deadlock; it is not a guarantee of physical recovery.

Final source SHA256: cobertura_plan.py 8cfc476e1d21300ef5b1bc239e3719326c705cfb89df1280b401ac9e16a14e49;
cobertura.py 1dc91c1f594a706b78ab94e6b4cb9968e896d8b913fbf397a48c27acd8b81321.

## Handoff

Do not use the smaller-radius option as an automatic fallback: it alters transit
throughout the plan. Bautista need not change mission for the current defaults,
but should retain/review the first-goal distance versus his in-place threshold.
The narrow-passage controller remains his separate work. Keep the publication
in draft and do not merge anything under this task's authorization.
