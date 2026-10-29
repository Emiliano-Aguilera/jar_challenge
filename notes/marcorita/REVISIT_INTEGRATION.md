# Observed-view fallback integration

Branch cobertura/puntos-ciegos, base be2ab90. Marco explicitly authorized the
mission integration, publication as a draft PR and one recorded 600 s
laberinto_1 run. The previous CPU optimization remains in its separate checkout.

## Behavior

The existing ring inspection runs first, unchanged. Only if it returns no
view, mission can revisit a recent measurement pose associated with the same
candidate. cobertura_revisita.py requires current-map evidence, finite values,
continuous-position disk clearance and connectivity with the existing transit
radius. If the measured position fails observation clearance, the helper can
offer a safe nearby cell centre within both revisit_adjustment_m and one map
cell. The hint retains the original evidence separately: adjusted visibility
is not measured. It preserves the measured yaw; it does not aim at a hidden
box centre or invent an arena coordinate.

Mission subscribes to the existing /jar/observaciones topic and obtains observer
TF at the observation's header stamp, not latest TF. That is the LiDAR stamp
used by the observation message, not necessarily the exact camera image stamp.
The helper returns a navigation hint, not proof of future camera visibility.
History is bounded and invalidated when the map changes. No new message or
topic was introduced.

After a fallback goal succeeds, mission waits briefly for new evidence. It
marks that fallback inspection complete only if the candidate gains observations
and reaches the existing min_observations count. Goal failure, elapsed time,
stale/revised-map evidence or insufficient new observations do not grant
inspection. Failures consume the existing bounded attempt policy. Global
report-count/deadline returns preempt the wait; an empty exploration plan alone
cannot cut it short. The wait applies live and rejects values above 5 s.

This does not relax report covariance, observation thresholds or fusion's
0.30 m baseline floor. Repeating a view may strengthen a weak candidate's
retention without making it eligible for an immediate report. No score gain
is established by the replay or unit tests.

## Parameters declared by mission

No shared YAML was edited. Defaults are declared/read live by mission:

| Parameter | Default | Meaning |
| --- | --- | --- |
| revisit_enabled | true | Allow fallback only after the normal inspection fails |
| revisit_max_age_s | 10.0 | Maximum measurement age, using the same ROS clock |
| revisit_min_shift_m | 0.05 | Minimum shift for repeat observation, not a triangulation promise |
| revisit_wait_s | 2.0 | Wait after successful arrival; valid range 0 through 5 s |
| revisit_max_records | 128 | Maximum retained measurement records |
| revisit_adjustment_m | 0.05 | Maximum goal adjustment, also capped to one map cell; zero disables adjustment |

Association, observation clearance, transit radius and occupancy threshold use
the existing mission parameters. Navigation remains responsible for live
obstacles and the actual path. The static-map check is not a collision guarantee.

## Evidence

- Initial helper/integration suite: 33 passed; initial full verifier passed
  170 package tests. Follow-up adjustment adds regression cases for continuous
  clearance, small corrections, connectivity and finer map resolutions.
- Full local verifier covers build, package tests, structure/params/static,
  interface audit (15 team nodes, zero problems), and no motion/reports before
  the map. Isolated ROS domain 177, pinned OpenCV.
- Final follow-up: 38 focused tests passed, then the full verifier passed
  all 171 package tests, build, static/parameter checks, interface audit and
  map-wait check. No motion or reports before the map; zero violations.
- Recorded-input mission replay: the original ring returns None; fallback
  sends a recent safe measurement pose; without new evidence it times out
  without marking inspection. Navigation was mocked in this replay.
- The one authorized recorded 600 s server job completed on ecdfb2f:
  [40778333572](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40778333572),
  batch 20261028-191404-ecdfb2f. Result: 300 points, about 394 s until done,
  RTF 0.974, zero protocol violations, collisions or close calls. The fallback
  did not activate; the second report still came after returning. This does
  not demonstrate an improvement.
- Actual TF replay in recorded arrival order shows both measured observers
  were available but failed the 0.25 m continuous clearance. With adjustment
  disabled the helper returns no hint. With a 0.05 m budget it offers a goal
  0.02738 m from the evidence pose, retaining yaw and 0.25 m clearance.
  The real observation callback and mission selector send that goal; with
  mocked arrival and no fresh observations, inspection is not credited.
- The adjusted goal has not been driven or observed in a new server run.
  Its future image visibility and score effect remain untested. No further
  server run is authorized. PR #36 stays draft for Bautista to review and
  merge himself, per Marco's explicit instruction. Do not merge it even if
  CI is green. PR #16 stays separate.

See BLIND_SPOTS.md for the corrected diagnosis and ignored recording evidence.
