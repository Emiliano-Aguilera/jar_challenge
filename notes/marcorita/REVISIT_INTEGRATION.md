# Observed-view fallback integration

Branch cobertura/puntos-ciegos, base be2ab90. Marco explicitly authorized the
mission integration, publication as a draft PR and one recorded 600 s
laberinto_1 run. The previous CPU optimization remains in its separate checkout.

## Behavior

The existing ring inspection runs first, unchanged. Only if it returns no
view, mission can revisit a recent measurement pose associated with the same
candidate. cobertura_revisita.py requires current-map evidence, finite values,
continuous-position disk clearance and connectivity with the existing transit
radius. It preserves the measured yaw; it does not aim at a potentially hidden
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

Association, observation clearance, transit radius and occupancy threshold use
the existing mission parameters. Navigation remains responsible for live
obstacles and the actual path. The static-map check is not a collision guarantee.

## Evidence

- Focused helper/integration suite: 33 passed.
- Full local verifier: build, 170 package tests, structure/params/static,
  interface audit (15 team nodes, zero problems), and no motion/reports before
  the map. Isolated ROS domain 177, pinned OpenCV.
- Recorded-input mission replay: the original ring returns None; fallback
  sends a recent safe measurement pose; without new evidence it times out
  without marking inspection. Navigation was mocked in this replay.
- Separate 600 s server run is authorized but pending dispatch/result. Keep the
  PR draft while there are unresolved behavior issues. PR #16 stays separate.

See BLIND_SPOTS.md for the corrected diagnosis and ignored recording evidence.
