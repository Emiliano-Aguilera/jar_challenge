# Coverage integration

Current state reviewed against development `c01ccdc` on 2026-10-22.
Follow-up proposals below do not change the agreed interface.
Current output stays `/jar/plan_cobertura`, `nav_msgs/Path`, frame `map`.

## Mission

Mission already owns execution progress: it remembers finished goals and takes
the first pending pose, using 0.30 m and 0.5 rad tolerances. Failed goals count as
finished to avoid endless retries. It also owns visits to doubtful candidates.
This supports a first integration with coverage's initial route, without a new
progress topic. Coverage does not yet consume navigation or observation results.

Mission starts returning at 480 s and forces finishing at 570 s. Some complete
coverage estimates exceed the search allowance, so mission's deadline must stay
active. A geometric plan is not a guarantee of executing all poses in time.

Information to agree for future adaptive coverage, before adding a message/topic:

- Which plan and point the result belongs to: plan version and goal index,
  or another stable identifier agreed by both owners.
- Active goal acceptance and release. Keep the active goal stable until its
  result; a replacement Path must not make mission restart from its first pose.
- Separate reached, observed, failed, cancelled and timed-out results.
  Reaching a pose alone does not prove that perception processed a usable image.
- For observation evidence: capture time and the actual robot pose/heading,
  plus a way to know perception was working. Candidate count zero can mean
  either no red box or a broken detector; it is not observation evidence.
- Failure policy: retry, a different view, defer, or abandon. A failed point
  must never be counted as observed. Mission owns the real clock and return policy.
- Duplicate or delayed feedback must be idempotent and must not apply to a
  different plan version.

Use the output as an initial route proposal. Coverage never publishes `/done`.
Mission currently returns when the received plan is exhausted and no doubtful
candidate is pending; that is an execution policy, not proof that all boxes were
found. Live parameter changes can replace the route; coordinate them at a goal
boundary and check mission's finished-point tolerances for newly sampled views.

Shared `mapa.py` geometry is a separate change (PR #6 starts with origin rotation).
Coverage currently retains its own tested Grid implementation. Align occupancy,
unknown-cell, footprint and ray-corner policies before migrating to shared code.

## Navigation and localization

- Confirm `base_footprint`, robot footprint, safety margin and goal/yaw tolerances.
- The current circle plus margin can disconnect narrow passages that a
  rectangular robot could pass in a particular orientation. Do not lower
  clearance just to improve a coverage percentage; validate it with navigation.
- Agree how to expose a newly blocked goal and whether coverage can consult
  a costmap or a path-cost query. No new input is implemented yet.
- TF age is checked; TF alone does not express localization confidence.
  Agree what happens during an AMCL jump or loss of localization.
- The test identity `map → odom` is not a substitute for real localization.

## Perception

- Measure useful detection distance and field of view on boxes, with the
  actual camera, LiDAR and detector. Current range is an initial parameter.
- Confirm observation dwell, blur while turning, partial occlusion and
  the camera/target heights. The planner uses only a horizontal projection.
- Fusion needs robot positions separated by 0.30 m to reduce its single-view
  uncertainty floor. Mission's inspection shift is 0.40 m. Rotation alone is not
  confirmation. The planner's 1 s dwell is a cost estimate, not an implemented
  mission wait; agree this with perception and mission before relying on it.
- Confirm physical box dimensions before filtering possible target centers
  based on box size. Simulator dimensions alone are not a confirmed rule.

## Next end-to-end check

Once execution is connected, measure actual time, goal success, processed
images, candidate confirmation and collision observations. Compare against
the current offline plan with the same starts and settings. A green build or
high predicted floor coverage does not measure competition performance.
