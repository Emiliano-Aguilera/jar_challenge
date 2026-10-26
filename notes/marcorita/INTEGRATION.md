# Coverage integration

Current base is development `3adfaeb`; local work reviewed on 2026-10-26.
Follow-up proposals below do not change the agreed interface.
Current output stays `/jar/plan_cobertura`, `nav_msgs/Path`, frame `map`.

## Observation goals and navigation

The local candidate separates transit clearance (0.21 m) from observation
clearance (minimum 0.24 m, preferred 0.30 m). Tighter recovery points are used
only after exhausting preferred witnesses. The current Nav2 chassis with
0.02 m padding has a farthest corner about 0.2341 m from base_footprint.
Recheck the minimum if that geometry changes. No Nav2 settings changed.

Mission does not need an interface change for this: the plan still contains
positions/headings, but their positions, number and order can change. Coverage
identity remains mission's 0.02 m / 0.1 rad rule. Nav2's actual arrival tolerance
(0.10 m / 0.25 rad) is a different rule and can shift the real viewing pose.
The clearance model does not certify localization/map errors or real arrival.
Do not count reaching a goal as a processed camera observation.

## Concrete follow-up with Bautista

The recorded no-view inspection failure and the proposed bounded-retry
agreement are in [INSPECTION_REPLAY.md](INSPECTION_REPLAY.md). This is separate
from the candidate-obstacle proposal below. Neither changes shared interfaces
without the affected owners' agreement.

Candidate boxes are absent from the static practice map. Mission already
receives `/jar/candidatas`; agree on a temporary goal-exclusion rule there
before adding coverage subscriptions or changing shared interfaces.

- Check the robot footprint against the candidate box and its uncertainty.
  A fixed 0.30 m center-distance rule does not cover every orientation/corner.
- Keep an excluded goal pending; excluding it is neither success nor observed
  coverage. Reconsider it as the candidate estimate changes and bound retries
  with mission's existing deadline. False candidates must not erase areas.
- Record first candidate, inspection, navigation result and report times.
  A late report can come from confirmation/uncertainty, not just a late view.

These are proposals, not implemented mission changes. Bautista owns selection
and execution. Hans should review arrival behavior at the changed wall goals;
full collision/startup instrumentation and real speed calibration remain open.

## Mission

Mission already owns execution progress: it remembers finished goals and takes
the first pending pose, using 0.02 m and 0.1 rad identity tolerances. It compares
requested plan coordinates, not the robot's measured arrival pose. These values
distinguish the current grid centers and headings; recheck finer map resolutions
or smaller heading intervals. Failed goals count as
finished to avoid endless retries. It also owns visits to doubtful candidates.
This supports a first integration with coverage's initial route, without a new
progress topic. Coverage does not yet consume navigation or observation results.

Mission starts returning at 480 s and forces finishing at 570 s. Some complete
coverage estimates exceed the search allowance, so mission's deadline must stay
active. A geometric plan is not a guarantee of executing all poses in time.
Mission now also waits 8 s initially and cancels trips that stay still or exceed
their timeout. Budget evaluation assumes TF is available immediately and every
goal succeeds; real delays and inspections still need measurement.

Local observation-order refinement retains every selected pose and heading.
It uses `order_check_interval_s` and `order_start_delay_s` to protect sampled
coverage on a predicted clock grid. The delay currently estimates the 8 s
startup wait, but does not read or control mission's actual clock. Recheck it
if startup wait or measured planning/TF delays change. This adds no interface
or execution-progress feedback. Dense visibility at the evaluated budgets is
checked offline; runtime checks are sampled, with no between-check guarantee.

The offline evaluator found useful goals grouped by the old mission tolerances
in four of 18 cases. Mission PR #9 resolves these losses: replaying all 18 saved
plans selects every intended view in order. See `MISSION_COMPATIBILITY.md`.
This identity correction does not validate actual observation or navigation.

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

The local route-ordering candidate based on merged PR #15 retains every pose and heading;
only their visiting order can change. No schema/topic or mission change is
required. The real mission selector accepts all intended views in all 18
offline cases. The 0.30 m ordering baseline is a geometric heuristic, not proof
of fusion confirmation at the requested or actual pose. Mission inspections,
dynamic obstacles and localization corrections can change real execution.
The occupied-candidate goal filter discussed with Bautista is still pending;
this ordering work neither implements it nor marks a failed goal as observed.

The first local ordering handoff attempt received the map about 54 s before
connected robot TF. Fake navigation does not move the robot, and the judge
ended at its 60 s inactivity limit. The communication fixture now waits for
fresh map/base/camera TF before starting that judge; the failed attempt remains
recorded. This prepares the test, not the official run. Actual TF readiness and
startup delay under the competition launch still need validation with localization.

Once execution is connected, measure actual time, goal success, processed
images, candidate confirmation and collision observations. Compare against
the current offline plan with the same starts and settings. A green build or
high predicted floor coverage does not measure competition performance.
