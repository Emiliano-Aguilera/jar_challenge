# Rotated-map inspection test

Marco requested fixing and merging our test so Bautista's PR #26 can pass.
He explicitly preferred using Bautista's final API without a compatibility layer.

Branch: `cobertura/test-inspeccion-alcanzable`.
PR destination: `mision/visitas-alcanzables`, based on `9b9fc1b`.
This exception keeps the fix together with the API change: merge the test into
mission's branch, then Bautista can finish his own PR to development.

The rotated-map regression now supplies the required `cell_grid` and `reachable`
arguments directly. It builds the Grid from the actual rotated test message and
computes its real safe connected component. The message fixture includes its map
frame. The expected position/orientation assertions remain unchanged. No test is
skipped and no all-cells connectivity placeholder is used.

This fix changes no production source or mission parameter. The 0.21 m radius
belongs to the generated open-map test fixture; it does not approve or modify
mission's pending 0.15 m transit-radius decision.

Validation: all 147 package tests passed on the mission branch plus this direct
test fix, including the nested map/coverage suites. Used ROS domain 177 to keep
the user's manual domain-77 simulator separate. No new manual simulator run.
