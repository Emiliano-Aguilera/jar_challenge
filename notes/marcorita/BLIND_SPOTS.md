# Blind-spot investigation (2026-10-28)

Branch cobertura/puntos-ciegos starts at development be2ab90 in a separate
checkout. The unpublished exact CPU change remains untouched in
jar_challenge-mapa / cobertura/orden-eficiente. Marco requested correcting the
laberinto_1 issue reported by Hans. Marco subsequently authorized the mission
fallback integration, publication as a draft PR and one recorded 600 s run.

## Corrected diagnosis

The approximate candidate coordinate is not the physical box center. Static
point-center visibility alone cannot establish a missing camera viewpoint.
The recorded map and local occupancy cells match exactly; resolution differs
only by ROS float32 serialization. Use the recorded map in further replay.

The known practice box's perimeter can be visible from two nominal plan views,
even though its center is hidden. A 72-heading safe-cell search finds additional
surface-view opportunities. All practice coordinates stay in ignored diagnostic
output or scripts outside the repository, never runtime source.

Hans run 40751918859, batch 20261028-161051-f9977bb, and recorded mission run
40619690019, batch 20261027-231920-93bdd59, both sent the two relevant goals and
produced a candidate with two observations during exploration. The latter
explicitly logs it near the true box, then cannot find an inspection point.
Do not describe this as zero detection throughout exploration.

## Recorded camera window

Extracted 2.1 s around arrival at the second view in the recorded mission run:
58 compressed camera frames, 96 truth poses, 243 TF messages, 8 AMCL poses,
2 observations. Only 6 MiB of new recording pages transferred in the successful
slice; the full 3.36 GB recording was not downloaded. A previous wider read
timed out; cached pages were retained and reused. Tailscale was initially
stopped; Marco reconnected it.

A diagnostic red-pixel check finds red in 49/58 frames (not the full detector
pipeline). The cube is visibly large in the contact sheet. The two published
observations have approximately 0.073 m raw sigma; fusion then imposes its
single-view floor until the observer baseline reaches 0.30 m. Visible observer
positions in this short window are closer together than that baseline.

This establishes visibility and some detection in that existing run, not the
cause of every run Hans mentioned. Next: evaluate complementary, spatially
separated coverage views and the existing inspection fallback, then measure
camera/selection behavior before claiming a fix.

The resulting fallback is integrated in mission with Marco's permission;
navigation, fusion, perception, launch and shared YAML remain unchanged.
See REVISIT_INTEGRATION.md for exact behavior and passing checks. Evidence
stays in build/verificacion/cobertura/puntos-ciegos/.
