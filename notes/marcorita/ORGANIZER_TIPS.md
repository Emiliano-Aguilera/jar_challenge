# Organizer tips and coverage impact, 2026-10-28

Source: the organizer email supplied by Marco in this chat. These instructions
take precedence over older assumptions in our notes. This review does not
change shared parameters, other modules or CLAUDE.md.

| Organizer information | Coverage impact | Action / owner |
| --- | --- | --- |
| Rotation is noisy; compare wheels, IMU and LiDAR | A nominal 61-degree view can miss targets near its edges. A reached goal does not establish the camera's true heading. | Coverage audits sampled angular errors; Hans owns sensor fusion/AMCL corrections. |
| Workshop 10 bag is more realistic than simulation | Simulator-only improvements do not establish real heading accuracy or detection latency. | Reuse that bag for the next integration validation. It was not replayed in this task. |
| Speed limit is 0.5 m/s | Our planning proxy is 0.25 m/s. Navigation's separate x/y limits need a resultant-speed check. | Hans: audit and bound sqrt(vx^2 + vy^2), not just each axis, if this is the organizers' intended speed definition. |
| Robot starts near map origin, with uncertainty | Start placement and localization error are different tests. Coverage already reads TF and rejects an unsafe start instead of snapping through a wall. | Keep localization Gaussian/covariance calibration with Hans. Test both offset starts and residual localization error; do not force coverage's start to zero. |
| Return bonus requires all four wheels inside the 0.25 m circle | A base-frame centre within 0.25 m is insufficient. Our return-distance estimate is only a cost proxy. | Bautista implemented return checks/retries in #32 and documented wheel geometry. Hans/Bautista still plan a tighter return-only goal checker to leave localization margin. The practice judge checks centre distance. |
| Windows allow LiDAR/camera sight | Optical visibility does not imply robot access; a single 2-D occupancy layer does not describe opening height. | Ask organizers how windows are encoded in /map. Keep traversal and visibility assumptions separate; do not treat every visible gap as traversable. |
| Fixed non-box obstacles can be absent from /map | Coverage's static-map component cannot exclude these objects. Candidate-box zones do not cover them. | Nav2 live costmaps/avoidance and mission's failure handling remain necessary. A costmap-to-coverage interface needs Hans's agreement. |
| Continuous point-cloud use can cause latency | Coverage itself does not subscribe to the depth cloud. Detection currently does, with fallback enabled. | Perception owners should contact organizers as requested and measure latency before changing the fallback. |

## Concrete navigation check

Current navigation parameters set vx_max = vy_max = 0.5 and smoother limits
to [0.5, 0.5, 2.0], with scale_velocities false. A diagonal command (0.5, 0.5)
has translational norm about 0.707 m/s. This is an allowed configuration range,
not evidence that a recorded run actually exceeded the limit.

The Humble [MPPI ConstraintCritic source](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/src/critics/constraint_critic.cpp)
derives its norm bound from sqrt(vx_max^2 + vy_max^2). The
[velocity smoother source](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp)
clamps the components separately. Those limits alone do not enforce a 0.5 m/s
resultant-speed ceiling. Do not blindly reduce or disable lateral motion here;
navigation's owner should choose and verify the fix.

## Messages Marco can send

To Hans:

> The organizer email sets 0.5 m/s. Current vx/vy and smoother limits each allow
> 0.5, hence up to 0.707 diagonally; Humble's ConstraintCritic uses the combined
> bound. Could you check /cmd_vel's translational norm and enforce the total
> limit? I have not changed navigation. Also, what map encoding should we
> assume for a window versus a traversable opening?

To perception:

> The organizers ask teams using the cloud throughout the run to contact them.
> Detection subscribes to /cam_1/depth/color/points and use_cloud_fallback is
> true (cloud_check is false). Please coordinate that and measure latency;
> coverage uses camera TF, not the cloud. I have not disabled the fallback.

To Bautista:

> I reviewed #32: return checking/retries are included. Our coverage return cost
> is only an estimate, not validation of the bonus. The remaining issue in your
> notes is the tighter return-only goal checker with Hans: the current 0.10 m
> tolerance leaves essentially no localization margin under your wheel-geometry
> bound. I will not change the general exploration tolerance from coverage.

Question for organizers (not sent):

> How are windows represented in /map: as occupied wall cells despite optical
> visibility, or as free gaps? Will there be any height/traversability annotation?

These messages are drafts only. No teammate or organizer was contacted.
