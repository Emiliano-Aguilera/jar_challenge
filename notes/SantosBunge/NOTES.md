# SantosBunge's notes

Module: deteccion and fusion (perception). Who keeps which one is still to be agreed
with Emiliano-Aguilera; this branch implements both so either split works.

Code: `src/equipo_jar/equipo_jar/vision.py` (image, LiDAR and geometry math),
`deteccion.py` (node), `candidate_fusion.py` (fusion math), `fusion.py` (node).
Tunables: sections `deteccion` and `fusion` of `src/equipo_jar/params/params.yaml`.
Every tunable has a `#PARAM:` comment with what it does and its valid range.

One entry per decision: the question, what was tried, what was decided.

## How does detection measure the distance to a box?

- Tried: the depth cloud as the main source (median of the cloud points inside the red
  blob). It works in Gazebo, but the real robot's cloud is valid only from 0.6 m, has
  ~38 % valid pixels and gaps of up to a second (team CLAUDE.md).
- Decided: LiDAR first. The rays through the left and right edges of the red blob are
  taken to the LiDAR frame, the beams inside that wedge are selected, and the nearest
  cluster (everything within 0.30 m of the closest beam) is the box face. The LiDAR
  plane (0.18 m) cuts the 0.30 m tall box, so it always sees it. The cloud is only a
  fallback when the wedge has no beams.

## How are the camera and the LiDAR aligned in time?

- Problem: the camera runs at 30 Hz, the LiDAR at ~7 Hz with ~136 ms latency. While the
  robot turns at 1 rad/s, 100 ms is 10 cm at 1 m.
- Decided: pair image and scan with `ApproximateTimeSynchronizer` (0.1 s), and take the
  camera rays to the LiDAR frame with `lookup_transform_full` through `odom`, which
  applies the robot's motion between the two timestamps. Everything downstream uses the
  scan's timestamp.

## Face or center?

- The sensors see the front face; the judge scores the center, 0.125 m behind it for a
  0.25 m box. Half the 100-point radius.
- Decided: in the robot frame, shift the face point half a side along the line of
  sight (`center_from_face`). `box_side_m` = 0.25 from the simulator; the real size is
  a pending question to the organizers.

## What does detection hand to fusion?

- Options: a new message in `equipo_jar_msgs` (needs an interface PR) or a standard one.
- Decided: `/jar/observaciones` as `geometry_msgs/PoseWithCovarianceStamped` in `map`,
  one per box per frame. Position = center estimate; `covariance[0] = covariance[7] =
  sigma^2`, with `sigma = 0.05 + 0.03 * distance`; stamp = scan time. Fusion looks up
  the robot pose at that stamp itself. No new message type needed.

## What covariance does fusion report?

- Mission (AlsinaBautista) filters on `observaciones >= 5` and on the std along the
  worst direction (largest eigenvalue of `covarianza_xy`) `<= 0.10 m`, and wants every
  candidate, doubtful ones too, with an honest covariance.
- Decided: `cov = S / n + I / W`, where `I / W` is the covariance of the weighted mean
  (weights `1 / sigma^2`) and `S` the weighted sample covariance of the observations
  (so an elongated cloud gives an elongated ellipse). Plus a floor: while the robot poses
  the candidate was seen from span less than 0.3 m, the diagonal is at least 0.15^2.
  Ten frames of a reflection from one spot would otherwise look very precise; the floor
  keeps them above mission's 0.10 m until a second viewpoint confirms them.

## Do candidates ever disappear or merge?

- Decided: two candidates closer than 0.3 m merge and the lower id survives (as
  `Candidata.msg` promises). A candidate with fewer than 5 observations and nothing new
  for 20 s is dropped. Candidates with 5 or more observations are never dropped.

## Parameters

- Decided: all tunables go through `comun.parametros_en_vivo` and are read from the
  dict at use time, so they apply during a live session. Two exceptions, read at
  startup: `sync_tolerance_s` (the synchronizer is built once) and `publish_debug`.
- HSV bounds are lists of 3 (the CI rejects numeric lists of 4 or more).

## Which dependencies?

- Tried: cv_bridge and python3-opencv as rosdep (apt) dependencies. The first run on the
  simulation server died at container start right after the build (batch
  20261022-130550-3826108, `rc docker`, 0 s), most likely because those apt packages are
  not in the server's base image and the install failed.
- Decided: no cv_bridge (images decoded with numpy in `vision.decode_image`, bgr8/rgb8)
  and OpenCV from pip (`opencv-python-headless==4.9.0.80` in `requirements.txt`, the
  path the template documents). The remaining `package.xml` additions
  (tf2_geometry_msgs, sensor_msgs_py, message_filters, numpy) ship with ros-base.

## Robot facts checked in the simulator repo (yahboom_rosmaster @ jar2026-v1, 2026-10-22)

- Camera: `cam_1_color_optical_frame`, 320x240 `rgb8`, fx = fy = 271.809, cx = 158.719,
  cy = 120.092, mounted level (rpy 0 0 0) at x = 0.057 m, z = 0.0376 m above base_link
  (0.107 m above the floor). The macro default pitch of 0.5 rad is not used.
- LiDAR: `laser_link` at x = 0.0435, z = 0.11 m above base_link (0.18 m above the floor),
  yaw = pi. base_link is 0.0714 m above base_footprint.
- Depth cloud: frame `cam_1_depth_frame` (x forward, not an optical frame), unorganized,
  valid from 0.6 m, ~38 % valid points. The fallback therefore moves the cloud into the
  color optical frame with TF before projecting it with K (`_cloud_as_image`).
- Boxes in the practice worlds: 0.25 x 0.25 x 0.30 m, red diffuse (0.8, 0.05, 0.05),
  blue distractors (0.05, 0.05, 0.8). Pure hue, so the HSV ranges hold.

## First measurement in the simulator (session 20261022-140046, maze_1_6x5, teleop)

- Setup: `pruebas.launch.py` (map = odom fixed), robot driven by hand to both red boxes
  and past a blue one, 9 min recording. Evaluated offline with
  `notes/SantosBunge/analyze_bag.py`, which removes the odometry drift using the
  simulator's ground truth (offline only, never in `src/`).
- Detection error, drift removed: median 0.022 m (p90 0.027) on box 1 (572 obs,
  0.4-1.0 m) and 0.016 m (p90 0.022) on box 2 (62 obs, 0.7-1.8 m). Radial bias under
  1 cm: the face-to-center shift of 0.125 m is right. Zero observations near the blue
  boxes, zero farther than 0.5 m from a red box.
- As published (with drift): 0.19 m and 0.36 m. That is the mecanum odometry drifting
  during a few minutes of teleop. Localization has to close it; detection cannot.
- Reported sigma is conservative: |error| / sigma ≈ 0.2-0.3 in the simulator. Left as is
  (the real camera and LiDAR are noisier); mission's 0.10 m threshold is still met after
  a handful of observations once the robot has moved 0.3 m.
- Fusion: two candidates, ids 1 and 2, stable for the whole session, never merged.
  Candidate 1 became reportable only after the robot moved (single-view floor), as
  designed.

## False reports: a wall corner in front of a distant box (2026-10-25 runs)

- Evidence (hans-die's notes and his write-up of 2026-10-25): on two practice maps mission
  reported a point with no red box near it, 3 of 3 runs on one of them. Each false point lay
  on the bearing of a real red box but ~1 m closer, and the robot moved away along that
  bearing, so the observations agreed with each other and passed mission's rule
  (5 observations, std <= 0.10). Cost: -100 for the missed box and the -200 fast bonus.
- Cause: `nearest_cluster` took the nearest LiDAR points in the whole blob wedge (plus a
  0.03 rad margin). When a box peeks past a wall corner, the corner sits at the edge of the
  blob and is the nearest thing in the wedge, so the range was the corner's. The corner is a
  real point in space, so more viewpoints do not expose it: a geometric fix is needed, not
  a statistical one.
- Decided, three independent defenses in `deteccion` (each alone catches the known cases):
  1. `central_cluster`: only the beams in the central 60 % of the blob's angular span, no
     margin, grouped by range; the largest group wins and must cover >= 60 % of those
     beams. The box always fills the center of its own blob, and that line of sight is free.
  2. `width_is_consistent`: the blob's pixel width must match a 0.25 m box at the LiDAR
     range (ratio in [0.6, 1.8]). A corner 1 m in front of a box at 3 m gives ~0.5. Costs the
     views where the box is more than 40 % hidden, which other viewpoints recover.
  3. `map_range_along`: the LiDAR hit must not lie on a wall of `/map` (within 0.15 m of the
     map's own range along that bearing). Boxes are not drawn in the practice maps. If the
     competition map draws them, a mapped obstacle smaller than 0.40 m is spared, and
     `map_wall_check: false` turns it off (params only, allowed after the cutoff).
  Plus `max_range_m: 3.0`: coverage looks from 2.0 m, and far observations are the
  ambiguous ones.
- Not done: a perpendicular-baseline rule in fusion. The corner is a real point, so it is
  consistent from every viewpoint; the baseline would not have helped.
- Validation (replays of server recordings, `notes/SantosBunge/replay/`): the same bag through
  `development` and through this branch, scored against the world's boxes.

  | Recording | development | this branch |
  | --- | --- | --- |
  | maze_3 (batch 20261027-112638, the false report) | 69 obs, 16 false, FALSE candidate reported | 60 obs, 0 false, only the 2 real candidates |
  | maze_4 (batch 20261027-112941) | 59 obs, 0 false | 59 obs, 0 false |
  | laberinto_1 (batch 20261027-111526-d14d0a4) | 14 obs, 0 false, box 2 candidate at 0.19 m | 13 obs, 0 false, box 2 candidate at 0.06 m |

  Ablation on the maze_3 bag, one defense at a time: the central beams alone give 0 false
  observations (80 obs). Width, map and range rejected nothing false; the cloud check rejected
  16 correct observations (cloud vs LiDAR off by 0.6-1.5 m beyond ~2.5 m and below 0.6 m), so
  it ships OFF. `width_ratio_min` went from 0.6 to 0.45 after it dropped two real views of a
  box seen through a doorway at 0.8 m (ratios 0.53-0.60). Blobs touching the image border are
  exempt from the size checks.

## Should detection skip frames while localization is not ready? (2026-11-02)

- Ask (hans-die, `INTERFACES.md`): `/jar/localizacion_lista` is false at start-up until AMCL
  settles, and after each turn until the heading is fixed (on the real robot `/odom` counts
  turns ~25 % too much; 15° at 1.5 m puts a box 0.39 m off). Detection should skip those frames.
- Start-up: yes. Before the flag, the start pose is still being fixed (hand placement, LiDAR
  estimate). A box seen then lands wrong; fusion keeps a candidate with >= 5 observations for
  good, so mission would drive to inspect a ghost. Cost: none, the robot stands still until
  mission sees the same flag (it waits up to `localization_wait_max_s`, 10 s). Detection uses the
  same rule and default: if localization never says ready, both start at 10 s.
- After turns: no, off by default (`skip_heading_doubt: false`). Mission decided not to wait for
  the flag after trips (up to 21-34 s per run). Coverage plans `observation_time` 1.0 s per view
  and hans-die measured the fix at 1.2-1.7 s in the sim (max 2.1 s), so detection would be blind
  for the whole view and mission would leave before the flag returns. With the IMU heading (#44)
  the error at stops is ~1.8° (0.05 m at 1.5 m), inside `obs_sigma` at that range. If the real
  robot shows boxes off right after turns, turn it on (params only) and ask mission to wait too.
- Decided: `vision.skip_for_localization` (pure, tested), wired in `deteccion` before the image
  is decoded. No flag ever received = never skip (`pruebas.launch.py` runs `localizacion` with
  `usar_amcl: false`, which publishes true at once, but a dead node must not blind detection).
  Skipped frames show as `localizing` in the 10 s summary. `settle_s` 0 in mission stays.
- Checked (server, maze_3_6x6, 150 s, largada 0.15,-0.1,0.2, batch 20261102-114028-39bd8c8):
  `localizing` 36 per 10 s before `/map` and until the flag (4.6 s after `/map`), then 0 for the
  rest of the run. Reports at 16.7 s (0.007 m) and 52.7 s (0.032 m); `development` the night
  before (batch 20261101-225642-dc3d2df, 300 s, no offset): 14-16 s and 46-51 s, 0.03-0.05 m.
  The third box needs ~170 s, so 150 s stops at 200 either way. 0 violations, 0 collisions.

## Can a yellow, green, orange or neutral distractor pass the red filter? (2026-11-02)

- Ask (AlsinaBautista, FOR_THE_TEAM.md): practice maps only have blue distractors; the
  competition has yellow, green and neutral ones. Check the red thresholds against them.
- Tried (synthetic, no real image of such a distractor exists yet): OpenCV HSV, the filter is
  H in [0, 8] U [170, 180] (10 before #64), S >= 100, V >= 60. Three checks through the repo's
  `red_mask` + `find_blobs`: every pure hue; 15 colors as 50x60 patches with hue jitter
  (sigma 3), shading and sensor noise, 20 draws each; and a white-balance shift (R x k, B / k,
  k 0.8-1.5) to mimic warm or cold venue light.
- Result: yellow (H 26-30), green (57-60), blue (120) and the neutrals (grey, white, black,
  beige: S < 100) never give a blob, not even one mask pixel, under any shift. Yellow stays
  above H 21 at k 1.5. The only near neighbours are orange (H 15) and brown cardboard
  (H 10-13): with the upper hue at 10 they leave 60-77 scattered mask pixels per frame (no
  blob) and pass as single pixels at k >= 1.3; at 8 (#64) they leave 6-16 pixels and pass at
  no k. Red boxes stay at H 178-1 under every shift.
- Decided: no change to `hsv_red_low_1` or the saturation minimum (18 hue steps of margin to
  yellow; lowering S would only bring the neutrals closer). #64 matters against the cardboard
  walls, not against the listed distractors. On site: point the camera at one distractor of
  each color with `publish_debug: true` before the first run, since this is synthetic only.

## Next

- Repeat the measurement with localization (AMCL) instead of the fixed map = odom.
- Decide with the team whether to tighten `obs_sigma_*` after seeing the real robot.
- Confirm with AlsinaBautista that the 0.15 m single-view floor fits mission's policy.

## Testing without the simulator

- `notes/SantosBunge/fake_sensors.py`: publishes static TF, camera_info, a 320x240 image
  with a red rectangle and a 1080-beam scan for a box 2.0 m ahead, then prints the
  observations and candidates it gets back. Run `deteccion` and `fusion` with
  `params.yaml`, then `python3 notes/SantosBunge/fake_sensors.py 12`. Expected center
  (2.0, 0.0); with the robot still, the candidate std stays at the 0.15 m floor.
- Unit tests (no ROS): `src/equipo_jar/test/test_vision.py`, `test_candidate_fusion.py`.
