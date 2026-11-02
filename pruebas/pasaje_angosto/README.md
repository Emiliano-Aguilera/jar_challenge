# Experimental narrow-passage mode

Branch: `navegacion/pasaje-angosto`, from `development`. Keep the PR in draft and do not
merge before recorded official-simulator comparisons. This is a navigation experiment,
not a validated competition improvement.

## Behavior

The Nav2 controller plugin delegates ordinary motion to the existing MPPI controller
with the same parameters. For a recognized short, two-sided narrow passage, it checks a
fixed-heading route against the full global raw costmap, including sensed obstacles.
It recenters nearby path positions and uses the existing **padded rectangular footprint**
plus `extra_margin` (0.02 m). It never reduces the chassis or costmap padding.

1. Check that the approach and an exit with room for the whole body to turn are known
   within `lookahead` (1.4 m). Unknown/outside cells block the geometry check.
2. Brake and observe a stationary interval; align only in a checked turning area.
3. Translate with a fixed body heading, allowing small yaw corrections and limited
   lateral mecanum translation. Recheck actual/proposed stopping envelopes against both
   the latest global/local grids and the retained initial global grid.
4. Stop at the exit. Release the mode only when the current full-body turning envelope
   is free and fresh odometry reports stationary motion. The normal goal checker is
   suppressed while the passage lock is active, including the precise return checker.

Replanning and goal cancellation cannot clear the passage lock. While locked, guarded
Spin and BackUp recoveries are denied by a fresh service query to the controller before
initialization and before every recovery cycle. Missing/timed-out permission is denied.
There is no second publisher of `/cmd_vel`; Nav2 still feeds the existing smoother and
permanent terminal stop guard.

`/jar/pasaje_estado` is a retained diagnostic String, with phases `normal`, `brake`,
`align`, `cross`, `exit`, `hold`. Logs include the same transitions and hold reason.
`/jar/pasaje_permitir_recuperacion` is an internal Trigger service; it is not a new
mission interface. Mission still uses `navigate_to_pose` and its existing cancel/stop path.

## Limits that matter before merging

- Only recognized, two-sided passages with a fully checked local approach/exit enter
  this mode. It is not a planner over arbitrary position/heading space. A long unseen
  exit or a route requiring a body turn inside is rejected. A 64 cm doorway with a post
  inside must be evaluated as a whole maneuver, not accepted by doorway width alone.
- Rejection while still outside leaves the passage lock unset, so another goal can be
  tried. Failure/data loss **after entering the mode** latches `hold` and denies recovery
  rotation. No autonomous reverse-escape controller is implemented in this first version.
  Do not clear the lock or restart a competition run to force movement.
- A frozen grid prevents costmap clearing from removing the original passage obstacles
  during the crossing. New obstacles are checked in current grids too.
- Raw global grid receipt/source age is bounded by 2.0 s: the unchanged 1 Hz publisher on
  a 2 Hz update loop can publish every 1.5 s. LiDAR is bounded by 0.5 s, robot TF by 0.3 s,
  and the local costmap must be current. Overloaded or paused simulations can stop the mode.
- Stopping checks use the configured reaction time and conservative constant-twist
  envelope, with assumed deceleration. They are not a proof about motor response, wheel
  slip, sensor error or unknown moving objects. The hardware response must be measured.
- The code default is disabled; this experimental branch's YAML enables it. Live edits
  of passage parameters are rejected while locked. To compare the same build in wide
  space while idle, set `FollowPath.passage.enabled` false on `controller_server`.
- The low speeds/corrections and goal approach still need physical-robot calibration.
  Unit tests and synthetic kinematics do not establish contact-free execution.

## Local checks

Build the complete workspace with ROS Humble, then source `install/setup.bash`:

```bash
colcon build --base-paths src
colcon test --packages-select equipo_jar_navegacion --event-handlers console_direct+
colcon test-result --verbose
```

The C++ test fixtures are outside `src/`; a runtime-only Docker context containing only
`src/` builds the plugins without these test executables. With the full checkout they
exercise occupied/unknown/outside cells, full footprint and swept motion, braking,
fixed heading, new obstacles, whole-body exit, real plugin loading, goal locking, and
recovery denial with valid TF plus a positive unguarded-behavior control.

For real Nav2 transport and idealized synthetic sensors/kinematics (no Gazebo):

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
python3 pruebas/pasaje_angosto/probe_transport.py \
  --output build/passage-transport-01 --timeout 80
```

Use a new output directory for every probe. It starts navigation only, publishes test
map/scan/odometry/TF, sends one goal, and saves the actual phases, action result and
trajectory. It does not test AMCL accuracy, collision physics or victim reports.

## Official simulator comparison

Run these **before merging**, one request at a time. The server builds the
pushed branch and runs the ordinary competition launch; no workflow changes are needed.
Start small, and inspect the first run before expanding repetitions:

```bash
gh workflow run simular.yml -R Emiliano-Aguilera/jar_challenge \
  -f rama=navegacion/pasaje-angosto -f mapas=ejemplo_senuelos \
  -f repeticiones=1 -f duracion=300 -f grabar=true
```

Then use the same command with `mapas=tars_borde_ventanas_ancha`, followed by
`mapas=tars_borde_ventanas`, and a previously successful easy map. After a promising
first run, repeat with `repeticiones=2`, then the start offset
`-f largada=0.15,-0.1,0.2`. Keep the robot/world/image versions recorded.

Inspect actual activation and entry/exit, true heading/lateral clearance, simulator
collision episodes, final pose, cancellation, recovery commands and terminal stop.
For full missions also compare victims, return, time budgets and protocol violations.
A green workflow or a newly reported victim alone does not establish a successful
passage. Separate runs with RTF below 0.9 from timing comparisons. Never attribute this
mode's success to a run where it did not activate.

## Verification recorded before publishing

The full existing Python suite passed: **792 tests**.
`tools/verificar.sh --template` passed all controls, including the interface/map-wait
audit; colcon reported 259 tests with zero errors or failures. The C++ tests cover geometry,
passage state/obstacles and real ROS plugin/permission/goal-checker behavior. A local
real-Nav2 transport probe completed `brake → align → cross → exit → normal` with action
status 4. Its sensors/kinematics were synthetic; it is not an official-simulator result.
No manual official batch or live simulator session was dispatched for this implementation.
PR CI smoke tests are separate from the recorded server comparison.

The alignment command honors the configured 0.5 rad/s turn-from-rest floor outside the
passage; translational commands use a configurable 0.15 m/s floor before component
limits. Lateral/yaw corrections while translating still need hardware calibration.
Independent review and the recorded official scenarios above remain merge gates.
