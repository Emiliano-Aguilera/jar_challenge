# For the team, from Emiliano

Things found that belong to other modules or to everyone. Each item says who it is for.

## 2026-10-29

### For localizacion (hans-die): scale the odometry before AMCL

**Problem.** On the week-10 bag the real motion is about 0.79 × what `/odom` reports.
Measured with LiDAR scan matching between stops: turns 0.80, forward 0.78, sideways 0.79.
IMU/odom on turns: 0.80. AMCL's `alpha` params model noise, not a bias, so AMCL can't absorb it.
Workshop 10, step 3, describes the same correction.

**Evidence.** Replay with `notes/hans-die/bag_replay` (PR #34), `run(..., odom_scale=...)`,
10 seeds, a box 1.5 m ahead, at the stops after turns:

| Setup | Heading p90 | Box error median / p90 | Boxes > 0.25 m |
| --- | --- | --- | --- |
| Today | 14.6° | 0.28 / 0.38 m | 59 % |
| Scale 0.788 | 3.7° | 0.07 / 0.10 m | 0 % |
| Scale 0.788 + PR #34 (5 updates) | 3.2° | 0.05 / 0.09 m | 0 % |

**Fix.**

1. A new node reads TF `odom → base_footprint`, scales each step in the robot frame (x and y
   by `escala_avance`, yaw by `escala_giro`), adds the steps up into a corrected pose, and
   publishes TF `jar_odom → odom` so that `jar_odom → base_footprint` is the corrected pose.
   We can't publish `/odom` or a second parent for `base_footprint`, hence the extra frame.
2. AMCL: `odom_frame_id: jar_odom`.
3. `params/`: `escala_avance: 0.79`, `escala_giro: 0.79` (1.0 turns it off). Params may change
   after the cutoff, so we can retune if the robot's odometry changes.
4. In the sim, force both to 1.0 (for example from the launch when `use_sim_time:=true`).
   Sim odometry is exact, and CI and the leaderboard use our params.
5. A `setup.py` entry and the launch change.

**Checks.**

- Sim run: same result as today (scale 1.0 there).
- Replay with 0.79: the `alpha` values were raised partly to cover the bias, so they can likely
  go back down.
- Not covered: `controller_server` still reads `/odom` speeds about 27 % high
  (`odom_topic: /odom`). Probably minor, but worth a look.

### For navegacion (hans-die): two recovery speeds too low for the real robot

**Problem.** The organizers' floor tests (`physical_rosmaster` validation reports, battery at
~10.3–11.2 V): from rest, the real robot needs about 0.15 m/s forward and about 0.5 rad/s
turning to move. 0.12 m/s barely moved it; 0.12 and 0.30 rad/s didn't move it at all. The sim
has no such limit.

1. **BackUp:** `backup_speed="0.05"` in `behavior_trees/navegar.xml` and `vuelta_precisa.xml`.
   - At that speed the real robot won't move, so it never covers the 0.30 m and waits until
     the timeout.
   - In a sim run of `development` 93bdd59 (2026-10-27) it ran 5+ s twice.
   - **Fix:** `backup_speed="0.15"` (0.20 for margin). These files aren't in `params/`, so the
     change has to be merged before the cutoff. If 0.05 was chosen to back away gently near
     walls, tell us.
2. **Spin:** `behavior_server.min_rotational_vel: 0.4`.
   - A spin is commanded below 0.5 rad/s for its last 2.2°, and at 0.4 for the last 1.4°
     (`nav2_behaviors/plugins/spin.cpp`, Humble, with our `rotational_acc_lim: 3.2`).
   - 0.4 was never tested on the floor. If the robot stalls there, the spin fails after its
     10 s time allowance (Nav2's default; our `<Spin>` doesn't set one).
   - **Fix:** 0.5. It's a param.

**Evidence the rest is fine.** `/cmd_vel` from a sim run of `development` 95a6adf on maze_1_6x5
([run](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40951055920)). Apart from
these two, the commands below these thresholds were all short (under 0.5 s, 4 % of command
time). No general fix is needed.

**Also seen, not a speed problem.** In that run Nav2 stalled for 21.9 s at (−0.66, −2.06),
about 0.5 m from victim 1: "Failed to make progress" 4 times, then a spin got it out. The sim
robot stalled too, and the 93bdd59 run was also slow there. Not diagnosed.
