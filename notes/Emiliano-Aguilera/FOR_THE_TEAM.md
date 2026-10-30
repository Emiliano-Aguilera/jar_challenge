# For the team, from Emiliano

Things found that belong to other modules or to everyone. Each item says who it is for.

## 2026-10-29

### For localizacion (hans-die): odometry overcounts turns and distance — done

On the week-10 bag the real motion is about 0.79 × what `/odom` reports (turns 0.80, forward
0.78, sideways 0.79, by LiDAR scan matching between stops). AMCL's `alpha` params model noise,
not a bias, so AMCL can't absorb it.

Done in `odometria_imu` (PR #44): heading from the gyro, distance scaled by `escala_avance`
(learned, 0.798 on the bag). Checked against a reference built without the IMU (comment on
PR #44): heading at stops 1.5° / 3.5° median / p90, boxes 1.5 m ahead > 0.25 m off 0 % (59 %
before). Details: `notes/hans-die/NOTES.md`.

### For navegacion (hans-die): recovery speeds too low for the real robot — done

From rest, the real robot needs about 0.15 m/s forward and 0.5 rad/s turning to move
(organizers' floor tests, `physical_rosmaster`). BackUp ran at 0.05 m/s and a Spin's last 2.2°
were below 0.5 rad/s. Done in PR #51 (BackUp 0.15 m/s, Spin min 0.5 rad/s).

Still open: in a sim run of `development` 95a6adf on maze_1_6x5
([run](https://github.com/Emiliano-Aguilera/jar_challenge/actions/runs/40951055920)) Nav2
stalled for 21.9 s at (−0.66, −2.06), about 0.5 m from victim 1 ("Failed to make progress" 4
times, then a spin got it out). The sim robot stalled too, so it isn't a motor-speed problem.
Not diagnosed.
