# On site

What we agreed to do at the venue, before our runs. Everything here ends up as a value in
`src/equipo_jar/params/`, the only thing that may change after the delivery cutoff. If an
item depends on code, that code must be merged before the cutoff.

If an organizer answer in [`ORGANIZER_QUESTIONS.md`](ORGANIZER_QUESTIONS.md) settles an
item, write the answer here and drop the on-site step.

| What | How | Params it sets | Open question |
| --- | --- | --- | --- |
| Box size | Tape-measure one real box: width, depth, height. | `box_side_m` (detection: the visible face is half a side in front of the center); check `map_box_extent_m` | "¿Las cajas reales miden lo mismo que en el simulador…?" (Not sent) |
| Robot still on `/odom` | `ros2 topic echo /odom` for 10 s with the robot still: the speeds must stay below the thresholds. In the organizers' bag they are exactly 0. | `parada.still_linear_mps`, `still_angular_rads` (`confirm_timeout_s` confirms the stop anyway after 3 s, PR #67) | |
| Gyro sign | Nothing to do: learned at the first turn. Check the `gyro sign` line in the log. The real gyro was inverted in both bags. | `odometria_imu.signo_giroscopo` (0 = learn) | |
| Distance scale | Drive 1 m straight, tape-measure it and compare with `/odom`. The organizers' second bag gave 0.988 against the LiDAR: leave 1.0 unless the error is over ~3 %. | `odometria_imu.escala_avance` | |
| Heading after turns | Turn 360° in place and compare `/odom`, the IMU and `/amcl_pose`. | `localizacion.actualizaciones_giro` | |
| Slowest speed that moves the robot | From rest, send 0.4 and 0.5 rad/s. Spin recovery uses this as its minimum. | `behavior_server.min_rotational_vel` | |
| LiDAR-to-map fit | Read the `Localization check` lines in the log (median score at AMCL's pose). The checks are tuned for a real-robot median of ~0.71; if the venue gives much less, lower the watch thresholds. | `localizacion.vigia_puntaje_bajo`, `vigia_puntaje_bueno` | |
| Motors keep up | Watch a first run: if the robot overshoots turns or slides, lower the limits. Today 0.5 m/s per axis and 1 rad/s, the robot's own caps. | `limites.lineal_max_ms`, `giro_max_rads` | |
| Map resolution | Nothing to do: every node reads it from `/map`. At 0.03 m/cell (the new organizers' map) the start-pose search takes ~7 ms, Smac 2D (#69) ~33 ms and coverage ~4 s. | | |
