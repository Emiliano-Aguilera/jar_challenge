# Bag replay: AMCL offline on a real-robot bag

Replays Nav2 Humble's AMCL (omni motion model, likelihood field, resampling, clustering) on a
real X3 bag, with the team's params, and compares it with a reference pose. Use it to tune
`alpha1`, `actualizaciones_giro` and friends when the organizers send new bags. Results of the
week-10 bag: `../NOTES.md`, "Localization: the real robot's odometry overcounts".

Needs `numpy`, `scipy`, `rosbags` (pip). The bag and the map stay outside the repo (CI rejects
`.pgm` and map YAML files anywhere in it).

1. `python extract.py BAG_DIR data.npz`: dumps `/scan`, `/odom`, `/imu/data`, `/tf_static`,
   `/cmd_vel`. The scripts load `data.npz` from this folder.
2. `python reference.py MAP.pgm 1 ref.npz`: reference pose per scan (each scan fitted to the
   map walls around an IMU + odometry prediction). Check the map origin in `common.py`
   (`load_map`) and the laser offset (`LASER_X/Y/YAW`) against the new bag's map and TF.
3. `python evaluate.py MAP.pgm ref.npz 10 ev`: AMCL variants (`VARIANTS`: alpha1, no-motion
   updates after turns), error at the stop after each turn. `turns.py` breaks it down per turn.
4. `python eval_nodo.py MAP.pgm ref.npz 10`: the same replay driven by the localization node's
   own class (`src/equipo_jar/equipo_jar/correccion_giro.py`), with the error when
   `/jar/localizacion_lista` turns true again.

Written by an agent on 2026-10-27/28 for the week-10 bag (`laberinto_victimas`, release
`semana-10-datos` of AIRclub-UdeSA/jar_workshops). Not part of the delivery.
