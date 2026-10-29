# Emiliano-Aguilera's notes

Module: perception (deteccion, fusion). I handle it now; Santos takes over when I finish.

One entry per decision: the question, what was tried, what was decided.

## Open: to discuss

With Santos and Hans:

- Branch `localizacion/correccion-giros` (no PR yet) adds a new topic,
  `/jar/localizacion_lista` (Bool): false at start-up and after each turn until AMCL
  has fixed the heading. Its `INTERFACES.md` change also says "Detection: skip frames
  taken while it is `false`". That is a rule for perception, so it needs our OK. Same
  idea as gating detection after turns (real `/odom` over-counts ~25 %, turns and
  straight driving). It also clashes with our `INTERFACES.md` change (7ebe48c,
  Detection → Fusion section). Hans already has the message.

With the organizers:

- Delivery cutoff date: still unknown (in `docs/team/ORGANIZER_QUESTIONS.md`, not sent).
  They told us we can calibrate on the tournament robot in place, before the tournament.
- Point cloud decimation on the tournament robot: which value, and can they set 2?
  A 4x smaller cloud may make the cloud usable over the competition Wi-Fi.

## Depth cloud: keep the fallback, subscribe on demand, or drop it?

Undecided. A separate analysis session is comparing the options and the measures needed.

- Today: detection uses the LiDAR distance; the cloud is only a fallback
  (`use_cloud_fallback: true`, `cloud_check: false`, the latter stays off). The source of
  each observation is logged only at debug level (`obs (lidar)` / `obs (cloud)`).
- Cost on the robot: `sensor_adapter.py` (AIRclub-UdeSA/physical_rosmaster) always builds
  and republishes the cloud, `cloud_decimation` default 1, ~540 KB per cloud at ~8.3 Hz.
- Organizers' router test (`physical_rosmaster/robot_artifacts/raphael_router_wifi_2026-10-16.md`):
  image + a remote cloud subscriber dropped the camera 30 → 17.8 Hz, latency ~160 ms, ping
  max 1.5 s, `/cmd_vel` ~0.5 s slower. A decimated cloud was not tested.
- Sim replay on the server, 2026-10-28 (5 maps, fallback on/off): 0 cloud observations out
  of 247. Not representative: the sim LiDAR has 0 % invalid rays, the real one ~25 %.
- Tried:
- Decided:
