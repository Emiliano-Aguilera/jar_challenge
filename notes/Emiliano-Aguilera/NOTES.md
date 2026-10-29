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

Decided 2026-10-29: off by default; subscribe on demand proposed, waits for Santos's OK.

- Until 2026-10-29: detection used the LiDAR distance; the cloud was only a fallback
  (`use_cloud_fallback: true`, `cloud_check: false`, the latter stays off). The source of
  each observation is logged only at debug level (`obs (lidar)` / `obs (cloud)`).
- Cost on the robot: `sensor_adapter.py` (AIRclub-UdeSA/physical_rosmaster) always builds
  and republishes the cloud, `cloud_decimation` default 1, ~540 KB per cloud at ~8.3 Hz.
- Organizers' router test (`physical_rosmaster/robot_artifacts/raphael_router_wifi_2026-10-16.md`):
  image + a remote cloud subscriber dropped the camera 30 → 17.8 Hz, latency ~160 ms, ping
  max 1.5 s, `/cmd_vel` ~0.5 s slower. A decimated cloud was not tested.
- Sim replay on the server, 2026-10-28 (5 maps, fallback on/off): 0 cloud observations out
  of 247. The sim LiDAR has 0 % invalid rays.
- Real week-10 bag (`laberinto_victimas`, release `semana-10-datos` of AIRclub-UdeSA/jar_workshops):
  5.8 % invalid rays, 4.0 % in the camera FOV (the "~25 %" was an open room). The LiDAR ranged
  324 of 324 red blobs, at 0.44-1.34 m. The bag has no cloud and no box beyond ~1.4 m.
- DDS: a subscription that exists pulls every cloud over Wi-Fi, used or not. The subscriber
  cannot ask for less (no rate filter in Fast DDS, no content filter in rclpy Humble).
- Tried: nothing on the real robot yet.
- Decided:
  - No cloud subscription unless `use_cloud_fallback` or `cloud_check` is true; both false.
  - Proposed, not built: subscribe only when a red blob has no LiDAR range, release after a
    few seconds without need, with a cooldown and a per-run budget, all behind params, off by
    default. Main risk is the Wi-Fi burst while subscribed, not the code (one-thread executor).
    Turn it on only if a bag shows LiDAR misses at 1.5-3 m or the organizers set decimation 2.
