# Corrected coverage: local navigation follow-up

2026-10-26. Branch cobertura/recorrido-laberinto. No Nav2 configuration or
mission source change applied. This is a diagnostic case for Hans and Marco.

## Server follow-up

The third authorized server run repeats navigation failures with RTF .972,
matching the reference's .973 on the same GPU 0/slot 0/simulator image.
Four actions fail, compared with zero in the reference. Near the first failure,
recorded AMCL and simulator truth at stamps 247.420/247.421 s differ by .5363 m:
estimate (-2.2653,.1776), actual (-2.7350,-.0812). Their map cells are occupied
(100) and free (0), respectively. World and map align in this practice world.
These synchronized samples strengthen the earlier local localization clue.
They do not isolate why the error arose or exclude route influence.
See SERVER_COMPARISON.md and ignored server-recorrido/corrected/
localization-diagnostic.json. Truth is used only for evaluation, never src.

## Earlier local smoke

A 300-wall-second full-team laberinto_1 smoke publishes 79 coverage goals,
completes 12 Nav2 goals and fails two. It reaches only 82.2 simulated seconds
(RTF about .273), with no candidates, reports or protocol violations; score 0.
The fixture prepares sensor TF before the judge, so it does not test production
startup. Different startup localization can change selected views.

| Failed requested pose (map) | Heading | Failure time after map |
| --- | --- | --- |
| (-2.8249999929, 0.2250000488) | -pi/2 | 233.53 wall / 64.01 sim s |
| (-2.7749999922, -0.0249999549) | -pi/4 | 300.04 wall / 82.07 sim s |

The first follows the successful goal (-2.6749999907, 2.2250000786, -pi/2).
Logs show repeated GridBased failures to find a path with tolerance .05,
costmap clears and backup/spin/wait recovery, then NavigateToPose failure.
At the next goal, Nav2 reports its estimated start near (-2.22, .95).
No claim is made that the requested goal alone, clearance, localization,
simulator load or the new ordering caused the failure. An equivalent baseline
execution under comparable conditions is needed to isolate it.

The second action failure is recorded just after 300 wall seconds, before
the launch SIGINT. It is not the later action-abort message caused by shutdown.
During SIGINT cleanup, mission and the timeline recorder raise RuntimeError
while taking a subscription message; coverage exits cleanly. The launch parent
exits 0. Parent success must not be reported as every child exiting cleanly.

Local ignored evidence: build/verificacion/cobertura/nav-vistas-protegidas/
candidate/{probe.json,equipo.log,timeline.jsonl,timeline.log,plan.json,plan.png,
trayectoria.jsonl,resultado.json}. Full-team behavior is not cleared for merge.
Earlier short Path-window successes do not override these longer-run failures.
