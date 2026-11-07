# Official runs, 2026-11-04 (qualification)

What the organizers recorded of our two qualification runs, committed unmodified. The organizers
sent each team a bundle with the data of every team; only ours is kept in the tree. The full
bundle is in the tag `archive/pre-cleanup`, for the team.

The tuning analysis built on top of it lives in [`../ajuste/`](../ajuste/README.md).

## Layout

In [`TARS/`](TARS/), folders without a suffix are **run 1** (commit `265bfec`, frozen at 09:00);
folders ending in `_c2` are **run 2** (commit `cec2c37`, only `params/` changed):

| Path | What |
| --- | --- |
| `corrida/`, `corrida_c2/` | What the judge recorded: `resultado.json` (score + events), `eventos.jsonl`, `juez.log`, `bag.log` |
| `analisis/`, `analisis_c2/` | Organizers' motion summary (`resumen_movimiento.md`) and the bag in text: `odom_2hz.csv` (pose every 0.5 s), `cmd_vel.csv` (commanded speeds) |
| `salida_del_equipo/`, `salida_del_equipo_c2/` | Our program's stdout/stderr on the organizers' laptop, unmodified |
| `logs_opcionales_del_equipo/`, `..._c2/` | Our own `log.txt` of each run |
| `organizacion/` | `AMBIENTE.md` (how the runs were made) and `NOTAS_TARS.md` (notes on our runs) |
| `LEEME.md` | The organizers' cover note |

## Scores

Best run counts. `+100` is the return bonus, `−50` per collision.

| Run | Score |
| --- | --- |
| 1 | 250: V1 and V2 + return − 50 (1 collision) |
| 2 | **300**: V2, V3 and V4, no return, no collisions loaded |

3rd of 12 teams, into the final.

## Read this before you trust a number

- **`log.txt` is lossy.** Our own log collapses repeated identical lines as `[+N same]`.
  Counting the literal occurrences undercounts: in run 2, `Optimizer fail to compute path`
  appears 42 times literally but **94** times expanded; `Starting point in lethal space` 18 → **65**.
  `salida_del_equipo/` has every line with its own timestamp: use it for counts.
- **Collisions and the return are human verdicts**, loaded through judge services after the run.
  They carry `agregado_manual` and have no time. Absence of a collision event does not mean the
  robot touched nothing: in run 2 the organizers stopped our robot for pushing walls and no
  collision was loaded. The contact analysis is in [`../ajuste/`](../ajuste/README.md).
- **No `/scan` and no camera in the bags.** Anything about what the robot saw has to come from the
  program log.
- **Victim ground-truth positions were removed** from `resultado.json`. Only the reported positions
  and the resulting distance and points are there.

## Rebuilding the qualification map

See ["The real qualification map"](../ajuste/README.md#the-real-qualification-map) in the tuning
analysis.
