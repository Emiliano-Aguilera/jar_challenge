# Official runs, 2026-11-04 (qualification)

This is the bundle the organizers sent to every team after the qualification: the judge's record,
the motion summaries and the raw program output of **all seven teams**, two runs each (a few teams
only ran once). It is committed unmodified so that everyone on the team works from the same data.

The tuning analysis built on top of it lives in [`../ajuste/`](../ajuste/README.md).

## Layout

Per team, folders without a suffix are **run 1** (the 09:00 frozen commit); folders ending in `_c2`
are **run 2**:

| Path | What |
| --- | --- |
| `corrida/`, `corrida_c2/` | What the judge recorded: `resultado.json` (score + events), `eventos.jsonl`, `juez.log`, `bag.log` |
| `analisis/`, `analisis_c2/` | Organizers' motion summary (`resumen_movimiento.md`) and the bag in text: `odom_2hz.csv` (pose every 0.5 s), `cmd_vel.csv` (commanded speeds) |
| `salida_del_equipo/`, `salida_del_equipo_c2/` | Our program's stdout/stderr on the organizers' laptop, unmodified |
| `logs_opcionales_del_equipo/`, `..._c2/` | The team's own `log.txt`, when they saved one |
| `organizacion/` | `AMBIENTE.md` (how the runs were made) and `NOTAS_<team>.md` (notes on that team's runs) |
| `LEEME.md` | The organizers' cover note for that team |

## Scores

Best run counts. `+100` is the return bonus, `−50` per collision.

| Team | PR | Run 1 | Run 2 |
| --- | --- | --- | --- |
| Grupo_de_Lie | 14 | **500** — 4 victims + return | 0 — V2 + return − 250 (**5 collisions**) |
| BoBots | 17 | 200 — V4 + return | **400** — V2, V3, V4 + return |
| **TARS** | **16** | 250 — V1, V2 + return − 50 (1 collision) | **300** — V2, V3, V4, no return, 0 collisions loaded |
| Stack_Queue | 15 | **150** — V2, V1 + return − 100 (2 collisions) | 0 — `inmovil`, no reports |
| Zonda_Robotics | 12 | **150** — V4 + return | — |
| JARVIS | 20 | 0 — timeout at 600 s, no reports | 0 — 1 report, missed |
| CapiBot | 13 | 0 — no reports | — |

Top four by best run: Grupo_de_Lie, BoBots, TARS, Stack_Queue/Zonda_Robotics (tied at 150).

## Read this before you trust a number

- **`log.txt` is lossy.** The team's own log collapses repeated identical lines as `[+N same]`.
  Counting the literal occurrences undercounts. For TARS run 2, `Optimizer fail to compute path`
  appears 42 times literally but **94** times expanded; `Starting point in lethal space` 18 → **65**.
  `salida_del_equipo/` has every line with its own timestamp — use it for counts.
- **Collisions and the return are human verdicts**, loaded through judge services after the run.
  They carry `agregado_manual` and have no time. A `correccion_manual` event records the original
  and corrected score. Absence of a collision event does not mean the robot touched nothing.
- **Collision counting was not uniform.** Grupo_de_Lie had 5 collisions loaded in run 2 (−250) and
  TARS had none, although TARS's robot was stopped by the organizers for pushing walls. Treat any
  "0 collisions" as unverified unless the run was also clean by the contact analysis in
  [`../ajuste/`](../ajuste/README.md).
- **No `/scan` and no camera in the bags.** Anything about what the robot saw has to come from the
  program log.
- **Victim ground-truth positions were removed** from `resultado.json`. Only the reported positions
  and the resulting distance/points are there.
- **Some captures run past `/done`.** For BoBots, Grupo_de_Lie and Stack_Queue the organizers cut the
  program late; lines after `/done` are not part of the run (see each `NOTAS_*.md`).
- **The judge itself can be cut short.** BoBots run 2 has no `FIN DE LA CORRIDA` line and an empty
  `termino_por`, so its return was loaded afterwards.

## Rebuilding the qualification map

The map is not in git: `tools/verificar_estructura.py` rejects map files (§10). It can be rebuilt
from Grupo_de_Lie's run-1 log, which prints the `/map` it received as zlib + base64:

```bash
python3 docs/team/ajuste/scripts/decode_map_from_log.py \
  docs/team/corridas/2026-11-04/Grupo_de_Lie/salida_del_equipo/pr14_c1_1540.txt \
  build/mapa_qualy/qualy_2026-11-04
```

Check it: `174x383 @ 0.02 m, origin (-0.553, -0.572)`. Our coverage planner on it with run-1 params
gives 57 views and 327 of 1056 samples unreachable; the run-1 log says 57 and 326.

## Where our own logs also live

`docs/team/ajuste/runs/` holds the TARS extracts the analysis scripts read (`run1_log.txt`,
`run1_odom_2hz.csv`, `run1_cmd_vel.csv`, `run1_result.json`, and the same for run 2). They are the
same bytes as `TARS/logs_opcionales_del_equipo*/` and `TARS/analisis*/` here, renamed. The full
`salida_del_equipo/` capture, which those extracts do not include, is only in this folder.
