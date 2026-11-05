# JARVIS: datos de su corrida del 4/11/2026

PR #20, commit probado `d99517bd8623a2e716e34b3f4f81d77a9bb205ba` (el congelado en el cierre de las 9:00). Es la **corrida 1** de la jornada.

## Qué hay en este zip
- `corrida/`: lo que registró el juez: `resultado.json` (puntaje y eventos), `eventos.jsonl`, `juez.log`, `bag.log`. El rosbag2 de la corrida (`/odom`, `/tf`, `/tf_static`, `/cmd_vel`, `/done`, `/victimas`; **no** incluye `/scan` ni la cámara) lo tenemos y se los pasamos si lo piden. Por seguridad quitamos de `resultado.json` el campo con las posiciones reales de las víctimas.
- `analisis/`: resumen del movimiento calculado por nosotros (`resumen_movimiento.md`) y los datos en texto: `odom_2hz.csv` (posición cada 0,5 s) y `cmd_vel.csv` (velocidades comandadas).
- `organizacion/`: `AMBIENTE.md` (cómo corrimos, puntaje, incidentes) y `NOTAS_JARVIS.md` (lo específico de su corrida).
- No hay `log.txt` opcional de ustedes: no lo tenemos guardado.
- `salida_del_equipo/`: lo que imprimió su programa en nuestra laptop (stdout/stderr de `ros2 launch`), **sin modificar**. El `Traceback` / `KeyboardInterrupt` del final aparece porque lo cortamos nosotros con Ctrl+C. Las rutas `/home/jar-equipos/…` son las de nuestro usuario aislado.

## Para leer rápido
- `juez.log` termina con la línea `=== FIN DE LA CORRIDA: … ===` y el resumen del puntaje.
- En `resultado.json`: `termino_por` (`done`, `inmovil`, `tiempo`…), `puntaje`, `reportes` y `eventos`. Los eventos `agregado_manual` o `correccion_manual` son veredictos humanos cargados después.
- Si algo no cuadra con lo que vieron, avísennos: preferimos revisarlo antes que dar por buena una corrida dudosa.

## Segunda corrida (corrida 2, 16:43, commit `ead38ffb786cec547a3ff110aeb2aee7d30c1828`)
Las carpetas `corrida_c2/`, `analisis_c2/` y `salida_del_equipo_c2/` son de la corrida 2; las carpetas sin sufijo son de la corrida 1 (commit congelado). El commit de la corrida 2 cambió solo `params/` respecto del congelado.
