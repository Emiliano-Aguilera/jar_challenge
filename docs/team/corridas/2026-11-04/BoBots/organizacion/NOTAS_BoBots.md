# Notas de la organización sobre su corrida: BoBots

Corrida 1 (16:55): **200 puntos** = 100 por 1 víctima (V4) + 100 de bono de regreso. Reportaron 4 veces y 1 puntuó. El primer reporte fue a los 245,6 s; los otros tres llegaron **juntos, en el mismo instante (t = 349,7 s)**, junto con el `/done` (17:02:14).
**Importante:** el regreso lo decide una persona y se registra con un servicio del juez. Se cargó **después** de que el juez cerró, así que `eventos.jsonl` y `resultado.json` tienen un evento `regreso` con `agregado_manual` (sin hora) y un `correccion_manual` con el puntaje original (100) y el corregido (200). No se registraron colisiones, salida ni vuelco. No dejaron `log.txt` opcional.
**Corte tardío:** nos olvidamos de cortar su programa a tiempo. El `/done` fue a las 17:02:14 y su software siguió corriendo hasta las 17:05:39 (unos 3 min más). **Lo que figura en `salida_del_equipo/` después del `/done` no es parte de la corrida**; la dejamos completa, sin recortar.

## Corrida 2 (19:36, commit `c96e6c1`, que cambió solo `params/` respecto del congelado)
**400 puntos** = 300 por 3 víctimas (V2, V3 y V4) + 100 de bono de regreso. Reportaron 4 veces (t = 384,1, 426,4, 489,0, 489,0 s): puntuaron 3, y los dos últimos reportes llegaron **juntos, en el mismo instante**, junto con el `/done` (19:44:53, 489,0 s). El juez se cortó antes de terminar (sin la línea «FIN DE LA CORRIDA» y con `termino_por` vacío), así que el regreso se cargó **después**: `eventos.jsonl` y `resultado.json` de esa corrida tienen un evento `regreso` con `agregado_manual` (sin hora) y un `correccion_manual` con el puntaje original (300) y el corregido (400). No se registraron colisiones, salida ni vuelco. No dejaron `log.txt` opcional.

## Para el ranking
Si se mantiene el criterio de la mejor corrida de cada equipo, cuenta la corrida 2 (400 puntos).
