# Notas de la organización sobre su corrida: TARS

Corrida oficial 1 (13:19): **250 puntos** = 2 víctimas (200) + bono de regreso (+100) − 1 colisión (−50).
**Importante:** el regreso y la colisión los decide una persona y se registran con servicios del juez. Se cargaron después de que el juez cerró (se pasó la ventana de 60 s desde `/done`), así que `eventos.jsonl` y `resultado.json` tienen eventos `agregado_manual` y un `correccion_manual` con el puntaje original (200) y el corregido (250). Los eventos agregados no tienen hora.
En `logs_opcionales_del_equipo/` está el `log.txt` opcional de esta corrida, que se comparte por reglamento.

## Corrida 2 (18:51, commit `cec2c37`, que cambió solo `params/` respecto del congelado)
**300 puntos por víctimas (sin penalizaciones cargadas).** Reportaron 3 veces (t = 56,7, 119,1, 146,7 s) y las 3 puntuaron (V2, V4 y V3, 100 c/u). **La organización cortó la corrida a los ~521 s porque el robot estaba rompiendo el laberinto**; el programa no llegó a publicar `/done`, y el juez cerró por `inmovil` a los 581,6 s. No hay veredicto de regreso. No se cargaron colisiones, salida ni vuelco. Su `log.txt` opcional de esta corrida está en `logs_opcionales_del_equipo_c2/`.

## Para el ranking
Si se mantiene el criterio de la mejor corrida de cada equipo, la corrida 2 aparece con 300 puntos; el puntaje final de esta corrida lo confirma la organización.
