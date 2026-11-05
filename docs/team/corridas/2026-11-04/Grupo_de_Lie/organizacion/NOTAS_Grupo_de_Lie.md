# Notas de la organización sobre su corrida: Grupo_de_Lie

Corrida 1 (15:39): **500 puntos** = 4 víctimas (400) + bono de regreso (+100). Los 4 reportes puntuaron (100 c/u), en t = 59,2, 199,7, 338,3, 453,3 s. `/done` a los 502.8 s (15:49:24); no alcanzaron el bono de 5 min (300 s) porque el último reporte fue a los 453.3 s.
El veredicto de regreso se dio **en vivo** (15:49:46, 22 s después del `/done`), así que `resultado.json` ya lo trae sin correcciones. No hubo colisiones, salida ni vuelco.
**Corte tardío:** nos olvidamos de cortar su programa a tiempo. El `/done` fue a las 15:49:24 y su software siguió corriendo hasta las 15:52:47 (unos 3 min más). **Lo que figura en `salida_del_equipo/` y en su `log.txt` después del `/done` no es parte de la corrida**; los dejamos completos, sin recortar. Su `log.txt` opcional está en `logs_opcionales_del_equipo/`.

## Corrida 2 (18:28, commit `aaf96df`, que cambió solo `params/` respecto del congelado)
**0 puntos.** Reportaron 4 veces (t = 108,8, 239,3, 346,5, 346,5 s) y 1 puntuó (V2, 100). `/done` a los 540,9 s y regreso dado en vivo (+100). Se cargaron **después** del cierre del juez **5 colisiones** (−250): 100 + 100 − 250 = −50, y el juez no baja de 0. Por eso `eventos.jsonl` y `resultado.json` de esa corrida tienen 5 eventos `colision` con `agregado_manual` (sin hora) y un `correccion_manual` con el puntaje original (200) y el corregido (0). Su `log.txt` opcional de esta corrida está en `logs_opcionales_del_equipo_c2/`.

## Para el ranking
Si se mantiene el criterio de la mejor corrida de cada equipo, cuenta la corrida 1 (500 puntos).
