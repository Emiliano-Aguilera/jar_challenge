# Notas de la organización sobre su corrida: Stack_Queue

Corrida 1 (15:07): **150 puntos** = 150 por víctimas (V2: 100, V1: 50) + 100 de bono de regreso − 100 por 2 colisiones. Cerraron con `/done` a los 365,8 s.
Reportaron 4 veces y 2 puntuaron. El primer reporte (t = 143,8 s) acertó V2; los otros tres llegaron **juntos, en el mismo instante (t = 365,4 s)**, justo antes del `/done`.
**Importante:** el regreso y las colisiones los decide una persona y se cargaron **después** de que el juez cerró, así que `eventos.jsonl` y `resultado.json` tienen eventos `regreso` y `colision` con `agregado_manual` (sin hora) y un `correccion_manual` con el puntaje original (150) y el corregido (150: el +100 del regreso y el −100 de las colisiones se compensan, el total no cambia). No hubo salida ni vuelco.
**Corte tardío:** nos olvidamos de cortar su programa a tiempo. El `/done` fue a las 15:14:12 y su software siguió corriendo hasta las 15:22:11 (unos 8 min más). **Lo que figura en `salida_del_equipo/` y en su `log.txt` después del `/done` no es parte de la corrida**; los dejamos completos, sin recortar. Su `log.txt` opcional está en `logs_opcionales_del_equipo/`.

## Corrida 2 (17:55, commit `72f577e`, que cambió solo `params/` respecto del congelado)
**0 puntos, sin reportes.** El juez cerró por `inmovil` a los 281,6 s. Según el bag, el robot recorrió 7,9 m (hasta 4,2 m de la largada) y su programa comandó movimiento hasta los 221 s; después el `/cmd_vel` quedó nulo.
**Durante esta corrida hubo problemas con la red del router:** el bag tiene huecos de más de 2 s en `/cmd_vel` y `/tf` y uno de 5 s en `/odom` (a los 229,6 s). La organización puede repetirla; si pasa, les avisamos. Su `log.txt` opcional está en `logs_opcionales_del_equipo_c2/`.

## Para el ranking
Si se mantiene el criterio de la mejor corrida de cada equipo, cuenta la corrida 1 (150 puntos).
