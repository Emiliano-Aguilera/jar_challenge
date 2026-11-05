# Notas de la organización sobre su corrida: CapiBot

Corrida 1 (15:59): **0 puntos**. **La organización abortó la corrida a los ~237 s** porque el robot estaba rompiendo el laberinto. Por eso `resultado.json` no tiene `termino_por` y `juez.log` no tiene la línea «FIN DE LA CORRIDA».
Lo que vemos en los datos:
- **No hubo reportes:** `/victimas` no recibió ningún mensaje en los 237 s (el grabador estaba suscrito desde antes de la largada) y el juez registró 0 reportes. La salida de su programa tampoco muestra ningún reporte.
- Su programa se lanzó como `principal --evasor true` (modo evasor). Casi toda la corrida son ciclos `avanzar → girar` por obstáculos cercanos.
- Desde los ~207 s el robot empezó a pegarse a obstáculos (la salida muestra «obstáculo a 0.10, 0.07, 0.04, 0.03, 0.02 y 0.01 m» seguidos); ahí se produjeron los choques con el laberinto.
- A los ~217 s su programa vio un objeto rojo y pasó a `acercarse`; 10 s después escribió «acercamiento abandonado» y no publicó nada.
No se cargaron colisiones, salida ni vuelco en el juez, porque la corrida se cortó antes de terminar. El equipo no dejó `log.txt` opcional.
