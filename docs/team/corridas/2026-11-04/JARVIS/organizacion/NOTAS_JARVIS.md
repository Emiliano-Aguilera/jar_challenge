# Notas de la organización sobre su corrida: JARVIS

Corrida 1 (13:36): 0 puntos, terminó por tiempo (600 s) sin reportes.
**Lo que vemos:** el robot recorrió 10,1 m en 397 s según `/odom`, pero nunca se alejó más de 0,68 m de la largada. Su localización pasó 9 veces a `RECUPERANDO` con 6 reinicios locales de AMCL; los diagnósticos fueron 7 `SENSORES_VENCIDOS`, 2 `TF_NO_DISPONIBLE` y 1 `DATOS_AUSENTES`. En los tramos donde la localización estuvo `LISTA`, el robot avanzó con normalidad.
**Hipótesis de ustedes:** con los obstáculos del laberinto AMCL no logra localizarse bien. Los datos son compatibles con eso (la localización se perdió 9 veces y casi todo el tiempo perdido fue en recuperaciones), pero desde acá no podemos confirmarlo: `/scan` no está en el bag.
**Del lado nuestro:** probamos otros códigos en el mismo robot y entorno y la localización anduvo bien, así que no parece un problema del entorno. El reloj robot–laptop estuvo en ~20 ms y estable toda la corrida; `/odom` (10 Hz) y `/tf` (20 Hz) llegaron estables.

## Corrida 2 (16:43, commit `ead38ff`, con cambios de código y parámetros respecto del congelado, aceptados por la organización)
**0 puntos.** Reportaron 1 vez (t = 340,9 s) y no puntuó. Su programa publicó `/done` a los 552,2 s. El juez cerró normal. No se registraron colisiones, salida ni vuelco; con 0 puntos por víctimas el bono de regreso no aplicaba. No dejaron `log.txt` opcional.
**Lo que vemos en su salida:** en esta corrida la localización **no se perdió**: 0 pasos a `RECUPERANDO` y 0 `SENSORES_VENCIDOS`, contra 9 y 7 en la corrida 1. Los datos de movimiento de esta corrida están en `analisis_c2/`.

## Para el ranking
Si se mantiene el criterio de la mejor corrida de cada equipo, ambas corridas valen 0 puntos y se desempata por tiempo.
