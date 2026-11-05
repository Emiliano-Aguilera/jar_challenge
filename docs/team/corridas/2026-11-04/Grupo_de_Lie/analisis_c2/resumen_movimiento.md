# Resumen de movimiento: Grupo_de_Lie (corrida 2)

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **596 s**
- Recorrido según `/odom`: **35,9 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **4,62 m**; posición final (0,22; -0,39) m
- `/cmd_vel`: 9382 mensajes, 7346 con velocidad no nula (78 %); velocidad lineal comandada máxima 0,35 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): [(12.9, 1.1), (27.2, 1.1), (28.3, 1.5), (42.7, 1.5), (48.8, 1.5), (51.5, 1.5), (57.6, 1.4), (62.9, 1.5), (82.3, 1.4), (104.6, 1.1), (105.7, 1.5), (108.4, 1.5)] … (43 en total)

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,54 | 99 % | 31 % |
| 20–40 | 0,99 | 97 % | 48 % |
| 40–60 | 1,18 | 96 % | 48 % |
| 60–80 | 0,81 | 78 % | 23 % |
| 80–100 | 1,33 | 95 % | 46 % |
| 100–120 | 1,09 | 98 % | 70 % |
| 120–140 | 0,64 | 98 % | 45 % |
| 140–160 | 1,26 | 99 % | 61 % |
| 160–180 | 0,56 | 88 % | 51 % |
| 180–200 | 2,12 | 68 % | 54 % |
| 200–220 | 0,00 | 2 % | 2 % |
| 220–240 | 0,70 | 90 % | 35 % |
| 240–260 | 2,23 | 75 % | 73 % |
| 260–280 | 0,39 | 64 % | 17 % |
| 280–300 | 1,63 | 87 % | 55 % |
| 300–320 | 0,95 | 78 % | 38 % |
| 320–340 | 0,93 | 98 % | 74 % |
| 340–360 | 2,45 | 97 % | 81 % |
| 360–380 | 0,60 | 19 % | 19 % |
| 380–400 | 1,71 | 47 % | 45 % |
| 400–420 | 0,74 | 23 % | 23 % |
| 420–440 | 2,85 | 85 % | 85 % |
| 440–460 | 1,96 | 89 % | 67 % |
| 460–480 | 2,48 | 88 % | 70 % |
| 480–500 | 1,60 | 94 % | 52 % |
| 500–520 | 1,35 | 98 % | 44 % |
| 520–540 | 2,28 | 97 % | 68 % |
| 540–560 | 0,22 | 75 % | 50 % |
| 560–580 | 0,00 | 0 % | 0 % |
| 580–600 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 108,8 | 0,28 | 3,76 |
| 239,3 | 2,23 | 1,06 |
| 346,5 | 1,15 | 5,34 |
| 346,5 | 1,37 | -0,16 |
