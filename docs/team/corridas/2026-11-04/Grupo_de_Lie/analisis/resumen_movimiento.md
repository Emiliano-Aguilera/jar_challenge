# Resumen de movimiento: Grupo_de_Lie

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **525 s**
- Recorrido según `/odom`: **37,2 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **6,98 m**; posición final (0,88; 0,02) m
- `/cmd_vel`: 8992 mensajes, 8595 con velocidad no nula (96 %); velocidad lineal comandada máxima 0,35 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): [(11.3, 1.5), (26.4, 1.5), (46.5, 1.5), (65.1, 1.5), (91.1, 1.5), (100.9, 1.5), (124.8, 1.5), (149.9, 1.6), (167.5, 1.6), (178.8, 1.6), (181.7, 1.5), (183.6, 1.6)] … (28 en total)

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,63 | 99 % | 17 % |
| 20–40 | 1,64 | 99 % | 38 % |
| 40–60 | 1,88 | 99 % | 43 % |
| 60–80 | 2,45 | 99 % | 64 % |
| 80–100 | 1,15 | 93 % | 34 % |
| 100–120 | 0,24 | 82 % | 9 % |
| 120–140 | 1,49 | 99 % | 46 % |
| 140–160 | 0,35 | 85 % | 14 % |
| 160–180 | 1,46 | 64 % | 50 % |
| 180–200 | 2,10 | 96 % | 72 % |
| 200–220 | 0,91 | 99 % | 24 % |
| 220–240 | 1,08 | 99 % | 36 % |
| 240–260 | 2,02 | 97 % | 68 % |
| 260–280 | 0,12 | 98 % | 18 % |
| 280–300 | 1,36 | 99 % | 38 % |
| 300–320 | 0,40 | 99 % | 17 % |
| 320–340 | 0,64 | 98 % | 41 % |
| 340–360 | 0,87 | 95 % | 33 % |
| 360–380 | 0,36 | 98 % | 27 % |
| 380–400 | 1,81 | 99 % | 57 % |
| 400–420 | 3,38 | 99 % | 76 % |
| 420–440 | 2,78 | 100 % | 69 % |
| 440–460 | 2,36 | 100 % | 50 % |
| 460–480 | 2,16 | 99 % | 64 % |
| 480–500 | 3,35 | 94 % | 78 % |
| 500–520 | 0,09 | 96 % | 55 % |
| 520–540 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 59,2 | 2,23 | 1,06 |
| 199,7 | 0,08 | 6,71 |
| 338,3 | 2,77 | 5,57 |
| 453,3 | 0,18 | 3,14 |
