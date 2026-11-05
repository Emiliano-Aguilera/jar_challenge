# Resumen de movimiento: Stack_Queue

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **426 s**
- Recorrido según `/odom`: **20,4 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **7,46 m**; posición final (-3,95; 3,67) m
- `/cmd_vel`: 3643 mensajes, 2176 con velocidad no nula (60 %); velocidad lineal comandada máxima 0,50 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): [(40.2, 5.0), (365.4, 5.0), (370.6, 3.4), (374.0, 1.2), (375.2, 1.1)]
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): [(39.6, 10.3), (50.3, 21.8), (86.2, 4.0), (97.8, 4.2), (116.6, 1.9), (124.4, 8.9), (141.2, 1.4), (154.7, 3.7), (188.2, 7.7), (207.8, 7.4), (234.8, 2.3), (257.4, 1.6)] … (19 en total)

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,00 | 0 % | 0 % |
| 20–40 | 0,07 | 87 % | 0 % |
| 40–60 | 0,00 | 0 % | 0 % |
| 60–80 | 0,10 | 65 % | 58 % |
| 80–100 | 0,49 | 44 % | 25 % |
| 100–120 | 1,91 | 67 % | 48 % |
| 120–140 | 1,16 | 79 % | 47 % |
| 140–160 | 2,78 | 92 % | 82 % |
| 160–180 | 1,31 | 79 % | 42 % |
| 180–200 | 0,04 | 74 % | 0 % |
| 200–220 | 1,60 | 80 % | 70 % |
| 220–240 | 0,33 | 26 % | 26 % |
| 240–260 | 0,74 | 49 % | 49 % |
| 260–280 | 0,68 | 53 % | 39 % |
| 280–300 | 1,17 | 35 % | 28 % |
| 300–320 | 0,07 | 3 % | 3 % |
| 320–340 | 0,92 | 43 % | 20 % |
| 340–360 | 6,04 | 92 % | 87 % |
| 360–380 | 0,86 | 68 % | 68 % |
| 380–400 | 0,00 | 0 % | 0 % |
| 400–420 | 0,00 | 0 % | 0 % |
| 420–440 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 143,8 | 2,24 | 0,99 |
| 365,4 | 0,18 | 4,38 |
| 365,4 | 0,08 | 3,62 |
| 365,4 | 0,14 | 2,62 |
