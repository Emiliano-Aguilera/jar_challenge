# Resumen de movimiento: Zonda_Robotics

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **448 s**
- Recorrido según `/odom`: **29,7 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **6,58 m**; posición final (0,56; -0,01) m
- `/cmd_vel`: 3412 mensajes, 2114 con velocidad no nula (62 %); velocidad lineal comandada máxima 0,30 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): [(7.3, 3.4), (17.1, 2.9), (27.5, 2.9), (34.9, 3.4), (53.9, 2.9), (64.7, 3.4), (143.5, 3.4), (293.1, 1.1), (330.9, 1.0), (418.1, 15.7)]

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,82 | 93 % | 43 % |
| 20–40 | 1,87 | 97 % | 78 % |
| 40–60 | 3,19 | 97 % | 90 % |
| 60–80 | 1,12 | 91 % | 73 % |
| 80–100 | 1,63 | 78 % | 67 % |
| 100–120 | 0,39 | 38 % | 32 % |
| 120–140 | 0,47 | 29 % | 27 % |
| 140–160 | 0,31 | 76 % | 68 % |
| 160–180 | 0,62 | 22 % | 19 % |
| 180–200 | 0,36 | 1 % | 1 % |
| 200–220 | 0,54 | 7 % | 3 % |
| 220–240 | 1,79 | 70 % | 67 % |
| 240–260 | 1,83 | 64 % | 63 % |
| 260–280 | 0,36 | 33 % | 33 % |
| 280–300 | 1,39 | 67 % | 64 % |
| 300–320 | 0,27 | 27 % | 27 % |
| 320–340 | 2,34 | 97 % | 95 % |
| 340–360 | 2,39 | 68 % | 67 % |
| 360–380 | 0,49 | 84 % | 84 % |
| 380–400 | 3,48 | 98 % | 93 % |
| 400–420 | 3,91 | 96 % | 96 % |
| 420–440 | 0,00 | 0 % | 0 % |
| 440–460 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 47,2 | 1,92 | 4,30 |
| 94,4 | 0,14 | 6,46 |
| 335,4 | 2,89 | 4,31 |
| 383,5 | 2,54 | 6,38 |
