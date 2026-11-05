# Resumen de movimiento: BoBots (corrida 2)

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **524 s**
- Recorrido según `/odom`: **43,2 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **6,06 m**; posición final (2,41; -0,34) m
- `/cmd_vel`: 10349 mensajes, 8451 con velocidad no nula (82 %); velocidad lineal comandada máxima 0,35 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 1,83 | 98 % | 46 % |
| 20–40 | 1,52 | 89 % | 69 % |
| 40–60 | 0,79 | 82 % | 38 % |
| 60–80 | 3,03 | 88 % | 72 % |
| 80–100 | 1,83 | 96 % | 55 % |
| 100–120 | 1,46 | 86 % | 55 % |
| 120–140 | 1,64 | 96 % | 70 % |
| 140–160 | 1,40 | 96 % | 46 % |
| 160–180 | 1,81 | 94 % | 57 % |
| 180–200 | 3,51 | 100 % | 94 % |
| 200–220 | 1,95 | 88 % | 57 % |
| 220–240 | 0,34 | 36 % | 34 % |
| 240–260 | 0,03 | 55 % | 44 % |
| 260–280 | 0,02 | 66 % | 55 % |
| 280–300 | 0,24 | 53 % | 32 % |
| 300–320 | 0,78 | 81 % | 52 % |
| 320–340 | 1,73 | 91 % | 53 % |
| 340–360 | 2,63 | 98 % | 70 % |
| 360–380 | 1,68 | 96 % | 42 % |
| 380–400 | 1,67 | 96 % | 54 % |
| 400–420 | 2,25 | 94 % | 94 % |
| 420–440 | 2,21 | 91 % | 70 % |
| 440–460 | 2,98 | 97 % | 96 % |
| 460–480 | 4,30 | 100 % | 98 % |
| 480–500 | 1,33 | 44 % | 44 % |
| 500–520 | 0,00 | 0 % | 0 % |
| 520–540 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 384,1 | 2,34 | 1,01 |
| 426,4 | -0,04 | 6,69 |
| 489,0 | 2,79 | 5,55 |
| 489,0 | 2,15 | 1,91 |
