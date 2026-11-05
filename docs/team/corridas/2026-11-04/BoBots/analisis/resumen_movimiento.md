# Resumen de movimiento: BoBots

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **410 s**
- Recorrido según `/odom`: **31,1 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **5,27 m**; posición final (1,30; -0,78) m
- `/cmd_vel`: 8199 mensajes, 5604 con velocidad no nula (68 %); velocidad lineal comandada máxima 0,35 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 1,96 | 98 % | 50 % |
| 20–40 | 2,44 | 98 % | 66 % |
| 40–60 | 1,13 | 74 % | 37 % |
| 60–80 | 0,18 | 60 % | 9 % |
| 80–100 | 1,08 | 75 % | 34 % |
| 100–120 | 2,85 | 97 % | 67 % |
| 120–140 | 2,73 | 98 % | 71 % |
| 140–160 | 2,67 | 92 % | 68 % |
| 160–180 | 2,84 | 98 % | 70 % |
| 180–200 | 0,80 | 34 % | 17 % |
| 200–220 | 0,53 | 14 % | 14 % |
| 220–240 | 2,37 | 92 % | 57 % |
| 240–260 | 1,67 | 97 % | 60 % |
| 260–280 | 2,28 | 97 % | 70 % |
| 280–300 | 0,04 | 34 % | 2 % |
| 300–320 | 3,52 | 99 % | 86 % |
| 320–340 | 1,04 | 96 % | 49 % |
| 340–360 | 0,82 | 48 % | 31 % |
| 360–380 | 0,00 | 0 % | 0 % |
| 380–400 | 0,00 | 0 % | 0 % |
| 400–420 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 245,6 | -0,01 | 6,70 |
| 349,7 | 0,50 | 2,69 |
| 349,7 | 1,95 | 5,54 |
| 349,7 | 1,91 | 4,37 |
