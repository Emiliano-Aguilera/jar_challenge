# Resumen de movimiento: TARS

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **508 s**
- Recorrido según `/odom`: **57,8 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **4,70 m**; posición final (-0,84; 0,80) m
- `/cmd_vel`: 10160 mensajes, 7748 con velocidad no nula (76 %); velocidad lineal comandada máxima 0,50 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 1,13 | 47 % | 47 % |
| 20–40 | 3,10 | 97 % | 97 % |
| 40–60 | 3,33 | 96 % | 94 % |
| 60–80 | 6,30 | 92 % | 92 % |
| 80–100 | 3,92 | 96 % | 96 % |
| 100–120 | 2,13 | 92 % | 92 % |
| 120–140 | 3,21 | 95 % | 95 % |
| 140–160 | 3,56 | 94 % | 94 % |
| 160–180 | 2,51 | 70 % | 68 % |
| 180–200 | 4,57 | 98 % | 97 % |
| 200–220 | 2,40 | 71 % | 70 % |
| 220–240 | 1,39 | 96 % | 96 % |
| 240–260 | 0,78 | 99 % | 85 % |
| 260–280 | 0,29 | 73 % | 70 % |
| 280–300 | 0,94 | 73 % | 69 % |
| 300–320 | 0,70 | 97 % | 97 % |
| 320–340 | 1,16 | 92 % | 78 % |
| 340–360 | 1,23 | 80 % | 80 % |
| 360–380 | 1,61 | 98 % | 97 % |
| 380–400 | 4,62 | 97 % | 96 % |
| 400–420 | 3,37 | 89 % | 89 % |
| 420–440 | 3,92 | 72 % | 72 % |
| 440–460 | 1,33 | 21 % | 21 % |
| 460–480 | 0,00 | 0 % | 0 % |
| 480–500 | 0,00 | 0 % | 0 % |
| 500–520 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 72,2 | 2,23 | 1,00 |
| 447,5 | 0,10 | 3,18 |
