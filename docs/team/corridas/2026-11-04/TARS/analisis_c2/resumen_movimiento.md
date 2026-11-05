# Resumen de movimiento: TARS (corrida 2)

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **582 s**
- Recorrido según `/odom`: **50,5 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **6,52 m**; posición final (-0,03; 5,06) m
- `/cmd_vel`: 10417 mensajes, 6910 con velocidad no nula (66 %); velocidad lineal comandada máxima 0,50 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 1,22 | 46 % | 46 % |
| 20–40 | 2,17 | 60 % | 60 % |
| 40–60 | 2,07 | 70 % | 68 % |
| 60–80 | 1,66 | 66 % | 65 % |
| 80–100 | 1,58 | 55 % | 38 % |
| 100–120 | 2,76 | 80 % | 80 % |
| 120–140 | 1,63 | 83 % | 78 % |
| 140–160 | 2,11 | 79 % | 79 % |
| 160–180 | 2,77 | 65 % | 64 % |
| 180–200 | 3,15 | 72 % | 69 % |
| 200–220 | 2,03 | 93 % | 82 % |
| 220–240 | 2,11 | 71 % | 70 % |
| 240–260 | 4,33 | 97 % | 97 % |
| 260–280 | 2,10 | 71 % | 71 % |
| 280–300 | 2,55 | 93 % | 80 % |
| 300–320 | 2,33 | 71 % | 70 % |
| 320–340 | 2,70 | 97 % | 97 % |
| 340–360 | 3,49 | 94 % | 83 % |
| 360–380 | 2,13 | 81 % | 81 % |
| 380–400 | 0,02 | 22 % | 2 % |
| 400–420 | 1,48 | 64 % | 64 % |
| 420–440 | 1,03 | 37 % | 25 % |
| 440–460 | 0,02 | 25 % | 0 % |
| 460–480 | 0,01 | 10 % | 0 % |
| 480–500 | 0,94 | 55 % | 53 % |
| 500–520 | 1,59 | 66 % | 64 % |
| 520–540 | 0,28 | 100 % | 100 % |
| 540–560 | 0,00 | 0 % | 0 % |
| 560–580 | 0,00 | 0 % | 0 % |
| 580–600 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 56,7 | 2,30 | 0,99 |
| 119,1 | 0,01 | 6,70 |
| 146,7 | 2,82 | 5,60 |
