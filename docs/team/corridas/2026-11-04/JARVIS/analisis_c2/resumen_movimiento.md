# Resumen de movimiento: JARVIS (corrida 2)

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **612 s**
- Recorrido según `/odom`: **14,9 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **0,82 m**; posición final (-0,09; 0,11) m
- `/cmd_vel`: 5778 mensajes, 4530 con velocidad no nula (78 %); velocidad lineal comandada máxima 0,29 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): ninguno
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,34 | 30 % | 13 % |
| 20–40 | 0,77 | 72 % | 68 % |
| 40–60 | 0,65 | 98 % | 45 % |
| 60–80 | 0,95 | 74 % | 64 % |
| 80–100 | 0,88 | 100 % | 84 % |
| 100–120 | 0,30 | 51 % | 26 % |
| 120–140 | 0,41 | 51 % | 41 % |
| 140–160 | 1,22 | 100 % | 94 % |
| 160–180 | 1,13 | 100 % | 95 % |
| 180–200 | 0,30 | 96 % | 28 % |
| 200–220 | 0,80 | 69 % | 64 % |
| 220–240 | 0,62 | 85 % | 73 % |
| 240–260 | 0,23 | 91 % | 47 % |
| 260–280 | 0,56 | 100 % | 69 % |
| 280–300 | 0,18 | 100 % | 14 % |
| 300–320 | 0,12 | 73 % | 12 % |
| 320–340 | 0,54 | 70 % | 55 % |
| 340–360 | 0,49 | 86 % | 53 % |
| 360–380 | 0,27 | 96 % | 23 % |
| 380–400 | 0,30 | 60 % | 27 % |
| 400–420 | 0,59 | 93 % | 54 % |
| 420–440 | 0,15 | 100 % | 10 % |
| 440–460 | 0,16 | 100 % | 11 % |
| 460–480 | 0,44 | 74 % | 56 % |
| 480–500 | 0,58 | 94 % | 66 % |
| 500–520 | 0,41 | 100 % | 54 % |
| 520–540 | 0,14 | 53 % | 10 % |
| 540–560 | 1,27 | 50 % | 47 % |
| 560–580 | 0,00 | 0 % | 0 % |
| 580–600 | 0,00 | 0 % | 0 % |
| 600–620 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

| t (s) | x | y |
| --- | --- | --- |
| 340,9 | 0,96 | 1,48 |
