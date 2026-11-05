# Resumen de movimiento: JARVIS

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **600 s**
- Recorrido según `/odom`: **16,3 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **1,07 m**; posición final (0,98; 0,44) m
- `/cmd_vel`: 6001 mensajes, 3250 con velocidad no nula (54 %); velocidad lineal comandada máxima 0,24 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): [(230.3, 2.4)]
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): ninguno

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,33 | 43 % | 23 % |
| 20–40 | 0,00 | 1 % | 0 % |
| 40–60 | 0,02 | 18 % | 0 % |
| 60–80 | 0,65 | 32 % | 27 % |
| 80–100 | 1,00 | 98 % | 98 % |
| 100–120 | 0,75 | 64 % | 56 % |
| 120–140 | 0,01 | 10 % | 0 % |
| 140–160 | 0,82 | 53 % | 51 % |
| 160–180 | 0,06 | 8 % | 6 % |
| 180–200 | 0,28 | 23 % | 16 % |
| 200–220 | 1,19 | 100 % | 99 % |
| 220–240 | 0,97 | 50 % | 50 % |
| 240–260 | 0,07 | 15 % | 15 % |
| 260–280 | 0,29 | 34 % | 33 % |
| 280–300 | 0,72 | 37 % | 37 % |
| 300–320 | 0,01 | 3 % | 0 % |
| 320–340 | 0,03 | 15 % | 7 % |
| 340–360 | 1,14 | 100 % | 97 % |
| 360–380 | 0,69 | 99 % | 88 % |
| 380–400 | 1,26 | 98 % | 94 % |
| 400–420 | 1,11 | 98 % | 97 % |
| 420–440 | 0,76 | 84 % | 80 % |
| 440–460 | 0,51 | 98 % | 77 % |
| 460–480 | 0,91 | 100 % | 97 % |
| 480–500 | 1,50 | 92 % | 92 % |
| 500–520 | 0,52 | 100 % | 50 % |
| 520–540 | 0,46 | 100 % | 38 % |
| 540–560 | 0,16 | 38 % | 19 % |
| 560–580 | 0,00 | 6 % | 6 % |
| 580–600 | 0,00 | 7 % | 7 % |

## Reportes de víctimas

Ninguno.
