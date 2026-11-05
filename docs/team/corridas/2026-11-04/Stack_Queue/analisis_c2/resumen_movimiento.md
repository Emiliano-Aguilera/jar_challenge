# Resumen de movimiento: Stack_Queue (corrida 2)

Calculado por la organización a partir de `/odom` y `/cmd_vel` del bag. t = 0 es el instante en que el juez publicó `/map`.

- Duración registrada en el bag desde la largada: **282 s**
- Recorrido según `/odom`: **7,9 m** (acumulado; incluye giros, vueltas y deriva de odometría)
- Máxima distancia a la largada: **4,16 m**; posición final (2,37; 3,40) m
- `/cmd_vel`: 1781 mensajes, 906 con velocidad no nula (51 %); velocidad lineal comandada máxima 0,50 m/s
- Huecos de `/odom` mayores a 1 s (t, duración): [(229.6, 5.0)]
- Huecos de `/cmd_vel` mayores a 1 s (t, duración): [(45.1, 10.4), (55.8, 1.7), (64.7, 3.2), (84.0, 1.9), (91.4, 1.7), (96.3, 7.3), (103.8, 1.4), (112.5, 2.5), (130.3, 3.0), (141.1, 12.2)]

## Por ventanas de 20 s

| ventana (s) | recorrido (m) | `/cmd_vel` no nulo | con avance lineal |
| --- | --- | --- | --- |
| 0–20 | 0,00 | 0 % | 0 % |
| 20–40 | 0,05 | 93 % | 0 % |
| 40–60 | 0,67 | 78 % | 52 % |
| 60–80 | 1,40 | 63 % | 37 % |
| 80–100 | 1,10 | 73 % | 44 % |
| 100–120 | 0,69 | 88 % | 43 % |
| 120–140 | 1,42 | 55 % | 38 % |
| 140–160 | 0,31 | 28 % | 28 % |
| 160–180 | 0,96 | 44 % | 44 % |
| 180–200 | 0,00 | 0 % | 0 % |
| 200–220 | 1,04 | 52 % | 52 % |
| 220–240 | 0,17 | 6 % | 6 % |
| 240–260 | 0,00 | 0 % | 0 % |
| 260–280 | 0,00 | 0 % | 0 % |
| 280–300 | 0,00 | 0 % | 0 % |

## Reportes de víctimas

Ninguno.
