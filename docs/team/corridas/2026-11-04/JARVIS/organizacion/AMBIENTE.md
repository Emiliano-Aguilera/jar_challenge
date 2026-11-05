# Cómo corrimos las pruebas (información de la organización)

## Condiciones
- Arena: `laberinto_2026-11-04_qualy`, 4 víctimas. `/map` se publica al largar y arranca el cronómetro.
- Su código corrió **en la laptop de la organización** (ROS 2 Humble, `ROS_DOMAIN_ID=11`, sin Docker), con el comando `ros2 launch equipo_jar competencia.launch.py victimas:=4`, usando el commit congelado del cierre de las 9:00 (PR en `LEEME.md`).
- El robot es `donatello` (Raspberry Pi; lidar `sllidar_node`, cámara Astra `cam_1`). Habla con la laptop por **wifi** («Robotica-Mecatronica»); la laptop está por cable al mismo router. Todo `BEST_EFFORT`.
- El juez es el `juez_mock` del repo; colisiones, salida, vuelco y regreso los decide una persona mirando la pista.

## Puntaje (del juez)
- Víctima: 100 puntos si el reporte queda a ≤ 0.25 m; 50 si queda a ≤ 0.5 m.
- Bono de 200 si se reportan todas antes de 300 s. Bono de regreso: +100 (círculo de 0.25 m, robot detenido, tras `/done`; el veredicto es humano y debe darse dentro de 60 s de `/done`).
- Colisión −50 c/u; salida del laberinto −50 (y termina la corrida); vuelco anula. El total no baja de 0.

## Qué medimos del entorno (13:36–13:48)
- Reloj robot–laptop: ~20 ms, estable. El robot **no tiene pila de reloj ni NTP** (la red no sale a internet), por eso lo ponemos en hora a mano cada vez que se cambia la batería.
- `ping` laptop→robot: 0 % de pérdida, 1,5–84 ms (promedio 14,5 ms).
- `/odom` ~10 Hz y `/tf` ~20 Hz estables; un único hueco de 2,4 s en `/odom`.

## Incidentes de la sesión que pudieron afectar corridas
- **Reloj del robot atrasado** tras un cambio de batería: provocó una corrida anulada y repetida (falla nuestra; no descontó cupo). Desde entonces se verifica antes de cada equipo.
- **ROS desactualizado hasta las 12:43** (`tf2_ros` 0.25.20): rompía código con `TransformListener(buffer, None, …)`. Corridas anteriores a esa hora pudieron verse afectadas.
- **Nuevo cambio de batería** antes de las 14:09: reloj corregido a las 14:09 (−0,022 s) antes de las dos últimas corridas.
- Las largadas con `/juez/iniciar` a veces no devuelven respuesta aunque se hagan; no afecta la corrida.
