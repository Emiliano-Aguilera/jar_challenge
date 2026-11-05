# Notas de la organización sobre su corrida: Zonda_Robotics

Corrida 1 (14:50): **150 puntos** = 50 por 1 víctima (su reporte quedó entre 0,25 y 0,5 m de ella) + 100 de bono de regreso. Reportaron 4 veces; solo 1 reporte quedó dentro del radio de una víctima. `/done` a los 420,3 s.
**Importante:** el regreso lo decide una persona y se registra con un servicio del juez. En esta corrida el juez se cortó con Ctrl+C antes de terminar, sin veredicto de regreso, así que se cargó **después**: `eventos.jsonl` y `resultado.json` tienen un evento `regreso` con `agregado_manual` (sin hora) y un `correccion_manual` con el puntaje original (50) y el corregido (150). `termino_por` está vacío por el mismo motivo. No se registraron colisiones, salida ni vuelco.
