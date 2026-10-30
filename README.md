# mundos: custom worlds for the simulation server

Never merged and never opened as a PR. One folder per world:

```
mundos/<name>/<name>.world        a Gazebo world (start from a copy of a practice world)
mundos/<name>/models/<model>/     optional extra models (model.config + model.sdf)
```

Run it by putting `<name>` in `mapas` (simular.yml) or `mapa` (sesion.yml). The server checks the
world and draws its map: walls only; `cuadrado_*` boxes and `senuelo_*` decoys are not drawn.
Rules, checks and errors: `docs/team/SIMULATION.md` on `development`, section "Your own worlds".

`ejemplo_senuelos` is maze_1_6x5 plus two decoys: an included model named `senuelo_a`
(`models/caja_prueba`) and a link `senuelo_2` inside `marcas_color`.
