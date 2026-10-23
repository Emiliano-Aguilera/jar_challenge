# Shared map: origin coordinates

2026-10-22. Local branch: `cobertura/mapa-coordenadas`, based on development
`c01ccdcb0a4d2563742d8bdbbabfb7f712b1f575`. Marco authorized changes to
`mapa.py` after coordinating with Bautista. This entry describes the first
change, not the complete shared-geometry refactor.

## Question and decision

`cell_value(grid, x, y)` subtracted the origin position but ignored the origin
orientation. On a rotated map it could return another cell or `None` for an
inside point. Inspection calls `is_free` and `line_is_clear`, which both call
`cell_value`, so this also affected mission's inspection positions.

The function now subtracts the origin translation and applies the inverse planar
rotation before dividing by resolution and rounding down to a cell. As with
normal planar ROS map poses, the quaternion is expected to be valid and unit
length. Roll/pitch map projection and malformed metadata are outside this change.

## Contract and effects for Bautista

- The function name, positional arguments and return values are unchanged:
  occupancy value for an available cell; `None` outside the grid or where the
  data array has no entry. An unknown cell still returns `-1`.
- Existing lightweight fake grids without `origin.orientation` are interpreted
  as zero rotation. The existing mission test fixtures still work unchanged.
- No changes are required to `inspeccion.py`, `mision.py` or their imports.
- On zero-rotation maps, the coordinate calculation is equivalent to the old
  calculation. On rotated maps, inspection now consults the intended cells,
  which can change which viewpoints are accepted or rejected.
- A new regression exercises mission's existing `viewpoint_for` call on a
  translated, rotated map and checks the chosen position and heading.

## Effects for coverage

- The existing local coverage planner has its own correct coordinate conversion.
  Its implementation, route selection and ROS adapter have not changed here.
- This establishes the same coordinate convention for the shared helper. Moving
  coverage onto shared geometry remains a separate change, with parity checks.
- Local coverage work remains in its original checkout and branch. This worktree
  contains only the map change and starts from current development.

## Policies deliberately retained for this first change

- `is_free`: occupied threshold 50; unknown and outside block; the existing
  sampled clearance calculation remains in use.
- `line_is_clear`: occupied threshold 50; unknown and outside do not block;
  the existing sampled line traversal remains in use.
- Parameters, ROS messages, topics and module ownership are unchanged.

Complete-cell clearance, corner-inclusive ray traversal and explicit occupancy/
unknown policies are follow-up changes. In particular, this correction does not
make the current clearance or visibility sampling conservative.

## Verification

- New regression suite: 13 cases. Before the fix: 10 failed, 3 passed. After the
  fix: 13 passed. The suite covers positive/negative quarter turns, a half turn,
  another angle, row-major lookup, transformed map bounds, missing data, unknown
  cells, legacy fake grids, wall queries and mission inspection.
- The 74 existing map, inspection, exploration and confirmation checks pass.
- The new colcon-discovered wrapper also passes when invoked by pytest. It runs
  the 13 geometry cases in a subprocess, keeping generated geometry outside src.
- Combined pytest invocation: **88 passed** (74 existing + 13 direct regressions
  + 1 wrapper). The wrapper repeats the regression suite internally.
- Official structure (`--template`), parameter and static checks pass. Static
  check: zero errors and zero hardcoding warnings.
- Full local verifier (`bash tools/verificar.sh --template`) passed: both packages
  built, **113 colcon checks passed**, and the map-waiting runtime audit found
  four team nodes and zero interface problems. The 113 entries include the wrapper
  that runs the 13 geometry cases internally.
- One official simulator smoke run on `maze_1_6x5`, limited to 20 s after the map,
  exited with code 0. Four team nodes, zero interface problems, no judge violations.
  It ended by time with zero reports and zero points. Development does not yet
  include working navigation or our local coverage planner, so this is startup
  and interface evidence, not a successful autonomous search.
- The runner's group shutdown produced `ExternalShutdownException` traces. Its
  plot command also failed because the checkout path contains spaces. Neither is
  changed by this patch; the result validator passed and JSON/logs were saved.
- Evidence is under ignored `build/verificacion/`: `mapa-verificador.log`,
  `mapa-simulador.log`, and `maze_1_6x5/resultado.json` plus process logs.

## Local environment used for the complete checks

The first complete run failed importing the system OpenCV with user-installed
NumPy 2.2.6. The repeated run isolated the dependencies: `PYTHONNOUSERSITE=1`,
system NumPy 1.21.5, and the repo's pinned `opencv-python-headless==4.9.0.80`
installed with `--no-deps` into a temporary directory added to `PYTHONPATH`.
The verifier then passed. No shared dependency file or user package was changed.
The isolated packages currently live at `/tmp/tars-mapa-deps-c01ccdc` in WSL;
recreate them from `requirements.txt` if that temporary directory is removed.

From the full source checkout, in the Ubuntu environment:

```bash
PYTHONPATH=src/equipo_jar python3 -m pytest \
  pruebas/test_mapa_coordenadas.py \
  src/equipo_jar/test/test_mapa.py \
  src/equipo_jar/test/test_inspeccion.py \
  src/equipo_jar/test/test_exploracion.py \
  src/equipo_jar/test/test_confirmacion.py \
  src/equipo_jar/test/test_mapa_geometria_suite.py -q
python3 tools/verificar_estructura.py --template
python3 tools/verificar_params.py
python3 tools/verificar_estatico.py
```

## Next small change

Replace point-sampled clearance with circle-versus-cell-area checks, keeping
the public API and testing which inspection viewpoints change. Document those
behavior changes before moving on to visibility policies and coverage reuse.
