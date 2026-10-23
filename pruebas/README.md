# Coverage checks

From the repository root, in Ubuntu 22.04 with ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/equipo_jar${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest pruebas/test_cobertura_plan.py pruebas/test_cobertura_node.py -q
python3 pruebas/benchmark_cobertura.py
bash tools/verificar.sh --template
```

The first suite checks generated geometry. The second needs ROS, but no
simulator; it checks waiting for map/TF, fresh localization, late subscribers,
parameter validation, pose corrections, cancellation and live replanning.
`src/equipo_jar/test/test_cobertura.py` runs these suites from `colcon test`,
so the normal verifier checks coverage too. Its wrapper contains no test geometry.
The benchmark measures only the
planner, on generated grids; it is not a navigation or detection benchmark.
Generated maps and test coordinates stay outside `src/`.

## Compare plans on all six practice maps

With the simulator installed and sourced:

```bash
python3 pruebas/evaluar_cobertura.py \
  --simulator "$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)" \
  --output build/verificacion/cobertura/evaluacion-final
```

This reads maps from the simulator install directory. It writes JSON, CSV,
a Markdown comparison and plots under ignored `build/`; it does not copy maps
into source. It evaluates three starts per map and compares the initial rule
against fine candidate recovery plus one bounded 2-opt sweep.

The improved plan retains every initial observation, appends useful views
within the configured budgets, then reorders them only when both estimated
total time and mean first-view time improve. It never drops a view just because
its sampled mask looks redundant: unsampled cells could lose visibility.

Plots show predicted full-grid visibility, sampled residuals, shortest grid
legs and the estimated coverage curve. Residuals distinguish targets outside
the sampled candidate model from targets visible but deferred by the goal
budget. The report also counts targets disconnected before footprint inflation.
Some remaining gaps can come from the conservative circular footprint, the
map itself or the discrete heading model; the report does not call them all
physically impossible.

Runtime measurements are one run per variant and case, alternating variant
order. They include diagnostic path reconstruction. Travel estimates omit
acceleration, localization delays, new obstacles and detector workload.
Return distance is separate. These are planning proxies, not a run-time promise.

## Inspect a plan in simulation

Start the official simulator and the practice judge using the team's simulation
guide. Use `pruebas.launch.py` while localization is missing, so `map → odom`
exists. That identity transform is only a test aid, not localization.

After building and sourcing `install/setup.bash`, in another terminal:

```bash
python3 pruebas/capturar_plan_cobertura.py
```

It subscribes to the map and plan and writes `build/verificacion/cobertura/plan.png`
and a JSON of the observation poses. Plotting uses matplotlib and NumPy, only
for this development tool; the shipped coverage node uses neither.

The numbered arrows are observation goals, not the path Nav2 will drive. Actual
visibility, camera processing, goal success and failed-goal recovery still need
integration tests with mission, navigation and perception.

When stopping a launch from a Python probe, send SIGINT to the launch parent.
ROS launch forwards it to its children. Sending it to the whole process group
first interrupts children twice and can break their cleanup. Use the known
process group only to force-stop your own probe after a shutdown timeout.

## Current mission integration

Coverage publishes an initial route. Mission in current development remembers
finished points, including failed goals, and selects the next pending position
and heading. Coverage does not remove those points or track actual observation.
See `notes/marcorita/INTEGRATION.md` for execution tolerances, deadlines and the
remaining perception/navigation agreements.

For a handoff check, start the official simulator and judge following
`docs/team/SIMULATION.md`, then use separate terminals with the same ROS domain:

```bash
python3 notes/AlsinaBautista/nav_falso.py --ros-args -p duracion:=0.5
ros2 launch equipo_jar pruebas.launch.py use_sim_time:=true victimas:=2
python3 pruebas/capturar_plan_cobertura.py
```

Pass the actual practice world's victim count instead of 2 when needed. Fake
navigation accepts goals but does not move the robot; mission's `Punto actual`
logs show which coverage poses it sends. This check tests communication only.
Use real navigation and localization for travel or competition measurements.
