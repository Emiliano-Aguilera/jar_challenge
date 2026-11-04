"""Run our coverage planner (cobertura_plan.make_plan) offline on a real map, per params variant.

Prints views, predicted visible samples, the plan's own time model, a real-robot time estimate
(qualification run 1 fit: 4.0 s + 2.8 s per straight-line meter per trip, 2.25 s per turn in
place, plus ~10 s to compute the plan) and when each given point (a victim) is first in view.

Usage, from the repo root:
  python3 docs/team/ajuste/scripts/coverage_on_map.py MAP.yaml src/equipo_jar/params/params.yaml \
      '{"base": {}, "transit 0.18": {"robot_radius": 0.15}}' --start 0.03 0.005 0.0 --point 0.08 6.71
Each JSON value overrides `cobertura` params from the params file.
"""
import argparse
import json
import math
import os
import sys
from dataclasses import asdict

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
sys.path.insert(0, os.path.join(ROOT, 'src/equipo_jar'))
sys.path.insert(0, os.path.dirname(__file__))
from equipo_jar.cobertura_plan import Camera, Settings, make_plan, visibility_mask  # noqa: E402
from equipo_jar.mapa import Grid, Pose  # noqa: E402
from mapfile import load_map  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('map'); ap.add_argument('params'); ap.add_argument('variants')
ap.add_argument('--start', nargs=3, type=float, default=[0.0, 0.0, 0.0])
ap.add_argument('--point', nargs=2, type=float, action='append', default=[])
a = ap.parse_args()

occ, res, ox, oy = load_map(a.map)
h, w = occ.shape
grid = Grid(w, h, res, [100 if v else 0 for v in occ.ravel()], Pose(ox, oy, 0.0))
base = yaml.safe_load(open(a.params))['cobertura']['ros__parameters']
base = {k: base[k] for k in asdict(Settings()) if k in base}
for name, override in json.loads(a.variants).items():
    s = Settings(**{**base, **override})
    plan = make_plan(grid, Pose(*a.start), s, Camera(), diagnostics=True)
    real, prev, first = 10.0, (a.start[0], a.start[1]), {}
    for pose, leg in zip(plan.poses, plan.diagnostics.legs):
        length = sum(math.dist(p, q) for p, q in zip(leg, leg[1:])) * res
        real += 2.25 if math.dist(prev, (pose.x, pose.y)) < 0.20 else 4.0 + 2.8 * length * 0.85
        prev = (pose.x, pose.y)
        for k, (px, py) in enumerate(a.point):
            if k not in first and visibility_mask(grid, grid.cell(pose.x, pose.y), pose.yaw, [(px, py)], s, Camera()):
                first[k] = round(real)
    seen = ' '.join(f'({px:.2f},{py:.2f}):{first.get(k, "never")}' for k, (px, py) in enumerate(a.point))
    print(f'{name}: transit {s.robot_radius + s.safety_margin:.2f} m, {len(plan.poses)} views, '
          f'visible {plan.predicted_count}/{plan.target_count}, plan model {plan.metrics.duration:.0f} s, '
          f'real estimate {real:.0f} s' + (f' | first in view (s): {seen}' if seen else ''), flush=True)
