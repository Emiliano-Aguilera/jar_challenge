import math
from equipo_jar.mapa import is_free, line_is_clear
from equipo_jar.confirmacion import near_any

def viewpoint_for(grid, cx, cy, robot_x, robot_y, distance, clearance, n_angles, min_shift, box_margin,
                cell_grid, reachable, blocked=None):
    # blocked(x, y): optional; True skips that spot (mission: occupied in Nav2's costmap).
    best = None
    best_dist = None
    for k in range(n_angles):
        angle = 2 * math.pi * k / n_angles
        c = math.cos(angle)
        s = math.sin(angle)
        vx = cx + distance * c  
        vy = cy + distance * s
        ex = cx + box_margin * c
        ey = cy + box_margin * s
        if not is_free(grid, vx, vy, clearance):
            continue
        if not line_is_clear(grid, vx, vy, ex, ey):
            continue
        if cell_grid.cell(vx, vy) not in reachable:
            continue
        if blocked is not None and blocked(vx, vy):
            continue
        d = math.hypot(vx - robot_x, vy - robot_y)
        if d < min_shift:
            continue
        if best is None or d < best_dist:
            yaw = math.atan2(cy - vy, cx - vx)
            best = (vx, vy, yaw)
            best_dist = d
    return best

def pick_doubtful(cands, reported_ids, reported_points, inspected_points, min_obs, dist_min, robot_x, robot_y, failed, max_attempts):
    best = None
    best_dist = None
    remaining = []
    for c in cands:
        if c.id in reported_ids or near_any(c.centro.x, c.centro.y, reported_points, dist_min):
            continue
        if near_any(c.centro.x, c.centro.y, inspected_points, dist_min):
            continue
        if not math.isfinite(c.centro.x) or not math.isfinite(c.centro.y):
            continue
        if c.observaciones < min_obs:
            continue
        if c.id in failed:
            last_obs, attempts = failed[c.id]
            if attempts >= max_attempts or c.observaciones <= last_obs:
                continue
        remaining.append(c)
    for c in remaining:
        dist = math.hypot(c.centro.x - robot_x, c.centro.y - robot_y)
        if best is None or dist < best_dist:
            best = c
            best_dist = dist
    return best

def forget_failed_inspection(inspected_points, failed, cand_id, x, y, observations):
    """Nav2 did not reach the view: the candidate is not inspected, and the attempt counts.

    Mission marks a candidate inspected when it sends the trip (no loops while it is
    on its way). If the trip fails, this takes it back out of `inspected_points` and
    adds a failed attempt with its observations at the time: `pick_doubtful` tries it
    again only with new observations and attempts left. Returns the attempt count.
    """
    if (x, y) in inspected_points:
        inspected_points.remove((x, y))
    _, attempts = failed.get(cand_id, (0, 0))
    failed[cand_id] = (observations, attempts + 1)
    return attempts + 1

def unhelpful_inspection(inspected_points, failed, cand_id, x, y):
    """Nav2 reached the view, but after the wait the candidate is still not reported.

    Like a failed trip: it is not inspected anymore and the attempt counts. But the retry
    does not need new observations: the next view is another spot (`viewpoint_for` skips
    spots near the robot, and the robot is at this one), so it can give fusion what this
    one did not. It is stored as 0 observations, and every candidate `pick_doubtful`
    looks at has more. Returns the attempt count.
    """
    return forget_failed_inspection(inspected_points, failed, cand_id, x, y, 0)

def inspection_outcome(cands, cand_id, reported_ids, reported_points, dist_min, now, look_until):
    """What happened to the candidate of an inspection that arrived, while mission waits.

    'reported': reported by id, or another id was reported at the same box.
    'gone': fusion dropped it or merged it into another one.
    'looking': still doubtful, and `now` is before `look_until`.
    'unhelpful': still doubtful, and the wait is over.
    """
    if cand_id in reported_ids:
        return 'reported'
    current = None
    for c in cands or []:
        if c.id == cand_id:
            current = c
    if current is None:
        return 'gone'
    if near_any(current.centro.x, current.centro.y, reported_points, dist_min):
        return 'reported'
    if now < look_until:
        return 'looking'
    return 'unhelpful'
