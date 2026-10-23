import math

def cell_value(grid, x, y):
    """Read a world point using the grid's planar origin pose.

    Return None outside the grid or where data is missing. Lightweight test
    grids without an orientation keep the previous zero-rotation behavior.
    """
    origin = grid.info.origin
    q = getattr(origin, 'orientation', None)
    yaw = 0.0 if q is None else math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - origin.position.x, y - origin.position.y
    # Undo the origin translation and rotation before finding the cell.
    i = math.floor((c * dx + s * dy) / grid.info.resolution)
    j = math.floor((-s * dx + c * dy) / grid.info.resolution)
    index = j * grid.info.width + i
    if i < 0 or j < 0 or i >= grid.info.width or j >= grid.info.height or index >= len(grid.data):
        return None
    return grid.data[index]

def is_free(grid, x, y, clearance):
    resolution = grid.info.resolution
    n = math.ceil(clearance / resolution)
    for a in range(-n, n+1):
        for b in range(-n, n+1):
            dx = a * resolution
            dy = b * resolution
            if math.hypot(dx, dy) > clearance:
                continue
            v = cell_value(grid, x + dx, y + dy)
            if v is None or v < 0 or v >= 50:
                return False
    return True

def line_is_clear(grid, x0, y0, x1, y1):
    resolution = grid.info.resolution
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(1, math.ceil(dist / resolution))
    for k in range(n+1):
        x = x0 + (x1 - x0) * k / n
        y = y0 + (y1 - y0) * k / n
        v = cell_value(grid, x, y)
        if v is not None and v >= 50:
            return False
    return True
