from concurrent.futures import ThreadPoolExecutor
import math
from threading import Event

from rclpy.time import Time
from tf2_ros import TransformException

from equipo_jar.cobertura import camera_from_transform
from equipo_jar.cobertura_plan import Settings
from equipo_jar.cobertura_repaso import propose_extra_views
from equipo_jar.exploracion import near_box, yaw_from_quaternion, angle_difference
from equipo_jar.historial_vistas import AttemptedView
from equipo_jar.mapa import Grid, Pose
from equipo_jar.presupuesto import fits_extra, return_due, route_seconds
from equipo_jar.rutas_presupuesto import build_route_field, estimate_route


def geometry(grid):
    return grid.width, grid.height, grid.resolution, grid.origin


def same_grid(a, b):
    return a is not None and b is not None and geometry(a) == geometry(b) and a.data == b.data


class ExtraSearch:
    def __init__(self, node):
        self.node = node
        self.executor = None
        self.provider = propose_extra_views
        self.epoch = 0
        self.cancel = Event()
        self.job = None
        self.live = None
        self.optics = None
        self.live_at = None
        self.field = None
        self.views = None
        self.attempts = ()
        self.requested = None
        self.active = False
        self.phase = 'primary'
        self.accepted_count = 0
        self.failed_count = 0
        self.wait_since = None
        self.dwell_until = None
        self.done_reason = None
        self.config = None
        self.enabled = False
        self.valid = False
        self.route_attempt_at = None
        self.last_kind = None

    def invalidate(self):
        self.epoch += 1
        self.cancel.set()
        if self.job is not None:
            self.job[0].cancel()
        self.field = None
        self.views = None

    def reset_map(self):
        self.invalidate()
        self.optics = None
        self.live = None
        self.live_at = None
        self.attempts = ()
        self.node.search_views = ()
        if self.active:
            self.done_reason = 'cambió el mapa durante la búsqueda extra'

    def on_costmap(self, message, at_s=None):
        if not self.node.p['extra_search_enabled']:
            return
        try:
            live = Grid.from_occupancy_grid(message)
        except (TypeError, ValueError, OverflowError):
            self.live = None
            self.live_at = None
            return
        if self.live is not None and geometry(self.live) != geometry(live):
            self.reset_map()
        if self.optics is None:
            self.optics = live
        else:
            remembered = tuple(100 if old == 100 or new == 100 else new
                               for old, new in zip(self.optics.data, live.data))
            self.optics = Grid(live.width, live.height, live.resolution, remembered, live.origin)
        self.live = live
        self.live_at = at_s if at_s is not None else self.node.costmap_at

    def settings(self):
        p = self.node.p
        for key in ('extra_max_goals', 'extra_max_failed'):
            if type(p[key]) is not int or p[key] < 1:
                raise ValueError(key)
        for key in ('extra_goal_timeout_s', 'extra_plan_timeout_s', 'extra_costmap_max_age_s'):
            if isinstance(p[key], bool) or not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(key)
        for key in ('return_reserve_s', 'extra_failed_shift_m'):
            if isinstance(p[key], bool) or not math.isfinite(p[key]) or p[key] < 0:
                raise ValueError(key)
        if self.seconds(0.0) is None:
            raise ValueError('return estimator')
        return Settings(robot_radius=p['extra_transit_radius_m'], safety_margin=0.0,
                        candidate_spacing=p['extra_spacing_m'], target_spacing=p['extra_target_spacing_m'],
                        view_range=p['extra_range_m'], fov_degrees=p['extra_fov_deg'],
                        heading_count=p['extra_heading_count'], travel_speed=p['return_estimate_speed_mps'],
                        turn_speed=p['extra_turn_speed_rads'], observation_time=p['extra_look_s'],
                        max_views=p['extra_max_views'], max_positions=p['extra_max_positions'],
                        max_targets=p['extra_max_targets'], view_clearance=p['extra_view_clearance_m'],
                        view_preferred_clearance=p['extra_preferred_clearance_m'])

    def seconds(self, distance):
        p = self.node.p
        return route_seconds(distance, speed_mps=p['return_estimate_speed_mps'],
                             factor=p['return_estimate_factor'], overhead_s=p['return_estimate_overhead_s'])

    def ready(self):
        now = self.node.seconds_since_t0()
        limit = self.node.p['extra_costmap_max_age_s']
        return (isinstance(limit, (int, float)) and not isinstance(limit, bool)
                and math.isfinite(limit) and limit > 0 and now is not None
                and self.live is not None and self.live_at is not None
                and 0 <= now - self.live_at <= limit
                and self.node.grid is not None and self.node.start_locked
                and self.node.start_pose is not None and self.node.fresh_search_pose() is not None)

    def submit(self, kind):
        if self.job is not None or not self.ready():
            return
        node = self.node
        if (kind == self.last_kind and self.route_attempt_at is not None
                and node.seconds_since_t0() - self.route_attempt_at < node.p['extra_costmap_max_age_s'] / 2):
            return
        try:
            settings = self.settings()
            robot = node.fresh_search_pose()
            start = Pose(*node.start_pose)
            camera = None
            if kind == 'plan':
                camera = camera_from_transform(node.tf_buffer.lookup_transform(
                    'base_footprint', node.p['extra_camera_frame'], Time()).transform)
            zones = tuple(node.current_box_zones())
        except (TypeError, ValueError, OverflowError, TransformException) as error:
            node.get_logger().warning(f'Búsqueda extra no disponible: {error}', throttle_duration_sec=5.0)
            return
        live, optics, raw = self.live, self.optics, node.grid
        observed, attempted = node.search_views, self.attempts
        at_s, epoch, provider = self.live_at, self.epoch, self.provider
        self.cancel = Event()
        cancel = self.cancel
        kwargs = dict(limit=settings.max_views, failed_shift_m=node.p['extra_failed_shift_m'],
                      excluded=zones, cancel=cancel, occlusion_grid=optics,
                      same_point_distance_m=node.p['same_point_distance_m'],
                      same_point_angle_rad=node.p['same_point_angle_rad'])

        def calculate():
            field = build_route_field(live, robot, start, at_s, cancel=cancel)
            plan = None
            if kind == 'plan' and field is not None:
                plan = provider(raw, live, robot, start, observed, attempted, camera, settings, **kwargs)
            return field, plan

        if self.executor is None:
            self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mission-extra')
        self.job = (self.executor.submit(calculate), epoch, kind, live, optics)
        self.route_attempt_at = node.seconds_since_t0()
        self.last_kind = kind

    def tick(self):
        node = self.node
        enabled = bool(node.p['extra_search_enabled'])
        key = tuple((k, v) for k, v in sorted(node.p.items())
                    if k.startswith(('extra_', 'return_estimate_', 'same_point_')) or k == 'return_reserve_s')
        if key != self.config:
            self.invalidate()
            self.config = key
        if not enabled:
            if self.enabled:
                self.invalidate()
                self.live = self.optics = None
                self.live_at = None
                node.search_views = ()
                if self.phase == 'extra':
                    self.done_reason = 'búsqueda extra desactivada'
            self.enabled = False
            return
        self.enabled = True
        if self.live is None and node.costmap is not None:
            self.on_costmap(node.costmap, node.costmap_at)
        if self.job is not None and self.job[0].done():
            future, epoch, kind, live, optics = self.job
            self.job = None
            if epoch == self.epoch and same_grid(live, self.live):
                try:
                    field, plan = future.result()
                    self.field = field
                    if kind == 'plan' and same_grid(optics, self.optics):
                        self.views = () if plan is None else plan.views
                        pending = None if plan is None else plan.pending_count
                        node.get_logger().info(f'Plan extra: {len(self.views)} vistas, {pending} muestras pendientes')
                        if not self.views:
                            self.done_reason = 'sin vistas extra alcanzables'
                except Exception as error:
                    node.get_logger().warning(f'Falló la búsqueda extra: {error}')
                    if self.phase == 'extra':
                        self.done_reason = 'falló el cálculo de vistas extra'
        self.valid = False
        try:
            self.settings()
            self.valid = True
        except (TypeError, ValueError, OverflowError):
            if self.phase == 'extra':
                self.done_reason = 'parámetros extra inválidos'
            return
        now = node.seconds_since_t0()
        if self.phase == 'extra' and self.wait_since is not None:
            if now - self.wait_since >= node.p['extra_plan_timeout_s']:
                self.done_reason = 'se agotó la espera del plan extra'
        if (self.phase == 'primary' or self.active or node.current_inspection is not None) and self.job is None:
            interval = node.p['extra_costmap_max_age_s'] / 2
            if self.route_attempt_at is None or now - self.route_attempt_at >= interval:
                self.submit('route')

    def enter(self):
        if not self.valid:
            return 'parámetros extra inválidos'
        if self.phase != 'extra':
            self.phase = 'extra'
            self.node.get_logger().info(f'Inicio de búsqueda extra, generación {self.epoch}')
            self.wait_since = self.node.seconds_since_t0()
            self.views = None
            self.submit('plan')
        return self.reason()

    def estimate(self, goal):
        node = self.node
        if not self.ready() or not same_grid(self.field.grid if self.field else None, self.live):
            return None
        now = node.seconds_since_t0()
        estimate = estimate_route(self.field, goal, now, max_age_s=node.p['extra_costmap_max_age_s'])
        current = node.fresh_search_pose()
        root = estimate_route(self.field, current, now, max_age_s=node.p['extra_costmap_max_age_s'])
        if estimate is None or root is None:
            return None
        return estimate.outward_m + root.outward_m, estimate.home_m

    def affordable(self, goal):
        estimate = self.estimate(goal)
        if estimate is None:
            return False
        node, p = self.node, self.node.p
        return fits_extra(node.seconds_since_t0(), outward_s=self.seconds(estimate[0]),
                          home_s=self.seconds(estimate[1]), look_s=p['extra_look_s'],
                          reserve_s=p['return_reserve_s'], return_at_s=p['return_time_s'],
                          finish_at_s=p['finish_time_s'])

    def reason(self):
        node, p = self.node, self.node.p
        if self.done_reason is not None:
            return self.done_reason
        if p['extra_search_enabled'] and self.valid:
            pose = node.fresh_search_pose()
            estimate = self.estimate(pose) if pose is not None else None
            if estimate is not None and return_due(
                    node.seconds_since_t0(), home_s=self.seconds(estimate[1]),
                    reserve_s=p['return_reserve_s'], return_at_s=p['return_time_s'],
                    finish_at_s=p['finish_time_s']):
                return 'reserva necesaria para volver'
        if self.phase == 'extra' and not self.active and node.current_inspection is None:
            if self.dwell_until is not None and node.seconds_since_t0() < self.dwell_until:
                return None
            if self.accepted_count >= p['extra_max_goals']:
                return 'máximo de objetivos extra'
            if self.failed_count >= p['extra_max_failed']:
                return 'máximo de fallos extra'
        return None

    def explore(self):
        node = self.node
        if self.done_reason is not None:
            return
        now = node.seconds_since_t0()
        if self.dwell_until is not None and now < self.dwell_until:
            return
        if self.wait_since is None:
            self.wait_since = now
        if self.views is None:
            self.submit('plan')
            return
        if not self.ready() or self.estimate(node.fresh_search_pose()) is None:
            self.submit('route')
            return
        for view in self.views:
            pose = view.pose
            if not math.isfinite(view.gain) or view.gain <= 0:
                continue
            repeated = any(a.outcome == 'arrived'
                           and math.hypot(a.pose.x - pose.x, a.pose.y - pose.y) <= node.p['same_point_distance_m']
                           and angle_difference(a.pose.yaw, pose.yaw) <= node.p['same_point_angle_rad']
                           for a in self.attempts)
            if repeated or near_box(pose.x, pose.y, node.current_box_zones()) or node.is_blocked(pose.x, pose.y):
                continue
            if not self.affordable(pose):
                continue
            estimate = self.estimate(pose)
            if estimate is None:
                continue
            self.active = True
            goal = node.make_pose(pose.x, pose.y, pose.yaw)
            if node.go_to(goal):
                node.current_point = goal
                node.current_in_place = False
                self.views = None
                self.wait_since = None
                node.get_logger().info(
                    f'Vista extra: ({pose.x:.2f}, {pose.y:.2f}), ganancia {view.gain}, '
                    f'ida/vuelta {self.seconds(estimate[0]):.1f}/{self.seconds(estimate[1]):.1f} s, '
                    f'reserva {node.p["return_reserve_s"]:.1f} s')
            else:
                self.active = False
            return
        self.done_reason = 'ninguna vista extra entra en el presupuesto'

    def sent(self, pose):
        if self.node.p['extra_search_enabled'] and self.node.state == 'exploring':
            if self.phase == 'extra' and not self.active:
                self.invalidate()
                self.wait_since = None
            self.requested = Pose(pose.pose.position.x, pose.pose.position.y,
                                  yaw_from_quaternion(pose.pose.orientation))

    def accepted(self):
        if self.active:
            self.accepted_count += 1

    def result(self, outcome):
        if self.requested is not None:
            if (not self.node.inspection_is_reported()
                    and type(self.node.p['extra_history_max']) is int and self.node.p['extra_history_max'] > 0):
                self.attempts = (*self.attempts, AttemptedView(self.requested, outcome))[-self.node.p['extra_history_max']:]
            self.requested = None
            if self.phase == 'extra':
                self.views = None
                self.wait_since = None
        if self.active:
            self.active = False
            self.node.get_logger().info(f'Resultado extra: {outcome}')
            self.failed_count = 0 if outcome == 'arrived' else self.failed_count + 1
            self.dwell_until = self.node.seconds_since_t0() + (self.node.p['extra_look_s'] if outcome == 'arrived' else 0.0)
            self.wait_since = None
            self.views = None

    def close(self):
        self.invalidate()
        if self.executor is not None:
            self.executor.shutdown(wait=False, cancel_futures=True)
