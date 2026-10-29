"""Misión: qué hace el robot ahora, y cuándo reportar y terminar.

Recibe /map, el plan de cobertura y las candidatas; manda objetivos a
navegación con la acción NavigateToPose y publica /victimas y /done
(docs/team/INTERFACES.md). Es el único que reporta y el único cliente de
navegación. No publica /cmd_vel: eso es solo de navegación.

Esqueleto: espera el mapa y no hace nada más. Hasta recibir /map no se puede
mover el robot ni reportar (§5).
"""
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA, correr, parametros_en_vivo
from equipo_jar.reportes import armar_reporte
from equipo_jar_msgs.msg import Candidatas
from equipo_jar.confirmacion import select_to_report
from equipo_jar.exploracion import yaw_from_quaternion, first_pending, angle_difference, goal_position, box_zones, near_box, return_verdict
from equipo_jar.inspeccion import pick_doubtful, viewpoint_for
from equipo_jar.mapa import Grid, Pose
from equipo_jar.cobertura_revisita import ObservedView, revisit_views

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
import math
from action_msgs.msg import GoalStatus

import traceback


class Mision(Node):
    def __init__(self):
        super().__init__('mision')
        # Cantidad de víctimas a buscar (N), que llega como argumento del launch (victimas:=N).
        self.declare_parameter('cantidad_victimas', 5)
        self.cantidad_victimas = self.get_parameter('cantidad_victimas').value
        self.p = parametros_en_vivo(self, {'min_observations': 5, 'max_deviation_m': 0.10, 
                                            'min_report_distance_m': 0.40, 'same_point_distance_m': 0.02, 
                                            'same_point_angle_rad': 0.1, 'return_time_s': 480.0,
                                            'min_observations_final': 1, 'max_deviation_final_m': 0.30, 
                                            'finish_time_s': 570.0, 'idle_spin_s': 10.0, 'idle_spin_rad': 1.57,
                                            'visit_min_observations': 2, 'visit_distance_m': 0.8, 
                                            'visit_clearance_m': 0.25, 'visit_angles': 12, 
                                            'visit_min_shift_m': 0.4, 'visit_box_margin_m': 0.25, 
                                            'start_wait_s': 4.0, 'goal_timeout_s': 90.0, 'stuck_time_s': 20.0,
                                            'stuck_distance_m': 0.05, 'stuck_angle_rad': 0.2, 'stop_wait_s': 1.0,
                                            'done_limit_s': 590.0, 'turn_in_place_m': 0.20, 
                                            'visit_max_attempts': 3, 'box_min_observations': 2,
                                            'box_clearance_m': 0.42, 'box_clearance_sigmas': 1.0,    
                                            'visit_transit_radius_m': 0.15, 'visit_free_threshold': 25,
                                            'settle_s': 0.0, 'max_recoveries': 10, 'return_tolerance_m': 0.10,
                                            'return_yaw_tolerance_rad': 0.25, 'return_max_tries': 3,
                                            'return_still_s': 0.5, 'return_still_m': 0.01,
                                            'revisit_enabled': True, 'revisit_max_age_s': 10.0,
                                            'revisit_min_shift_m': 0.05, 'revisit_wait_s': 2.0,
                                            'revisit_max_records': 128,
                                            'revisit_adjustment_m': 0.05})  # parámetros ajustables

        self.mapa = None
        self.grid = None
        self.plan = None
        self.candidatas = None
        self.observed_views = []
        self.observation_map_revision = 0
        self.revisit_pending = None

        self.t0 = None
        self.create_timer(0.1, self.step)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_pose = None

        self.reported_ids = set()
        self.reported_points = []
        self.inspected_points = []
        self.failed_inspections = {}

        self.nav_state = 'idle'
        self.state = 'waiting_for_map'

        self.visited = []
        self.current_point = None
        self.settle_until = None
        self.current_goal = None

        self.return_sent = False
        self.return_tries = 0
        self.check_since = None
        self.check_pose = None
        self.canceling = False

        self.idle_since = None

        self.start_locked = False

        self.goal_start_time = None
        self.last_motion_pose = None
        self.last_motion_time = None
        self.stuck_canceling = False

        self.stop_canceling = False
        self.stop_since = None

        self.last_skipped = None

        self.recoveries = 0

        self.create_subscription(OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        self.create_subscription(Path, '/jar/plan_cobertura', self.al_recibir_plan, QOS_ESTADO)
        self.create_subscription(
            Candidatas, '/jar/candidatas', self.al_recibir_candidatas, QOS_ESTADO)
        self.create_subscription(
            PoseWithCovarianceStamped, '/jar/observaciones', self.al_recibir_observacion, 10)
        self.navegacion = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_victimas = self.create_publisher(PointStamped, '/victimas', 10)
        self.pub_done = self.create_publisher(Empty, '/done', 10)
        self.get_logger().info(
            f'Hay que encontrar {self.cantidad_victimas} víctimas. Esperando el mapa en /map...')

    def al_recibir_mapa(self, mapa):
        if self.mapa is None:
            self.get_logger().info(
                f'Mapa recibido: {mapa.info.width}x{mapa.info.height} celdas, '
                f'resolución {mapa.info.resolution:.3f} m')
            self.t0 = self.get_clock().now()
        self.mapa = mapa
        try:
            grid = Grid.from_occupancy_grid(mapa)
            key = lambda g: (g.width, g.height, g.resolution, g.origin, g.data)
            if self.grid is None or key(grid) != key(self.grid):
                self.observation_map_revision += 1
                self.observed_views.clear()
            self.grid = grid
        except ValueError as e:
            self.grid = None
            self.observation_map_revision += 1
            self.observed_views.clear()
            self.get_logger().error(f'No pude armar la grilla del mapa: {e}. Sin inspecciones.')

    def al_recibir_observacion(self, msg):
        if self.grid is None or not self.p['revisit_enabled'] or msg.header.frame_id != 'map':
            return
        limit = self.p['revisit_max_records']
        if type(limit) is not int or limit < 1:
            return
        try:
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds <= 0:
                return  # Time(0) means latest in TF, not the observation pose.
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', stamp)
            point = tr.transform.translation
            q = tr.transform.rotation
            components = (q.x, q.y, q.z, q.w)
            if (not all(math.isfinite(v) for v in components)
                    or abs(sum(v * v for v in components) - 1.0) > 1e-3):
                return
            pose = Pose(point.x, point.y, yaw_from_quaternion(q))
            target = msg.pose.pose.position
            diagonal = (msg.pose.covariance[0], msg.pose.covariance[7])
            if not all(math.isfinite(v) and v > 0 for v in diagonal):
                return
            sigma = math.sqrt(max(diagonal))
            view = ObservedView(pose, target.x, target.y, stamp.nanoseconds / 1e9,
                                sigma, self.observation_map_revision)
        except (TransformException, ValueError, TypeError):
            return
        self.observed_views.append(view)
        del self.observed_views[:-limit]

    def seconds_since_t0(self):
        if self.t0 is None:
            return None
        now = self.get_clock().now()
        elapsed = now - self.t0
        seconds = elapsed.nanoseconds / 1e9
        return float(seconds)

    def current_pose(self):
        try:
            pose = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            x = pose.transform.translation.x
            y = pose.transform.translation.y
            q = pose.transform.rotation
            yaw = yaw_from_quaternion(q)
            return (x, y, yaw)
        except TransformException:
            return None

    def odom_pose(self):
        try:
            pose = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
            x = pose.transform.translation.x
            y = pose.transform.translation.y
            q = pose.transform.rotation
            yaw = yaw_from_quaternion(q)
            return (x, y, yaw)
        except TransformException:
            return None

    def check_reports(self, min_obs, max_deviation):
        if self.candidatas is None:
            return
        selected = select_to_report(self.candidatas, self.reported_ids, self.reported_points,
                                        self.cantidad_victimas, min_obs, 
                                        max_deviation, self.p['min_report_distance_m'])
        for c in selected:
            self.reportar_victima(c.centro.x, c.centro.y)
            self.reported_ids.add(c.id)
            self.reported_points.append((c.centro.x, c.centro.y))
            i = len(self.reported_points)
            self.get_logger().info(f'Reporte {i}. id={c.id} x={c.centro.x :.2f} y={c.centro.y:.2f}')

    def go_to(self, pose):
        if not self.navegacion.server_is_ready():
            self.get_logger().warn('!!! El servidor de navegacion no esta listo !!!', throttle_duration_sec=5.0)
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_state = "sending"
        self.recoveries = 0
        future = self.navegacion.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        now = self.seconds_since_t0()
        self.goal_start_time = now
        self.last_motion_time = now
        self.last_motion_pose = self.odom_pose()
        self.stuck_canceling = False
        if not self.start_locked:
            self.start_locked = True
            if self.start_pose is not None:
                x, y, yaw = self.start_pose
                self.get_logger().info(f'Pose de inicio fija: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
            else:
                self.get_logger().warning('Primer viaje sin pose de inicio: no se va a poder volver')
        return True

    def check_stuck(self):
        if self.nav_state != 'moving' or self.stuck_canceling:
            return
        now = self.seconds_since_t0()
        pose = self.odom_pose()

        if pose is None or self.last_motion_pose is None:
            self.last_motion_pose = pose
            self.last_motion_time = now
        else:
            x1, y1, yaw1 = pose
            x2, y2, yaw2 = self.last_motion_pose
            dist = math.hypot(x1 - x2, y1 - y2)
            spin = angle_difference(yaw1, yaw2)
            if dist > self.p['stuck_distance_m'] or spin > self.p['stuck_angle_rad']:
                self.last_motion_pose = pose
                self.last_motion_time = now

        reason = None
        if now - self.goal_start_time > self.p['goal_timeout_s']:
            reason = 'viaje demasiado largo'
        elif now - self.last_motion_time > self.p['stuck_time_s']:
            reason = 'robot quieto'
        elif self.state != 'returning' and self.recoveries >= self.p['max_recoveries']:
            reason = f'Nav2 lleva {self.recoveries} recuperaciones'
        if reason is not None:
            self.current_goal.cancel_goal_async()
            self.stuck_canceling = True
            self.get_logger().warning(f'Cancelo el viaje: {reason}')

    def check_goal_near_box(self):
        if self.state != 'exploring':
            return
        elif self.nav_state != 'moving':
            return
        elif self.current_point is None:
            return
        elif self.stuck_canceling:
            return
        x = self.current_point.pose.position.x
        y = self.current_point.pose.position.y
        if near_box(x, y, self.current_box_zones()):
            self.current_goal.cancel_goal_async()
            self.get_logger().info(f'Cancelo el viaje: el punto del plan ({x:.2f},{y:.2f}) '
                                    f'quedó cerca de una caja')
            self.current_point = None

    def handle_on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_state = 'failed'
            self.get_logger().warn('Fallo el objetivo')
            return
        if self.state == 'finishing' or self.state == 'finished':
            goal_handle.cancel_goal_async()
            return
        self.current_goal = goal_handle
        self.nav_state = 'moving'
        goal_handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_response(self, future):
        try:
            self.handle_on_goal_response(future)
        except Exception:
            self.nav_state = 'failed'
            self.get_logger().error(f'Error en on_goal_response:\n{traceback.format_exc()}', throttle_duration_sec=5.0)

    def handle_on_goal_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_state = 'arrived'
            self.get_logger().info(f'Status succeded (recuperaciones: {self.recoveries})')
        else:
            self.nav_state = 'failed'
            self.get_logger().info(f'Status failed (recuperaciones: {self.recoveries})')

    def on_goal_result(self, future):
        try:
            self.handle_on_goal_result(future)
        except Exception:
            self.nav_state = 'failed'
            self.get_logger().error(f'Error en on_goal_result:\n{traceback.format_exc()}', throttle_duration_sec=5.0)

    def handle_on_feedback(self, msg):
        if self.nav_state != 'moving':
            return
        self.recoveries = msg.feedback.number_of_recoveries

    def on_feedback(self, msg):
        try:
            self.handle_on_feedback(msg)
        except Exception:
            self.get_logger().error(f'Error en on_feedback:\n{traceback.format_exc()}',
                                    throttle_duration_sec=5.0)

    def change_state(self, new_state):
        self.get_logger().info(f'Estado: <{self.state}> -> <{new_state}>')
        self.state = new_state

    def step_logic(self):
        if self.state == 'waiting_for_map':
            if self.t0 is not None:
                self.change_state('exploring')
            return
        if self.state == 'finished':
            return
        if self.state == 'finishing':
            self.pub_done.publish(Empty())
            self.change_state('finished')
            return
        if self.state == 'stopping':
            self.wait_for_stop()
            return
        if not self.start_locked:
            pose = self.current_pose()
            if pose is not None:
                if self.start_pose is None:
                    x, y, yaw = pose
                    self.get_logger().info(f'Pose de inicio (provisoria): x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
                self.start_pose = pose
        if self.seconds_since_t0() >= self.p['finish_time_s']:
            self.get_logger().info('Se termina el tiempo de la corrida')
            self.change_state('stopping')
            return
        self.check_reports(self.p['min_observations'], self.p['max_deviation_m'])
        if self.seconds_since_t0() < self.p['start_wait_s']:
            return
        self.check_stuck()
        self.check_goal_near_box()
        if self.state == 'exploring':
            reason = self.reason_to_return()
            if reason is not None:
                self.get_logger().info(f'Vuelvo: {reason}')
                self.change_state('returning')
            else:
                self.explore()
        if self.state == 'returning':
            self.return_to_start()
        if self.state == 'checking':
            self.check_return()

    def step(self):
        try:
            self.step_logic()
        except Exception:
            self.get_logger().error(f'Error en step:\n{traceback.format_exc()}', throttle_duration_sec=5.0)

    def current_box_zones(self):
        if self.candidatas is None:
            return []
        return box_zones(self.candidatas, self.p['box_min_observations'],
                        self.p['box_clearance_m'], self.p['box_clearance_sigmas'])

    def explore(self):
        if self.nav_state == 'sending' or self.nav_state == 'moving':
            return
        if self.nav_state == 'arrived' or self.nav_state == 'failed':
            arrived = self.nav_state == 'arrived'
            if self.current_point is not None:
                x = self.current_point.pose.position.x
                y = self.current_point.pose.position.y
                yaw = yaw_from_quaternion(self.current_point.pose.orientation)
                self.visited.append((x, y, yaw))
            self.current_point = None
            self.nav_state = 'idle'
            delay = self.p['settle_s']
            if self.revisit_pending is not None:
                if arrived:
                    delay = max(delay, self.revisit_pending['wait_s'])
                    self.revisit_pending['arrived'] = True
                    self.revisit_pending['arrived_at'] = self.seconds_since_t0()
                else:
                    self.finish_revisit(False)
            self.settle_until = self.seconds_since_t0() + delay
        if self.revisit_pending is not None and self.revisit_pending.get('arrived_at') is not None:
            since = self.revisit_pending['arrived_at']
            wait_s = self.p['revisit_wait_s']
            valid_wait = math.isfinite(wait_s) and 0 <= wait_s <= 5.0
            same_map = self.revisit_pending['map_revision'] == self.observation_map_revision
            if valid_wait and same_map and self.p['revisit_enabled']:
                self.settle_until = since + max(self.p['settle_s'], wait_s)
            else:
                self.finish_revisit(False)
                self.settle_until = since + self.p['settle_s']
        if self.settle_until is not None and self.seconds_since_t0() < self.settle_until: 
            return
        self.settle_until = None
        if self.revisit_pending is not None:
            self.finish_revisit(True)
        if self.inspect_doubtful():
            return
        if self.plan is None:
            self.spin_if_idle()
            return
        zones = self.current_box_zones()
        point = first_pending(self.plan.poses, self.visited, self.p['same_point_distance_m'], 
                                self.p['same_point_angle_rad'], zones)
        first_free = first_pending(self.plan.poses, self.visited, self.p['same_point_distance_m'],
                                    self.p['same_point_angle_rad'], [])
        if first_free is not None and first_free is not point and first_free is not self.last_skipped:
            x = first_free.pose.position.x
            y = first_free.pose.position.y
            self.get_logger().info(f'Salteo el punto del plan ({x:.2f},{y:.2f}): cerca de una caja')
            self.last_skipped = first_free
        if point is None:
            self.spin_if_idle()
            return
        plan_x = point.pose.position.x
        plan_y = point.pose.position.y
        q = point.pose.orientation
        yaw = yaw_from_quaternion(q)
        x, y, in_place = goal_position(plan_x, plan_y, self.current_pose(), self.p['turn_in_place_m'])
        goal = self.make_pose(x, y, yaw)
        sent = self.go_to(goal)
        if sent:
            self.current_point = point
            if in_place:
                self.get_logger().info(f'Giro en el lugar: yaw={yaw:.2f} desde ({x:.2f},{y:.2f}), '
                                        f'punto del plan ({plan_x:.2f},{plan_y:.2f})')
            else:
                self.get_logger().info(f'Viaje al punto del plan: ({plan_x:.2f}, {plan_y:.2f}, {yaw})')
            self.idle_since = None

    def spin_if_idle(self):
        if self.idle_since is None:
            time = self.seconds_since_t0()
            self.idle_since = time
            return
        if self.seconds_since_t0() - self.idle_since < self.p['idle_spin_s']:
            return
        pose = self.current_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self.go_to(self.make_pose(x, y, yaw + self.p['idle_spin_rad'])):
            self.get_logger().info('Sin plan: giro en el lugar')
            self.idle_since = None


    def reason_to_return(self):
        if len(self.reported_points) >= self.cantidad_victimas:
            return 'todas reportadas'
        elif self.seconds_since_t0() >= self.p['return_time_s']:
            return 'se termina el tiempo'
        elif (self.plan is not None and self.revisit_pending is None
                and self.nav_state != 'sending' and self.nav_state != 'moving'
                and first_pending(self.plan.poses, self.visited, self.p['same_point_distance_m'], 
                self.p['same_point_angle_rad'], self.current_box_zones()) is None and self.next_doubtful() is None):
            return 'no quedan puntos ni dudosas'
        return None

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def return_to_start(self):
        if self.nav_state == 'sending':
            return
        if self.nav_state == 'moving' and not self.return_sent:
            if not self.canceling:
                self.current_goal.cancel_goal_async()
                self.canceling = True
                self.get_logger().info("Cancelando objetivo...")
            return
        if not self.return_sent:
            if self.start_pose is None:
                self.get_logger().warning('No hay pose de inicio, termino sin volver')
                self.change_state('stopping')
                return
            else:
                x, y, yaw = self.start_pose
                sent = self.go_to(self.make_pose(x, y, yaw))
                if sent:
                    self.return_sent = True
                    self.return_tries += 1
                    return
        if self.return_sent and (self.nav_state == 'arrived' or self.nav_state == 'failed'):
            self.get_logger().info(f'Vuelta terminada: {self.nav_state}')
            self.check_since = None
            self.change_state('checking')
            return

    def check_return(self):
        now = self.seconds_since_t0()
        odom = self.odom_pose()
        if odom is None:
            return
        if (self.check_since is None or 
            math.hypot(odom[0] - self.check_pose[0], odom[1] - self.check_pose[1]) > self.p['return_still_m']):
            self.check_since = now
            self.check_pose = odom
            return
        if now - self.check_since < self.p['return_still_s']:
            return
        pose = self.current_pose()
        if pose is None:
            return
        x, y, yaw = pose
        x0, y0, yaw0 = self.start_pose
        error = math.hypot(x - x0, y - y0)
        error_yaw = angle_difference(yaw, yaw0)
        self.get_logger().info(f'Regreso {self.return_tries}: a {error:.3f} m y '
                                f'{error_yaw:.2f} rad de la largada')
        verdict = return_verdict(error, error_yaw, self.return_tries, self.p['return_tolerance_m'],
                                self.p['return_yaw_tolerance_rad'], self.p['return_max_tries'])
        if verdict == 'ok':
            self.change_state('stopping')
        elif verdict == 'retry':
            self.get_logger().info('Lejos de la largada: vuelvo a intentar')
            self.return_sent = False
            self.change_state('returning')
        else:
            self.get_logger().warning('Termino lejos de la largada')
            self.change_state('stopping')

    def next_doubtful(self):
        if self.candidatas is None:
            return None
        pose = self.current_pose()
        if pose is None:
            return None
        return pick_doubtful(self.candidatas, self.reported_ids, self.reported_points, self.inspected_points, 
                            self.p['visit_min_observations'], self.p['min_report_distance_m'], pose[0], pose[1], 
                            self.failed_inspections, self.p['visit_max_attempts'])

    def inspect_doubtful(self):
        c = self.next_doubtful()
        if c is None:
            return False
        x_robot, y_robot, _ = self.current_pose()
        if self.grid is None:
            return False
        try:
            reachable = self.grid.reachable_from(x_robot, y_robot, self.p['visit_transit_radius_m'],
                                                self.p['visit_free_threshold'])
        except ValueError as e:
            self.get_logger().info(f'No calculo zona alcanzable: {e}. Inspecciono más tarde',
                                    throttle_duration_sec=5.0)
            return False
        view = viewpoint_for(self.mapa, c.centro.x, c.centro.y, x_robot, y_robot, self.p['visit_distance_m'], 
                            self.p['visit_clearance_m'], self.p['visit_angles'], self.p['visit_min_shift_m'],
                            self.p['visit_box_margin_m'], self.grid, reachable)
        recorded = None
        wait_s = self.p['revisit_wait_s']
        if view is None and self.p['revisit_enabled']:
            try:
                if not math.isfinite(wait_s) or not 0 <= wait_s <= 5.0:
                    raise ValueError('revisit_wait_s must be between 0 and 5 seconds')
                current = self.current_pose()
                hints = revisit_views(
                    self.grid, Pose(*current), c.centro.x, c.centro.y, self.observed_views,
                    now_s=self.get_clock().now().nanoseconds / 1e9,
                    map_revision=self.observation_map_revision, max_age_s=self.p['revisit_max_age_s'],
                    association_m=self.p['min_report_distance_m'], min_shift_m=self.p['revisit_min_shift_m'],
                    transit_radius=self.p['visit_transit_radius_m'],
                    view_clearance=self.p['visit_clearance_m'], threshold=self.p['visit_free_threshold'],
                    max_adjustment_m=self.p['revisit_adjustment_m'])
                if hints:
                    recorded = hints[0]
                    view = (recorded.pose.x, recorded.pose.y, recorded.pose.yaw)
            except (ValueError, TypeError) as e:
                self.get_logger().warning(f'Observed-view fallback unavailable: {e}', throttle_duration_sec=5.0)
        if view is None:
            _, attempts = self.failed_inspections.get(c.id, (0, 0))
            self.failed_inspections[c.id] = (c.observaciones, attempts + 1)
            self.get_logger().info(f'Candidata id: {c.id} x={c.centro.x:.2f} y={c.centro.y:.2f} '
                                    f'obs={c.observaciones}. Sin lugar para mirarla (intento {attempts + 1} '
                                    f'de {self.p["visit_max_attempts"]})')
            return False
        else:
            x, y, yaw = view
            if self.go_to(self.make_pose(x, y, yaw)):
                if recorded is None:
                    self.inspected_points.append((c.centro.x, c.centro.y))
                else:
                    self.revisit_pending = dict(id=c.id, x=c.centro.x, y=c.centro.y,
                                                before=c.observaciones, wait_s=wait_s, arrived=False,
                                                map_revision=self.observation_map_revision)
                    self.get_logger().info(
                        f'Observed-view fallback id={c.id}; goal adjustment {recorded.adjustment_m:.3f} m')
                self.get_logger().info(f'Voy a mirar la candidata id={c.id} desde ({x:.2f},{y:.2f})')
                self.idle_since = None
                return True
        return False

    def finish_revisit(self, succeeded):
        pending = self.revisit_pending
        if pending is None:
            return
        self.revisit_pending = None
        candidates = [] if self.candidatas is None else self.candidatas
        nearby = [c for c in candidates
                  if math.hypot(c.centro.x - pending['x'], c.centro.y - pending['y'])
                  < self.p['min_report_distance_m']]
        current = next((c for c in nearby if c.id == pending['id']), None)
        if current is None and nearby:
            current = min(nearby, key=lambda c: math.hypot(c.centro.x - pending['x'],
                                                         c.centro.y - pending['y']))
        count = current.observaciones if current is not None else pending['before']
        confirmed = (succeeded and pending['arrived']
                     and pending['map_revision'] == self.observation_map_revision
                     and count > pending['before'] and count >= self.p['min_observations'])
        if confirmed:
            self.inspected_points.append((current.centro.x, current.centro.y))
            self.get_logger().info(f'Observed-view revisit received new observations: {count}')
        else:
            identity = current.id if current is not None else pending['id']
            _, attempts = self.failed_inspections.get(identity, (0, 0))
            self.failed_inspections[identity] = (count, attempts + 1)
            self.get_logger().info('Observed-view revisit did not confirm new observations')

    def wait_for_stop(self):
        now = self.seconds_since_t0()
        if self.nav_state == 'moving' and not self.stop_canceling:
            self.current_goal.cancel_goal_async()
            self.stop_canceling = True
            self.get_logger().info('Paro: cancelo el viaje')
        if self.nav_state == 'sending' or self.nav_state == 'moving':
            if now < self.p['done_limit_s']:
                return
            self.get_logger().warning('Nav2 no contesta: termino igual')
            self.terminar()
            return
        if self.stop_since is None:
            self.stop_since = now
        if (now - self.stop_since) < self.p['stop_wait_s']:
            return
        self.terminar()

    def al_recibir_plan(self, plan):
        self.plan = plan

    def al_recibir_candidatas(self, msg):
        self.candidatas = msg.candidatas

    def reportar_victima(self, x, y):
        """Publica un reporte. Cada llamada gasta un reporte (§7.2)."""
        self.pub_victimas.publish(armar_reporte(x, y, self.get_clock().now().to_msg()))

    def terminar(self):
        self.check_reports(self.p['min_observations_final'], self.p['max_deviation_final_m'])
        self.change_state('finishing')


def main(args=None):
    correr(Mision, args)
