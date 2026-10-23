"""Publish an initial viewpoint route after the map and valid TF are available.

Mission tracks finished goals. Coverage does not track actual observations or
navigation results; it replans when the map or a planning parameter changes.
"""
from concurrent.futures import CancelledError, ThreadPoolExecutor
from dataclasses import asdict
import math
from threading import Event

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA, correr
from equipo_jar.cobertura_plan import Camera, Grid, Pose, Settings, angle_difference, make_plan


def planar_yaw(q):
    """Require a normalized quaternion representing a planar rotation."""
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('rotation must be finite')
    if abs(sum(v * v for v in values) - 1.0) > 1e-3:
        raise ValueError('rotation quaternion must be normalized')
    if abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
        raise ValueError('map and robot rotations must be planar')
    return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)


def camera_from_transform(transform):
    """The optical camera looks along +Z; project that axis into the base XY plane."""
    q, p = transform.rotation, transform.translation
    values = (q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(v) for v in values)
            or abs(sum(v * v for v in values) - 1.0) > 1e-3):
        raise ValueError('invalid camera quaternion')
    forward_x = 2 * (q.x * q.z + q.w * q.y)
    forward_y = 2 * (q.y * q.z - q.w * q.x)
    if math.hypot(forward_x, forward_y) < 1e-3:
        raise ValueError('camera forward axis has no horizontal projection')
    return Camera(p.x, p.y, math.atan2(forward_y, forward_x))


class Cobertura(Node):
    def __init__(self):
        super().__init__('cobertura')
        defaults = {**asdict(Settings()), 'base_frame': 'base_footprint',
                    'camera_frame': 'cam_1_color_optical_frame', 'tf_max_age': 1.0}
        self.p = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        self._validate(self.p)
        self._generation = 0
        self._planned_generation = -1
        self._future = None
        self._future_generation = None
        self._failed_start = None
        self._planning_start = None
        self._planning_pose = None
        self._planning_camera = None
        self._cancel_event = Event()
        self._grid = None
        self._map_key = None
        self._last_wait_reason = None
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='coverage')
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.pub_plan = self.create_publisher(Path, '/jar/plan_cobertura', QOS_ESTADO)
        self.create_subscription(OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.create_timer(0.25, self._tick)
        self.get_logger().info('Waiting for /map and localization TF.')

    @staticmethod
    def _validate(parameters):
        Settings(**{key: parameters[key] for key in asdict(Settings())})
        for name in ('base_frame', 'camera_frame'):
            if not isinstance(parameters[name], str) or not parameters[name].strip():
                raise ValueError(f'{name} must be a nonempty frame name')
        age = parameters['tf_max_age']
        if not math.isfinite(age) or age <= 0:
            raise ValueError('tf_max_age must be finite and positive')

    def _validate_parameters(self, parameters):
        proposed = {name: self.get_parameter(name).value for name in self.p}
        for parameter in parameters:
            if parameter.name in proposed:
                proposed[parameter.name] = parameter.value
        try:
            self._validate(proposed)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _sync_parameters(self):
        # Humble has no post-set callback. Read only values ROS has committed.
        current = {name: self.get_parameter(name).value for name in self.p}
        if current != self.p:
            self.p = current
            self._generation += 1
            self._cancel_event.set()

    def al_recibir_mapa(self, mapa):
        try:
            if mapa.header.frame_id != 'map':
                raise ValueError('map frame must be map')
            origin = mapa.info.origin
            grid = Grid(mapa.info.width, mapa.info.height, mapa.info.resolution,
                        list(mapa.data), Pose(origin.position.x, origin.position.y,
                                              planar_yaw(origin.orientation)))
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Map rejected: {exc}')
            return
        key = (grid.width, grid.height, grid.resolution, grid.origin, grid.data)
        if key == self._map_key:
            return
        self._grid, self._map_key = grid, key
        self._generation += 1
        self._cancel_event.set()
        self.get_logger().info(f'Map received: {grid.width} x {grid.height}.')

    def _pose_and_camera(self):
        transform = self._tf.lookup_transform('map', self.p['base_frame'], Time())
        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds:
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if abs(age) > self.p['tf_max_age']:
                raise ValueError('localization TF is stale or in the future')
        p = transform.transform.translation
        pose = Pose(p.x, p.y, planar_yaw(transform.transform.rotation))
        optical = self._tf.lookup_transform(self.p['base_frame'], self.p['camera_frame'], Time())
        return pose, camera_from_transform(optical.transform)

    def _tick(self):
        self._sync_parameters()
        if self._future is not None:
            if not self._future.done():
                return
            future, generation = self._future, self._future_generation
            self._future = None
            try:
                result = future.result()
            except CancelledError:
                return  # A newer map/configuration or shutdown superseded this calculation.
            except Exception as exc:
                # Invalid input or a planning failure must not kill the ROS node.
                self.get_logger().error(f'Planning failed: {exc}')
                self._failed_start = (generation, self._planning_start)
                return
            if generation != self._generation:
                return  # A map/parameter changed while the worker was running.
            try:
                pose, camera = self._pose_and_camera()
            except (TransformException, ValueError) as exc:
                self._wait(str(exc))
                return  # Retry from a fresh pose, without publishing an obsolete plan.
            if self._start_changed(pose, camera):
                self.get_logger().info('Pose changed during planning; rebuilding from the current pose.')
                return
            self._publish(result)
            self._planned_generation = generation
        if self._grid is None or self._planned_generation == self._generation:
            return
        try:
            pose, camera = self._pose_and_camera()
        except (TransformException, ValueError) as exc:
            self._wait(str(exc))
            return
        self._last_wait_reason = None
        start_cell = self._grid.cell(pose.x, pose.y)
        if self._failed_start == (self._generation, start_cell):
            return
        settings = Settings(**{key: self.p[key] for key in asdict(Settings())})
        self._future_generation = self._generation
        self._planning_start = start_cell
        self._planning_pose, self._planning_camera = pose, camera
        self._cancel_event = Event()
        self._future = self._worker.submit(make_plan, self._grid, pose, settings, camera,
                                          cancel=self._cancel_event)

    def _start_changed(self, pose, camera):
        # Changes smaller than half a sampling step stay within this plan's
        # spatial/angular discretization. Bigger corrections need a fresh plan.
        linear = self.p['candidate_spacing'] / 2
        angular = math.pi / self.p['heading_count']
        old, optical = self._planning_pose, self._planning_camera
        return (old is None or optical is None
                or math.hypot(pose.x - old.x, pose.y - old.y) > linear
                or abs(angle_difference(pose.yaw, old.yaw)) > angular
                or math.hypot(camera.x - optical.x, camera.y - optical.y) > self._grid.resolution / 2
                or abs(angle_difference(camera.yaw, optical.yaw)) > angular)

    def _wait(self, reason):
        # The exact missing timestamp can change every tick; log once per waiting phase.
        if self._last_wait_reason is None:
            self.get_logger().info(f'Waiting for valid TF: {reason}')
        self._last_wait_reason = reason

    def _publish(self, result):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for pose in result.poses:
            point = PoseStamped()
            point.header = msg.header
            point.pose.position.x, point.pose.position.y = pose.x, pose.y
            point.pose.orientation.z = math.sin(pose.yaw / 2)
            point.pose.orientation.w = math.cos(pose.yaw / 2)
            msg.poses.append(point)
        self.pub_plan.publish(msg)
        self.get_logger().info(
            f'Initial plan: {len(msg.poses)} views; predicted visibility '
            f'{result.predicted_count}/{result.target_count} sampled cells. '
            'Actual observation and navigation failures are not tracked yet.')
        if result.metrics is not None:
            m = result.metrics
            self.get_logger().info(
                f'Grid estimate: {m.distance:.1f} m, {m.turning:.1f} rad, {m.duration:.1f} s; '
                f'{m.return_distance:.1f} m to return. '
                f'{result.target_count - m.visible_from_candidates} samples outside the candidate model, '
                f'{m.visible_from_candidates - result.predicted_count} visible but deferred.')

    def destroy_node(self):
        self._cancel_event.set()
        self._worker.shutdown(wait=True, cancel_futures=True)
        self._listener.unregister()
        return super().destroy_node()


def main(args=None):
    correr(Cobertura, args)
