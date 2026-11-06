"""Record effective Nav2 geometry and one optional controlled goal.

Test-only client. Never launch alongside mission. No parameter writes, direct
velocity commands, map/TF publishers or competition-launch modifications.
"""
import argparse
import hashlib
import math
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from auditoria_navegacion import (check_consumers, check_costmap, evidence_json,
                                 points_in_base, trial_summary)


def pose_values(pose):
    q = pose.orientation
    return [pose.position.x, pose.position.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]


class Recorder(Node):
    def __init__(self, destination, truth_topic=None, truth_frame_is_map=False):
        super().__init__('navigation_diagnostic')
        destination.mkdir(parents=True, exist_ok=False)
        self.destination = destination
        self.stream = (destination / 'events.jsonl').open('w')
        self.started = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.polygons = {}
        self.map_seen = False
        self.localized = self.truth = None
        self.truth_frame_is_map = truth_frame_is_map
        self.recoveries = None
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_handle = self.acceptance = self.result = None
        self.dispatch_started = False
        self.interrupt_requested = None
        self.status = None
        self.stop_confirmed = False
        self.arrival = None
        self.inhibit = self.create_publisher(Bool, '/jar/parar',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.record('cmd_vel', m), 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(NavPath, '/plan', lambda m: self.record('plan', m), 10)
        self.create_subscription(Bool, '/jar/parada_confirmada',
                                 self.on_stop_confirmation, durable)
        for name in ('local_costmap', 'global_costmap'):
            self.create_subscription(PolygonStamped, f'/{name}/published_footprint',
                                     lambda m, n=name: self.on_polygon(n, m), 10)
            self.create_subscription(Costmap, f'/{name}/costmap_raw',
                                     lambda m, n=name: self.record(f'{n}_raw', m), durable)
        for name in ('spin', 'backup', 'wait', 'navigate_to_pose'):
            self.create_subscription(GoalStatusArray, f'/{name}/_action/status',
                                     lambda m, n=name: self.record(f'{n}_status', m), durable)
        if truth_topic:
            self.create_subscription(Odometry, truth_topic, self.on_truth, qos_profile_sensor_data)
        self.create_timer(.1, self.sample_pose)

    def record(self, kind, message=None, **data):
        row = {'event': kind, 'wall_elapsed_s': time.monotonic() - self.started,
               'ros_time_s': self.get_clock().now().nanoseconds / 1e9, **data}
        if message is not None:
            row['message'] = message_to_ordereddict(message)
        self.stream.write(evidence_json(row) + '\n')
        self.stream.flush()

    def on_map(self, message):
        self.map_seen = True
        self.record('map', message)

    def on_log(self, message):
        if any(n in message.name for n in ('controller', 'planner', 'behavior', 'bt_navigator')):
            self.record('rosout', message)

    def on_polygon(self, name, message):
        self.polygons[name] = (message, time.monotonic())
        self.record(f'{name}_footprint', message)

    def on_truth(self, message):
        self.record('ground_truth', message)
        age = (self.get_clock().now() - Time.from_msg(message.header.stamp)).nanoseconds / 1e9
        pose = pose_values(message.pose.pose)
        if (message.header.frame_id == 'map' or self.truth_frame_is_map) \
                and 0 <= age <= .5 and all(math.isfinite(v) for v in pose):
            self.truth = (pose, time.monotonic() - age)

    def on_stop_confirmation(self, message):
        self.stop_confirmed = message.data
        self.record('stop_confirmation', message)

    def current_trial(self, target):
        def fresh(sample):
            return sample[0] if sample and time.monotonic() - sample[1] <= .5 else None
        return trial_summary(target, self.status, fresh(self.localized), fresh(self.truth),
                             self.recoveries)

    def sample_pose(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', Time())
            p, q = transform.transform.translation, transform.transform.rotation
            stamp = Time.from_msg(transform.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            pose = [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                        1 - 2 * (q.y*q.y + q.z*q.z))]
            self.record('localized_pose', transform, pose=pose, source_age_s=age)
            if 0 <= age <= .5:
                self.localized = (pose, time.monotonic() - age)
        except TransformException:
            pass

    def spin_until(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            self.check_interrupt()
            if predicate():
                return True
        return bool(predicate())

    def check_interrupt(self):
        if self.interrupt_requested is not None:
            signum = self.interrupt_requested
            self.interrupt_requested = None
            raise KeyboardInterrupt(f'signal {signum}')

    def parameter_snapshot(self):
        snapshots = {}
        for name in ('local_costmap/local_costmap', 'global_costmap/global_costmap',
                     'controller_server', 'planner_server', 'behavior_server', 'bt_navigator'):
            services = []
            try:
                listing = self.create_client(ListParameters, f'/{name}/list_parameters')
                services.append(listing)
                getting = self.create_client(GetParameters, f'/{name}/get_parameters')
                services.append(getting)
                if not listing.wait_for_service(timeout_sec=.5):
                    raise RuntimeError('parameter service unavailable')
                future = listing.call_async(ListParameters.Request(depth=0))
                if not self.spin_until(future.done, 2):
                    raise RuntimeError('parameter listing timeout')
                names = list(future.result().result.names)
                future = getting.call_async(GetParameters.Request(names=names))
                if not self.spin_until(future.done, 2):
                    raise RuntimeError('parameter reading timeout')
                snapshots[name] = dict(zip(names, (parameter_value_to_python(v)
                                                   for v in future.result().values)))
            except Exception as exc:
                snapshots[name] = {'_error': str(exc)}
            finally:
                for service in services:
                    self.destroy_client(service)
        self.record('parameters', values=snapshots)
        return snapshots

    def installed_tree(self, parameters):
        path = parameters.get('bt_navigator', {}).get('default_nav_to_pose_bt_xml')
        if not path:
            return {'error': 'default_nav_to_pose_bt_xml is unavailable'}
        try:
            content = Path(path).read_bytes()
            return {'path': path, 'sha256': hashlib.sha256(content).hexdigest(),
                    'xml': content.decode()}
        except OSError as exc:
            return {'path': path, 'error': str(exc)}

    def geometry(self, parameters):
        result = {}
        for name in ('local_costmap', 'global_costmap'):
            snapshot = {'parameters': parameters.get(f'{name}/{name}', {})}
            if name in self.polygons:
                message, received = self.polygons[name]
                try:
                    transform = self.tf.lookup_transform(message.header.frame_id, 'base_footprint',
                                                         Time.from_msg(message.header.stamp))
                    t, q = transform.transform.translation, transform.transform.rotation
                    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                    points = points_in_base([(p.x, p.y) for p in message.polygon.points],
                                            (t.x, t.y, yaw))
                    snapshot['published'] = {'points_base': points,
                                              'age_s': time.monotonic() - received,
                                              'source_age_s': (self.get_clock().now() - Time.from_msg(
                                                  message.header.stamp)).nanoseconds / 1e9,
                                              'header': message_to_ordereddict(message.header)}
                except TransformException as exc:
                    snapshot['transform_error'] = str(exc)
            result[name] = {**check_costmap(snapshot), 'evidence': snapshot}
        result['consumers'] = check_consumers(parameters)
        return result

    def feedback(self, message):
        self.recoveries = message.feedback.number_of_recoveries
        self.record('navigation_feedback', message.feedback)

    def execute(self, target, timeout):
        if 'mision' in self.get_node_names():
            raise RuntimeError('Mission is running. Use navigation-only launch for controlled trials.')
        if not self.map_seen or self.localized is None \
                or time.monotonic() - self.localized[1] > .5:
            raise RuntimeError('A map and a fresh localized pose are required before a goal.')
        if not self.action.wait_for_server(timeout_sec=3):
            raise RuntimeError('navigate_to_pose is unavailable')
        # Discovery may take seconds. Process queued TF and recheck at dispatch.
        rclpy.spin_once(self, timeout_sec=0)
        self.check_interrupt()
        if self.localized is None or time.monotonic() - self.localized[1] > .5:
            raise RuntimeError('A fresh localized pose is required before dispatch.')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2] / 2)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2)
        self.record('requested_goal', goal)
        self.dispatch_started = True
        self.acceptance = self.action.send_goal_async(goal, feedback_callback=self.feedback)
        if not self.spin_until(self.acceptance.done, 5):
            raise RuntimeError('Goal acceptance timed out; goal state is unknown')
        self.goal_handle = self.acceptance.result()
        if not self.goal_handle.accepted:
            self.status = -1
            return
        self.result = self.goal_handle.get_result_async()
        if not self.spin_until(self.result.done, timeout):
            raise RuntimeError('Controlled goal timed out')
        response = self.result.result()
        self.status = response.status
        self.arrival = self.current_trial(target)
        self.record('navigation_result', response.result, status=response.status)

    def stop_after_error(self):
        """An uncertain trial cannot leave a moving goal behind."""
        self.stop_confirmed = False
        self.inhibit.publish(Bool(data=True))
        self.record('terminal_stop_requested', restart_navigation_required=True)
        if self.acceptance is not None and not self.acceptance.done():
            self.spin_until(self.acceptance.done, 2)
        if self.goal_handle is None and self.acceptance is not None and self.acceptance.done():
            try:
                self.goal_handle = self.acceptance.result()
            except Exception:
                pass
        if self.goal_handle is not None and self.goal_handle.accepted:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.record('cancel_error', error=str(exc))
        deadline = time.monotonic() + 3
        while not self.stop_confirmed and time.monotonic() < deadline:
            self.inhibit.publish(Bool(data=True))
            self.spin_until(lambda: self.stop_confirmed, .1)
        self.record('terminal_stop_result', confirmed=self.stop_confirmed)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new directory; never overwrites')
    parser.add_argument('--case', choices=('audit', 'wide', 'problem'), default='audit')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='send one controlled goal; omit for a read-only audit')
    parser.add_argument('--duration', type=float, default=5, help='read-only observation wall seconds')
    parser.add_argument('--timeout', type=float, default=120, help='goal wall-time budget')
    parser.add_argument('--truth-topic', help='simulation/test only; e.g. /ground_truth/odom')
    parser.add_argument('--truth-frame-is-map', action='store_true',
                        help='explicitly declare truth frame aligned with map (recorded in report)')
    args, ros_args = parser.parse_known_args(argv)
    if (not all(math.isfinite(v) and v > 0 for v in (args.duration, args.timeout))
            or args.goal is not None and not all(math.isfinite(v) for v in args.goal)):
        parser.error('durations must be positive and all goal values finite')
    # Keep ROS alive until inhibit/cancel have been delivered on interruption.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    pending_signal = None
    def interrupt(signum, frame):
        # Never throw through a ROS callback: let spin_once finish its cleanup.
        nonlocal pending_signal
        pending_signal = signum
        if node is not None:
            node.interrupt_requested = signum
    previous_handlers = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    report = {'case': args.case, 'goal': args.goal, 'movement_test': 'not_run',
              'truth_topic': args.truth_topic, 'truth_frame_is_map': args.truth_frame_is_map,
              'collision_free': None}
    failed = False
    try:
        node = Recorder(args.output, args.truth_topic, args.truth_frame_is_map)
        node.interrupt_requested = pending_signal
        report['ros_distro'] = __import__('os').environ.get('ROS_DISTRO')
        version = subprocess.run(['dpkg-query', '-W', 'ros-humble-nav2-*'],
                                 capture_output=True, text=True, check=False)
        report['installed_nav2_packages'] = version.stdout
        node.spin_until(lambda: False, args.duration)
        parameters = node.parameter_snapshot()
        report['parameters_before'] = parameters
        report['installed_tree'] = node.installed_tree(parameters)
        report['geometry_before'] = node.geometry(parameters)
        if args.goal is not None:
            if any(v['status'] != 'confirmed' for v in report['geometry_before'].values()):
                raise RuntimeError('Geometry/consumer checks incomplete or mismatched; no goal sent')
            report['movement_test'] = 'attempted'
            node.execute(args.goal, args.timeout)
            node.spin_until(lambda: False, .5)
        report['parameters_after'] = node.parameter_snapshot()
        report['geometry_after'] = node.geometry(report['parameters_after'])
    except (Exception, KeyboardInterrupt) as exc:
        for signum in previous_handlers:
            signal.signal(signum, signal.SIG_IGN)
        failed = True
        report['error'] = f'{type(exc).__name__}: {exc}'
        if node is not None and node.dispatch_started:
            node.interrupt_requested = None
            node.stop_after_error()
            report['terminal_stop_confirmed'] = node.stop_confirmed
    finally:
        # A repeated signal must not interrupt the bounded stop/flush cleanup.
        for signum in previous_handlers:
            signal.signal(signum, signal.SIG_IGN)
        if node is not None:
            if args.goal is not None:
                report['trial'] = node.arrival or node.current_trial(args.goal)
            (args.output / 'summary.json').write_text(evidence_json(report, indent=2) + '\n')
            node.stream.close()
            node.destroy_node()
        rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    print(evidence_json(report, indent=2))
    checks = report.get('geometry_after', {})
    return 1 if failed or not checks or any(v['status'] != 'confirmed' for v in checks.values()) \
        or args.goal is not None and node.status != 4 else 0


if __name__ == '__main__':
    raise SystemExit(main())
