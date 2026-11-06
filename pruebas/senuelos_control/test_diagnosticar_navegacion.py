"""Real ROS transport with synthetic evidence; these are not Nav2 motion trials."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from threading import Event, Thread
import time

from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from diagnosticar_navegacion import Recorder, main


@pytest.fixture
def recorder(tmp_path):
    # Other suites create mission nodes; DDS discovery can retain their names
    # briefly after teardown. Keep this transport fixture in its own ROS domain.
    rclpy.init(domain_id=159)
    node = Recorder(tmp_path / 'capture')
    try:
        yield node
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()


def test_missing_runtime_is_reported_as_missing_and_never_sends_a_goal(recorder):
    result = recorder.geometry({})
    assert all(row['status'] == 'missing' for row in result.values())
    assert recorder.acceptance is None


def test_stale_pose_cannot_dispatch_even_after_once_being_available(recorder):
    recorder.map_seen = True
    recorder.localized = ([0., 0., 0.], time.monotonic() - 10)
    with pytest.raises(RuntimeError, match='fresh localized pose'):
        recorder.execute([1., 2., 0.], 3)
    assert recorder.acceptance is None


def test_existing_output_is_preserved(tmp_path):
    directory = tmp_path / 'old'
    directory.mkdir()
    saved = directory / 'events.jsonl'
    saved.write_text('previous evidence\n')
    assert main(['--output', str(directory), '--duration', '.01']) == 1
    assert saved.read_text() == 'previous evidence\n'


def test_reads_effective_parameters_and_stamped_polygons_over_ros(recorder):
    nodes = []
    executor = MultiThreadedExecutor(num_threads=2)
    for name in ('local_costmap', 'global_costmap'):
        node = Node(name, namespace=name)
        node.declare_parameter('footprint', '[[0.119, 0.1], [0.119, -0.1], [-0.181, -0.1], [-0.181, 0.1]]')
        node.declare_parameter('footprint_padding', .02)
        node.declare_parameter('robot_base_frame', 'base_footprint')
        pub = node.create_publisher(PolygonStamped, f'/{name}/published_footprint', 10)

        def publish(n=node, publisher=pub):
            message = PolygonStamped()
            message.header.frame_id = 'base_footprint'
            message.header.stamp = n.get_clock().now().to_msg()
            message.polygon.points = [Point32(x=x, y=y) for x, y in
                                      ((.139, .12), (.139, -.12), (-.201, -.12), (-.201, .12))]
            publisher.publish(message)

        node.create_timer(.1, publish)
        nodes.append(node)
    for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator'):
        nodes.append(Node(name))
    for node in nodes:
        executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert recorder.spin_until(lambda: len(recorder.polygons) == 2, 4)
        params = recorder.parameter_snapshot()
        assert params['local_costmap/local_costmap']['footprint_padding'] == .02
        geometry = recorder.geometry(params)
        assert geometry['local_costmap']['status'] == 'confirmed'
        assert geometry['global_costmap']['status'] == 'confirmed'
        # A good polygon cannot hide absent controller/behavior evidence.
        assert geometry['consumers']['status'] == 'missing'
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        for node in nodes:
            node.destroy_node()


def test_records_goal_feedback_result_and_rejects_concurrent_mission(recorder):
    server = Node('diagnostic_fixture')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)

    def execute(handle):
        feedback = NavigateToPose.Feedback()
        feedback.number_of_recoveries = 2
        handle.publish_feedback(feedback)
        handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(server, NavigateToPose, '/navigate_to_pose', execute)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = server.create_publisher(OccupancyGrid, '/map', durable)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    mission = None
    try:
        map_pub.publish(OccupancyGrid())
        assert recorder.spin_until(lambda: recorder.map_seen, 4)
        # Controlled pose boundary. No simulator or collision checker is faked.
        import time
        recorder.localized = ([0, 0, 0], time.monotonic())
        recorder.execute([1., 2., .3], 3)
        assert recorder.status == 4
        assert recorder.recoveries == 2
        rows = [json.loads(line) for line in
                (recorder.destination / 'events.jsonl').read_text().splitlines()]
        request = next(row['message'] for row in rows if row['event'] == 'requested_goal')
        assert request['pose']['header']['frame_id'] == 'map'
        assert request['pose']['pose']['position']['x'] == 1.
        assert any(row['event'] == 'navigation_result' and row['status'] == 4 for row in rows)
        mission = Node('mision')
        assert recorder.spin_until(lambda: 'mision' in recorder.get_node_names(), 4)
        previous = recorder.acceptance
        with pytest.raises(RuntimeError, match='Mission is running'):
            recorder.execute([9., 9., 0.], 3)
        assert recorder.acceptance is previous
    finally:
        if mission is not None:
            mission.destroy_node()
        executor.shutdown()
        thread.join(timeout=3)
        action.destroy()
        server.destroy_node()


@pytest.mark.parametrize('interrupt_signal', [signal.SIGINT, signal.SIGTERM, None])
def test_process_interrupt_delivers_terminal_stop_before_ros_shutdown(tmp_path, interrupt_signal):
    """Probe only signal/transport lifecycle, with synthetic startup evidence."""
    domain = 160 + (int(interrupt_signal) if interrupt_signal is not None else 0)
    rclpy.init(domain_id=domain)
    server = Node('interrupt_fixture')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server)
    received, stopped = Event(), Event()
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ack = server.create_publisher(Bool, '/jar/parada_confirmada', durable)

    def on_stop(message):
        if message.data:
            stopped.set()
            ack.publish(Bool(data=True))

    server.create_subscription(Bool, '/jar/parar', on_stop, durable)

    def execute(handle):
        received.set()
        stopped.wait(timeout=10)
        handle.abort()
        return NavigateToPose.Result()

    action = ActionServer(server, NavigateToPose, '/navigate_to_pose', execute)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    # Bypass geometric startup only in the child fixture, to isolate interruption
    # after a real accepted action. Geometry and parameter transport have own tests.
    probe = '''
import sys, time
import diagnosticar_navegacion as d
class Probe(d.Recorder):
    def parameter_snapshot(self):
        return {}
    def geometry(self, parameters):
        return {'fixture': {'status': 'confirmed'}}
    def execute(self, target, timeout):
        self.map_seen = True
        self.localized = ([0., 0., 0.], time.monotonic())
        if sys.argv[2] == 'dispatch':
            original = self.action.send_goal_async
            def interrupted_send(*args, **kwargs):
                original(*args, **kwargs)
                raise KeyboardInterrupt('during dispatch')
            self.action.send_goal_async = interrupted_send
        return super().execute(target, timeout)
d.Recorder = Probe
sys.exit(d.main(['--output', sys.argv[1], '--duration', '.1', '--goal', '1', '2', '0']))
'''
    process = subprocess.Popen([sys.executable, '-c', probe, str(tmp_path / 'probe'),
                                'dispatch' if interrupt_signal is None else 'signal'],
                               cwd=Path(__file__).parent, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True,
                                   env={**os.environ, 'ROS_DOMAIN_ID': str(domain)})
    try:
        if not received.wait(timeout=8):
            if process.poll() is None:
                process.kill()
            stdout, stderr = process.communicate(timeout=3)
            pytest.fail('probe never dispatched its action\n' + stderr + stdout)
        if interrupt_signal is not None:
            process.send_signal(interrupt_signal)
        stdout, stderr = process.communicate(timeout=8)
        assert stopped.is_set(), stderr + stdout
        report = json.loads((tmp_path / 'probe' / 'summary.json').read_text())
        assert report['terminal_stop_confirmed'] is True
        assert 'KeyboardInterrupt' in report['error']
        assert process.returncode == 1
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        stopped.set()
        executor.shutdown()
        thread.join(timeout=3)
        action.destroy()
        server.destroy_node()
        rclpy.shutdown()
