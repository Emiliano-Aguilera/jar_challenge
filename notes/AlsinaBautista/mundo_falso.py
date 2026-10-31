"""Fake world to test mission alone: map, TF, coverage plan, fusion and Nav2, no simulator.

Map: 6 x 4 m in an S (three corridors joined at alternating ends), start (0.5, 0.5).
One red box at (4.5, 3.3), at the far end: the way back from it is ~15 m.

- Nav2 (navigate_to_pose): fails, after 0.5 s, every goal whose path on the map is longer
  than `limit_m` (like NavFn's limit in Humble); otherwise waits path / `speed` and moves
  the robot to the goal.
- Fusion (/jar/candidatas): the box appears with 2 observations and 0.15 m of error when
  the robot gets within 2.5 m. It gains observations only while the robot stands still
  within 1.2 m of it, and not at the first spot where it stood (`hide_first_view`: as in
  Ventanas, that view gives nothing). At 5 observations the error drops to 0.05 m.
- At /done it prints one RESULT line (reports, time, distance to the start) and exits.

    python3 notes/AlsinaBautista/mundo_falso.py
    python3 notes/AlsinaBautista/mundo_falso.py --ros-args -p limit_m:=100.0 -p hide_first_view:=false

Mission in another terminal (N = 1):
    ros2 run equipo_jar mision --ros-args --params-file src/equipo_jar/params/params.yaml -p cantidad_victimas:=1
Only for local tests; notes/ is not part of the delivery.
"""
import math
import time

import rclpy
from equipo_jar.comun import QOS_ESTADO, QOS_MAPA
from equipo_jar.mapa import Grid
from equipo_jar.vuelta import steps_to_start, way_back_m
from equipo_jar_msgs.msg import Candidata, Candidatas
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from tf2_ros import TransformBroadcaster

RES = 0.05
WIDTH, HEIGHT = 120, 80
START = (0.5, 0.5)
BOX = (4.5, 3.3)
# Coverage plan: x, y, yaw along the S.
PLAN = [(2.5, 0.6, 0.0), (4.5, 0.6, 0.0), (5.4, 1.9, math.pi), (3.5, 1.9, math.pi),
        (1.5, 1.9, math.pi), (0.6, 3.3, 0.0), (2.5, 3.3, 0.0)]


def s_map():
    """Cells of the S: borders, a wall at y 1.20-1.30 from x 0 to 5, one at y 2.50-2.60 from x 1 to 6."""
    data = [0] * (WIDTH * HEIGHT)
    for j in range(HEIGHT):
        for i in range(WIDTH):
            edge = i in (0, WIDTH - 1) or j in (0, HEIGHT - 1)
            low = j in (24, 25) and i < 100
            high = j in (50, 51) and i >= 20
            if edge or low or high:
                data[j * WIDTH + i] = 100
    return data


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FakeWorld(Node):
    def __init__(self):
        super().__init__('mundo_falso')
        self.declare_parameter('limit_m', 8.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('hide_first_view', True)
        self.data = s_map()
        self.grid = Grid(WIDTH, HEIGHT, RES, self.data)
        self.pose = (START[0], START[1], 0.0)
        self.moved_at = time.monotonic()
        self.t0 = None
        self.detected = False
        self.observations = 0
        self.first_view = None
        self.reports = []
        self.group = ReentrantCallbackGroup()

        self.tf = TransformBroadcaster(self)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', QOS_MAPA)
        self.pub_plan = self.create_publisher(Path, '/jar/plan_cobertura', QOS_ESTADO)
        self.pub_cands = self.create_publisher(Candidatas, '/jar/candidatas', QOS_ESTADO)
        self.pub_ready = self.create_publisher(Bool, '/jar/localizacion_lista', QOS_ESTADO)
        self.create_subscription(PointStamped, '/victimas', self.on_report, 10)
        self.create_subscription(Empty, '/done', self.on_done, 10)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.navigate,
                     cancel_callback=lambda request: CancelResponse.ACCEPT,
                     callback_group=self.group)
        self.create_timer(0.05, self.publish_tf, callback_group=self.group)
        self.create_timer(0.2, self.fusion, callback_group=self.group)
        self.create_timer(2.0, self.publish_map_once, callback_group=self.group)
        self.done = False

    def seconds(self):
        return 0.0 if self.t0 is None else time.monotonic() - self.t0

    def publish_map_once(self):
        if self.t0 is not None:
            return
        stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp
        msg.info.resolution = RES
        msg.info.width, msg.info.height = WIDTH, HEIGHT
        msg.info.origin.orientation.w = 1.0
        msg.data = self.data
        self.pub_map.publish(msg)
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = stamp
        for x, y, yaw in PLAN:
            p = PoseStamped()
            p.header = plan.header
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            plan.poses.append(p)
        self.pub_plan.publish(plan)
        self.pub_ready.publish(Bool(data=True))
        self.t0 = time.monotonic()
        self.get_logger().info('t0: /map, plan and localization ready published')

    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.pose
        a = TransformStamped()
        a.header.stamp, a.header.frame_id, a.child_frame_id = stamp, 'map', 'odom'
        a.transform.rotation.w = 1.0
        b = TransformStamped()
        b.header.stamp, b.header.frame_id, b.child_frame_id = stamp, 'odom', 'base_footprint'
        b.transform.translation.x, b.transform.translation.y = x, y
        b.transform.rotation.z, b.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform([a, b])

    def path_m(self, gx, gy):
        steps = steps_to_start(self.grid, gx, gy, 0.10, 25, 0.3)
        return way_back_m(self.grid, steps, self.pose[0], self.pose[1], 0.3)

    def navigate(self, goal_handle):
        p = goal_handle.request.pose.pose
        gx, gy, gyaw = p.position.x, p.position.y, yaw_of(p.orientation)
        length = self.path_m(gx, gy)
        limit = self.get_parameter('limit_m').value
        if length is None or length > limit:
            self.get_logger().info(f'[{self.seconds():5.1f} s] goal ({gx:.2f},{gy:.2f}): path '
                                   f'{length if length is None else round(length, 1)} m > {limit} m, plan fails')
            time.sleep(0.5)
            goal_handle.abort()
            return NavigateToPose.Result()
        end = time.monotonic() + max(0.3, length / self.get_parameter('speed').value)
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)
        self.pose = (gx, gy, gyaw)
        self.moved_at = time.monotonic()
        self.get_logger().info(f'[{self.seconds():5.1f} s] arrived ({gx:.2f},{gy:.2f}), path {length:.1f} m')
        goal_handle.succeed()
        return NavigateToPose.Result()

    def fusion(self):
        if self.t0 is None:
            return
        x, y, _ = self.pose
        dist = math.hypot(x - BOX[0], y - BOX[1])
        if not self.detected and dist < 2.5:
            self.detected = True
            self.observations = 2
            self.get_logger().info(f'[{self.seconds():5.1f} s] fusion: box seen from far (2 obs, 0.15 m)')
        still = time.monotonic() - self.moved_at > 0.3
        if self.detected and still and dist < 1.2:
            if self.first_view is None:
                self.first_view = (x, y)
                if self.get_parameter('hide_first_view').value:
                    self.get_logger().info(f'[{self.seconds():5.1f} s] fusion: nothing from ({x:.2f},{y:.2f}) (hidden view)')
            hidden = (self.get_parameter('hide_first_view').value
                      and math.hypot(x - self.first_view[0], y - self.first_view[1]) < 0.3)
            if not hidden:
                self.observations += 1
        if not self.detected:
            return
        sigma = 0.05 if self.observations >= 5 else 0.15
        c = Candidata()
        c.id = 1
        c.centro.x, c.centro.y = BOX[0] + 0.05, BOX[1]
        c.covarianza_xy = [sigma ** 2, 0.0, 0.0, sigma ** 2]
        c.observaciones = self.observations
        msg = Candidatas()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.candidatas = [c]
        self.pub_cands.publish(msg)

    def on_report(self, msg):
        err = math.hypot(msg.point.x - BOX[0], msg.point.y - BOX[1])
        self.reports.append((round(self.seconds(), 1), round(err, 2)))
        self.get_logger().info(f'[{self.seconds():5.1f} s] REPORT ({msg.point.x:.2f},{msg.point.y:.2f}), {err:.2f} m from the box')

    def on_done(self, msg):
        if self.done:
            return
        self.done = True
        back = math.hypot(self.pose[0] - START[0], self.pose[1] - START[1])
        self.get_logger().info(f'RESULT reports(t, error)={self.reports} done_at={self.seconds():.1f}s '
                               f'distance_to_start={back:.2f}m')


def main():
    rclpy.init()
    node = FakeWorld()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
