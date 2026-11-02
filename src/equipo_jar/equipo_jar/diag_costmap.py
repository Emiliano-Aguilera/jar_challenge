"""Diagnostic only (branch navegacion/diag-costmap, never merged).

Logs whether the local costmap window follows the robot, and how far the global plan
starts from the robot. Looking for the cause of MPPI's 'Resulting plan has 0 poses'.
"""
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class DiagCostmap(Node):
    def __init__(self):
        super().__init__('diag_costmap')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_costmap, qos)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.window = None          # (arrival s, stamp s, center x, center y)
        self.count = 0
        self.last_warn = 0.0
        self.create_timer(2.0, self.report)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self, frame):
        try:
            t = self.tf.lookup_transform(frame, 'base_footprint', Time())
        except Exception as exc:  # noqa: BLE001 - diagnostic
            return None, str(exc)[:80]
        p = t.transform.translation
        return (p.x, p.y, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9), None

    def on_costmap(self, msg):
        info = msg.info
        cx = info.origin.position.x + info.width * info.resolution / 2
        cy = info.origin.position.y + info.height * info.resolution / 2
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.window = (self.now(), stamp, cx, cy)
        self.count += 1
        robot, err = self.pose(msg.header.frame_id or 'odom')
        if robot is not None:
            off = math.hypot(robot[0] - cx, robot[1] - cy)
            if off > 0.5:
                self.get_logger().warn(
                    f'DIAG window {off:.2f} m from robot: center ({cx:.2f},{cy:.2f}) '
                    f'robot ({robot[0]:.2f},{robot[1]:.2f}) stamp age {robot[2] - stamp:.2f} s')

    def on_plan(self, msg):
        if not msg.poses:
            return
        robot, err = self.pose(msg.header.frame_id or 'map')
        if robot is None:
            return
        first = msg.poses[0].pose.position
        d = math.hypot(first.x - robot[0], first.y - robot[1])
        if d > 0.3:
            self.get_logger().warn(
                f'DIAG plan starts {d:.2f} m from robot: first ({first.x:.2f},{first.y:.2f}) '
                f'robot ({robot[0]:.2f},{robot[1]:.2f}) poses {len(msg.poses)}')

    def report(self):
        if self.window is None:
            self.get_logger().info('DIAG no local costmap yet')
            return
        age = self.now() - self.window[0]
        robot, err = self.pose('odom')
        off = (math.hypot(robot[0] - self.window[2], robot[1] - self.window[3])
               if robot else float('nan'))
        line = (f'DIAG costmap msgs {self.count}, last {age:.1f} s ago, window {off:.2f} m '
                f'from robot{"" if robot else " (tf: " + err + ")"}')
        if age > 2.0 or off > 0.5:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)


def main():
    rclpy.init()
    node = DiagCostmap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
