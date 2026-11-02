"""Real Nav2 + synthetic ROS inputs/kinematics. Not a Gazebo/robot safety test."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Fixture(Node):
    def __init__(self):
        super().__init__('passage_transport_fixture')
        self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.cmd_at = self.last = time.monotonic()
        self.phases = []
        self.result = None
        self.sent = False
        self.state_future = None
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.trace = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(String, '/jar/pasaje_estado', self.phase, qos)
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, z in [('map', 'odom', 0.), ('base_footprint', 'laser_link', .18)]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.z = z
            t.transform.rotation.w = 1.
            transforms.append(t)
        self.static.sendTransform(transforms)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width, self.map.info.height, self.map.info.resolution = 180, 100, .025
        self.map.info.origin.position.x, self.map.info.origin.position.y = -1., -1.25
        self.map.info.origin.orientation.w = 1.
        xx = -1 + (np.arange(180) + .5) * .025
        yy = -1.25 + (np.arange(100) + .5) * .025
        self.cells = np.where((xx[None, :] >= .35) & (xx[None, :] <= 1.1)
                              & (np.abs(yy[:, None]) >= .2), 100, 0)
        self.map.data = self.cells.astype('int8').ravel().tolist()
        self.create_timer(.02, self.step)
        self.create_timer(.1, self.scan)
        self.create_timer(.5, self.publish_map)
        self.create_timer(.2, self.goal)

    def command(self, msg):
        self.cmd, self.cmd_at = msg, time.monotonic()

    def phase(self, msg):
        if not self.phases or self.phases[-1] != msg.data:
            self.phases.append(msg.data)
            print(msg.data, flush=True)

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)

    def step(self):
        now = time.monotonic()
        dt, self.last = min(now - self.last, .05), now
        v = self.cmd if now - self.cmd_at < .2 else Twist()
        self.x += (math.cos(self.yaw)*v.linear.x - math.sin(self.yaw)*v.linear.y)*dt
        self.y += (math.sin(self.yaw)*v.linear.x + math.cos(self.yaw)*v.linear.y)*dt
        self.yaw += v.angular.z*dt
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom', 'base_footprint', stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.twist.twist = v
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(t)
        self.trace.append([now, self.x, self.y, self.yaw, v.linear.x, v.linear.y, v.angular.z])

    def scan(self):
        msg = LaserScan()
        msg.header.frame_id = 'laser_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2*math.pi/719
        msg.range_min, msg.range_max, msg.scan_time = .05, 3.0, .1
        directions = self.yaw + np.linspace(-math.pi, math.pi, 720)
        distances = np.arange(.05, 3., .0125)
        px = self.x + np.cos(directions)[:, None] * distances
        py = self.y + np.sin(directions)[:, None] * distances
        ix, iy = np.floor((px+1)/.025).astype(int), np.floor((py+1.25)/.025).astype(int)
        inside = (ix >= 0) & (ix < 180) & (iy >= 0) & (iy < 100)
        hit = inside & (self.cells[np.clip(iy, 0, 99), np.clip(ix, 0, 179)] == 100)
        first = hit.argmax(axis=1)
        msg.ranges = np.where(hit.any(axis=1), distances[first], np.inf).astype('float32').tolist()
        self.scan_pub.publish(msg)

    def goal(self):
        if self.sent or not self.action.server_is_ready() or not self.lifecycle.service_is_ready():
            return
        if self.state_future is None:
            self.state_future = self.lifecycle.call_async(GetState.Request())
            return
        if not self.state_future.done():
            return
        state = self.state_future.result().current_state.id
        self.state_future = None
        if state != 3:
            return
        self.sent = True
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.6
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(.5), math.cos(.5)
        future = self.action.send_goal_async(goal)
        def accepted(future):
            handle = future.result()
            if not handle.accepted:
                self.result = -1
            else:
                handle.get_result_async().add_done_callback(lambda f: setattr(self, 'result', f.result().status))
        future.add_done_callback(accepted)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=80)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    log = (args.output/'navigation.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'equipo_jar', 'navegacion.launch.py',
                                'localizacion:=false', 'use_sim_time:=false'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Fixture()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(s.startswith('hold:') for s in node.phases):
                break
        result = {'action_status': node.result, 'phases': node.phases,
                  'final_pose': [node.x, node.y, node.yaw], 'trace': node.trace,
                  'limitation': 'Synthetic ROS/kinematic integration, not official simulator or physical validation.'}
        (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        ok = node.result == 4 and any(s.startswith('cross:') for s in node.phases) and any(s.startswith('exit:') for s in node.phases)
        print(json.dumps({k:v for k,v in result.items() if k!='trace'}), flush=True)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    raise SystemExit(main())
