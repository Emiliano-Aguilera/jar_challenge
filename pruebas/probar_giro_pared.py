"""Local simulator-only goal probe; never launched by the competition package.

Only Nav2 publishes velocities. This client records navigation results and
simulator truth for evaluation; it does not change production planning.
"""
import argparse
import json
import math
from pathlib import Path
import time

from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as ViewPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from equipo_jar.comun import QOS_ESTADO


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('output must stay under build/')
    goals = json.loads(args.goals.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('coverage_wall_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    state = node.create_client(GetState, '/bt_navigator/get_state')
    truth = {}
    plans = []
    node.create_subscription(ViewPath, '/jar/plan_cobertura', plans.append, QOS_ESTADO)
    def actual(message):
        p = message.pose.pose
        truth.update(x=p.position.x, y=p.position.y, yaw=yaw(p.orientation))
    node.create_subscription(Odometry, '/ground_truth/odom', actual, qos_profile_sensor_data)
    def wait(future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        return future.done()
    records = []
    try:
        deadline = time.monotonic() + 160
        while time.monotonic() < deadline:
            if state.wait_for_service(timeout_sec=.2):
                future = state.call_async(GetState.Request())
                if wait(future, min(deadline, time.monotonic() + 3)) and future.result().current_state.id == 3:
                    break
            rclpy.spin_once(node, timeout_sec=.05)
        else:
            raise RuntimeError('Nav2 did not become active')
        assert client.wait_for_server(timeout_sec=5)
        simulated_ready = time.monotonic() + 10
        while (not truth or node.get_clock().now().nanoseconds <= 0) and time.monotonic() < simulated_ready:
            rclpy.spin_once(node, timeout_sec=.05)
        if not truth or node.get_clock().now().nanoseconds <= 0:
            raise RuntimeError('Simulator clock and truth odometry are required for this evaluation client')
        ready_sim = node.get_clock().now().nanoseconds / 1e9
        wait_until = time.monotonic() + 60
        while (node.get_clock().now().nanoseconds / 1e9 - ready_sim < goals.get('delay_sim', 0.0)
               and time.monotonic() < wait_until):
            rclpy.spin_once(node, timeout_sec=.05)
        if node.get_clock().now().nanoseconds / 1e9 - ready_sim < goals.get('delay_sim', 0.0):
            raise RuntimeError('Simulation did not advance through the requested warmup')
        if goals.get('from_plan'):
            while not plans and time.monotonic() < wait_until:
                rclpy.spin_once(node, timeout_sec=.05)
            if not plans:
                raise RuntimeError('Coverage did not publish its actual plan')
            actual = [dict(x=p.pose.position.x, y=p.pose.position.y, yaw=yaw(p.pose.orientation))
                      for p in plans[-1].poses]
            fx, fy = goals['focus']
            nearest = min(range(len(actual)), key=lambda i: math.hypot(actual[i]['x'] - fx, actual[i]['y'] - fy))
            begin, end = max(0, nearest - 1), min(len(actual), nearest + 4)
            goals.update(goals=actual[begin:end], path_indices=list(range(begin, end)), actual_path=actual)
        for target in goals['goals']:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target['x'], target['y']
            q = goal.pose.pose.orientation
            q.z, q.w = math.sin(target['yaw'] / 2), math.cos(target['yaw'] / 2)
            wall_begin = time.monotonic()
            sim_begin = node.get_clock().now().nanoseconds / 1e9
            result = dict(target=target, truth_before=dict(truth))
            request = client.send_goal_async(goal)
            if not wait(request, wall_begin + 10):
                raise RuntimeError('Nav2 goal response timed out')
            handle = request.result()
            result['accepted'] = handle.accepted
            if handle.accepted:
                response = handle.get_result_async()
                completed = wait(response, wall_begin + args.timeout)
                result['timed_out'] = not completed
                if not completed:
                    cancellation = handle.cancel_goal_async()
                    wait(cancellation, time.monotonic() + 8)
                    wait(response, time.monotonic() + 8)
                result['status'] = response.result().status if response.done() else None
            result.update(wall_seconds=time.monotonic() - wall_begin,
                          sim_seconds=node.get_clock().now().nanoseconds / 1e9 - sim_begin,
                          truth_after=dict(truth))
            result['rtf'] = result['sim_seconds'] / result['wall_seconds']
            records.append(result)
            args.output.write_text(json.dumps(dict(configuration=goals, results=records), indent=2)+'\n')
            print(json.dumps(result), flush=True)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
