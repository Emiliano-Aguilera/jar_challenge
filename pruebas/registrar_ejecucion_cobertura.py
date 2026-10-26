"""Evaluation-only candidate/report/navigation timeline using both ROS and wall time.

Never launched by competition code. Candidate covariance is logged as received;
only mission publishes actual reports. A plan is not a processed observation.
"""
import argparse
import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Path as ViewPath
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from equipo_jar.comun import QOS_ESTADO, QOS_MAPA
from equipo_jar_msgs.msg import Candidatas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('timeline output must stay under build/')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('coverage_timeline', parameter_overrides=[Parameter('use_sim_time', value=True)])
    origin, first_seen, last_logged, statuses = {}, set(), {}, {}
    stream = args.output.open('w')

    def emit(kind, **values):
        wall, simulated = time.monotonic(), node.get_clock().now().nanoseconds / 1e9
        record = dict(event=kind, wall_clock=wall, sim_clock=simulated, **values)
        if origin:
            record.update(elapsed_wall=wall-origin['wall'], elapsed_sim=simulated-origin['sim'])
        stream.write(json.dumps(record)+'\n')
        stream.flush()

    def received_map(message):
        if not origin:
            origin.update(wall=time.monotonic(), sim=node.get_clock().now().nanoseconds / 1e9)
            emit('map', width=message.info.width, height=message.info.height)

    def candidates(message):
        now = node.get_clock().now().nanoseconds / 1e9
        for candidate in message.candidatas:
            initial = candidate.id not in first_seen
            if not initial and now-last_logged.get(candidate.id, -1e10) < 1.0:
                continue
            first_seen.add(candidate.id)
            last_logged[candidate.id] = now
            emit('candidate_first' if initial else 'candidate', id=candidate.id,
                 x=candidate.centro.x, y=candidate.centro.y,
                 observations=candidate.observaciones, covariance=list(candidate.covarianza_xy))

    def navigation(message):
        for status in message.status_list:
            identity = bytes(status.goal_info.goal_id.uuid).hex()
            if statuses.get(identity) != status.status:
                statuses[identity] = status.status
                emit('navigation_status', id=identity, status=status.status)

    node.create_subscription(OccupancyGrid, '/map', received_map, QOS_MAPA)
    node.create_subscription(Candidatas, '/jar/candidatas', candidates, QOS_ESTADO)
    node.create_subscription(ViewPath, '/jar/plan_cobertura',
                             lambda message: emit('plan', goals=len(message.poses)), QOS_ESTADO)
    node.create_subscription(PointStamped, '/victimas',
                             lambda message: emit('report', x=message.point.x, y=message.point.y), 10)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', navigation, 10)
    node.create_timer(1.0, lambda: emit('clock'))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        emit('recorder_stopped')
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
