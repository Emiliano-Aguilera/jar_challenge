"""One controlled trial per fresh simulator. Test-only service client for Nav2."""
from pathlib import Path
import json
import time
import math
import tempfile

import rclpy
from std_srvs.srv import SetBool, Trigger
from diagnosticar_navegacion import Recorder
from auditoria_navegacion import evidence_json


class Control(Recorder):
    def __init__(self):
        self.truth_samples = []
        self.last_truth_sample = -math.inf
        super().__init__(Path(tempfile.mkdtemp(prefix='jar-control-'))/'capture',
                         '/ground_truth/odom', True)
        self.mode = None
        self.report = {'phase':'armed','note':'One trial per simulator; reset before another.'}
        self.create_service(SetBool, '/jar/diagnostico/iniciar', self.start)
        self.create_service(Trigger, '/jar/diagnostico/estado', self.state)

    def start(self, request, response):
        if self.mode is not None or self.report['phase'] != 'armed':
            response.success=False;response.message='Trial already requested; restart simulator.'
            return response
        self.mode='aligned' if request.data else 'direct'
        response.success=True;response.message=self.mode
        return response

    def state(self, request, response):
        response.success=True
        response.message=evidence_json(self.report)
        return response

    def on_truth(self, message):
        super().on_truth(message)
        now=time.monotonic()
        if self.truth is not None and now-self.last_truth_sample >= .1:
            self.truth_samples.append([now-self.started,*self.truth[0]])
            self.last_truth_sample=now


def main():
    rclpy.init()
    node=Control()
    try:
        while node.mode is None and rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
        config=json.loads((Path(__file__).parent/'casos.json').read_text())
        report={'phase':'running','mode':node.mode,'steps':[],'collision_free':None}
        node.report=report
        try:
            if not node.spin_until(lambda: node.map_seen and node.localized is not None
                                  and node.action.server_is_ready(),45):
                raise RuntimeError('Map, localization or Nav2 unavailable')
            node.spin_until(lambda:False,3)
            params=node.parameter_snapshot()
            report['parameters_before']=params
            report['geometry_before']=node.geometry(params)
            report['installed_tree']=node.installed_tree(params)
            if any(c['status']!='confirmed' for c in report['geometry_before'].values()):
                raise RuntimeError('Effective geometry/consumer audit failed; no goal sent')
            if 'mision' in node.get_node_names():raise RuntimeError('Mission is active; no goal sent')
            report['initial_truth']=node.truth[0] if node.truth else None
            report['initial_localized']=node.localized[0] if node.localized else None
            for target in config[node.mode]:
                node.status=None;node.arrival=None;node.recoveries=None
                start=time.monotonic()
                entry={'target':target,'status':'pending'};report['steps'].append(entry)
                node.execute(target,config['step_timeout_wall_s'])
                node.spin_until(lambda:False,.5)
                entry.update(status=node.status,wall_s=time.monotonic()-start,
                             trial=node.arrival or node.current_trial(target))
                if node.status!=4:raise RuntimeError('Nav2 did not succeed; remaining steps skipped')
            report['success']=True
        except Exception as error:
            report['success']=False;report['error']=f'{type(error).__name__}: {error}'
            if node.dispatch_started:node.stop_after_error()
        # Every trial ends with the permanent inhibit, including successful trials.
        if not node.stop_confirmed:node.stop_after_error()
        report['stop_confirmed']=node.stop_confirmed
        report['truth_samples']=node.truth_samples
        report['parameters_after']=node.parameter_snapshot()
        report['geometry_after']=node.geometry(report['parameters_after'])
        report['phase']='finished'
        node.get_logger().info('CONTROLLED_RESULT '+evidence_json({k:v for k,v in report.items() if k not in ('parameters_before','parameters_after','truth_samples','geometry_before','geometry_after')}))
        (node.destination/'summary.json').write_text(evidence_json(report,indent=2)+'\n')
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.stream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
