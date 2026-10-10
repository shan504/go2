"""Native-avoid motion/input release tests on isolated ROS mock endpoints."""
import copy
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from unitree_api.msg import Request, Response


class Robot(Node):
    def __init__(self):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control', False)
        self.declare_parameter('allow_motion', True)
        self.events = []
        self.enabled = True
        self.reject_source = False
        self.drop_source_reply = False
        self.reenable = False
        self.reject_move = False
        self.selected = False
        self.response_pubs = {}
        for service in ('sport', 'obstacles_avoid'):
            self.response_pubs[service] = self.create_publisher(Response, f'/api/{service}/response', 10)
            self.create_subscription(Request, f'/api/{service}/request', self.callback_for(service), 10)

    def callback_for(self, service):
        def receive(message):
            api = message.header.identity.api_id
            parameter = json.loads(message.parameter)
            self.events.append((service, api, parameter, time.monotonic()))
            response = Response()
            response.header.identity = copy.deepcopy(message.header.identity)
            response.data = '{}'
            if service == 'obstacles_avoid':
                if api == 1002:
                    response.data = json.dumps({'enable': self.enabled})
                elif api == 1004:
                    selected = parameter['is_remote_commands_from_api']
                    if selected and self.reject_source:
                        response.header.status.code = 5432
                    else:
                        self.selected = selected
                    if selected and self.drop_source_reply:
                        return
                elif api == 1003:
                    if self.reject_move and parameter['x'] != 0:
                        response.header.status.code = 5433
                    if self.reenable and parameter['x'] != 0:
                        self.set_parameters([Parameter('enable_control', value=True)])
            self.response_pubs[service].publish(response)
        return receive


script = Path(__file__).resolve().parents[1]/'go2_3d/native_drivecheck.py'
rclpy.init()
robot = Robot()
executor = SingleThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()


def invoke(interrupt=None):
    before = len(robot.events)
    process = subprocess.Popen([sys.executable, str(script)], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    if interrupt is not None:
        deadline = time.monotonic()+6
        while time.monotonic()<deadline and process.poll() is None:
            if any(event[0:2] == ('obstacles_avoid', 1003) and event[2]['x'] != 0
                   for event in robot.events[before:]):
                process.send_signal(interrupt)
                break
            time.sleep(0.02)
        else:
            process.kill()
            raise AssertionError('No native Move observed before interruption')
    output, _ = process.communicate(timeout=10)
    time.sleep(0.05)
    return process.returncode, robot.events[before:], output


def assert_released(events, output):
    native = [event for event in events if event[0] == 'obstacles_avoid']
    assert native[-1][:3] == ('obstacles_avoid', 1004, {'is_remote_commands_from_api': False}), events
    assert robot.selected is False
    assert 'release_acknowledged=True' in output, output
    assert any(event[:3] == ('obstacles_avoid', 1003,
                            {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'mode': 0}) for event in events)


try:
    code, events, output = invoke()
    assert code == 0, output
    moves = [event for event in events if event[:2] == ('obstacles_avoid', 1003) and event[2]['x'] != 0]
    assert 30 <= len(moves) <= 40, (len(moves), output)
    assert all(event[2] == {'x': 0.15, 'y': 0.0, 'yaw': 0.0, 'mode': 0} for event in moves)
    assert 1.5 < moves[-1][3]-moves[0][3] < 2.1
    assert any(event[:2] == ('sport', 1003) for event in events), 'Missing Sport StopMove'
    assert all(event[1] in (1003, 1034) for event in events if event[0] == 'sport'), events
    assert all(event[1] in (1002, 1003, 1004) for event in events if event[0] == 'obstacles_avoid'), \
        'Test must not switch avoidance off or change modes'
    assert_released(events, output)
    for interrupt in (signal.SIGINT, signal.SIGTERM):
        code, events, output = invoke(interrupt)
        assert code == 130, output
        assert_released(events, output)
    robot.enabled = False
    code, events, output = invoke()
    assert code == 2 and 'not enabled' in output, output
    assert all(event[1] == 1002 for event in events if event[0] == 'obstacles_avoid')
    robot.enabled = True
    robot.reject_source = True
    code, events, output = invoke()
    assert code == 2 and 'code=5432' in output, output
    assert_released(events, output)
    robot.reject_source = False
    robot.drop_source_reply = True
    code, events, output = invoke()
    assert code == 2 and 'no matching reply' in output, output
    assert_released(events, output)
    robot.drop_source_reply = False
    robot.reject_move = True
    code, events, output = invoke()
    assert code == 2 and 'code=5433' in output, output
    assert_released(events, output)
    robot.reject_move = False
    robot.reenable = True
    code, events, output = invoke()
    assert code == 2 and 'first disable' in output, output
    assert_released(events, output)
    assert sum(event[:2] == ('obstacles_avoid', 1003) and event[2]['x'] != 0 for event in events) < 30
    print('PASS native-avoid: exact API input/Move yaw/mode, fixed 2s, zero+Stop+release, '
          'SIGINT/SIGTERM, disabled avoidance refusal, rejected/lost source reply cleanup, bridge re-enable stop')
finally:
    executor.shutdown()
    thread.join(timeout=2)
    robot.destroy_node()
    rclpy.shutdown()
