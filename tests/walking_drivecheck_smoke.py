"""Real ROS endpoints, isolated from hardware: fixed speed, telemetry and Stop."""
import copy
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from unitree_api.msg import Request, Response


class MockRobot(Node):
    def __init__(self):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control', False)
        self.declare_parameter('allow_motion', True)
        self.requests, self.odom_y = [], 0.0
        self.emit_odom = self.walk = True
        self.drop_odom_on_move = self.reenable = False
        self.reject_move = self.drop_stop = False
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.responses = self.create_publisher(Response, '/api/sport/response', 10)
        self.create_subscription(Request, '/api/sport/request', self.receive, 10)
        self.create_timer(0.02, self.telemetry)

    def telemetry(self):
        if not self.emit_odom:
            return
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.y = self.odom_y
        message.pose.pose.orientation.z = math.sin(math.pi/4)
        message.pose.pose.orientation.w = math.cos(math.pi/4)
        self.odom.publish(message)

    def receive(self, message):
        api = message.header.identity.api_id
        self.requests.append(message)
        if api == 1008:
            if self.drop_odom_on_move:
                self.emit_odom = False
            if self.walk:
                self.odom_y += 0.015  # 0.30m/s at the test's 20Hz, initial heading +Y.
            if self.reenable:
                self.set_parameters([Parameter('enable_control', value=True)])
        if api == 1003 and self.drop_stop:
            return
        reply = Response()
        reply.header.identity = copy.deepcopy(message.header.identity)
        reply.header.status.code = 5433 if api == 1008 and self.reject_move else 0
        self.responses.publish(reply)
        reply.header.identity.id += 1
        reply.header.status.code = 9999
        self.responses.publish(reply)


script = Path(__file__).resolve().parents[1]/'go2_3d/walking_drivecheck.py'
rclpy.init()
robot = MockRobot()
executor = SingleThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()


def invoke(interrupt=None):
    before = len(robot.requests)
    process = subprocess.Popen([sys.executable, str(script)], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    if interrupt is not None:
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and process.poll() is None:
            if any(request.header.identity.api_id == 1008 for request in robot.requests[before:]):
                process.send_signal(interrupt)
                break
            time.sleep(0.02)
        else:
            process.kill()
            raise AssertionError('No Move observed before interruption')
    output, _ = process.communicate(timeout=9)
    time.sleep(0.05)
    observed = robot.requests[before:]
    assert all(request.header.identity.api_id in (1008, 1003, 1034) for request in observed)
    controls = [request for request in observed if request.header.identity.api_id != 1034]
    return process.returncode, controls, output


try:
    code, requests, output = invoke()
    assert code == 0, output
    moves = [request for request in requests if request.header.identity.api_id == 1008]
    assert 30 <= len(moves) <= 41, (len(moves), output)
    assert all(json.loads(request.parameter) == {'x': 0.30, 'y': 0.0, 'z': 0.0} for request in moves)
    assert 1.5 < (moves[-1].header.identity.id-moves[0].header.identity.id)/1e9 <= 2.1
    assert requests[-1].header.identity.api_id == 1003
    assert 'StopMove API1003 code=0' in output and '9999' not in output, output
    line = next(line for line in output.splitlines() if line.startswith('/odom during Move:'))
    fields = dict(part.split('=', 1) for part in line.split() if '=' in part)
    assert 0.4 <= float(fields['forward'].removesuffix('m')) <= 0.61, output
    assert abs(float(fields['lateral'].removesuffix('m'))) < 0.001, output
    assert 'confirm actual footsteps' in output, output

    robot.walk = False
    code, requests, output = invoke()
    assert code == 0 and 'net_XY=0.000m forward=0.000m lateral=0.000m' in output, output
    for interrupt in (signal.SIGINT, signal.SIGTERM):
        code, requests, output = invoke(interrupt)
        assert code == 130 and requests[-1].header.identity.api_id == 1003, output
    robot.set_parameters([Parameter('enable_control', value=True)])
    code, requests, output = invoke()
    assert code == 2 and requests == [] and 'first disable' in output, output
    robot.set_parameters([Parameter('enable_control', value=False)])

    robot.emit_odom = False
    code, requests, output = invoke()
    assert code == 2 and requests == [] and 'Fresh /odom' in output, output
    robot.emit_odom = True
    robot.drop_odom_on_move = True
    code, requests, output = invoke()
    assert code == 2 and requests[-1].header.identity.api_id == 1003, output
    assert 1 <= sum(request.header.identity.api_id == 1008 for request in requests) < 30
    robot.drop_odom_on_move = False
    robot.emit_odom = True

    robot.reject_move = True
    code, requests, output = invoke()
    assert code == 2 and 'Move rejected with code=5433' in output, output
    assert requests[-1].header.identity.api_id == 1003
    robot.reject_move = False
    robot.reenable = True
    code, requests, output = invoke()
    assert code == 2 and requests[-1].header.identity.api_id == 1003, output
    robot.reenable = False
    robot.set_parameters([Parameter('enable_control', value=False)])

    robot.drop_stop = True
    code, requests, output = invoke()
    assert code == 2 and requests[-1].header.identity.api_id == 1003, output
    assert 'API1003 has no matching reply' in output, output
    print('PASS walking-speed drive: fixed 0.30m/s 2s, no posture/mode writes, '
          'heading-projected and stationary odometry, matched Stop, SIGINT/SIGTERM, '
          'disabled-bridge guard, missing/stale odometry, rejected Move and lost Stop acknowledgement')
finally:
    executor.shutdown()
    thread.join(timeout=2)
    robot.destroy_node()
    rclpy.shutdown()
