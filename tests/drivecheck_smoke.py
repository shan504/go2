"""Actual ROS mock endpoints only; no robot or external network connection."""
import json
import copy
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from unitree_api.msg import Request, Response


class MockBridge(Node):
    def __init__(self):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control',False)
        self.declare_parameter('allow_motion',True)
        self.requests = []
        self.responses = self.create_publisher(Response,'/api/sport/response',10)
        self.reenable = False
        self.create_subscription(Request,'/api/sport/request',self.receive,10)

    def receive(self,message):
        self.requests.append(message)
        response = Response()
        response.header.identity = copy.deepcopy(message.header.identity)
        response.header.status.code = 0
        self.responses.publish(response)
        # An unrelated client response must not be attributed to this test.
        response.header.identity.id += 1
        response.header.status.code = 9999
        self.responses.publish(response)
        if self.reenable and message.header.identity.api_id==1008:
            self.set_parameters([Parameter('enable_control',value=True)])


script = Path(__file__).resolve().parents[1]/'go2_3d/drivecheck.py'
rclpy.init()
bridge = MockBridge()
executor = SingleThreadedExecutor()
executor.add_node(bridge)
thread = threading.Thread(target=executor.spin,daemon=True)
thread.start()


def invoke(interrupt=None):
    before = len(bridge.requests)
    process = subprocess.Popen([sys.executable,str(script)],stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT,text=True)
    if interrupt is not None:
        deadline = time.monotonic()+6
        while time.monotonic()<deadline and process.poll() is None:
            if any(message.header.identity.api_id==1008 for message in bridge.requests[before:]):
                process.send_signal(interrupt)
                break
            time.sleep(0.02)
        else:
            process.kill()
            raise AssertionError('No Move observed before interruption')
    output,_ = process.communicate(timeout=8)
    time.sleep(0.05)
    observed = bridge.requests[before:]
    assert all(message.header.identity.api_id in (1003, 1008, 1034) for message in observed)
    # A read-only GetState may arrive during Stop flushing; check the last
    # control command, rather than treating a subsequent query as motion.
    controls = [message for message in observed if message.header.identity.api_id != 1034]
    return process.returncode,controls,output


try:
    code,requests,output = invoke()
    assert code==0,output
    moves = [message for message in requests if message.header.identity.api_id==1008]
    assert 30<=len(moves)<=41,(len(moves),output)
    assert requests[-1].header.identity.api_id==1003,'Bounded drive did not stop'
    assert all(json.loads(message.parameter)=={'x':0.15,'y':0.0,'z':0.0} for message in moves)
    assert all(message.header.policy.noreply is False and message.header.lease.id==0
               for message in requests)
    assert 'Own Sport API1003: requests=1 matched_response_codes={0: 1}' in output,output
    assert '9999' not in output,'An unrelated response was attributed to this diagnostic'
    span = (moves[-1].header.identity.id-moves[0].header.identity.id)/1e9
    assert 1.5<span<=2.1,span
    for interrupt in (signal.SIGINT,signal.SIGTERM):
        code,requests,output = invoke(interrupt)
        assert code==130,output
        assert any(message.header.identity.api_id==1008 for message in requests)
        assert requests[-1].header.identity.api_id==1003,'Interrupted drive did not stop'
    bridge.set_parameters([Parameter('enable_control',value=True)])
    code,requests,output = invoke()
    assert code==2 and requests==[] and 'first disable' in output,(requests,output)
    bridge.set_parameters([Parameter('enable_control',value=False),Parameter('allow_motion',value=False)])
    code,requests,output = invoke()
    assert code==2 and requests==[] and 'allow_motion' in output,(requests,output)
    bridge.set_parameters([Parameter('allow_motion',value=True)])
    bridge.reenable = True
    code,requests,output = invoke()
    assert code==2 and requests[-1].header.identity.api_id==1003,(requests,output)
    assert 1<=sum(message.header.identity.api_id==1008 for message in requests)<30
    executor.remove_node(bridge)
    bridge.destroy_node()
    code,requests,output = invoke()
    assert code==2 and requests==[] and 'service unavailable' in output,(requests,output)
    print('PASS bounded manual drive: fixed 0.15m/s 2s, automatic Stop, SIGINT/SIGTERM Stop, enabled/wrong-mode/missing-service refusal, live re-enable Stop')
finally:
    executor.shutdown()
    thread.join(timeout=2)
    if bridge in executor.get_nodes():
        bridge.destroy_node()
    rclpy.shutdown()
