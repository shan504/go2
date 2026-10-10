"""Exercise native state inspection and read-only RPCs on isolated ROS endpoints."""
import copy
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
import robotcheck
from robotcheck import RobotCheck


ALLOWED = {'motion_switcher': 1001, 'robot_state': 1003, 'obstacles_avoid': 1002, 'sport': 1034}


class Harness:
    def __init__(self):
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, condition, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if condition():
                return
        raise AssertionError('Native robot inspection condition timed out')

    def close(self):
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()


class NativeEndpoints(Node):
    def __init__(self):
        super().__init__('simulated_native_state_endpoints')
        self.requests = {service: [] for service in ALLOWED}
        self.responses = {}
        for service in ALLOWED:
            self.responses[service] = self.create_publisher(Response, f'/api/{service}/response', 10)
            self.create_subscription(Request, f'/api/{service}/request',
                                     self.record_callback(service), 10)
        self.sport_requests = []
        self.create_subscription(Request, '/api/sport/request', self.sport_requests.append, 10)
        self.high = self.create_publisher(SportModeState, '/sportmodestate', qos_profile_sensor_data)
        self.low = self.create_publisher(SportModeState, '/lf/sportmodestate', qos_profile_sensor_data)
        self.high_state = SportModeState()
        self.high_state.mode, self.high_state.gait_type = 3, 1
        self.high_state.body_height = 0.28
        self.high_state.velocity = [0.12, -0.03, 0.0]
        self.high_state.yaw_speed = 0.25
        self.high_state.position = [1.0, 2.0, 0.3]
        self.high_state.range_obstacle = [0.6, 1.2, 2.3, 4.5]
        self.low_state = SportModeState()
        self.low_state.mode, self.low_state.gait_type, self.low_state.error_code = 254, 253, 17
        self.create_timer(0.05, self.publish_states)

    def record_callback(self, service):
        return lambda message: self.requests[service].append(copy.deepcopy(message))

    def publish_states(self):
        self.high.publish(self.high_state)
        self.low.publish(self.low_state)

    def reply(self, service, code, data, identity_offset=0, api_offset=0):
        response = Response()
        response.header.identity = copy.deepcopy(self.requests[service][0].header.identity)
        response.header.identity.id += identity_offset
        response.header.identity.api_id += api_offset
        response.header.status.code = code
        response.data = data
        self.responses[service].publish(response)


def passive_without_servers():
    harness = Harness()
    try:
        monitor = harness.add(RobotCheck())
        assert monitor.QUERIES == ALLOWED, 'Read-only query allowlist changed'
        harness.spin(0.6)
        monitor.send_queries()
        assert monitor.sent == {}, 'Query sent without a matched server'
        summary = monitor.summary()
        assert summary.count('NO MATCHED REQUEST SUBSCRIBER; query not sent') == 4
        assert '/sportmodestate: NO MESSAGES' in summary
        assert '/lf/sportmodestate: NO MESSAGES' in summary
        published = {endpoint.topic_name for endpoint in monitor.query_publishers.values()}
        assert published == {f'/api/{service}/request' for service in ALLOWED}
        assert monitor.QUERIES['sport'] == 1034, 'Sport must only query GetState'
        print('PASS native inspection: missing servers remain inconclusive and send no request')
    finally:
        harness.close()


def states_and_readonly_queries():
    harness = Harness()
    try:
        endpoints = harness.add(NativeEndpoints())
        monitor = harness.add(RobotCheck())
        harness.wait(lambda: all(endpoints.requests[service] for service in ALLOWED))
        harness.wait(lambda: all(topic in monitor.latest for topic in monitor.STATE_TOPICS))
        ids = set()
        for service, api in ALLOWED.items():
            request = endpoints.requests[service][0]
            assert request.header.identity.api_id == api
            expected = monitor.PARAMETERS.get(service, {})
            assert json.loads(request.parameter) == expected
            assert request.header.policy.noreply is False
            assert request.header.lease.id == 0
            assert request.header.identity.id > 0
            ids.add(request.header.identity.id)
        assert len(ids) == 4, 'Queries need separate response identities'

        # Existing traffic and mismatched API responses are not replies to us.
        endpoints.reply('motion_switcher', 0, '{"name":"unrelated"}', identity_offset=1)
        endpoints.reply('motion_switcher', 0, '{"name":"wrong-api"}', api_offset=1)
        harness.spin(0.1)
        assert monitor.replies == {}, 'Unmatched response was accepted'
        assert 'NO MATCHING REPLY (not proof of disabled service)' in monitor.summary()
        endpoints.reply('motion_switcher', 0, '{"name":"normal","form":0}')
        endpoints.reply('robot_state', 0, '[{"name":"sport_mode","status":1,"protect":true}]')
        endpoints.reply('obstacles_avoid', 9876, 'firmware-specific status')
        endpoints.reply('sport', 0, '{"state":0,"speedLevel":1}')
        harness.wait(lambda: len(monitor.replies) == 4)
        assert monitor.replies['motion_switcher'][1:] == (0, '{"name":"normal","form":0}')
        assert monitor.replies['obstacles_avoid'][1:] == (9876, 'firmware-specific status')
        summary = monitor.summary()
        assert "motion_switcher query api=1001: code=0 data={'name': 'normal', 'form': 0}" in summary
        assert 'robot_state query api=1003: code=0' in summary
        assert "obstacles_avoid query api=1002: code=9876 data='firmware-specific status'" in summary
        assert "sport query api=1034: code=0 data={'state': 0, 'speedLevel': 1}" in summary
        assert "'mode': 3, 'gait_type': 1, 'error_code': 0" in summary
        assert "'mode': 254, 'gait_type': 253, 'error_code': 17" in summary
        assert "'position': [1.0, 2.0" in summary
        assert 'observed_mode_gait_error=[(254, 253, 17)]' in summary
        assert abs(monitor.maximum_velocity['/sportmodestate'][0] - 0.12) < 1e-6

        endpoints.high_state.mode, endpoints.high_state.gait_type = 1, 0
        endpoints.high_state.velocity = [0.0, 0.0, 0.0]
        endpoints.high_state.position = [1.3, 2.4, 0.3]
        harness.wait(lambda: (1, 0, 0) in monitor.modes['/sportmodestate'])
        assert (3, 1, 0) in monitor.modes['/sportmodestate']
        assert abs(monitor.maximum_velocity['/sportmodestate'][0] - 0.12) < 1e-6
        assert abs(monitor.maximum_displacement['/sportmodestate']-0.5) < 1e-6
        for _ in range(3):
            monitor.send_queries()
        harness.spin(0.55)
        assert all(len(requests) == 1 for requests in endpoints.requests.values()), 'Query was retried'
        assert len(endpoints.sport_requests) == 1 and endpoints.sport_requests[0].header.identity.api_id == 1034, \
            'Inspection must only send GetState, never Move, Stop, or sport control requests'
        print('PASS native inspection: real state schemas, raw firmware values, query allowlist, ID/API correlation and no control writes')
    finally:
        harness.close()


def unavailable_state_interface():
    harness = Harness()
    original = robotcheck.get_message
    try:
        native = harness.add(Node('native_state_with_unavailable_type'))
        native.create_publisher(SportModeState, '/sportmodestate', qos_profile_sensor_data)
        monitor = harness.add(RobotCheck())

        def missing_interface(kind):
            raise ImportError('native interface package unavailable in test')

        robotcheck.get_message = missing_interface
        harness.wait(lambda: '/sportmodestate' in monitor.type_errors)
        assert 'native interface package unavailable in test' in monitor.summary()
        assert monitor.sent == {} and monitor.latest == {}
        # Once an interface is available, discovery can recover without restart.
        robotcheck.get_message = original
        harness.wait(lambda: '/sportmodestate' in monitor.observers)
        assert '/sportmodestate' not in monitor.type_errors
        print('PASS native inspection: unavailable dynamic state type is reported and discovery can recover')
    finally:
        robotcheck.get_message = original
        harness.close()


rclpy.init()
try:
    passive_without_servers()
    states_and_readonly_queries()
    unavailable_state_interface()
finally:
    rclpy.shutdown()
