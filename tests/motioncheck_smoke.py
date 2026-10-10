"""Exercise read-only velocity tracing on isolated ROS endpoints, no robot."""
import copy
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from motioncheck import MotionCheck


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

    def wait(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Velocity trace condition timed out')

    def close(self):
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()


def active(request, response):
    response.current_state.id, response.current_state.label = 3, 'active'
    return response


class Controller(Node):
    def __init__(self, kind=Twist, qos=10, context=None):
        super().__init__('controller_server', context=context)
        self.declare_parameter('controller_frequency', 10.0)
        self.declare_parameter('FollowPath.max_vel_x', 0.15)
        self.declare_parameter('FollowPath.max_speed_xy', 0.15)
        for name in MotionCheck.PARAMETERS['controller_server']:
            if not self.has_parameter(name):
                self.declare_parameter(name, 0.001)
        self.create_service(GetState, '/controller_server/get_state', active)
        self.velocity = self.create_publisher(kind, '/cmd_vel_nav', qos)
        self.sent = 0
        self.command = Twist()
        self.command.linear.x = 0.08
        self.kind = kind
        self.create_timer(0.05, self.publish)

    def publish(self):
        # Mirror Humble ControllerServer's observer-sensitive publication rule.
        if not self.velocity.get_subscription_count():
            return
        command = copy.deepcopy(self.command)
        if self.kind is TwistStamped:
            command = TwistStamped(twist=command)
        self.velocity.publish(command)
        self.sent += 1


def observer_effect():
    harness = Harness()
    try:
        controller = harness.add(Controller())
        monitor = harness.add(MotionCheck())
        harness.wait(lambda: monitor.count_publishers('/cmd_vel_nav') == 1)
        monitor.refresh()
        harness.wait(lambda: 'controller_server' in monitor.parameters)
        assert monitor.parameters['controller_server']['FollowPath.max_vel_x'] == 0.15
        harness.spin(0.15)
        assert controller.sent == 0, 'MotionCheck subscribed to velocities before arming'
        assert '/cmd_vel_nav' not in monitor.stats
        monitor.arm()
        assert monitor.initial['/cmd_vel_nav']['subscribers'] == []
        harness.wait(lambda: monitor.stats['/cmd_vel_nav'].xy >= 3)
        summary = monitor.summary()
        assert controller.sent > 0, 'Observer effect fixture did not trigger publication'
        assert 'no velocity_smoother Twist input on /cmd_vel_nav before monitoring' in summary
        assert 'monitor-only velocity messages do not prove this connection works' in summary
        assert 'expected velocity_smoother Twist output -> go2_edu_dds_bridge Twist input' in summary
        assert '/controller_server lifecycle: active [3]' in summary
        print('PASS velocity observer effect: monitor starts publication without hiding missing functional subscriber')
    finally:
        harness.close()


class Smoother(Node):
    def __init__(self, input_qos=10, context=None):
        super().__init__('velocity_smoother', context=context)
        self.declare_parameter('feedback', 'OPEN_LOOP')
        self.declare_parameter('deadband_velocity', [0.0, 0.0, 0.0])
        for name in MotionCheck.PARAMETERS['velocity_smoother']:
            if not self.has_parameter(name):
                self.declare_parameter(name, [0.15, 0.0, 0.3] if name in
                                       ('max_velocity', 'min_velocity', 'max_accel', 'max_decel') else 20.0)
        self.create_service(GetState, '/velocity_smoother/get_state', active)
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.received = 0
        self.create_subscription(Twist, '/cmd_vel_nav', self.command, input_qos)

    def command(self, message):
        self.received += 1
        command = copy.deepcopy(message)
        command.linear.x *= 0.5
        command.linear.y *= 0.5
        self.velocity.publish(command)


class Bridge(Node):
    def __init__(self):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control', True)
        self.declare_parameter('allow_motion', True)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_yaw_speed', 0.30)
        self.requests = self.create_publisher(Request, '/api/sport/request', 10)
        self.record_enabled = False
        self.next_id = 10000
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)

    def command(self, message):
        if not self.record_enabled:
            return
        self.next_id += 1
        request = Request()
        request.header.identity.id, request.header.identity.api_id = self.next_id, 1008
        request.parameter = json.dumps(dict(x=message.linear.x, y=message.linear.y,
                                            z=message.angular.z))
        self.requests.publish(request)


class RobotEndpoints(Node):
    def __init__(self):
        super().__init__('simulated_sport_endpoints')
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.responses = self.create_publisher(Response, '/api/sport/response', 10)
        self.control_status = self.create_publisher(String, '/control/status', retained)
        self.operator_status = self.create_publisher(String, '/operator/status', retained)
        self.localization = self.create_publisher(Bool, '/localization/valid', 10)
        self.odometry = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Request, '/api/sport/request', self.request, 10)
        self.create_timer(0.05, self.publish)

    def publish(self):
        self.control_status.publish(String(data='gate=forwarding Move requests; age=0.01s'))
        self.operator_status.publish(String(data='Nav2 accepted goal'))
        self.localization.publish(Bool(data=True))
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        self.odometry.publish(odom)

    def request(self, message):
        response = Response()
        response.header.identity = copy.deepcopy(message.header.identity)
        response.header.status.code = 501 if message.header.identity.id >= 1000000 else 0
        self.responses.publish(response)


def request_message(identity, api, payload):
    message = Request()
    message.header.identity.id, message.header.identity.api_id = identity, api
    message.parameter = payload
    return message


def working_pipeline():
    harness = Harness()
    try:
        controller = harness.add(Controller())
        smoother = harness.add(Smoother())
        bridge = harness.add(Bridge())
        endpoints = harness.add(RobotEndpoints())
        monitor = harness.add(MotionCheck())
        harness.wait(lambda: any(v.node_name == 'velocity_smoother'
                                for v in monitor.get_subscriptions_info_by_topic('/cmd_vel_nav')))
        harness.wait(lambda: monitor.count_publishers('/api/sport/request') == 1)
        monitor.refresh()
        harness.wait(lambda: 'velocity_smoother' in monitor.parameters)
        monitor.arm()
        bridge.record_enabled = True
        try:
            harness.wait(lambda: monitor.request_stats.get('/go2_edu_dds_bridge', {}).get('move')
                                and monitor.request_stats['/go2_edu_dds_bridge']['move'].xy >= 3)
        except AssertionError:
            print(monitor.summary())
            raise
        harness.wait(lambda: monitor.request_stats['/go2_edu_dds_bridge']['responses'][(1008, 0)] >= 3)
        bridge.requests.publish(request_message(99999, 1003, '{}'))
        harness.wait(lambda: monitor.request_stats['/go2_edu_dds_bridge']['responses'][(1003, 0)] == 1)
        assert monitor.request_stats['/go2_edu_dds_bridge']['stop'] == 1
        summary = monitor.summary()
        assert 'no velocity_smoother Twist input' not in summary
        assert 'expected velocity_smoother Twist output' not in summary
        assert 'incompatible endpoints' not in summary
        assert 'under 1cm odometry displacement' in summary

        # Humble's native take_message metadata lacks publisher GIDs. Once a
        # second publisher exists, identity correlation must not invent source
        # ownership or attribute another SDK client's response to the bridge.
        bridge.record_enabled = False
        harness.spin(0.15)
        foreign = harness.add(Node('foreign_sport_client'))
        foreign_requests = foreign.create_publisher(Request, '/api/sport/request', 10)
        harness.wait(lambda: monitor.count_publishers('/api/sport/request') == 2)
        monitor.refresh()
        foreign_requests.publish(request_message(1000000, 1008, '{"x":0.12,"y":0.0,"z":0.0}'))
        foreign_requests.publish(request_message(1000001, 1003, '{}'))
        foreign_requests.publish(request_message(1000002, 1008, 'invalid-json'))
        harness.wait(lambda: any(v['invalid'] == 1 for k, v in monitor.request_stats.items()
                                if k != '/go2_edu_dds_bridge'))
        unattributed = next(k for k in monitor.request_stats if k != '/go2_edu_dds_bridge')
        assert unattributed.startswith('UNATTRIBUTED'), unattributed
        harness.wait(lambda: monitor.request_stats[unattributed]['responses'][(1008, 501)] >= 2)
        assert monitor.request_stats[unattributed]['stop'] == 1
        assert monitor.request_stats[unattributed]['move'].xy == 1
        assert monitor.request_stats[unattributed]['responses'][(1003, 501)] == 1
        assert monitor.request_stats['/go2_edu_dds_bridge']['responses'][(1008, 501)] == 0
        assert monitor.stats['/cmd_vel_nav'].maximum[0] == 0.08
        assert monitor.stats['/cmd_vel'].maximum[0] == 0.04
        harness.wait(lambda: monitor.odom_samples >= 3 and '/control/status' in monitor.latest)
        summary = monitor.summary()
        assert 'api1008:code501' in summary
        assert monitor.latest['/control/status'][1].startswith('gate=forwarding Move requests')

        # A rotation-only capture must not be counted as forward movement.
        controller.command = Twist()
        controller.command.angular.z = 0.1
        harness.wait(lambda: monitor.stats['/cmd_vel'].rotation_only >= 3)
        controller.command = Twist()
        harness.wait(lambda: monitor.stats['/cmd_vel'].zero >= 3)
        controller.command.linear.x = float('nan')
        harness.wait(lambda: monitor.stats['/cmd_vel'].invalid >= 1)
        assert monitor.stats['/cmd_vel'].maximum[0] == 0.04
        assert monitor.stats['/cmd_vel'].xy > 0 and monitor.stats['/cmd_vel'].rotation_only > 0

        # Direct parameter services are reread, rather than caching initial enable.
        bridge.set_parameters([Parameter('enable_control', value=False)])
        harness.wait(lambda: monitor.parameters.get('go2_edu_dds_bridge', {}).get('enable_control') is False)
        print('PASS velocity trace: real services, complete pipeline, per-axis samples, Humble source attribution limits and API-specific responses')
    finally:
        harness.close()


def incompatible_pipeline(kind, qos):
    harness = Harness()
    contexts = []
    try:
        if kind is TwistStamped:
            # DDS type conflicts must occur across separate participants, as
            # they do between real ROS processes. One participant cannot own
            # both message types under the same DDS topic name.
            contexts = [Context(), Context()]
            for context in contexts:
                rclpy.init(context=context)
        controller = harness.add(Controller(kind=kind, qos=qos,
                                           context=contexts[0] if contexts else None))
        smoother = harness.add(Smoother(context=contexts[1] if contexts else None))
        bridge = harness.add(Bridge())
        monitor = harness.add(MotionCheck())
        harness.wait(lambda: monitor.count_publishers('/cmd_vel_nav') == 1)
        harness.wait(lambda: any(v.node_name == 'velocity_smoother'
                                for v in monitor.get_subscriptions_info_by_topic('/cmd_vel_nav')))
        harness.spin(0.15)
        assert smoother.received == 0
        monitor.arm()
        harness.wait(lambda: monitor.stats['/cmd_vel_nav'].xy >= 3)
        assert smoother.received == 0, 'Incompatible functional subscriber received a command'
        summary = monitor.summary()
        assert 'FINDING: incompatible endpoints on /cmd_vel_nav: /controller_server -> /velocity_smoother' in summary
        if kind is Twist:
            assert 'controller XY velocity observed, but no final /cmd_vel XY velocity' in summary
        assert not any(v['node'] == '/go2_motion_readonly_check'
                       for v in monitor.initial['/cmd_vel_nav']['subscribers'])
        if kind is TwistStamped:
            assert monitor.initial['/cmd_vel_nav']['publishers'][0]['type'] == 'geometry_msgs/msg/TwistStamped'
        else:
            assert monitor.initial['/cmd_vel_nav']['publishers'][0]['reliability'] == 'BEST_EFFORT'
        print('PASS velocity trace: incompatible ' + ('TwistStamped/Twist types' if kind is TwistStamped else 'BEST_EFFORT/RELIABLE QoS'))
    finally:
        harness.close()
        for context in contexts:
            rclpy.shutdown(context=context)


def rotation_only_pipeline():
    harness = Harness()
    try:
        controller = harness.add(Controller())
        controller.command = Twist()
        controller.command.angular.z = 0.1
        harness.add(Smoother())
        harness.add(Bridge())
        monitor = harness.add(MotionCheck())
        harness.wait(lambda: any(v.node_name == 'velocity_smoother'
                                for v in monitor.get_subscriptions_info_by_topic('/cmd_vel_nav')))
        monitor.arm()
        harness.wait(lambda: monitor.stats['/cmd_vel'].rotation_only >= 3)
        assert monitor.stats['/cmd_vel'].xy == 0
        assert 'FINDING: /cmd_vel contains rotation only; XY progress checking can still time out.' in monitor.summary()
        print('PASS velocity trace: yaw-only commands remain distinct from XY progress')
    finally:
        harness.close()


rclpy.init()
try:
    observer_effect()
    working_pipeline()
    incompatible_pipeline(TwistStamped, 10)
    incompatible_pipeline(Twist, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    rotation_only_pipeline()
finally:
    rclpy.shutdown()
