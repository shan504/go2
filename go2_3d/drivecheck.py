#!/usr/bin/env python3
"""Explicit manual diagnostic: forward Sport Move at 0.15m/s for 2s, then stop."""
import signal
import time
import json
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from unitree_api.msg import Request, Response


class DriveRefused(RuntimeError):
    pass


class ManualDrive(Node):
    DESCRIPTION = 'MANUAL direct Sport diagnostic, not navigation: vx=0.15m/s, vy=0, yaw=0, 20Hz, 2s; automatic StopMove.'
    def __init__(self):
        super().__init__('go2_bounded_manual_drivecheck')
        self.guard = self.create_client(GetParameters,'/go2_edu_dds_bridge/get_parameters')
        self.requests = self.create_publisher(Request,'/api/sport/request',10)
        self.moves = 0
        self.cancel_check = lambda: False
        self.sent, self.replied = {}, {}
        self.create_subscription(Response, '/api/sport/response', self.response,
                                 qos_profile_sensor_data)

    def response(self, message):
        identity = message.header.identity
        key = (identity.id, identity.api_id)
        if key in self.sent:
            self.replied[key] = message.header.status.code

    def summary(self):
        lines = []
        for api in (1008, 1003):
            sent = sum(key[1] == api for key in self.sent)
            codes = Counter(code for key, code in self.replied.items() if key[1] == api)
            lines.append(f'Own Sport API{api}: requests={sent} matched_response_codes={dict(codes)}')
        lines.append('RPC code=0 confirms request handling, not physical movement. '
                     'Missing replies do not prove that the robot rejected a command.')
        return '\n'.join(lines)

    @staticmethod
    def validate_guard(response):
        if response is None:
            raise DriveRefused('Bridge parameter query returned no response')
        values = response.values
        if len(values)!=2 or any(value.type!=ParameterType.PARAMETER_BOOL for value in values):
            raise DriveRefused('Bridge must return boolean enable_control and allow_motion')
        if values[0].bool_value:
            raise DriveRefused('Bridge motion enabled; first disable navigation motion')
        if not values[1].bool_value:
            raise DriveRefused('Bridge is not in navigation mode (allow_motion must be true)')

    def query_guard(self):
        return self.guard.call_async(GetParameters.Request(names=['enable_control','allow_motion']))

    def publish_request(self,api_id,parameter='{}'):
        request = Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id = api_id
        request.header.policy.noreply = False
        request.parameter = parameter
        self.sent[(request.header.identity.id, api_id)] = time.monotonic()
        self.requests.publish(request)
        return (request.header.identity.id, api_id)

    def call_request(self, api_id, parameter, spin, timeout=2.0):
        key = self.publish_request(api_id, parameter)
        deadline = time.monotonic()+timeout
        while key not in self.replied and time.monotonic()<deadline:
            spin(0.02)
        code = self.replied.get(key)
        if code is None:
            raise DriveRefused(f'Sport API{api_id} has no matching reply')
        if code != 0:
            raise DriveRefused(f'Sport API{api_id} rejected with code={code}')

    def prepare_drive(self, spin):
        pass

    def publish_move(self):
        self.publish_request(1008,json.dumps({'x':0.15,'y':0.0,'z':0.0}))

    def stop_drive(self, spin):
        if self.moves and rclpy.ok():
            self.publish_request(1003)
            flush_end = time.monotonic()+0.2
            while time.monotonic()<flush_end:
                spin(0.02)
            print(f'StopMove API1003 sent; Move requests={self.moves}.',flush=True)

    def run(self,executor=None):
        def cleanup_spin(timeout):
            if executor is None:
                rclpy.spin_once(self,timeout_sec=timeout)
            else:
                executor.spin_once(timeout_sec=timeout)
        def spin(timeout):
            cleanup_spin(timeout)
            # Raise in Python after the ROS call returns. Raising directly from
            # a signal handler can interrupt pybind and hide KeyboardInterrupt
            # inside a conversion error, including during Stop flushing.
            if self.cancel_check():
                raise KeyboardInterrupt
        def wait(future):
            deadline = time.monotonic()+2.0
            while not future.done() and time.monotonic()<deadline:
                spin(0.02)
        if not self.guard.wait_for_service(timeout_sec=3.0):
            raise DriveRefused('Bridge GetParameters service unavailable')
        future = self.query_guard()
        wait(future)
        if not future.done():
            raise DriveRefused('Bridge parameter query timed out')
        self.validate_guard(future.result())
        # Allow discovery to finish before sending the first command.
        settle_end = time.monotonic()+0.3
        while time.monotonic()<settle_end:
            spin(0.02)
        if self.requests.get_subscription_count()==0:
            raise DriveRefused('No matched subscriber on /api/sport/request')
        future = self.query_guard()
        wait(future)
        if not future.done():
            raise DriveRefused('Bridge parameter query timed out')
        self.validate_guard(future.result())
        print('Bridge readback: enable_control=False, allow_motion=True; navigation bridge disabled.',flush=True)
        try:
            self.prepare_drive(spin)
            # Preparation may take time: read the bridge again before any Move.
            future = self.query_guard()
            wait(future)
            if not future.done():
                raise DriveRefused('Bridge parameter query timed out after preparation')
            self.validate_guard(future.result())
            print(self.DESCRIPTION,flush=True)
            started = time.monotonic()
            guard_at,next_guard,next_move = started,started+0.5,started
            pending,requested_at = None,None
            deadline = started+2.0
            while time.monotonic()<deadline:
                spin(0.01)
                now = time.monotonic()
                if now >= deadline:
                    break
                if pending is not None:
                    if pending.done():
                        self.validate_guard(pending.result())
                        pending,guard_at = None,now
                    elif now-requested_at>1.0:
                        raise DriveRefused('Bridge parameter query stopped responding')
                if now-guard_at>1.0:
                    raise DriveRefused('Bridge motion guard is stale')
                if pending is None and now>=next_guard:
                    pending,requested_at = self.query_guard(),now
                    next_guard = now+0.5
                if self.requests.get_subscription_count()==0:
                    raise DriveRefused('Sport request subscriber disappeared')
                if now>=next_move:
                    self.publish_move()
                    self.moves += 1
                    next_move = now+0.05
        finally:
            # Refused preflight never stops an existing navigation session.
            # Once this diagnostic sent Move, every exit sends StopMove.
            # Ignore the latched cancellation during bounded cleanup, so zero
            # velocity, Stop and input release can all finish.
            self.stop_drive(cleanup_spin)


def main(drive_type=ManualDrive):
    cancelled = False
    def terminate(signum,frame):
        nonlocal cancelled
        cancelled = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,terminate)
    signal.signal(signal.SIGTERM,terminate)
    node = drive_type()
    node.cancel_check = lambda: cancelled
    from rclpy.executors import SingleThreadedExecutor
    from robotcheck import RobotCheck
    native = RobotCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(native)
    try:
        node.run(executor)
        return 0
    except KeyboardInterrupt:
        print('Manual diagnostic interrupted.',flush=True)
        return 130
    except Exception as error:
        print(f'Manual diagnostic refused/stopped: {error}',flush=True)
        return 2
    finally:
        print(node.summary(),flush=True)
        print(native.summary(),flush=True)
        executor.shutdown()
        native.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
