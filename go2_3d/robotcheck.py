#!/usr/bin/env python3
"""Read native state and three official read-only RPCs; never change robot mode."""
import argparse
from collections import Counter
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from unitree_api.msg import Request, Response


class RobotCheck(Node):
    # Service AND API form the allowlist: 1003 on sport would mean StopMove,
    # whereas 1003 on robot_state is the read-only ServiceList query.
    QUERIES = {'motion_switcher': 1001, 'robot_state': 1003, 'obstacles_avoid': 1002}
    STATE_TOPICS = ('/sportmodestate', '/lf/sportmodestate')

    def __init__(self):
        super().__init__('go2_native_readonly_check')
        self.latest, self.counts, self.modes, self.maximum_velocity = {}, Counter(), {}, {}
        self.observers, self.type_errors = {}, {}
        self.sent, self.replies = {}, {}
        self.initial_xy, self.maximum_displacement = {}, {}
        self.query_publishers = {}
        for service in self.QUERIES:
            self.query_publishers[service] = self.create_publisher(Request, f'/api/{service}/request', 10)
            def callback_for(service):
                return lambda message: self.response(message, service)
            self.create_subscription(Response, f'/api/{service}/response',
                                     callback_for(service), qos_profile_sensor_data)
        self.create_timer(0.5, self.discover)
        self.create_timer(0.5, self.send_queries)

    def discover(self):
        topics = dict(self.get_topic_names_and_types())
        for topic in self.STATE_TOPICS:
            if topic in self.observers:
                continue
            types = topics.get(topic, [])
            kind = next((name for name in types if name.endswith('/SportModeState')), None)
            if kind is None:
                continue
            try:
                message_type = get_message(kind)
            except (ImportError, AttributeError, ValueError) as error:
                self.type_errors[topic] = f'{kind}: {error}'
                continue
            def callback_for(topic):
                return lambda message: self.state(message, topic)
            self.observers[topic] = self.create_subscription(message_type, topic,
                                                            callback_for(topic), qos_profile_sensor_data)
            self.type_errors.pop(topic, None)

    def state(self, message, topic):
        self.latest[topic] = (time.monotonic(), message)
        self.counts[topic] += 1
        self.modes.setdefault(topic, set()).add((message.mode, message.gait_type, message.error_code))
        maximum = self.maximum_velocity.setdefault(topic, [0.0, 0.0, 0.0])
        self.maximum_velocity[topic] = [max(old, abs(float(value)))
                                        for old, value in zip(maximum, message.velocity)]
        xy = [float(value) for value in message.position[:2]]
        if all(math.isfinite(value) for value in xy):
            initial = self.initial_xy.setdefault(topic, xy)
            self.maximum_displacement[topic] = max(self.maximum_displacement.get(topic, 0.0),
                                                   math.hypot(xy[0]-initial[0], xy[1]-initial[1]))

    def send_queries(self):
        for service, api in self.QUERIES.items():
            if service in self.sent or not self.query_publishers[service].get_subscription_count():
                continue
            request = Request()
            request.header.identity.id = time.time_ns()
            request.header.identity.api_id = api
            request.header.policy.noreply = False
            request.parameter = '{}'
            self.sent[service] = (request.header.identity.id, api, time.monotonic())
            self.query_publishers[service].publish(request)

    def response(self, message, service):
        identity = message.header.identity
        if service not in self.sent or self.sent[service][:2] != (identity.id, identity.api_id):
            return
        self.replies[service] = (time.monotonic(), message.header.status.code, message.data)

    def summary(self):
        now = time.monotonic()
        lines = ['READ-ONLY native robot check; no Move/Stop/StandUp, mode/avoidance/service writes.']
        for service, api in self.QUERIES.items():
            reply = self.replies.get(service)
            if reply is not None:
                received, code, payload = reply
                try:
                    data = json.loads(payload)
                except (ValueError, TypeError):
                    data = payload
                lines.append(f'{service} query api={api}: code={code} data={data!r} receipt_age={now-received:.2f}s')
            elif service in self.sent:
                lines.append(f'{service} query api={api}: NO MATCHING REPLY (not proof of disabled service)')
            else:
                lines.append(f'{service} query api={api}: NO MATCHED REQUEST SUBSCRIBER; query not sent')
        for topic in self.STATE_TOPICS:
            if topic not in self.latest:
                detail = self.type_errors.get(topic, f'publishers={self.count_publishers(topic)}')
                lines.append(f'{topic}: NO MESSAGES; {detail}')
                continue
            received, message = self.latest[topic]
            fields = {name: getattr(message, name) for name in
                      ('mode', 'gait_type', 'error_code', 'body_height', 'yaw_speed')}
            fields.update({name: list(getattr(message, name)) for name in
                           ('velocity', 'position', 'range_obstacle')})
            lines.append(f'{topic}: messages={self.counts[topic]} receipt_age={now-received:.2f}s {fields}')
            lines.append(f'  observed_mode_gait_error={sorted(self.modes[topic])} '
                         f'max_abs_native_velocity={self.maximum_velocity[topic]}')
            lines.append(f'  max_native_XY_displacement={self.maximum_displacement.get(topic, 0.0):.3f}m '
                         '(since first observed state)')
        lines.append('Native velocity is measured state; the command velocity is different. '
                     'Preserve raw mode/gait numbers for the installed Go2W firmware.')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seconds', type=int, nargs='?', default=20)
    args = parser.parse_args()
    if not 8 <= args.seconds <= 60:
        parser.error('duration must be 8..60 seconds')
    rclpy.init()
    node = RobotCheck()
    try:
        print(f'READ-ONLY native robot check: recording {args.seconds}s; three status queries only.', flush=True)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(node.summary(), flush=True)
    except KeyboardInterrupt:
        print(node.summary(), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
