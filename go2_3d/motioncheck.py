#!/usr/bin/env python3
"""Observe the velocity/Sport chain; never publish or change robot state."""
import argparse
from collections import Counter, OrderedDict, deque
import json
import math
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from unitree_api.msg import Request, Response


class VelocityStats:
    def __init__(self):
        self.samples = self.xy = self.rotation_only = self.zero = self.invalid = 0
        self.maximum = [0.0, 0.0, 0.0]
        self.last = self.first_at = self.last_at = None

    def receive(self, values, now):
        self.samples += 1
        self.last = list(values)
        self.first_at = now if self.first_at is None else self.first_at
        self.last_at = now
        if not all(math.isfinite(v) for v in values):
            self.invalid += 1
            return
        xy = any(abs(v) > 0.001 for v in values[:2])
        yaw = abs(values[2]) > 0.001
        self.xy += int(xy)
        self.rotation_only += int(yaw and not xy)
        self.zero += int(not xy and not yaw)
        self.maximum = [max(old, abs(v)) for old, v in zip(self.maximum, values)]


class MotionCheck(Node):
    PARAMETERS = {
        'velocity_smoother': ['feedback', 'smoothing_frequency', 'velocity_timeout',
            'max_velocity', 'min_velocity', 'max_accel', 'max_decel', 'deadband_velocity'],
        'controller_server': ['controller_frequency', 'min_x_velocity_threshold',
            'min_y_velocity_threshold', 'min_theta_velocity_threshold',
            'FollowPath.max_vel_x', 'FollowPath.max_vel_y', 'FollowPath.max_vel_theta',
            'FollowPath.min_speed_xy', 'FollowPath.max_speed_xy',
            'FollowPath.vx_samples', 'FollowPath.trajectory_generator_name',
            'progress_checker.required_movement_radius',
            'progress_checker.movement_time_allowance'],
        'go2_edu_dds_bridge': ['enable_control', 'allow_motion', 'max_linear_speed', 'max_yaw_speed'],
    }

    def __init__(self):
        super().__init__('go2_motion_readonly_check')
        self.started = None
        self.initial = {}
        self.stats, self.type_stats, self.subscriptions_by_type = {}, {}, {}
        self.latest, self.status_events = {}, deque(maxlen=16)
        self.control_baseline = None
        self.lifecycle, self.parameters, self.parameter_receipts, self.pending = {}, {}, {}, {}
        self.request_stats, self.request_ids = {}, OrderedDict()
        self.early_responses = OrderedDict()
        self.unmatched_responses = 0
        self.source_gids = {}
        self.request_publishers = set()
        self.first_odom = self.last_odom = None
        self.odom_samples, self.odom_max_distance = 0, 0.0
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                              for name in ('controller_server', 'velocity_smoother', 'bt_navigator')}
        self.parameter_clients = {name: self.create_client(GetParameters, f'/{name}/get_parameters')
                                  for name in self.PARAMETERS}
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def status_callback(topic):
            return lambda message: self.status(message, topic)
        for topic in ('/control/status', '/operator/status', '/navigation/startup_status'):
            self.create_subscription(String, topic, status_callback(topic), retained)
        self.create_subscription(Bool, '/localization/valid',
                                 lambda msg: self.status(msg, '/localization/valid'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        # Humble's executor discards MessageInfo and calls callback(msg) only.
        # Keep this observer outside every executor and drain take_message()
        # ourselves to preserve any metadata the installed RMW exposes. Older
        # Humble builds omit publisher_gid; request attribution then stays
        # explicitly inferred/unresolved, without changing ROS middleware.
        self.request_observer = Node('go2_motion_request_observer')
        self.request_subscription = self.request_observer.create_subscription(
            Request, '/api/sport/request', lambda message: None,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, self.drain_requests)
        self.create_subscription(Response, '/api/sport/response', self.response, qos_profile_sensor_data)
        self.create_timer(1.0, self.refresh)

    def snapshot(self):
        topics = {'/cmd_vel_nav', '/cmd_vel', '/api/sport/request', '/api/sport/response'}
        topics.update(name for name, _ in self.get_topic_names_and_types() if 'cmd_vel' in name)
        result = {}
        for topic in sorted(topics):
            endpoints = {}
            for label, getter in (('publishers', self.get_publishers_info_by_topic),
                                  ('subscribers', self.get_subscriptions_info_by_topic)):
                endpoints[label] = []
                for endpoint in getter(topic):
                    name = f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}'
                    if endpoint.node_name in (self.get_name(), 'go2_motion_request_observer'):
                        continue
                    endpoints[label].append(dict(node=name, type=endpoint.topic_type,
                        reliability=endpoint.qos_profile.reliability.name,
                        durability=endpoint.qos_profile.durability.name))
                    if label == 'publishers' and topic == '/api/sport/request':
                        self.source_gids[bytes(endpoint.endpoint_gid)] = name
            result[topic] = endpoints
        self.request_publishers = {v['node'] for v in result['/api/sport/request']['publishers']}
        return result

    def subscribe_velocities(self):
        kinds = {'geometry_msgs/msg/Twist': Twist, 'geometry_msgs/msg/TwistStamped': TwistStamped}
        advertised = dict(self.get_topic_names_and_types())
        for topic in sorted(set(advertised) | {'/cmd_vel_nav', '/cmd_vel'}):
            if 'cmd_vel' not in topic:
                continue
            # Topic discovery includes subscriber types too. Observe only an
            # offered publisher type: Fast DDS rejects two incompatible types
            # on one topic inside the same participant. Prefer the expected
            # Humble Twist type when both are offered, and keep one per topic.
            offered = {v.topic_type for v in self.get_publishers_info_by_topic(topic)}
            selected = next((name for name in kinds if name in offered), None)
            if any(path == topic for path, _ in self.subscriptions_by_type):
                continue
            for name in ([selected] if selected else []):
                if name not in kinds or (topic, name) in self.subscriptions_by_type:
                    continue
                self.stats.setdefault(topic, VelocityStats())
                self.type_stats[topic, name] = VelocityStats()
                def callback_for(topic, name):
                    def receive(msg):
                        velocity = msg.twist if isinstance(msg, TwistStamped) else msg
                        values = (velocity.linear.x, velocity.linear.y, velocity.angular.z)
                        now = time.monotonic()
                        self.stats[topic].receive(values, now)
                        self.type_stats[topic, name].receive(values, now)
                    return receive
                self.subscriptions_by_type[topic, name] = self.create_subscription(
                    kinds[name], topic, callback_for(topic, name), qos_profile_sensor_data)

    def arm(self):
        # ControllerServer only publishes if it has a subscription. Freeze graph
        # evidence first: this monitor must not be mistaken for the smoother.
        self.initial = self.snapshot()
        self.started = time.monotonic()
        self.control_baseline = self.latest.get('/control/status', (None, None))[1]
        self.subscribe_velocities()

    def refresh(self):
        self.snapshot()
        if self.started is not None:
            self.subscribe_velocities()
        for name, client in self.state_clients.items():
            key = ('state', name)
            if key in self.pending or not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self.pending[key] = future
            def done(future, key=key, name=name):
                self.pending.pop(key, None)
                try:
                    state = future.result().current_state
                    self.lifecycle[name] = (state.id, state.label)
                except Exception as error:
                    self.lifecycle[name] = ('ERROR', str(error))
            future.add_done_callback(done)
        for name, client in self.parameter_clients.items():
            key = ('parameters', name)
            if (time.monotonic()-self.parameter_receipts.get(name, 0) < 2.0 or
                    key in self.pending or not client.service_is_ready()):
                continue
            request = GetParameters.Request(names=self.PARAMETERS[name])
            future = client.call_async(request)
            self.pending[key] = future
            def done(future, key=key, name=name):
                self.pending.pop(key, None)
                try:
                    values = future.result().values
                    self.parameters[name] = dict(zip(self.PARAMETERS[name],
                                                     map(parameter_value_to_python, values)))
                except Exception as error:
                    self.parameters[name] = {'ERROR': str(error)}
                self.parameter_receipts[name] = time.monotonic()
            future.add_done_callback(done)

    def status(self, message, topic):
        now = time.monotonic()
        self.latest[topic] = (now, message.data)
        if self.started is None or topic == '/localization/valid':
            return
        if topic == '/control/status' and self.control_baseline is None:
            self.control_baseline = message.data
        value = message.data
        if topic == '/control/status':
            value = value.split(';', 1)[0]  # Ages/counters must not flood the timeline.
        previous = next((v for _, t, v in reversed(self.status_events) if t == topic), None)
        if value != previous:
            self.status_events.append((now-self.started, topic, value))

    def odom(self, message):
        if self.started is None:
            return
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.last_odom = (pose.position.x, pose.position.y, yaw)
        if self.first_odom is None:
            self.first_odom = self.last_odom
        self.odom_samples += 1
        distance = math.hypot(self.last_odom[0]-self.first_odom[0], self.last_odom[1]-self.first_odom[1])
        self.odom_max_distance = max(self.odom_max_distance, distance)

    def request(self, message, info):
        if self.started is None:
            return
        gid = info.get('publisher_gid', ()) if isinstance(info, dict) else info.publisher_gid
        source = self.source_gids.get(bytes(gid))
        attribution = 'publisher_gid'
        if source is None:
            # Humble metadata can omit publisher_gid. Graph inference is clearly
            # labelled and must never attribute a multi-publisher stream.
            attribution = 'single_advertised_publisher_inferred'
            if len(self.request_publishers) == 1:
                source = next(iter(self.request_publishers))
            else:
                source, attribution = 'UNATTRIBUTED', 'publisher_gid_unavailable'
        stats = self.request_stats.setdefault(source, dict(move=VelocityStats(), stop=0,
            invalid=0, responses=Counter(), attribution=Counter()))
        stats['attribution'][attribution] += 1
        identity = message.header.identity
        key = (identity.id, identity.api_id)
        self.request_ids[key] = source
        while len(self.request_ids) > 512:
            self.request_ids.popitem(last=False)
        if key in self.early_responses:
            stats['responses'][(identity.api_id, self.early_responses.pop(key))] += 1
        if identity.api_id == 1003:
            stats['stop'] += 1
        elif identity.api_id == 1008:
            try:
                payload = json.loads(message.parameter)
                values = [float(payload[axis]) for axis in ('x', 'y', 'z')]
                stats['move'].receive(values, time.monotonic())
            except (ValueError, TypeError, KeyError, OverflowError):
                stats['invalid'] += 1

    def response(self, message):
        if self.started is None:
            return
        identity = message.header.identity
        key = (identity.id, identity.api_id)
        source = self.request_ids.pop(key, None)
        code = message.header.status.code
        if source is not None:
            self.request_stats[source]['responses'][(identity.api_id, code)] += 1
        else:
            self.unmatched_responses += 1
            self.early_responses[key] = code
            while len(self.early_responses) > 512:
                self.early_responses.popitem(last=False)

    def drain_requests(self):
        with self.request_subscription.handle:
            for _ in range(256):
                sample = self.request_subscription.handle.take_message(Request, False)
                if sample is None:
                    break
                self.request(*sample)

    def destroy_node(self):
        self.request_observer.destroy_node()
        return super().destroy_node()

    @staticmethod
    def graph_lines(snapshot):
        lines = []
        for topic, endpoints in snapshot.items():
            lines.append(topic)
            for label, items in endpoints.items():
                if not items:
                    lines.append(f'  {label}: NONE')
                for item in items:
                    lines.append(f'  {label}: {item["node"]} {item["type"]} '
                                 f'{item["reliability"]}/{item["durability"]}')
        return lines

    @staticmethod
    def velocity_line(topic, stats, now):
        if stats is None:
            return f'{topic}: messages=0'
        rate = ((stats.samples-1)/(stats.last_at-stats.first_at)
                if stats.samples > 1 and stats.last_at > stats.first_at else 0.0)
        age = now-stats.last_at if stats.last_at is not None else float('inf')
        return (f'{topic}: messages={stats.samples} xy_nonzero={stats.xy} '
                f'rotation_only={stats.rotation_only} zero={stats.zero} invalid={stats.invalid} '
                f'rate={rate:.1f}Hz max_abs_vx_vy_yaw={[round(v,4) for v in stats.maximum]} '
                f'last={stats.last} last_age={age:.2f}s')

    def summary(self):
        now = time.monotonic()
        lines = [f'READ-ONLY velocity trace: capture={now-self.started:.1f}s',
                 'Initial endpoints BEFORE velocity monitor subscriptions:']
        lines.extend(self.graph_lines(self.initial))
        current = self.snapshot()
        # Show endpoint changes without presenting our subscription as the consumer.
        for topic, endpoints in current.items():
            endpoints['subscribers'] = [v for v in endpoints['subscribers']
                                       if v['node'] != '/go2_motion_readonly_check']
        if current != self.initial:
            lines.append('Current endpoints (velocity monitor excluded):')
            lines.extend(self.graph_lines(current))
        for name in self.state_clients:
            state = self.lifecycle.get(name)
            lines.append(f'/{name} lifecycle: {state[1]} [{state[0]}]' if state else
                         f'/{name} lifecycle: NO SERVICE RESPONSE')
        for name in self.PARAMETERS:
            age = now-self.parameter_receipts.get(name, float('-inf'))
            lines.append(f'/{name} actual parameters: {self.parameters.get(name, "NO SERVICE RESPONSE")} '
                         f'receipt_age={age:.2f}s')
        for topic in sorted(set(self.stats) | {'/cmd_vel_nav', '/cmd_vel'}):
            lines.append(self.velocity_line(topic, self.stats.get(topic), now))
            typed = [(kind, stats) for (path, kind), stats in self.type_stats.items() if path == topic]
            for kind, stats in typed:
                if len(typed) > 1 or kind != 'geometry_msgs/msg/Twist':
                    lines.append('  '+self.velocity_line(kind, stats, now))
            offered = {v['type'] for v in current.get(topic, {}).get('publishers', [])}
            monitored = {kind for path, kind in self.subscriptions_by_type if path == topic}
            if offered-monitored:
                lines.append(f'  offered types not monitored={sorted(offered-monitored)} '
                             '(one type per topic; endpoint mismatches remain visible)')
        for source, stats in sorted(self.request_stats.items()):
            lines.append(self.velocity_line(f'Sport Move1008 publisher={source}', stats['move'], now))
            codes = {f'api{api}:code{code}': count for (api, code), count in stats['responses'].items()}
            lines.append(f'  Stop1003={stats["stop"]} invalid_Move_JSON={stats["invalid"]} '
                         f'correlated_response_codes={codes} attribution={dict(stats["attribution"])}')
        lines.append('Humble may omit publisher_gid: single advertised publisher is inferred only; '
                     'multi-publisher Sport requests stay UNATTRIBUTED. Use bridge-owned /control/status counters.')
        if not self.request_stats:
            lines.append('Sport requests observed=0')
        lines.append('Published Move or response code=0 is not proof of physical movement; '
                     'missing responses alone do not prove failure.')
        lines.append(f'/odom: messages={self.odom_samples} max_xy_displacement={self.odom_max_distance:.4f}m '
                     f'first={self.first_odom} last={self.last_odom}')
        for topic, (received, value) in sorted(self.latest.items()):
            lines.append(f'{topic}: {value} receipt_age={now-received:.2f}s')
        final_status = self.latest.get('/control/status', (None, ''))[1]
        deltas = {}
        for counter in ('cmd_received', 'cmd_nonzero', 'cmd_gate_accepted',
                        'Move_requests', 'nonzero_Move_requests', 'Stop_requests'):
            before = re.search(r'\b'+counter+r'=(\d+)', self.control_baseline or '')
            after = re.search(r'\b'+counter+r'=(\d+)', final_status)
            if before and after:
                delta = int(after[1])-int(before[1])
                deltas[counter] = delta if delta >= 0 else 'counter reset during capture'
        lines.append(f'Bridge-owned counter changes during capture: {deltas or "baseline/fields unavailable"}')
        lines.append('Status transitions during capture:')
        lines.extend(f'  +{elapsed:.2f}s {topic}: {value}' for elapsed, topic, value in self.status_events)
        # This finding is intentionally based on the pre-monitor graph. The
        # observer can make ControllerServer publish by adding a subscription.
        smoother_inputs = [v for v in self.initial.get('/cmd_vel_nav', {}).get('subscribers', [])
                           if v['node'].split('/')[-1] == 'velocity_smoother' and
                           v['type'] == 'geometry_msgs/msg/Twist']
        if not smoother_inputs:
            lines.append('FINDING: no velocity_smoother Twist input on /cmd_vel_nav before monitoring; '
                         'monitor-only velocity messages do not prove this connection works.')
        smoother_outputs = [v for v in current.get('/cmd_vel', {}).get('publishers', [])
                            if v['node'].split('/')[-1] == 'velocity_smoother' and
                            v['type'] == 'geometry_msgs/msg/Twist']
        bridge_inputs = [v for v in current.get('/cmd_vel', {}).get('subscribers', [])
                         if v['node'].split('/')[-1] == 'go2_edu_dds_bridge' and
                         v['type'] == 'geometry_msgs/msg/Twist']
        if not smoother_outputs or not bridge_inputs:
            lines.append('FINDING: expected velocity_smoother Twist output -> '
                         'go2_edu_dds_bridge Twist input on /cmd_vel is incomplete.')
        for topic, endpoints in current.items():
            for pub in endpoints['publishers']:
                for sub in endpoints['subscribers']:
                    if pub['type'] != sub['type'] or (pub['reliability'] == 'BEST_EFFORT' and
                            sub['reliability'] == 'RELIABLE') or (pub['durability'] == 'VOLATILE' and
                            sub['durability'] == 'TRANSIENT_LOCAL'):
                        lines.append(f'FINDING: incompatible endpoints on {topic}: '
                                     f'{pub["node"]} -> {sub["node"]}')
        for name, state in self.lifecycle.items():
            if state[0] != 3:
                lines.append(f'FINDING: /{name} is not active.')
        nav = self.type_stats.get(('/cmd_vel_nav', 'geometry_msgs/msg/Twist'))
        final = self.type_stats.get(('/cmd_vel', 'geometry_msgs/msg/Twist'))
        if nav and nav.xy and (not final or not final.xy):
            lines.append('FINDING: controller XY velocity observed, but no final /cmd_vel XY velocity.')
        if final and final.rotation_only and not final.xy:
            lines.append('FINDING: /cmd_vel contains rotation only; XY progress checking can still time out.')
        if final and final.xy:
            own = self.request_stats.get('/go2_edu_dds_bridge')
            unassigned = self.request_stats.get('UNATTRIBUTED')
            if unassigned and unassigned['move'].xy:
                lines.append('FINDING: XY Sport requests observed but publisher ownership is unresolved; '
                             'use bridge-owned /control/status receive/drop/Move counters.')
            elif not own or not own['move'].xy:
                lines.append('FINDING: final XY velocity observed, but no bridge XY Move1008 request observed; '
                             'inspect control gate/status and bridge subscription.')
            elif self.odom_samples and self.odom_max_distance < 0.01:
                lines.append('FINDING: XY Move1008 requests observed with under 1cm odometry displacement; '
                             'inspect correlated Sport responses and robot control state next.')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seconds', nargs='?', type=int, default=30)
    args = parser.parse_args()
    if not 15 <= args.seconds <= 120:
        parser.error('capture duration must be 15..120 seconds')
    rclpy.init()
    node = MotionCheck()
    try:
        print('READ-ONLY velocity trace; no goal, parameter, mode or motion writes. Discovering endpoints...', flush=True)
        discovery_end = time.monotonic()+3.0
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.arm()
        print(f'READY: recording {args.seconds}s. Now select a NEW goal and enable motion in Foxglove.', flush=True)
        deadline = node.started+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(node.summary(), flush=True)
    except KeyboardInterrupt:
        if node.started is not None:
            print(node.summary(), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
