#!/usr/bin/env python3
"""Reuse DDS adapter, adding a mandatory live-localization gate for motion."""
import sys
import time
import math
from collections import OrderedDict
sys.path.insert(0, '/opt/go2_project/patches/dds')
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from unitree_api.msg import Response
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from go2_edu_dds_bridge import Go2EduBridge
from motion_profile import LINEAR_SPEED, YAW_SPEED
import signal


class LocalizedBridge(Go2EduBridge):
    def __init__(self):
        self.localization_valid = False
        self.localization_received = None
        self.motion_allowed = False
        self.move_requests = self.nonzero_move_requests = self.stop_requests = 0
        self.last_move = None
        self.own_requests = OrderedDict()
        self.last_response = None
        # Keep counters and one sample, rather than an unbounded command log.
        self.cmd_received = self.cmd_nonzero = self.cmd_gate_accepted = 0
        self.cmd_dropped = dict.fromkeys(('motion_not_allowed','motion_disabled',
            'localization_invalid_or_stale','odometry_or_cloud_stale','nonfinite'),0)
        self.last_raw_cmd = None
        super().__init__()
        # Override only the 3D route's envelope. The standalone EDU patch keeps
        # its original limit; neither route amplifies a small command.
        self.declare_parameter('max_linear_speed',LINEAR_SPEED,ParameterDescriptor(read_only=True))
        self.declare_parameter('max_yaw_speed',YAW_SPEED,ParameterDescriptor(read_only=True))
        self.gate.max_linear = self.get_parameter('max_linear_speed').value
        self.gate.max_yaw = self.get_parameter('max_yaw_speed').value
        self.declare_parameter('allow_motion',False,ParameterDescriptor(read_only=True))
        self.motion_allowed = self.get_parameter('allow_motion').value
        self.create_subscription(Bool, '/localization/valid', self.on_validity, 1)
        self.create_subscription(Response, '/api/sport/response', self.on_response, qos_profile_sensor_data)
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.motion_status = self.create_publisher(String, '/control/status', retained)
        self.create_timer(0.5, self.publish_motion_status)
        self.get_logger().info(f'3D navigation limits: linear={self.gate.max_linear}m/s yaw={self.gate.max_yaw}rad/s')

    def on_validity(self, message):
        self.localization_valid = message.data
        self.localization_received = time.monotonic()

    def motion_inputs_fresh(self, now):
        return (self.motion_allowed and self.localization_valid and
                self.localization_received is not None and 0 <= now-self.localization_received < 0.5 and
                all(t is not None and 0 <= now-t < 1.0 for t in (self.odom_received,self.cloud_received)))

    def on_cmd(self, message):
        now = time.monotonic()
        command = (message.linear.x,message.linear.y,message.angular.z)
        self.cmd_received += 1
        self.last_raw_cmd = (now,command)
        finite = all(math.isfinite(value) for value in command)
        self.cmd_nonzero += int(finite and any(abs(value)>0.001 for value in command))
        if not finite:
            reason = 'nonfinite'
            # Match CommandGate's invalid-command behavior: do not keep an
            # earlier velocity alive after an invalid command arrives.
            self.gate.command = self.gate.received_at = None
        elif not self.motion_allowed:
            reason = 'motion_not_allowed'
        elif not self.gate.enabled:
            reason = 'motion_disabled'
        elif (not self.localization_valid or self.localization_received is None or
              not 0 <= now-self.localization_received < 0.5):
            reason = 'localization_invalid_or_stale'
        elif not all(t is not None and 0 <= now-t < 1.0
                     for t in (self.odom_received,self.cloud_received)):
            reason = 'odometry_or_cloud_stale'
        else:
            reason = None
        # Commands arriving during a localization/sensor outage are discarded,
        # so recovery requires a new velocity rather than replaying that backlog.
        if reason is None:
            self.cmd_gate_accepted += 1
            super().on_cmd(message)
        else:
            self.cmd_dropped[reason] += 1

    def remember_request(self, request_id, api_id):
        self.own_requests[request_id] = api_id
        while len(self.own_requests) > 256:
            self.own_requests.popitem(last=False)

    def send_move(self, command):
        request_id = super().send_move(command)
        self.remember_request(request_id,1008)
        self.move_requests += 1
        self.nonzero_move_requests += int(any(abs(v)>0.001 for v in command))
        self.last_move = (time.monotonic(),command)
        return request_id

    def send_stop(self):
        request_id = super().send_stop()
        self.remember_request(request_id,1003)
        self.stop_requests += 1
        return request_id

    def on_response(self, message):
        identity = message.header.identity
        # Other SDK clients can share /api/sport/response. Only report our IDs.
        if self.own_requests.get(identity.id) != identity.api_id:
            return
        self.own_requests.pop(identity.id)
        self.last_response = (time.monotonic(),identity.api_id,message.header.status.code)

    def publish_motion_status(self):
        now = time.monotonic()
        def age(received):
            return now-received if received is not None else float('inf')
        odom_age,cloud_age = age(self.odom_received),age(self.cloud_received)
        localization_age,command_age = age(self.localization_received),age(self.gate.received_at)
        if not self.motion_allowed:
            reason = 'navigation motion not allowed'
        elif not self.gate.enabled:
            reason = 'motion disabled'
        elif not self.localization_valid or not 0 <= localization_age < 0.5:
            reason = 'localization invalid or stale'
        elif not (0 <= odom_age < 1.0 and 0 <= cloud_age < 1.0):
            reason = 'odometry or cloud stale'
        elif self.gate.command is None or not 0 <= command_age <= self.gate.timeout:
            reason = 'waiting for fresh /cmd_vel'
        elif not any(abs(v)>0.001 for v in self.gate.command):
            reason = 'received zero /cmd_vel'
        else:
            reason = 'forwarding Move requests'
        command = list(self.gate.command) if self.gate.command is not None else None
        text = (f'gate={reason}; enable_control={self.gate.enabled}; '
                f'max_linear_speed={self.gate.max_linear:.2f} max_yaw_speed={self.gate.max_yaw:.2f}; '
                f'localization_age={localization_age:.2f}s odom_age={odom_age:.2f}s '
                f'cloud_age={cloud_age:.2f}s cmd_age={command_age:.2f}s cmd={command}; '
                f'Move_requests={self.move_requests} nonzero_Move_requests={self.nonzero_move_requests} '
                f'Stop_requests={self.stop_requests}')
        text += (f'; cmd_received={self.cmd_received} cmd_nonzero={self.cmd_nonzero} '
                 f'cmd_gate_accepted={self.cmd_gate_accepted} cmd_dropped={self.cmd_dropped}')
        if self.last_raw_cmd is not None:
            received,velocity = self.last_raw_cmd
            text += f'; last_raw_cmd_age={age(received):.2f}s last_raw_cmd={list(velocity)}'
        if self.last_move is not None:
            received,velocity = self.last_move
            text += f'; last_Move_age={age(received):.2f}s last_Move={list(velocity)}'
        if self.last_response is None:
            text += '; own_Sport_response=NONE (not proof of failure)'
        else:
            received,api_id,code = self.last_response
            text += f'; own_Sport_response api={api_id} code={code} age={age(received):.2f}s'
        self.motion_status.publish(String(data=text))

    def control_tick(self):
        now = time.monotonic()
        valid = self.motion_inputs_fresh(now)
        if not valid:
            action, _ = self.gate.poll(now, False)
            self.gate.command = self.gate.received_at = None
            if action == 'stop':
                self.send_stop()
            return
        super().control_tick()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    node = LocalizedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gate.set_enabled(False) and rclpy.ok():
            node.send_stop()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
