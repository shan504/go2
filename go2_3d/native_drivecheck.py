#!/usr/bin/env python3
"""Bounded movement through Unitree's enabled native obstacle-avoid interface."""
from collections import Counter
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response

from drivecheck import DriveRefused, ManualDrive, main


class NativeAvoidDrive(ManualDrive):
    DESCRIPTION = ('MANUAL native-avoid diagnostic, not navigation: vx=0.15m/s, '
                   'vy=0, yaw=0, 20Hz, 2s; avoidance stays enabled; '
                   'automatic zero Move, Sport StopMove and API-input release.')

    def __init__(self):
        super().__init__()
        self.avoid_requests = self.create_publisher(Request, '/api/obstacles_avoid/request', 10)
        self.native_sent, self.native_replied = {}, {}
        self.source_attempted = False
        self.source_released = False
        self.create_subscription(Response, '/api/obstacles_avoid/response',
                                 self.native_response, qos_profile_sensor_data)

    def native_response(self, message):
        identity = message.header.identity
        key = (identity.id, identity.api_id)
        if key in self.native_sent:
            self.native_replied[key] = (message.header.status.code, message.data)

    def native_request(self, api, parameter):
        request = Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id = api
        request.header.policy.noreply = False
        request.parameter = json.dumps(parameter)
        key = (request.header.identity.id, api)
        self.native_sent[key] = parameter
        self.avoid_requests.publish(request)
        return key

    def native_call(self, api, parameter, spin, timeout=2.0):
        key = self.native_request(api, parameter)
        deadline = time.monotonic()+timeout
        while key not in self.native_replied and time.monotonic()<deadline:
            spin(0.02)
        reply = self.native_replied.get(key)
        if reply is None:
            raise DriveRefused(f'Native-avoid API{api} has no matching reply')
        code, payload = reply
        if code != 0:
            raise DriveRefused(f'Native-avoid API{api} rejected with code={code}')
        return payload

    def prepare_drive(self, spin):
        if self.avoid_requests.get_subscription_count() == 0:
            raise DriveRefused('No matched subscriber on /api/obstacles_avoid/request')
        payload = self.native_call(1002, {}, spin)
        try:
            enabled = json.loads(payload).get('enable')
        except (ValueError, AttributeError):
            raise DriveRefused('Native-avoid SwitchGet returned invalid data')
        if enabled is not True:
            raise DriveRefused('Native avoidance is not enabled; this test does not enable or disable it')
        # Official SDK example selects API velocity input and releases it after
        # sending zero. Even a lost acknowledgement needs a release attempt.
        self.source_attempted = True
        self.native_call(1004, {'is_remote_commands_from_api': True}, spin)
        print('Native avoidance readback: enable=True; API velocity input selected (API1004 code=0).',
              flush=True)

    def publish_move(self):
        if self.avoid_requests.get_subscription_count() == 0:
            raise DriveRefused('Native-avoid subscriber disappeared')
        for key, (code, _) in self.native_replied.items():
            if key[1] == 1003 and code != 0 and self.native_sent[key].get('x') != 0:
                raise DriveRefused(f'Native-avoid Move rejected with code={code}')
        self.native_request(1003, {'x': 0.15, 'y': 0.0, 'yaw': 0.0, 'mode': 0})

    def stop_drive(self, spin):
        failures = []
        if self.source_attempted and rclpy.ok():
            try:
                self.native_call(1003, {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'mode': 0},
                                 spin, timeout=0.5)
                print('Native-avoid zero velocity acknowledged.', flush=True)
            except Exception as error:
                failures.append(str(error))
        try:
            super().stop_drive(spin)
        finally:
            if self.source_attempted and rclpy.ok():
                try:
                    self.native_call(1004, {'is_remote_commands_from_api': False},
                                     spin, timeout=1.0)
                    self.source_released = True
                    print('Native API velocity input released (API1004 code=0); avoidance remains enabled.',
                          flush=True)
                except Exception as error:
                    failures.append(str(error))
        if failures:
            raise DriveRefused('Native cleanup could not be confirmed: '+'; '.join(failures))

    def summary(self):
        lines = [super().summary()]
        for api in (1002, 1003, 1004):
            sent = sum(key[1] == api for key in self.native_sent)
            codes = Counter(reply[0] for key, reply in self.native_replied.items() if key[1] == api)
            lines.append(f'Own native-avoid API{api}: requests={sent} matched_response_codes={dict(codes)}')
        lines.append(f'Native API velocity source: selected_attempt={self.source_attempted} '
                     f'release_acknowledged={self.source_released}')
        return '\n'.join(lines)


if __name__ == '__main__':
    raise SystemExit(main(NativeAvoidDrive))
