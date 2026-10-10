#!/usr/bin/env python3
"""Explicit 0.30m/s, two-second Sport test; no posture or control-mode writes."""
import math
import time

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from drivecheck import DriveRefused, ManualDrive, main


class WalkingDrive(ManualDrive):
    # Match the official Go2 forward example. This is a test speed, not a
    # universal gait threshold and not a change to the navigation envelope.
    FORWARD_SPEED = 0.30
    DESCRIPTION = ('MANUAL walking-speed diagnostic, not navigation: '
                   'vx=0.30m/s, vy=0, yaw=0, 20Hz, 2s; automatic StopMove. '
                   'No BalanceStand, gait, joystick, mode or avoidance writes.')

    def __init__(self):
        super().__init__()
        self.pose = self.start_pose = self.end_pose = None
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)

    def odometry(self, message):
        position, rotation = message.pose.pose.position, message.pose.pose.orientation
        values = (position.x, position.y, rotation.x, rotation.y, rotation.z, rotation.w)
        if not all(math.isfinite(value) for value in values):
            return
        if message.header.frame_id.lstrip('/') != 'odom' or message.child_frame_id.lstrip('/') != 'base_link':
            return
        yaw = math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                         1-2*(rotation.y**2+rotation.z**2))
        self.pose = (time.monotonic(), position.x, position.y, yaw)

    def fresh_pose(self):
        if self.pose is None or time.monotonic()-self.pose[0] > 0.5:
            raise DriveRefused('Fresh /odom in odom -> base_link is required; no continued Move')
        return self.pose

    def prepare_drive(self, spin):
        deadline = time.monotonic()+2.0
        while self.pose is None and time.monotonic() < deadline:
            spin(0.02)
        self.fresh_pose()

    def publish_move(self):
        pose = self.fresh_pose()
        if self.start_pose is None:
            self.start_pose = pose
        for key, code in self.replied.items():
            if key[1] == 1008 and code != 0:
                raise DriveRefused(f'Sport Move rejected with code={code}')
        super().publish_move()

    def stop_drive(self, spin):
        if not self.moves:
            return
        self.end_pose = self.pose  # Sample before Stop, not during later body settling.
        self.call_request(1003, '{}', spin)
        print(f'StopMove API1003 code=0; Move requests={self.moves}.', flush=True)

    def summary(self):
        lines = [super().summary()]
        if self.start_pose is not None and self.end_pose is not None:
            started, x0, y0, yaw = self.start_pose
            ended, x1, y1, _ = self.end_pose
            dx, dy = x1-x0, y1-y0
            forward = dx*math.cos(yaw)+dy*math.sin(yaw)
            lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
            lines.append(f'/odom during Move: sample_span={ended-started:.2f}s '
                         f'net_XY={math.hypot(dx,dy):.3f}m '
                         f'forward={forward:.3f}m lateral={lateral:.3f}m')
        else:
            lines.append('/odom during Move: no motion interval recorded')
        lines.append('Nominal command: 0.30m/s for at most 2s (0.60m). '
                     'Odometry can include body sway; confirm actual footsteps and travel.')
        return '\n'.join(lines)


if __name__ == '__main__':
    raise SystemExit(main(WalkingDrive))
