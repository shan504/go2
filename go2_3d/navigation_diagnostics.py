#!/usr/bin/env python3
"""Bounded, read-only command/odometry evidence for navigation stalls.

The pure history collector has no ROS dependency. The ROS adapter only subscribes
and emits diagnostic text; it never enables motion or calls a navigation service.
"""
from array import array
from collections import deque
import json
import math
import re
import time


WINDOW_SECONDS = 20.0
REPORT_SECONDS = 4.0
MAX_SAMPLES = 2048
MAX_GRID_CELLS = 65536
MAX_STATUS_CHARS = 2048


def finite_values(values):
    values = tuple(float(value) for value in values)
    return values if all(math.isfinite(value) for value in values) else None


def yaw_from_quaternion(quaternion):
    values = finite_values(quaternion)
    if values is None:
        return None
    norm = math.sqrt(sum(value*value for value in values))
    if norm < 1e-9:
        return None
    x, y, z, w = (value/norm for value in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def age(now, received):
    return None if received is None else max(0.0, now-received)


class ControlHistory:
    def __init__(self, window_seconds=WINDOW_SECONDS, max_samples=MAX_SAMPLES):
        if not math.isfinite(window_seconds) or window_seconds <= 0 or max_samples < 2:
            raise ValueError('positive window and at least two retained samples required')
        self.window = float(window_seconds)
        self.commands = {name: deque(maxlen=max_samples) for name in ('cmd_vel_nav', 'cmd_vel')}
        self.odometry = deque(maxlen=max_samples)
        self.latest_commands = {}
        self.latest_odom = None
        self.capped_until = {}
        self.statuses = {}
        self.plan = None
        self.grid = None
        self.executing = False
        self.route = {}
        self.failure_latched = False

    def _prune(self, samples, now):
        while samples and samples[0][0] < now-self.window:
            samples.popleft()

    def _append(self, name, samples, sample, now):
        self._prune(samples, now)
        if len(samples) == samples.maxlen:
            self.capped_until[name] = samples[0][0]+self.window
        samples.append(sample)

    def command(self, name, velocity, now):
        values = finite_values(velocity)
        if values is not None and len(values) != 3:
            raise ValueError('expected vx, vy, yaw velocity')
        self.latest_commands[name] = (now, values)
        self._append(name, self.commands[name], (now, values), now)

    def odom(self, frame, x, y, yaw, velocity, now, stamp=None):
        values = finite_values((x, y, yaw)) if yaw is not None else None
        previous_frame = self.latest_odom['frame'] if self.latest_odom else None
        if frame != previous_frame:
            self.odometry.clear()
        self.latest_odom = dict(received=now, frame=frame, pose=values,
                                velocity=finite_values(velocity), stamp=stamp)
        if frame == 'odom' and values is not None:
            self._append('odom', self.odometry, (now, values), now)

    def status(self, name, value, now):
        value = str(value)[:MAX_STATUS_CHARS]
        self.statuses[name] = (now, value)
        if name != 'operator':
            return False
        self.route = dict(re.findall(r'\b(current|queued|completed|paused|started)=([^;\s]+)', value))
        self.executing = (self.route.get('started') == 'True' and
                          self.route.get('current', '-') != '-')
        if self.executing:
            self.failure_latched = False
        failed = ('action status=6' in value or '(ABORTED)' in value or
                  any(text in value for text in ('Nav2 rejected goal', 'Nav2 result failed',
                      'Nav2 goal request failed', 'Queued goal rejected on start')))
        if failed and not self.failure_latched:
            self.failure_latched = True
            return True
        return False

    def path(self, frame, count, target, now, stamp=None):
        self.plan = dict(frame=frame, points=int(count), target=finite_values(target)
                         if target is not None else None, received=now, stamp=stamp)

    def local_grid(self, frame, width, height, resolution, origin, data, now, stamp=None):
        self.grid = dict(frame=frame, received=now, stamp=stamp, reason=None)
        cells = width*height
        if frame != 'odom':
            self.grid['reason'] = 'local costmap frame is not odom; probes unavailable'
        elif width <= 0 or height <= 0 or not 0 < cells <= MAX_GRID_CELLS:
            self.grid['reason'] = 'local costmap empty or exceeds bounded cell limit'
        elif (len(data) != cells or not math.isfinite(resolution) or resolution <= 0 or
              finite_values(origin) is None):
            self.grid['reason'] = 'invalid local costmap geometry'
        else:
            try:
                values = array('b', data)
                if any(value < -1 or value > 100 for value in values):
                    raise ValueError('invalid occupancy value')
            except (ValueError, OverflowError):
                self.grid['reason'] = 'invalid local costmap occupancy values'
            else:
                self.grid.update(width=width, height=height, resolution=resolution,
                                 origin=tuple(origin), data=values)

    def _command_summary(self, name, now):
        samples = self.commands[name]
        self._prune(samples, now)
        values = [sample[1] for sample in samples if sample[1] is not None]
        signs = [1 if value[0] > 0.001 else -1 for value in values if abs(value[0]) > 0.001]
        yaw_signs = [1 if value[2] > 0.001 else -1 for value in values if abs(value[2]) > 0.001]
        latest = self.latest_commands.get(name)
        received_age = age(now, latest[0]) if latest else None
        return dict(samples=len(samples), invalid_samples=len(samples)-len(values),
            history_capped=now < self.capped_until.get(name, -1),
            nonzero=sum(any(abs(component) > 0.001 for component in value) for value in values),
            forward=sum(value[0] > 0.001 for value in values),
            reverse=sum(value[0] < -0.001 for value in values),
            vx_sign_changes=sum(a != b for a, b in zip(signs, signs[1:])),
            yaw_sign_changes=sum(a != b for a, b in zip(yaw_signs, yaw_signs[1:])),
            mean_abs=[math.fsum(abs(value[index])/len(values) for value in values)
                      for index in range(3)] if values else None,
            min=[min(value[index] for value in values) for index in range(3)] if values else None,
            max=[max(value[index] for value in values) for index in range(3)] if values else None,
            last=latest[1] if latest else None, receipt_age_s=received_age,
            fresh=received_age is not None and received_age <= 0.5)

    def _odom_summary(self, now, ros_now):
        self._prune(self.odometry, now)
        latest = self.latest_odom
        received_age = age(now, latest['received']) if latest else None
        source_age = (ros_now-latest['stamp'] if latest and ros_now is not None and
                      latest['stamp'] is not None else None)
        reason = ('odom not received' if latest is None else
                  'odom frame is not odom' if latest['frame'] != 'odom' else
                  'nonfinite odom pose' if latest['pose'] is None else
                  'odom receipt stale' if received_age > 0.5 else
                  'odom source stamp stale/future' if source_age is not None and
                  not -0.1 <= source_age <= 0.5 else None)
        samples = list(self.odometry)
        poses = [sample[1] for sample in samples]
        delta = [math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
                 for a, b in zip(poses, poses[1:])]
        return dict(available=reason is None, reason=reason, samples=len(samples),
            history_capped=now < self.capped_until.get('odom', -1),
            receipt_age_s=received_age, source_age_s=source_age,
            sample_span_s=samples[-1][0]-samples[0][0] if len(samples) >= 2 else None,
            net_displacement_m=math.hypot(poses[-1][0]-poses[0][0], poses[-1][1]-poses[0][1])
                if len(poses) >= 2 else None,
            travelled_m=sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:]))
                if len(poses) >= 2 else None,
            yaw_change_rad=sum(delta) if delta else None,
            yaw_travel_rad=sum(abs(value) for value in delta) if delta else None,
            last_reported_velocity=latest['velocity'] if latest else None)

    def _grid_summary(self, now, ros_now, odom):
        grid = self.grid
        if grid is None:
            return dict(available=False, reason='local costmap not received')
        received_age = age(now, grid['received'])
        source_age = (ros_now-grid['stamp'] if ros_now is not None and
                      grid['stamp'] is not None else None)
        reason = (grid['reason'] or ('local costmap receipt stale' if received_age > 3.0 else
                  'local costmap source stamp stale/future' if source_age is not None and
                  not -0.1 <= source_age <= 3.0 else
                  'fresh odom unavailable for probes' if not odom['available'] else None))
        result = dict(available=reason is None, reason=reason, receipt_age_s=received_age,
                      source_age_s=source_age, frame=grid['frame'])
        if grid['reason'] is not None:
            return result
        data = grid['data']
        result.update(cells=len(data), unknown=sum(value == -1 for value in data),
                      occupied_or_inscribed=sum(value >= 99 for value in data))
        if reason is None:
            x, y, yaw = self.latest_odom['pose']
            ox, oy, origin_yaw = grid['origin']
            c, s = math.cos(origin_yaw), math.sin(origin_yaw)
            probes = []
            for distance in (0.0, 0.4, 0.8, 1.2):
                dx, dy = x+distance*math.cos(yaw)-ox, y+distance*math.sin(yaw)-oy
                col = math.floor((c*dx+s*dy)/grid['resolution'])
                row = math.floor((-s*dx+c*dy)/grid['resolution'])
                inside = 0 <= col < grid['width'] and 0 <= row < grid['height']
                probes.append(dict(distance_m=distance, cell=[col, row],
                                   value=int(data[row*grid['width']+col]) if inside else None))
            result['heading_probes'] = probes
        return result

    def snapshot(self, now, reason='periodic', ros_now=None):
        odom = self._odom_summary(now, ros_now)
        plan = dict(self.plan) if self.plan else None
        if plan is not None:
            plan['receipt_age_s'] = age(now, plan.pop('received'))
            plan['fresh'] = plan['receipt_age_s'] <= 3.0
        return dict(reason=reason, window_s=self.window, route=self.route.copy(),
            commands={name: self._command_summary(name, now) for name in self.commands},
            odometry=odom, local_costmap=self._grid_summary(now, ros_now, odom), plan=plan,
            latest={name: dict(available=name in self.statuses,
                    receipt_age_s=age(now, self.statuses[name][0]) if name in self.statuses else None,
                    fresh=(name in self.statuses and age(now, self.statuses[name][0]) <= limit),
                    text=self.statuses[name][1] if name in self.statuses else None)
                    for name, limit in (('control', 3.0), ('localization_valid', 0.5),
                                        ('operator', self.window))})


def create_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from std_msgs.msg import Bool, String

    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec/1e9

    class NavigationDiagnostics(Node):
        def __init__(self):
            super().__init__('go2_navigation_diagnostics')
            self.history = ControlHistory()
            retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(String, '/navigation/control_diagnostics', retained)
            for name in self.history.commands:
                self.create_subscription(Twist, '/'+name, lambda message, name=name:
                    self.history.command(name, (message.linear.x, message.linear.y, message.angular.z),
                                         time.monotonic()), qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Bool, '/localization/valid', lambda message:
                self.history.status('localization_valid', str(message.data), time.monotonic()),
                qos_profile_sensor_data)
            self.create_subscription(String, '/control/status', lambda message:
                self.history.status('control', message.data, time.monotonic()), retained)
            self.create_subscription(String, '/operator/status', self.on_operator, retained)
            self.create_subscription(Path, '/plan', self.on_plan, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid,
                                     qos_profile_sensor_data)
            self.create_timer(REPORT_SECONDS, self.periodic)

        def on_odom(self, message):
            p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            self.history.odom(message.header.frame_id, p.x, p.y,
                yaw_from_quaternion((q.x, q.y, q.z, q.w)), (v.linear.x, v.linear.y, v.angular.z),
                time.monotonic(), stamp(message))

        def on_operator(self, message):
            if self.history.status('operator', message.data, time.monotonic()):
                self.report('navigation_failure')

        def on_plan(self, message):
            p = message.poses[-1].pose.position if message.poses else None
            self.history.path(message.header.frame_id, len(message.poses),
                              (p.x, p.y) if p else None, time.monotonic(), stamp(message))

        def on_grid(self, message):
            p, q = message.info.origin.position, message.info.origin.orientation
            yaw = yaw_from_quaternion((q.x, q.y, q.z, q.w))
            self.history.local_grid(message.header.frame_id, message.info.width, message.info.height,
                message.info.resolution, (p.x, p.y, yaw if yaw is not None else math.nan),
                message.data, time.monotonic(), stamp(message))

        def periodic(self):
            if self.history.executing:
                self.report('periodic')

        def report(self, reason):
            result = self.history.snapshot(time.monotonic(), reason,
                                           self.get_clock().now().nanoseconds/1e9)
            text = json.dumps(result, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
            self.publisher.publish(String(data=text))
            self.get_logger().info('Navigation control diagnostics: '+text)

    return NavigationDiagnostics()


def main():
    import rclpy
    rclpy.init()
    node = create_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
