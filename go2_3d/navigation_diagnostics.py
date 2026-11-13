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
                                        ('localization_state', 0.5), ('startup', self.window),
                                        ('operator', self.window))})


def no_motion_observation(history, now, ros_now, active_since):
    """Conservative evidence flag; stale data or an intentional stop is not one."""
    if not history.executing or active_since is None or now-active_since < 10.0:
        return None
    sample = history.snapshot(now, ros_now=ros_now)
    command = sample['commands']['cmd_vel']
    valid = sample['latest']['localization_valid']
    gate = sample['latest']['control']
    if (not sample['odometry']['available'] or not command['fresh'] or
        command['last'] is None or command['last'][0] <= 0.05 or
        not valid['fresh'] or valid['text'] != 'True' or not gate['fresh'] or
        'enable_control=True' not in (gate['text'] or '') or
        'gate=forwarding Move requests' not in (gate['text'] or '')):
        return None
    recent = [item for item in history.odometry if item[0] >= now-10.0]
    if len(recent) < 2 or recent[-1][0]-recent[0][0] < 9.0 or now-recent[-1][0] > 0.5:
        return None
    distance = math.hypot(recent[-1][1][0]-recent[0][1][0], recent[-1][1][1]-recent[0][1][1])
    yaw_travel = sum(abs(math.atan2(math.sin(b[1][2]-a[1][2]), math.cos(b[1][2]-a[1][2])))
                     for a, b in zip(recent, recent[1:]))
    commands = [v for received, v in history.commands['cmd_vel']
                if received >= now-10.0 and v is not None]
    if distance < 0.03 and yaw_travel < 0.1 and sum(v[0] > 0.05 for v in commands) >= 10:
        return 'Fresh forward commands and <3cm net odom displacement over about10s observed; physical cause unconfirmed'
    return None


def create_node(session_dir=None):
    import os
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data, qos_profile_action_status_default
    from rcl_interfaces.msg import Log
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from std_msgs.msg import Bool, Empty, String
    from action_msgs.msg import GoalStatusArray
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException
    from rclpy.duration import Duration
    from rclpy.time import Time
    from fault_recorder import AsyncEvidenceWriter, FlightRecorder, classify_log, json_safe, utc_text

    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec/1e9

    def pose_value(pose):
        p, q = pose.position, pose.orientation
        return json_safe(dict(position=[p.x, p.y, p.z], orientation=[q.x, q.y, q.z, q.w]))

    class NavigationDiagnostics(Node):
        PARAMETER_NAMES = {
            'controller_server': ('controller_frequency', 'failure_tolerance', 'FollowPath.plugin',
                'FollowPath.primary_controller', 'FollowPath.max_vel_x', 'FollowPath.max_vel_theta',
                'FollowPath.rotate_to_heading_angular_vel', 'FollowPath.closed_loop',
                'FollowPath.angular_dist_threshold', 'FollowPath.angular_disengage_threshold',
                'FollowPath.forward_sampling_distance', 'FollowPath.critics',
                'FollowPath.sim_time', 'FollowPath.vx_samples', 'FollowPath.vtheta_samples',
                'FollowPath.acc_lim_x', 'FollowPath.acc_lim_theta', 'FollowPath.max_angular_accel',
                'FollowPath.PathAlign.scale', 'FollowPath.PathDist.scale',
                'FollowPath.GoalAlign.scale', 'FollowPath.GoalDist.scale',
                'progress_checker.plugin', 'progress_checker.required_movement_radius',
                'progress_checker.required_movement_angle', 'progress_checker.movement_time_allowance'),
            'planner_server': ('GridBased.plugin', 'GridBased.tolerance', 'GridBased.allow_unknown',
                               'GridBased.use_astar'),
            'velocity_smoother': ('smoothing_frequency', 'feedback', 'max_velocity', 'min_velocity',
                'max_accel', 'max_decel', 'deadband_velocity', 'velocity_timeout'),
            'local_costmap/local_costmap': ('global_frame', 'robot_base_frame', 'footprint',
                'footprint_padding', 'resolution', 'width', 'height', 'rolling_window',
                'track_unknown_space', 'plugins', 'inflation_layer.inflation_radius',
                'inflation_layer.cost_scaling_factor'),
            'global_costmap/global_costmap': ('global_frame', 'robot_base_frame', 'footprint',
                'footprint_padding', 'resolution', 'track_unknown_space', 'plugins',
                'inflation_layer.inflation_radius', 'inflation_layer.cost_scaling_factor'),
            'bt_navigator': ('global_frame', 'robot_base_frame', 'default_nav_to_pose_bt_xml',
                             'default_server_timeout', 'bt_loop_duration'),
            'map_server': ('yaml_filename', 'frame_id')}

        def __init__(self):
            super().__init__('go2_navigation_diagnostics')
            self.history = ControlHistory()
            path = os.environ.get('GO2_DIAGNOSTICS_SESSION') if session_dir is None else session_dir
            self.recorder = FlightRecorder(AsyncEvidenceWriter(path) if path else None)
            self._last_storage_error = None
            self._last_heartbeat = 0.0
            self._active_goal = None
            self._active_since = None
            self._manual_until = -1.0
            self._heartbeat_latched = False
            self._still_latched = False
            self.full_path = None
            self.waypoints = None
            self.transforms = {}
            self.lifecycle = {}
            self.parameters = {}
            self.last_action = None
            self._action_states = {}
            self._requests = {}
            retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(String, '/navigation/control_diagnostics', retained)
            self.storage_publisher = self.create_publisher(String, '/navigation/recorder_status', retained)
            for name in self.history.commands:
                self.create_subscription(Twist, '/'+name, lambda message, name=name:
                    self.on_command(name, message), qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Bool, '/localization/valid', self.on_validity, qos_profile_sensor_data)
            self.create_subscription(String, '/localization/state', lambda msg:
                self.on_status('localization_state', msg.data), qos_profile_sensor_data)
            self.create_subscription(String, '/control/status', lambda msg:
                self.on_status('control', msg.data), retained)
            self.create_subscription(String, '/operator/status', self.on_operator, retained)
            self.create_subscription(String, '/navigation/startup_status', lambda msg:
                self.on_status('startup', msg.data), retained)
            self.create_subscription(Path, '/plan', self.on_plan, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid,
                                     qos_profile_sensor_data)
            self.create_subscription(Log, '/rosout', self.on_log, qos_profile_sensor_data)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     self.on_action, qos_profile_action_status_default)
            self.create_subscription(MarkerArray, '/navigation/waypoints', self.on_waypoints, retained)
            self.create_subscription(Bool, '/control/enable', lambda msg:
                self.on_manual('manual_enable' if msg.data else 'manual_stop', dict(enabled=msg.data)), 1)
            self.create_subscription(Empty, '/navigation/cancel', lambda msg:
                self.on_manual('manual_cancel', {}), 1)
            self.create_subscription(Empty, '/navigation/resume', lambda msg:
                self.on_manual('manual_resume', {}), 1)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda msg:
                self.on_manual('manual_initialpose', dict(frame=msg.header.frame_id,
                    pose=pose_value(msg.pose.pose), source_stamp=stamp(msg))), 1)
            self.create_subscription(PoseStamped, '/goal_pose', lambda msg:
                self.observe('goal_intent', dict(frame=msg.header.frame_id,
                    pose=pose_value(msg.pose)), stamp(msg)), 1)
            self.buffer = Buffer(cache_time=Duration(seconds=2.0))
            self.listener = TransformListener(self.buffer, self)
            self._state_clients = {name: self.create_client(GetState, '/'+name+'/get_state')
                for name in ('controller_server', 'planner_server', 'bt_navigator',
                             'behavior_server', 'velocity_smoother', 'map_server',
                             'local_costmap/local_costmap', 'global_costmap/global_costmap')}
            self._parameter_clients = {name: self.create_client(GetParameters, '/'+name+'/get_parameters')
                                       for name in self.PARAMETER_NAMES}
            self.create_timer(REPORT_SECONDS, self.periodic)
            self.create_timer(0.5, self.poll_tf)
            self.create_timer(1.0, self.maintenance)
            self.create_timer(5.0, self.poll_services)
            self.get_logger().info('Navigation evidence recorder: '+
                ('persistent session='+str(path) if path else 'console only (session path unset)'))

        def now_values(self):
            return time.monotonic(), self.get_clock().now().nanoseconds/1e9

        def observe(self, name, data, source_stamp=None):
            now, _ = self.now_values()
            return self.recorder.observe(name, data, now, source_stamp)

        def snapshot(self, reason='periodic'):
            now, ros_now = self.now_values()
            result = self.history.snapshot(now, reason, ros_now)
            result.update(captured_at_utc=utc_text(), received_monotonic=now,
                          ros_time=ros_now, recorder_status=self.recorder.status())
            return result

        def evidence_context(self):
            now, _ = self.now_values()
            grid = None
            if self.history.grid is not None:
                grid = {key: value for key, value in self.history.grid.items() if key != 'data'}
                if 'data' in self.history.grid:
                    # Copy at most 64KiB here. Compression and all writes run on
                    # the background writer, not the diagnostic callback.
                    grid['raw_int8'] = self.history.grid['data'].tobytes()
            return dict(captured_at_utc=utc_text(), received_monotonic=now,
                local_grid=grid, path=self.full_path, waypoints=self.waypoints,
                transforms=self.transforms.copy(), lifecycle=self.lifecycle.copy(),
                live_parameters=self.parameters.copy(), last_action=self.last_action,
                graph=self.graph_snapshot(), coverage=self.recorder.coverage_snapshot(now))

        def graph_snapshot(self):
            graph = {}
            for topic in ('/cmd_vel_nav', '/cmd_vel', '/odom', '/plan'):
                entry = {}
                for direction, method in (('publishers', self.get_publishers_info_by_topic),
                                          ('subscribers', self.get_subscriptions_info_by_topic)):
                    try:
                        endpoints = method(topic)
                        entry[direction] = [dict(node=item.node_name, namespace=item.node_namespace,
                            type=item.topic_type, diagnostic_observer=item.node_name == self.get_name(),
                            reliability=str(item.qos_profile.reliability),
                            durability=str(item.qos_profile.durability)) for item in endpoints[:32]]
                        entry[direction+'_capped'] = len(endpoints) > 32
                    except Exception as error:
                        entry[direction] = dict(unavailable=str(error)[:256])
                graph[topic] = entry
            return graph

        def trigger(self, code, message, source, source_stamp=None, severity='error'):
            now, _ = self.now_values()
            episode = self.recorder.episode
            refresh = (episode is None or now-episode['last_update'] >= 1.0 or
                       code in ('navigation_aborted', 'goal_rejected', 'action_failure'))
            snapshot = self.snapshot(code) if refresh else episode['latest_snapshot']
            context = self.evidence_context() if refresh else episode['latest_context']
            self.recorder.trigger(code, message, source, now, snapshot, context,
                                  ros_stamp=source_stamp, severity=severity)

        def on_command(self, name, message):
            now, _ = self.now_values()
            velocity = (message.linear.x, message.linear.y, message.angular.z)
            self.history.command(name, velocity, now)
            self.recorder.observe(name, dict(velocity=finite_values(velocity)), now)

        def on_odom(self, message):
            p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            now, _ = self.now_values()
            yaw = yaw_from_quaternion((q.x, q.y, q.z, q.w))
            self.history.odom(message.header.frame_id, p.x, p.y, yaw,
                             (v.linear.x, v.linear.y, v.angular.z), now, stamp(message))
            self.recorder.observe('odom', dict(frame=message.header.frame_id,
                child_frame=message.child_frame_id, pose=pose_value(message.pose.pose),
                velocity=finite_values((v.linear.x, v.linear.y, v.angular.z))), now, stamp(message))

        def on_validity(self, message):
            now, _ = self.now_values()
            self.history.status('localization_valid', str(message.data), now)
            self.recorder.observe('localization_valid', dict(valid=message.data), now)
            if message.data:
                self._heartbeat_latched = False
            elif self.history.executing and now > self._manual_until and not self._heartbeat_latched:
                self._heartbeat_latched = True
                self.trigger('localization_loss', 'Localization valid=False while route executing',
                             '/localization/valid')

        def on_status(self, name, value):
            now, _ = self.now_values()
            self.history.status(name, value, now)
            self.recorder.observe(name, dict(text=value[:MAX_STATUS_CHARS]), now)
            if name == 'startup' and ('failed' in value.lower() or 'failure' in value.lower()):
                self.trigger('startup_failed', value, '/navigation/startup_status')

        def on_operator(self, message):
            now, _ = self.now_values()
            failed = self.history.status('operator', message.data, now)
            self.recorder.observe('operator', dict(text=message.data[:MAX_STATUS_CHARS]), now)
            current = self.history.route.get('current', '-')
            if self.history.executing:
                if self._active_since is None:
                    self._active_since = now
                if current != self._active_goal:
                    self.recorder.reset_episode(now, 'new_waypoint')
                    self._active_goal = current
                    self._still_latched = False
            else:
                self._active_since = None
            if failed:
                code = 'navigation_aborted' if '(ABORTED)' in message.data or 'action status=6' in message.data else 'goal_rejected'
                self.trigger(code, message.data, '/operator/status')
                self.report('navigation_failure')
            elif 'Route paused: live localization lost' in message.data and now > self._manual_until:
                self.trigger('localization_loss', message.data, '/operator/status')
            elif '(SUCCEEDED)' in message.data:
                self.recorder.outcome('navigation_succeeded', message.data, now,
                                      self.snapshot('navigation_succeeded'), self.evidence_context())
                self.recorder.reset_episode(now, 'waypoint_succeeded')

        def on_manual(self, code, data):
            now, _ = self.now_values()
            self.recorder.observe('manual_intent', dict(code=code, **data), now)
            if code in ('manual_stop', 'manual_cancel', 'manual_initialpose'):
                self._manual_until = now+2.0
                self.recorder.trigger(code, code, 'manual_intent', now,
                    self.snapshot(code), self.evidence_context(), severity='info', manual=True)
            else:
                self.recorder.summary(dict(reason=code, event=data,
                    observed_at=utc_text(), snapshot=self.snapshot(code)))
            if code in ('manual_resume', 'manual_enable'):
                self.recorder.reset_episode(now, code)
                self._active_goal = None
                self._still_latched = False

        def on_plan(self, message):
            now, _ = self.now_values()
            p = message.poses[-1].pose.position if message.poses else None
            self.history.path(message.header.frame_id, len(message.poses),
                              (p.x, p.y) if p else None, now, stamp(message))
            self.full_path = dict(frame=message.header.frame_id, stamp=stamp(message),
                received_monotonic=now, total_points=len(message.poses),
                capped=len(message.poses) > 2048,
                poses=[pose_value(point.pose) for point in message.poses[:2048]])
            self.recorder.observe('plan', dict(frame=message.header.frame_id,
                points=len(message.poses), target=[p.x, p.y] if p else None), now, stamp(message))

        def on_waypoints(self, message):
            points_left = 2048
            markers = []
            for marker in message.markers[:256]:
                points = marker.points[:points_left]
                points_left -= len(points)
                markers.append(dict(id=marker.id, namespace=marker.ns, action=marker.action,
                    frame=marker.header.frame_id, stamp=stamp(marker), text=marker.text[:256],
                    pose=pose_value(marker.pose),
                    points=json_safe([[p.x, p.y, p.z] for p in points])))
            self.waypoints = dict(markers=markers, capped=len(message.markers) > 256 or points_left == 0,
                                  received_monotonic=time.monotonic())
            self.observe('waypoints', dict(markers=len(message.markers), capped=self.waypoints['capped']))

        def on_grid(self, message):
            p, q = message.info.origin.position, message.info.origin.orientation
            yaw = yaw_from_quaternion((q.x, q.y, q.z, q.w))
            now, _ = self.now_values()
            self.history.local_grid(message.header.frame_id, message.info.width, message.info.height,
                message.info.resolution, (p.x, p.y, yaw if yaw is not None else math.nan),
                message.data, now, stamp(message))
            self.recorder.observe('local_costmap', dict(frame=message.header.frame_id,
                width=message.info.width, height=message.info.height,
                resolution=message.info.resolution, retained='data' in self.history.grid,
                reason=self.history.grid.get('reason')), now, stamp(message))

        def on_log(self, message):
            if self.get_name() in message.name:
                return
            relevant = ('controller', 'planner', 'bt_navigator', 'go2_', 'velocity_smoother',
                        'behavior_server', 'costmap', 'transformPoseInTargetFrame', 'BehaviorTreeEngine',
                        'DWB', 'Critic', 'lifecycle_manager', 'PathAlignmentController')
            if not any(word in message.name for word in relevant):
                return
            now, _ = self.now_values()
            source_stamp = message.stamp.sec+message.stamp.nanosec/1e9
            text = message.msg[:2048]
            self.recorder.observe('rosout', dict(node=message.name[:128], level=message.level,
                text=text, file=message.file[-256:], line=message.line,
                function=message.function[:128]), now, source_stamp)
            code = classify_log(message.name, message.level, text)
            cleanup = any(word in text.lower() for word in
                          ('node halt', 'failed to cancel', 'failed to get result', 'goal failed'))
            if code and not (now < self._manual_until and cleanup and code == 'component_error'):
                self.trigger(code, text, message.name, source_stamp)

        def on_action(self, message):
            names = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                     4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            now, _ = self.now_values()
            for item in message.status_list[-128:]:
                identity = bytes(item.goal_info.goal_id.uuid).hex()
                state = int(item.status)
                if self._action_states.get(identity) == state:
                    continue
                self._action_states[identity] = state
                if len(self._action_states) > 128:
                    self._action_states.pop(next(iter(self._action_states)))
                source_stamp = item.goal_info.stamp.sec+item.goal_info.stamp.nanosec/1e9
                self.last_action = dict(goal_id=identity, status=state, name=names.get(state, str(state)),
                    goal_stamp=source_stamp, received_monotonic=now)
                self.recorder.observe('action_status', self.last_action, now, source_stamp)
                if state == 6:
                    if now < self._manual_until and self.recorder.episode is None:
                        self.recorder.summary(dict(reason='manual_cleanup_action_abort',
                            action=self.last_action, snapshot=self.snapshot('manual_cleanup')))
                    else:
                        self.trigger('navigation_aborted', 'NavigateToPose action ABORTED',
                                     '/navigate_to_pose/_action/status', source_stamp)
                elif state in (4, 5):
                    self.recorder.outcome('navigation_'+names[state].lower(), names[state], now,
                                          self.snapshot(names[state].lower()), self.evidence_context())

        def poll_tf(self):
            now, ros_now = self.now_values()
            transforms = {}
            for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('map', 'base_footprint')):
                key = parent+' -> '+child
                try:
                    tf = self.buffer.lookup_transform(parent, child, Time())
                    source_stamp = stamp(tf)
                    t, q = tf.transform.translation, tf.transform.rotation
                    transforms[key] = dict(available=True, stamp=source_stamp,
                        source_age_s=ros_now-source_stamp,
                        within_nav_age_envelope=(source_stamp > 0 and -0.1 <= ros_now-source_stamp <= 0.3),
                        translation=finite_values((t.x, t.y, t.z)),
                        rotation=finite_values((q.x, q.y, q.z, q.w)))
                except TransformException as error:
                    transforms[key] = dict(available=False, reason=str(error)[:256])
            self.transforms = dict(received_monotonic=now, edges=transforms)
            self.recorder.observe('tf', self.transforms, now)

        def poll_services(self):
            now, _ = self.now_values()
            for key, (future, started) in list(self._requests.items()):
                if now-started > 5.0:
                    self._requests.pop(key, None)
                    client = (self._state_clients if key[0] == 'lifecycle' else self._parameter_clients)[key[1]]
                    client.remove_pending_request(future)
            for name, client in self._state_clients.items():
                key = ('lifecycle', name)
                if key in self._requests:
                    continue
                if not client.service_is_ready():
                    self.lifecycle[name] = dict(available=False, reason='get_state service unavailable')
                    continue
                future = client.call_async(GetState.Request())
                self._requests[key] = (future, now)
                def completed(result, key=key, name=name):
                    self._requests.pop(key, None)
                    try:
                        state = result.result().current_state
                        self.lifecycle[name] = dict(available=True, id=state.id, label=state.label,
                                                     received_monotonic=time.monotonic())
                    except Exception as error:
                        self.lifecycle[name] = dict(available=False, reason=str(error)[:256])
                future.add_done_callback(completed)
            attributes = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                          5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
                          8: 'double_array_value', 9: 'string_array_value'}
            for name, client in self._parameter_clients.items():
                key = ('parameters', name)
                if key in self._requests:
                    continue
                if not client.service_is_ready():
                    self.parameters[name] = dict(available=False, reason='get_parameters service unavailable')
                    continue
                request = GetParameters.Request()
                request.names = list(self.PARAMETER_NAMES[name])
                future = client.call_async(request)
                self._requests[key] = (future, now)
                def completed(result, key=key, name=name):
                    self._requests.pop(key, None)
                    try:
                        values = {}
                        for parameter, value in zip(self.PARAMETER_NAMES[name], result.result().values):
                            raw = getattr(value, attributes[value.type]) if value.type in attributes else None
                            if value.type >= 5:
                                raw = list(raw)[:64]
                            values[parameter] = json_safe(raw)
                        self.parameters[name] = dict(available=True, values=values,
                                                      received_monotonic=time.monotonic())
                    except Exception as error:
                        self.parameters[name] = dict(available=False, reason=str(error)[:256])
                future.add_done_callback(completed)

        def maintenance(self):
            now, ros_now = self.now_values()
            if self.recorder.episode and not self.recorder.episode['post_complete'] and now >= self.recorder.episode['deadline']:
                self.recorder.tick(now, self.snapshot('post_trigger'), self.evidence_context())
            else:
                self.recorder.tick(now)
            if self.recorder.writer and now-self._last_heartbeat >= 4.0:
                self.recorder.writer.submit('heartbeat')
                self._last_heartbeat = now
            status = self.recorder.status()
            self.storage_publisher.publish(String(data=json.dumps(status, separators=(',', ':'), allow_nan=False)))
            if status.get('last_error') and status['last_error'] != self._last_storage_error:
                self.get_logger().warning('Navigation evidence persistence unavailable: '+status['last_error'])
            self._last_storage_error = status.get('last_error')
            if not self.history.executing or now < self._manual_until or self._active_since is None:
                return
            valid = self.history.statuses.get('localization_valid')
            if valid is not None and now-valid[0] > 0.5 and not self._heartbeat_latched:
                self._heartbeat_latched = True
                self.trigger('localization_heartbeat_stale', 'Localization validity heartbeat stale during route',
                             '/localization/valid')
            if now-self._active_since < 10.0 or self._still_latched:
                return
            observation = no_motion_observation(self.history, now, ros_now, self._active_since)
            if observation:
                self._still_latched = True
                self.trigger('no_motion_observed', observation,
                             'read_only_motion_observation', severity='warning')

        def periodic(self):
            if self.history.executing:
                self.report('periodic')

        def report(self, reason):
            result = self.snapshot(reason)
            text = json.dumps(result, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
            self.publisher.publish(String(data=text))
            self.get_logger().info('Navigation control diagnostics: '+text)
            self.recorder.summary(result)

        def destroy_node(self):
            flushed = self.recorder.close(time.monotonic(), self.snapshot('recorder_shutdown'), self.evidence_context())
            if not flushed:
                self.get_logger().error('Navigation recorder stopped with unsaved evidence: '+
                                        json.dumps(self.recorder.status(), separators=(',', ':')))
            return super().destroy_node()

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
