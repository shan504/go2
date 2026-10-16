#!/usr/bin/env python3
"""Odometry-seeded 3D scan-to-map GICP and saved-map localization (ROS2 Humble)."""
import threading
import time
from collections import deque
from pathlib import Path
from map_library import map_name, new_directory, publish
import numpy as np
import open3d as o3d
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry import (pose_matrix, quaternion_from_matrix, transform_points,
                      cloud_xyz, filter_points, rotation_angle, ObservedGrid, planar_pose, first_map_pose)
from registration import cloud, align, static_correction_is_consistent, blend_correction, odometry_is_stationary
from ros_cloud import xyz_message
from ground import estimate_ground,body_seed_position
from denoise import clean_directory
from navigation_grid import build_bundle
from map_start import start_pose_from_metadata
from localization_hold import OdometryBudget
from mapping_store import MappingStore

INITIAL_POSE_WAIT_SECONDS = 10.0


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return pose_matrix([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])


class ScanMatchUnavailable(ValueError):
    """Discard this scan without renewing the previous accepted match."""


def require_match(result, prefix=''):
    if not result.accepted:
        error = (ScanMatchUnavailable if result.reason in
                 ('too few points', 'overlap/residual rejected') else ValueError)
        raise error(f'{prefix}{result.reason}: fitness={result.fitness:.3f} rmse={result.rmse:.3f}')


class Mapper(Node):
    def __init__(self):
        super().__init__('go2_gicp')
        defaults = dict(mode='mapping', map_directory='/maps/latest', output_directory='/maps', map_name='default',
                        voxel_size=0.15, map_voxel_size=0.06, keyframe_max_interval=2.0,
                        scan_window=0.0, map_publish_period=2.0,
                        grid_resolution=0.10, floor_z=-0.30,
                        obstacle_min_height=0.10, obstacle_max_height=1.5,
                        max_map_points=200000, submap_radius=25.0,
                        registration_period=0.5, validity_timeout=1.0,
                        correspondence_distance=0.7, min_fitness=0.55, max_rmse=0.20,
                        max_translation=0.6, max_rotation=0.35,
                        localization_voxel_size=0.10, localization_correspondence=0.70,
                        localization_max_translation=1.0, localization_max_rotation_degrees=45.0,
                        localization_min_fitness=0.20, localization_max_rmse=0.20,
                        stationary_translation_limit=0.35, stationary_rotation_limit_degrees=12.0,
                        localization_hold_timeout=3.0, localization_hold_distance=1.0,
                        localization_hold_rotation_degrees=60.0, localization_odom_timeout=0.3,
                        correction_alpha=0.25,auto_initialize=True,initial_from_map_start=True,
                        initial_x=0.0,initial_y=0.0,initial_z=0.0,initial_yaw_degrees=0.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if not 0 < self.p['correction_alpha'] <= 1:
            raise ValueError('correction_alpha must be greater than 0 and at most 1')
        if self.p['mode'] not in ('mapping','localization'):
            raise ValueError('mode must be mapping or localization')
        for name in ('stationary_translation_limit', 'stationary_rotation_limit_degrees',
                     'localization_correspondence', 'localization_max_translation',
                     'localization_max_rotation_degrees', 'validity_timeout',
                     'localization_hold_timeout', 'localization_hold_distance',
                     'localization_hold_rotation_degrees', 'localization_odom_timeout'):
            if not np.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        map_name(self.p['map_name'])
        self.lock = threading.RLock()
        self.generation = 0
        self.latest = None
        self.history = deque(maxlen=32)
        self.pending_clouds = deque(maxlen=32)
        self.last_received_cloud_stamp = None
        self.cloud_status = 'waiting for first cloud'
        self.cloud_counts = dict.fromkeys(('received', 'aligned', 'tf_waits', 'expired',
                                           'invalid_tf', 'age_rejected', 'duplicate', 'queue_dropped'), 0)
        self.first_received = None
        self.processed_stamp = None
        self.correction = np.eye(4)
        self.initialized = self.p['mode'] == 'mapping'
        self.seeded = False
        self.last_accepted = None
        self.accepted_odom = None
        self.accepted_match_correction = None
        self.odom_budget = OdometryBudget()
        self.localization_state = ''
        self.localization_detail = ''
        self.hold_exhausted = ''
        self.last_tf_stamp = None
        self.initial_hint_used = False
        self.pending_initial_pose = None
        self.waiting_status_time = 0.0
        self.last_failure = ''
        self.last_keyframe = None
        self.last_keyframe_time = None
        self.mapping_start_pose = None
        self.mapping_store = (MappingStore(self.p['map_voxel_size'], self.p['voxel_size'])
                              if self.p['mode'] == 'mapping' else None)
        self.capacity_warned = False
        self._points = np.empty((0,3))
        self._registration_points = np.empty((0,3))
        self.localization_points = np.empty((0,3))
        self.grid = ObservedGrid(self.p['grid_resolution'])
        self.map_metadata = {}
        self.saved_start_pose = None
        if self.p['mode'] == 'localization':
            directory = Path(self.p['map_directory'])
            self.map_metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
            if self.p['auto_initialize'] and self.p['initial_from_map_start']:
                self.saved_start_pose = start_pose_from_metadata(self.map_metadata)
            self._points = np.asarray(o3d.io.read_point_cloud(str(directory/'map.pcd')).points).copy()
            if len(self.points) < 100:
                raise ValueError('Missing/empty 3D map.pcd; localization cannot start')
            self._registration_points = np.asarray(cloud(self.points,self.p['voxel_size']).points).copy()
            self.localization_points = np.asarray(cloud(self.points,self.p['localization_voxel_size']).points).copy()
            self.get_logger().info('Loaded 3D map; automatic initialization from saved mapping start'
                                   if self.p['auto_initialize'] and self.p['initial_from_map_start'] else
                                   'Loaded 3D map; waiting for configured or manual /initialpose')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, '/map_cloud', retained)
        self.registered_pub = self.create_publisher(PointCloud2, '/registered_cloud', qos_profile_sensor_data)
        self.valid_pub = self.create_publisher(Bool, '/localization/valid', 1)
        self.status_pub = self.create_publisher(String, '/localization/status', 1)
        self.state_pub = self.create_publisher(String, '/localization/state', 1)
        self.create_subscription(PointCloud2, '/point_cloud2', self.receive_cloud, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose, 1)
        self.create_service(Trigger, '/save_3d_map', self.save_map)
        self.create_timer(0.1, self.health)
        # Cloud and TF arrive on independent DDS topics. Retry asynchronously
        # at the original scan timestamp instead of discarding fresh packets.
        self.create_timer(0.05, self.process_pending_clouds)
        # Nav2 needs transforms between GICP results, not only at the older
        # scan stamp after a registration finishes. Hold only a fresh accepted
        # map correction, timestamped by actual odometry (never future-dated).
        self.create_timer(0.05, self.publish_correction)
        self.create_timer(self.p['map_publish_period'], self.publish_map)
        self.registration_timer = self.create_timer(self.p['registration_period'], self.process,
                                                    callback_group=MutuallyExclusiveCallbackGroup())

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    @property
    def points(self):
        """Complete immutable snapshot for publishing/saving, never a live crop."""
        return self.mapping_store.snapshot() if self.mapping_store is not None else self._points

    @property
    def registration_points(self):
        return (self.mapping_store.registration_snapshot() if self.mapping_store is not None
                else self._registration_points)

    @property
    def map_point_count(self):
        return self.mapping_store.count if self.mapping_store is not None else len(self._points)

    def cloud_diagnostics(self):
        with self.lock:
            return ('clouds_'+' '.join(f'{name}={count}' for name,count in self.cloud_counts.items())+
                    f' pending={len(self.pending_clouds)}')

    def receive_cloud(self, message):
        received = time.monotonic()
        self.cloud_counts['received'] += 1
        if message.header.frame_id != 'base_link':
            self.get_logger().error('Expected cloud already transformed to base_link', throttle_duration_sec=5)
            return
        stamp = Time.from_msg(message.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if not -0.1 <= age <= 1.0:
            self.cloud_counts['age_rejected'] += 1
            self.cloud_status = f'Dropped cloud: timestamp age={age:.3f}s outside [-0.1, 1.0]s'
            self.get_logger().warning(self.cloud_status, throttle_duration_sec=5)
            return
        with self.lock:
            if (self.last_received_cloud_stamp is not None and
                    stamp.nanoseconds <= self.last_received_cloud_stamp):
                self.cloud_counts['duplicate'] += 1
                return
            self.last_received_cloud_stamp = stamp.nanoseconds
            if len(self.pending_clouds) == self.pending_clouds.maxlen:
                self.cloud_counts['queue_dropped'] += 1
                self.cloud_status = 'Dropped oldest cloud: pending odometry TF queue full'
                self.get_logger().warning(self.cloud_status, throttle_duration_sec=5)
            self.pending_clouds.append((message,received))
        self.process_pending_clouds()

    def process_pending_clouds(self):
        with self.lock:
            # A missing older transform must not block a newer ready packet.
            for _ in range(len(self.pending_clouds)):
                message,received = self.pending_clouds.popleft()
                stamp = Time.from_msg(message.header.stamp)
                age = (self.get_clock().now()-stamp).nanoseconds/1e9
                receipt_age = time.monotonic()-received
                if not -0.1 <= age <= 1.0 or not 0 <= receipt_age <= 1.0:
                    self.cloud_counts['expired'] += 1
                    self.cloud_status = (f'Dropped cloud waiting for odometry TF: '
                                         f'timestamp_age={age:.3f}s receipt_age={receipt_age:.3f}s')
                    self.get_logger().warning(self.cloud_status, throttle_duration_sec=5)
                    continue
                if self.latest is not None and stamp <= Time.from_msg(self.latest[0].header.stamp):
                    continue
                try:
                    odom = self.buffer.lookup_transform('odom','base_link',stamp)
                except TransformException as error:
                    self.cloud_counts['tf_waits'] += 1
                    self.pending_clouds.append((message,received))
                    self.cloud_status = (f'Waiting for odom -> base_link at cloud timestamp '
                                         f'{stamp.nanoseconds/1e9:.9f}; retrying fresh packet: {error}')
                    self.get_logger().warning(self.cloud_status, throttle_duration_sec=5)
                    continue
                try:
                    matrix = transform_matrix(odom.transform)
                except ValueError as error:
                    self.cloud_counts['invalid_tf'] += 1
                    self.cloud_status = f'Dropped cloud: invalid odom -> base_link transform: {error}'
                    self.get_logger().error(self.cloud_status, throttle_duration_sec=5)
                    continue
                self.latest = (message,matrix,received)
                if self.first_received is None:
                    self.first_received = received
                self.history.append(self.latest)
                self.cloud_counts['aligned'] += 1
                self.cloud_status = 'cloud aligned with timestamped odometry'
        self.apply_pending_initial_pose()
        with self.lock:
            if (self.latest is not None and self.p['mode'] == 'localization' and self.p['auto_initialize'] and
                    not self.initial_hint_used):
                hint = PoseWithCovarianceStamped()
                hint.header.frame_id = 'map'
                p,q = hint.pose.pose.position,hint.pose.pose.orientation
                if self.p['initial_from_map_start']:
                    start = self.saved_start_pose
                    p.x,p.y,p.z = map(float, start[:3,3])
                    q.x,q.y,q.z,q.w = map(float, quaternion_from_matrix(start))
                    self.seed_initial_pose(hint, exact_body_pose=True)
                else:
                    p.x,p.y,p.z = self.p['initial_x'],self.p['initial_y'],self.p['initial_z']
                    angle = np.deg2rad(self.p['initial_yaw_degrees'])/2
                    q.z,q.w = float(np.sin(angle)),float(np.cos(angle))
                    self.initial_pose(hint)

    def initial_pose(self, message):
        self.seed_initial_pose(message)

    def seed_initial_pose(self, message, exact_body_pose=False):
        if self.p['mode'] != 'localization' or message.header.frame_id != 'map':
            self.get_logger().warning('/initialpose is supported only in localization, frame map')
            return
        with self.lock:
            pose = message.pose.pose
            try:
                position = [pose.position.x,pose.position.y,pose.position.z]
                if not exact_body_pose:
                    position = body_seed_position(position, self.map_metadata.get('ground_model'),
                                                  self.map_metadata.get('body_height_above_ground'))
                map_base = pose_matrix(position,
                                       [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            except ValueError as error:
                self.get_logger().error(str(error))
                return
            # A browser hint can arrive before the first timestamp-aligned
            # sensor packet. Keep the newest valid user hint, never replace
            # it with the CLI fallback or continue publishing an old match.
            self.pending_initial_pose = (map_base, time.monotonic())
            self.initial_hint_used = True
            self.initialized, self.seeded = False, False
            self.last_accepted = None
            self.accepted_odom = None
            self.accepted_match_correction = None
            self.odom_budget.reset(None, None)
            self.hold_exhausted = ''
            self.processed_stamp = None
            self.last_failure = ''
            self.generation += 1
        if not self.apply_pending_initial_pose():
            self.report(f'Initial-pose hint queued for up to {INITIAL_POSE_WAIT_SECONDS:.0f}s; '
                        'waiting for fresh cloud and timestamp-aligned odometry; keep robot stationary')

    def apply_pending_initial_pose(self):
        with self.lock:
            if self.pending_initial_pose is None:
                return False
            map_base, received = self.pending_initial_pose
            now = time.monotonic()
            if now-received > INITIAL_POSE_WAIT_SECONDS:
                self.pending_initial_pose = None
                self.last_failure = 'Initial-pose hint expired before fresh cloud/odometry; publish actual /initialpose again'
                text = self.last_failure
            elif self.latest is None or not 0 <= now-self.latest[2] <= 1.0:
                return False
            else:
                age = (self.get_clock().now()-Time.from_msg(self.latest[0].header.stamp)).nanoseconds/1e9
                if not -0.1 <= age <= 1.0:
                    return False
                self.correction = map_base @ np.linalg.inv(self.latest[1])
                self.initialized = True
                self.pending_initial_pose = None
                text = 'Initial-pose seed received; waiting for accepted GICP match'
        self.report(text)
        return self.initialized

    def process(self):
        started = time.monotonic()
        with self.lock:
            if self.latest is None or not self.initialized or time.monotonic()-self.latest[2] > 1.0:
                return
            message, odom_base, received = self.latest
            if not self.map_point_count and time.monotonic()-self.first_received < self.p['scan_window']:
                return
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == self.processed_stamp:
                return
            self.processed_stamp = stamp
            generation, correction, seeded = self.generation, self.correction.copy(), self.seeded
            # Arrays are replaced, never edited in place; snapshot references
            # stay valid while callbacks replace the current map.
            points = self._registration_points
            map_count = self.map_point_count
            localization_points = self.localization_points
            history = list(self.history)
            previous_odom = self.accepted_odom
            previous_match = (self.accepted_match_correction.copy()
                              if self.accepted_match_correction is not None else correction)
        try:
            frames = []
            frame_sources = []
            stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
            inverse_odom = np.linalg.inv(odom_base)
            for previous,frame_odom,frame_received in history:
                difference = (stamp_ns-Time.from_msg(previous.header.stamp).nanoseconds)/1e9
                if 0 <= difference <= self.p['scan_window']:
                    xyz = filter_points(cloud_xyz(previous))
                    if not len(xyz):
                        continue
                    # Align scan packets using their own timestamped odometry.
                    # This is not per-return IMU deskew within a scan packet.
                    frames.append(transform_points(xyz,inverse_odom@frame_odom))
                    frame_sources.append((frame_received,previous.header.stamp,frame_odom))
            if not frames:
                raise ScanMatchUnavailable('Too few finite/range-filtered cloud points')
            # Empty current packets cannot refresh an estimate made solely
            # from older window geometry. Keep the newest CONTRIBUTING
            # scan's original receipt/stamp as the freshness source.
            source_received,source_stamp,source_odom = max(frame_sources,key=lambda item:item[0])
            if (self.p['mode'] == 'localization' and self.last_accepted is not None and
                    source_received <= self.last_accepted):
                raise ScanMatchUnavailable('No new contributing scan; previous geometry cannot renew acceptance')
            input_points = filter_points(np.vstack(frames))
            source = cloud(input_points, self.p['voxel_size'])
            registration_count = len(source.points)
            if len(source.points) < 100:
                raise ScanMatchUnavailable('Too few finite/range-filtered cloud points')
            # Predict from the last accepted measurement, not the lagging
            # output filter. At a waypoint stop a sparse/repeating scan can
            # otherwise chase filter residual into a different ICP basin.
            # Keep the filtered correction for localization TF/output only.
            # Mapping predicts from its established coordinate anchor.
            prediction = previous_match if self.p['mode'] == 'localization' and seeded else correction
            guess = prediction @ odom_base
            if not map_count and self.p['mode'] == 'mapping':
                # Zero first position/heading, but preserve the odometry Z
                # direction. Identity here had baked initial body roll/pitch
                # into the entire map and its ground/costmap projection.
                matched, fitness, rmse = first_map_pose(odom_base), 1.0, 0.0
            elif self.p['mode'] == 'mapping' or not seeded:
                if self.mapping_store is not None:
                    # Only nearby coarse blocks participate in registration;
                    # all fine blocks remain in the complete saved map.
                    nearby = self.mapping_store.nearby(guess[:3,3], self.p['submap_radius'])
                else:
                    nearby = points[np.linalg.norm(points-guess[:3,3],axis=1) < self.p['submap_radius']]
                target = cloud(nearby, None)
                result = align(source, target, guess,
                               correspondence=self.p['correspondence_distance'],
                               min_fitness=(self.p['localization_min_fitness']
                                            if self.p['mode'] == 'localization' else self.p['min_fitness']),
                               max_rmse=self.p['max_rmse'],
                               max_translation=self.p['max_translation'] if seeded else 2.0,
                               max_rotation=self.p['max_rotation'] if seeded else 0.8)
                require_match(result)
                matched, fitness, rmse = result.transform, result.fitness, result.rmse
            else:
                # Once localized, trust timestamped odometry for the initial
                # guess rather than starting from a coarse, potentially aliased match.
                matched = guess
            if self.p['mode'] == 'localization':
                nearby = localization_points[np.linalg.norm(localization_points-guess[:3,3],axis=1) < self.p['submap_radius']]
                expected = transform_points(input_points,matched)
                margin = 2*self.p['localization_correspondence']
                lower,upper = expected.min(axis=0)-margin,expected.max(axis=0)+margin
                nearby = nearby[np.all((nearby >= lower)&(nearby <= upper),axis=1)]
                fine_source = cloud(input_points,self.p['localization_voxel_size'])
                registration_count = len(fine_source.points)
                result = align(fine_source,cloud(nearby,None),matched,
                               correspondence=self.p['localization_correspondence'],
                               min_fitness=self.p['localization_min_fitness'],
                               max_rmse=min(self.p['max_rmse'],self.p['localization_max_rmse']),
                               max_translation=self.p['localization_max_translation'] if seeded else 2.0,
                               max_rotation=np.deg2rad(self.p['localization_max_rotation_degrees']) if seeded else 0.8,
                               iterations=25,innovation_guess=guess)
                require_match(result, 'Fine GICP ')
                matched,fitness,rmse = result.transform,result.fitness,result.rmse
            new_correction = matched@np.linalg.inv(odom_base)
            # A standing robot's accepted scan-to-map noise must not move
            # the mapping coordinate anchor on every frame. Use a stricter
            # 2mm/0.002rad odometry check than the existing jump guard; real
            # odometry motion still passes through unchanged.
            hold_mapping_anchor = (self.p['mode'] == 'mapping' and seeded and
                                   odometry_is_stationary(previous_odom, odom_base, 0.002, 0.002))
            # Localization compares raw accepted measurements, independent
            # of display-filter lag. Stationary mapping compares its held
            # anchor so small successive corrections cannot drift away.
            translation_limit,rotation_limit = 0.05,0.035
            if self.p['mode'] == 'localization':
                translation_limit = self.p['stationary_translation_limit']
                rotation_limit = np.deg2rad(self.p['stationary_rotation_limit_degrees'])
            reference = correction if hold_mapping_anchor else previous_match
            if not static_correction_is_consistent(reference,new_correction,previous_odom,odom_base,
                                                   translation_limit,rotation_limit):
                jump = np.linalg.inv(reference@odom_base)@matched
                raise ValueError(f'Rejected GICP correction jump while odometry stationary: {np.linalg.norm(jump[:3,3]):.3f}m / {np.degrees(rotation_angle(jump)):.2f}deg (limit {translation_limit:.2f}m / {np.degrees(rotation_limit):.1f}deg)')
            if hold_mapping_anchor:
                # Validate the pose actually used for accumulation as well
                # as the raw GICP solution; holding a frame cannot turn a
                # low-quality scan into a fresh accepted map update.
                held = o3d.pipelines.registration.evaluate_registration(
                    source, target, self.p['correspondence_distance'], correction@odom_base)
                if held.fitness < self.p['min_fitness'] or held.inlier_rmse > self.p['max_rmse']:
                    raise ScanMatchUnavailable('Stationary mapping anchor overlap/residual rejected')
                fitness, rmse = held.fitness, held.inlier_rmse
            age = (self.get_clock().now()-Time.from_msg(source_stamp)).nanoseconds/1e9
            if (not 0 <= time.monotonic()-source_received <= self.p['validity_timeout'] or
                    not -0.1 <= age <= self.p['validity_timeout']):
                raise ScanMatchUnavailable('Registration result too old; reduce cloud/submap size or processing load')
            # Prepare every fallible map/grid/payload calculation before
            # committing pose or acceptance. A failed keyframe cannot move
            # the map frame while its geometry remains at the old pose.
            output_correction = (correction if hold_mapping_anchor else
                                 blend_correction(correction,new_correction,self.p['correction_alpha'],odom_base)
                                 if self.p['mode'] == 'localization' and seeded else new_correction)
            matched = output_correction@odom_base
            mapped_points = transform_points(input_points,matched)
            registered = cloud(mapped_points,self.p['map_voxel_size'])
            new = np.asarray(registered.points)
            header = Header(stamp=message.header.stamp,frame_id='map')
            registered_message = xyz_message(header,new)
            now = time.monotonic()
            update, free_additions = None, None
            if self.mapping_store is not None:
                delta = np.eye(4) if self.last_keyframe is None else np.linalg.inv(self.last_keyframe)@matched
                elapsed = self.last_keyframe_time is None or now-self.last_keyframe_time >= self.p['keyframe_max_interval']
                if self.last_keyframe is None or elapsed or np.linalg.norm(delta[:3,3]) > 0.25 or rotation_angle(delta) > 0.17:
                    update = self.mapping_store.prepare(mapped_points)
                    free_additions = self.grid.prepare_observation(new,matched[:3,3])
            age = (self.get_clock().now()-Time.from_msg(source_stamp)).nanoseconds/1e9
            if (not 0 <= time.monotonic()-source_received <= self.p['validity_timeout'] or
                    not -0.1 <= age <= self.p['validity_timeout']):
                raise ScanMatchUnavailable('Registration result too old after map update preparation; '
                                           'no pose or map changes committed')
            with self.lock:
                if generation != self.generation:
                    return
                if update is not None:
                    self.mapping_store.commit(update)
                    self.grid.free.update(free_additions)
                    self.last_keyframe = matched.copy()
                    self.last_keyframe_time = now
                    if self.mapping_start_pose is None:
                        self.mapping_start_pose = matched.copy()
                self.correction = output_correction
                self.seeded = True
                self.last_accepted = source_received
                self.last_failure = ''
                self.accepted_odom = odom_base.copy()
                self.accepted_match_correction = new_correction.copy()
                # Empty latest packets can shift the window reference, but
                # cannot move the hold's time OR motion anchor forward.
                self.odom_budget.reset(source_odom, Time.from_msg(source_stamp).nanoseconds)
                self.odom_budget.observe(odom_base, Time.from_msg(message.header.stamp).nanoseconds)
                self.hold_exhausted = ''
                self.registered_pub.publish(registered_message)
                count = self.map_point_count
                threshold = self.p['max_map_points']
                warn_capacity = (self.mapping_store is not None and not self.capacity_warned and
                                 threshold > 0 and count > threshold)
                if warn_capacity:
                    self.capacity_warned = True
            if warn_capacity:
                self.get_logger().warning(f'Map size advisory crossed: {count} points > {threshold}; '
                    'mapping continues at unchanged resolution; all historical blocks retained. '
                    'Save a checkpoint when convenient; monitor available memory and storage.')
            self.status_pub.publish(String(data=f'GICP fitness={fitness:.3f} rmse={rmse:.3f} map_points={count} scan_frames={len(frames)} scan_points={len(input_points)} registration_points={registration_count} fine={self.p["mode"] == "localization"}; mapping_anchor={"held" if hold_mapping_anchor else "updated" if self.p["mode"] == "mapping" else "n/a"}; processing_time={time.monotonic()-started:.3f}s source_age={time.monotonic()-source_received:.3f}s; {self.cloud_diagnostics()}'))
        except Exception as error:
            with self.lock:
                if generation != self.generation:
                    return
                # A single poor scan does not destroy a still-fresh accepted
                # estimate. Keep its original receipt deadline and unchanged
                # correction/reference; do not claim this scan was accepted.
                recoverable = self.p['mode'] == 'localization' and isinstance(error, ScanMatchUnavailable)
                if not recoverable:
                    self.last_accepted = None
                self.last_failure = str(error)
                _, state, _ = self.localization_readiness()
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            self.status_pub.publish(String(data=f'{error}; state={state}; {self.cloud_diagnostics()}'))

    def localization_readiness(self):
        """Shared validity/TF gate. Caller holds lock; use only real odom stamps."""
        if not self.seeded or self.last_accepted is None:
            hint = ('initial-pose hint queued; waiting for fresh cloud/odometry'
                    if self.pending_initial_pose is not None else
                    'waiting for initial pose' if not self.initialized else 'waiting for accepted GICP')
            return False, 'registration_invalid', hint+'; last_match_failure='+self.last_failure
        now = time.monotonic()
        accepted_age = now-self.last_accepted
        detail = (f'accepted_age={accepted_age:.2f}s travel={self.odom_budget.distance:.3f}m'
                  f' turn={np.degrees(self.odom_budget.angle):.1f}deg'
                  f'; last_match_failure={self.last_failure or "none"}')
        if not 0 <= accepted_age <= self.p['localization_hold_timeout']:
            return False, 'registration_timeout', detail
        if self.hold_exhausted:
            return False, self.hold_exhausted, detail
        if self.latest is None or not 0 <= now-self.latest[2] <= self.p['validity_timeout']:
            receipt_age = now-self.latest[2] if self.latest is not None else float('inf')
            return False, 'cloud_timeout', detail+f' cloud_receipt_age={receipt_age:.2f}s'
        cloud_age = (self.get_clock().now()-Time.from_msg(self.latest[0].header.stamp)).nanoseconds/1e9
        if not -0.1 <= cloud_age <= self.p['validity_timeout']:
            return False, 'cloud_timestamp_stale', detail+f' cloud_age={cloud_age:.2f}s'
        try:
            odom = self.buffer.lookup_transform('odom', 'base_link', Time())
            stamp = Time.from_msg(odom.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds/1e9
            if not -0.1 <= age <= self.p['localization_odom_timeout']:
                return False, 'odometry_timeout', detail+f' odom_age={age:.2f}s'
            matrix = transform_matrix(odom.transform)
        except (TransformException, ValueError) as error:
            return False, 'odometry_unavailable', detail+f' {error}'
        if not self.odom_budget.observe(matrix, stamp.nanoseconds):
            return False, 'odometry_clock_regressed', detail
        self.hold_exhausted = self.odom_budget.exceeded(
            self.p['localization_hold_distance'], np.deg2rad(self.p['localization_hold_rotation_degrees']))
        detail = (f'accepted_age={accepted_age:.2f}s odom_age={age:.2f}s travel={self.odom_budget.distance:.3f}m'
                  f' turn={np.degrees(self.odom_budget.angle):.1f}deg; last_match_failure={self.last_failure or "none"}')
        if self.hold_exhausted:
            return False, self.hold_exhausted, detail
        state = 'odometry_hold' if self.last_failure or accepted_age > self.p['validity_timeout'] else 'tracking'
        return True, state, detail

    def health(self):
        with self.lock:
            valid = False
            if self.p['mode'] == 'localization':
                valid, state, detail = self.localization_readiness()
                if state != self.localization_state or time.monotonic()-self.waiting_status_time > 2:
                    self.report(f'Localization state={state} valid={valid}; {detail}; '
                                f'{self.cloud_status}; {self.cloud_diagnostics()}')
                    self.waiting_status_time = time.monotonic()
                self.localization_state, self.localization_detail = state, detail
                self.state_pub.publish(String(data=f'state={state} valid={valid}; {detail}'))
        self.valid_pub.publish(Bool(data=valid))

    def publish_correction(self):
        with self.lock:
            if self.p['mode'] == 'localization':
                valid, _, _ = self.localization_readiness()
            else:
                valid = (self.seeded and self.last_accepted is not None and
                         0 <= time.monotonic()-self.last_accepted <= self.p['validity_timeout'])
            if not valid:
                return
            try:
                odom = self.buffer.lookup_transform('odom','base_link',Time())
            except TransformException:
                return
            stamp = Time.from_msg(odom.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds/1e9
            if not -0.1 <= age <= self.p['validity_timeout']:
                return
            if self.last_tf_stamp is not None and stamp.nanoseconds <= self.last_tf_stamp:
                return
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id,transform.child_frame_id = 'map','odom'
            t,q = self.correction[:3,3],quaternion_from_matrix(self.correction)
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z = map(float,t)
            transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w = map(float,q)
            # Keep true 6D body/point-cloud poses. Nav2 gets a separate 2D
            # footprint on the map grid, with measured XY and yaw preserved.
            body = self.correction@transform_matrix(odom.transform)
            body_footprint = np.linalg.inv(body)@planar_pose(body)
            footprint = TransformStamped()
            footprint.header.stamp = odom.header.stamp
            footprint.header.frame_id,footprint.child_frame_id = 'base_link','base_footprint'
            t,q = body_footprint[:3,3],quaternion_from_matrix(body_footprint)
            footprint.transform.translation.x,footprint.transform.translation.y,footprint.transform.translation.z = map(float,t)
            footprint.transform.rotation.x,footprint.transform.rotation.y,footprint.transform.rotation.z,footprint.transform.rotation.w = map(float,q)
            self.broadcaster.sendTransform([transform,footprint])
            self.last_tf_stamp = stamp.nanoseconds

    def publish_map(self):
        with self.lock:
            points = self.points
        if len(points):
            header = Header(stamp=self.get_clock().now().to_msg(),frame_id='map')
            self.map_pub.publish(xyz_message(header,points))

    def save_map(self, request, response):
        if self.p['mode'] != 'mapping':
            response.success, response.message = False, 'Save is available only in mapping mode'
            return response
        with self.lock:
            points, free = self.points.copy(), self.grid.free.copy()
            start_pose = self.mapping_start_pose.copy() if self.mapping_start_pose is not None else np.eye(4)
        if len(points) < 100:
            response.success, response.message = False, 'No usable 3D map yet'
            return response
        root = Path(self.p['output_directory'])
        directory = new_directory(root,map_name(self.p['map_name']))
        try:
            directory.mkdir(parents=True,exist_ok=False)
            # MappingStore already retained every voxel at the configured
            # resolution. Export its exact full snapshot without re-thinning.
            if not o3d.io.write_point_cloud(str(directory/'map.pcd'),cloud(points,None),compressed=True):
                raise RuntimeError('PCD write failed')
            grid = ObservedGrid(self.p['grid_resolution'])
            grid.free = free
            ground = estimate_ground(points,self.p['floor_z'])
            grid.export(points,directory,self.p['floor_z'],self.p['obstacle_min_height'],self.p['obstacle_max_height'],ground)
            denoise_report = clean_directory(directory)
            (directory/'metadata.yaml').write_text(yaml.safe_dump(dict(frame_id='map',parameters=self.p,
                map_start_pose=start_pose.tolist(),
                ground_model=ground,navigation_denoise=denoise_report,
                map_type='incremental GICP scan-to-map; no loop closure',point_count=len(points))))
            # Raw observations remain as a retained version. Publish only
            # the complete shared-coordinate bundle used by every map load.
            aligned = new_directory(root,map_name(self.p['map_name']),'leveled-')
            build_bundle(root,directory,aligned)
            publish(root,aligned)
            response.success, response.message = True, str(aligned)
        except Exception as error:
            response.success, response.message = False, f'{error}; partial files, if any: {directory}'
        return response


def main():
    rclpy.init()
    node = Mapper()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
