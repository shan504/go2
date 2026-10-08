#!/usr/bin/env python3
"""Odometry-seeded 3D scan-to-map GICP and saved-map localization (ROS2 Humble)."""
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
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
from registration import cloud, align, static_correction_is_consistent, blend_correction
from ros_cloud import xyz_message
from ground import estimate_ground


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return pose_matrix([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])


class Mapper(Node):
    def __init__(self):
        super().__init__('go2_gicp')
        defaults = dict(mode='mapping', map_directory='/maps/latest', output_directory='/maps',
                        voxel_size=0.15, map_voxel_size=0.06, keyframe_max_interval=2.0,
                        scan_window=0.0, map_publish_period=2.0,
                        grid_resolution=0.10, floor_z=-0.30,
                        obstacle_min_height=0.10, obstacle_max_height=1.5,
                        max_map_points=200000, submap_radius=25.0,
                        registration_period=0.5, validity_timeout=1.0,
                        correspondence_distance=0.7, min_fitness=0.55, max_rmse=0.20,
                        max_translation=0.6, max_rotation=0.35,
                        localization_voxel_size=0.05, localization_correspondence=0.20,
                        correction_alpha=0.25,auto_initialize=False,
                        initial_x=0.0,initial_y=0.0,initial_z=0.0,initial_yaw_degrees=0.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if not 0 < self.p['correction_alpha'] <= 1:
            raise ValueError('correction_alpha must be greater than 0 and at most 1')
        if self.p['mode'] not in ('mapping','localization'):
            raise ValueError('mode must be mapping or localization')
        self.lock = threading.RLock()
        self.generation = 0
        self.latest = None
        self.history = deque(maxlen=32)
        self.first_received = None
        self.processed_stamp = None
        self.correction = np.eye(4)
        self.initialized = self.p['mode'] == 'mapping'
        self.seeded = False
        self.last_accepted = None
        self.accepted_odom = None
        self.last_tf_stamp = None
        self.initial_hint_used = False
        self.waiting_status_time = 0.0
        self.last_failure = ''
        self.last_keyframe = None
        self.last_keyframe_time = None
        self.points = np.empty((0,3))
        self.registration_points = np.empty((0,3))
        self.localization_points = np.empty((0,3))
        self.grid = ObservedGrid(self.p['grid_resolution'])
        if self.p['mode'] == 'localization':
            directory = Path(self.p['map_directory'])
            self.points = np.asarray(o3d.io.read_point_cloud(str(directory/'map.pcd')).points).copy()
            if len(self.points) < 100:
                raise ValueError('Missing/empty 3D map.pcd; localization cannot start')
            self.registration_points = np.asarray(cloud(self.points,self.p['voxel_size']).points).copy()
            self.localization_points = np.asarray(cloud(self.points,self.p['localization_voxel_size']).points).copy()
            self.get_logger().info('Loaded 3D map; waiting for /initialpose in map frame')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, '/map_cloud', retained)
        self.registered_pub = self.create_publisher(PointCloud2, '/registered_cloud', qos_profile_sensor_data)
        self.valid_pub = self.create_publisher(Bool, '/localization/valid', 1)
        self.status_pub = self.create_publisher(String, '/localization/status', 1)
        self.create_subscription(PointCloud2, '/point_cloud2', self.receive_cloud, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose, 1)
        self.create_service(Trigger, '/save_3d_map', self.save_map)
        self.create_timer(0.1, self.health)
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

    def receive_cloud(self, message):
        if message.header.frame_id != 'base_link':
            self.get_logger().error('Expected cloud already transformed to base_link', throttle_duration_sec=5)
            return
        age = (self.get_clock().now()-Time.from_msg(message.header.stamp)).nanoseconds/1e9
        if not -0.1 <= age <= 1.0:
            return
        try:
            odom = self.buffer.lookup_transform('odom','base_link',Time.from_msg(message.header.stamp))
            matrix = transform_matrix(odom.transform)
        except (TransformException, ValueError):
            self.get_logger().warning('Waiting for odom -> base_link at cloud timestamp', throttle_duration_sec=5)
            return
        with self.lock:
            if self.latest is not None and Time.from_msg(message.header.stamp) <= Time.from_msg(self.latest[0].header.stamp):
                return
            self.latest = (message, matrix, time.monotonic())
            if self.first_received is None:
                self.first_received = self.latest[2]
            self.history.append(self.latest)
        if (self.p['mode'] == 'localization' and self.p['auto_initialize'] and
                not self.initial_hint_used):
            self.initial_hint_used = True
            hint = PoseWithCovarianceStamped()
            hint.header.frame_id = 'map'
            p,q = hint.pose.pose.position,hint.pose.pose.orientation
            p.x,p.y,p.z = self.p['initial_x'],self.p['initial_y'],self.p['initial_z']
            angle = np.deg2rad(self.p['initial_yaw_degrees'])/2
            q.z,q.w = float(np.sin(angle)),float(np.cos(angle))
            self.initial_pose(hint)

    def initial_pose(self, message):
        if self.p['mode'] != 'localization' or message.header.frame_id != 'map':
            self.get_logger().warning('/initialpose is supported only in localization, frame map')
            return
        with self.lock:
            if self.latest is None or time.monotonic()-self.latest[2] > 1.0:
                self.get_logger().warning('Need fresh cloud and timestamp-aligned odometry before initialpose')
                return
            pose = message.pose.pose
            try:
                map_base = pose_matrix([pose.position.x,pose.position.y,pose.position.z],
                                       [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            except ValueError as error:
                self.get_logger().error(str(error))
                return
            self.correction = map_base @ np.linalg.inv(self.latest[1])
            self.initialized, self.seeded = True, False
            self.last_accepted = None
            self.accepted_odom = None
            self.processed_stamp = None
            self.generation += 1
        self.report('Initial-pose seed received; waiting for accepted GICP match')

    def process(self):
        with self.lock:
            if self.latest is None or not self.initialized or time.monotonic()-self.latest[2] > 1.0:
                return
            message, odom_base, received = self.latest
            if not len(self.points) and time.monotonic()-self.first_received < self.p['scan_window']:
                return
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == self.processed_stamp:
                return
            self.processed_stamp = stamp
            generation, correction, seeded = self.generation, self.correction.copy(), self.seeded
            # Arrays are replaced, never edited in place; snapshot references
            # stay valid while callbacks replace the current map.
            points = self.registration_points
            localization_points = self.localization_points
            history = list(self.history)
            previous_odom = self.accepted_odom
        try:
            frames = []
            stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
            inverse_odom = np.linalg.inv(odom_base)
            for previous,frame_odom,_ in history:
                difference = (stamp_ns-Time.from_msg(previous.header.stamp).nanoseconds)/1e9
                if 0 <= difference <= self.p['scan_window']:
                    xyz = filter_points(cloud_xyz(previous))
                    # Align scan packets using their own timestamped odometry.
                    # This is not per-return IMU deskew within a scan packet.
                    frames.append(transform_points(xyz,inverse_odom@frame_odom))
            input_points = filter_points(np.vstack(frames))
            source = cloud(input_points, self.p['voxel_size'])
            registration_count = len(source.points)
            if len(source.points) < 100:
                raise ValueError('Too few finite/range-filtered cloud points')
            guess = correction @ odom_base
            if not len(points) and self.p['mode'] == 'mapping':
                # Zero first position/heading, but preserve the odometry Z
                # direction. Identity here had baked initial body roll/pitch
                # into the entire map and its ground/costmap projection.
                matched, fitness, rmse = first_map_pose(odom_base), 1.0, 0.0
            elif self.p['mode'] == 'mapping' or not seeded:
                nearby = points[np.linalg.norm(points-guess[:3,3],axis=1) < self.p['submap_radius']]
                result = align(source, cloud(nearby, None), guess,
                               correspondence=self.p['correspondence_distance'],
                               min_fitness=self.p['min_fitness'], max_rmse=self.p['max_rmse'],
                               max_translation=self.p['max_translation'] if seeded else 2.0,
                               max_rotation=self.p['max_rotation'] if seeded else 0.8)
                if not result.accepted:
                    raise ValueError(f'{result.reason}: fitness={result.fitness:.3f} rmse={result.rmse:.3f}')
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
                               min_fitness=self.p['min_fitness'],max_rmse=min(self.p['max_rmse'],0.10),
                               max_translation=self.p['max_translation'] if seeded else 2.0,
                               max_rotation=self.p['max_rotation'] if seeded else 0.8,
                               iterations=25,innovation_guess=guess)
                if not result.accepted:
                    raise ValueError(f'Fine GICP {result.reason}: fitness={result.fitness:.3f} rmse={result.rmse:.3f}')
                matched,fitness,rmse = result.transform,result.fitness,result.rmse
            new_correction = matched@np.linalg.inv(odom_base)
            if not static_correction_is_consistent(correction,new_correction,previous_odom,odom_base):
                jump = np.linalg.inv(guess)@matched
                raise ValueError(f'Rejected GICP correction jump while odometry stationary: {np.linalg.norm(jump[:3,3]):.3f}m / {np.degrees(rotation_angle(jump)):.2f}deg (limit 0.05m / 2deg)')
            age = (self.get_clock().now()-Time.from_msg(message.header.stamp)).nanoseconds/1e9
            if time.monotonic()-received > self.p['validity_timeout'] or age > self.p['validity_timeout']:
                raise ValueError('Registration result too old; reduce cloud/submap size or processing load')
            with self.lock:
                if generation != self.generation:
                    return
                # Never filter a rejected match. Bootstrap/reseed snaps only
                # after GICP acceptance; later correction noise is damped while
                # odom->base_link retains the full measured robot motion.
                self.correction = (blend_correction(correction,new_correction,self.p['correction_alpha'],odom_base)
                                   if self.p['mode'] == 'localization' and seeded else new_correction)
                matched = self.correction@odom_base
                self.seeded = True
                # Registration stays coarse for ARM CPU cost. Retain original
                # XYZ at a finer resolution for the 3D map and visual scan.
                registered = cloud(transform_points(input_points,matched),self.p['map_voxel_size'])
                new = np.asarray(registered.points)
                now = time.monotonic()
                if self.p['mode'] == 'mapping':
                    delta = np.eye(4) if self.last_keyframe is None else np.linalg.inv(self.last_keyframe)@matched
                    elapsed = self.last_keyframe_time is None or now-self.last_keyframe_time >= self.p['keyframe_max_interval']
                    if self.last_keyframe is None or elapsed or np.linalg.norm(delta[:3,3]) > 0.25 or rotation_angle(delta) > 0.17:
                        combined = np.asarray(cloud(np.vstack((self.points,new)),self.p['map_voxel_size']).points).copy()
                        if len(combined) > self.p['max_map_points']:
                            raise ValueError('Map point budget reached; save map and begin localization')
                        self.grid.observe(new,matched[:3,3])
                        self.points, self.last_keyframe = combined, matched.copy()
                        self.registration_points = np.asarray(cloud(combined,self.p['voxel_size']).points).copy()
                        self.last_keyframe_time = now
                self.last_accepted = received
                self.last_failure = ''
                self.accepted_odom = odom_base.copy()
                header = Header(stamp=message.header.stamp,frame_id='map')
                self.registered_pub.publish(xyz_message(header,new))
            self.status_pub.publish(String(data=f'GICP fitness={fitness:.3f} rmse={rmse:.3f} map_points={len(self.points)} scan_frames={len(frames)} scan_points={len(input_points)} registration_points={registration_count} fine={self.p["mode"] == "localization"}'))
        except Exception as error:
            with self.lock:
                if generation == self.generation:
                    self.last_accepted = None
                    self.last_failure = str(error)
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            self.status_pub.publish(String(data=str(error)))

    def health(self):
        with self.lock:
            valid = (self.p['mode'] == 'localization' and self.last_accepted is not None and
                     time.monotonic()-self.last_accepted <= self.p['validity_timeout'])
            if self.p['mode']=='localization' and not self.initialized and time.monotonic()-self.waiting_status_time>2:
                text = ('Waiting for fresh cloud and timestamped odometry' if self.latest is None else
                        'Waiting for actual initial map pose; Nav2 startup deferred')
                self.status_pub.publish(String(data=text))
                self.waiting_status_time = time.monotonic()
            elif self.p['mode']=='localization' and not valid and time.monotonic()-self.waiting_status_time>2:
                cloud_age = time.monotonic()-self.latest[2] if self.latest else float('inf')
                accepted_age = time.monotonic()-self.last_accepted if self.last_accepted else float('inf')
                self.status_pub.publish(String(data=f'Localization invalid: {self.last_failure or "waiting for a fresh accepted GICP match"}; cloud_age={cloud_age:.2f}s accepted_age={accepted_age:.2f}s'))
                self.waiting_status_time = time.monotonic()
        self.valid_pub.publish(Bool(data=valid))

    def publish_correction(self):
        with self.lock:
            if (not self.seeded or self.last_accepted is None or
                    time.monotonic()-self.last_accepted > self.p['validity_timeout']):
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
        if len(points) < 100:
            response.success, response.message = False, 'No usable 3D map yet'
            return response
        root = Path(self.p['output_directory'])
        directory = root/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        try:
            directory.mkdir(parents=True,exist_ok=False)
            if not o3d.io.write_point_cloud(str(directory/'map.pcd'),cloud(points,self.p['map_voxel_size']),compressed=True):
                raise RuntimeError('PCD write failed')
            grid = ObservedGrid(self.p['grid_resolution'])
            grid.free = free
            ground = estimate_ground(points,self.p['floor_z'])
            grid.export(points,directory,self.p['floor_z'],self.p['obstacle_min_height'],self.p['obstacle_max_height'],ground)
            (directory/'metadata.yaml').write_text(yaml.safe_dump(dict(frame_id='map',parameters=self.p,
                ground_model=ground,map_type='incremental GICP scan-to-map; no loop closure',point_count=len(points))))
            link = root/'.latest-new'
            if link.is_symlink():
                link.unlink()
            link.symlink_to(directory.name)
            os.replace(link,root/'latest')
            response.success, response.message = True, str(directory)
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
