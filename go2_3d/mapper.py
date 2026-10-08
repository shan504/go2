#!/usr/bin/env python3
"""Odometry-seeded 3D scan-to-map GICP and saved-map localization (ROS2 Humble)."""
import os
import threading
import time
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
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry import (pose_matrix, quaternion_from_matrix, transform_points,
                      cloud_xyz, filter_points, rotation_angle, ObservedGrid)
from registration import cloud, align


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return pose_matrix([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])


class Mapper(Node):
    def __init__(self):
        super().__init__('go2_gicp')
        defaults = dict(mode='mapping', map_directory='/maps/latest', output_directory='/maps',
                        voxel_size=0.15, grid_resolution=0.10, floor_z=-0.30,
                        obstacle_min_height=0.10, obstacle_max_height=1.5,
                        max_map_points=200000, submap_radius=25.0,
                        registration_period=0.5, validity_timeout=1.0,
                        correspondence_distance=0.7, min_fitness=0.55, max_rmse=0.20,
                        max_translation=0.6, max_rotation=0.35)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if self.p['mode'] not in ('mapping','localization'):
            raise ValueError('mode must be mapping or localization')
        self.lock = threading.RLock()
        self.generation = 0
        self.latest = None
        self.processed_stamp = None
        self.correction = np.eye(4)
        self.initialized = self.p['mode'] == 'mapping'
        self.seeded = False
        self.last_accepted = None
        self.last_keyframe = None
        self.points = np.empty((0,3))
        self.grid = ObservedGrid(self.p['grid_resolution'])
        if self.p['mode'] == 'localization':
            directory = Path(self.p['map_directory'])
            self.points = np.asarray(o3d.io.read_point_cloud(str(directory/'map.pcd')).points).copy()
            if len(self.points) < 100:
                raise ValueError('Missing/empty 3D map.pcd; localization cannot start')
            self.get_logger().info('Loaded 3D map; waiting for /initialpose in map frame')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, '/map_cloud', retained)
        self.valid_pub = self.create_publisher(Bool, '/localization/valid', 1)
        self.status_pub = self.create_publisher(String, '/localization/status', 1)
        self.create_subscription(PointCloud2, '/point_cloud2', self.receive_cloud, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose, 1)
        self.create_service(Trigger, '/save_3d_map', self.save_map)
        self.create_timer(0.1, self.health)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(self.p['registration_period'], self.process,
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
            self.latest = (message, matrix, time.monotonic())

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
            self.processed_stamp = None
            self.generation += 1
        self.report('Initial-pose seed received; waiting for accepted GICP match')

    def process(self):
        with self.lock:
            if self.latest is None or not self.initialized or time.monotonic()-self.latest[2] > 1.0:
                return
            message, odom_base, received = self.latest
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == self.processed_stamp:
                return
            self.processed_stamp = stamp
            generation, correction, seeded = self.generation, self.correction.copy(), self.seeded
            points = self.points.copy()
        try:
            source = cloud(filter_points(cloud_xyz(message)), self.p['voxel_size'])
            if len(source.points) < 100:
                raise ValueError('Too few finite/range-filtered cloud points')
            guess = correction @ odom_base
            if not len(points) and self.p['mode'] == 'mapping':
                # First body pose is the map origin, with floor_z relative to it.
                matched, fitness, rmse = np.eye(4), 1.0, 0.0
            else:
                nearby = points[np.linalg.norm(points-guess[:3,3],axis=1) < self.p['submap_radius']]
                result = align(source, cloud(nearby, self.p['voxel_size']), guess,
                               correspondence=self.p['correspondence_distance'],
                               min_fitness=self.p['min_fitness'], max_rmse=self.p['max_rmse'],
                               max_translation=self.p['max_translation'] if seeded else 2.0,
                               max_rotation=self.p['max_rotation'] if seeded else 0.8)
                if not result.accepted:
                    raise ValueError(f'{result.reason}: fitness={result.fitness:.3f} rmse={result.rmse:.3f}')
                matched, fitness, rmse = result.transform, result.fitness, result.rmse
            age = (self.get_clock().now()-Time.from_msg(message.header.stamp)).nanoseconds/1e9
            if time.monotonic()-received > self.p['validity_timeout'] or age > self.p['validity_timeout']:
                raise ValueError('Registration result too old; reduce cloud/submap size or processing load')
            with self.lock:
                if generation != self.generation:
                    return
                self.correction = matched @ np.linalg.inv(odom_base)
                self.seeded = True
                if self.p['mode'] == 'mapping':
                    delta = np.eye(4) if self.last_keyframe is None else np.linalg.inv(self.last_keyframe)@matched
                    if self.last_keyframe is None or np.linalg.norm(delta[:3,3]) > 0.25 or rotation_angle(delta) > 0.17:
                        new = transform_points(np.asarray(source.points),matched)
                        combined = np.asarray(cloud(np.vstack((self.points,new)),self.p['voxel_size']).points).copy()
                        if len(combined) > self.p['max_map_points']:
                            raise ValueError('Map point budget reached; save map and begin localization')
                        self.grid.observe(new,matched[:3,3])
                        self.points, self.last_keyframe = combined, matched.copy()
                self.last_accepted = received
                transform = TransformStamped()
                transform.header.stamp = message.header.stamp
                transform.header.frame_id, transform.child_frame_id = 'map','odom'
                t = self.correction[:3,3]
                q = quaternion_from_matrix(self.correction)
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float,t)
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float,q)
                self.broadcaster.sendTransform(transform)
            self.status_pub.publish(String(data=f'GICP fitness={fitness:.3f} rmse={rmse:.3f} map_points={len(self.points)}'))
        except Exception as error:
            with self.lock:
                if generation == self.generation:
                    self.last_accepted = None
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            self.status_pub.publish(String(data=str(error)))

    def health(self):
        with self.lock:
            valid = (self.p['mode'] == 'localization' and self.last_accepted is not None and
                     time.monotonic()-self.last_accepted <= self.p['validity_timeout'])
        self.valid_pub.publish(Bool(data=valid))

    def publish_map(self):
        with self.lock:
            points = self.points.copy()
        if len(points):
            header = Header(stamp=self.get_clock().now().to_msg(),frame_id='map')
            self.map_pub.publish(point_cloud2.create_cloud_xyz32(header,points.astype(np.float32).tolist()))

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
            if not o3d.io.write_point_cloud(str(directory/'map.pcd'),cloud(points,self.p['voxel_size']),compressed=True):
                raise RuntimeError('PCD write failed')
            grid = ObservedGrid(self.p['grid_resolution'])
            grid.free = free
            grid.export(points,directory,self.p['floor_z'],self.p['obstacle_min_height'],self.p['obstacle_max_height'])
            (directory/'metadata.yaml').write_text(yaml.safe_dump(dict(frame_id='map',parameters=self.p,
                map_type='incremental GICP scan-to-map; no loop closure',point_count=len(points))))
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
