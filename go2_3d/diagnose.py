#!/usr/bin/env python3
"""Read-only sensor/TF diagnostics: no parameters, services or motion requests."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import String, Bool
from tf2_ros import Buffer, TransformListener, TransformException
from geometry import cloud_xyz


class Diagnostics(Node):
    def __init__(self):
        super().__init__('go2_3d_readonly_diagnostics')
        self.latest, self.counts, self.received = {}, {}, {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.types = {
            '/utlidar/cloud_base':PointCloud2,'/utlidar/robot_odom':Odometry,
            '/point_cloud2':PointCloud2,'/odom':Odometry,'/map_cloud':PointCloud2,
            '/camera/image/compressed':CompressedImage,
            '/localization/status':String,'/localization/valid':Bool,
        }
        for topic,kind in self.types.items():
            def receive(message,topic=topic):
                self.latest[topic] = message
                self.counts[topic] = self.counts.get(topic,0)+1
                self.received[topic] = time.monotonic()
            self.create_subscription(kind,topic,receive,
                                     self.retained if topic == '/map_cloud' else qos_profile_sensor_data)

    def summary(self):
        lines = ['READ-ONLY 3D diagnostics (raw timestamps are not host timestamps)',
                 'Nodes: '+', '.join(sorted(self.get_node_names()))]
        now = self.get_clock().now().nanoseconds
        for topic,kind in self.types.items():
            message = self.latest.get(topic)
            if message is None:
                lines.append(f'{topic}: NO MESSAGES; publishers={self.count_publishers(topic)}')
                continue
            prefix = f'{topic}: messages={self.counts[topic]} receipt_age={time.monotonic()-self.received[topic]:.2f}s'
            if kind in (String,Bool):
                lines.append(prefix+f' data={message.data}')
                continue
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            prefix += f' frame={message.header.frame_id!r} stamp={stamp:.6f}'
            if not topic.startswith('/utlidar/'):
                prefix += f' host_age={now/1e9-stamp:.3f}s'
            if kind is Odometry:
                prefix += f' child={message.child_frame_id!r}'
            if kind is PointCloud2:
                prefix += f' points={message.width*message.height} fields={[f.name for f in message.fields]}'
                try:
                    points = cloud_xyz(message)
                    points = points[np.isfinite(points).all(axis=1)]
                    prefix += f' finite_xyz={len(points)}'
                    if len(points):
                        prefix += f' xyz_min={np.round(points.min(axis=0),2).tolist()} xyz_max={np.round(points.max(axis=0),2).tolist()}'
                except Exception as error:
                    prefix += f' XYZ_DECODE_ERROR={error}'
            lines.append(prefix)
        if all(k in self.latest for k in ('/utlidar/cloud_base','/utlidar/robot_odom')):
            def stamp(message):
                return message.header.stamp.sec+message.header.stamp.nanosec/1e9
            difference = stamp(self.latest['/utlidar/cloud_base'])-stamp(self.latest['/utlidar/robot_odom'])
            lines.append(f'Latest raw cloud_stamp - odom_stamp = {difference:.6f}s (asynchronous samples)')
        for parent,child in (('odom','base_link'),('map','odom')):
            try:
                tf = self.buffer.lookup_transform(parent,child,Time())
                stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
                lines.append(f'TF {parent} -> {child}: OK host_age={now/1e9-stamp:.3f}s')
            except TransformException as error:
                lines.append(f'TF {parent} -> {child}: MISSING {error}')
        if '/point_cloud2' in self.latest:
            try:
                self.buffer.lookup_transform('odom','base_link',Time.from_msg(self.latest['/point_cloud2'].header.stamp))
                lines.append('TF at latest cloud timestamp: OK')
            except TransformException as error:
                lines.append(f'TF at latest cloud timestamp: UNAVAILABLE {error}')
        lines.append('Other lidar topics: '+str([(t,k) for t,k in self.get_topic_names_and_types() if t.startswith('/utlidar/')]))
        return '\n'.join(lines)


def main():
    rclpy.init()
    node = Diagnostics()
    try:
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=0.2)
        print(node.summary(),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
