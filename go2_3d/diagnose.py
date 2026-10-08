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
from tf2_msgs.msg import TFMessage
from geometry import cloud_xyz
from geometry import pose_matrix, rotation_angle


class Diagnostics(Node):
    def __init__(self):
        super().__init__('go2_3d_readonly_diagnostics')
        self.latest, self.counts, self.received = {}, {}, {}
        self.tf_samples = {('odom','base_link'):[], ('map','odom'):[]}
        self.tf_sources = {edge:set() for edge in self.tf_samples}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.create_subscription(TFMessage,'/tf',self.observe_tf_sources,qos_profile_sensor_data)
        self.retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.types = {
            '/utlidar/cloud_base':PointCloud2,'/utlidar/robot_odom':Odometry,
            '/utlidar/cloud_deskewed':PointCloud2,'/utlidar/voxel_map':PointCloud2,
            '/uslam/cloud_map':PointCloud2,'/uslam/frontend/cloud_world_ds':PointCloud2,
            '/uslam/frontend/odom':Odometry,
            '/point_cloud2':PointCloud2,'/registered_cloud':PointCloud2,'/odom':Odometry,'/map_cloud':PointCloud2,
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
        self.create_timer(0.05,self.sample_tf)

    def observe_tf_sources(self,message,info):
        for tf in message.transforms:
            edge = (tf.header.frame_id,tf.child_frame_id)
            if edge in self.tf_sources:
                self.tf_sources[edge].add(bytes(info.publisher_gid))

    def sample_tf(self):
        for (parent,child),samples in self.tf_samples.items():
            try:
                tf = self.buffer.lookup_transform(parent,child,Time())
                stamp = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                if samples and samples[-1][0] == stamp:
                    continue
                t,q = tf.transform.translation,tf.transform.rotation
                matrix = pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
                samples.append((stamp,matrix))
                del samples[:-500]
            except (TransformException,ValueError):
                pass

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
            if not topic.startswith(('/utlidar/','/uslam/')):
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
                        prefix += f' z_span={np.ptp(points[:,2]):.3f}m'
                except Exception as error:
                    prefix += f' XYZ_DECODE_ERROR={error}'
            lines.append(prefix)
        if all(k in self.latest for k in ('/utlidar/cloud_base','/utlidar/robot_odom')):
            def stamp(message):
                return message.header.stamp.sec+message.header.stamp.nanosec/1e9
            difference = stamp(self.latest['/utlidar/cloud_base'])-stamp(self.latest['/utlidar/robot_odom'])
            lines.append(f'Latest raw cloud_stamp - odom_stamp = {difference:.6f}s (asynchronous samples)')
        for parent,child in (('odom','base_link'),('map','odom')):
            sources = len(self.tf_sources[(parent,child)])
            lines.append(f'TF {parent} -> {child}: observed_dynamic_publishers={sources}'+
                         (' CONFLICT: multiple publishers for the same TF edge' if sources>1 else ''))
            try:
                tf = self.buffer.lookup_transform(parent,child,Time())
                stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
                lines.append(f'TF {parent} -> {child}: OK host_age={now/1e9-stamp:.3f}s')
            except TransformException as error:
                lines.append(f'TF {parent} -> {child}: MISSING {error}')
            samples = self.tf_samples[(parent,child)]
            if len(samples)>1:
                positions = np.array([m[:3,3] for _,m in samples])
                changes = [np.linalg.inv(a)@b for (_,a),(_,b) in zip(samples,samples[1:])]
                displacement = max(np.linalg.norm(m[:3,3]) for m in changes)
                angle = np.degrees(max(rotation_angle(m) for m in changes))
                lines.append(f'TF {parent} -> {child} variation: samples={len(samples)} position_span={np.round(np.ptp(positions,axis=0),4).tolist()}m max_step={displacement:.4f}m max_rotation_step={angle:.3f}deg')
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
