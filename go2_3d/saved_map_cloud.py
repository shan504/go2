#!/usr/bin/env python3
"""Publish a complete saved map in a process separate from localization timers.

The selected map is immutable for the lifetime of this node. Only the display
message timestamp changes; all saved XYZ coordinates and points are preserved.
"""
import math
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from ros_cloud import xyz_message


class SavedMapCloud(Node):
    def __init__(self):
        super().__init__('go2_saved_map_cloud')
        self.declare_parameter('map_directory', '/maps/latest')
        self.declare_parameter('map_publish_period', 2.0)
        self.declare_parameter('map_frame', 'map')
        directory = Path(self.get_parameter('map_directory').value)
        period = float(self.get_parameter('map_publish_period').value)
        frame = self.get_parameter('map_frame').value
        if not math.isfinite(period) or period <= 0:
            raise ValueError('map_publish_period must be finite and positive')
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError('map_frame must be nonempty')
        source = directory/'map.pcd'
        if not source.is_file():
            raise ValueError(f'Missing saved map: {source}')
        cloud = o3d.io.read_point_cloud(str(source))
        points = np.asarray(cloud.points)
        if not len(points):
            raise ValueError(f'Empty saved map: {source}')
        # Match Mapper's complete XYZ message, with no filtering, voxelization,
        # coordinate correction or truncation. Pack once instead of every tick.
        self.message = xyz_message(Header(frame_id=frame), points)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/map_cloud', retained)
        self.timer = self.create_timer(period, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'Saved map display ready: {len(points)} points from {source}; '
            f'frame={frame}; period={period:g}s; isolated from localization')

    def publish_map(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = None
    try:
        node = SavedMapCloud()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
