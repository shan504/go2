#!/usr/bin/env python3
"""Recreate only native odom->base_link TF from a recorded /odom stream.

Replay raw input topics, not recorded /tf: recorded global correction would
conflict with the baseline or candidate backend's newly computed result.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ReplayOdometryTF(Node):
    def __init__(self):
        super().__init__('go2_replay_odom_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link'
                or not all(math.isfinite(value) for value in values)
                or abs(sum(value * value for value in values[3:]) - 1.0) > 1e-3
                or (message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0)):
            self.get_logger().warning('Rejected malformed recorded native odometry', throttle_duration_sec=2.0)
            return
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ReplayOdometryTF()
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
