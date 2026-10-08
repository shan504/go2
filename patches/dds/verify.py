#!/usr/bin/env python3
"""Read-only checks for live sensors, usable scans, map and TF."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = Node("go2_edu_readonly_check")
    latest = {}
    counts = {}
    def receive(topic):
        def callback(msg):
            latest[topic] = msg
            counts[topic] = counts.get(topic, 0) + 1
        return callback
    for topic, kind in (("/odom", Odometry), ("/point_cloud2", PointCloud2), ("/scan", LaserScan)):
        node.create_subscription(kind, topic, receive(topic), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/map", receive("/map"), QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    okay = True
    now_ns = node.get_clock().now().nanoseconds
    for topic in ("/odom", "/point_cloud2", "/scan", "/map"):
        msg = latest.get(topic)
        if msg is None:
            print("FAIL %s: no messages" % topic)
            okay = False
            continue
        detail = "messages=%d frame=%s" % (counts[topic], msg.header.frame_id)
        if topic == "/map":
            known = sum(v >= 0 for v in msg.data)
            detail += " size=%dx%d known_cells=%d" % (msg.info.width, msg.info.height, known)
            okay &= known > 0 and msg.info.width > 0 and msg.info.height > 0
        else:
            age = (now_ns - msg.header.stamp.sec * 1_000_000_000 - msg.header.stamp.nanosec) / 1e9
            detail += " age=%.3fs" % age
            okay &= -0.5 <= age < 2.0
            if topic == "/point_cloud2":
                detail += " points=%d" % (msg.width * msg.height)
                okay &= msg.width * msg.height > 0
            if topic == "/scan":
                valid = sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)
                detail += " finite_ranges=%d" % valid
                okay &= valid > 0
        print(topic + ": " + detail)
    for parent, child in (("odom", "base_link"), ("map", "odom")):
        try:
            tf = buffer.lookup_transform(parent, child, Time())
            print("TF %s -> %s: OK (stamp %d.%09d)" %
                  (parent, child, tf.header.stamp.sec, tf.header.stamp.nanosec))
        except TransformException as exc:
            print("FAIL TF %s -> %s: %s" % (parent, child, exc))
            okay = False
    print("PASS: mapping data chain is live" if okay else "INCOMPLETE: send the output for diagnosis")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if okay else 1


if __name__ == "__main__":
    raise SystemExit(main())
