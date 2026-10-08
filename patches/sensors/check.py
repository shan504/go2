#!/usr/bin/env python3
"""Read-only preview checks; inspect metadata rather than dumping image/cloud bytes."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CompressedImage
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


def main():
    rclpy.init()
    node = Node("go2_sensor_preview_check")
    latest, counts = {}, {}
    def receive(topic):
        def callback(msg):
            latest[topic] = msg
            counts[topic] = counts.get(topic, 0) + 1
        return callback
    for topic, typ in (("/point_cloud2", PointCloud2), ("/odom", Odometry),
                       ("/camera/image/compressed", CompressedImage)):
        node.create_subscription(typ, topic, receive(topic), qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    end = time.monotonic() + 10.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    okay = True
    now_ns = node.get_clock().now().nanoseconds
    for topic in ("/point_cloud2", "/odom", "/camera/image/compressed"):
        msg = latest.get(topic)
        if msg is None:
            print("MISSING " + topic)
            okay = False
            continue
        age = (now_ns - msg.header.stamp.sec * 1_000_000_000 - msg.header.stamp.nanosec) / 1e9
        detail = "messages=%d frame=%s age=%.3fs" % (counts[topic], msg.header.frame_id, age)
        okay &= -0.5 <= age < 2.0
        if topic == "/point_cloud2":
            detail += " points=%d fields=%s" % (msg.width * msg.height, ",".join(f.name for f in msg.fields))
            okay &= msg.width * msg.height > 0
        if topic == "/camera/image/compressed":
            detail += " format=%s bytes=%d" % (msg.format, len(msg.data))
            okay &= len(msg.data) > 0
        print(topic + ": " + detail)
    try:
        tf = buffer.lookup_transform("odom", "base_link", Time())
        print("TF odom -> base_link: OK")
    except TransformException as error:
        print("MISSING TF: " + str(error))
        okay = False
    print("PASS: live sensor preview" if okay else "INCOMPLETE: send this output and camera-node logs")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if okay else 1


if __name__ == "__main__":
    raise SystemExit(main())
