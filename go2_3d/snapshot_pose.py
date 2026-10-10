#!/usr/bin/env python3
"""Read a fresh valid map pose for a navigation restart; no robot writes."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = Node('go2_restart_pose_readonly')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    validity = [False, 0.0]
    def valid(message):
        validity[:] = [message.data, time.monotonic()]
    node.create_subscription(Bool, '/localization/valid', valid, qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not validity[0] or time.monotonic()-validity[1] > 0.5:
                continue
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                continue
            age = (node.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds/1e9
            if not -0.1 <= age <= 1.0:
                continue
            position, rotation = transform.transform.translation, transform.transform.rotation
            values = (position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w)
            if not all(math.isfinite(value) for value in values):
                continue
            yaw = math.degrees(math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                                         1-2*(rotation.y**2+rotation.z**2)))
            print(json.dumps(dict(x=position.x, y=position.y, z=position.z,
                                  yaw_degrees=yaw, captured_at=time.time())), flush=True)
            return 0
        print('Cannot retain pose: fresh valid GICP localization and map -> base_link TF required. '
              'Navigation was disabled; the container has not been restarted.', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
