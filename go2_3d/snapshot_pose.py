#!/usr/bin/env python3
"""Read a fresh valid map pose for a navigation restart; no robot writes."""
import json
import math
import sys
import time
from pathlib import Path
import yaml
from geometry import pose_matrix
from ground import level_transform

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
            body = pose_matrix(values[:3], values[3:])
            # Old running navigation may still use a body-origin map. Convert
            # its retained pose to the same ground frame the next load uses.
            for path in (Path('/runtime/navigation_map/metadata.yaml'), Path('/maps/latest/metadata.yaml')):
                if path.is_file():
                    metadata = yaml.safe_load(path.read_text())
                    if metadata.get('ground_model'):
                        body = level_transform(metadata['ground_model'])@body
                    break
            yaw = math.degrees(math.atan2(body[1,0], body[0,0]))
            print(json.dumps(dict(x=float(body[0,3]), y=float(body[1,3]), z=float(body[2,3]),
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
