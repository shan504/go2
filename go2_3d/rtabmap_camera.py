#!/usr/bin/env python3
"""Opt-in calibrated mono camera for RTAB-Map. Never supplies guessed calibration."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from camera_calibration import (load_calibration, validate_input_topic,
                                corrected_stamp_ns, CameraRectifier)


class LoopCamera(Node):
    def __init__(self):
        super().__init__('go2_loop_camera')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('input_image_topic', '/camera/image/compressed')
        self.declare_parameter('max_sync_error_sec', 0.05)
        self.declare_parameter('max_image_age_sec', 1.0)
        self.declare_parameter('max_future_sec', 0.05)
        value = lambda key: self.get_parameter(key).value
        self.calibration = load_calibration(value('calibration_file'), value('max_sync_error_sec'))
        self.max_age = float(value('max_image_age_sec'))
        self.max_future = float(value('max_future_sec'))
        if not math.isfinite(self.max_age) or not math.isfinite(self.max_future) or self.max_age <= 0 or self.max_future < 0:
            raise ValueError('image freshness limits must be finite, age positive and future >=0')
        self.rectifier = CameraRectifier(self.calibration)
        self.images = self.create_publisher(Image, '/camera/loop/image_rect', 5)
        self.infos = self.create_publisher(CameraInfo, '/camera/loop/camera_info', 5)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/mapping/camera_status', status_qos)
        self.status = {'state': 'waiting_for_image', 'received': 0, 'published': 0, 'rejected': 0,
                       'last_error': None, 'timing_mode': self.calibration.timing_mode,
                       'timing_uncertainty_sec': self.calibration.uncertainty_sec,
                       'stamp_offset_sec': self.calibration.stamp_offset_sec,
                       'timing_source': self.calibration.timing_source,
                       'calibration_source': self.calibration.calibration_source,
                       'extrinsics_source': self.calibration.extrinsics_source}
        self.last_accepted_monotonic = None
        self.last_stamp_ns = None
        self.subscription = self.create_subscription(CompressedImage, value('input_image_topic'), self.on_image, qos_profile_sensor_data)
        validate_input_topic(self.calibration, self.subscription.topic_name)
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = self.calibration.optical_frame
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = self.calibration.translation
        q.x, q.y, q.z, q.w = self.calibration.quaternion_xyzw
        self.static_tf.sendTransform(transform)
        self.create_timer(1., self.publish_status)
        self.get_logger().info('Calibrated camera adapter enabled; input=%s, timing=%s, uncertainty=%.4fs' %
                               (self.subscription.topic_name, self.calibration.timing_mode, self.calibration.uncertainty_sec))
        self.publish_status()

    def publish_status(self):
        status = dict(self.status)
        age = None if self.last_accepted_monotonic is None else time.monotonic() - self.last_accepted_monotonic
        status['receipt_age_sec'] = age
        status['stale'] = age is None or age > self.max_age
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def on_image(self, message):
        self.status['received'] += 1
        try:
            input_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            stamp_ns, age = corrected_stamp_ns(self.calibration, input_ns, self.get_clock().now().nanoseconds,
                                                self.max_age, self.max_future)
            if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
                raise ValueError('duplicate or out-of-order image timestamp')
            pixels, metrics = self.rectifier.decode_rectify(message.data)
            image = Image()
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            image.header.frame_id = self.calibration.optical_frame
            image.height, image.width = self.calibration.height, self.calibration.width
            image.encoding, image.is_bigendian = 'bgr8', 0
            image.step = image.width * 3
            image.data = pixels.tobytes()
            info = CameraInfo()
            info.header = image.header
            info.height, info.width = image.height, image.width
            # These parameters describe the rectified output, not the input.
            info.distortion_model, info.d = 'plumb_bob', [0.] * 5
            info.k = list(self.calibration.rectified_k)
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = list(self.calibration.p)
            self.infos.publish(info)
            self.images.publish(image)
            self.last_stamp_ns = stamp_ns
            self.last_accepted_monotonic = time.monotonic()
            self.status.update(metrics, state='publishing', last_error=None, image_age_sec=age,
                               last_stamp_ns=stamp_ns, published=self.status['published'] + 1)
        except Exception as error:
            self.status.update(state='image_rejected', last_error=str(error), rejected=self.status['rejected'] + 1)
            self.get_logger().warning('Loop camera rejected image: %s' % error, throttle_duration_sec=5.)
        self.publish_status()


def main():
    rclpy.init()
    node = None
    try:
        node = LoopCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
