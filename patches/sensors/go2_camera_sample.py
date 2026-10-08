#!/usr/bin/env python3
"""Official Go2 GetImageSample request translated to a ROS CompressedImage.

No motion requests and no guessed decoding of /frontvideostream.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage
from unitree_api.msg import Request, Response
from camera_reply import sample_response


class CameraSample(Node):
    def __init__(self):
        super().__init__("go2_camera_sample")
        self.request_id = None
        self.sent_at = None
        self.received_frames = 0
        self.requests = self.create_publisher(Request, "/api/videohub/request", 1)
        self.images = self.create_publisher(CompressedImage, "/camera/image/compressed", 2)
        self.create_subscription(Response, "/api/videohub/response", self.on_response, qos_profile_sensor_data)
        self.create_timer(0.5, self.poll_camera)
        self.get_logger().info("GetImageSample -> /camera/image/compressed (up to 2 Hz)")

    def poll_camera(self):
        now = time.monotonic()
        if self.request_id is not None:
            if now - self.sent_at < 3.0:
                return
            self.get_logger().warning("videohub sample timeout: no matching response", throttle_duration_sec=5.0)
        request = Request()
        self.request_id = time.monotonic_ns()
        request.header.identity.id = self.request_id
        request.header.identity.api_id = 1001
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = ""
        request.binary = []
        self.sent_at = now
        self.requests.publish(request)

    def on_response(self, msg):
        fmt, payload, error = sample_response(msg, self.request_id)
        if fmt is None:
            return
        self.request_id = None
        self.sent_at = None
        if error is not None:
            self.get_logger().warning(error, throttle_duration_sec=5.0)
            return
        image = CompressedImage()
        image.header.stamp = self.get_clock().now().to_msg()
        # Logical camera frame only; no camera-to-body extrinsic is invented.
        image.header.frame_id = "go2_front_camera"
        image.format = fmt
        image.data = payload
        self.images.publish(image)
        self.received_frames += 1
        if self.received_frames == 1:
            self.get_logger().info("First camera sample published: %d bytes, %s" % (len(payload), fmt))


def main():
    rclpy.init()
    node = CameraSample()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
