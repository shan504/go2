#!/usr/bin/env python3
"""Real Humble image/CameraInfo/TF smoke; synthetic calibration, no hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import yaml

from test_camera_calibration import synthetic_calibration


ROOT = Path(__file__).resolve().parents[1]


def main():
    rclpy.init()
    observer = Node('camera_adapter_test_observer')
    images, infos, statuses = [], [], []
    observer.create_subscription(Image, '/camera/loop/image_rect', images.append, 10)
    observer.create_subscription(CameraInfo, '/camera/loop/camera_info', infos.append, 10)
    observer.create_subscription(String, '/mapping/camera_status',
        lambda msg: statuses.append(json.loads(msg.data)), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    source = observer.create_publisher(CompressedImage, '/camera/image/compressed', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, observer)
    process = None
    with tempfile.TemporaryDirectory(prefix='go2-camera-test-') as directory:
        directory = Path(directory)
        calibration_path = directory / 'synthetic-camera.yaml'
        calibration_path.write_text(yaml.safe_dump(synthetic_calibration()))
        log_path = directory / 'camera.log'

        def wait_for(predicate, label, timeout=8.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process is not None and process.poll() is not None:
                    raise AssertionError(f'camera exited: {log_path.read_text()}')
                rclpy.spin_once(observer, timeout_sec=.03)
                if predicate():
                    return
            raise AssertionError(f'timed out: {label}; status={statuses[-1:]}; log={log_path.read_text()}')

        def message(payload, offset_ns=0):
            result = CompressedImage()
            stamp = observer.get_clock().now().nanoseconds + offset_ns
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            result.header.frame_id, result.format = 'go2_front_camera', 'png'
            result.data = payload
            return result

        try:
            missing = subprocess.run([sys.executable, str(ROOT / 'go2_3d/rtabmap_camera.py')],
                                     capture_output=True, text=True, timeout=10)
            assert missing.returncode != 0 and 'calibration_file is required' in missing.stderr, missing.stderr
            with log_path.open('w') as log:
                process = subprocess.Popen([sys.executable, str(ROOT / 'go2_3d/rtabmap_camera.py'),
                    '--ros-args', '-p', f'calibration_file:={calibration_path}',
                    '-p', 'max_image_age_sec:=0.5'], stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
            wait_for(lambda: source.get_subscription_count() == 1 and statuses, 'camera startup')
            wait_for(lambda: buffer.can_transform('base_link', 'synthetic_camera_optical', Time()), 'measured static TF')
            transform = buffer.lookup_transform('base_link', 'synthetic_camera_optical', Time())
            assert abs(transform.transform.translation.x - .2) < 1e-10
            assert abs(transform.transform.translation.y - .01) < 1e-10
            assert abs(transform.transform.translation.z - .1) < 1e-10
            assert transform.transform.rotation.w == 1.
            pixels = np.random.default_rng(234).integers(0, 256, (120, 160, 3), dtype=np.uint8)
            payload = cv2.imencode('.png', pixels)[1].tobytes()
            first = message(payload)
            source.publish(first)
            wait_for(lambda: len(images) == 1 and len(infos) == 1 and statuses[-1]['published'] == 1, 'rectified first image')
            out, info = images[0], infos[0]
            first_ns = first.header.stamp.sec * 1_000_000_000 + first.header.stamp.nanosec
            output_ns = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
            assert output_ns == first_ns - 60_000_000
            assert info.header == out.header
            assert out.header.frame_id == 'synthetic_camera_optical'
            assert (out.width, out.height, out.step, out.encoding) == (160, 120, 480, 'bgr8')
            np.testing.assert_array_equal(np.asarray(out.data).reshape(120, 160, 3), pixels)
            np.testing.assert_allclose(info.k, synthetic_calibration()['camera_matrix']['data'])
            np.testing.assert_allclose(info.d, 0.)
            assert info.distortion_model == 'plumb_bob'
            assert statuses[-1]['feature_count'] > 0
            assert statuses[-1]['sharpness_laplacian_variance'] > 0.
            assert statuses[-1]['timing_uncertainty_sec'] == .01
            source.publish(first)
            wait_for(lambda: statuses[-1]['rejected'] == 1, 'duplicate image rejection')
            assert 'duplicate' in statuses[-1]['last_error'] and len(images) == 1
            source.publish(message(payload, -2_000_000_000))
            wait_for(lambda: statuses[-1]['rejected'] == 2, 'stale image rejection')
            assert 'stale/future' in statuses[-1]['last_error']
            source.publish(message(payload, 2_000_000_000))
            wait_for(lambda: statuses[-1]['rejected'] == 3, 'future image rejection')
            assert 'stale/future' in statuses[-1]['last_error']
            source.publish(message(b'bad compressed data'))
            wait_for(lambda: statuses[-1]['rejected'] == 4, 'malformed image rejection')
            assert 'cannot be decoded' in statuses[-1]['last_error']
            wrong_resolution = cv2.imencode('.png', np.zeros((60, 80, 3), dtype=np.uint8))[1].tobytes()
            source.publish(message(wrong_resolution))
            wait_for(lambda: statuses[-1]['rejected'] == 5, 'resolution mismatch rejection')
            assert 'do not match calibration' in statuses[-1]['last_error']
            assert len(images) == 1
            source.publish(message(payload))
            wait_for(lambda: len(images) == 2 and len(infos) == 2 and statuses[-1]['published'] == 2, 'valid stream recovery')
            assert statuses[-1]['last_error'] is None
            wait_for(lambda: statuses[-1]['stale'] and statuses[-1]['receipt_age_sec'] > .5, 'no-image staleness diagnostic')
            print('PASS ROS2 Humble calibrated camera: measured TF, corrected stamps, rectified Image/CameraInfo, quality diagnostics, duplicate/stale/future/malformed/resolution rejection, recovery; synthetic calibration only')
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            listener.unregister()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
