#!/usr/bin/env python3
"""Exercise the real RTAB-Map RGB + lidar + external-odometry input path.

Synthetic camera intrinsics and extrinsics belong only to this test scene.
This proves supported ROS interfaces and generated lidar depth, not farm loop
closure, real camera calibration, map seam quality or robot performance.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info, MapData
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml


ROOT = Path(__file__).resolve().parents[1]


def spin_until(node, predicate, seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('Timed out waiting for RTAB-Map ROS condition')


def run():
    spec = importlib.util.spec_from_file_location('go2_rtab_launch',
        ROOT / 'go2_3d/rtabmap_backend.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    node = rclpy.create_node('rtabmap_synthetic_input_probe')
    maps, infos, tf_edges = [], [], []
    node.create_subscription(MapData, '/rtabmap/mapData', maps.append, 20)
    node.create_subscription(Info, '/rtabmap/info', infos.append, 20)
    node.create_subscription(TFMessage, '/tf',
        lambda message: tf_edges.extend((t.header.frame_id, t.child_frame_id)
                                       for t in message.transforms), 100)
    publishers = {
        'odom': node.create_publisher(Odometry, '/rtabmap/input/odom', 20),
        'scan_cloud': node.create_publisher(PointCloud2, '/rtabmap/input/scan_cloud', 20),
        'image': node.create_publisher(Image, '/rtabmap/input/image', 20),
        'camera_info': node.create_publisher(CameraInfo, '/rtabmap/input/camera_info', 20),
    }
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    camera_tf = TransformStamped()
    camera_tf.header.stamp = node.get_clock().now().to_msg()
    camera_tf.header.frame_id, camera_tf.child_frame_id = 'base_link', 'test_camera_optical'
    # Optical x right, y down, z forward in this generated body-frame scene.
    camera_tf.transform.rotation.x = -0.5
    camera_tf.transform.rotation.y = 0.5
    camera_tf.transform.rotation.z = -0.5
    camera_tf.transform.rotation.w = 0.5
    static.sendTransform(camera_tf)
    with tempfile.TemporaryDirectory(prefix='go2-rtabmap-smoke-') as temp:
        directory = Path(temp)
        parameters = module.rtabmap_parameters(str(directory / 'test.db'))
        config = directory / 'params.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        logfile = directory / 'rtabmap.log'
        command = ['ros2', 'run', 'rtabmap_slam', 'rtabmap', '--ros-args',
            '-r', '__ns:=/rtabmap', '-r', '__node:=rtabmap',
            '--params-file', str(config),
            '-r', 'rgb/image:=/rtabmap/input/image',
            '-r', 'rgb/camera_info:=/rtabmap/input/camera_info',
            '-r', 'scan_cloud:=/rtabmap/input/scan_cloud',
            '-r', 'odom:=/rtabmap/input/odom']
        with logfile.open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        try:
            spin_until(node, lambda: all(p.get_subscription_count() > 0
                                        for p in publishers.values()), 30)
            # A dense synthetic front wall supplies metric depth at image
            # features. A fixed random world texture is sampled in each view.
            width, height, focal = 320, 240, 220.0
            u, v = np.meshgrid(np.arange(width), np.arange(height))
            rng = np.random.default_rng(381)
            texture = rng.integers(0, 256, (1200, 1600, 3), dtype=np.uint8)
            sent_stamps = []
            for index in range(6):
                stamp = node.get_clock().now().to_msg()
                sent_stamps.append(stamp.sec + stamp.nanosec * 1e-9)
                position = 0.1 + index * 0.08
                depth = 5.0 - position
                optical_x = (u - width / 2) * depth / focal
                optical_y = (v - height / 2) * depth / focal
                world_y, world_z = -optical_x, -optical_y
                rows = np.clip(np.rint(600 + world_z * 170), 0, 1199).astype(int)
                cols = np.clip(np.rint(800 + world_y * 170), 0, 1599).astype(int)
                image = texture[rows, cols].copy()
                cloud = np.ascontiguousarray(np.column_stack((
                    np.full(u.size, depth), world_y.ravel(), world_z.ravel())), dtype='<f4')
                header = Header(stamp=stamp, frame_id='test_camera_optical')
                image_message = Image(header=header, height=height, width=width,
                    encoding='rgb8', is_bigendian=0, step=width * 3, data=image.tobytes())
                info = CameraInfo(header=header, height=height, width=width,
                    distortion_model='plumb_bob', d=[0.] * 5,
                    k=[focal, 0., width / 2, 0., focal, height / 2, 0., 0., 1.],
                    r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                    p=[focal, 0., width / 2, 0., 0., focal, height / 2, 0., 0., 0., 1., 0.])
                scan = PointCloud2(header=Header(stamp=stamp, frame_id='base_link'),
                    height=1, width=len(cloud), is_bigendian=False,
                    fields=[PointField(name=n, offset=4 * i, datatype=7, count=1)
                            for i, n in enumerate('xyz')],
                    point_step=12, row_step=len(cloud) * 12, is_dense=True,
                    data=cloud.tobytes())
                odom = Odometry(header=Header(stamp=stamp, frame_id='gicp_odom'),
                                child_frame_id='base_link')
                odom.pose.pose.position.x = position
                odom.pose.pose.orientation.w = 1.0
                for axis in range(6):
                    odom.pose.covariance[axis * 7] = 0.01
                transform = TransformStamped(header=odom.header, child_frame_id='base_link')
                transform.transform.translation.x = position
                transform.transform.rotation.w = 1.0
                dynamic.sendTransform(transform)
                # Give the independent process its timestamped TF before data.
                finish = time.monotonic() + 0.1
                while time.monotonic() < finish:
                    rclpy.spin_once(node, timeout_sec=0.01)
                for name, message in (('odom', odom), ('scan_cloud', scan),
                                      ('camera_info', info), ('image', image_message)):
                    publishers[name].publish(message)
                spin_until(node, lambda: any(abs(m.header.stamp.sec +
                    m.header.stamp.nanosec * 1e-9 - sent_stamps[-1]) < 1e-5 for m in infos), 20)
            assert maps, 'Real RTAB-Map did not publish MapData'
            final = maps[-1]
            assert len(final.graph.poses_id) >= 3, final.graph.poses_id
            nodes = {entry.id: entry for message in maps for entry in message.nodes}
            assert len(nodes) >= 3, list(nodes)
            depths = sum(sum(math.isfinite(p.z) and abs(p.z) > 1e-4
                             for p in entry.word_pts) for entry in nodes.values())
            assert depths > 20, f'No useful lidar-derived metric feature depths: {depths}'
            assert not any(parent == 'map' for parent, child in tf_edges), tf_edges
            # MapData must retain original GICP odometry poses, allowing the
            # map adapter to associate optimized nodes with its scan journal.
            for entry in nodes.values():
                closest = int(np.argmin(np.abs(np.asarray(sent_stamps) - entry.stamp)))
                assert abs(sent_stamps[closest] - entry.stamp) < 1e-5
                assert abs(entry.pose.position.x - (0.1 + closest * 0.08)) < 1e-4
            print(json.dumps(dict(result='PASS', synthetic_only=True,
                package_versions={name: ET.parse(Path(get_package_share_directory(name)) /
                    'package.xml').getroot().findtext('version')
                    for name in ('rtabmap_slam', 'rtabmap_msgs')},
                infos=len(infos), graph_nodes=len(final.graph.poses_id),
                graph_links=len(final.graph.links), metric_feature_depths=depths,
                rtabmap_map_tf_count=0, loop_closure_claimed=False), sort_keys=True))
        except Exception:
            print(logfile.read_text(), file=sys.stderr)
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
