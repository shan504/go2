"""Synthetic ROS regression for complete saved-map display process isolation.

No robot connection or motion publisher. Real saved PCD payloads are checked
separately from the real Mapper health/TF timing test, so decoding a 61 MB test
message cannot itself block the timing observer.
"""
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'go2_3d'))
from saved_map_cloud import SavedMapCloud
from ros_smoke import Mapper, Sensors, wait_for
from test_3d import room


def save_map(directory, points):
    directory.mkdir(parents=True)
    assert o3d.io.write_point_cloud(str(directory/'map.pcd'),
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
    (directory/'metadata.yaml').write_text(yaml.safe_dump(
        dict(ground_model=dict(plane=[0., 0., 1., 0.3]))))
    return hashlib.sha256((directory/'map.pcd').read_bytes()).hexdigest()


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def publisher(directory, log, slow=False, frame='map', period=1.0):
    script = Path(__file__) if slow else ROOT/'go2_3d/saved_map_cloud.py'
    command = [sys.executable, str(script)]
    if slow:
        command += ['--slow-publisher']
    command += ['--ros-args', '-p', f'map_directory:={directory}',
                '-p', f'map_frame:={frame}', '-p', f'map_publish_period:={period}']
    return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)


def geometry(directory, expected, log_path):
    rclpy.init()
    probe = Node('saved_map_geometry_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    samples = []
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    with log_path.open('w') as log:
        process = publisher(directory, log, frame='saved_map_frame', period=5.0)
        try:
            # Attach after the publisher starts; retained delivery is required.
            deadline = time.monotonic()+20
            while ('Saved map display ready:' not in log_path.read_text() or
                   not probe.get_publishers_info_by_topic('/map_cloud')):
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                executor.spin_once(timeout_sec=.05)
            endpoints = probe.get_publishers_info_by_topic('/map_cloud')
            assert len(endpoints) == 1
            assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert endpoints[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            subscribed_at = probe.get_clock().now()
            probe.create_subscription(PointCloud2, '/map_cloud', samples.append, retained)
            while len(samples) < 2:
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                executor.spin_once(timeout_sec=.05)
            assert Time.from_msg(samples[0].header.stamp) < subscribed_at, 'Initial retained map not delivered'
            for sample in samples[-2:]:
                assert sample.header.frame_id == 'saved_map_frame'
                assert sample.width == len(expected) and sample.height == 1
                assert sample.point_step == 12 and sample.row_step == len(expected)*12
                assert [(f.name, f.offset) for f in sample.fields] == [('x', 0), ('y', 4), ('z', 8)]
                np.testing.assert_array_equal(
                    np.frombuffer(sample.data, dtype='<f4').reshape(-1, 3), expected.astype('<f4'))
            assert Time.from_msg(samples[-1].header.stamp) > Time.from_msg(samples[-2].header.stamp)
            publications = {name for name, _ in probe.get_publisher_names_and_types_by_node(
                'go2_saved_map_cloud', '/')}
            assert '/map_cloud' in publications
            assert not publications & {'/cmd_vel', '/cmd_vel_nav', '/control/enable', '/goal_pose', '/tf'}
            print(f'PASS saved map {directory}: {len(expected)} complete XYZ points, '
                  'frame, repeated timestamped publication and no motion/TF publishers', flush=True)
        finally:
            stop(process)
            executor.shutdown()
            probe.destroy_node()
            rclpy.shutdown()


def isolation(directory, display_directory, log_path):
    rclpy.init(args=['--ros-args', '-p', 'mode:=localization',
                    '-p', f'map_directory:={directory}', '-p', 'scan_window:=0.0',
                    '-p', 'publish_map_cloud:=false'])
    mapper, sensors = Mapper(), Sensors()
    sensors.packet_wait = .02
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mapper)
    executor.add_node(sensors)
    health, transforms, invalid = [], [], []
    observing = threading.Event()

    def validity(message):
        if observing.is_set():
            health.append(time.monotonic())
            if not message.data:
                invalid.append(time.monotonic())

    def transform(message):
        if observing.is_set() and any(t.header.frame_id == 'map' and t.child_frame_id == 'odom'
                                     for t in message.transforms):
            transforms.append(time.monotonic())

    sensors.create_subscription(Bool, '/localization/valid', validity, 10)
    sensors.create_subscription(TFMessage, '/tf', transform, 100)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()
    with log_path.open('w') as log:
        process = None
        try:
            wait_for(lambda: sensors.valid is True and sensors.buffer.can_transform(
                'map', 'base_footprint', Time()), seconds=25)
            assert mapper.seeded and mapper.last_accepted is not None
            assert not mapper.p['publish_map_cloud']
            observing.set()
            process = publisher(display_directory, log, slow=True, period=1.5)
            wait_for(lambda: log_path.read_text().count('SLOW_DISPLAY_END') >= 3, seconds=25)
            assert process.poll() is None, log_path.read_text()
            observing.clear()
            assert len(health) >= 20 and len(transforms) >= 30
            health_gap, tf_gap = float(np.diff(health).max()), float(np.diff(transforms).max())
            assert not invalid, f'Localization became invalid during display blocking: {invalid}'
            assert health_gap < .3 and tf_gap < .3, (health_gap, tf_gap)
            print('PASS separate 5,077,530-point display process with three deliberate 0.8s blocking callbacks: '
                  f'real accepted GICP and Mapper validity remain active; '
                  f'max health gap={health_gap:.3f}s, TF gap={tf_gap:.3f}s', flush=True)
        finally:
            if process is not None:
                stop(process)
            sensors.stop.set()
            sensors.emitter.join(timeout=3)
            executor.shutdown()
            spinner.join(timeout=3)
            mapper.destroy_node()
            sensors.destroy_node()
            rclpy.shutdown()


def slow_publisher():
    # Test-only blocking publication; production has no delay or test switch.
    class SlowDisplay(SavedMapCloud):
        def publish_map(self):
            before = time.monotonic()
            while time.monotonic()-before < .8:
                sum(range(1000))  # Hold this process's Python interpreter busy.
            super().publish_map()
            print('SLOW_DISPLAY_END '+json.dumps({'elapsed': time.monotonic()-before}), flush=True)
    rclpy.init(args=sys.argv[2:])
    node = SlowDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        small = room()
        small_dir, dense_dir = root/'retained-indoor/version', root/'new-field/version'
        before_small = save_map(small_dir, small)
        dense = np.empty((5_077_530, 3), dtype=np.float64)
        dense[:, 0] = np.linspace(-25, 40, len(dense))
        dense[:, 1] = np.arange(len(dense)) % 103 * .031
        dense[:, 2] = np.arange(len(dense)) % 97 * .017 - .30
        before_dense = save_map(dense_dir, dense)
        geometry(small_dir, small, root/'small.log')
        geometry(dense_dir, dense, root/'dense.log')
        geometry(small_dir, small, root/'reload.log')
        isolation(small_dir, dense_dir, root/'isolated.log')
        assert hashlib.sha256((small_dir/'map.pcd').read_bytes()).hexdigest() == before_small
        assert hashlib.sha256((dense_dir/'map.pcd').read_bytes()).hexdigest() == before_dense
        print('PASS two immutable saved map paths, full 5,077,530-point payload and repeated '
              'load; source PCD files unchanged. Synthetic ROS validation only.', flush=True)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--slow-publisher':
        slow_publisher()
    else:
        main()
