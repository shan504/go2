"""ROS/Humble integration with the original Open3D and an isolated KISS child.

Synthetic accepted observations are injected into the mapping journal. This
tests the real backend, map rebuild/save, TF and saved-map GICP loading; it does
not measure sensor-front-end tracking, L1 farm accuracy or robot navigation.
No robot driver, controller, Nav2 action server or motion publisher is started.
"""
import argparse
import hashlib
import json
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'go2_3d'))
sys.path.insert(0, str(ROOT/'tests'))
from geometry import cloud_xyz, pose_matrix, quaternion_from_matrix, transform_points
from kiss_backend_fixture import submap_revisit
from map_library import fingerprints, select
from mapper import Mapper, transform_matrix
from mapping_rebuild import MappingObservation, append_rebuilt
from navigation_grid import prepare
from ros_cloud import xyz_message


def wait_for(predicate, seconds=30):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('Synthetic ROS condition timed out')


class Sensors(Node):
    def __init__(self):
        super().__init__('kiss_synthetic_sensors')
        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cloud = self.create_publisher(PointCloud2, '/point_cloud2', qos_profile_sensor_data)
        self.map_cloud = None
        self.loop_report = None
        self.valid = None
        self.localization_status = ''
        self.fine_seen = False
        self.odom = np.eye(4)
        self.scan = None
        self.enabled = False
        self.create_subscription(PointCloud2, '/map_cloud', lambda m: setattr(self, 'map_cloud', m), 1)
        self.create_subscription(String, '/mapping/loop_closure',
                                 lambda m: setattr(self, 'loop_report', json.loads(m.data)), 1)
        self.create_subscription(Bool, '/localization/valid', lambda m: setattr(self, 'valid', m.data), 1)
        self.create_subscription(String, '/localization/status', self.localization_message, 1)
        self.create_timer(.1, self.emit)

    def localization_message(self, message):
        self.localization_status = message.data
        self.fine_seen = self.fine_seen or 'fine=True' in message.data

    def emit(self):
        if not self.enabled:
            return
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped(header=Header(stamp=stamp, frame_id='odom'), child_frame_id='base_link')
        t, q = self.odom[:3, 3], quaternion_from_matrix(self.odom)
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, t)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
        self.broadcaster.sendTransform(transform)
        if self.scan is not None:
            self.cloud.publish(xyz_message(Header(stamp=stamp, frame_id='base_link'), self.scan))


def spin_nodes(nodes):
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def close_nodes(executor, thread, nodes):
    executor.shutdown()
    thread.join(timeout=5)
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def mapping_phase(directory, python):
    rclpy.init(args=['--ros-args', '-p', 'mode:=mapping', '-p', 'mapping_backend:=kiss',
                    '-p', 'kiss_python:='+python, '-p', 'kiss_submap_max_observations:=2',
                    '-p', 'kiss_submap_distance:=1000.0', '-p', 'kiss_max_iterations:=40',
                    '-p', 'output_directory:='+str(directory), '-p', 'map_name:=kiss-alpha',
                    '-p', 'map_voxel_size:=0.25', '-p', 'grid_resolution:=0.5',
                    '-p', 'map_publish_period:=3600.0'])
    mapper, sensors = Mapper(), Sensors()
    mapper.registration_timer.cancel()
    executor, thread = spin_nodes((mapper, sensors))
    # Nonzero XY/yaw verify the original map anchor is preserved, not reset to I.
    anchor = pose_matrix([8., -2., 0.], [0., 0., np.sin(.1), np.cos(.1)])
    measurements = []
    try:
        assert o3d.__version__.startswith('0.14'), o3d.__version__
        assert not any(name.startswith(('kiss_slam', 'map_closures')) for name in sys.modules)
        assert mapper.loop_worker.backend.status['runtime']['versions']['open3d'] == '0.19.0'
        for index, (points, relative) in enumerate(submap_revisit()):
            points = points.copy()
            points[:, 2] -= .30
            # Keep this artificial scene inside the unchanged grid-cell budget.
            # Preserve the final injected 3 m revisit error.
            if index < 6:
                relative[0, 3] *= .5
            measured = anchor @ relative
            # Six complete submaps, then one final partial submap that only a
            # save-triggered flush can submit for place recognition.
            for _ in range(2 if index < 6 else 1):
                observation = MappingObservation(points, measured)
                measurements.append(measured.copy())
                with mapper.lock:
                    append_rebuilt(mapper.mapping_store, mapper.grid, observation, measured)
                    mapper.mapping_start_pose = anchor.copy()
                    mapper.last_keyframe = measured.copy()
                    mapper.loop_worker.append(observation)
            wait_for(lambda: mapper.loop_worker.ready_for_save() or mapper.loop_worker.error, seconds=60)
            assert not mapper.loop_worker.error, mapper.loop_worker.report()
        assert mapper.loop_worker.applied_loops == 0
        # Real acceptance timestamps must not be refreshed by graph optimization.
        old_acceptance = time.monotonic()-10
        mapper.seeded, mapper.last_accepted = True, old_acceptance
        mapper.accepted_odom = measurements[-1].copy()
        pending = mapper.save_map(Trigger.Request(), Trigger.Response())
        assert not pending.success and 'pending' in pending.message, pending.message
        wait_for(lambda: mapper.loop_worker.ready_for_save() or mapper.loop_worker.error, seconds=120)
        report = mapper.loop_worker.report()
        assert not report['error'], report
        assert report['applied_loops'] == 1, report
        assert report['flush_completed'] == len(measurements)
        assert report['backend']['constraints_added'] == 1
        assert mapper.last_accepted == old_acceptance
        assert mapper.generation == 1
        np.testing.assert_array_equal(mapper.mapping_start_pose, anchor)
        corrected = mapper.loop_worker.observations[-1].pose
        np.testing.assert_allclose((np.linalg.inv(anchor) @ corrected)[:3, 3], [3/7, 0, 0], atol=1e-5)
        expected_delta = corrected @ np.linalg.inv(measurements[-1])
        np.testing.assert_allclose(mapper.correction, expected_delta, atol=1e-8)

        sensors.odom = measurements[-1]
        sensors.enabled = True
        mapper.last_accepted = time.monotonic()  # One explicit synthetic acceptance for TF test.
        wait_for(lambda: sensors.buffer.can_transform('map', 'odom', Time()))
        actual = transform_matrix(sensors.buffer.lookup_transform('map', 'odom', Time()).transform)
        np.testing.assert_allclose(actual, expected_delta, atol=1e-6)
        publishers = sensors.get_publishers_info_by_topic('/tf')
        assert sorted(item.node_name for item in publishers) == ['go2_gicp', 'kiss_synthetic_sensors']
        assert mapper.p['map_frame'] == 'map' and not mapper.external_backend
        mapper.publish_map()
        wait_for(lambda: sensors.map_cloud is not None)
        assert sensors.map_cloud.header.frame_id == 'map'
        np.testing.assert_allclose(cloud_xyz(sensors.map_cloud), mapper.points, atol=2e-5)
        wait_for(lambda: sensors.loop_report is not None and sensors.loop_report['applied_loops'] == 1)
        assert sensors.valid is False, 'Mapping cannot grant navigation authority'
        time.sleep(1.1)
        stamp = Time.from_msg(sensors.buffer.lookup_transform('map', 'odom', Time()).header.stamp).nanoseconds
        time.sleep(.2)
        assert Time.from_msg(sensors.buffer.lookup_transform('map', 'odom', Time()).header.stamp).nanoseconds == stamp

        bundles = {}
        for name in ('kiss-alpha', 'kiss-beta'):
            mapper.p['map_name'] = name
            saved = mapper.save_map(Trigger.Request(), Trigger.Response())
            assert saved.success, saved.message
            selected = (directory/'latest').resolve()
            assert selected.parent.name == name
            metadata = yaml.safe_load((selected/'metadata.yaml').read_text())
            alignment = np.asarray(metadata['map_from_source_map'])
            np.testing.assert_allclose(metadata['map_start_pose'], alignment @ anchor, atol=1e-8)
            assert metadata['loop_closure']['applied_loops'] == 1
            assert metadata['loop_closure']['backend']['upstream_tag'] == 'IROS25'
            assert metadata['ground_aligned'] and .1 < metadata['body_height_above_ground'] < 1.
            points = np.asarray(o3d.io.read_point_cloud(str(selected/'map.pcd')).points)
            np.testing.assert_allclose(points, transform_points(mapper.points, alignment), atol=2e-5)
            assert (selected/'observed_free.npz').is_file()
            bundles[name] = (selected, fingerprints(selected))
        print('PASS KISS ROS mapping: actual detector/ICP/g2o, final-tail flush, full map/cloud/TF '
              'update, immutable freshness, fixed nonzero anchor and two saved map names', flush=True)
        return bundles
    finally:
        close_nodes(executor, thread, (mapper, sensors))


def localization_phase(directory, name, bundle, hashes):
    select(directory, name)
    with tempfile.TemporaryDirectory(prefix='kiss-nav-copy-') as runtime:
        prepare(directory, runtime)
        first = hashlib.sha256((Path(runtime)/'map.pcd').read_bytes()).hexdigest()
        prepare(directory, runtime)
        assert hashlib.sha256((Path(runtime)/'map.pcd').read_bytes()).hexdigest() == first
        metadata = yaml.safe_load((Path(runtime)/'metadata.yaml').read_text())
        start = np.asarray(metadata['map_start_pose'])
        points = np.asarray(o3d.io.read_point_cloud(str(Path(runtime)/'map.pcd')).points)
        rclpy.init(args=['--ros-args', '-p', 'mode:=localization', '-p', 'map_directory:='+runtime,
                        '-p', 'kiss_python:=/nonexistent/not-needed', '-p', 'map_publish_period:=3600.0',
                        '-p', 'submap_radius:=10.0'])
        mapper, sensors = Mapper(), Sensors()
        executor, thread = spin_nodes((mapper, sensors))
        try:
            assert mapper.p['mapping_backend'] == 'lidar' and mapper.loop_worker is None
            assert not any(module.startswith(('kiss_slam', 'map_closures')) for module in sys.modules)
            np.testing.assert_allclose(mapper.saved_start_pose, start, atol=1e-8)
            np.testing.assert_array_equal(mapper.points, points)
            body = transform_points(points, np.linalg.inv(start))
            # This is an interface/coordinate smoke, not a throughput test of
            # the large artificial scene. Restrict its synthetic visible range
            # and matching crop; keep all production acceptance/time gates.
            body = body[np.linalg.norm(body, axis=1) < 8.]
            rng = np.random.default_rng(42)
            if len(body) > 1200:
                body = body[rng.choice(len(body), 1200, replace=False)]
            sensors.scan, sensors.enabled = body, True
            wait_for(lambda: sensors.valid is True and sensors.fine_seen, seconds=60)
            actual = transform_matrix(sensors.buffer.lookup_transform('map', 'odom', Time()).transform)
            np.testing.assert_allclose(actual, start, atol=.07)
            wait_for(lambda: sensors.buffer.can_transform('map', 'base_footprint', Time()))
            footprint = transform_matrix(sensors.buffer.lookup_transform('map', 'base_footprint', Time()).transform)
            assert abs(footprint[2, 3]) < .01
            assert fingerprints(bundle) == hashes, 'Loading modified a retained source bundle'
            print(f'PASS KISS saved-map localization: {name}, repeated grid preparation, '
                  'original Open3D GICP accepted pose and correct footprint, no KISS runtime dependency', flush=True)
        finally:
            close_nodes(executor, thread, (mapper, sensors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kiss-python', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='kiss-ros-mapping-') as temporary:
        directory = Path(temporary)
        bundles = mapping_phase(directory, args.kiss_python)
        for name, (bundle, hashes) in bundles.items():
            localization_phase(directory, name, bundle, hashes)


if __name__ == '__main__':
    main()
