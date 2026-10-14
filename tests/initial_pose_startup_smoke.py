"""Real ROS early manual-pose delivery, exact-stamp sensors and GICP startup.

Synthetic scans exercise two retained raw/canonical bundles and no motion
publisher. Lifecycle calls go to a recorder, not a real Nav2 stack or robot.
"""
import copy
import shutil
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

from ros_smoke import Sensors, wait_for
from mapper import Mapper, transform_matrix
from nav2_ready import Nav2Ready
from geometry import ObservedGrid, planar_pose, pose_matrix, quaternion_from_matrix, transform_points
from ground import body_seed_position
from map_library import fingerprints
from navigation_grid import build_bundle
from test_3d import room


class StartupSensors(Sensors):
    def __init__(self):
        super().__init__()
        self.data_enabled = False
        self.observed_hints = []
        self.valid_events = []
        self.lifecycle_calls = []
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.observed_hints.append, 10)
        self.create_subscription(Bool, '/localization/valid',
            lambda message: self.valid_events.append(message.data), 10)
        self.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', self.startup)

    def startup(self, request, response):
        self.lifecycle_calls.append(request.command)
        response.success = True
        return response

    def packet(self):
        points = transform_points(self.points, np.linalg.inv(self.truth))
        return point_cloud2.create_cloud_xyz32(
            Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link'),
            points.astype(np.float32).tolist())

    def odometry_tf(self, stamp):
        transform = TransformStamped(header=Header(stamp=stamp, frame_id='odom'), child_frame_id='base_link')
        t, q = self.odom[:3, 3], quaternion_from_matrix(self.odom)
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, t)
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
        self.tf.sendTransform(transform)

    def emit(self):
        while not self.stop.is_set():
            if self.data_enabled:
                packet = self.packet()
                self.odometry_tf(packet.header.stamp)
                self.pub.publish(packet)
            self.stop.wait(0.10)


def hint(matrix):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    projected = planar_pose(matrix)
    p, q = message.pose.pose.position, message.pose.pose.orientation
    p.x, p.y = map(float, projected[:2, 3])
    q.x, q.y, q.z, q.w = map(float, quaternion_from_matrix(projected))
    return message


def check(directory, truth, expire=False):
    metadata = yaml.safe_load((directory / 'metadata.yaml').read_text())
    rclpy.init(args=['--ros-args', '-p', 'mode:=localization', '-p', f'map_directory:={directory}',
                    '-p', 'auto_initialize:=true', '-p', 'initial_from_map_start:=false',
                    '-p', 'initial_x:=12.0', '-p', 'initial_y:=-11.0',
                    '-p', 'initial_yaw_degrees:=-120.0', '-p', 'scan_window:=0.5'])
    mapper, gate, sensors = Mapper(), Nav2Ready(), StartupSensors()
    # Holding registration separates receiving a hint from accepting a real
    # GICP match. All sensor, TF, health and lifecycle delivery remains real ROS.
    mapper.registration_timer.cancel()
    sensors.points = np.asarray(o3d.io.read_point_cloud(str(directory / 'map.pcd')).points).copy()
    sensors.truth = truth
    sensors.odom = pose_matrix([2.3, -1.7, 0.12], [0.04, -0.025, 0.20, 0.97])
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (mapper, gate, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()
    expected_hint = hint(truth)
    expected_seed = pose_matrix(
        [truth[0, 3], truth[1, 3], metadata['body_height_above_ground']],
        quaternion_from_matrix(planar_pose(truth)))

    def publish(message):
        received = len(sensors.observed_hints)
        sensors.initial.publish(message)
        wait_for(lambda: len(sensors.observed_hints) == received + 1)
        # The independent observer confirms DDS delivery; let Mapper's
        # serialized callback consume the same message before the next input.
        time.sleep(0.10)

    def map_stamp():
        # Inspect the correction edge directly. After the 10s hint expiry,
        # native odom history may no longer overlap its old timestamp.
        return Time.from_msg(sensors.buffer.lookup_transform('map', 'odom', Time()).header.stamp).nanoseconds

    def accepted_pose():
        body = transform_matrix(sensors.buffer.lookup_transform('map', 'base_link', Time()).transform)
        footprint = transform_matrix(sensors.buffer.lookup_transform('map', 'base_footprint', Time()).transform)
        np.testing.assert_allclose(body, truth, atol=0.025)
        np.testing.assert_allclose(footprint, planar_pose(truth), atol=0.025)
        assert abs(body[2, 3] - metadata['body_height_above_ground']) < 0.025
        assert abs(footprint[2, 3]) < 0.003

    try:
        wait_for(lambda: sensors.initial.get_subscription_count() >= 2 and sensors.valid is False and
                 gate.client.service_is_ready())
        first = copy.deepcopy(expected_hint)
        first.pose.pose.position.x -= 0.2
        first.pose.pose.position.y += 0.1
        publish(first)
        publish(expected_hint)
        wrong_frame = copy.deepcopy(expected_hint)
        wrong_frame.header.frame_id = 'base_link'
        wrong_frame.pose.pose.position.x = 20.0
        publish(wrong_frame)
        invalid = copy.deepcopy(expected_hint)
        invalid.pose.pose.position.y = 20.0
        invalid.pose.pose.orientation.x = invalid.pose.pose.orientation.y = 0.0
        invalid.pose.pose.orientation.z = invalid.pose.pose.orientation.w = 0.0
        publish(invalid)
        assert mapper.latest is None and not mapper.initialized
        assert not any(sensors.valid_events) and not sensors.lifecycle_calls
        assert not sensors.buffer.can_transform('map', 'base_footprint', Time())

        packet = sensors.packet()
        sensors.pub.publish(packet)
        wait_for(lambda: len(mapper.pending_clouds) == 1)
        time.sleep(0.10)
        assert mapper.latest is None and not mapper.initialized
        assert sensors.valid is False and not sensors.lifecycle_calls
        sensors.odometry_tf(packet.header.stamp)
        wait_for(lambda: mapper.latest is not None and mapper.initialized)
        # This assertion uses the resulting user pose, not merely the new
        # pending-hint attribute. The old code silently uses the CLI fallback.
        np.testing.assert_allclose(mapper.correction @ mapper.latest[1], expected_seed, atol=1e-7)
        assert mapper.initial_hint_used and not mapper.seeded and mapper.last_accepted is None
        sensors.data_enabled = True
        wait_for(lambda: len(mapper.history) >= 6)
        assert sensors.valid is False and not any(sensors.valid_events) and not sensors.lifecycle_calls
        assert not sensors.buffer.can_transform('map', 'base_footprint', Time())
        mapper.registration_timer.reset()
        wait_for(lambda: sensors.valid is True and gate.started and
                 sensors.buffer.can_transform('map', 'base_footprint', Time()), seconds=30)
        wait_for(lambda: 'fine=True' in sensors.localization_status)
        accepted_pose()
        assert sensors.lifecycle_calls == [ManageLifecycleNodes.Request.STARTUP]

        # Invalid inputs also leave an existing accepted localization intact.
        wait_for(lambda: mapper.last_accepted is not None and time.monotonic() - mapper.last_accepted < 0.35)
        mapper.registration_timer.cancel()
        generation = mapper.generation
        publish(invalid)
        assert mapper.generation == generation and mapper.seeded and sensors.valid is True
        publish(expected_hint)
        wait_for(lambda: sensors.valid is False)
        assert mapper.generation == generation + 1 and not mapper.seeded and mapper.last_accepted is None
        time.sleep(0.10)
        stamp = map_stamp()
        time.sleep(0.25)
        assert map_stamp() == stamp, 'Reseeding renewed map TF before GICP acceptance'
        assert sensors.valid is False and len(sensors.lifecycle_calls) == 1
        mapper.registration_timer.reset()
        wait_for(lambda: sensors.valid is True and mapper.seeded)
        accepted_pose()

        if expire:
            sensors.data_enabled = False
            wait_for(lambda: mapper.latest is not None and time.monotonic() - mapper.latest[2] > 1.1)
            publish(expected_hint)
            wait_for(lambda: getattr(mapper, 'pending_initial_pose', None) is not None)
            wait_for(lambda: mapper.pending_initial_pose is None and
                     'Initial-pose hint expired' in mapper.last_failure, seconds=12)
            stamp = map_stamp()
            sensors.data_enabled = True
            wait_for(lambda: mapper.latest is not None and time.monotonic() - mapper.latest[2] < 0.2)
            time.sleep(0.7)
            assert not mapper.initialized and sensors.valid is False and map_stamp() == stamp
            assert len(sensors.lifecycle_calls) == 1, 'Expired hint allowed navigation startup'
            publish(expected_hint)
            wait_for(lambda: sensors.valid is True and mapper.seeded, seconds=30)
            accepted_pose()
            print('PASS manual hint expiration: 10s bound, no late sensor/CLI revival, fresh actual hint recovers', flush=True)
        print(f'PASS manual startup {directory.name}: newest valid pre-data hint; cloud-before-TF; '
              'manual overrides CLI; accepted 3D body/ground footprint; gated lifecycle; safe reseed', flush=True)
    finally:
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        executor.shutdown()
        spinner.join(timeout=3)
        for node in (mapper, gate, sensors):
            node.destroy_node()
        rclpy.shutdown()


def main():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / 'maps'
        source = root / 'raw-room'
        source.mkdir(parents=True)
        raw_transform = pose_matrix([0.0, 0.0, -0.1], [0.025, -0.04, 0.0, 0.999])
        points = transform_points(room(), raw_transform)
        plane = np.linalg.inv(raw_transform).T @ [0.0, 0.0, 1.0, 0.3]
        plane /= np.linalg.norm(plane[:3])
        ground = dict(plane=plane.tolist(), height_at_origin=float(-plane[3] / plane[2]))
        assert o3d.io.write_point_cloud(str(source / 'map.pcd'),
            o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)), compressed=True)
        ObservedGrid(0.1).export(points, source, ground_model=ground)
        (source / 'metadata.yaml').write_text(yaml.safe_dump(dict(ground_model=ground,
            parameters=dict(grid_resolution=0.1, floor_z=-0.4,
                            obstacle_min_height=0.1, obstacle_max_height=1.5))))
        source_before = fingerprints(source)
        runtime_raw = Path(tmp) / 'runtime-from-raw'
        build_bundle(root, source, runtime_raw)
        metadata = yaml.safe_load((runtime_raw / 'metadata.yaml').read_text())
        original_alignment = np.asarray(metadata['map_from_source_map'])
        raw_body = pose_matrix(body_seed_position([0.8, -0.5, 0.0], ground,
            metadata['body_height_above_ground']), [0.02, -0.025, 0.16, 0.986])
        truth = original_alignment @ raw_body
        check(runtime_raw, truth)
        assert fingerprints(source) == source_before

        # Repeated loading must keep the same physical 6D pose although the
        # current alignment metadata becomes identity on canonical reload.
        canonical = root / 'canonical-room'
        shutil.copytree(runtime_raw, canonical)
        canonical_before = fingerprints(canonical)
        runtime_canonical = Path(tmp) / 'runtime-from-canonical'
        build_bundle(root, canonical, runtime_canonical)
        check(runtime_canonical, truth, expire=True)
        assert fingerprints(canonical) == canonical_before and fingerprints(source) == source_before
        print('PASS shared raw/canonical initialization: two retained bundles, same full 3D physical pose, original files preserved', flush=True)


if __name__ == '__main__':
    main()
