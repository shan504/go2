"""Real ROS/GICP angular innovation during turns; no robot or motion writes."""
import threading
import tempfile
import time
import re
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

from ros_smoke import Mapper, Sensors, wait_for
from geometry import pose_matrix, quaternion_from_matrix, transform_points
from mapper import transform_matrix
from test_3d import room


def yaw(degrees, translation=(0.0, 0.0, 0.0)):
    angle = np.deg2rad(degrees) / 2
    return pose_matrix(translation, [0.0, 0.0, np.sin(angle), np.cos(angle)])


class TurnSensors(Sensors):
    def __init__(self):
        super().__init__()
        self.pose_lock = threading.Lock()

    def emit(self):
        while not self.stop.is_set():
            with self.pose_lock:
                truth, odom = self.truth.copy(), self.odom.copy()
                points = self.points[self.sampling.choice(len(self.points), 1500, replace=False)]
                points = points + self.sampling.normal(0.0, 0.003, points.shape)
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
            transform = TransformStamped(header=header, child_frame_id='base_link')
            t, q = odom[:3, 3], quaternion_from_matrix(odom)
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, t)
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
            self.tf.sendTransform(transform)
            self.stop.wait(0.03)
            body_points = transform_points(points, np.linalg.inv(truth))
            self.pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=header.stamp, frame_id='base_link'), body_points.astype(np.float32).tolist()))
            self.stop.wait(0.03)


def check(directory, base, widened):
    arguments = ['--ros-args', '-p', 'mode:=localization', '-p', f'map_directory:={directory}',
                 '-p', 'scan_window:=0.5']
    if not widened:
        arguments += ['-p', 'localization_correspondence:=0.30',
                      '-p', 'localization_max_translation:=0.60',
                      '-p', 'localization_max_rotation_degrees:=20.0']
    rclpy.init(args=arguments)
    mapper, sensors = Mapper(), TurnSensors()
    sensors.points = np.asarray(o3d.io.read_point_cloud(str(directory / 'map.pcd')).points).copy()
    sensors.truth = base.copy()
    valid_events = []
    sensors.create_subscription(Bool, '/localization/valid', lambda message: valid_events.append(message.data), 10)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mapper, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()

    def next_match():
        def fresh():
            with mapper.lock:
                if mapper.latest is None:
                    return False
                stamp = mapper.latest[0].header.stamp
                return (stamp.sec, stamp.nanosec) != mapper.processed_stamp
        wait_for(fresh)
        mapper.process()

    def change(truth, native):
        # Renew the current accepted measurement before holding the next
        # window. This keeps the original 1s deadline meaningful throughout.
        next_match()
        reference = mapper.accepted_match_correction.copy()
        with sensors.pose_lock:
            sensors.truth, sensors.odom = truth, native
        wait_for(lambda: mapper.latest is not None and np.allclose(mapper.latest[1], native, atol=1e-6))
        # At the initial 24-degree residual, old/new packets have different
        # correction errors. Match a complete current window, not its smear.
        time.sleep(0.60)
        next_match()
        return reference

    try:
        wait_for(lambda: mapper.seeded and sensors.valid is True, seconds=30)
        mapper.registration_timer.cancel()
        assert mapper.p['max_rotation'] == 0.35 and mapper.p['max_translation'] == 0.6
        assert mapper.p['localization_min_fitness'] == 0.30 and mapper.p['localization_max_rmse'] == 0.15
        assert mapper.p['stationary_translation_limit'] == 0.25
        assert mapper.p['stationary_rotation_limit_degrees'] == 8.0
        assert mapper.p['validity_timeout'] == 1.0 and mapper.p['correction_alpha'] == 0.25
        if widened:
            assert mapper.p['localization_correspondence'] == 0.50
            assert mapper.p['localization_max_translation'] == 0.80
            assert mapper.p['localization_max_rotation_degrees'] == 35.0
        beginning = len(valid_events)
        previous = change(base @ yaw(30.0), yaw(6.0))
        if not widened:
            assert mapper.last_accepted is None, 'Old 20-degree innovation gate accepted a 24-degree residual'
            assert mapper.last_failure.startswith('Fine GICP innovation rejected'), mapper.last_failure
            quality = re.search(r'fitness=([0-9.]+) rmse=([0-9.]+)', mapper.last_failure)
            assert quality and float(quality[1]) >= 0.30 and float(quality[2]) <= 0.15, mapper.last_failure
            np.testing.assert_array_equal(mapper.accepted_match_correction, previous)
            print('PASS old localization bounds: actual quality-passing 24deg GICP innovation is rejected by 20deg gate', flush=True)
            return
        assert mapper.last_accepted is not None, mapper.last_failure
        np.testing.assert_allclose(mapper.accepted_match_correction @ yaw(6.0), base @ yaw(30.0), atol=0.025)
        for degrees in (60.0, 90.0, 120.0, 150.0, 180.0):
            change(base @ yaw(degrees), yaw(degrees - 24.0))
            assert mapper.last_accepted is not None, mapper.last_failure
            np.testing.assert_allclose(mapper.accepted_match_correction @ mapper.accepted_odom,
                                       base @ yaw(degrees), atol=0.025)
        wait_for(lambda: len(valid_events) > beginning)
        assert all(valid_events[beginning:]), 'Measured moving turns interrupted fresh localization'
        print(f'PASS {directory.name}: real 24deg odometry/GICP residual accepted; cumulative90/180deg turns remain valid', flush=True)

        # The output filter remains unchanged. Let fresh accepted matches
        # settle it before comparing full TF to the final physical pose.
        for _ in range(12):
            next_match()
            assert mapper.last_accepted is not None, mapper.last_failure
        body = transform_matrix(sensors.buffer.lookup_transform('map', 'base_link', Time()).transform)
        np.testing.assert_allclose(body, base @ yaw(180.0), atol=0.025)
        native = yaw(156.0)
        for contradiction in (base @ yaw(190.0), base @ yaw(180.0, (0.28, 0.0, 0.0))):
            reference = change(contradiction, native)
            assert mapper.last_accepted is None, 'Wider moving bounds admitted a contradictory stationary pose'
            assert 'Rejected GICP correction jump while odometry stationary' in mapper.last_failure, mapper.last_failure
            np.testing.assert_array_equal(mapper.accepted_match_correction, reference)
            mapper.health()
            wait_for(lambda: sensors.valid is False)
            stamp = sensors.correction_stamp()
            time.sleep(0.15)
            assert sensors.correction_stamp() == stamp, 'Rejected stationary pose renewed map TF'
            with sensors.pose_lock:
                sensors.truth = base @ yaw(180.0)
            time.sleep(0.60)
            next_match()
            assert mapper.last_accepted is not None, mapper.last_failure
            wait_for(lambda: sensors.valid is True)
        print(f'PASS {directory.name}: stationary10deg/28cm contradictions still invalidate; accepted reference and recovery preserved', flush=True)
    finally:
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        executor.shutdown()
        spinner.join(timeout=3)
        mapper.destroy_node()
        sensors.destroy_node()
        rclpy.shutdown()


def main():
    with tempfile.TemporaryDirectory() as tmp:
        points = room()
        points[:, :2] *= 0.30
        for name, base in (('raw-room', np.eye(4)), ('aligned-room', yaw(17.0, (1.3, -0.8, 0.3)))):
            directory = Path(tmp) / name
            directory.mkdir()
            world_points = transform_points(points, base)
            assert o3d.io.write_point_cloud(str(directory / 'map.pcd'),
                o3d.geometry.PointCloud(o3d.utility.Vector3dVector(world_points)), compressed=True)
            plane = np.linalg.inv(base).T @ [0.0, 0.0, 1.0, 0.3]
            (directory / 'metadata.yaml').write_text(yaml.safe_dump(dict(
                ground_model=dict(plane=plane.tolist()), map_start_pose=base.tolist())))
            original = (directory / 'map.pcd').read_bytes()
            if name == 'raw-room':
                check(directory, base, widened=False)
            check(directory, base, widened=True)
            assert (directory / 'map.pcd').read_bytes() == original
        print('PASS scoped global turn bounds: raw/aligned maps; unchanged mapping, quality, freshness and stationary limits', flush=True)


if __name__ == '__main__':
    main()
