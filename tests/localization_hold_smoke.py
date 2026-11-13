"""Real ROS/GICP: bounded dead reckoning, sensor loss and no false acceptance."""
import tempfile
import threading
import time
from pathlib import Path
import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_smoke import Mapper, Sensors, wait_for
from geometry import pose_matrix
from mapper import transform_matrix
from rclpy.time import Time
from test_3d import room


def main():
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        points = room()
        o3d.io.write_point_cloud(str(directory/'map.pcd'),
                                o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
        (directory/'metadata.yaml').write_text(yaml.safe_dump(
            dict(ground_model=dict(plane=[0., 0., 1., 0.3]))))
        rclpy.init(args=['--ros-args', '-p', 'mode:=localization',
                        '-p', f'map_directory:={tmp}', '-p', 'scan_window:=0.0'])
        mapper, sensors = Mapper(), Sensors()
        sensors.packet_wait = 0.03
        sensors.points = np.zeros((3, 3))
        mapper.registration_timer.cancel()
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (mapper, sensors):
            executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        sensors.emitter.start()

        def match():
            wait_for(lambda: mapper.latest is not None and
                     (mapper.latest[0].header.stamp.sec, mapper.latest[0].header.stamp.nanosec) != mapper.processed_stamp)
            mapper.process()

        def recover():
            changed = time.monotonic()
            sensors.cloud_enabled = sensors.odom_enabled = True
            sensors.truth = sensors.odom = np.eye(4)
            sensors.points = points
            wait_for(lambda: mapper.latest[0].width == len(points) and mapper.latest[2] > changed+0.1)
            match()
            wait_for(lambda: sensors.valid is True)
            assert mapper.last_accepted is not None and mapper.last_accepted > changed, mapper.last_failure

        def bad_scans():
            sensors.points = np.zeros((3, 3))
            wait_for(lambda: mapper.latest[0].width == 3)
            match()

        try:
            wait_for(lambda: mapper.latest is not None)
            match()
            assert not mapper.seeded and mapper.last_accepted is None
            wait_for(lambda: sensors.valid is False)
            print('PASS bootstrap: fresh odometry and bad scans cannot start hold', flush=True)

            recover()
            accepted, reference = mapper.last_accepted, mapper.correction.copy()
            raw = mapper.accepted_match_correction.copy()
            bad_scans()
            # Follow real moving odometry during rejected matches, beyond 1s.
            for x in (0.1, 0.2, 0.3):
                sensors.odom = pose_matrix([x, 0, 0], [0, 0, 0, 1])
                time.sleep(0.4)
                match()
                assert mapper.last_accepted == accepted
                np.testing.assert_array_equal(mapper.correction, reference)
                np.testing.assert_array_equal(mapper.accepted_match_correction, raw)
                wait_for(lambda: sensors.valid is True)
            assert mapper.localization_state == 'odometry_hold'
            wait_for(lambda: sensors.buffer.can_transform('map', 'base_link', Time()))
            body = transform_matrix(sensors.buffer.lookup_transform('map', 'base_link', Time()).transform)
            np.testing.assert_allclose(body, reference@sensors.odom, atol=0.02)
            assert sensors.correction_stamp() <= sensors.get_clock().now().nanoseconds
            wait_for(lambda: mapper.localization_state == 'registration_timeout', seconds=4)
            wait_for(lambda: sensors.valid is False)
            stamp = sensors.correction_stamp()
            time.sleep(0.2)
            assert sensors.correction_stamp() == stamp
            assert mapper.last_accepted == accepted
            print('PASS rejected scans: >1s real-odom TF hold; original 3s deadline stops validity/TF without renewing accepted pose/time', flush=True)

            recover()
            bad_scans()
            # Both outward and return motion consume the distance budget.
            sensors.odom = pose_matrix([0.6, 0, 0], [0, 0, 0, 1])
            wait_for(lambda: mapper.odom_budget.distance >= 0.59)
            sensors.odom = np.eye(4)
            wait_for(lambda: mapper.localization_state == 'odometry_distance_limit')
            assert mapper.odom_budget.distance >= 1.19
            time.sleep(0.15)
            assert mapper.localization_state == 'odometry_distance_limit'
            print('PASS cumulative 1m travel budget: a round trip cannot replenish it', flush=True)

            recover()
            bad_scans()
            angle = np.deg2rad(40)/2
            sensors.odom = pose_matrix([0, 0, 0], [0, 0, np.sin(angle), np.cos(angle)])
            wait_for(lambda: mapper.odom_budget.angle >= np.deg2rad(39))
            sensors.odom = np.eye(4)
            wait_for(lambda: mapper.localization_state == 'odometry_angle_limit')
            assert mapper.odom_budget.angle >= np.deg2rad(79)
            print('PASS cumulative 60deg turn budget: opposite turns cannot replenish it', flush=True)

            recover()
            sensors.odom_enabled = False
            wait_for(lambda: mapper.localization_state == 'odometry_timeout', seconds=2)
            wait_for(lambda: sensors.valid is False)
            assert time.monotonic()-mapper.last_accepted < 2
            recover()
            sensors.cloud_enabled = False
            wait_for(lambda: mapper.localization_state == 'cloud_timeout', seconds=2)
            wait_for(lambda: sensors.valid is False)
            assert time.monotonic()-mapper.last_accepted < 2
            print('PASS sensor loss: stale odometry/cloud stops before hold deadline', flush=True)

            recover()
            hint = PoseWithCovarianceStamped()
            hint.header.frame_id = 'map'
            hint.pose.pose.orientation.w = 1.0
            mapper.initial_pose(hint)
            mapper.health()
            assert not mapper.seeded and mapper.last_accepted is None
            assert mapper.localization_state == 'registration_invalid'
            print('PASS explicit reseed revokes prior hold until a new qualified match', flush=True)
        finally:
            sensors.stop.set()
            sensors.emitter.join(timeout=3)
            executor.shutdown()
            spinner.join(timeout=3)
            mapper.destroy_node()
            sensors.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
