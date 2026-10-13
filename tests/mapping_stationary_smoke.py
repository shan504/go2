"""Real ROS/Open3D: stationary match noise must not move the mapping frame."""
import tempfile
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros_smoke import Mapper, Sensors, wait_for
from geometry import pose_matrix
from rclpy.time import Time
from ros_smoke import transform_matrix


def run(dense=False):
    with tempfile.TemporaryDirectory() as tmp:
        args = ['--ros-args', '-p', 'mode:=mapping', '-p', f'output_directory:={tmp}']
        if dense:
            args += ['--params-file', str(Path(__file__).resolve().parents[1]/'go2_3d/gicp_dense.yaml')]
        rclpy.init(args=args)
        mapper, sensors = Mapper(), Sensors()
        mapper.registration_timer.cancel()
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (mapper, sensors):
            executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        sensors.emitter.start()

        def match(truth, odom):
            sensors.truth, sensors.odom = truth, odom
            wait_for(lambda: mapper.latest is not None and np.allclose(mapper.latest[1], odom))
            # Fill the actual timestamp-aligned window with the new scan.
            time.sleep(mapper.p['scan_window']+0.22)
            mapper.process()

        try:
            identity = np.eye(4)
            match(identity, identity)
            assert mapper.seeded and mapper.last_accepted is not None
            anchor = mapper.correction.copy()
            raw_measurements = []
            for offset in (0.024, -0.024, 0.024, -0.024):
                # Controlled scan perturbations produce actual GICP solutions,
                # rather than replacing align() with a synthetic success.
                match(pose_matrix([0., offset, 0.], [0., 0., 0., 1.]), identity)
                assert mapper.last_accepted is not None, mapper.last_failure
                raw_measurements.append(mapper.accepted_match_correction[1, 3])
                np.testing.assert_allclose(mapper.correction, anchor, atol=1e-9,
                                           err_msg='An accepted stationary scan moved map -> odom')
            assert np.ptp(raw_measurements) > 0.03, raw_measurements
            assert len(mapper.points) > 100, 'Stationary scans stopped map accumulation'
            wait_for(lambda: sensors.buffer.can_transform('map', 'odom', Time()))
            actual = transform_matrix(sensors.buffer.lookup_transform('map', 'odom', Time()).transform)
            np.testing.assert_allclose(actual, anchor, atol=1e-9)

            # Real movement with odometry bias must still apply GICP, rather
            # than keeping the stationary anchor or freezing robot motion.
            truth = pose_matrix([0.4, 0., 0.], [0., 0., 0., 1.])
            odom = pose_matrix([0.5, 0., 0.], [0., 0., 0., 1.])
            match(truth, odom)
            assert mapper.last_accepted is not None, mapper.last_failure
            np.testing.assert_allclose(mapper.correction@odom, truth, atol=0.025)
            assert mapper.correction[0, 3] < -0.05, 'Moving GICP correction was frozen'
            # Actual odometry rotation must also leave the stationary gate.
            truth = pose_matrix([0.4, 0., 0.], [0., 0., np.sin(0.3/2), np.cos(0.3/2)])
            odom = pose_matrix([0.5, 0., 0.], [0., 0., np.sin(0.4/2), np.cos(0.4/2)])
            match(truth, odom)
            assert mapper.last_accepted is not None, mapper.last_failure
            np.testing.assert_allclose(mapper.correction@odom, truth, atol=0.025)
            anchor = mapper.correction.copy()
            match(truth, odom)
            np.testing.assert_allclose(mapper.correction, anchor, atol=1e-9)

            # A genuine large stationary contradiction must be rejected, not
            # accepted and concealed behind the held output transform.
            points_before = mapper.points.copy()
            match(pose_matrix([0.55, 0., 0.], [0., 0., np.sin(0.3/2), np.cos(0.3/2)]), odom)
            assert mapper.last_accepted is None
            assert 'Rejected GICP correction jump' in mapper.last_failure, mapper.last_failure
            np.testing.assert_array_equal(mapper.points, points_before)
            np.testing.assert_allclose(mapper.correction, anchor, atol=1e-9)
            match(truth, odom)
            assert mapper.last_accepted is not None, mapper.last_failure
            np.testing.assert_allclose(mapper.correction, anchor, atol=1e-9)
            print(f'PASS mapping/{"dense" if dense else "standard"}: real GICP scan noise '
                  f'{np.ptp(raw_measurements):.3f}m holds the stationary frame; '
                  'walking/turning correct odometry; large jumps reject without corrupting the map', flush=True)
        finally:
            sensors.stop.set()
            sensors.emitter.join(timeout=3)
            executor.shutdown()
            spinner.join(timeout=3)
            mapper.destroy_node()
            sensors.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
    run(dense=True)
