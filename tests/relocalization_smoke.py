"""Real ROS/GICP recapture after tracking loss, with synthetic sensor endpoints.

Every positive case first tracks normally, then changes odometry while the
physical scan stays fixed. Normal fine GICP must reject that innovation before
the independent bounded recovery worker can restore accepted tracking. These
checks do not validate a robot, a Nav2 control loop or farm place observability.
"""
import json
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np
import open3d as o3d
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

from ros_smoke import Mapper, Sensors, wait_for, transform_matrix
from geometry import planar_pose, pose_matrix, quaternion_from_matrix
from map_library import fingerprints
from navigation_grid import build_bundle
from test_map_start import saved_raw


def manual_override_race(mapper, sensors, truth):
    """A real completed GICP proposal cannot overwrite a newer manual hint."""
    backend = mapper.recovery_worker.backend
    original = backend.observe
    computed, release, returned = threading.Event(), threading.Event(), threading.Event()

    def delayed(*args, **kwargs):
        result = original(*args, **kwargs)
        if result.accepted:
            computed.set()
            if not release.wait(10):
                raise AssertionError('Manual override test did not release recovery proposal')
            returned.set()
        return result

    with patch.object(backend, 'observe', side_effect=delayed):
        try:
            # A second genuine loss generates another actual confirmed result.
            changed = time.monotonic()
            sensors.odom = pose_matrix([3.6, 0., 0.], [0., 0., 0., 1.])
            wait_for(lambda: sensors.valid is False, seconds=10)
            try:
                wait_for(computed.is_set, seconds=40)
            except AssertionError as failure:
                raise AssertionError('No actual recovery proposal for manual-race test; '
                                     +repr(mapper.recovery_report)) from failure
            mapper.registration_timer.cancel()
            old_generation = mapper.generation
            manual = truth @ pose_matrix([.65, .1, 0.], [0., 0., 0., 1.])
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            position, rotation = message.pose.pose.position, message.pose.pose.orientation
            position.x, position.y, position.z = map(float, manual[:3, 3])
            rotation.x, rotation.y, rotation.z, rotation.w = map(float, quaternion_from_matrix(manual))
            sensors.initial.publish(message)
            wait_for(lambda: mapper.generation > old_generation and mapper.pending_initial_pose is None)
            expected = mapper.correction.copy()
            assert mapper.last_accepted is None and not mapper.seeded
            release.set()
            wait_for(returned.is_set)
            time.sleep(.2)
            np.testing.assert_allclose(mapper.correction, expected, atol=1e-10)
            assert mapper.last_accepted is None and not mapper.seeded
            assert mapper.recovery_report['state'] == 'manual_initialization'
            assert mapper.generation == old_generation+1
            print('PASS manual pose race: actual confirmed GICP proposal discarded '
                  'after a newer manual pose; no acceptance or TF freshness restored', flush=True)
        finally:
            release.set()
            mapper.registration_timer.reset()
    wait_for(lambda: sensors.valid is True and mapper.last_accepted > changed, seconds=15)


def check(directory, truth, missing_scan_check=False):
    before = fingerprints(directory)
    metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
    scan_window = .5 if metadata.get('ground_aligned') else 0.0
    points = np.asarray(o3d.io.read_point_cloud(str(directory/'map.pcd')).points).copy()
    rclpy.init(args=['--ros-args', '-p', 'mode:=localization', '-p',
                    'map_directory:='+str(directory), '-p', 'scan_window:='+str(scan_window)])
    mapper, sensors = Mapper(), Sensors()
    sensors.points, sensors.truth, sensors.odom = points, truth, np.eye(4)
    sensors.packet_wait = .06
    events = []
    last_event = [None]
    def recovery_event(message):
        event = json.loads(message.data)
        events.append(event)
        signature = event.get('state'), event.get('confirmations'), event.get('reason')
        if signature != last_event[0]:
            print('RECOVERY '+json.dumps(event, sort_keys=True), flush=True)
            last_event[0] = signature
    sensors.create_subscription(String, '/localization/recovery',
        recovery_event, 10)
    revalidate = sensors.create_publisher(Empty, '/localization/recover', 1)
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (mapper, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()
    try:
        wait_for(lambda: sensors.valid is True and mapper.seeded, seconds=20)
        if scan_window:
            wait_for(lambda: 'scan_frames=' in sensors.localization_status and
                     int(sensors.localization_status.split('scan_frames=')[1].split()[0]) >= 2)
            assert mapper.p['scan_window'] == .5
        np.testing.assert_allclose(mapper.correction, truth, atol=.03)
        initial_accepted = mapper.last_accepted
        assert not any(event.get('state') == 'recovered' for event in events), events
        assert mapper.p['localization_max_translation'] == 1.0
        assert mapper.p['localization_min_fitness'] == .20
        assert mapper.p['validity_timeout'] == 1.0
        if missing_scan_check:
            wait_for(lambda: revalidate.get_subscription_count() > 0)
            requested = time.monotonic()
            revalidate.publish(Empty())
            wait_for(lambda: any(event.get('state') == 'recovered' and
                                 event.get('method') == 'tracking' and
                                 event.get('confirmations', 0) >= 3 for event in events), seconds=12)
            assert mapper.last_accepted > requested
            assert mapper.recovery_report['method'] == 'tracking'
            proof_stamps = {event['source_stamp'] for event in events
                            if event.get('method') == 'tracking' and
                            event.get('confirmations', 0) > 0}
            assert len(proof_stamps) >= 3, events
            # Recovery JSON and Bool health use separate ROS subscriptions;
            # their delivery order cannot establish the health edge.
            wait_for(lambda: sensors.valid is True, seconds=4)
            print('PASS heartbeat revalidation: explicit recovery request requires '
                  'three fresh quality-qualified normal GICP matches', flush=True)
            # A later spatial-loss test must produce its own local-search
            # evidence; the heartbeat proof cannot satisfy that assertion.
            events.clear()

        initial_accepted = mapper.last_accepted
        lost = time.monotonic()
        offset = pose_matrix([1.8, 0., 0.], [0., 0., 0., 1.])
        sensors.odom = offset
        # With packet accumulation the first window across an odometry jump
        # can still match its older frames. Wait for an actual rejected fresh
        # window; only the single-frame case rules out every newer acceptance.
        wait_for(lambda: sensors.valid is False and bool(mapper.last_failure) and
                 (scan_window or mapper.last_accepted is None or
                  mapper.last_accepted <= initial_accepted), seconds=10)
        assert initial_accepted < lost
        wait_for(lambda: any(event.get('state') in ('searching', 'confirming')
                             for event in events), seconds=10)
        if missing_scan_check:
            # No new geometry can confirm recapture, even with live odometry.
            mapper.registration_timer.cancel()
            sensors.points = np.zeros((3, 3))
            wait_for(lambda: mapper.latest is not None and mapper.latest[0].width == 3)
            # Start a new loss epoch from an actual fresh rejected tiny scan;
            # never seed a successful match or replace a registration result.
            mapper.process()
            time.sleep(1.3)
            assert sensors.valid is False
            accepted = mapper.last_accepted
            assert accepted is None or accepted < lost
            changed = time.monotonic()
            sensors.points = points
            wait_for(lambda: mapper.latest[0].width == len(points) and mapper.latest[2] > changed)
            mapper.registration_timer.reset()

        wait_for(lambda: sensors.valid is True and mapper.last_accepted is not None and
                 mapper.last_accepted > lost and
                 any(event.get('state') == 'recovered' and event.get('method') == 'local_search'
                     for event in events), seconds=40)
        successes = [event for event in events if event.get('state') == 'recovered' and
                     event.get('method') == 'local_search']
        assert successes[-1]['confirmations'] >= 3, successes[-1]
        expected = truth @ np.linalg.inv(offset)
        np.testing.assert_allclose(mapper.correction, expected, atol=.05)
        wait_for(lambda: sensors.buffer.can_transform('map', 'base_footprint', Time()))
        body = transform_matrix(sensors.buffer.lookup_transform('map', 'base_link', Time()).transform)
        footprint = transform_matrix(sensors.buffer.lookup_transform('map', 'base_footprint', Time()).transform)
        np.testing.assert_allclose(body, truth, atol=.05)
        np.testing.assert_allclose(footprint, planar_pose(truth), atol=.05)
        assert abs(footprint[2, 3]) < .005
        assert sensors.registered is not None and sensors.registered.header.frame_id == 'map'
        time.sleep(.8)
        assert sensors.valid is True, mapper.last_failure
        assert fingerprints(directory) == before, 'Relocalization modified the selected map'
        if missing_scan_check:
            manual_override_race(mapper, sensors, truth)

        # A recovered correction must obey the original sensor/TF freshness
        # gate after the input stops. Recovery cannot keep renewing acceptance.
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        wait_for(lambda: sensors.valid is False, seconds=4)
        stamp = sensors.correction_stamp()
        time.sleep(.25)
        assert sensors.correction_stamp() == stamp
        print(f'PASS relocalization {directory.name}: actual tracking rejection, '
              'three fresh geometric confirmations, corrected body/footprint TF, '
              f'unchanged selected map and expired-sensor stop; scan_window={scan_window}', flush=True)
    finally:
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        # Stop producers before removing the ROS nodes; a background result
        # must finish while its publishers and the executor still exist.
        for timer in mapper.timers:
            timer.cancel()
        if mapper.recovery_worker is not None:
            mapper.recovery_worker.close()
        executor.shutdown()
        spinner.join(timeout=3)
        executor.remove_node(mapper)
        executor.remove_node(sensors)
        mapper.destroy_node()
        sensors.destroy_node()
        rclpy.shutdown()


def main():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)/'maps'
        root.mkdir()
        for name, tilted in (('recovery-a', False), ('recovery-b', True)):
            source = saved_raw(root, name, tilted=tilted, start=np.eye(4))
            check(source, np.eye(4), missing_scan_check=(name == 'recovery-a'))
            aligned = root/('aligned-'+name)
            build_bundle(root, source, aligned)
            metadata = yaml.safe_load((aligned/'metadata.yaml').read_text())
            truth = np.asarray(metadata['map_start_pose'])
            check(aligned, truth)
            # The same canonical bundle can be loaded repeatedly without any
            # extra pose or standing-height transformation.
            repeated = root/('repeated-'+name)
            build_bundle(root, aligned, repeated)
            np.testing.assert_allclose(yaml.safe_load(
                (repeated/'metadata.yaml').read_text())['map_start_pose'], truth, atol=1e-9)
            assert (repeated/'map.pcd').read_bytes() == (aligned/'map.pcd').read_bytes()


if __name__ == '__main__':
    main()
