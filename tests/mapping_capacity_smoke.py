"""Real ROS/GICP capacity and atomic map-update regression; no robot commands.

Run in the existing ROS2 Humble test environment, in an unused ROS_DOMAIN_ID:
    OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 tests/mapping_capacity_smoke.py

The intentionally tiny point threshold exercises the former hard-stop branch
without allocating a million ROS points. Offline tests cover large maps.
"""
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from ros_smoke import Mapper, Sensors, wait_for
from geometry import cloud_xyz, filter_points, pose_matrix, transform_points
from test_3d import room


ROOT = Path(__file__).resolve().parents[1]


def nearest_distances(reference, queries):
    """Bounded test-size coverage checks, independent of map point ordering."""
    reference_cloud = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.array(reference, copy=True)))
    tree = o3d.geometry.KDTreeFlann(reference_cloud)
    return np.asarray([np.sqrt(tree.search_knn_vector_3d(point, 1)[2][0])
                       for point in queries])


def state_snapshot(mapper):
    with mapper.lock:
        fields = {}
        for name in ('correction', 'seeded', 'accepted_odom',
                     'accepted_match_correction', 'mapping_start_pose',
                     'last_keyframe', 'last_keyframe_time'):
            value = getattr(mapper, name)
            fields[name] = value.copy() if isinstance(value, np.ndarray) else value
        fields.update(points=mapper.points.copy(),
                      registration_points=mapper.registration_points.copy(),
                      free=mapper.grid.free.copy(),
                      revision=mapper.mapping_store.revision,
                      count=mapper.mapping_store.count,
                      last_accepted=mapper.last_accepted)
    return fields


def assert_not_partially_committed(mapper, before):
    after = state_snapshot(mapper)
    for name, expected in before.items():
        if name == 'last_accepted':
            # An internal update error may revoke TF validity, but must not
            # certify the failed scan by refreshing the accepted deadline.
            assert after[name] is None or after[name] == expected, name
        elif isinstance(expected, np.ndarray):
            np.testing.assert_array_equal(after[name], expected, err_msg=name)
        else:
            assert after[name] == expected, (name, after[name], expected)


def verify_bundle(mapper, output, name, first_pose, full_points):
    before = state_snapshot(mapper)
    result = mapper.save_map(Trigger.Request(), Trigger.Response())
    assert result.success, result.message
    selected = (output/'latest').resolve(strict=True)
    assert selected.parent == output/'library'/name
    assert (output/'library'/name/'latest').resolve(strict=True) == selected
    metadata = yaml.safe_load((selected/'metadata.yaml').read_text())
    original = Path(metadata['source_map'])
    assert original != selected and original.parent == selected.parent
    for directory in (original, selected):
        for filename in ('map.pcd', 'nav.yaml', 'nav.pgm', 'metadata.yaml', 'observed_free.npz'):
            assert (directory/filename).is_file(), (directory, filename)
        nav = yaml.safe_load((directory/'nav.yaml').read_text())
        assert (directory/nav['image']).is_file()
    raw = np.asarray(o3d.io.read_point_cloud(str(original/'map.pcd')).points)
    aligned = np.asarray(o3d.io.read_point_cloud(str(selected/'map.pcd')).points)
    assert len(raw) > mapper.p['max_map_points']
    # Saving and leveling must preserve the original dense geometry, not
    # replace it with the much smaller registration target.
    assert len(raw) >= len(full_points)*0.97, (len(raw), len(full_points))
    assert len(aligned) == len(raw), (len(aligned), len(raw))
    assert metadata['point_count'] == len(aligned)
    assert metadata['ground_aligned']
    np.testing.assert_allclose(metadata['ground_model']['plane'], [0, 0, 1, 0], atol=1e-8)
    assert 0.1 < metadata['body_height_above_ground'] < 1.0
    transform = np.asarray(metadata['map_from_source_map'])
    np.testing.assert_allclose(metadata['map_start_pose'], transform@first_pose, atol=1e-8)
    raw_metadata = yaml.safe_load((original/'metadata.yaml').read_text())
    np.testing.assert_allclose(raw_metadata['map_start_pose'], first_pose, atol=1e-8)
    expected = transform_points(raw, transform)
    assert nearest_distances(aligned, expected).max() < 2e-5
    # Saving a snapshot must not mutate the live map or its accepted pose.
    after = state_snapshot(mapper)
    for key in before:
        if isinstance(before[key], np.ndarray):
            np.testing.assert_array_equal(after[key], before[key], err_msg=key)
        else:
            assert after[key] == before[key], key
    return selected


def phase(output, dense, name):
    output.mkdir(parents=True)
    # Named-node YAML overrides have the same precedence as stack.launch;
    # later command-line -p options alone do not override dense named YAML.
    config = output/'capacity-smoke.yaml'
    config.write_text(yaml.safe_dump({'go2_gicp': {'ros__parameters': dict(
        mode='mapping', output_directory=str(output), map_name=name,
        map_directory=str(output/'latest'), max_map_points=200,
        registration_period=60.0, map_publish_period=60.0,
        keyframe_max_interval=0.01)}}))
    args = ['--ros-args', '--params-file', str(ROOT/'go2_3d/gicp.yaml')]
    if dense:
        args += ['--params-file', str(ROOT/'go2_3d/gicp_dense.yaml')]
    args += ['--params-file', str(config)]
    rclpy.init(args=args)
    mapper = sensors = executor = spinner = None
    emitter_started = False
    try:
        mapper, sensors = Mapper(), Sensors()
        mapper.registration_timer.cancel()
        sensors.packet_wait = 0.04
        all_points = room()
        # Interleaving covers the same asymmetric surfaces for a real GICP
        # match, while leaving half the observed 3D geometry genuinely new.
        sensors.points = all_points[::2]
        published = []
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensors.create_subscription(PointCloud2, '/map_cloud', published.append, retained)
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (mapper, sensors):
            executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        sensors.emitter.start()
        emitter_started = True

        def fresh_scan(after=0.0):
            def ready():
                with mapper.lock:
                    if mapper.latest is None:
                        return False
                    message, _, receipt = mapper.latest
                    stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
                    return (receipt > after and stamp != mapper.processed_stamp and
                            time.monotonic()-mapper.first_received > mapper.p['scan_window']+0.05)
            wait_for(ready)

        def accepted_scan(after=0.0):
            fresh_scan(after)
            previous = mapper.last_accepted
            mapper.process()
            assert mapper.last_accepted is not None, mapper.last_failure
            assert mapper.last_accepted > after, mapper.last_failure
            if previous is not None:
                assert mapper.last_accepted > previous, 'Accepted time did not advance'
            assert not mapper.last_failure, mapper.last_failure

        def failed_scan(target, attribute, label):
            fresh_scan()
            before = state_snapshot(mapper)
            with patch.object(target, attribute, side_effect=RuntimeError(label)) as injected:
                mapper.process()
            assert injected.called, f'{attribute} was not reached: {mapper.last_failure}'
            assert label in mapper.last_failure, mapper.last_failure
            assert_not_partially_committed(mapper, before)

        # A first-scan storage failure must not set seeded/start/keyframe or
        # modify the accepted correction before the map transaction commits.
        failed_scan(mapper.mapping_store, 'prepare', 'injected spatial-map preparation failure')
        assert not mapper.seeded and mapper.mapping_start_pose is None
        accepted_scan()
        first = state_snapshot(mapper)
        first_pose = first['mapping_start_pose'].copy()
        assert first['count'] == len(first['points']) > mapper.p['max_map_points']
        assert mapper.p['map_voxel_size'] == (0.02 if dense else 0.06)
        assert mapper.p['voxel_size'] == 0.15
        assert len(mapper.registration_points) < len(mapper.points)
        assert mapper.seeded

        if not dense:
            # Simulate a slow map update without relaxing the original scan
            # freshness rule. The prepared geometry must remain uncommitted.
            fresh_scan()
            before = state_snapshot(mapper)
            prepare = mapper.mapping_store.prepare
            def slow_prepare(points):
                update = prepare(points)
                time.sleep(mapper.p['validity_timeout']+0.05)
                return update
            with patch.object(mapper.mapping_store, 'prepare', side_effect=slow_prepare):
                mapper.process()
            assert 'too old after map update preparation' in mapper.last_failure, mapper.last_failure
            assert_not_partially_committed(mapper, before)
            accepted_scan()

        changed = time.monotonic()
        sensors.points = all_points
        # Empty the dense rolling window of the previous half scans, while
        # still using real DDS, timestamped TF and the production GICP path.
        wait_for(lambda: mapper.latest is not None and mapper.latest[0].width == len(all_points)
                 and mapper.latest[2] > changed+mapper.p['scan_window']+0.1)
        accepted_scan(changed)
        expanded = state_snapshot(mapper)
        assert expanded['count'] > first['count']*1.3, (expanded['count'], first['count'])
        assert expanded['revision'] > first['revision']
        tolerance = 2*np.sqrt(3)*mapper.p['map_voxel_size']
        assert nearest_distances(expanded['points'], first['points']).max() < tolerance
        # Coverage applies to actual admissible returns; the production
        # range filter deliberately excludes points within 0.35m of the body.
        additions = filter_points(all_points[1::2])
        novel = additions[nearest_distances(first['points'], additions) > tolerance]
        assert len(novel) > 10, 'Synthetic scan did not contain independently checkable new geometry'
        assert nearest_distances(expanded['points'], novel).max() < tolerance
        np.testing.assert_array_equal(mapper.correction, first['correction'],
                                      err_msg='Stationary enrichment moved the mapping anchor')

        # An already-seeded scan with changed odometry exercises rollback of
        # candidate pose fields, not only the empty-map bootstrap case.
        sensors.truth = pose_matrix([0.03, 0, 0], [0, 0, 0, 1])
        sensors.odom = pose_matrix([0.04, 0, 0], [0, 0, 0, 1])
        changed = time.monotonic()
        wait_for(lambda: mapper.latest is not None and mapper.latest[2] > changed+mapper.p['scan_window']+0.1
                 and abs(mapper.latest[1][0, 3]-0.04) < 1e-6)
        failed_scan(mapper.grid, 'prepare_observation', 'injected grid preparation failure')
        accepted_scan(changed)
        np.testing.assert_allclose(mapper.accepted_odom, sensors.odom, atol=1e-6)

        sensors.truth = sensors.odom = np.eye(4)
        changed = time.monotonic()
        wait_for(lambda: mapper.latest is not None and mapper.latest[2] > changed+mapper.p['scan_window']+0.1
                 and abs(mapper.latest[1][0, 3]) < 1e-6)
        accepted_scan(changed)
        anchor = mapper.correction.copy()
        for _ in range(3):
            accepted_scan()
            np.testing.assert_array_equal(mapper.correction, anchor,
                                          err_msg='Repeated stationary scans drifted the mapping anchor')
            np.testing.assert_array_equal(mapper.mapping_start_pose, first_pose)
        assert nearest_distances(mapper.points, first['points']).max() < tolerance
        assert nearest_distances(mapper.points, novel).max() < tolerance
        assert mapper.mapping_store.count == len(mapper.points) > mapper.p['max_map_points']
        mapper.publish_map()
        wait_for(lambda: published and published[-1].width == mapper.mapping_store.count)
        payload = cloud_xyz(published[-1])
        assert payload.shape == mapper.points.shape
        np.testing.assert_allclose(payload, mapper.points, atol=2e-5, rtol=0)
        selected = verify_bundle(mapper, output, name, first_pose, expanded['points'])
        wait_for(lambda: sensors.valid is False)
        print(f'PASS ROS mapping capacity: {"dense" if dense else "standard"} {name}; '
              f'points={mapper.mapping_store.count} threshold=200; both preparation failures atomic; '
              f'GICP recovery, stationary anchor, full cloud and retained/aligned bundle={selected}', flush=True)
    finally:
        if sensors is not None:
            sensors.stop.set()
            if emitter_started:
                sensors.emitter.join(timeout=3)
        if executor is not None:
            executor.shutdown()
        if spinner is not None:
            spinner.join(timeout=3)
        for node in (mapper, sensors):
            if node is not None:
                node.destroy_node()
        rclpy.shutdown()


def main():
    with tempfile.TemporaryDirectory(prefix='go2-mapping-capacity-') as tmp:
        for dense in (False, True):
            for name in ('capacity_alpha', 'capacity_beta'):
                phase(Path(tmp)/('dense' if dense else 'standard')/name, dense, name)


if __name__ == '__main__':
    main()
