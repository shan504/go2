"""Real automatic GICP startup at each selected map's saved mapping start."""
import shutil
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
import yaml
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from initial_pose_startup_smoke import StartupSensors, hint
from ros_smoke import wait_for
from mapper import Mapper, transform_matrix
from nav2_ready import Nav2Ready
from geometry import first_map_pose, planar_pose, pose_matrix
from map_library import fingerprints
from navigation_grid import build_bundle
from test_map_start import metadata, saved_raw


def check(directory, truth, automatic=True):
    # These are the launcher defaults. No initial XYZ/yaw or manual pose is
    # supplied in automatic cases; the metadata supplies the full body seed.
    rclpy.init(args=['--ros-args', '-p', 'mode:=localization', '-p', f'map_directory:={directory}',
                    '-p', f'auto_initialize:={str(automatic).lower()}', '-p', 'initial_from_map_start:=true',
                    '-p', 'scan_window:=0.5'])
    mapper, gate, sensors = Mapper(), Nav2Ready(), StartupSensors()
    mapper.registration_timer.cancel()
    map_points = np.asarray(o3d.io.read_point_cloud(str(directory / 'map.pcd')).points).copy()
    sensors.points = map_points[:40]
    sensors.truth = truth
    sensors.odom = pose_matrix([3.0, -2.0, 0.15], [0.04, -0.025, 0.20, 0.97])
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (mapper, gate, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()
    try:
        wait_for(lambda: sensors.pub.get_subscription_count() > 0 and sensors.valid is False and
                 gate.client.service_is_ready())
        assert not mapper.initialized and not sensors.lifecycle_calls
        packet = sensors.packet()
        sensors.pub.publish(packet)
        wait_for(lambda: len(mapper.pending_clouds) == 1)
        assert mapper.latest is None and not mapper.initialized
        assert not sensors.buffer.can_transform('map', 'base_footprint', Time())
        sensors.odometry_tf(packet.header.stamp)
        wait_for(lambda: mapper.latest is not None)
        sensors.data_enabled = True
        wait_for(lambda: len(mapper.history) >= 6)
        if automatic:
            wait_for(lambda: mapper.initialized)
            np.testing.assert_allclose(mapper.correction @ mapper.latest[1], truth, atol=1e-7)
            assert not sensors.observed_hints, 'Automatic startup required a browser pose publication'
        else:
            assert not mapper.initialized and not mapper.initial_hint_used
            sensors.initial.publish(hint(truth))
            wait_for(lambda: mapper.initialized and mapper.initial_hint_used)
        assert sensors.valid is False and not any(sensors.valid_events) and not mapper.seeded
        assert not sensors.lifecycle_calls and not sensors.buffer.can_transform('map', 'base_footprint', Time())
        mapper.registration_timer.reset()
        wait_for(lambda: 'Too few finite/range-filtered cloud points' in mapper.last_failure)
        assert sensors.valid is False and not any(sensors.valid_events) and not mapper.seeded
        assert not sensors.lifecycle_calls and not sensors.buffer.can_transform('map', 'base_footprint', Time())
        sensors.points = map_points
        wait_for(lambda: sensors.valid is True and gate.started and
                 sensors.buffer.can_transform('map', 'base_footprint', Time()), seconds=30)
        wait_for(lambda: 'fine=True' in sensors.localization_status)
        body = transform_matrix(sensors.buffer.lookup_transform('map', 'base_link', Time()).transform)
        footprint = transform_matrix(sensors.buffer.lookup_transform('map', 'base_footprint', Time()).transform)
        np.testing.assert_allclose(body, truth, atol=0.025)
        np.testing.assert_allclose(footprint, planar_pose(truth), atol=0.025)
        assert abs(footprint[2, 3]) < 0.003
        assert sensors.lifecycle_calls == [ManageLifecycleNodes.Request.STARTUP]
        assert mapper.p['localization_min_fitness'] == 0.20 and mapper.p['localization_max_rmse'] == 0.20
        assert mapper.p['validity_timeout'] == 1.0
        print(f'PASS {directory.name}: {"automatic saved-start" if automatic else "explicit manual mode"}; '
              'cloud-before-TF, real accepted fine GICP, full body/ground footprint and gated lifecycle', flush=True)
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
        root.mkdir()
        raw_start = first_map_pose(pose_matrix([4.0, -2.0, 0.4], [0.035, -0.02, 0.25, 0.967]))
        originals = []
        for name, tilted, stored in (('new-room', True, raw_start), ('legacy-room', False, None)):
            source = saved_raw(root, name, tilted=tilted, start=stored)
            originals.append((source, fingerprints(source)))
            output = Path(tmp) / f'runtime-{name}'
            build_bundle(root, source, output)
            transform = np.asarray(metadata(output)['map_from_source_map'])
            truth = transform @ (raw_start if stored is not None else np.eye(4))
            check(output, truth)
            aligned = root / f'aligned-{name}'
            shutil.copytree(output, aligned)
            if stored is None:
                old = metadata(aligned)
                old.pop('map_start_pose', None)
                old['leveled_from'] = '/maps/legacy-room'
                old['map_from_previous_map'] = transform.tolist()
                (aligned / 'metadata.yaml').write_text(yaml.safe_dump(old))
                repeated = root / 'legacy-identity-reload'
                shutil.copytree(aligned, repeated)
                old['leveled_from'] = str(aligned)
                old['map_from_previous_map'] = np.eye(4).tolist()
                (repeated / 'metadata.yaml').write_text(yaml.safe_dump(old))
                aligned = repeated
            originals.append((aligned, fingerprints(aligned)))
            build_bundle(root, aligned, output)
            np.testing.assert_allclose(metadata(output)['map_start_pose'], truth, atol=1e-10)
            check(output, truth)
        check(output, truth, automatic=False)
        for source, before in originals:
            assert fingerprints(source) == before
        print('PASS shared automatic mapping-start initialization: new/raw/canonical/legacy chains and manual opt-out; sources unchanged', flush=True)


if __name__ == '__main__':
    main()
