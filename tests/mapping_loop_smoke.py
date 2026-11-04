"""Real ROS publication/save integration for the synthetic 200m loop backend.

The accepted-observation journal is supplied directly, so this test isolates
loop detection/rebuild/mapper commit and ROS publication. It does not measure
scan-to-map tracking or robot behavior; ros_smoke.py exercises live GICP.
"""
import json
import math
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2

from ros_smoke import Mapper, Sensors, wait_for, transform_matrix
from geometry import cloud_xyz
from mapping_rebuild import MappingObservation, append_rebuilt
from test_loop_closure import farm_geometry, local_scan, square_route, pose


with tempfile.TemporaryDirectory() as temporary:
    rclpy.init(args=['--ros-args', '-p', 'mode:=mapping', '-p',
                    'output_directory:='+temporary, '-p', 'map_name:=farm-loop'])
    mapper, sensors = Mapper(), Sensors()
    mapper.registration_timer.cancel()
    # Keep sensor callbacks available to validate real ROS TF/publication.
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mapper, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    received = {'cloud': None, 'loop': None}
    sensors.create_subscription(PointCloud2, '/map_cloud',
        lambda message: received.update(cloud=message), 1)
    sensors.create_subscription(String, '/mapping/loop_closure',
        lambda message: received.update(loop=json.loads(message.data)), 1)
    world, route = farm_geometry(), square_route() + [pose(1.5, 0)]
    originals = []
    original_start = route[0].copy()
    try:
        for index, truth in enumerate(route):
            progress = min(index/40., 1.)
            drift = pose(5.*progress, -2.*progress,
                         math.radians(6.)*progress, .25*progress)
            measured = drift @ truth
            observation = MappingObservation(local_scan(world, truth), measured)
            originals.append(measured.copy())
            with mapper.lock:
                append_rebuilt(mapper.mapping_store, mapper.grid, observation, measured)
                mapper.mapping_start_pose = original_start.copy()
                mapper.last_keyframe = measured.copy()
                mapper.loop_worker.append(observation)
            if index >= 37:
                wait_for(lambda: mapper.loop_worker.report()['checked_observations'] >= index+1
                         or mapper.loop_worker.report()['state'] == 'error', seconds=90)
                assert not mapper.loop_worker.report()['error'], mapper.loop_worker.report()
                if mapper.loop_worker.report()['applied_loops']:
                    break
        wait_for(lambda: mapper.loop_worker.ready_for_save(), seconds=90)
        report = mapper.loop_worker.report()
        assert report['applied_loops'] >= 1, report
        assert mapper.generation >= 1, 'An old in-flight map update remains publishable'
        np.testing.assert_allclose(mapper.mapping_start_pose, original_start, atol=1e-8)
        with mapper.lock:
            corrected_last = mapper.loop_worker.observations[-1].pose
            expected_delta = corrected_last @ np.linalg.inv(originals[-1])
            np.testing.assert_allclose(mapper.correction, expected_delta, atol=1e-8)
            # Seed exactly one synthetic accepted observation; its freshness
            # must expire normally, never renew because the loop was applied.
            mapper.seeded = True
            mapper.last_accepted = time.monotonic()
            mapper.accepted_odom = originals[-1].copy()
        sensors.truth, sensors.odom = route[len(originals)-1], originals[-1]
        sensors.points = world
        sensors.emitter.start()
        wait_for(lambda: sensors.buffer.can_transform('map', 'odom', Time()), seconds=3)
        actual = transform_matrix(sensors.buffer.lookup_transform('map', 'odom', Time()).transform)
        np.testing.assert_allclose(actual, expected_delta, atol=1e-6)
        mapper.publish_map()
        wait_for(lambda: received['cloud'] is not None and
                 received['cloud'].width == mapper.map_point_count, seconds=3)
        np.testing.assert_allclose(cloud_xyz(received['cloud']), mapper.points, atol=2e-5)
        wait_for(lambda: received['loop'] is not None, seconds=3)
        assert received['loop']['applied_loops'] >= 1, received['loop']
        time.sleep(1.1)
        stamp = sensors.correction_stamp()
        time.sleep(.2)
        assert sensors.correction_stamp() == stamp, 'Loop correction renewed stale TF acceptance'
        result = mapper.save_map(Trigger.Request(), Trigger.Response())
        assert result.success, result.message
        selected = Path(temporary)/'latest'
        metadata = yaml.safe_load((selected/'metadata.yaml').read_text())
        assert metadata['loop_closure']['applied_loops'] >= 1
        assert metadata['ground_aligned']
        assert .1 < metadata['body_height_above_ground'] < 1.
        saved = np.asarray(o3d.io.read_point_cloud(str(selected/'map.pcd')).points)
        assert len(saved) > 100 and np.ptp(saved[:, 2]) > 2.
        assert (selected/'observed_free.npz').is_file()
        np.testing.assert_allclose(metadata['map_start_pose'],
            np.asarray(metadata['map_from_source_map'])@original_start, atol=1e-8)
        print('PASS ROS loop integration: actual synthetic 200m geometric closure; '
              'full map replay, anchored start, corrected ROS cloud/TF, '
              'unrenewed freshness and saved aligned map metadata')
    finally:
        sensors.stop.set()
        if sensors.emitter.is_alive():
            sensors.emitter.join(timeout=3)
        executor.shutdown()
        spinner.join(timeout=3)
        mapper.destroy_node()
        sensors.destroy_node()
        rclpy.shutdown()
