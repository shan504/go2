#!/usr/bin/env python3
"""Actual Humble adapter/TF/export test with synthetic RTAB-Map graph messages.

This exercises ROS transport and graph application, not visual recognition or
the RTAB-Map optimizer. No robot, camera calibration or real loop is claimed.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from unittest.mock import patch

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header, String, Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from rtabmap_msgs.msg import MapData, Node, Link, Info
from rtabmap_msgs.srv import GetMap
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'go2_3d'))
from geometry import cloud_xyz, pose_matrix
from ros_cloud import xyz_message
from rtabmap_backend import RtabmapBackend
from map_export import save_snapshot as real_save_snapshot
from test_camera_calibration import synthetic_calibration


def pose(x):
    value = Pose()
    value.position.x = float(x)
    value.orientation.w = 1.
    return value


def header(stamp, frame):
    value = Header(frame_id=frame)
    value.stamp.sec, value.stamp.nanosec = divmod(int(stamp), 1_000_000_000)
    return value


def main():
    with tempfile.TemporaryDirectory(prefix='go2-rtab-backend-test-') as directory:
        directory = Path(directory)
        calibration = directory/'synthetic_camera.yaml'
        calibration.write_text(yaml.safe_dump(synthetic_calibration()))
        rclpy.init(args=['--ros-args', '-p', f'calibration_file:={calibration}',
                        '-p', f'session_directory:={directory / "session"}',
                        '-p', f'output_directory:={directory / "maps"}',
                        '-p', 'map_name:=synthetic_farm', '-p', 'map_publish_period:=0.1'])
        backend, observer = RtabmapBackend(), RosNode('rtabmap_backend_smoke_observer')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(backend)
        executor.add_node(observer)
        spinner = threading.Thread(target=executor.spin)
        spinner.start()
        received, maps, transforms, statuses = {name: [] for name in ('odom', 'scan_cloud', 'image', 'camera_info')}, [], [], []
        readiness = []
        for name, message_type in (('odom', Odometry), ('scan_cloud', PointCloud2), ('image', Image), ('camera_info', CameraInfo)):
            observer.create_subscription(message_type, '/rtabmap/input/'+name,
                lambda message, key=name: received[key].append(message), 100)
        observer.create_subscription(PointCloud2, '/map_cloud', maps.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        observer.create_subscription(TFMessage, '/tf', lambda message: transforms.extend(message.transforms), 100)
        observer.create_subscription(String, '/mapping/loop_closure',
            lambda message: statuses.append(json.loads(message.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        observer.create_subscription(Bool, '/mapping/backend_ready',
            lambda message: readiness.append(message.data),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sources = {}
        for name, message_type, topic in (
            ('odom', Odometry, '/gicp/odom'), ('cloud', PointCloud2, '/gicp/scan_cloud'),
            ('key_odom', Odometry, '/gicp/keyframe/odom'), ('key_cloud', PointCloud2, '/gicp/keyframe/cloud'),
            ('image', Image, '/camera/loop/image_rect'), ('camera_info', CameraInfo, '/camera/loop/camera_info'),
            ('graph', MapData, '/rtabmap/mapData'), ('info', Info, '/rtabmap/info')):
            sources[name] = observer.create_publisher(message_type, topic, 100)
        save_client = observer.create_client(Trigger, '/save_3d_map')
        generator = np.random.default_rng(932)
        floor = generator.uniform([.5, -1., -.3], [2.5, 1., -.3], (2000, 3))
        poles = generator.uniform([1.7, -.15, -.25], [1.9, .15, .8], (500, 3))
        local_points = np.vstack((floor, poles))
        stamps, source_poses = [], []

        def wait_for(predicate, label, timeout=12., allow_fatal=False):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if backend.fatal and not allow_fatal:
                    raise AssertionError(backend.fatal)
                if predicate():
                    return
                time.sleep(.015)
            raise AssertionError(f'{label}; status={backend.report_cache}; fatal={backend.fatal}')

        def send_scan(x, camera=True):
            stamp = observer.get_clock().now().nanoseconds
            odom = Odometry(header=header(stamp, 'gicp_odom'), child_frame_id='base_link')
            odom.pose.pose = pose(x)
            cloud = xyz_message(header(stamp, 'base_link'), local_points)
            # Deliberately put mapping messages ahead of accepted stream.
            sources['key_odom'].publish(odom)
            sources['key_cloud'].publish(cloud)
            sources['odom'].publish(odom)
            sources['cloud'].publish(cloud)
            stamps.append(stamp)
            source_poses.append(float(x))
            if camera:
                send_camera(stamp)
            return stamp

        def send_camera(stamp):
            image = Image(header=header(stamp+10_000_000, 'synthetic_camera_optical'),
                          width=160, height=120, encoding='bgr8', step=480,
                          data=bytes(160*120*3))
            info = CameraInfo(header=deepcopy(image.header), width=160, height=120,
                              k=[100., 0., 80., 0., 100., 60., 0., 0., 1.],
                              p=[100., 0., 80., 0., 0., 100., 60., 0., 0., 0., 1., 0.],
                              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], distortion_model='plumb_bob')
            sources['image'].publish(image)
            sources['camera_info'].publish(info)

        def graph(xs, indices=None, loop=False):
            indices = list(range(len(xs))) if indices is None else indices
            message = MapData(header=header(observer.get_clock().now().nanoseconds, 'map'))
            message.graph.header = deepcopy(message.header)
            message.graph.map_to_odom.rotation.w = 1.
            for index in indices:
                node = Node(id=index+1, stamp=stamps[index]/1e9, pose=pose(source_poses[index]))
                message.nodes.append(node)
                message.graph.poses_id.append(index+1)
                message.graph.poses.append(pose(xs[index]))
            if loop:
                link = Link(from_id=3, to_id=1, type=1,
                            information=(np.eye(6)*100.).ravel().tolist())
                link.transform.rotation.w = 1.
                message.graph.links.append(link)
            return message

        def save():
            future = save_client.call_async(Trigger.Request())
            wait_for(future.done, 'save response', timeout=30., allow_fatal=bool(backend.fatal))
            return future.result()

        try:
            wait_for(lambda: all(source.get_subscription_count() for source in sources.values()), 'DDS discovery')
            wait_for(lambda: readiness and readiness[-1], 'durable backend readiness')
            assert save_client.wait_for_service(timeout_sec=5.)
            for x in (0., 1., 3.):
                send_scan(x)
                wait_for(lambda: backend.graph.report()['observations'] == len(stamps), 'full keyframe journal')
                time.sleep(.04)
            wait_for(lambda: all(len(messages) == 3 for messages in received.values()), 'four exact RTAB inputs')
            for index in range(3):
                actual = [message[index].header.stamp for message in received.values()]
                assert all(value == actual[0] for value in actual)
            assert len(list((directory/'session/keyframes').glob('*.npz'))) == 3
            assert not save().success, 'An unanchored graph was exported'
            initial = graph(source_poses)
            sources['graph'].publish(initial)
            wait_for(lambda: backend.graph.report()['graph_revision'] == 1, 'initial graph applied')
            assert backend.graph.report()['map_rebuilds'] == 0
            # While one graph is rebuilding, newer graph notifications replace
            # each other; original measurement journals are never dropped.
            rebuilding, graph_release = threading.Event(), threading.Event()
            original_prepare = backend.graph.prepare_graph
            def delayed_prepare(*args, **kwargs):
                rebuilding.set()
                if not graph_release.wait(12.):
                    raise RuntimeError('test graph release missing')
                return original_prepare(*args, **kwargs)
            with patch.object(backend.graph, 'prepare_graph', delayed_prepare):
                try:
                    backend.receive_graph(graph(source_poses))
                    wait_for(rebuilding.is_set, 'graph preparation held')
                    for index in range(150):
                        backend.receive_graph(graph(source_poses))
                    assert backend.graph_notice_pending
                    assert backend.jobs.qsize() < 10, 'obsolete graphs accumulated as full rebuild jobs'
                finally:
                    graph_release.set()
                wait_for(lambda: backend.graph.report()['graph_revision'] >= 3 and not backend.graph_notice_pending,
                         'latest coalesced graph applied')
            before = backend.graph.snapshot()
            info = Info(ref_id=3, loop_closure_id=1,
                        stats_keys=['Loop/Highest_hypothesis_id/', 'Loop/Highest_hypothesis_value/',
                                    'Loop/RejectedHypothesis/', 'Loop/Visual_inliers/', 'Loop/Optimization_iterations/'],
                        stats_values=[1., .9, 0., 47., 9.])
            sources['info'].publish(info)
            wait_for(lambda: 3 in backend.info_by_ref, 'matching optimization info')
            # A partial graph must neither change historical geometry nor save.
            sources['graph'].publish(graph([0., .8, 0.], indices=[2], loop=True))
            wait_for(lambda: bool(backend.pending_reason), 'partial graph blocked')
            assert backend.graph.snapshot().map_revision == before.map_revision
            assert not save().success, 'Incomplete historical optimization was exported'
            complete_queries = []
            def full_graph(request, response):
                complete_queries.append((request.global_map, request.optimized, request.graph_only))
                response.data = graph([0., .8, 0.], loop=True)
                # Services use wall time, not their newest node's sensor time.
                # A future service header must not suppress later source nodes.
                response.data.header.stamp.sec += 60
                response.data.graph.header = deepcopy(response.data.header)
                return response
            service = observer.create_service(GetMap, '/rtabmap/get_map_data', full_graph)
            wait_for(lambda: backend.graph.report()['map_rebuilds'] == 1, 'global graph recovery and dense rebuild')
            assert complete_queries == [(True, True, False)]
            after = backend.graph.snapshot()
            assert after.map_revision > before.map_revision
            assert not np.array_equal(before.points, after.points)
            assert before.free != after.free
            np.testing.assert_allclose(after.poses[-1][:3, 3], [0., 0., 0.], atol=1e-5)
            np.testing.assert_allclose(after.correction[:3, 3], [-3., 0., 0.], atol=1e-5)
            assert len(backend.links) == 1
            entered, release = threading.Event(), threading.Event()
            def slow_export(*args):
                entered.set()
                if not release.wait(15.):
                    raise RuntimeError('test export release missing')
                return real_save_snapshot(*args)
            with patch('rtabmap_backend.save_snapshot', slow_export):
                checkpoint = save_client.call_async(Trigger.Request())
                try:
                    wait_for(entered.is_set, 'checkpoint writer started')
                    parallel = save()
                    assert not parallel.success and 'already running' in parallel.message
                    # More than a bounded queue's capacity must be processed
                    # while the disk exporter is deliberately held up.
                    for index in range(105):
                        reference = 1000+index
                        sources['info'].publish(Info(ref_id=reference))
                        wait_for(lambda ref=reference: ref in backend.info_by_ref,
                                 'journal continues during slow export')
                    assert not checkpoint.done()
                    assert not backend.fatal
                finally:
                    release.set()
                wait_for(checkpoint.done, 'checkpoint result', timeout=30.)
                saved = checkpoint.result()
            assert saved.success, saved.message
            destination = Path(saved.message)
            metadata = yaml.safe_load((destination/'metadata.yaml').read_text())
            assert metadata['loop_closure']['global_loop_constraints'] == 1
            assert metadata['loop_closure']['map_rebuilds'] == 1
            assert len(o3d.io.read_point_cloud(str(destination/'map.pcd')).points) > 100
            assert (destination/'nav.yaml').exists()
            assert (directory/'maps/latest').resolve() == destination.resolve()
            # A new local scan displayed with held correction cannot be saved
            # until RTAB-Map acknowledges a graph anchor covering it.
            send_scan(3.4, camera=False)
            wait_for(lambda: backend.graph.report()['observations'] == 4, 'new keyframe tail')
            assert not save().success, 'Unacknowledged final keyframe was exported'
            send_camera(stamps[-1])
            wait_for(lambda: len(received['odom']) == 4, 'tail synchronization')
            tail_graph = graph([0., .8, 0., .4], loop=True)
            tail_node = deepcopy(tail_graph.nodes[-1])
            tail_graph.nodes = []
            sources['graph'].publish(tail_graph)
            wait_for(lambda: 'timestamp metadata' in backend.pending_reason, 'tail graph waits for node metadata')
            # A delayed complete-map service response contains only node 3.
            # It must not replace pending graph 4 before its NodeData arrives.
            backend.enqueue('full_graph', graph([0., .8, 0.], loop=True))
            wait_for(lambda: 'full_graph_metadata_only' in (directory/'session/events.jsonl').read_text(),
                     'older service response retained newer pending graph')
            assert backend.graph_ref(backend.pending_graph) == 4
            backend.enqueue('nodes', [tail_node])
            wait_for(lambda: backend.graph.ready_for_save() and backend.pending_graph is None, 'tail covered')
            wait_for(lambda: bool(transforms), 'global correction TF')
            assert {(value.header.frame_id, value.child_frame_id) for value in transforms} == {('map', 'gicp_odom')}
            events = [json.loads(line) for line in (directory/'session/events.jsonl').read_text().splitlines()]
            closures = [event for event in events if event['event'] == 'graph_applied' and event['global_loop_constraints_added']]
            assert closures and closures[0]['optimization_iterations'] == 9.
            assert closures[0]['graph_ref_id'] == closures[0]['info_ref_id'] == 3
            assert closures[0]['map_rebuilt'] is True
            associations = [event for event in events if event['event'] == 'backend_input']
            assert all(event['exposure_stamp_ns']-event['scan_stamp_ns'] == 10_000_000 for event in associations)
            retained_graph = json.loads((directory/'session/graph_latest.json').read_text())
            assert retained_graph['links'][0]['transform'] == np.eye(4).tolist()
            # A wrong frontend frame is a hard failure, never a fallback to
            # native odometry or an unoptimized save.
            bad_stamp = observer.get_clock().now().nanoseconds
            wrong = Odometry(header=header(bad_stamp, 'odom'), child_frame_id='base_link')
            wrong.pose.pose.orientation.w = 1.
            sources['odom'].publish(wrong)
            sources['cloud'].publish(xyz_message(header(bad_stamp, 'base_link'), local_points))
            wait_for(lambda: bool(backend.fatal), 'wrong frame rejected', allow_fatal=True)
            wait_for(lambda: readiness and not readiness[-1], 'backend readiness revoked', allow_fatal=True)
            assert not save().success
            # SIGINT shuts down rclpy's context before destroy_node. The real
            # subprocess must still drain/join its non-daemon writer cleanly.
            shutdown_log = directory/'shutdown.log'
            shutdown_session = directory/'shutdown_session'
            process = None
            try:
                with shutdown_log.open('w') as log:
                    process = subprocess.Popen([sys.executable, str(ROOT/'go2_3d/rtabmap_backend.py'),
                        '--ros-args', '-p', f'calibration_file:={calibration}',
                        '-p', f'session_directory:={shutdown_session}'],
                        stdout=log, stderr=subprocess.STDOUT,
                        env=dict(os.environ, ROS_DOMAIN_ID='137'))
                wait_for(lambda: (shutdown_session/'events.jsonl').exists() and
                         'backend_started' in (shutdown_session/'events.jsonl').read_text(),
                         'shutdown subprocess startup', allow_fatal=True)
                process.send_signal(signal.SIGINT)
                code = process.wait(timeout=8.)
                assert code == 0, shutdown_log.read_text()
                assert 'backend_stopped' in (shutdown_session/'events.jsonl').read_text()
            finally:
                if process is not None and process.poll() is None:
                    process.kill()
                    process.wait()
            print('PASS RTAB adapter: exact ROS quartet, durable local keyframes, partial graph fail-closed/global query, '
                  '3m synthetic optimized cloud+free-ray rebuild, shared PCD/grid export, tail save gate, '
                  'slow export processes >100 events/concurrent-save rejection, 150 graph notifications coalesced, '
                  'backend readiness, single TF edge, source-frame rejection and clean SIGINT. '
                  'No real loop recognition claimed.')
        finally:
            executor.shutdown()
            spinner.join(timeout=5.)
            backend.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
