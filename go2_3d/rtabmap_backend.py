#!/usr/bin/env python3
"""RTAB-Map input association, durable graph journal and corrected map export.

GICP remains the sole local front end. This node never changes its poses or
registration parameters. RTAB-Map performs recognition/VisIcp/optimization;
this adapter applies its graph to the original local mapping keyframes.
"""
from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

import message_filters
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header, String, Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from rtabmap_msgs.msg import MapData, Info
from rtabmap_msgs.srv import GetNodeData, GetMap

from camera_calibration import load_calibration
from geometry import cloud_xyz, pose_matrix, quaternion_from_matrix, transform_points
from map_export import save_snapshot
from map_library import map_name
from ros_cloud import xyz_message
from rtabmap_graph import GraphMap
from rtabmap_sync import AssociationGate


def stamp_ns(header):
    result = int(header.stamp.sec)*1_000_000_000 + int(header.stamp.nanosec)
    if result <= 0:
        raise ValueError('unset or negative input timestamp')
    return result


def pose_from_message(message):
    p, q = message.position, message.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def _json(value):
    """ROS statistics may use NaN for unavailable measurements."""
    if isinstance(value, dict):
        return {str(key): _json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


@dataclass
class SaveRequest:
    event: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)
    started: bool = False
    cancelled: bool = False
    success: bool = False
    message: str = ''


class RtabmapBackend(Node):
    def __init__(self):
        super().__init__('go2_rtabmap_backend')
        defaults = dict(map_voxel_size=.06, voxel_size=.15, grid_resolution=.10,
                        floor_z=-.30, obstacle_min_height=.10, obstacle_max_height=1.5,
                        map_publish_period=2., map_name='default', output_directory='/maps',
                        session_directory='', calibration_file='', max_pair_skew=.10)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        map_name(self.p['map_name'])
        if not math.isfinite(self.p['map_publish_period']) or self.p['map_publish_period'] <= 0:
            raise ValueError('map_publish_period must be positive')
        calibration = load_calibration(self.p['calibration_file'], self.p['max_pair_skew'])
        self.gate = AssociationGate(self.p['max_pair_skew'], calibration.uncertainty_sec)
        self.graph = GraphMap(self.p['map_voxel_size'], self.p['voxel_size'], self.p['grid_resolution'])
        if not self.p['session_directory']:
            raise ValueError('session_directory must identify a fresh writable session')
        self.directory = Path(self.p['session_directory'])
        self.directory.mkdir(parents=True, exist_ok=True)
        # RTAB-Map may already have opened rtabmap.db in this fresh directory.
        # An existing journal is never resumed or overwritten implicitly.
        self.keyframes_dir = self.directory/'keyframes'
        self.keyframes_dir.mkdir(exist_ok=False)
        self.events = (self.directory/'events.jsonl').open('x', buffering=1)
        self.poses_file = (self.directory/'gicp_poses.jsonl').open('x', buffering=1)
        (self.directory/'camera_calibration.yaml').write_bytes(Path(self.p['calibration_file']).read_bytes())
        (self.directory/'backend_parameters.json').write_text(json.dumps(self.p, indent=2))
        self.jobs = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        self.state_lock = threading.RLock()
        self.event_lock = threading.RLock()
        self.export_threads = []
        self.save_active = False
        self.fatal = ''
        self.fatal_recorded = False
        self.state = 'waiting_for_inputs'
        self.pair_pending = {kind: {} for kind in ('scan', 'keyframe', 'camera')}
        self.pending_keyframes = {}
        self.accepted_poses = {}
        self.accepted_stamps = []
        self.node_stamps = {}
        self.retained_graph = {}
        self.omitted_graph_anchors = set()
        self.links = set()
        self.pending_graph = None
        self.graph_mailbox = None
        self.graph_notice_pending = False
        self.pending_reason = ''
        self.node_query = None
        self.map_query = None
        self.last_map_query = float('-inf')
        self.last_graph_source_stamp = 0
        self.info_by_ref = {}
        self.last_dispatched_stamp = 0
        self.latest_odom = None
        self.last_map_publish = 0.
        self.last_published_revision = -1
        self.optimization_iterations = None
        self.report_cache = dict(state=self.state, error='', session_directory=str(self.directory))
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inputs = {
            'odom': self.create_publisher(Odometry, '/rtabmap/input/odom', reliable),
            'cloud': self.create_publisher(PointCloud2, '/rtabmap/input/scan_cloud', reliable),
            'image': self.create_publisher(Image, '/rtabmap/input/image', reliable),
            'info': self.create_publisher(CameraInfo, '/rtabmap/input/camera_info', reliable)}
        self.map_pub = self.create_publisher(PointCloud2, '/map_cloud', retained)
        self.registered_pub = self.create_publisher(PointCloud2, '/registered_cloud', 5)
        self.status_pub = self.create_publisher(String, '/mapping/loop_closure', retained)
        self.ready_pub = self.create_publisher(Bool, '/mapping/backend_ready', retained)
        self.transforms = TransformBroadcaster(self)
        self.filters = []
        for kind, first_type, first_topic, second_type, second_topic in (
                ('scan', Odometry, '/gicp/odom', PointCloud2, '/gicp/scan_cloud'),
                ('keyframe', Odometry, '/gicp/keyframe/odom', PointCloud2, '/gicp/keyframe/cloud'),
                ('camera', Image, '/camera/loop/image_rect', CameraInfo, '/camera/loop/camera_info')):
            first = message_filters.Subscriber(self, first_type, first_topic, qos_profile=reliable)
            second = message_filters.Subscriber(self, second_type, second_topic, qos_profile=reliable)
            # TimeSynchronizer normally discards old unmatched messages. Track
            # both halves first, so a lost keyframe cannot vanish silently.
            first.registerCallback(lambda message, k=kind: self.track_half(k, 0, message))
            second.registerCallback(lambda message, k=kind: self.track_half(k, 1, message))
            synchronizer = message_filters.TimeSynchronizer([first, second], 101)
            synchronizer.registerCallback(lambda a, b, k=kind: self.paired(k, a, b))
            self.filters.append((first, second, synchronizer))
        self.create_subscription(MapData, '/rtabmap/mapData', self.receive_graph, reliable)
        self.create_subscription(Info, '/rtabmap/info', lambda value: self.enqueue('info', value), reliable)
        self.node_client = self.create_client(GetNodeData, '/rtabmap/get_node_data')
        self.map_client = self.create_client(GetMap, '/rtabmap/get_map_data')
        self.create_service(Trigger, '/save_3d_map', self.save, callback_group=ReentrantCallbackGroup())
        self.create_timer(.5, self.tick)
        self.worker = threading.Thread(target=self.work, name='rtabmap-map-journal')
        self.worker.start()

    def fail(self, error):
        with self.state_lock:
            if not self.fatal:
                self.fatal = str(error)
                self.get_logger().error('Camera-assisted mapping stopped: '+self.fatal)
                try:
                    if rclpy.ok():
                        self.ready_pub.publish(Bool(data=False))
                except Exception:
                    pass  # Shutdown must still stop/join the journal worker.
                try:
                    self.jobs.put_nowait(('fault', None))
                except queue.Full:
                    pass  # An already queued item wakes the journal writer.

    def track_half(self, kind, side, message):
        try:
            stamp = stamp_ns(message.header)
            with self.state_lock:
                item = self.pair_pending[kind].setdefault(stamp, (set(), time.monotonic()))
                item[0].add(side)
                if len(self.pair_pending[kind]) > 100:
                    raise RuntimeError(f'{kind} exact-pair backlog full; no keyframe may be dropped')
        except Exception as error:
            self.fail(error)

    def paired(self, kind, first, second):
        try:
            stamp = stamp_ns(first.header)
            if stamp != stamp_ns(second.header):
                raise ValueError('exact-pair callback received different timestamps')
            with self.state_lock:
                self.pair_pending[kind].pop(stamp, None)
            self.enqueue(kind, (first, second))
        except Exception as error:
            self.fail(error)

    def enqueue(self, kind, payload):
        if self.stop_event.is_set():
            return False
        if hasattr(self, 'worker') and not self.worker.is_alive():
            return False
        if self.fatal and kind != 'save':
            return False
        try:
            self.jobs.put_nowait((kind, payload))
            return True
        except queue.Full:
            self.fail('backend work queue overflow; refusing incomplete map export')
            return False

    def receive_graph(self, message):
        if self.fatal or self.stop_event.is_set():
            return
        with self.state_lock:
            # Only optimized graph notifications may be coalesced. Every raw
            # accepted pose and mapping keyframe is still journaled. If an
            # intermediate notification held missing Node metadata, the
            # official GetNodeData service supplies it before applying a map.
            self.graph_mailbox = message
            if self.graph_notice_pending:
                return
            self.graph_notice_pending = True
        self.enqueue('graph_notice', None)

    def tick(self):
        with self.state_lock:
            for kind, pending in self.pair_pending.items():
                if pending and time.monotonic()-min(item[1] for item in pending.values()) > 10.:
                    self.fail(f'{kind} input missing its matching timestamp for >10 seconds')
            report = dict(self.report_cache, error=self.fatal or self.report_cache.get('error', ''))
        # The frontend waits for this AND discovery of all four subscribers
        # before committing its first map observation. Launch order alone
        # cannot prevent an entire volatile keyframe pair being lost at start.
        discovered = all(self.count_publishers(source.sub.topic_name) > 0
                         for first, second, _ in self.filters[:2] for source in (first, second))
        self.ready_pub.publish(Bool(data=not self.fatal and not self.stop_event.is_set()
                                   and self.worker.is_alive() and discovered))
        self.status_pub.publish(String(data=json.dumps(_json(report), allow_nan=False)))
        self.enqueue('tick', self.get_clock().now().nanoseconds)

    def event(self, kind, **details):
        record = dict(event=kind, host_time_ns=self.get_clock().now().nanoseconds, **details)
        with self.event_lock:
            self.events.write(json.dumps(_json(record), allow_nan=False)+'\n')
            self.events.flush()
            os.fsync(self.events.fileno())

    def write_json(self, filename, value):
        target = self.directory/filename
        temporary = target.with_suffix(target.suffix+'.tmp')
        with temporary.open('w') as stream:
            json.dump(_json(value), stream, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)

    def work(self):
        try:
            self.event('backend_started', parameters=self.p,
                       graph_source='RTAB-Map RGB place recognition + VisIcp; no visual odometry')
            while not self.stop_event.is_set() or not self.jobs.empty():
                try:
                    kind, payload = self.jobs.get(timeout=.2)
                except queue.Empty:
                    continue
                try:
                    if self.fatal:
                        if not self.fatal_recorded:
                            self.event('fatal', error=self.fatal)
                            self.fatal_recorded = True
                        if kind == 'save':
                            self.finish_save(payload, False, self.fatal)
                        continue
                    if kind == 'scan':
                        self.handle_scan(*payload)
                    elif kind == 'keyframe':
                        self.handle_keyframe(*payload)
                    elif kind == 'camera':
                        self.handle_camera(*payload)
                    elif kind == 'graph':
                        self.pending_graph = payload
                        self.apply_pending_graph()
                    elif kind == 'graph_notice':
                        with self.state_lock:
                            self.pending_graph = self.graph_mailbox
                            self.graph_mailbox = None
                            self.graph_notice_pending = False
                        self.apply_pending_graph()
                    elif kind == 'full_graph':
                        self.map_query = None
                        self.handle_full_graph(payload)
                    elif kind == 'nodes':
                        self.cache_nodes(payload)
                        self.node_query = None
                        self.apply_pending_graph()
                    elif kind == 'info':
                        self.handle_info(payload)
                    elif kind == 'tick':
                        self.gate.expire_before(payload-2_000_000_000)
                        self.dispatch()
                        self.apply_pending_graph()
                        self.publish_map()
                    elif kind == 'save':
                        self.perform_save(payload)
                    self.update_report()
                except Exception as error:
                    self.fail(f'{kind}: {error}')
                    if kind == 'save':
                        self.finish_save(payload, False, str(error))
                finally:
                    self.jobs.task_done()
        finally:
            self.event('journal_worker_stopped', error=self.fatal, graph=self.graph.report())

    @staticmethod
    def check_scan(odom, cloud):
        stamp = stamp_ns(odom.header)
        if (stamp != stamp_ns(cloud.header) or odom.header.frame_id != 'gicp_odom' or
                odom.child_frame_id != 'base_link' or cloud.header.frame_id != 'base_link'):
            raise ValueError('GICP inputs require one stamp, gicp_odom->base_link pose and body-frame local cloud')
        return stamp, pose_from_message(odom.pose.pose)

    def handle_scan(self, odom, cloud):
        stamp, pose = self.check_scan(odom, cloud)
        if stamp in self.accepted_poses:
            raise ValueError('duplicate accepted GICP scan timestamp')
        self.graph.record_pose(stamp, pose)
        self.accepted_stamps.append(stamp)
        self.accepted_poses[stamp] = pose
        self.poses_file.write(json.dumps(dict(stamp_ns=stamp, pose=pose.tolist()))+'\n')
        self.poses_file.flush()
        os.fsync(self.poses_file.fileno())
        self.latest_odom = odom
        self.gate.add_scan(stamp, (odom, cloud))
        self.flush_keyframes()
        self.dispatch()
        self.apply_pending_graph()
        matrix = self.graph.correction @ pose
        if rclpy.ok() and not self.stop_event.is_set():
            self.registered_pub.publish(xyz_message(Header(stamp=cloud.header.stamp, frame_id='map'),
                                                   transform_points(cloud_xyz(cloud), matrix)))
        self.publish_transform(odom.header.stamp)

    def handle_keyframe(self, odom, cloud):
        stamp, _ = self.check_scan(odom, cloud)
        if stamp in self.pending_keyframes:
            raise ValueError('duplicate pending mapping keyframe')
        self.pending_keyframes[stamp] = (odom, cloud)
        if len(self.pending_keyframes) > 100:
            raise ValueError('mapping keyframes are missing accepted GICP odometry')
        self.flush_keyframes()

    def flush_keyframes(self):
        while self.pending_keyframes:
            stamp = min(self.pending_keyframes)
            if stamp not in self.accepted_poses:
                break
            odom, message = self.pending_keyframes[stamp]
            pose = pose_from_message(odom.pose.pose)
            if not np.allclose(pose, self.accepted_poses[stamp], rtol=0, atol=1e-10):
                raise ValueError('mapping keyframe disagrees with accepted GICP odometry')
            points = cloud_xyz(message)
            target = self.keyframes_dir/f'{stamp}.npz'
            if target.exists():
                raise ValueError('keyframe journal timestamp already exists')
            temporary = target.with_suffix('.npz.tmp')
            with temporary.open('xb') as stream:
                np.savez(stream, stamp_ns=np.int64(stamp), points=points, pose=pose)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
            self.graph.append(stamp, points, pose)
            del self.pending_keyframes[stamp]
            self.event('keyframe_retained', stamp_ns=stamp, points=len(points), file=str(target.relative_to(self.directory)))
        self.publish_map()

    def handle_camera(self, image, info):
        stamp = stamp_ns(image.header)
        if (stamp != stamp_ns(info.header) or image.header.frame_id != info.header.frame_id or
                image.width != info.width or image.height != info.height):
            raise ValueError('rectified image and calibrated CameraInfo disagree')
        self.gate.add_camera(stamp, (image, info))
        self.dispatch()

    def dispatch(self):
        if self.stop_event.is_set() or not rclpy.ok():
            return
        for stamp, (odom, cloud), camera_stamp, (image, info) in self.gate.ready():
            if self.fatal:
                return
            image, info = deepcopy(image), deepcopy(info)
            image.header.stamp = deepcopy(odom.header.stamp)
            info.header.stamp = deepcopy(odom.header.stamp)
            self.event('backend_input', scan_stamp_ns=stamp, exposure_stamp_ns=camera_stamp,
                       association_skew_sec=abs(stamp-camera_stamp)/1e9,
                       timing_uncertainty_sec=self.gate.uncertainty_sec,
                       association='bounded timestamp association; not hardware synchronization')
            self.inputs['odom'].publish(odom)
            self.inputs['cloud'].publish(cloud)
            self.inputs['info'].publish(info)
            self.inputs['image'].publish(image)
            self.last_dispatched_stamp = stamp

    def nearest_source_stamp(self, seconds):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('invalid RTAB-Map node timestamp')
        value = round(seconds*1e9)
        index = bisect_left(self.accepted_stamps, value)
        candidates = self.accepted_stamps[max(0, index-1):index+1]
        if not candidates:
            return None
        closest = min(candidates, key=lambda stamp: abs(stamp-value))
        # RTAB-Map Node.stamp is float64 seconds (~0.24us ULP in 2026).
        return closest if abs(closest-value) <= 1000 else None

    def cache_nodes(self, nodes):
        for node in nodes:
            if node.id <= 0:
                continue
            stamp = self.nearest_source_stamp(node.stamp)
            if stamp is None:
                raise ValueError(f'RTAB-Map node {node.id} has no exact accepted GICP timestamp')
            # Core Transform stores float32, including translations. Comparing
            # directly to float64 with fixed 10um tolerance rejects legitimate
            # poses past 256m (one float32 ULP is already 30um there).
            quantized_source = self.accepted_poses[stamp].astype(np.float32).astype(np.float64)
            if not np.allclose(pose_from_message(node.pose), quantized_source, rtol=0, atol=1e-5):
                raise ValueError(f'RTAB-Map node {node.id} odometry differs from GICP source')
            if node.id in self.node_stamps and self.node_stamps[node.id] != stamp:
                raise ValueError('RTAB-Map reused a node ID with a different timestamp; fresh database required')
            self.node_stamps[node.id] = stamp

    def request_nodes(self, ids):
        if self.node_query is not None or not self.node_client.service_is_ready():
            return
        request = GetNodeData.Request(ids=sorted(ids), images=False, scan=False, grid=False, user_data=False)
        self.node_query = self.node_client.call_async(request)
        def completed(future):
            try:
                self.enqueue('nodes', future.result().data)
            except Exception as error:
                self.fail('cannot retrieve RTAB-Map node timestamp metadata: '+str(error))
        self.node_query.add_done_callback(completed)

    def request_complete_graph(self):
        if (self.map_query is not None or time.monotonic()-self.last_map_query < 2. or
                not self.map_client.service_is_ready()):
            return
        self.last_map_query = time.monotonic()
        request = GetMap.Request(global_map=True, optimized=True, graph_only=False)
        self.map_query = self.map_client.call_async(request)
        def completed(future):
            try:
                self.enqueue('full_graph', future.result().data)
            except Exception as error:
                self.fail('cannot retrieve complete optimized RTAB-Map graph: '+str(error))
        self.map_query.add_done_callback(completed)

    @staticmethod
    def graph_ref(message):
        return max((value for value in message.graph.poses_id if value > 0), default=0) if message is not None else 0

    def handle_full_graph(self, message):
        # A service response may describe the graph at an earlier instant than
        # a newer topic graph already waiting for its node timestamp metadata.
        # Use any metadata it contains, but never discard that newer graph.
        # Fresh-session generated positive IDs are monotonic in this config.
        self.cache_nodes(message.nodes)
        with self.state_lock:
            candidates = (self.pending_graph, self.graph_mailbox)
            newest = max(candidates, key=self.graph_ref)
        if self.graph_ref(newest) > self.graph_ref(message):
            self.pending_graph = newest
            self.event('full_graph_metadata_only', response_ref_id=self.graph_ref(message),
                       retained_pending_ref_id=self.graph_ref(newest))
        else:
            self.pending_graph = message
        self.apply_pending_graph()

    def apply_pending_graph(self):
        message = self.pending_graph
        if message is None:
            return
        self.cache_nodes(message.nodes)
        if len(message.graph.poses_id) != len(message.graph.poses):
            raise ValueError('RTAB-Map graph ID/pose lengths disagree')
        missing = {node_id for node_id in message.graph.poses_id if node_id > 0 and node_id not in self.node_stamps}
        if missing:
            self.pending_reason = 'waiting for RTAB-Map node timestamp metadata: '+str(sorted(missing))
            self.request_nodes(missing)
            return
        updates = {self.node_stamps[node_id]: pose_from_message(pose)
                   for node_id, pose in zip(message.graph.poses_id, message.graph.poses) if node_id > 0}
        if not updates:
            self.pending_reason = 'waiting for a nonempty optimized RTAB-Map graph'
            return
        source_stamp = max(updates)
        # get_map_data uses wall time in its response header, whereas the live
        # mapData publisher uses sensor time. Ordering those headers can reject
        # later sensor graphs indefinitely after a service recovery. Compare
        # the actual original GICP measurement represented by the newest node.
        if source_stamp < self.last_graph_source_stamp:
            self.event('stale_graph_ignored', source_stamp_ns=source_stamp,
                       latest_graph_source_stamp_ns=self.last_graph_source_stamp)
            self.pending_graph, self.pending_reason = None, ''
            return
        omitted = set(self.retained_graph)-set(updates)
        self.omitted_graph_anchors = omitted
        if omitted:
            # A partial working-memory graph cannot establish corrected poses
            # for the complete historical map. Keep the previous committed
            # map intact and request the actual global optimized graph.
            self.pending_reason = 'incomplete graph; waiting for global optimized poses for '+str(len(omitted))+' retained anchors'
            self.request_complete_graph()
            return
        complete = updates
        proposed = self.graph.prepare_graph(complete)
        ref_id = max((node_id for node_id in message.graph.poses_id if node_id > 0), default=0)
        info = self.info_by_ref.get(ref_id)
        graph_record = dict(graph_revision=proposed.revision,
                            graph_source_stamp_ns=source_stamp,
                            optimized_poses={str(stamp): pose.tolist() for stamp, pose in complete.items()},
                            node_stamps={str(node_id): stamp for node_id, stamp in self.node_stamps.items()},
                            omitted_retained_anchors=sorted(omitted),
                            links=[dict(from_id=link.from_id, to_id=link.to_id, type=link.type,
                                        transform=pose_matrix(
                                            [link.transform.translation.x, link.transform.translation.y, link.transform.translation.z],
                                            [link.transform.rotation.x, link.transform.rotation.y,
                                             link.transform.rotation.z, link.transform.rotation.w]).tolist(),
                                        information=list(link.information)) for link in message.graph.links])
        self.write_json('graph_pending.json', graph_record)
        self.graph.commit_graph(proposed)
        os.replace(self.directory/'graph_pending.json', self.directory/'graph_latest.json')
        self.retained_graph = complete
        self.omitted_graph_anchors = omitted
        self.last_graph_source_stamp = source_stamp
        links = {tuple(sorted((link.from_id, link.to_id))) for link in message.graph.links
                 if link.type == 1 and link.from_id > 0 and link.to_id > 0}
        added = sorted(links-self.links)
        removed = sorted(self.links-links)
        self.links = links
        self.pending_graph, self.pending_reason = None, ''
        self.event('graph_applied', graph_revision=proposed.revision,
                   global_loop_constraints_added=added, global_loop_constraints_removed=removed,
                   global_loop_constraints=len(links),
                   geometric_validation='RTAB-Map VisIcp accepted link type=1' if added else 'no new global loop constraint',
                   icp_score=None, icp_score_note='not exposed per loop in the ROS Info/MapData interface',
                   graph_ref_id=ref_id, info_ref_id=ref_id if info is not None else None,
                   optimization_iterations=info['optimization_iterations'] if info is not None else None,
                   optimization_note='match loop_candidate events by ref_id; Info can arrive after MapData',
                   map_rebuilt=proposed.rebuilt, graph=self.graph.report(),
                   omitted_retained_anchors=sorted(omitted))
        self.publish_map(force=proposed.rebuilt)
        if self.latest_odom is not None:
            self.publish_transform(self.latest_odom.header.stamp)

    def handle_info(self, message):
        if len(message.stats_keys) != len(message.stats_values):
            raise ValueError('RTAB-Map Info statistic keys/values disagree')
        stats = dict(zip(message.stats_keys, message.stats_values))
        def statistic(prefix):
            return next((float(value) for key, value in stats.items() if key.startswith(prefix+'/')), None)
        self.optimization_iterations = statistic('Loop/Optimization_iterations')
        self.info_by_ref[message.ref_id] = {'optimization_iterations': self.optimization_iterations}
        if len(self.info_by_ref) > 1024:
            del self.info_by_ref[min(self.info_by_ref)]
        self.event('loop_candidate', ref_id=message.ref_id, loop_result_id=message.loop_closure_id,
                   highest_hypothesis_id=statistic('Loop/Highest_hypothesis_id'),
                   highest_hypothesis_score=statistic('Loop/Highest_hypothesis_value'),
                   rejected_hypothesis=statistic('Loop/RejectedHypothesis'),
                   visual_inliers=statistic('Loop/Visual_inliers'),
                   optimization_iterations=self.optimization_iterations,
                   optimization_executed=(self.optimization_iterations > 0
                                          if self.optimization_iterations is not None else None),
                   constraint_evidence='see graph_applied link type=1; candidate alone is not a constraint',
                   statistics=stats)

    def publish_transform(self, stamp):
        if self.stop_event.is_set() or not rclpy.ok():
            return
        transform = TransformStamped()
        transform.header = Header(stamp=stamp, frame_id='map')
        transform.child_frame_id = 'gicp_odom'
        matrix = self.graph.correction
        q = quaternion_from_matrix(matrix)
        t, r = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = map(float, matrix[:3, 3])
        r.x, r.y, r.z, r.w = map(float, q)
        self.transforms.sendTransform(transform)

    def publish_map(self, force=False):
        if self.stop_event.is_set() or not rclpy.ok():
            return
        if not force and time.monotonic()-self.last_map_publish < self.p['map_publish_period']:
            return
        snapshot = self.graph.snapshot()
        if len(snapshot.points) and (force or snapshot.map_revision != self.last_published_revision):
            self.map_pub.publish(xyz_message(Header(stamp=self.get_clock().now().to_msg(), frame_id='map'), snapshot.points))
            self.last_published_revision = snapshot.map_revision
            self.last_map_publish = time.monotonic()

    def update_report(self):
        state = ('waiting_for_graph' if not self.retained_graph else
                 'waiting_for_graph_metadata' if self.pending_graph is not None else 'tracking')
        with self.state_lock:
            self.report_cache = dict(state='saving' if self.save_active else state,
                save_active=self.save_active, enabled=True, backend='rtabmap_vis_icp',
                session_directory=str(self.directory), error=self.fatal, graph=self.graph.report(),
                association=self.gate.report(), global_loop_constraints=len(self.links),
                optimization_iterations=self.optimization_iterations,
                pending_keyframes=len(self.pending_keyframes), pending_graph=self.pending_reason,
                omitted_retained_anchors=len(self.omitted_graph_anchors),
                last_dispatched_stamp_ns=self.last_dispatched_stamp,
                exact_pair_pending={name: len(values) for name, values in self.pair_pending.items()})

    @staticmethod
    def finish_save(request, success, message):
        request.success, request.message = bool(success), str(message)
        request.event.set()

    def perform_save(self, request):
        with request.lock:
            if request.cancelled:
                return
            request.started = True
        with self.state_lock:
            pending_inputs = any(self.pair_pending[name] for name in ('scan', 'keyframe'))
            if self.save_active:
                self.finish_save(request, False, 'A map checkpoint export is already running; wait for its result')
                return
        graph_report = self.graph.report()
        if (self.fatal or pending_inputs or self.pending_keyframes or self.pending_graph is not None or
                self.omitted_graph_anchors or
                not self.graph.ready_for_save() or
                (graph_report['last_graph_stamp_ns'] or 0) < self.last_dispatched_stamp):
            self.finish_save(request, False, 'Graph/input processing is incomplete; keep the robot stationary, wait for graph coverage, then save. '+json.dumps(self.report_cache))
            return
        self.event('map_save_started', graph=graph_report)
        with self.state_lock:
            self.save_active = True
            self.report_cache = dict(self.report_cache, state='saving')
        snapshot = self.graph.snapshot()
        report = dict(enabled=True, backend='rtabmap_vis_icp', **graph_report,
                      global_loop_constraints=len(self.links),
                      session_directory=str(self.directory),
                      validation='graph replay applied; real closure quality requires dataset/structure inspection')
        # This immutable snapshot is a precise checkpoint. Continue recording
        # new observations/graphs while the large PCD/grid export is running;
        # a synchronous disk export could otherwise overflow the input queue.
        thread = threading.Thread(target=self.export_snapshot, args=(request, snapshot, report),
                                  name='rtabmap-checkpoint-export')
        self.export_threads.append(thread)
        thread.start()

    def export_snapshot(self, request, snapshot, report):
        success, message = False, 'Map export did not complete'
        try:
            directory = save_snapshot(snapshot.points, snapshot.free, snapshot.start_pose, self.p, report,
                                      'GICP local frontend with RTAB-Map camera-assisted VisIcp pose graph')
            self.event('map_saved', directory=str(directory), map_revision=snapshot.map_revision,
                       graph_revision=snapshot.graph_revision)
            success, message = True, str(directory)
        except Exception as error:
            message = str(error)
            try:
                self.event('map_save_failed', error=message)
            except Exception as journal_error:
                self.fail('cannot journal map export failure: '+str(journal_error))
        finally:
            with self.state_lock:
                self.save_active = False
            self.finish_save(request, success, message)

    def save(self, unused_request, response):
        request = SaveRequest()
        if not self.enqueue('save', request):
            response.success, response.message = False, self.fatal or 'backend stopping'
            return response
        if not request.event.wait(60.):
            with request.lock:
                if not request.started:
                    request.cancelled = True
                    response.success, response.message = False, 'Save did not start within 60s; request cancelled without publishing a map'
                    return response
            # Once export starts, report its actual outcome. Returning a timeout
            # failure while that same job later selects a new map is misleading.
            request.event.wait()
        response.success, response.message = request.success, request.message
        return response

    def destroy_node(self):
        if hasattr(self, 'worker'):
            self.stop_event.set()
            try:
                if rclpy.ok():
                    self.ready_pub.publish(Bool(data=False))
            except Exception:
                pass
            self.worker.join()
            for thread in self.export_threads:
                thread.join()
            try:
                self.event('backend_stopped', error=self.fatal, graph=self.graph.report())
            finally:
                self.events.close()
                self.poses_file.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = RtabmapBackend()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
