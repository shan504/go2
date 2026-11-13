"""Real ROS persisted incident windows; synthetic evidence, no motor requests."""
import base64
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import time
import zlib

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data, qos_profile_action_status_default
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import Bool, Empty, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from navigation_diagnostics import create_node
from fault_store import create_session, list_faults, load_fault, export_fault


def main():
    rclpy.init()
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)/'diagnostics'
        session = create_session(root, {'mode': 'navigation', 'test': 'synthetic_read_only'})
        (session/'launch.log').write_text('startup context before the planner failure\n')
        diagnostics, probe = create_node(str(session)), Node('recorder_test_inputs')
        executor = SingleThreadedExecutor()
        executor.add_node(diagnostics)
        executor.add_node(probe)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        commands = [probe.create_publisher(Twist, topic, 10) for topic in ('/cmd_vel_nav', '/cmd_vel')]
        odom = probe.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        valid = probe.create_publisher(Bool, '/localization/valid', 1)
        gate = probe.create_publisher(String, '/control/status', retained)
        operator = probe.create_publisher(String, '/operator/status', retained)
        state = probe.create_publisher(String, '/localization/state', 1)
        plan = probe.create_publisher(NavPath, '/plan', 10)
        grid = probe.create_publisher(OccupancyGrid, '/local_costmap/costmap', retained)
        markers = probe.create_publisher(MarkerArray, '/navigation/waypoints', retained)
        logs = probe.create_publisher(Log, '/rosout', 10)
        actions = probe.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                                         qos_profile_action_status_default)
        cancel = probe.create_publisher(Empty, '/navigation/cancel', 1)
        tf = TransformBroadcaster(probe)
        fixed_tf = StaticTransformBroadcaster(probe)
        fixed = TransformStamped()
        fixed.header.frame_id, fixed.child_frame_id = 'base_link', 'base_footprint'
        fixed.transform.rotation.w = 1.0
        fixed_tf.sendTransform(fixed)
        tick = 0

        def inputs():
            nonlocal tick
            tick += 1
            stamp = probe.get_clock().now().to_msg()
            for index, publisher in enumerate(commands):
                velocity = Twist()
                velocity.linear.x = 0.3 if index == 0 else 0.2
                publisher.publish(velocity)
            sample = Odometry()
            sample.header.frame_id, sample.child_frame_id = 'odom', 'base_link'
            sample.header.stamp = stamp
            sample.pose.pose.position.x = tick*0.003
            sample.pose.pose.orientation.w = 1.0
            sample.twist.twist.linear.x = 0.06
            odom.publish(sample)
            valid.publish(Bool(data=True))
            gate.publish(String(data='gate=forwarding Move requests; enable_control=True; cmd=[0.2,0,0]'))
            state.publish(String(data='Localization state=tracking valid=True'))
            path = NavPath()
            path.header.frame_id, path.header.stamp = 'map', stamp
            first, target = PoseStamped(), PoseStamped()
            first.pose.orientation.w = target.pose.orientation.w = 1.0
            target.pose.position.x = 5.0
            path.poses = [first, target]
            plan.publish(path)
            map_message = OccupancyGrid()
            map_message.header.frame_id, map_message.header.stamp = 'odom', stamp
            map_message.info.width = map_message.info.height = 32
            map_message.info.resolution = 0.1
            map_message.info.origin.position.x = map_message.info.origin.position.y = -1.0
            map_message.info.origin.orientation.w = 1.0
            map_message.data = [0]*1024
            map_message.data[17] = -1
            grid.publish(map_message)
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'map', stamp
            marker.id, marker.text = 5, 'waypoint #5'
            marker.pose = target.pose
            markers.publish(MarkerArray(markers=[marker]))
            transforms = []
            for parent, child, x in (('map', 'odom', 1.0), ('odom', 'base_link', sample.pose.pose.position.x)):
                transform = TransformStamped()
                transform.header.frame_id, transform.header.stamp = parent, stamp
                transform.child_frame_id = child
                transform.transform.translation.x = x
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            tf.sendTransform(transforms)

        timer = probe.create_timer(0.05, inputs)

        def spin_until(predicate, timeout=8):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                executor.spin_once(timeout_sec=0.03)
            assert predicate(), 'fault recorder ROS condition timed out'

        def spin_for(seconds):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.03)

        def log(name, level, text):
            sample = Log()
            sample.name, sample.level, sample.msg = name, level, text
            sample.stamp = probe.get_clock().now().to_msg()
            logs.publish(sample)

        closed = False
        try:
            spin_until(lambda: tick >= 10 and diagnostics.recorder.writer.status()['queue_pending'] == 0)
            operator.publish(String(data='route current=5 queued=8 completed=4 paused=False started=True'))
            spin_until(lambda: diagnostics.history.executing)
            log('go2_gicp', 30, 'Waiting for odom -> base_link; retrying fresh packet: future transform')
            spin_for(0.3)
            assert not list_faults(root), 'benign sensor wait created a false fault'
            log('planner_server', 30, 'GridBased: failed to create plan with tolerance 0.25.')
            spin_until(lambda: len(list_faults(root)) == 1)
            first, evidence, _ = load_fault(root)
            first_id = first['fault_id']
            assert first['trigger']['code'] == 'planner_failed'
            assert first_id.startswith(session.name+'--')
            assert evidence['post_capture']['state'] == 'pending'
            assert any(event['source'] == 'cmd_vel' for event in evidence['events_before'])
            assert any(event['source'] == 'rosout' and 'failed to create plan' in event['data']['text']
                       for event in evidence['events_before'])
            # Let the initial post window complete before the eventual abort.
            spin_until(lambda: load_fault(root)[1]['post_capture']['state'] == 'complete', timeout=9)
            with (session/'launch.log').open('a') as output:
                output.write('Goal failed after bounded retries; late abort context\n')
            failure = ('Nav2 goal completed: action status=6 (ABORTED); waypoint #5; '
                       'route current=- queued=9 completed=4 paused=True started=False')
            operator.publish(String(data=failure))
            status = GoalStatus()
            status.goal_info.goal_id.uuid = [5]*16
            status.goal_info.stamp = probe.get_clock().now().to_msg()
            status.status = 6
            actions.publish(GoalStatusArray(status_list=[status]))
            spin_until(lambda: load_fault(root)[1].get('final_action', {}).get('status') == 6)
            cancel.publish(Empty())
            spin_until(lambda: any(item['code'] == 'manual_cancel'
                       for item in load_fault(root)[1]['trigger_timeline']))
            log('controller_server', 40, 'Failed to get result for follow_path in node halt!')
            spin_for(0.3)
            assert len(list_faults(root)) == 1, 'retries/abort/manual stop created duplicate fault files'
            publishers = {name for name, _ in diagnostics.get_publisher_names_and_types_by_node(
                diagnostics.get_name(), diagnostics.get_namespace())}
            assert not publishers & {'/cmd_vel', '/cmd_vel_nav', '/control/enable', '/goal_pose'}
            clients = diagnostics.get_client_names_and_types_by_node(diagnostics.get_name(), diagnostics.get_namespace())
            assert all(name.endswith('/get_state') or name.endswith('/get_parameters') for name, _ in clients)
            probe.destroy_timer(timer)
            diagnostics.destroy_node()
            closed = True
            first, evidence, manifest = load_fault(root)
            assert first['fault_id'] == first_id and first['trigger']['code'] == 'planner_failed'
            assert first['snapshot']['route']['paused'] == 'True'
            assert evidence['initial_snapshot']['route']['current'] == '5'
            assert evidence['events_after']
            assert evidence['later_trigger_context']
            assert evidence['final_action']['status'] == 6
            assert manifest['recorder_liveness']['active'] is False
            context = evidence['context']
            raw = context['local_grid']['raw_int8']
            decoded = zlib.decompress(base64.b64decode(raw['data']))
            assert len(decoded) == 1024 and decoded[17] == 255
            assert context['path']['total_points'] == 2
            assert context['waypoints']['markers'][0]['text'] == 'waypoint #5'
            assert context['transforms']['edges']['map -> base_footprint']['available']
            assert '/cmd_vel' in context['graph']
            assert evidence['coverage']['sources']['rosout']['available']
            archive = export_fault(root)
            with tarfile.open(archive) as exported:
                names = exported.getnames()
                assert 'fault/summary.json' in names and 'fault/evidence.json' in names
                assert 'session/manifest.json' in names and 'session/launch.log' in names
            print('PASS persisted recorder: immutable pre-window, post-window, same incident after retries/ABORT, '
                  'manual-stop distinction, TF/path/grid/rawROSout evidence, offline export after node exit')
        finally:
            executor.shutdown()
            if not closed:
                diagnostics.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
