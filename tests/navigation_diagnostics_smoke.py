"""Real ROS diagnostics topics and failure report; synthetic data, no motor API."""
from collections import deque
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from navigation_diagnostics import create_node, MAX_SAMPLES


def main():
    rclpy.init()
    diagnostics, probe = create_node(), Node('go2_diagnostics_test_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(diagnostics)
    executor.add_node(probe)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    nav = probe.create_publisher(Twist, '/cmd_vel_nav', 10)
    smooth = probe.create_publisher(Twist, '/cmd_vel', 10)
    odom = probe.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    valid = probe.create_publisher(Bool, '/localization/valid', 1)
    gate = probe.create_publisher(String, '/control/status', retained)
    operator = probe.create_publisher(String, '/operator/status', retained)
    plan = probe.create_publisher(NavPath, '/plan', 10)
    grid = probe.create_publisher(OccupancyGrid, '/local_costmap/costmap', retained)
    results = deque(maxlen=10)
    probe.create_subscription(String, '/navigation/control_diagnostics',
                              lambda msg: results.append(json.loads(msg.data)), retained)
    counter = 0

    def publish_inputs():
        nonlocal counter
        counter += 1
        stamp = probe.get_clock().now().to_msg()
        command = Twist()
        command.linear.x = 0.3 if (counter//10) % 2 else -0.2
        nav.publish(command)
        command.linear.x /= 2
        smooth.publish(command)
        sample = Odometry()
        sample.header.frame_id = 'odom'
        sample.header.stamp = stamp
        sample.child_frame_id = 'base_link'
        sample.pose.pose.position.x = (counter % 2)*0.01
        sample.pose.pose.orientation.w = 1.0
        odom.publish(sample)
        valid.publish(Bool(data=True))
        gate.publish(String(data='gate=forwarding Move requests; enable_control=True; cmd=[0.15,0,0]'))
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        target = PoseStamped()
        target.pose.position.x = 20.0
        target.pose.orientation.w = 1.0
        path.poses = [PoseStamped(), target]
        plan.publish(path)
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'odom'
        costmap.header.stamp = stamp
        costmap.info.width = costmap.info.height = 32
        costmap.info.resolution = 0.1
        costmap.info.origin.position.x = costmap.info.origin.position.y = -1.0
        costmap.info.origin.orientation.w = 1.0
        costmap.data = [0]*(32*32)
        grid.publish(costmap)

    timer = probe.create_timer(0.05, publish_inputs)

    def spin_until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), 'diagnostics ROS condition timed out'

    try:
        # No report is emitted for an idle/merely queued route.
        spin_until(lambda: counter >= 5)
        assert not results
        operator.publish(String(data='route current=1 queued=2 completed=0 paused=False started=True'))
        spin_until(lambda: bool(results))
        result = results[-1]
        assert result['reason'] == 'periodic', result
        assert result['commands']['cmd_vel_nav']['forward'] > 0
        assert result['commands']['cmd_vel_nav']['reverse'] > 0
        assert result['commands']['cmd_vel_nav']['vx_sign_changes'] > 0
        assert result['commands']['cmd_vel']['max'][0] == 0.15
        assert result['odometry']['available'] and result['odometry']['net_displacement_m'] <= 0.011
        assert result['latest']['localization_valid']['text'] == 'True'
        assert result['latest']['control']['text'].startswith('gate=forwarding')
        assert result['plan']['points'] == 2 and result['plan']['target'] == [20.0, 0.0]
        assert result['local_costmap']['available'] and result['local_costmap']['cells'] == 1024
        assert all(sample['value'] == 0 for sample in result['local_costmap']['heading_probes'])
        failure = ('Nav2 goal completed: action status=6 (ABORTED); waypoint #1; '
                   'route current=- queued=3 completed=0 paused=True started=False')
        operator.publish(String(data=failure))
        spin_until(lambda: results[-1]['reason'] == 'navigation_failure', timeout=1)
        count = len(results)
        operator.publish(String(data=failure))
        deadline = time.monotonic()+4.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert len(results) == count, 'paused route produced repeated failure/periodic reports'
        assert all(len(samples) <= MAX_SAMPLES for samples in diagnostics.history.commands.values())
        assert len(diagnostics.history.odometry) <= MAX_SAMPLES
        publishers = {name for name, _ in diagnostics.get_publisher_names_and_types_by_node(
            diagnostics.get_name(), diagnostics.get_namespace())}
        assert not publishers & {'/cmd_vel', '/cmd_vel_nav', '/control/enable', '/goal_pose'}
        print('PASS read-only diagnostics: active-route summary, both velocity streams, '
              'odometry/heading probes, immediate deduplicated failure report and bounded history')
    finally:
        probe.destroy_timer(timer)
        executor.shutdown()
        diagnostics.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
