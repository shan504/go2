"""Real DDS bridge clock regression with synthetic inputs, no robot connection.

Uses the genuine Unitree Request message type. Motion stays disabled throughout.
Only the bridge's host-clock reader is substituted for the explicit host-clock
jump case; all sensor delivery, output messages and TF use actual ROS endpoints.
"""
import copy
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from unitree_api.msg import Request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'patches/dds'))
sys.path.insert(0, str(ROOT/'go2_3d'))
from go2_edu_dds_bridge import Go2EduBridge, set_stamp, stamp_ns
from ros_cloud import xyz_message


def main():
    rclpy.init(args=['--ros-args', '-p', 'enable_control:=false'])
    bridge, source = Go2EduBridge(), Node('synthetic_native_clock_source')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    odom_pub = source.create_publisher(Odometry, '/utlidar/robot_odom', qos_profile_sensor_data)
    cloud_pub = source.create_publisher(PointCloud2, '/utlidar/cloud_base', qos_profile_sensor_data)
    odometry, clouds, transforms, requests = [], [], [], []
    inputs = {}
    source.create_subscription(Odometry, '/odom', odometry.append, 10)
    source.create_subscription(PointCloud2, '/point_cloud2', clouds.append, qos_profile_sensor_data)
    source.create_subscription(TFMessage, '/tf', lambda message: transforms.extend(
        tf for tf in message.transforms if tf.header.frame_id == 'odom' and
        tf.child_frame_id == 'base_link'), 100)
    source.create_subscription(Request, '/api/sport/request', requests.append, 10)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.01, max(0., deadline-time.monotonic())))

    def wait(predicate, seconds=5):
        deadline = time.monotonic()+seconds
        while not predicate():
            assert time.monotonic() < deadline, 'ROS clock regression timed out'
            executor.spin_once(timeout_sec=.01)

    def send(index, extra_ns=0, raw_ns=None):
        # Both native topics use one sensor epoch, with the cloud acquired
        # exactly 20 ms before the corresponding odometry sample.
        sensor_ns = (source.get_clock().now().nanoseconds-900_000_000_000+extra_ns
                     if raw_ns is None else raw_ns)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        set_stamp(odom.header.stamp, sensor_ns)
        odom.pose.pose.position.x = float(index)
        odom.pose.pose.position.y, odom.pose.pose.position.z = -.25, .45
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = .125
        cloud = xyz_message(Header(frame_id='base_link'),
                            np.array([[float(index), -2., .3], [.2, .4, -.1]]))
        set_stamp(cloud.header.stamp, sensor_ns-20_000_000)
        inputs[index] = (copy.deepcopy(odom), copy.deepcopy(cloud))
        odom_pub.publish(odom)
        cloud_pub.publish(cloud)
        return sensor_ns

    def cloud_index(message):
        return int(np.frombuffer(message.data, dtype='<f4')[0])

    original_clock = bridge.get_clock
    try:
        wait(lambda: odom_pub.get_subscription_count() and cloud_pub.get_subscription_count()
             and bridge.odom_pub.get_subscription_count() and bridge.cloud_pub.get_subscription_count())
        assert not bridge.gate.enabled
        send(0, extra_ns=-170_000_000)
        spin(.07)
        assert not odometry and not clouds and not transforms
        for index in range(1, 5):
            send(index)
            spin(.05)
        assert not odometry and not clouds and not transforms, 'Uncalibrated timestamps escaped'
        assert bridge.clock_map.convert(stamp_ns(inputs[4][0].header.stamp)) is None
        for index in range(5, 26):
            send(index)
            spin(.05)
        spin(.05)
        assert len(odometry) >= 10 and len(clouds) >= 10 and len(transforms) >= 10
        offset = bridge.clock_map.offset_ns
        assert offset is not None
        assert abs(offset-900_000_000_000) < 100_000_000, offset
        odom_by_index = {int(msg.pose.pose.position.x): msg for msg in odometry}
        cloud_by_index = {cloud_index(msg): msg for msg in clouds}
        tf_by_index = {int(msg.transform.translation.x): msg for msg in transforms}
        odom_stamps = [stamp_ns(msg.header.stamp) for msg in odometry]
        assert all(b > a for a, b in zip(odom_stamps, odom_stamps[1:]))
        cloud_stamps = [stamp_ns(msg.header.stamp) for msg in clouds]
        assert all(b > a for a, b in zip(cloud_stamps, cloud_stamps[1:]))
        common = set(odom_by_index) & set(cloud_by_index) & set(tf_by_index)
        assert len(common) >= 10
        for index in common:
            out, cloud, tf = odom_by_index[index], cloud_by_index[index], tf_by_index[index]
            native_odom, native_cloud = inputs[index]
            assert stamp_ns(out.header.stamp)-stamp_ns(native_odom.header.stamp) == offset
            assert stamp_ns(cloud.header.stamp)-stamp_ns(native_cloud.header.stamp) == offset
            assert stamp_ns(out.header.stamp)-stamp_ns(cloud.header.stamp) == 20_000_000
            assert stamp_ns(tf.header.stamp) == stamp_ns(out.header.stamp)
            assert out.pose == native_odom.pose and out.twist == native_odom.twist
            assert tf.transform.translation.x == out.pose.pose.position.x
            assert tf.transform.translation.y == out.pose.pose.position.y
            assert tf.transform.translation.z == out.pose.pose.position.z
            assert tf.transform.rotation == out.pose.pose.orientation
            assert bytes(cloud.data) == bytes(native_cloud.data)
            assert cloud.fields == native_cloud.fields and cloud.header.frame_id == 'base_link'
        latest_age = (source.get_clock().now().nanoseconds-odom_stamps[-1])/1e9
        assert -.1 <= latest_age < .3, latest_age
        print('PASS native DDS calibration: delayed first packet does not bias frozen offset; '
              'no output before calibration; genuine odom/cloud/TF preserve geometry, '
              'monotonic timestamps and 20 ms relative timing', flush=True)

        counts = len(odometry), len(clouds), len(transforms)
        latest_raw = stamp_ns(inputs[25][0].header.stamp)
        send(100, raw_ns=latest_raw)  # Replayed stamp with a conflicting pose.
        spin(.08)
        send(101, raw_ns=latest_raw-2_000_000_000)
        spin(.08)
        assert (len(odometry), len(clouds), len(transforms)) == counts, 'Replay was republished'
        assert bridge.clock_map.offset_ns == offset
        send(102)
        wait(lambda: any(msg.pose.pose.position.x == 102 for msg in odometry))
        spin(.03)
        counts = len(odometry), len(clouds), len(transforms)
        send(103, extra_ns=250_000_000)
        spin(.08)
        assert (len(odometry), len(clouds), len(transforms)) == counts, 'Future packet was published'
        assert bridge.clock_map.offset_ns == offset
        send(104)
        wait(lambda: any(msg.pose.pose.position.x == 104 for msg in odometry))
        spin(.03)
        assert bridge.clock_map.offset_ns == offset
        print('PASS rejected raw data: duplicate, old and >100 ms future packets do not '
              'refresh output; subsequent good data recovers without changing the offset', flush=True)

        counts = len(odometry), len(clouds), len(transforms)
        # Simulate a host wall-clock step while the monotonic clock and sensor
        # producer keep advancing. No system clock or sensor stamp is altered.
        shifted_clock = SimpleNamespace(now=lambda: Time(
            nanoseconds=original_clock().now().nanoseconds-250_000_000))
        bridge.get_clock = lambda: shifted_clock
        send(105)
        spin(.08)
        assert (len(odometry), len(clouds), len(transforms)) == counts
        bridge.get_clock = original_clock
        send(106)
        spin(.08)
        assert (len(odometry), len(clouds), len(transforms)) == counts, 'Clock fault silently reset'
        assert bridge.clock_map.convert(stamp_ns(inputs[106][0].header.stamp)) is None
        assert not bridge.gate.enabled and not requests, 'Test emitted a Sport request'
        print('PASS explicit host-clock jump: clock fault withholds sensor output even after '
              'clock restoration; disabled bridge emits no Sport requests. Synthetic ROS only.', flush=True)
    finally:
        bridge.get_clock = original_clock
        executor.shutdown()
        bridge.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
