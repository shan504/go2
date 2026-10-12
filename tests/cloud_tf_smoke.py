"""Real ROS cloud-before-TF regression; synthetic sensors, no motion topics.

Run with DDS isolated from robot hardware, for example Docker --network none.
"""
import copy
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from mapper import Mapper
from test_3d import room


def wait_for(predicate, seconds=5):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('Cloud/TF regression condition timed out')


def stamp_ns(message):
    return Time.from_msg(message.header.stamp).nanoseconds


class Sensors(Node):
    def __init__(self):
        super().__init__('cloud_before_tf_sensors')
        self.cloud = self.create_publisher(PointCloud2, '/point_cloud2', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.valid = None
        self.valid_events = []
        def receive_valid(message):
            self.valid = message.data
            self.valid_events.append(message.data)
        self.create_subscription(Bool, '/localization/valid',
                                 receive_valid, 10)
        self.points = room().astype(np.float32).tolist()

    def packet(self, stamp=None):
        stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        return point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='base_link'), self.points)

    def odometry_tf(self, message):
        transform = TransformStamped()
        transform.header = Header(stamp=message.header.stamp, frame_id='odom')
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)


def check(directory):
    rclpy.init(args=['--ros-args', '-p', 'mode:=localization',
                    '-p', f'map_directory:={directory}', '-p', 'auto_initialize:=true',
                    '-p', 'registration_period:=0.2'])
    mapper, sensors = Mapper(), Sensors()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mapper, sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def pending_received(message):
        with mapper.lock:
            return next((received for packet, received in mapper.pending_clouds
                         if stamp_ns(packet) == stamp_ns(message)), None)

    def latest_is(message):
        with mapper.lock:
            return mapper.latest is not None and stamp_ns(mapper.latest[0]) == stamp_ns(message)

    def cloud_then_tf():
        message = sensors.packet()
        sensors.cloud.publish(message)
        wait_for(lambda: pending_received(message) is not None)
        received = pending_received(message)
        time.sleep(0.07)  # TF intentionally arrives after the cloud callback.
        sensors.odometry_tf(message)
        wait_for(lambda: latest_is(message))
        with mapper.lock:
            assert mapper.latest[2] == received, 'TF retry relabeled the original receipt as fresh'
            assert stamp_ns(mapper.latest[0]) == stamp_ns(message), 'TF retry changed the cloud stamp'
        return message, received

    try:
        wait_for(lambda: sensors.cloud.get_subscription_count() > 0 and sensors.valid is False)
        first, received = cloud_then_tf()
        wait_for(lambda: sensors.valid is True)
        assert mapper.last_accepted == received
        assert len(mapper.history) == 1, 'First cloud was accepted more than once'
        # Sustained cloud-first delivery must keep localizing without replay.
        valid_start = len(sensors.valid_events)
        for _ in range(10):
            last, _ = cloud_then_tf()
        wait_for(lambda: sensors.valid is True and mapper.last_accepted == mapper.latest[2])
        assert sensors.valid_events[valid_start:] and all(sensors.valid_events[valid_start:]), (
            'Fresh cloud-first stream caused a localization validity interruption')
        assert len(mapper.history) == 11

        # Duplicates and older clouds cannot enter the pending queue or replace
        # newer accepted sensor data, even while their timestamps are fresh.
        count = len(mapper.history)
        duplicates = mapper.cloud_counts['duplicate']
        sensors.cloud.publish(last)
        older = copy.deepcopy(last)
        older.header.stamp = Time(nanoseconds=stamp_ns(last)-1_000_000).to_msg()
        sensors.cloud.publish(older)
        wait_for(lambda: mapper.cloud_counts['duplicate'] == duplicates+2)
        assert latest_is(last) and len(mapper.history) == count
        assert not mapper.pending_clouds

        # Hold only Mapper's host-age clock at a real ROS timestamp, avoiding
        # scheduler-dependent timing at the one-second boundary. Actual ROS
        # delivery/timers and monotonic receipt age continue to advance. The
        # slightly future packet remains host-fresh but must still expire.
        get_clock = mapper.get_clock
        held_now = get_clock().now()
        held_clock = SimpleNamespace(now=lambda: held_now)
        missing = sensors.packet(Time(nanoseconds=held_now.nanoseconds+80_000_000).to_msg())
        mapper.get_clock = lambda: held_clock
        try:
            sensors.cloud.publish(missing)
            wait_for(lambda: pending_received(missing) is not None)
            original_receipt = pending_received(missing)
            wait_for(lambda: not mapper.pending_clouds, seconds=2)
            assert time.monotonic()-original_receipt >= 1.0
            assert (mapper.get_clock().now()-Time.from_msg(missing.header.stamp)).nanoseconds == -80_000_000
            assert 'Dropped cloud waiting for odometry TF' in mapper.cloud_status
            assert mapper.cloud_counts['expired'] == 1
            wait_for(lambda: sensors.valid is False)
        finally:
            mapper.get_clock = get_clock
        sensors.odometry_tf(missing)
        wait_for(lambda: mapper.buffer.can_transform('odom', 'base_link', Time.from_msg(missing.header.stamp)))
        time.sleep(0.25)
        assert latest_is(last) and sensors.valid is False, 'Expired cloud became valid when late TF arrived'
        sensors.cloud.publish(missing)
        time.sleep(0.1)
        assert not mapper.pending_clouds and latest_is(last), 'Expired replay was accepted as fresh'

        # Verify both host-age bounds before any lookup or queuing.
        for offset in (-1_200_000_000, 200_000_000):
            message = sensors.packet(Time(nanoseconds=sensors.get_clock().now().nanoseconds+offset).to_msg())
            mapper.receive_cloud(message)
            assert not mapper.pending_clouds and 'outside [-0.1, 1.0]s' in mapper.cloud_status

        # TF2 rejects malformed quaternions at ingestion. Inject only this
        # lookup result to exercise Mapper's separate invalid-pose drop path;
        # delivery ordering and localization above use actual ROS/TF2.
        invalid = sensors.packet()
        lookup = mapper.buffer.lookup_transform
        bad_transform = TransformStamped()
        bad_transform.transform.rotation.w = 0.0  # invalid all-zero quaternion
        def invalid_lookup(parent, child, stamp):
            if stamp.nanoseconds == stamp_ns(invalid):
                return bad_transform
            return lookup(parent, child, stamp)
        mapper.buffer.lookup_transform = invalid_lookup
        try:
            sensors.cloud.publish(invalid)
            wait_for(lambda: 'invalid odom -> base_link transform: Invalid quaternion' in mapper.cloud_status)
            assert not mapper.pending_clouds and latest_is(last)
            assert mapper.cloud_counts['invalid_tf'] == 1
            assert sensors.valid is False, 'Invalid transform restored localization validity'
        finally:
            mapper.buffer.lookup_transform = lookup

        # Missing TF is bounded in memory as well as age. Invoke the real
        # callback directly for deterministic queue saturation with ROS msgs.
        mapper.registration_timer.cancel()
        with mapper.lock:
            base_stamp = mapper.get_clock().now().nanoseconds
            for index in range(40):
                message = PointCloud2(header=Header(frame_id='base_link',
                    stamp=Time(nanoseconds=base_stamp+index*1_000).to_msg()))
                mapper.receive_cloud(message)
            assert len(mapper.pending_clouds) == mapper.pending_clouds.maxlen == 32
            assert mapper.cloud_counts['queue_dropped'] == 8
        wait_for(lambda: not mapper.pending_clouds, seconds=2)
        assert latest_is(last) and sensors.valid is False
        print('PASS cloud/TF delivery: cloud-first asynchronous exact-stamp acceptance; '
              'sustained localization; original receipt expiry; duplicate/age rejection; '
              'invalid TF dropped; bounded missing-TF queue', flush=True)
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        for node in (mapper, sensors):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(room()))
        assert o3d.io.write_point_cloud(str(root/'map.pcd'), points)
        (root/'metadata.yaml').write_text('{}\n')
        check(root)
