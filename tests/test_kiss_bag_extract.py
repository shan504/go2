"""Offline conversion contract, including an actual generated ROS 2 SQLite bag.

The generated messages are synthetic; this does not validate robot timestamps,
LiDAR quality, GICP odometry accuracy or farm loop closure.
"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from kiss_bag_extract import DEFAULT_CLOUD, DEFAULT_ODOM, extract_bag, extract_messages


def odom(stamp, x=0., frame='gicp_odom'):
    return NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9), frame_id=frame),
              child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=x, y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))))


def cloud(stamp, points=None, frame='base_link'):
    points = np.array([[1., 2., .3], [2., 3., .4]] if points is None else points, dtype='<f4')
    return NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9), frame_id=frame),
              fields=[NS(name=name, datatype=7, offset=4*i, count=1) for i, name in enumerate('xyz')],
              width=len(points), height=1, point_step=12, row_step=12*len(points),
              is_bigendian=False, data=points.tobytes())


def pairs():
    return [(DEFAULT_CLOUD, cloud(10**9)), (DEFAULT_ODOM, odom(10**9)),
            (DEFAULT_ODOM, odom(2*10**9, 1.)), (DEFAULT_CLOUD, cloud(2*10**9))]


def digest_files(directory):
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in directory.rglob('*') if path.is_file()}


class ExtractMessagesTests(unittest.TestCase):
    def test_exact_pairs_out_of_order_identical_duplicates_and_finite_filter(self):
        messages = pairs()+[(DEFAULT_ODOM, odom(10**9)), (DEFAULT_CLOUD, cloud(10**9)),
                            (DEFAULT_ODOM, odom(3*10**9, 2.)),
                            (DEFAULT_CLOUD, cloud(3*10**9, [[1., 1., 1.], [float('nan'), 0., 0.]]))]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'journal'
            result = extract_messages(messages, output, provenance='synthetic')
            self.assertEqual(result['extraction']['paired_frames'], 3)
            self.assertEqual(result['extraction']['identical_duplicates'], 2)
            self.assertEqual(result['extraction']['nonfinite_points_removed'], 1)
            self.assertEqual(result['pose_convention'], 'uncorrected_gicp')
            self.assertEqual(json.loads((output/'session.json').read_text()), result)
            with np.load(output/'keyframes/3000000000.npz', allow_pickle=False) as frame:
                self.assertEqual(frame['stamp_ns'], 3*10**9)
                self.assertEqual(frame['points'].shape, (1, 3))
                self.assertEqual(frame['pose'][0, 3], 2.)

    def test_conflicting_duplicate_mismatched_stamp_and_frames_fail_closed(self):
        variants = [
            pairs()+[(DEFAULT_ODOM, odom(10**9, 9.))],
            pairs()+[(DEFAULT_CLOUD, cloud(10**9, [[9., 8., 7.]]))],
            pairs()[:-1]+[(DEFAULT_CLOUD, cloud(2*10**9+1))],
            pairs()[:-1]+[(DEFAULT_CLOUD, cloud(2*10**9, frame='map'))],
            [(DEFAULT_ODOM, odom(10**9, frame='odom'))]+pairs()[1:],
        ]
        for messages in variants:
            with self.subTest(messages=messages), tempfile.TemporaryDirectory() as directory:
                output = Path(directory)/'journal'
                with self.assertRaises(ValueError):
                    extract_messages(messages, output)
                self.assertFalse((output/'session.json').exists())
                self.assertFalse(json.loads((output/'FAILED.json').read_text())['complete'])

    def test_custom_raw_odom_is_not_silently_labelled_gicp(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'journal'
            with self.assertRaisesRegex(ValueError, 'continuous, uncorrected GICP'):
                extract_messages([], output, odom_topic='/odom', cloud_topic='/point_cloud2', odom_frame='odom')
            self.assertFalse(output.exists())

    def test_existing_output_remains_byte_identical(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'journal'
            extract_messages(pairs(), output)
            before = digest_files(output)
            with self.assertRaises(FileExistsError):
                extract_messages(pairs(), output)
            self.assertEqual(digest_files(output), before)

    def test_empty_single_invalid_timestamp_and_invalid_quaternion_rejected(self):
        bad_stamp = odom(10**9)
        bad_stamp.header.stamp.nanosec = 10**9
        bad_quaternion = odom(10**9)
        bad_quaternion.pose.pose.orientation.w = 0.
        variants = [[], pairs()[:2], [(DEFAULT_ODOM, bad_stamp)], [(DEFAULT_ODOM, bad_quaternion)]]
        for messages in variants:
            with self.subTest(messages=messages), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ValueError):
                    extract_messages(messages, Path(directory)/'journal')


class ActualRosbagTests(unittest.TestCase):
    def test_generated_ros2_bag_roundtrip_is_read_only(self):
        try:
            from rosbags.rosbag2 import Writer
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            self.skipTest('isolated rosbags dependency is not installed')
        ts = get_typestore(Stores.ROS2_HUMBLE)
        t = ts.types
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag = root/'synthetic_bag'
            with Writer(bag, version=9) as writer:
                connections = {topic: writer.add_connection(topic, typ, typestore=ts)
                               for topic, typ in [(DEFAULT_ODOM, 'nav_msgs/msg/Odometry'),
                                                  (DEFAULT_CLOUD, 'sensor_msgs/msg/PointCloud2')]}
                for i in range(3):
                    stamp = t['builtin_interfaces/msg/Time'](sec=10+i, nanosec=123)
                    position = t['geometry_msgs/msg/Point'](x=float(i), y=0., z=0.)
                    quaternion = t['geometry_msgs/msg/Quaternion'](x=0., y=0., z=0., w=1.)
                    pose = t['geometry_msgs/msg/Pose'](position=position, orientation=quaternion)
                    v = t['geometry_msgs/msg/Vector3'](x=0., y=0., z=0.)
                    twist = t['geometry_msgs/msg/Twist'](linear=v, angular=v)
                    message = t['nav_msgs/msg/Odometry'](
                        header=t['std_msgs/msg/Header'](stamp=stamp, frame_id='gicp_odom'),
                        child_frame_id='base_link',
                        pose=t['geometry_msgs/msg/PoseWithCovariance'](pose=pose, covariance=np.zeros(36)),
                        twist=t['geometry_msgs/msg/TwistWithCovariance'](twist=twist, covariance=np.zeros(36)))
                    fields = [t['sensor_msgs/msg/PointField'](name=name, offset=4*j, datatype=7, count=1)
                              for j, name in enumerate('xyz')]
                    points = np.array([[1., 2., 3.], [4., 5., 6.]], dtype='<f4')
                    scan = t['sensor_msgs/msg/PointCloud2'](
                        header=t['std_msgs/msg/Header'](stamp=stamp, frame_id='base_link'),
                        height=1, width=2, fields=fields, is_bigendian=False,
                        point_step=12, row_step=24, data=np.frombuffer(points.tobytes(), dtype=np.uint8),
                        is_dense=True)
                    for topic, msg in [(DEFAULT_CLOUD, scan), (DEFAULT_ODOM, message)]:
                        connection = connections[topic]
                        writer.write(connection, (10+i)*10**9+(0 if topic == DEFAULT_CLOUD else 50),
                                     ts.serialize_cdr(msg, connection.msgtype))
            before = digest_files(bag)
            report = extract_bag(bag, root/'journal', provenance='synthetic')
            self.assertEqual(report['extraction']['paired_frames'], 3)
            self.assertEqual(report['first_stamp_ns'], 10*10**9+123)
            self.assertEqual(report['last_stamp_ns'], 12*10**9+123)
            self.assertEqual(report['provenance'], 'synthetic')
            self.assertEqual(digest_files(bag), before)
            with np.load(root/'journal/keyframes/12000000123.npz', allow_pickle=False) as frame:
                np.testing.assert_array_equal(frame['points'], points)
                self.assertEqual(frame['pose'][0, 3], 2.)


if __name__ == '__main__':
    unittest.main()
