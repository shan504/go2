#!/usr/bin/env python3
"""Generate a synthetic raw ROS2 bag for the isolated GICP replay smoke.

Only /odom and body-local /point_cloud2 are written. No TF or command topics
are included. Exact analytic odometry and idealized room scans are not Go2
measurements and do not establish field accuracy or loop-closure performance.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import pose_matrix, transform_points
from test_3d import room


def generate(output, frames=80, hz=10., speed=0.):
    if isinstance(frames, bool) or frames < 30 or int(frames) != frames:
        raise ValueError('frames must be an integer >= 30 for multiple mapper keyframes')
    if not np.isfinite(hz) or not 1 <= hz <= 50:
        raise ValueError('hz must be finite and between 1 and 50')
    if not np.isfinite(speed) or not 0 <= speed <= .20:
        raise ValueError('synthetic speed must be within 0..0.20 m/s')
    if (frames-1)/hz < 5:
        raise ValueError('record at least 5 seconds for the mapper keyframe interval')
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = store.types
    world = room()
    fields = [types['sensor_msgs/msg/PointField'](name=name, offset=4*i, datatype=7, count=1)
              for i, name in enumerate('xyz')]
    topics = {'/odom': 'nav_msgs/msg/Odometry', '/point_cloud2': 'sensor_msgs/msg/PointCloud2'}
    # Version 8 retains the string-serialized QoS profiles understood by Humble.
    # Version 9 changed that metadata representation to a YAML sequence.
    with Writer(output, version=8) as writer:
        connections = {name: writer.add_connection(name, kind, typestore=store)
                       for name, kind in topics.items()}
        for index in range(frames):
            elapsed = index/hz
            stamp_ns = 10_000_000_000+round(elapsed*1_000_000_000)
            stamp = types['builtin_interfaces/msg/Time'](
                sec=stamp_ns//1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
            x = speed*elapsed
            matrix = pose_matrix([x, 0., 0.], [0., 0., 0., 1.])
            points = np.asarray(transform_points(world, np.linalg.inv(matrix)), dtype='<f4')
            position = types['geometry_msgs/msg/Point'](x=float(x), y=0., z=0.)
            orientation = types['geometry_msgs/msg/Quaternion'](x=0., y=0., z=0., w=1.)
            pose = types['geometry_msgs/msg/Pose'](position=position, orientation=orientation)
            linear = types['geometry_msgs/msg/Vector3'](x=float(speed), y=0., z=0.)
            angular = types['geometry_msgs/msg/Vector3'](x=0., y=0., z=0.)
            twist = types['geometry_msgs/msg/Twist'](linear=linear, angular=angular)
            odom = types['nav_msgs/msg/Odometry'](
                header=types['std_msgs/msg/Header'](stamp=stamp, frame_id='odom'),
                child_frame_id='base_link',
                pose=types['geometry_msgs/msg/PoseWithCovariance'](pose=pose, covariance=np.zeros(36)),
                twist=types['geometry_msgs/msg/TwistWithCovariance'](twist=twist, covariance=np.zeros(36)))
            scan = types['sensor_msgs/msg/PointCloud2'](
                header=types['std_msgs/msg/Header'](stamp=stamp, frame_id='base_link'),
                height=1, width=len(points), fields=fields, is_bigendian=False,
                point_step=12, row_step=12*len(points),
                data=np.frombuffer(points.tobytes(), dtype=np.uint8), is_dense=True)
            # Odometry is received first, but both measurement headers agree.
            for delta, (name, message) in enumerate((('/odom', odom), ('/point_cloud2', scan))):
                writer.write(connections[name], stamp_ns+delta,
                             store.serialize_cdr(message, topics[name]))
    counts = {}
    with AnyReader([output]) as reader:
        for connection, _, _ in reader.messages():
            counts[connection.topic] = counts.get(connection.topic, 0)+1
    if counts != {name: frames for name in topics}:
        raise RuntimeError(f'generated bag verification failed: {counts}')
    report = dict(provenance='synthetic', real_robot_data=False, schema_version=1,
                  source='tests/test_3d.py room; exact analytic poses and idealized point scans',
                  topics=topics, messages=counts, point_count_per_frame=len(world),
                  header_stamp_association='exact', odometry_frame='odom', body_frame='base_link',
                  hz=hz, frames=frames, duration_seconds=(frames-1)/hz,
                  synthetic_speed_mps=speed, ground_z=-.30,
                  motion_command_topics=False, tf_topics=False, rosbag_metadata_version=8)
    (output/'fixture.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--frames', type=int, default=80)
    parser.add_argument('--hz', type=float, default=10.)
    parser.add_argument('--speed', type=float, default=0., help='Synthetic motion only; default stationary')
    args = parser.parse_args()
    print(json.dumps(generate(args.output, args.frames, args.hz, args.speed), indent=2))


if __name__ == '__main__':
    main()
