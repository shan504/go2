#!/usr/bin/env python3
"""Read a recorded ROS 2 GICP stream into an offline backend journal.

This program never starts ROS nodes or publishes messages. Header timestamps
must agree exactly: native odometry and a roughly nearby scan are not a
substitute for the accepted GICP pose and its corresponding local cloud.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from geometry import cloud_xyz, pose_matrix


DEFAULT_ODOM = '/gicp/keyframe/odom'
DEFAULT_CLOUD = '/gicp/keyframe/cloud'
KNOWN_PAIRS = {(DEFAULT_ODOM, DEFAULT_CLOUD), ('/gicp/odom', '/gicp/scan_cloud')}
PROVENANCE = ('unknown', 'robot_recording', 'synthetic')


def _stamp(message):
    stamp = message.header.stamp
    sec, nano = stamp.sec, stamp.nanosec
    if (isinstance(sec, (bool, np.bool_)) or isinstance(nano, (bool, np.bool_))
            or not isinstance(sec, (int, np.integer))
            or not isinstance(nano, (int, np.integer))
            or sec < 0 or not 0 <= nano < 1_000_000_000):
        raise ValueError('message header has an invalid nonnegative ROS timestamp')
    result = int(sec)*1_000_000_000 + int(nano)
    if result > np.iinfo(np.int64).max:
        raise ValueError('message timestamp does not fit the journal int64 format')
    return result


def _frame(value):
    if not isinstance(value, str) or not value or value != value.strip() or value.startswith('/'):
        raise ValueError('frames must be nonempty ROS frame IDs without a leading slash')
    return value


def _matrix(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def _points(message):
    fields = {field.name: field for field in message.fields}
    if any(name not in fields or fields[name].count != 1 for name in ('x', 'y', 'z')):
        raise ValueError('cloud must contain one scalar field for each of x, y and z')
    try:
        points = np.asarray(cloud_xyz(message), dtype=np.float64)
    except (ValueError, KeyError, TypeError) as error:
        raise ValueError('invalid PointCloud2 field layout or data size') from error
    good = np.isfinite(points).all(axis=1)
    count = int(len(points) - np.count_nonzero(good))
    points = points[good]
    if not len(points):
        raise ValueError('cloud contains no finite XYZ points')
    return points, count


def _fingerprint(value):
    return hashlib.sha256(np.ascontiguousarray(value, dtype=np.float64).tobytes()).hexdigest()


def extract_messages(messages, output, *, odom_topic=DEFAULT_ODOM, cloud_topic=DEFAULT_CLOUD,
                     odom_frame='gicp_odom', body_frame='base_link', provenance='unknown',
                     confirmed_external_gicp=False, source=None):
    """Write exact, unique (accepted GICP pose, body scan) pairs to a NEW directory.

    ``messages`` yields (topic, deserialized_message). Identical duplicates are
    counted, conflicting duplicates and unpaired timestamps fail the extraction.
    An incomplete output retains FAILED.json instead of session.json; it cannot
    be confused with a completed journal or overwrite any earlier recording.
    """
    if odom_topic == cloud_topic:
        raise ValueError('odometry and cloud topics must differ')
    if provenance not in PROVENANCE:
        raise ValueError('provenance must be unknown, robot_recording or synthetic')
    odom_frame, body_frame = _frame(odom_frame), _frame(body_frame)
    if (odom_topic, cloud_topic) not in KNOWN_PAIRS or odom_frame != 'gicp_odom':
        if not confirmed_external_gicp:
            raise ValueError('nonstandard topics/frame require explicit confirmation that the source is '
                             'continuous, uncorrected GICP, not native /odom or a loop-corrected map pose')
    if not all(isinstance(topic, str) and topic.startswith('/') for topic in (odom_topic, cloud_topic)):
        raise ValueError('topic names must be absolute')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    frames = output/'keyframes'
    frames.mkdir()
    pending = {odom_topic: {}, cloud_topic: {}}
    seen = {odom_topic: {}, cloud_topic: {}}
    counts = {'odom_messages': 0, 'cloud_messages': 0, 'identical_duplicates': 0,
              'paired_frames': 0, 'unique_cloud_points': 0, 'nonfinite_points_removed': 0}
    stamps = []
    try:
        for topic, message in messages:
            if topic not in pending:
                continue
            stamp = _stamp(message)
            if topic == odom_topic:
                counts['odom_messages'] += 1
                if message.header.frame_id != odom_frame or message.child_frame_id != body_frame:
                    raise ValueError(f'odometry frame mismatch at {stamp}: expected {odom_frame} -> {body_frame}')
                value = _matrix(message)
                removed = 0
            else:
                counts['cloud_messages'] += 1
                if message.header.frame_id != body_frame:
                    raise ValueError(f'cloud frame mismatch at {stamp}: expected body-local {body_frame}')
                value, removed = _points(message)
            digest = _fingerprint(value)
            if stamp in seen[topic]:
                if seen[topic][stamp] != digest:
                    raise ValueError(f'conflicting duplicate on {topic} at {stamp}')
                counts['identical_duplicates'] += 1
                continue
            seen[topic][stamp] = digest
            pending[topic][stamp] = value
            if topic == cloud_topic:
                counts['unique_cloud_points'] += len(value)+removed
                counts['nonfinite_points_removed'] += removed
            if stamp in pending[odom_topic] and stamp in pending[cloud_topic]:
                np.savez(frames/f'{stamp}.npz', stamp_ns=np.int64(stamp),
                         points=pending[cloud_topic].pop(stamp), pose=pending[odom_topic].pop(stamp))
                counts['paired_frames'] += 1
                stamps.append(stamp)
        unpaired = {topic: sorted(values) for topic, values in pending.items() if values}
        if unpaired:
            summary = {topic: {'count': len(values), 'first_stamps_ns': values[:5]}
                       for topic, values in unpaired.items()}
            raise ValueError(f'exact timestamp pairing failed; unmatched headers: {summary}')
        if len(stamps) < 2:
            raise ValueError('need at least two distinct exact GICP/cloud pairs')
        metadata = {
            'schema_version': 1, 'coordinate_frame': odom_frame, 'cloud_frame': body_frame,
            'pose_source': 'gicp', 'pose_convention': 'uncorrected_gicp',
            'provenance': provenance, 'source': dict(source or {}),
            'topics': {'odom': odom_topic, 'cloud': cloud_topic},
            'external_gicp_semantics_confirmed': bool(confirmed_external_gicp),
            'timestamp_association': 'exact_header_stamp_no_interpolation',
            'first_stamp_ns': min(stamps), 'last_stamp_ns': max(stamps), 'extraction': counts,
        }
        (output/'session.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
        return metadata
    except Exception as error:
        (output/'FAILED.json').write_text(json.dumps(
            {'complete': False, 'error': str(error), 'extraction': counts}, indent=2)+'\n')
        raise


def extract_bag(bag, output, **kwargs):
    """Read a local ROS 2 SQLite/MCAP bag using rosbags; no ROS runtime is used."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    bag = Path(bag).resolve(strict=True)
    odom_topic = kwargs.get('odom_topic', DEFAULT_ODOM)
    cloud_topic = kwargs.get('cloud_topic', DEFAULT_CLOUD)
    expected = {odom_topic: 'nav_msgs/msg/Odometry', cloud_topic: 'sensor_msgs/msg/PointCloud2'}
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        if not reader.is2:
            raise ValueError('this converter accepts ROS 2 bags only')
        connections = [connection for connection in reader.connections if connection.topic in expected]
        if {connection.topic for connection in connections} != set(expected):
            raise ValueError('bag lacks the selected accepted GICP/local-cloud pair; '
                             'raw lidar + native odometry are not interchangeable inputs')
        for connection in connections:
            if connection.msgtype != expected[connection.topic]:
                raise ValueError(f'unexpected type on {connection.topic}: {connection.msgtype}')
        source = {'bag_path': str(bag), 'reader': 'rosbags',
                  'interpretation': 'backend-only replay; frontend is not recomputed',
                  'input_bytes': sum(path.stat().st_size for path in bag.rglob('*') if path.is_file())
                                 if bag.is_dir() else bag.stat().st_size}
        def messages():
            for connection, _, raw in reader.messages(connections=connections):
                yield connection.topic, reader.deserialize(raw, connection.msgtype)
        return extract_messages(messages(), output, source=source, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='recorded ROS 2 bag directory or MCAP file')
    parser.add_argument('output', help='new offline journal directory; must not exist')
    parser.add_argument('--odom-topic', default=DEFAULT_ODOM)
    parser.add_argument('--cloud-topic', default=DEFAULT_CLOUD)
    parser.add_argument('--odom-frame', default='gicp_odom')
    parser.add_argument('--body-frame', default='base_link')
    parser.add_argument('--provenance', choices=PROVENANCE, default='unknown',
                        help='do not label generated/demo data as a robot recording')
    parser.add_argument('--confirm-external-gicp', action='store_true',
                        help='attest custom topics/frame contain continuous GICP poses and matching local scans')
    args = parser.parse_args()
    try:
        report = extract_bag(args.bag, args.output, odom_topic=args.odom_topic,
                             cloud_topic=args.cloud_topic, odom_frame=args.odom_frame,
                             body_frame=args.body_frame, provenance=args.provenance,
                             confirmed_external_gicp=args.confirm_external_gicp)
        print(json.dumps(report, indent=2, allow_nan=False))
    except (OSError, ValueError, ImportError) as error:
        parser.exit(2, f'Cannot extract GICP journal: {error}\n')


if __name__ == '__main__':
    main()
