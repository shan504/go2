"""Ten-million-point storage/PCD and real ROS message capacity checks.

Run --storage with numpy/Open3D, and --ros inside ROS2 Humble. These are
synthetic capacity tests, not Foxglove network throughput or robot benchmarks.
The byte-budget assertion covers the legacy bridge parameter only; it does not
measure or diagnose the Rust bridge outbox queue.
"""
import argparse
import gc
import importlib.util
from pathlib import Path
import resource
import sys
import tempfile
import time
from unittest.mock import patch

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'go2_3d'))
COUNT = 10_000_000
CHUNK = 500_000


def coordinates(start, stop):
    """Distinct exact binary XYZ, spanning positive and negative tile bounds."""
    indices = np.arange(start, stop, dtype=np.int64)
    result = np.empty((len(indices), 3), dtype=np.float64)
    result[:, 0] = (indices % 500 - 250) * .125 + .03125
    result[:, 1] = (indices // 500 % 200 - 100) * .125 + .03125
    result[:, 2] = (indices // 100_000 - 50) * .125 + .03125
    return result


def verify_complete(points):
    """Check all 10M identities and coordinates, not only first/last samples."""
    assert len(points) == COUNT, (len(points), COUNT)
    counts = np.zeros(COUNT, dtype=np.uint32)
    for start in range(0, COUNT, CHUNK):
        part = points[start:start + CHUNK]
        cells = np.rint((part - .03125) / .125).astype(np.int64)
        cells += [250, 100, 50]
        assert np.all((cells >= 0) & (cells < [500, 200, 100]))
        assert np.array_equal(part, (cells - [250, 100, 50]) * .125 + .03125)
        indices = cells[:, 0] + 500 * cells[:, 1] + 100_000 * cells[:, 2]
        np.add.at(counts, indices, 1)
    assert np.all(counts == 1), 'Missing or duplicated retained XYZ'


def storage():
    import open3d as o3d
    from mapping_store import MappingStore
    from registration import cloud as point_cloud

    with tempfile.TemporaryDirectory(prefix='go2-ten-million-') as temporary:
        for name, resolution in (('standard', .06), ('dense', .02)):
            started = time.monotonic()
            store = MappingStore(resolution, .15)
            for start in range(0, COUNT, CHUNK):
                points = coordinates(start, start + CHUNK)
                update = store.prepare(points)
                store.commit(update)
                assert store.count == start + CHUNK
            del points, update
            snapshot = store.snapshot()
            assert not snapshot.flags.writeable
            verify_complete(snapshot)
            # Exercise the real Open3D PCD writer/reader used by map saves.
            cloud = point_cloud(snapshot, None)
            path = Path(temporary) / (name + '.pcd')
            assert o3d.io.write_point_cloud(str(path), cloud, compressed=True)
            del cloud
            for _ in range(2):
                restored = o3d.io.read_point_cloud(str(path))
                assert np.array_equal(np.asarray(restored.points), snapshot), 'PCD changed XYZ or point count'
                del restored
            # Capacity remains advisory: a later observation must not evict
            # old geometry, truncate at 10M, or mutate an earlier snapshot.
            store.commit(store.prepare(np.array([[100., -100., 10.]])))
            assert store.count == COUNT + 1
            assert len(snapshot) == COUNT
            assert len(store.snapshot()) == COUNT + 1
            verify_complete(snapshot)
            print(f'PASS {name}: 10M unique XYZ retained, PCD roundtrip twice, '
                  f'growth beyond advisory retains history; seconds={time.monotonic()-started:.2f}', flush=True)
            del store, snapshot
            gc.collect()


def launch_buffer():
    """Evaluate the legacy bridge byte-budget default without starting nodes."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    with patch('ament_index_python.packages.get_package_share_directory', return_value='/unused'):
        spec = importlib.util.spec_from_file_location('capacity_stack', PROJECT / 'go2_3d/stack.launch.py')
        stack = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(stack)
        with patch.object(Path, 'read_text', return_value='<robot name="capacity_test"/>'):
            description = stack.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return int(context.launch_configurations['send_buffer_limit'])


def ros():
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from ros_cloud import xyz_message

    started = time.monotonic()
    points = coordinates(0, COUNT)
    header = Header(frame_id='map')
    header.stamp.sec, header.stamp.nanosec = 123, 456
    message = xyz_message(header, points)
    assert message.width == COUNT and message.height == 1
    assert message.point_step == 12 and message.row_step == COUNT * 12
    assert len(message.data) == 120_000_000
    wire = serialize_message(message)
    buffer = launch_buffer()
    assert len(wire) > 32 * 1024**2, 'Fixture no longer exceeds the old legacy byte budget'
    assert 2 * len(wire) < buffer, (len(wire), buffer)
    restored = deserialize_message(wire, PointCloud2)
    assert restored.header == header and restored.width == COUNT
    assert [(f.name, f.offset, f.datatype, f.count) for f in restored.fields] == [
        (name, index * 4, 7, 1) for index, name in enumerate(('x', 'y', 'z'))]
    decoded = np.frombuffer(restored.data, dtype='<f4').reshape(-1, 3)
    assert np.array_equal(decoded, points), 'ROS serialization lost or changed XYZ'
    print(f'PASS ROS: 10M XYZ exact CDR roundtrip; data_bytes={len(message.data)} '
          f'cdr_bytes={len(wire)} legacy_send_buffer_bytes={buffer}; '
          f'seconds={time.monotonic()-started:.2f}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--storage', action='store_true')
    modes.add_argument('--ros', action='store_true')
    arguments = parser.parse_args()
    storage() if arguments.storage else ros()
    print(f'Peak process RSS: {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024:.1f} MiB', flush=True)
