"""Optimized geometry exports through the existing shared map-library path."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import ObservedGrid, pose_matrix, transform_points
from map_export import save_snapshot
from map_library import fingerprints
from navigation_grid import build_bundle
from test_3d import room


def parameters(root, name):
    return dict(output_directory=str(root), map_name=name, grid_resolution=0.1,
                floor_z=-0.3, obstacle_min_height=0.1, obstacle_max_height=1.5)


class MapExportTests(unittest.TestCase):
    def test_two_corrected_maps_keep_points_start_pose_and_original_versions(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            retained = []
            for name, offset in (('field_a', [0.0, 0.0, 0.0]),
                                 ('field_b', [2.0, -1.0, 0.2])):
                pose = pose_matrix(offset, [0.02, -0.03, 0.1, 0.99])
                points = transform_points(room(), pose)
                grid = ObservedGrid(0.1)
                grid.observe(points, pose[:3, 3])
                output = save_snapshot(points, grid.free, pose, parameters(root, name),
                    dict(enabled=True, backend='rtabmap', map_revision=3),
                    'GICP frontend with camera-assisted RTAB-Map graph')
                self.assertEqual((root / 'latest').resolve(), output)
                metadata = yaml.safe_load((output / 'metadata.yaml').read_text())
                source = Path(metadata['source_map'])
                np.testing.assert_allclose(
                    np.asarray(o3d.io.read_point_cloud(str(source / 'map.pcd')).points),
                    points, atol=5e-7)
                np.testing.assert_allclose(metadata['map_start_pose'],
                    np.asarray(metadata['map_from_source_map']) @ pose, atol=1e-10)
                self.assertEqual(metadata['loop_closure']['map_revision'], 3)
                for _ in range(2):
                    runtime = Path(tmp) / ('runtime_' + name)
                    build_bundle(root, output, runtime)
                    loaded = yaml.safe_load((runtime / 'metadata.yaml').read_text())
                    np.testing.assert_allclose(loaded['map_start_pose'],
                                               metadata['map_start_pose'], atol=1e-10)
                    self.assertAlmostEqual(loaded['body_height_above_ground'],
                                           metadata['body_height_above_ground'])
                retained.extend(((source, fingerprints(source)),
                                 (output, fingerprints(output))))
            for directory, hashes in retained:
                self.assertEqual(fingerprints(directory), hashes)

    def test_failed_geometry_check_does_not_select_partial_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            points = room()
            saved = save_snapshot(points, set(), np.eye(4), parameters(root, 'valid'),
                                  dict(enabled=False), 'GICP')
            before = fingerprints(saved)
            with patch('map_export.build_bundle', side_effect=ValueError('bad geometry')):
                with self.assertRaisesRegex(RuntimeError, 'bad geometry; partial files'):
                    save_snapshot(points, set(), np.eye(4), parameters(root, 'invalid'),
                                  dict(enabled=True), 'RTAB-Map')
            self.assertEqual((root / 'latest').resolve(), saved)
            self.assertEqual(fingerprints(saved), before)

    def test_invalid_snapshot_rejected_before_directory_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            for points in (np.zeros((99, 3)), np.full((100, 3), np.nan)):
                with self.assertRaises(ValueError):
                    save_snapshot(points, set(), np.eye(4), parameters(root, 'bad'), {}, '')
                self.assertFalse(root.exists())


if __name__ == '__main__':
    unittest.main()
