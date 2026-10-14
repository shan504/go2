"""Mapping-start metadata follows every shared map-coordinate transform."""
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import ObservedGrid, first_map_pose, pose_matrix, transform_points
from map_library import fingerprints
from map_start import resolve_start_pose, start_pose_from_metadata, validate_start_pose
from navigation_grid import build_bundle
from test_3d import room


def saved_raw(root, name, tilted=True, start=None):
    directory = root / name
    directory.mkdir(parents=True)
    transform = (pose_matrix([0.0, 0.0, -0.1], [0.025, -0.04, 0.0, 0.999])
                 if tilted else np.eye(4))
    points = transform_points(room(), transform)
    plane = np.linalg.inv(transform).T @ [0.0, 0.0, 1.0, 0.3]
    plane /= np.linalg.norm(plane[:3])
    ground = dict(plane=plane.tolist(), height_at_origin=float(-plane[3] / plane[2]))
    assert o3d.io.write_point_cloud(str(directory / 'map.pcd'),
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)), compressed=True)
    ObservedGrid(0.1).export(points, directory, ground_model=ground)
    metadata = dict(ground_model=ground, frame_id='map', parameters=dict(
        grid_resolution=0.1, floor_z=-0.4, obstacle_min_height=0.1, obstacle_max_height=1.5))
    if start is not None:
        metadata['map_start_pose'] = np.asarray(start).tolist()
    (directory / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    return directory


def metadata(directory):
    return yaml.safe_load((directory / 'metadata.yaml').read_text())


class MapStartTests(unittest.TestCase):
    def test_legacy_canonical_height_and_rigid_pose_validation(self):
        expected = np.eye(4)
        expected[2, 3] = 0.42
        np.testing.assert_allclose(start_pose_from_metadata(dict(
            ground_model=dict(plane=[0.0, 0.0, 1.0, 0.0]),
            body_height_above_ground=0.42)), expected)
        for bad in (np.eye(3), np.full((4, 4), np.nan),
                    np.diag([1.0, 1.0, 1.0, 0.0])):
            with self.assertRaises(ValueError):
                validate_start_pose(bad)

    def test_saved_full_body_start_survives_two_maps_and_repeated_canonical_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            root.mkdir()
            first = first_map_pose(pose_matrix([4.0, -2.0, 0.4], [0.035, -0.02, 0.25, 0.967]))
            self.assertGreater(abs(first[2, 0]) + abs(first[2, 1]), 0.01)
            for name, tilted in (('room-a', False), ('room-b', True)):
                source = saved_raw(root, name, tilted=tilted, start=first)
                before = fingerprints(source)
                output = Path(tmp) / f'runtime-{name}'
                build_bundle(root, source, output)
                current = metadata(output)
                expected = np.asarray(current['map_from_source_map']) @ first
                np.testing.assert_allclose(current['map_start_pose'], expected, atol=1e-10)
                self.assertAlmostEqual(expected[2, 3], current['body_height_above_ground'])
                aligned = root / f'aligned-{name}'
                shutil.copytree(output, aligned)
                aligned_before = fingerprints(aligned)
                for _ in range(2):
                    build_bundle(root, aligned, output)
                    loaded = metadata(output)
                    np.testing.assert_allclose(loaded['map_from_source_map'], np.eye(4), atol=1e-10)
                    np.testing.assert_allclose(loaded['map_start_pose'], expected, atol=1e-10)
                self.assertEqual(fingerprints(source), before)
                self.assertEqual(fingerprints(aligned), aligned_before)

    def test_legacy_origin_composes_retained_nonidentity_and_identity_steps(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            root.mkdir()
            raw = saved_raw(root, 'raw-legacy')
            raw_before = fingerprints(raw)
            output = Path(tmp) / 'runtime'
            build_bundle(root, raw, output)
            expected = np.asarray(metadata(output)['map_from_source_map'])
            np.testing.assert_allclose(metadata(output)['map_start_pose'], expected, atol=1e-10)
            previous = raw
            saved = []
            for index in range(2):
                aligned = root / f'legacy-level-{index}'
                shutil.copytree(output, aligned)
                old = metadata(aligned)
                old.pop('map_start_pose', None)
                old['leveled_from'] = str(previous)
                old['map_from_previous_map'] = (expected if index == 0 else np.eye(4)).tolist()
                # Use the Docker path convention on the first ancestor link.
                if index == 0:
                    old['leveled_from'] = '/maps/raw-legacy'
                (aligned / 'metadata.yaml').write_text(yaml.safe_dump(old))
                saved.append((aligned, fingerprints(aligned)))
                previous = aligned
            build_bundle(root, previous, output)
            np.testing.assert_allclose(metadata(output)['map_start_pose'], expected, atol=1e-10)
            self.assertEqual(fingerprints(raw), raw_before)
            for directory, before in saved:
                self.assertEqual(fingerprints(directory), before)

    def test_invalid_stored_start_keeps_saved_source_and_previous_runtime_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            root.mkdir()
            source = saved_raw(root, 'valid', start=np.eye(4))
            output = Path(tmp) / 'runtime'
            build_bundle(root, source, output)
            runtime_before = fingerprints(output)
            for name, bad in (('scaled', np.diag([2.0, 1.0, 1.0, 1.0])),
                              ('reflected', np.diag([-1.0, 1.0, 1.0, 1.0])),
                              ('wrong-row', np.diag([1.0, 1.0, 1.0, 2.0]))):
                invalid = saved_raw(root, name, start=bad)
                before = fingerprints(invalid)
                with self.assertRaises(ValueError):
                    build_bundle(root, invalid, output)
                self.assertEqual(fingerprints(invalid), before)
                self.assertEqual(fingerprints(output), runtime_before)

    def test_stored_start_bypasses_missing_ancestors_and_cycles_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            root.mkdir()
            stored = pose_matrix([1.0, -0.5, 0.4], [0.025, -0.03, 0.2, 0.97])
            source = saved_raw(root, 'stored', start=stored)
            values = metadata(source)
            values.update(leveled_from='/maps/missing', map_from_previous_map=np.eye(4).tolist())
            (source / 'metadata.yaml').write_text(yaml.safe_dump(values))
            before = fingerprints(source)
            np.testing.assert_allclose(resolve_start_pose(root, source), stored)
            self.assertEqual(fingerprints(source), before)
            left = saved_raw(root, 'cycle-a')
            right = saved_raw(root, 'cycle-b')
            for current, previous in ((left, right), (right, left)):
                values = metadata(current)
                values.update(leveled_from=str(previous), map_from_previous_map=np.eye(4).tolist())
                (current / 'metadata.yaml').write_text(yaml.safe_dump(values))
            with self.assertRaisesRegex(ValueError, 'Cyclic'):
                resolve_start_pose(root, left)

    def test_outside_library_ancestor_is_not_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'maps'
            root.mkdir()
            source = saved_raw(root, 'legacy')
            outside = Path(tmp) / 'outside'
            outside.mkdir()
            unrelated = np.eye(4)
            unrelated[0, 3] = 20.0
            (outside / 'metadata.yaml').write_text(yaml.safe_dump(dict(map_start_pose=unrelated.tolist())))
            step = pose_matrix([0.0, 0.0, 0.4], [0.025, -0.04, 0.0, 0.999])
            values = metadata(source)
            values.update(leveled_from=str(outside), map_from_previous_map=step.tolist())
            (source / 'metadata.yaml').write_text(yaml.safe_dump(values))
            np.testing.assert_allclose(resolve_start_pose(root, source), step)
            # A flattened canonical export may retain only an identity step.
            # Its standing-height hint is intentionally refined by GICP.
            values.update(ground_model=dict(plane=[0.0, 0.0, 1.0, 0.0]),
                          body_height_above_ground=0.42, ground_aligned=True,
                          map_from_previous_map=np.eye(4).tolist())
            (source / 'metadata.yaml').write_text(yaml.safe_dump(values))
            standing = np.eye(4)
            standing[2, 3] = 0.42
            np.testing.assert_allclose(resolve_start_pose(root, source), standing)


if __name__ == '__main__':
    unittest.main()
