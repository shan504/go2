"""Cross-map policy updates use actual PCDs and preserve saved map bundles."""
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import patch
import numpy as np
import open3d as o3d
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from navigation_grid import current_settings, prepare
from map_library import archive, select, fingerprints
from level_map import level
from free_space import resample_free
from export_map import export
from test_free_space import sparse_corridor, reaches, old_centres
from test_ground import pixel, tilted_room
from ground import estimate_ground, heights, body_seed_position, level_transform
from geometry import ObservedGrid, transform_points, pose_matrix


class SharedNavigationGridTests(unittest.TestCase):
    def test_switching_old_and_new_maps_uses_current_policy_without_editing_them(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime/navigation_map'
            root.mkdir()
            original = sparse_corridor(root)
            # Use a wall with continuous fine-grid support; sparse 5cm samples
            # can split at float32 cell boundaries and legitimately be denoised.
            cloud = o3d.io.read_point_cloud(str(original/'map.pcd'))
            x, y = np.meshgrid(np.arange(2.075, 2.126, 0.01), np.arange(-0.1, 1.11, 0.01))
            wall = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.55)))
            cloud.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(cloud.points), wall)))
            o3d.io.write_point_cloud(str(original/'map.pcd'), cloud, compressed=True)
            metadata = yaml.safe_load((original/'metadata.yaml').read_text())
            metadata['parameters'].update(obstacle_min_height=0.9, obstacle_max_height=1.0)
            (original/'metadata.yaml').write_text(yaml.safe_dump(metadata))
            indoor = archive(root, 'indoor')
            farm = level(root)
            archive(root, 'farm')
            before = {path: fingerprints(path) for path in (original, indoor, farm)}
            for name in ('indoor', 'farm', 'indoor'):
                selected = select(root, name)
                prepare(root, output)
                nav = yaml.safe_load((output/'nav.yaml').read_text())
                runtime = yaml.safe_load((output/'metadata.yaml').read_text())
                source_metadata = yaml.safe_load((selected/'metadata.yaml').read_text())
                self.assertEqual(nav['resolution'], 0.05)
                self.assertEqual(runtime['source_map'], str(selected))
                self.assertEqual(runtime['source_ground_model'], source_metadata['ground_model'])
                self.assertEqual(runtime['ground_model']['plane'], [0.,0.,1.,0.])
                self.assertAlmostEqual(runtime['body_height_above_ground'], 0.4)
                original_points = np.asarray(o3d.io.read_point_cloud(str(selected/'map.pcd')).points)
                loaded_points = np.asarray(o3d.io.read_point_cloud(str(output/'map.pcd')).points)
                self.assertEqual(len(loaded_points), len(original_points))
                np.testing.assert_allclose(loaded_points, transform_points(
                    original_points, np.asarray(runtime['map_from_source_map'])), atol=2e-6)
                for key, value in current_settings().items():
                    self.assertEqual(runtime['parameters'][key], value)
                self.assertEqual((root/'latest').resolve(), selected)
                self.assertTrue(reaches(output, (0.25, 0.45), (1.75, 0.45)))
                self.assertEqual(pixel(output, (2.1, 0.5)), 0, 'Current policy lost the wall')
                for path, hashes in before.items():
                    self.assertEqual(fingerprints(path), hashes)
            self.assertEqual(runtime['navigation_denoise']['minimal_group_size'], 8)

    def test_policy_changes_rebuild_from_pcd_instead_of_reusing_previously_cleaned_grid(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime'
            root.mkdir()
            source = sparse_corridor(root)
            cloud = o3d.io.read_point_cloud(str(source/'map.pcd'))
            points = np.vstack((np.asarray(cloud.points), [[0.42, 0.42, -0.1]]))
            cloud.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(str(source/'map.pcd'), cloud, compressed=True)
            before = fingerprints(source)
            prepare(root, output)
            self.assertEqual(pixel(output, (0.42, 0.42)), 254)
            with patch('denoise.settings', return_value=dict(minimal_group_size=1, group_connectivity_type=8)):
                prepare(root, output)
            self.assertEqual(pixel(output, (0.42, 0.42)), 0,
                             'A global filter update must recover the real PCD obstacle')
            policy = dict(current_settings(), grid_resolution=0.025)
            with patch('navigation_grid.current_settings', return_value=policy):
                prepare(root, output)
            self.assertEqual(yaml.safe_load((output/'nav.yaml').read_text())['resolution'], 0.025)
            self.assertEqual(fingerprints(source), before)

    def test_legacy_map_gets_ground_repair_at_startup_without_publishing_a_new_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime'
            root.mkdir()
            source = sparse_corridor(root)
            metadata = yaml.safe_load((source/'metadata.yaml').read_text())
            metadata.pop('ground_model')
            (source/'metadata.yaml').write_text(yaml.safe_dump(metadata))
            before = fingerprints(source)
            prepare(root, output)
            runtime = yaml.safe_load((output/'metadata.yaml').read_text())
            self.assertEqual(runtime['ground_model']['height_at_origin'], 0.)
            self.assertAlmostEqual(runtime['body_height_above_ground'], 0.4, delta=0.01)
            self.assertEqual(fingerprints(source), before)
            self.assertEqual((root/'latest').resolve(), source)
            self.assertFalse((root/'library').exists())

    def test_invalid_map_retains_previous_runtime_grid_and_saved_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime'
            root.mkdir()
            source = sparse_corridor(root)
            prepare(root, output)
            before = fingerprints(output)
            with patch('navigation_grid.current_settings', return_value=dict(
                    grid_resolution=0.05, obstacle_min_height=10.0, obstacle_max_height=11.0)):
                with self.assertRaises(ValueError):
                    prepare(root, output)
            self.assertEqual(fingerprints(output), before)
            self.assertEqual((root/'latest').resolve(), source)
            with self.assertRaises(ValueError):
                prepare(root, source)

    def test_legacy_free_area_fix_is_applied_to_runtime_without_editing_selected_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime/navigation_map'
            root.mkdir()
            sparse_corridor(root)
            with patch('level_map.resample_free', side_effect=old_centres):
                broken = level(root)
            metadata = yaml.safe_load((broken/'metadata.yaml').read_text())
            metadata.pop('free_area_version')
            (broken/'metadata.yaml').write_text(yaml.safe_dump(metadata))
            self.assertFalse(reaches(broken, (0.25, 0.45), (1.75, 0.45)))
            before = fingerprints(broken)
            prepare(root, output)
            self.assertTrue(reaches(output, (0.25, 0.45), (1.75, 0.45)))
            self.assertGreater(yaml.safe_load((output/'metadata.yaml').read_text())['restored_free_cells'], 0)
            self.assertEqual(fingerprints(broken), before)
            self.assertEqual((root/'latest').resolve(), broken)
            with zipfile.ZipFile(export(Path(tmp))) as bundle:
                self.assertIn('navigation_map/nav.pgm', bundle.namelist())
                self.assertIn('navigation_map/metadata.yaml', bundle.namelist())
                self.assertEqual(bundle.read('map/map.pcd'), (broken/'map.pcd').read_bytes())

    def test_coarser_runtime_grid_requires_entire_source_area_to_be_free(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)
            image = np.full((4, 4), 254, np.uint8)
            image[1, 1] = 205
            image[3, 3] = 0
            (source/'nav.pgm').write_bytes(b'P5\n4 4\n255\n'+image.tobytes())
            (source/'nav.yaml').write_text(yaml.safe_dump(dict(
                image='nav.pgm', resolution=0.025, origin=[0, 0, 0])))
            cells = resample_free(source, dict(plane=[0., 0., 1., 0.4]),
                                  np.eye(4), 0.05, allow_coarser=True)
            self.assertEqual(cells, {(0, 0), (1, 1)})

    def test_tilted_map_pcd_floor_grid_and_body_seed_share_one_frame_on_every_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime'
            source = root/'original'; source.mkdir(parents=True)
            points = tilted_room()
            model = estimate_ground(points)
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            o3d.io.write_point_cloud(str(source/'map.pcd'), pc, compressed=True)
            grid = ObservedGrid(0.1); grid.free.add((0,0))
            grid.export(points, source, ground_model=model)
            (source/'metadata.yaml').write_text(yaml.safe_dump(dict(
                ground_model=model, parameters=dict(floor_z=-0.3, grid_resolution=0.1))))
            (root/'latest').symlink_to(source.name)
            before = fingerprints(source)
            prepare(root, output)
            metadata = yaml.safe_load((output/'metadata.yaml').read_text())
            loaded = np.asarray(o3d.io.read_point_cloud(str(output/'map.pcd')).points)
            original = np.asarray(o3d.io.read_point_cloud(str(source/'map.pcd')).points)
            self.assertEqual(len(loaded), len(original), 'Ground alignment erased 3D points')
            floor = np.abs(heights(original, model)) < 0.02
            self.assertLess(np.max(np.abs(loaded[floor,2])), 0.021)
            indices = np.arange(0, len(original)-1, 11)
            np.testing.assert_allclose(np.linalg.norm(loaded[indices]-loaded[indices+1], axis=1),
                                       np.linalg.norm(original[indices]-original[indices+1], axis=1), atol=3e-6)
            seed = body_seed_position([0.,0.,0.], metadata['ground_model'], metadata['body_height_above_ground'])
            self.assertGreater(seed[2], 0.4)
            self.assertEqual(metadata['parameters']['floor_z'], 0.)
            self.assertEqual(yaml.safe_load((output/'nav.yaml').read_text())['origin'][2], 0.)
            first = fingerprints(output)
            prepare(root, output)
            self.assertEqual(fingerprints(output), first, 'Repeated load changed map coordinates')
            self.assertEqual(fingerprints(source), before)
            # A saved aligned bundle must be byte-preserved when loaded again.
            aligned = root/'aligned'; aligned.mkdir()
            import shutil
            for path in output.iterdir():
                shutil.copy2(path, aligned/path.name)
            (root/'latest').unlink(); (root/'latest').symlink_to(aligned.name)
            prepare(root, output)
            self.assertEqual((output/'map.pcd').read_bytes(), (aligned/'map.pcd').read_bytes())
            self.assertEqual(yaml.safe_load((output/'metadata.yaml').read_text())['body_height_above_ground'], seed[2])

    def test_failed_pcd_alignment_does_not_replace_previous_runtime_or_selected_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, output = Path(tmp)/'maps', Path(tmp)/'runtime'
            root.mkdir(); source = sparse_corridor(root)
            prepare(root, output)
            previous, original = fingerprints(output), fingerprints(source)
            with patch('open3d.io.write_point_cloud', return_value=False):
                with self.assertRaises(RuntimeError):
                    prepare(root, output)
            self.assertEqual(fingerprints(output), previous)
            self.assertEqual(fingerprints(source), original)
            self.assertEqual((root/'latest').resolve(), source)

    def test_retained_pose_converts_old_height_once_and_preserves_aligned_height(self):
        pose = pose_matrix([0.423, 0.025, 0.04], [0.,0.,np.sin(0.3/2),np.cos(0.3/2)])
        raw = dict(plane=[0.,0.,1.,0.4])
        aligned = dict(plane=[0.,0.,1.,0.])
        converted = level_transform(raw)@pose
        np.testing.assert_allclose(converted[:3,3], [0.423, 0.025, 0.44])
        np.testing.assert_allclose(level_transform(aligned)@converted, converted)


if __name__ == '__main__':
    unittest.main()
