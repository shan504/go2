import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import ObservedGrid,transform_points
from ground import estimate_ground,heights,level_transform,body_seed_position
from level_map import level
from repair_map import repair
from test_ground import tilted_room,pixel


class LevelMapTests(unittest.TestCase):
    def test_rigid_alignment_preserves_height_and_distance(self):
        points = tilted_room()
        model = estimate_ground(points)
        matrix = level_transform(model)
        transformed = transform_points(points,matrix)
        np.testing.assert_allclose(transformed[:,2],heights(points,model),atol=1e-10)
        np.testing.assert_allclose(transform_points(transformed,np.linalg.inv(matrix)),points,atol=1e-10)
        np.testing.assert_allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(matrix[:3,:3]),1.)

    def test_2d_seed_lift_and_legacy_explicit_z(self):
        leveled = dict(plane=[0.,0.,1.,0.])
        np.testing.assert_allclose(body_seed_position([0.2,0.3,0.],leveled,0.383),[0.2,0.3,0.383])
        np.testing.assert_allclose(body_seed_position([0.2,0.3,0.7],leveled,0.383),[0.2,0.3,0.7])
        np.testing.assert_allclose(body_seed_position([0.2,0.3,0.]),[0.2,0.3,0.])
        tilted = dict(plane=[0.0,-0.05,np.sqrt(1-0.05**2),0.38])
        lifted = body_seed_position([0.2,0.3,0.],tilted,0.38)
        self.assertAlmostEqual(heights(lifted[None,:],tilted)[0],0.38)
        with self.assertRaises(ValueError):
            body_seed_position([0.,0.,0.],leveled,float('nan'))
        with self.assertRaises(ValueError):
            level_transform(dict(plane=[0.,1.,0.,0.]))

    def test_new_map_floor_grid_repair_and_repeat_preserve_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);old = root/'original';old.mkdir()
            points = tilted_room();ground = estimate_ground(points)
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            o3d.io.write_point_cloud(str(old/'map.pcd'),pc,compressed=True)
            grid = ObservedGrid(0.1)
            grid.free.update([(0,0),(1,0)])
            grid.export(points,old,ground_model=ground)
            params = dict(floor_z=-0.3,grid_resolution=0.1,obstacle_min_height=0.1,obstacle_max_height=1.5)
            (old/'metadata.yaml').write_text(yaml.safe_dump(dict(frame_id='map',parameters=params,ground_model=ground)))
            (root/'latest').symlink_to(old.name)
            before = {p.name:p.read_bytes() for p in old.iterdir()}
            with patch('level_map.o3d.io.write_point_cloud',return_value=False):
                with self.assertRaises(RuntimeError):
                    level(root)
            self.assertEqual((root/'latest').resolve(),old,'Failed write must keep original selected')
            self.assertEqual(sorted(p.name for p in root.iterdir()),['latest','library','original'])
            self.assertEqual(list(root.glob('library/*/*')), [], 'Failed write left a published/partial version')
            new = level(root)
            metadata = yaml.safe_load((new/'metadata.yaml').read_text())
            matrix = np.array(metadata['map_from_previous_map'])
            after = np.asarray(o3d.io.read_point_cloud(str(new/'map.pcd')).points)
            original = np.asarray(o3d.io.read_point_cloud(str(old/'map.pcd')).points)
            self.assertEqual(len(after),len(original))
            np.testing.assert_allclose(after,transform_points(original,matrix),atol=2e-6)
            np.testing.assert_allclose(metadata['ground_model']['plane'],[0.,0.,1.,0.])
            self.assertAlmostEqual(metadata['body_height_above_ground'],ground['plane'][3])
            self.assertEqual(pixel(new,transform_points([[3.5,0.,-0.18]],matrix)[0,:2]),254)
            self.assertEqual(pixel(new,transform_points([[1.1,0.8,0.]],matrix)[0,:2]),0)
            self.assertEqual({p.name:p.read_bytes() for p in old.iterdir()},before)
            self.assertEqual(level(root),new,'Repeat must not change origin again')
            repaired = repair(root)
            self.assertEqual((repaired/'map.pcd').read_bytes(),(new/'map.pcd').read_bytes())
            self.assertEqual(yaml.safe_load((repaired/'metadata.yaml').read_text())['ground_model']['plane'],[0.,0.,1.,0.])

    def test_bad_ground_keeps_latest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);old = root/'original';old.mkdir()
            points = np.random.default_rng(3).uniform([-2,-2,1],[2,2,2],(500,3))
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            o3d.io.write_point_cloud(str(old/'map.pcd'),pc)
            (old/'metadata.yaml').write_text(yaml.safe_dump(dict(parameters=dict(floor_z=-0.3))))
            (root/'latest').symlink_to(old.name)
            with self.assertRaises(ValueError):
                level(root)
            self.assertEqual((root/'latest').resolve(),old)


if __name__ == '__main__':
    unittest.main()
