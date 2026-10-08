import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from ground import estimate_ground,heights
from geometry import ObservedGrid
from repair_map import repair


def tilted_room():
    x,y = np.meshgrid(np.arange(-1,4,0.025),np.arange(-2,2,0.025))
    floor = np.column_stack((x.ravel(),y.ravel(),(-0.46+0.08*x).ravel()))
    rng = np.random.default_rng(7)
    floor[:,2] += rng.normal(0,0.004,len(floor))
    wall = np.column_stack((np.full(2500,4.0),rng.uniform(-2,2,2500),rng.uniform(-0.14,1.5,2500)))
    obstacle = rng.uniform([1.0,0.7,-0.25],[1.3,1.0,0.6],(1000,3))
    # A tabletop below the body must not replace the lower broad floor.
    tx,ty = np.meshgrid(np.arange(-0.9,0.9,0.06),np.arange(-1.8,-0.9,0.06))
    table = np.column_stack((tx.ravel(),ty.ravel(),np.full(tx.size,-0.14)))
    return np.vstack((floor,wall,obstacle,table))


def pixel(directory,xy):
    cfg = yaml.safe_load((directory/'nav.yaml').read_text())
    raw = (directory/'nav.pgm').read_bytes().split(b'\n',3)
    w,h = map(int,raw[1].split())
    image = np.frombuffer(raw[3],dtype=np.uint8).reshape(h,w)
    cx,cy = np.floor((np.asarray(xy)-cfg['origin'][:2])/cfg['resolution']+1e-8).astype(int)
    return int(image[h-1-cy,cx])


class GroundTests(unittest.TestCase):
    def test_tilted_floor_and_real_obstacle(self):
        points = tilted_room()
        model = estimate_ground(points)
        self.assertAlmostEqual(model['height_at_origin'],-0.46,delta=0.01)
        self.assertLess(np.std(heights(points[:32000],model)),0.01)
        with tempfile.TemporaryDirectory() as tmp:
            grid = ObservedGrid()
            grid.free.add((0,0))
            grid.export(points,tmp,ground_model=model)
            self.assertEqual(pixel(Path(tmp),(3.5,0.0)),254,'Tilted observed floor became obstacle')
            self.assertEqual(pixel(Path(tmp),(1.1,0.8)),0,'Real obstacle was erased')

    def test_repair_preserves_original_pcd_and_changes_wrong_grid(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root/'old';old.mkdir()
            points = tilted_room()
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            self.assertTrue(o3d.io.write_point_cloud(str(old/'map.pcd'),pc,compressed=True))
            grid = ObservedGrid();grid.free.add((0,0))
            grid.export(points,old)
            # Simulate the legacy file set without observed-free archive.
            (old/'observed_free.npz').unlink()
            params = dict(floor_z=-0.30,grid_resolution=0.10,obstacle_min_height=0.10,obstacle_max_height=1.5)
            (old/'metadata.yaml').write_text(yaml.safe_dump(dict(parameters=params,frame_id='map')))
            (root/'latest').symlink_to('old')
            before = {p.name:p.read_bytes() for p in old.iterdir()}
            self.assertEqual(pixel(old,(3.5,0.0)),0)
            new = repair(root)
            self.assertEqual((root/'latest').resolve(),new)
            self.assertEqual(pixel(new,(3.5,0.0)),254)
            self.assertEqual(pixel(new,(1.1,0.8)),0)
            self.assertEqual((new/'map.pcd').read_bytes(),before['map.pcd'])
            self.assertEqual({p.name:p.read_bytes() for p in old.iterdir()},before)
            self.assertIn('ground_model',yaml.safe_load((new/'metadata.yaml').read_text()))

    def test_no_ground_refuses_repair_without_switching_latest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);old = root/'old';old.mkdir()
            points = np.random.default_rng(3).uniform([-2,-2,1],[2,2,2],(500,3))
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            o3d.io.write_point_cloud(str(old/'map.pcd'),pc)
            (old/'metadata.yaml').write_text(yaml.safe_dump(dict(parameters=dict(floor_z=-0.3))))
            (root/'latest').symlink_to('old')
            with self.assertRaises(ValueError):
                repair(root)
            self.assertEqual((root/'latest').resolve(),old)

    def test_random_volume_is_not_accepted_as_floor(self):
        points = np.random.default_rng(8).uniform([-2,-2,-0.79],[2,2,-0.11],(20000,3))
        with self.assertRaises(ValueError):
            estimate_ground(points)


if __name__ == '__main__':
    unittest.main()
