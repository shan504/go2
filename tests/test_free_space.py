import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from collections import deque
import numpy as np
import open3d as o3d
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from free_space import resample_free,visible_free
from geometry import ObservedGrid,transform_points
from level_map import level
from repair_map import repair


def reaches(directory,start,goal):
    nav,free = visible_free(directory)
    origin = np.asarray(nav['origin'][:2]);res=nav['resolution']
    start,goal = [tuple(np.floor((np.asarray(p)-origin)/res).astype(int)) for p in (start,goal)]
    pending=deque([start]);seen={start}
    while pending:
        x,y=pending.popleft()
        if (x,y)==goal:
            return True
        for xy in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            xx,yy=xy
            if 0<=xx<free.shape[1] and 0<=yy<free.shape[0] and free[yy,xx] and xy not in seen:
                seen.add(xy);pending.append(xy)
    return False


def old_centres(source,ground,matrix,resolution,max_cells):
    nav,free=visible_free(source);row,col=np.nonzero(free)
    xy=(np.column_stack((col,row))+0.5)*nav['resolution']+nav['origin'][:2]
    plane=np.asarray(ground['plane']);z=-(xy@plane[:2]+plane[3])/plane[2]
    points=transform_points(np.column_stack((xy,z)),matrix)
    return set(map(tuple,np.floor(points[:,:2]/resolution).astype(int)))


def sparse_corridor(root):
    old=root/'original';old.mkdir()
    x,y=np.meshgrid(np.arange(20)*0.1,np.arange(10)*0.1)
    floor=np.vstack([np.column_stack((x.ravel()+v,y.ravel()+v,np.full(x.size,-0.4)))
                     for v in (0.025,0.075)])
    wall=np.array([[2.1,y,z] for y in np.arange(0,1.01,0.05) for z in np.arange(-0.2,0.8,0.05)])
    points=np.vstack((floor,wall));ground=dict(plane=[0.,0.,1.,0.4],height_at_origin=-0.4)
    pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    o3d.io.write_point_cloud(str(old/'map.pcd'),pc,compressed=True)
    grid=ObservedGrid(0.1);grid.export(points,old,ground_model=ground)
    (old/'metadata.yaml').write_text(yaml.safe_dump(dict(ground_model=ground,parameters=dict(
        grid_resolution=0.1,floor_z=-0.4,obstacle_min_height=0.1,obstacle_max_height=1.5))))
    (root/'latest').symlink_to(old.name)
    return old


class FreeAreaTests(unittest.TestCase):
    def test_area_unknown_and_occupied_survive_finer_resampling(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)
            image=np.full((5,8),205,np.uint8);image[1:4,1:7]=254;image[2,4]=0
            (source/'nav.pgm').write_bytes(b'P5\n8 5\n255\n'+image.tobytes())
            (source/'nav.yaml').write_text(yaml.safe_dump(dict(image='nav.pgm',resolution=0.1,origin=[0,0,0])))
            cells=resample_free(source,dict(plane=[0.,0.,1.,0.4]),np.eye(4),0.05)
            self.assertEqual(len(cells),4*np.count_nonzero(image==254))
            for x,y in cells:
                self.assertEqual(image[4-y//2,x//2],254,'Unknown/occupied source area was cleared')
            self.assertIn((2,2),cells);self.assertIn((3,3),cells)
            self.assertNotIn((8,4),cells)
            with self.assertRaises(ValueError):
                resample_free(source,dict(plane=[0.,0.,1.,0.4]),np.eye(4),0.2)

    def test_sparse_floor_route_and_legacy_repair_keep_pcd(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);old=sparse_corridor(root)
            self.assertTrue(reaches(old,(0.25,0.45),(1.75,0.45)))
            # Reproduce the previously shipped converter, not dense floor
            # sampling that could hide its loss of coarse-cell area evidence.
            with patch('level_map.resample_free',side_effect=old_centres):
                broken=level(root)
            meta=yaml.safe_load((broken/'metadata.yaml').read_text());meta.pop('free_area_version')
            (broken/'metadata.yaml').write_text(yaml.safe_dump(meta))
            self.assertFalse(reaches(broken,(0.25,0.45),(1.75,0.45)))
            before=(broken/'map.pcd').read_bytes()
            fixed=repair(root)
            self.assertTrue(reaches(fixed,(0.25,0.45),(1.75,0.45)))
            self.assertEqual((fixed/'map.pcd').read_bytes(),before,'Grid repair moved/replaced the 3D PCD')
            self.assertEqual((broken/'map.pcd').read_bytes(),before)
            # The level-map entry also repairs an existing legacy leveled
            # map without applying a second coordinate transform.
            (root/'latest').unlink();(root/'latest').symlink_to(broken.name)
            via_level=level(root)
            self.assertEqual((via_level/'map.pcd').read_bytes(),before)
            self.assertTrue(reaches(via_level,(0.25,0.45),(1.75,0.45)))


if __name__=='__main__':
    unittest.main()
