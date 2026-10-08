import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
import open3d as o3d
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from geometry import (pose_matrix, quaternion_from_matrix, transform_points,
                      cloud_xyz, filter_points, ObservedGrid)
from registration import cloud, align, static_correction_is_consistent, blend_correction


def room():
    rng = np.random.default_rng(23)
    # Asymmetric perpendicular surfaces plus a column: a genuinely 3D match.
    xy = rng.uniform([-3,-2],[4,3],(1600,2))
    floor = np.column_stack((xy,np.full(len(xy),-0.3)))
    wall1 = np.column_stack((rng.uniform(-3,4,1100),np.full(1100,3.0),rng.uniform(-0.3,2.0,1100)))
    wall2 = np.column_stack((np.full(900,-3.0),rng.uniform(-2,3,900),rng.uniform(-0.3,2.0,900)))
    column = rng.uniform([0.5,-0.9,-0.3],[0.8,-0.5,1.7],(500,3))
    return np.vstack((floor,wall1,wall2,column))


class GeometryTests(unittest.TestCase):
    def test_se3_roundtrip_and_map_odom_composition(self):
        for angle in (0.0,0.7,math.pi-1e-7):
            map_base = pose_matrix([1,2,0.2],[0,0,math.sin(angle/2),math.cos(angle/2)])
            np.testing.assert_allclose(pose_matrix(map_base[:3,3],quaternion_from_matrix(map_base)),map_base,atol=1e-7)
            odom_base = pose_matrix([0.3,-0.7,0],[0,0,0,1])
            np.testing.assert_allclose((map_base@np.linalg.inv(odom_base))@odom_base,map_base,atol=1e-7)

    def test_cloud_padding_big_endian_and_nan(self):
        data = bytearray(48)
        rows = np.ndarray((2,1),dtype=np.dtype({'names':['x','y','z'],
            'formats':['>f4']*3,'offsets':[0,4,8],'itemsize':16}),buffer=data,strides=(24,16))
        rows[0,0] = (1,2,3)
        rows[1,0] = (float('nan'),0,0)
        message = SimpleNamespace(fields=[SimpleNamespace(name=k,datatype=7,offset=i*4) for i,k in enumerate('xyz')],
            height=2,width=1,is_bigendian=True,point_step=16,row_step=24,data=data)
        np.testing.assert_allclose(filter_points(cloud_xyz(message)),[[1,2,3]])

    def test_grid_unknown_occupied_and_origin(self):
        grid = ObservedGrid(0.1)
        # Known free ray to wall, elevated return beyond the wall stays hidden.
        points = np.array([[1.0,0,0.4],[2.0,0,2.0],[1.0,1,0.4],[-1.0,-1,0.4]])
        grid.observe(points,[0,0,0])
        self.assertNotIn((15,0),grid.free)
        with tempfile.TemporaryDirectory() as tmp:
            metadata = grid.export(points,tmp)
            raw = (Path(tmp)/'nav.pgm').read_bytes().split(b'\n',3)
            width,height = map(int,raw[1].split())
            image = np.frombuffer(raw[3],dtype=np.uint8).reshape(height,width)
            self.assertEqual(set(np.unique(image)),{0,205,254})
            ox,oy,_ = metadata['origin']
            cx,cy = round((1.0-ox)/0.1),round((0-oy)/0.1)
            self.assertEqual(image[height-1-cy,cx],0)
            self.assertEqual(yaml.safe_load((Path(tmp)/'nav.yaml').read_text())['image'],'nav.pgm')

    def test_grid_refuses_empty_obstacles(self):
        grid = ObservedGrid()
        grid.free.add((0,0))
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                grid.export(np.array([[1,0,-0.3]]),tmp)


class GICPTests(unittest.TestCase):
    def test_fine_gicp_with_changing_low_height_scans(self):
        points = room()
        points = points[points[:,2] < 0.5]
        truth = pose_matrix([0.42,-0.18,0.04],[0,0,math.sin(0.08/2),math.cos(0.08/2)])
        guess = pose_matrix([0.40,-0.16,0.03],[0,0,math.sin(0.07/2),math.cos(0.07/2)])
        rng = np.random.default_rng(71)
        target = cloud(points,0.05)
        for _ in range(5):
            sampled = points[rng.choice(len(points),1300,replace=False)]
            scan = transform_points(sampled,np.linalg.inv(truth))+rng.normal(0,0.005,sampled.shape)
            result = align(cloud(scan,0.05),target,guess,correspondence=0.20,max_rmse=0.10,iterations=25)
            self.assertTrue(result.accepted,result)
            np.testing.assert_allclose(result.transform,truth,atol=0.02)

    def test_filter_reduces_noise_without_filtering_robot_motion(self):
        previous = np.eye(4)
        observed,filtered = [],[]
        for sign in [1,-1]*12:
            noisy = pose_matrix([sign*0.02,0,0],[0,0,math.sin(sign*0.01/2),math.cos(sign*0.01/2)])
            previous = blend_correction(previous,noisy,0.25)
            observed.append(noisy[0,3])
            filtered.append(previous[0,3])
            np.testing.assert_allclose(previous[:3,:3].T@previous[:3,:3],np.eye(3),atol=1e-8)
        self.assertLess(np.std(filtered),np.std(observed)/2)
        odom = pose_matrix([0.4,0,0],[0,0,0,1])
        np.testing.assert_allclose(blend_correction(np.eye(4),np.eye(4))@odom,odom,atol=1e-8)
        with self.assertRaises(ValueError):
            blend_correction(np.eye(4),np.eye(4),0)

    def test_stationary_check_and_filter_at_distant_odometry_origin(self):
        odom = pose_matrix([100,0,0],[0,0,0,1])
        measured_base = pose_matrix([100,0,0],[0,0,math.sin(0.02/2),math.cos(0.02/2)])
        correction = measured_base@np.linalg.inv(odom)
        self.assertGreater(np.linalg.norm(correction[:3,3]),1.0)
        self.assertTrue(static_correction_is_consistent(np.eye(4),correction,odom,odom))
        filtered_base = blend_correction(np.eye(4),correction,0.25,odom)@odom
        np.testing.assert_allclose(filtered_base[:3,3],[100,0,0],atol=1e-8)

    def test_stationary_false_match_jump_rejected(self):
        prior = np.eye(4)
        jitter = pose_matrix([0.147,0,0],[0,0,math.sin(math.radians(5.2)/2),math.cos(math.radians(5.2)/2)])
        tiny_odom_motion = pose_matrix([0.001,0.002,0],[0,0,0,1])
        self.assertFalse(static_correction_is_consistent(prior,jitter,prior,tiny_odom_motion))
        small = pose_matrix([0.01,0.01,0],[0,0,math.sin(0.01/2),math.cos(0.01/2)])
        self.assertTrue(static_correction_is_consistent(prior,small,prior,tiny_odom_motion))

    def test_robot_motion_and_explicit_reseed_allowed(self):
        prior = np.eye(4)
        correction = pose_matrix([0.1,0,0],[0,0,0,1])
        moving_odom = pose_matrix([0.4,0,0],[0,0,0,1])
        self.assertTrue(static_correction_is_consistent(prior,correction,prior,moving_odom))
        self.assertTrue(static_correction_is_consistent(prior,correction,None,prior))

    def test_known_3d_pose_and_pcd_reload(self):
        points = room()
        truth = pose_matrix([0.45,-0.23,0.10],[0,0,math.sin(0.10/2),math.cos(0.10/2)])
        scan = transform_points(points,np.linalg.inv(truth))
        guess = pose_matrix([0.35,-0.15,0.06],[0,0,math.sin(0.06/2),math.cos(0.06/2)])
        with tempfile.TemporaryDirectory() as tmp:
            target = cloud(points)
            self.assertTrue(o3d.io.write_point_cloud(str(Path(tmp)/'map.pcd'),target,compressed=True))
            target = o3d.io.read_point_cloud(str(Path(tmp)/'map.pcd'))
            result = align(cloud(scan),target,guess)
        self.assertTrue(result.accepted,result)
        np.testing.assert_allclose(result.transform,truth,atol=0.02)

    def test_no_overlap_is_rejected(self):
        target = cloud(room())
        result = align(cloud(room()+[80,0,0]),target,np.eye(4))
        self.assertFalse(result.accepted)
        self.assertEqual(result.fitness,0)

    def test_large_pose_correction_is_rejected(self):
        target = cloud(room())
        moved = cloud(room()-[0.3,0,0])
        result = align(moved,target,np.eye(4),max_translation=0.05)
        self.assertFalse(result.accepted)


if __name__ == '__main__':
    unittest.main()
