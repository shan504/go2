"""Ground-plane estimation and height classification without changing map XYZ."""
import numpy as np


def estimate_ground(points, expected_z=-0.30):
    import open3d as o3d
    points = np.asarray(points)
    # Fit near the first body/map origin, below the standing body. Do not let
    # an arbitrary tabletop or ceiling become ground merely by being largest.
    mask = (np.isfinite(points).all(axis=1) &
            (np.linalg.norm(points[:,:2],axis=1)<3.0) &
            (points[:,2]>expected_z-0.5) & (points[:,2]<-0.10))
    nearby = points[mask]
    if len(nearby)>30000:
        nearby = nearby[np.random.default_rng(42).choice(len(nearby),30000,replace=False)]
    original_count = len(nearby)
    candidates = []
    for _ in range(5):
        if len(nearby)<150:
            break
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(nearby))
        plane,indices = pc.segment_plane(distance_threshold=0.025,ransac_n=3,num_iterations=150)
        plane = np.asarray(plane,dtype=float)
        plane /= np.linalg.norm(plane[:3])
        if plane[2]<0:
            plane = -plane
        support = nearby[indices]
        remaining = np.ones(len(nearby),dtype=bool)
        remaining[indices] = False
        nearby = nearby[remaining]
        if plane[2]<np.cos(np.deg2rad(12)) or len(support)<max(150,0.25*original_count):
            continue
        height = -plane[3]/plane[2]
        area = len(np.unique(np.floor(support[:,:2]/0.2).astype(int),axis=0))*0.04
        if (not expected_z-0.5<height<-0.10 or area<1.0 or
                np.min(np.ptp(support[:,:2],axis=0))<0.8 or
                np.min(np.linalg.norm(support[:,:2],axis=1))>1.0):
            continue
        residual = support@plane[:3]+plane[3]
        candidates.append(dict(plane=plane.tolist(),height_at_origin=float(height),
                               support_points=len(support),observed_area=float(area),
                               support_fraction=float(len(support)/original_count),
                               rmse=float(np.sqrt(np.mean(residual**2)))))
    if not candidates:
        raise ValueError('No broad ground plane below body; keep original map and inspect PCD/ground height')
    ground = min(candidates,key=lambda item:item['height_at_origin'])
    return ground


def heights(points, model):
    plane = np.asarray(model['plane'],dtype=float)
    return np.asarray(points)@plane[:3]+plane[3]


def floor_and_obstacles(points, model, min_height=0.10, max_height=1.5):
    h = heights(points,model)
    floor = points[np.abs(h)<=0.035]
    obstacles = points[(h>=min_height)&(h<=max_height)]
    return floor,obstacles
