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


def level_transform(model):
    """Rigidly rotate an upward ground normal to Z and put that plane at z=0."""
    plane = np.asarray(model['plane'],dtype=float)
    if plane.shape != (4,) or not np.isfinite(plane).all():
        raise ValueError('Invalid ground plane')
    norm = np.linalg.norm(plane[:3])
    if norm < 1e-8:
        raise ValueError('Invalid ground normal')
    plane /= norm
    n = plane[:3]
    if n[2] < np.cos(np.deg2rad(12)):
        raise ValueError('Ground normal must face up and be within 12 degrees of Z')
    v = np.cross(n,[0.,0.,1.])
    skew = np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
    transform = np.eye(4)
    transform[:3,:3] = np.eye(3)+skew+skew@skew/(1+n[2])
    transform[2,3] = plane[3]
    return transform


def body_seed_position(position, model=None, body_height=None):
    """Lift a conventional z=0 2D initialpose to standing body height.

    Legacy maps without a saved body-height contract and explicit nonzero Z
    retain their existing body-pose semantics. Height is normal to the floor.
    """
    result = np.asarray(position,dtype=float).copy()
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('Invalid initial position')
    if model is None or body_height is None or abs(result[2]) > 1e-8:
        return result
    plane = np.asarray(model['plane'],dtype=float)
    if (plane.shape != (4,) or not np.isfinite(plane).all() or
            plane[2] < 0.9 or not np.isclose(np.linalg.norm(plane[:3]),1.,atol=1e-5) or
            not np.isfinite(body_height) or not 0.1 < body_height < 1.0):
        raise ValueError('Invalid saved standing height/ground plane')
    result[2] = (body_height-plane[3]-plane[:2]@result[:2])/plane[2]
    return result


def floor_and_obstacles(points, model, min_height=0.10, max_height=1.5):
    h = heights(points,model)
    floor = points[np.abs(h)<=0.035]
    obstacles = points[(h>=min_height)&(h<=max_height)]
    return floor,obstacles
