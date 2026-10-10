#!/usr/bin/env python3
"""Level a saved 3D map and regenerate a finer grid without erasing obstacles."""
import argparse
import copy
from pathlib import Path
import shutil
import numpy as np
import open3d as o3d
import yaml
from geometry import ObservedGrid
from ground import estimate_ground,heights,level_transform
from repair_map import grid_stats,repair
from free_space import resample_free
from denoise import clean_directory
from map_library import inside, new_directory, publish, source_name


def level(root, resolution=0.05):
    if not np.isfinite(resolution) or not 0.025 <= resolution <= 0.20:
        raise ValueError('Grid resolution must be between 0.025 and 0.20 metres')
    root = Path(root).resolve()
    source = inside(root,(root/'latest').resolve(strict=True))
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())
    if metadata.get('ground_aligned') and np.isclose(metadata['parameters']['grid_resolution'],resolution):
        if metadata.get('free_area_version',0)<2:
            # Keep the already leveled PCD byte-for-byte; recover coarse free
            # areas from its retained predecessor in the same coordinates.
            return repair(root)
        print(f'ALREADY LEVELED: {source}; latest unchanged',flush=True)
        return source
    pc = o3d.io.read_point_cloud(str(source/'map.pcd'))
    points = np.asarray(pc.points)
    if len(points)<100 or not np.isfinite(points).all():
        raise ValueError('Missing/nonfinite 3D map; original retained')
    params = copy.deepcopy(metadata['parameters'])
    ground = metadata.get('ground_model') or estimate_ground(points,params.get('floor_z',-0.3))
    transform = level_transform(ground)
    support = points[(np.abs(heights(points,ground))<=0.035)&(np.linalg.norm(points[:,:2],axis=1)<3.0)]
    area = len(np.unique(np.floor(support[:,:2]/0.2).astype(int),axis=0))*0.04
    if len(support)<150 or area<1.0 or np.min(np.ptp(support[:,:2],axis=0))<0.8:
        raise ValueError('Saved ground lacks broad observed support; original retained')
    body_height = metadata.get('body_height_above_ground',float(transform[2,3]))
    if not 0.1 < body_height < 1.0:
        raise ValueError('Cannot infer standing body height; original retained')
    grid = ObservedGrid(resolution)
    grid.free = resample_free(source,ground,transform,resolution,grid.max_cells)
    destination = new_directory(root,source_name(root,source),'leveled-')
    destination.mkdir(parents=True)
    try:
        pc.transform(transform)
        leveled = np.asarray(pc.points)
        model = copy.deepcopy(ground)
        model.update(plane=[0.,0.,1.,0.],height_at_origin=0.)
        grid.export(leveled,destination,0.,params.get('obstacle_min_height',0.10),
                    params.get('obstacle_max_height',1.5),model)
        denoise_report = clean_directory(destination)
        if not o3d.io.write_point_cloud(str(destination/'map.pcd'),pc,compressed=True):
            raise RuntimeError('PCD write failed')
        params.update(grid_resolution=float(resolution),floor_z=0.)
        stats = dict(before=grid_stats(source),after=grid_stats(destination))
        metadata.update(parameters=params,ground_model=model,ground_aligned=True,
                        navigation_denoise=denoise_report,
                        free_area_version=2,
                        body_height_above_ground=float(body_height),point_count=len(leveled),
                        leveled_from=str(source),map_from_previous_map=transform.tolist(),
                        level_grid_stats=stats,
                        initial_pose_reference='z=0: standing body above floor; nonzero z: explicit body height',
                        level='Rigid ground alignment; all 3D points and physical obstacles retained')
        (destination/'metadata.yaml').write_text(yaml.safe_dump(metadata))
        publish(root,destination)
    except Exception:
        shutil.rmtree(destination)
        raise
    print(f'LEVELED 3D map: {destination}\nOriginal retained: {source}\n'
          f'PCD points retained: {len(points)}; ground z=0; standing body height={body_height:.4f}m\n'
          f'Grid resolution: {resolution:.3f}m; cells: {stats}\n'
          'Old goal coordinates must be selected again in the leveled map.',flush=True)
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps',default='/maps')
    parser.add_argument('--resolution',type=float,default=0.05)
    args = parser.parse_args()
    level(args.maps,args.resolution)
