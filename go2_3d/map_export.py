"""Shared, ROS-independent export of a complete mapping revision.

Both the original lidar mapper and the camera-assisted graph backend use this
path. Callers must supply points, free-space evidence and start pose from the
same committed revision; this function never mixes live mutable map state.
"""
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

from denoise import clean_directory
from geometry import ObservedGrid
from ground import estimate_ground
from map_library import map_name, new_directory, publish
from map_start import validate_start_pose
from navigation_grid import build_bundle
from registration import cloud


def save_snapshot(points, free, start_pose, parameters, loop_report, map_type):
    """Save retained source + leveled navigation bundle, returning its Path.

    The selected map is changed only after the common geometry validation in
    ``build_bundle`` succeeds. Existing saved bundles remain untouched. On
    failure, the exception identifies the retained partial source directory.
    """
    # Callers already supply a stable committed snapshot. Avoid a second
    # 240 MB copy for a ten-million-point map; exporters never mutate it.
    points = np.asarray(points, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or
            not np.isfinite(points).all()):
        raise ValueError('No usable finite 3D map snapshot (at least 100 XYZ points required)')
    start_pose = validate_start_pose(start_pose)
    parameters = dict(parameters)
    root = Path(parameters['output_directory'])
    name = map_name(parameters['map_name'])
    directory = new_directory(root, name)
    try:
        directory.mkdir(parents=True, exist_ok=False)
        # Preserve the committed MappingStore resolution without re-thinning.
        if not o3d.io.write_point_cloud(str(directory / 'map.pcd'),
                                       cloud(points, None), compressed=True):
            raise RuntimeError('PCD write failed')
        grid = ObservedGrid(parameters['grid_resolution'])
        grid.free = set(free)
        ground = estimate_ground(points, parameters['floor_z'])
        grid.export(points, directory, parameters['floor_z'],
                    parameters['obstacle_min_height'],
                    parameters['obstacle_max_height'], ground)
        denoise_report = clean_directory(directory)
        (directory / 'metadata.yaml').write_text(yaml.safe_dump(dict(
            frame_id='map', parameters=parameters,
            map_start_pose=start_pose.tolist(), ground_model=ground,
            navigation_denoise=denoise_report, loop_closure=dict(loop_report),
            map_type=str(map_type), point_count=len(points))))
        aligned = new_directory(root, name, 'leveled-')
        build_bundle(root, directory, aligned)
        publish(root, aligned)
        return aligned
    except Exception as error:
        raise RuntimeError(f'{error}; partial files, if any: {directory}') from error
