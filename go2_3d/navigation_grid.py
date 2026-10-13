"""Shared ground-aligned PCD, grid and standing-height contract for every map."""
import math
from pathlib import Path
import tempfile
import shutil
import numpy as np
import yaml
from denoise import clean_directory
from free_space import resample_free, restore_ancestor_free
from geometry import ObservedGrid, transform_points
from ground import estimate_ground, ground_alignment
from map_library import inside, validate_bundle


def current_settings():
    """Saved metadata describes geometry; the checkout supplies current policy."""
    params = yaml.safe_load(Path(__file__).with_suffix('.yaml').read_text())
    values = {name: float(params[name]) for name in
              ('grid_resolution', 'obstacle_min_height', 'obstacle_max_height')}
    if (not all(math.isfinite(value) for value in values.values()) or
            not 0.025 <= values['grid_resolution'] <= 0.20 or
            not 0 <= values['obstacle_min_height'] < values['obstacle_max_height']):
        raise ValueError('Invalid shared navigation grid/height settings in navigation_grid.yaml')
    return values


def prepare(root, output):
    """Prepare a complete common-coordinate runtime copy of any selected map."""
    root, output = Path(root).resolve(), Path(output).resolve()
    source = validate_bundle(inside(root, (root/'latest').resolve(strict=True)))
    if output == root or root in output.parents or output in source.parents:
        raise ValueError('Runtime navigation output must be separate from saved maps')
    return build_bundle(root, source, output)


def build_bundle(root, source, output):
    """Apply the same validated geometry at save and at localization/navigation.

    Transform the complete PCD, grid evidence and ground model together. Keep
    every 3D point and the standing-height contract; never flatten the scene.
    Source files and selection links are unchanged, including on failures.
    """
    import open3d as o3d
    root, output = Path(root).resolve(), Path(output).resolve()
    source = validate_bundle(inside(root, Path(source)))
    if source == output or source in output.parents or output in source.parents:
        raise ValueError('Aligned output must be separate from its source bundle')
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())
    pc = o3d.io.read_point_cloud(str(source/'map.pcd'))
    points = np.asarray(pc.points)
    if len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('Missing/nonfinite 3D map; cannot prepare navigation')
    policy = current_settings()
    ground = metadata.get('ground_model') or estimate_ground(
        points, metadata.get('parameters', {}).get('floor_z', -0.30))
    transform, model, body_height = ground_alignment(
        points, ground, metadata.get('body_height_above_ground'))
    aligned_points = transform_points(points, transform)
    grid = ObservedGrid(policy['grid_resolution'])
    grid.free = resample_free(source, ground, transform, grid.resolution,
                              grid.max_cells, allow_coarser=True)
    restored = 0
    if metadata.get('ground_aligned') and metadata.get('free_area_version', 0) < 2:
        restored = restore_ancestor_free(root, metadata, grid, target_from_map=transform)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.navigation-grid-', dir=output.parent) as tmp:
        staged = Path(tmp)
        if np.allclose(transform, np.eye(4), atol=1e-10):
            shutil.copy2(source/'map.pcd', staged/'map.pcd')
        else:
            pc.transform(transform)
            if not o3d.io.write_point_cloud(str(staged/'map.pcd'), pc, compressed=True):
                raise RuntimeError('Aligned PCD write failed')
        grid.export(aligned_points, staged, floor_z=0., min_height=policy['obstacle_min_height'],
                    max_height=policy['obstacle_max_height'], ground_model=model)
        report = clean_directory(staged)
        runtime_metadata = dict(metadata, source_map=str(source),
                                parameters={**metadata.get('parameters', {}), **policy, 'floor_z':0.},
                                ground_model=model, source_ground_model=ground,
                                ground_aligned=True, body_height_above_ground=body_height,
                                leveled_from=str(source), map_from_previous_map=transform.tolist(),
                                map_from_source_map=transform.tolist(), point_count=len(points),
                                initial_pose_reference='z=0: standing body above floor; nonzero z: explicit body height',
                                navigation_denoise=report, free_area_version=2, restored_free_cells=restored)
        (staged/'metadata.yaml').write_text(yaml.safe_dump(runtime_metadata))
        validate_bundle(staged)
        output.mkdir(parents=True, exist_ok=True)
        for name in ('map.pcd', 'nav.pgm', 'observed_free.npz', 'metadata.yaml', 'nav.yaml'):
            (staged/name).replace(output/name)
    print(f'Prepared ground-aligned map: {source} -> {output}; '
          f'resolution={grid.resolution:.3f}m; heights='
          f'{policy["obstacle_min_height"]}..{policy["obstacle_max_height"]}m; '
          f'body_height={body_height:.3f}m; floor_z=0; PCD_points={len(points)}; '
          f'denoise={report}; original bundle retained', flush=True)
    return output
