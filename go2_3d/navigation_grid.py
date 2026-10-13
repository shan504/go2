"""Apply current shared navigation settings to any saved map, without moving XYZ."""
import math
from pathlib import Path
import tempfile
import numpy as np
import yaml
from denoise import clean_directory
from free_space import resample_free, restore_ancestor_free
from geometry import ObservedGrid
from ground import estimate_ground
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
    """Regenerate the selected map's runtime grid under today's shared policy.

    Read the full PCD and transfer observed free *areas*, never just cell
    centres. The source bundle, latest links and localization coordinates
    remain intact. All files are generated successfully before publication.
    """
    import open3d as o3d
    root, output = Path(root).resolve(), Path(output).resolve()
    source = validate_bundle(inside(root, (root/'latest').resolve(strict=True)))
    if output == root or root in output.parents or output in source.parents:
        raise ValueError('Runtime navigation output must be separate from saved maps')
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())
    points = np.asarray(o3d.io.read_point_cloud(str(source/'map.pcd')).points)
    if len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('Missing/nonfinite 3D map; cannot prepare navigation')
    policy = current_settings()
    ground = metadata.get('ground_model') or estimate_ground(
        points, metadata.get('parameters', {}).get('floor_z', -0.30))
    grid = ObservedGrid(policy['grid_resolution'])
    grid.free = resample_free(source, ground, np.eye(4), grid.resolution,
                              grid.max_cells, allow_coarser=True)
    restored = 0
    if metadata.get('ground_aligned') and metadata.get('free_area_version', 0) < 2:
        restored = restore_ancestor_free(root, metadata, grid)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.navigation-grid-', dir=output.parent) as tmp:
        staged = Path(tmp)
        grid.export(points, staged, min_height=policy['obstacle_min_height'],
                    max_height=policy['obstacle_max_height'], ground_model=ground)
        report = clean_directory(staged)
        runtime_metadata = dict(metadata, source_map=str(source),
                                parameters={**metadata.get('parameters', {}), **policy},
                                ground_model=ground, navigation_denoise=report,
                                free_area_version=2, restored_free_cells=restored)
        (staged/'metadata.yaml').write_text(yaml.safe_dump(runtime_metadata))
        output.mkdir(parents=True, exist_ok=True)
        for name in ('nav.pgm', 'observed_free.npz', 'metadata.yaml', 'nav.yaml'):
            (staged/name).replace(output/name)
    print(f'Prepared navigation grid: {source} -> {output}; '
          f'resolution={grid.resolution:.3f}m; heights='
          f'{policy["obstacle_min_height"]}..{policy["obstacle_max_height"]}m; '
          f'denoise={report}; saved PCD and coordinates unchanged', flush=True)
    return output
