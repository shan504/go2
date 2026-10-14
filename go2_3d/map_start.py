"""The mapping start pose travels with the complete saved map coordinates."""
from pathlib import Path
import numpy as np
import yaml
from ground import body_seed_position


def validate_start_pose(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-8) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)):
        raise ValueError('Invalid saved map_start_pose; expected a finite rigid body pose')
    return matrix.copy()


def start_pose_from_metadata(metadata):
    if 'map_start_pose' in metadata:
        return validate_start_pose(metadata['map_start_pose'])
    # Older raw maps put the first body position and heading at zero. Older
    # aligned maps without retained provenance still retain standing height.
    # GICP must refine this legacy hint; no pose is reported valid from it.
    matrix = np.eye(4)
    matrix[:3, 3] = body_seed_position([0., 0., 0.], metadata.get('ground_model'),
                                      metadata.get('body_height_above_ground'))
    return matrix


def _ancestor(root, directory, reference):
    recorded = Path(reference)
    candidates = [recorded] if recorded.is_absolute() else [directory/recorded, root/recorded]
    if recorded.is_absolute() and recorded.parts[:2] == ('/', 'maps'):
        candidates.append(root/Path(*recorded.parts[2:]))
    for candidate in candidates:
        candidate = candidate.resolve()
        if root not in candidate.parents:
            continue
        if (candidate/'metadata.yaml').is_file():
            return candidate
    return None


def resolve_start_pose(root, directory, seen=None):
    """Recover older retained alignment chains without changing originals."""
    root, directory = Path(root).resolve(), Path(directory).resolve()
    if root not in directory.parents:
        raise ValueError('Map start provenance must stay within the map library')
    seen = set() if seen is None else set(seen)
    if directory in seen or len(seen) >= 32:
        raise ValueError('Cyclic or excessive map start provenance')
    seen.add(directory)
    metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
    if 'map_start_pose' in metadata:
        return start_pose_from_metadata(metadata)
    if 'map_from_previous_map' in metadata:
        transform = validate_start_pose(metadata['map_from_previous_map'])
        predecessor = _ancestor(root, directory, metadata['leveled_from']) if metadata.get('leveled_from') else None
        if predecessor is not None:
            return validate_start_pose(transform @ resolve_start_pose(root, predecessor, seen))
        if not np.allclose(transform, np.eye(4), atol=1e-8):
            # One retained leveling step still describes the raw body-origin
            # convention even when its original PCD has been exported away.
            return transform
    return start_pose_from_metadata(metadata)
