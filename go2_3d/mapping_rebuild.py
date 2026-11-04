"""Rebuild complete mapping geometry and ray evidence after trajectory correction.

The source journal lives in body coordinates. Reprojecting fused voxels or old
free cells cannot undo a nonrigid trajectory change; every observation must be
replayed at its corrected pose with its corrected ray origin instead. All work
here creates new objects so callers can publish one complete revision atomically.
"""
from dataclasses import dataclass

import numpy as np

from geometry import ObservedGrid, transform_points
from map_start import validate_start_pose
from mapping_store import MappingStore
from registration import cloud


def _pose(value):
    matrix = validate_start_pose(value)
    matrix.setflags(write=False)
    return matrix


@dataclass(frozen=True)
class MappingObservation:
    """An accepted full-resolution body scan and its original mapping pose."""
    points_body: np.ndarray
    pose: np.ndarray

    def __post_init__(self):
        points = np.array(self.points_body, dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('mapping observation must be a finite N x 3 array')
        points.setflags(write=False)
        object.__setattr__(self, 'points_body', points)
        object.__setattr__(self, 'pose', _pose(self.pose))

    def with_pose(self, pose):
        """Share immutable body geometry while replacing only the map pose."""
        result = object.__new__(type(self))
        object.__setattr__(result, 'points_body', self.points_body)
        object.__setattr__(result, 'pose', _pose(pose))
        return result


def corrected_poses(observations, original_prefix, new_prefix):
    """Correct a journal prefix and carry its latest correction to newer scans.

    The prefix is the immutable snapshot optimized by a background worker. New
    observations can arrive while it runs. Their relative poses are retained by
    applying the last prefix pose's rigid change; none are dropped or assigned
    an unrelated correction. A stale/mismatched source prefix is an error.
    """
    observations = tuple(observations)
    original_prefix, new_prefix = tuple(original_prefix), tuple(new_prefix)
    count = len(original_prefix)
    if not count or len(new_prefix) != count or count > len(observations):
        raise ValueError('correction requires matching nonempty journal prefixes')
    originals = tuple(_pose(pose) for pose in original_prefix)
    corrected = tuple(_pose(pose) for pose in new_prefix)
    for observation, original in zip(observations, originals):
        if not np.allclose(observation.pose, original, rtol=0, atol=1e-10):
            raise ValueError('optimized prefix does not match the observation journal')
    change = corrected[-1] @ np.linalg.inv(originals[-1])
    return corrected + tuple(_pose(change @ observation.pose)
                             for observation in observations[count:])


def rebuild(observations, poses, map_voxel_size, registration_voxel_size,
            grid_resolution):
    """Return a fresh MappingStore and ObservedGrid at the original resolutions.

    Retain all original body points for fine voxel accumulation. Free rays use
    exactly the map-voxel-downsampled transformed returns used by live mapping,
    rather than rays through a coarse registration cloud or a warped old grid.
    A validation/allocation failure cannot mutate the journal or any live map.
    """
    observations, poses = tuple(observations), tuple(poses)
    if not observations or len(observations) != len(poses):
        raise ValueError('rebuild requires one pose per nonempty observation journal')
    if not np.isfinite(grid_resolution) or grid_resolution <= 0:
        raise ValueError('grid_resolution must be finite and positive')
    matrices = tuple(_pose(pose) for pose in poses)
    store = MappingStore(map_voxel_size, registration_voxel_size)
    grid = ObservedGrid(grid_resolution)
    for observation, matrix in zip(observations, matrices):
        append_rebuilt(store, grid, observation, matrix)
    return store, grid


def append_rebuilt(store, grid, observation, pose):
    """Replay one newer observation into a worker-private rebuild candidate.

    Callers discard the candidate on any error; never pass the live map here.
    Both fallible preparations finish before either geometry/grid is committed.
    """
    matrix = _pose(pose)
    mapped = transform_points(observation.points_body, matrix)
    update = store.prepare(mapped)
    endpoints = np.asarray(cloud(mapped, store.map_voxel_size).points)
    additions = grid.prepare_observation(endpoints, matrix[:3, 3])
    store.commit(update)
    grid.free.update(additions)
