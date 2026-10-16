"""Transactional, spatially tiled storage for a complete fine-resolution map.

Voxel coordinates are always ``floor(XYZ / resolution)`` in the map frame.
Unlike Open3D's default ``voxel_down_sample``, the voxel origin does not move
when a new observation changes the cloud's bounds. Tiles align with both voxel
grids, so a retained voxel belongs to exactly one tile, including at negative
coordinates. No tile is evicted and no point-count limit changes resolution.

Only tiles touched by an observation are rebuilt. Whole-map arrays are lazy,
revision-cached snapshots; registration should normally use ``nearby`` instead.
"""
from dataclasses import dataclass
from itertools import product
import threading
from types import MappingProxyType

import numpy as np


def _readonly(array):
    array.setflags(write=False)
    return array


def _points(value):
    array = np.array(value, dtype=np.float64, copy=True)
    if array.size == 0:
        return array.reshape(0, 3)
    if array.ndim != 2 or array.shape[1] != 3 or not np.isfinite(array).all():
        raise ValueError('points must be a finite N x 3 array')
    return array


def _keys(points, resolution):
    scaled = np.floor(points / resolution)
    # Leave ample room for integer tile arithmetic and neighbour bounds.
    if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 2**62):
        raise ValueError('point coordinates exceed the supported voxel grid')
    return scaled.astype(np.int64)


def _aggregate(keys, sums, weights, resolution):
    """Aggregate observations without reassigning previously retained voxels."""
    if not len(keys):
        return (np.empty((0, 3), dtype=np.int64), np.empty((0, 3)),
                np.empty(0, dtype=np.int64), np.empty((0, 3)))
    # np.unique(axis=0) performs a costly structured-row sort. Retained
    # keys are already ordered; lexsort can reuse that ordering efficiently,
    # then reduce contiguous voxel runs without inverse-index scatter adds.
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    ordered = keys[order]
    starts = np.flatnonzero(np.r_[True, np.any(ordered[1:] != ordered[:-1], axis=1)])
    unique = ordered[starts]
    counts = np.add.reduceat(weights[order], starts)
    totals = np.add.reduceat(sums[order], starts, axis=0)
    if np.any(counts <= 0) or not np.isfinite(totals).all():
        raise ValueError('voxel observation accumulation overflowed')
    centers = totals / counts[:, None]
    lower, upper = unique * resolution, (unique + 1) * resolution
    # A rounded mean of returns immediately below a boundary can equal that
    # boundary. Keep its representative in its original canonical voxel.
    centers = np.maximum(lower, np.minimum(centers, np.nextafter(upper, -np.inf)))
    return unique, totals, counts, centers


@dataclass(frozen=True)
class _Tile:
    keys: np.ndarray
    sums: np.ndarray
    weights: np.ndarray
    fine: np.ndarray
    coarse: np.ndarray


@dataclass(frozen=True)
class MappingUpdate:
    """Prepared immutable tile replacements, valid for one store revision."""
    base_revision: int
    total_points: int
    tiles: tuple
    _owner: object
    _replacement_tiles: object


class MappingStore:
    def __init__(self, map_voxel_size, registration_voxel_size, tile_size=6.0):
        for name, value in (('map_voxel_size', map_voxel_size),
                            ('registration_voxel_size', registration_voxel_size),
                            ('tile_size', tile_size)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.map_voxel_size = float(map_voxel_size)
        self.registration_voxel_size = float(registration_voxel_size)
        self.tile_size = float(tile_size)
        for resolution in (self.map_voxel_size, self.registration_voxel_size):
            cells = self.tile_size / resolution
            if not np.isclose(cells, round(cells), rtol=0, atol=1e-8) or cells < 1:
                raise ValueError('tile_size must be a whole multiple of both voxel sizes')
        self._fine_cells_per_tile = round(self.tile_size / self.map_voxel_size)
        self._tiles = MappingProxyType({})
        self._count = 0
        self._revision = 0
        self._owner = object()
        self._lock = threading.RLock()
        self._fine_cache = None
        self._coarse_cache = None

    @property
    def count(self):
        with self._lock:
            return self._count

    @property
    def revision(self):
        with self._lock:
            return self._revision

    def _build_tile(self, previous, keys, points):
        weights = np.ones(len(points), dtype=np.int64)
        if previous is not None:
            keys = np.vstack((previous.keys, keys))
            points = np.vstack((previous.sums, points))
            weights = np.concatenate((previous.weights, weights))
        keys, sums, weights, fine = _aggregate(
            keys, points, weights, self.map_voxel_size)
        coarse_keys = _keys(fine, self.registration_voxel_size)
        _, _, _, coarse = _aggregate(
            coarse_keys, fine, np.ones(len(fine), dtype=np.int64),
            self.registration_voxel_size)
        return _Tile(*map(_readonly, (keys, sums, weights, fine, coarse)))

    def prepare(self, points):
        """Build replacements without changing any visible map state.

        Commit may reject this update if another update was committed while
        preparation ran. Reprepare against the new revision in that case.
        """
        points = _points(points)
        keys = _keys(points, self.map_voxel_size)
        tile_keys = np.floor_divide(keys, self._fine_cells_per_tile)
        unique, inverse = np.unique(tile_keys, axis=0, return_inverse=True)
        identifiers = [tuple(map(int, key)) for key in unique]
        with self._lock:
            revision, count = self._revision, self._count
            previous_tiles = self._tiles
            previous = [previous_tiles.get(key) for key in identifiers]
        # Sort once rather than scanning the entire observation per tile.
        order = np.argsort(inverse, kind='stable')
        groups = np.split(order, np.cumsum(np.bincount(inverse))[:-1]) if len(order) else []
        replacements = []
        for identifier, old, indices in zip(identifiers, previous, groups):
            new = self._build_tile(old, keys[indices], points[indices])
            count += len(new.fine) - (len(old.fine) if old is not None else 0)
            replacements.append((identifier, new))
        complete = dict(previous_tiles)
        complete.update(replacements)
        return MappingUpdate(revision, count, tuple(replacements), self._owner,
                             MappingProxyType(complete))

    def commit(self, update):
        """Atomically publish a prepared update; reject stale/foreign updates."""
        if not isinstance(update, MappingUpdate) or update._owner is not self._owner:
            raise ValueError('mapping update belongs to another store')
        with self._lock:
            if update.base_revision != self._revision:
                raise ValueError('mapping update is stale; prepare it again')
            # All validation/allocation completes before changing live refs.
            self._tiles = update._replacement_tiles
            self._count = update.total_points
            self._revision += 1
            self._fine_cache = self._coarse_cache = None

    def _snapshot(self, coarse):
        attribute = '_coarse_cache' if coarse else '_fine_cache'
        with self._lock:
            cached = getattr(self, attribute)
            if cached is not None:
                return cached
            revision = self._revision
            arrays = [tile.coarse if coarse else tile.fine for tile in self._tiles.values()]
        snapshot = _readonly(np.vstack(arrays) if arrays else np.empty((0, 3)))
        with self._lock:
            if revision == self._revision:
                setattr(self, attribute, snapshot)
        return snapshot

    def snapshot(self):
        """Return all retained fine points; old returned arrays stay stable."""
        return self._snapshot(False)

    def registration_snapshot(self):
        """Return the complete coarse map for diagnostics/compatibility."""
        return self._snapshot(True)

    def nearby(self, center, radius):
        """Return coarse points in a sphere, reading only intersecting tiles."""
        center = np.asarray(center, dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError('center must contain three finite coordinates')
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError('radius must be finite and positive')
        lower = _keys((center - radius)[None, :], self.tile_size)[0]
        upper = _keys((center + radius)[None, :], self.tile_size)[0]
        arrays = []
        with self._lock:
            for key in product(*(range(int(a), int(b) + 1) for a, b in zip(lower, upper))):
                minimum = np.asarray(key, dtype=float) * self.tile_size
                distance = np.maximum(np.maximum(minimum-center, center-(minimum+self.tile_size)), 0)
                if np.dot(distance, distance) > radius * radius:
                    continue
                tile = self._tiles.get(key)
                if tile is not None:
                    arrays.append(tile.coarse)
        if not arrays:
            return _readonly(np.empty((0, 3)))
        points = np.vstack(arrays)
        delta = points - center
        return _readonly(points[np.einsum('ij,ij->i', delta, delta) <= radius * radius])
