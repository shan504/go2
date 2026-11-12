"""Apply an external pose graph to retained GICP body-frame observations.

This module does not detect loops, optimize a graph, or change GICP odometry.
RTAB-Map supplies optimized poses. Their corrections are interpolated in time
between graph anchors and replayed through the existing fine map/free-ray
pipeline. A rigid transform of an already fused map cannot do this job.

Graph inputs are complete retained anchor snapshots. Every anchor needs the
exact original GICP pose, recorded with ``record_pose`` even if that scan was
not selected as a mapping keyframe. Missing source poses are never invented.
"""
from bisect import bisect_right
from dataclasses import dataclass
import threading
from types import MappingProxyType

import numpy as np

from geometry import ObservedGrid, pose_matrix, quaternion_from_matrix
from map_start import validate_start_pose
from mapping_rebuild import MappingObservation, append_rebuilt, rebuild
from mapping_store import MappingStore


def _stamp(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value <= 0:
        raise ValueError('timestamp must be a positive integer in nanoseconds')
    return int(value)


def _pose(value):
    result = validate_start_pose(value)
    result.setflags(write=False)
    return result


def _same(left, right):
    return np.allclose(left, right, rtol=0., atol=1e-10)


def _interpolate(left, right, fraction):
    """Interpolate translation and shortest-arc quaternion rotation."""
    first, second = quaternion_from_matrix(left), quaternion_from_matrix(right)
    dot = float(np.dot(first, second))
    if dot < 0:
        second, dot = -second, -dot
    dot = float(np.clip(dot, -1., 1.))
    if dot > .9995:
        rotation = first + fraction * (second-first)
    else:
        angle = np.arccos(dot)
        rotation = (np.sin((1-fraction)*angle)*first + np.sin(fraction*angle)*second) / np.sin(angle)
    translation = (1-fraction)*left[:3, 3] + fraction*right[:3, 3]
    return _pose(pose_matrix(translation, rotation))


def correction_at(stamp_ns, anchors):
    """Interpolate corrections; use the nearest correction outside anchors.

    ``anchors`` is a sorted sequence of ``(stamp_ns, map_from_gicp)`` pairs.
    Unanchored tail scans may be displayed but must not pass the save gate.
    """
    if not anchors:
        return _pose(np.eye(4))
    stamps = [item[0] for item in anchors]
    index = bisect_right(stamps, stamp_ns)
    if index == 0:
        return anchors[0][1]
    if index == len(anchors):
        return anchors[-1][1]
    left_stamp, left = anchors[index-1]
    right_stamp, right = anchors[index]
    fraction = (stamp_ns-left_stamp) / (right_stamp-left_stamp)
    return _interpolate(left, right, fraction)


@dataclass(frozen=True)
class GraphMapSnapshot:
    """Stable geometry/metadata from one committed map revision."""
    points: np.ndarray
    free: frozenset
    stamps: tuple
    poses: tuple
    observations: tuple
    start_pose: np.ndarray
    correction: np.ndarray
    map_revision: int
    graph_revision: int


@dataclass(frozen=True)
class GraphProposal:
    """Private rebuilt candidate; commit rejects any intervening input."""
    revision: int
    source_generation: int
    previous_graph_revision: int
    anchors: tuple
    graph_poses: object
    poses: tuple
    store: object
    grid: object
    rebuilt: bool
    _owner: object


class GraphMap:
    """Single map journal with transactional external graph corrections.

    Append and graph commit are serialized. ``prepare_graph`` can run outside
    the caller's lock; if another input arrives, committing that candidate
    raises rather than dropping a scan. Retry with a new snapshot in that case.
    """
    def __init__(self, map_voxel_size, registration_voxel_size, grid_resolution):
        if not np.isfinite(grid_resolution) or grid_resolution <= 0:
            raise ValueError('grid_resolution must be finite and positive')
        self.resolutions = (map_voxel_size, registration_voxel_size, grid_resolution)
        self._store = MappingStore(map_voxel_size, registration_voxel_size)
        self._grid = ObservedGrid(grid_resolution)
        self._source_poses = {}
        self._observations = ()
        self._stamps = ()
        self._poses = ()
        self._anchors = ()
        self._graph_poses = MappingProxyType({})
        self._graph_revision = 0
        self._map_revision = 0
        self._generation = 0
        self._rebuilds = 0
        self._owner = object()
        self._lock = threading.RLock()

    def record_pose(self, stamp_ns, pose):
        """Retain accepted GICP odometry, independent of mapping keyframes."""
        stamp_ns, matrix = _stamp(stamp_ns), _pose(pose)
        with self._lock:
            old = self._source_poses.get(stamp_ns)
            if old is not None:
                if not _same(old, matrix):
                    raise ValueError('conflicting GICP poses at the same timestamp')
                return
            if self._source_poses and stamp_ns < next(reversed(self._source_poses)):
                raise ValueError('GICP odometry timestamps must increase')
            self._source_poses[stamp_ns] = matrix
            self._generation += 1

    def append(self, stamp_ns, points, pose):
        """Retain one local keyframe and incrementally update current geometry."""
        stamp_ns = _stamp(stamp_ns)
        observation = MappingObservation(points, pose)
        if not len(observation.points_body):
            raise ValueError('mapping keyframe must contain local points')
        with self._lock:
            if self._stamps and stamp_ns <= self._stamps[-1]:
                raise ValueError('mapping keyframe timestamps must strictly increase')
            old = self._source_poses.get(stamp_ns)
            if old is not None and not _same(old, observation.pose):
                raise ValueError('keyframe pose conflicts with recorded GICP odometry')
            if old is None and self._source_poses and stamp_ns < next(reversed(self._source_poses)):
                raise ValueError('keyframe source pose missing from accepted GICP journal')
            corrected = _pose(correction_at(stamp_ns, self._anchors) @ observation.pose)
            observations = self._observations + (observation,)
            stamps, poses = self._stamps + (stamp_ns,), self._poses + (corrected,)
            # Existing helper prepares the fine voxels AND rays before either
            # commit. A malformed point cannot leave a partial map update.
            append_rebuilt(self._store, self._grid, observation, corrected)
            self._source_poses.setdefault(stamp_ns, observation.pose)
            self._observations, self._stamps, self._poses = observations, stamps, poses
            self._generation += 1
            self._map_revision += 1

    def prepare_graph(self, graph_poses, revision=None):
        """Build a complete corrected map candidate without changing live state."""
        poses_by_stamp = {_stamp(stamp): _pose(pose) for stamp, pose in graph_poses.items()}
        if not poses_by_stamp:
            raise ValueError('RTAB-Map graph has no usable anchored poses')
        with self._lock:
            revision = self._graph_revision+1 if revision is None else _stamp(revision)
            if revision <= self._graph_revision:
                raise ValueError('stale or conflicting graph revision')
            if not self._graph_poses.keys() <= poses_by_stamp.keys():
                raise ValueError('graph snapshot omits retained anchors; supply the complete graph')
            missing = poses_by_stamp.keys() - self._source_poses.keys()
            if missing:
                raise ValueError('graph anchors lack exact recorded GICP poses: '+str(sorted(missing)))
            # RTAB-Map stores transforms as float32. Casting an unchanged
            # source pose through that representation produces micrometre
            # corrections. Canonicalize ONLY <=1e-5 component roundoff to an
            # existing correction (or identity), rather than rebuilding a
            # ten-million-point map for every newly inserted graph node.
            # Poses and geometry both use these same canonical corrections.
            previous = dict(self._anchors)
            anchors = []
            identity = _pose(np.eye(4))
            for stamp in sorted(poses_by_stamp):
                correction = _pose(poses_by_stamp[stamp] @ np.linalg.inv(self._source_poses[stamp]))
                references = [previous[stamp]] if stamp in previous else []
                if anchors:
                    references.append(anchors[-1][1])
                references.append(identity)
                for reference in references:
                    quantized_expected = (reference @ self._source_poses[stamp]).astype(np.float32).astype(np.float64)
                    if (np.allclose(correction, reference, rtol=0., atol=1e-5) or
                            np.allclose(poses_by_stamp[stamp], quantized_expected, rtol=0., atol=1e-5)):
                        correction = reference
                        break
                anchors.append((stamp, correction))
            anchors = tuple(anchors)
            observations, stamps = self._observations, self._stamps
            corrected = tuple(_pose(correction_at(stamp, anchors) @ observation.pose)
                              for stamp, observation in zip(stamps, observations))
            changed = any(not _same(old, new) for old, new in zip(self._poses, corrected))
            generation, previous_revision = self._generation, self._graph_revision
        store, grid = None, None
        if changed:
            store, grid = rebuild(observations, corrected, *self.resolutions)
        return GraphProposal(revision, generation, previous_revision, anchors,
                             MappingProxyType(poses_by_stamp), corrected,
                             store, grid, changed, self._owner)

    def commit_graph(self, proposal):
        """Publish rebuilt cloud, rays, poses and TF correction as one revision."""
        with self._lock:
            if not isinstance(proposal, GraphProposal) or proposal._owner is not self._owner:
                raise ValueError('graph proposal belongs to another map')
            if (proposal.source_generation != self._generation or
                    proposal.previous_graph_revision != self._graph_revision):
                raise ValueError('graph proposal is stale; retry with the current journal')
            if proposal.rebuilt:
                self._store, self._grid = proposal.store, proposal.grid
                self._map_revision += 1
                self._rebuilds += 1
            self._poses = proposal.poses
            self._anchors, self._graph_poses = proposal.anchors, proposal.graph_poses
            self._graph_revision = proposal.revision

    def apply_graph(self, graph_poses, revision=None):
        proposal = self.prepare_graph(graph_poses, revision)
        self.commit_graph(proposal)
        return proposal.rebuilt

    @property
    def correction(self):
        with self._lock:
            return self._anchors[-1][1] if self._anchors else _pose(np.eye(4))

    @property
    def start_pose(self):
        with self._lock:
            return self._poses[0] if self._poses else _pose(np.eye(4))

    def snapshot(self):
        with self._lock:
            return GraphMapSnapshot(self._store.snapshot(), frozenset(self._grid.free),
                                    self._stamps, self._poses, self._observations,
                                    self.start_pose, self.correction,
                                    self._map_revision, self._graph_revision)

    def ready_for_save(self):
        with self._lock:
            return bool(self._stamps and self._anchors and self._stamps[-1] <= self._anchors[-1][0])

    def report(self):
        with self._lock:
            first = self._anchors[0][0] if self._anchors else None
            last = self._anchors[-1][0] if self._anchors else None
            return dict(observations=len(self._stamps), source_poses=len(self._source_poses),
                        graph_anchors=len(self._anchors), graph_revision=self._graph_revision,
                        map_revision=self._map_revision, map_rebuilds=self._rebuilds,
                        point_count=self._store.count, last_observation_stamp_ns=self._stamps[-1] if self._stamps else None,
                        last_graph_stamp_ns=last,
                        initial_extrapolated_observations=sum(stamp < first for stamp in self._stamps) if first else 0,
                        pending_after_last_anchor=sum(stamp > last for stamp in self._stamps) if last else len(self._stamps),
                        ready_for_save=self.ready_for_save())
