"""External-odometry adapter for the fixed, official KISS-SLAM IROS25 backend.

GICP owns continuous scan matching. This adapter uses the original KISS-SLAM
VoxelMap, MapClosures/LoopCloser and g2o PoseGraphOptimizer; it never instantiates
KissICP. Submaps retain body scans through the mapper's existing observation
journal. A proposal changes neither that journal nor the live map until apply.

Only numpy is needed to import the configuration. Native dependencies are lazy
so selecting the original backend does not require the experimental environment.
"""
from dataclasses import dataclass, field
import math

import numpy as np


UPSTREAM_COMMIT = 'db81ee31b59bc7a18af3fec8641d5867646ab57a'
UPSTREAM_TAG = 'IROS25'


def _pose(value):
    matrix = np.array(value, dtype=np.float64, copy=True)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-7) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), rtol=0, atol=1e-5) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1, rtol=0, atol=1e-5)):
        raise ValueError('KISS backend requires finite, proper SE(3) poses')
    matrix.setflags(write=False)
    return matrix


def _angle(matrix):
    return float(np.arccos(np.clip((np.trace(matrix[:3, :3])-1)/2, -1, 1)))


@dataclass(frozen=True)
class KissConfig:
    # These affect backend submaps only, not live/save cloud density or GICP.
    # 10 m is an experimental initial choice, not a validated L1 farm setting.
    submap_distance: float = 10.0
    submap_max_observations: int = 80
    voxel_size: float = 0.5
    density_map_resolution: float = 0.5
    density_threshold: float = 0.05
    hamming_distance_threshold: int = 50
    inliers_threshold: int = 5
    overlap_threshold: float = 0.4
    min_loop_travel: float = 20.0
    min_loop_separation: int = 2
    max_iterations: int = 20
    min_geometry_points: int = 30
    max_correction_fraction: float = 0.30
    max_correction_rotation_degrees: float = 60.0
    graph_loop_translation_limit: float = 0.50
    graph_loop_rotation_degrees: float = 5.0

    def __post_init__(self):
        integer_fields = ('submap_max_observations', 'hamming_distance_threshold',
                          'inliers_threshold', 'min_loop_separation', 'max_iterations',
                          'min_geometry_points')
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            if name in integer_fields and int(value) != value:
                raise ValueError(f'{name} must be an integer')
        if self.overlap_threshold >= 1 or self.density_threshold >= 1:
            raise ValueError('overlap_threshold and density_threshold must be below one')
        if not np.isclose(self.voxel_size, self.density_map_resolution, rtol=0, atol=1e-12):
            raise ValueError('voxel_size must equal density_map_resolution for upstream voxel-overlap accounting')


@dataclass(frozen=True)
class _Constraint:
    source: int
    target: int
    transform: np.ndarray
    inliers: int = 0


@dataclass
class _Submap:
    anchor_observation: int
    anchor_travel: float
    observations: list = field(default_factory=list)
    sealed: bool = False
    query_points: object = None
    candidate: object = None


@dataclass(frozen=True)
class KissOptimization:
    poses: tuple
    original_poses: tuple
    loop_edges: tuple
    status: dict
    _revision: int
    _owner: object
    _constraints: tuple
    _keyposes: tuple


class KissBackend:
    """Single-worker-thread backend with two-phase map/graph commit semantics."""

    def __init__(self, config=None):
        self.config = config if config is not None else KissConfig()
        from check_kiss_env import verify_environment
        runtime = verify_environment()
        # Import only the upstream reusable backend, never kiss_slam.slam/KissICP.
        from kiss_slam.config import LoopCloserConfig, PoseGraphOptimizerConfig
        from kiss_slam.local_map_graph import LocalMap, LocalMapGraph
        from kiss_slam.loop_closer import LoopCloser
        from kiss_slam.pose_graph_optimizer import PoseGraphOptimizer
        from kiss_slam.voxel_map import VoxelMap
        from map_closures.config import MapClosuresConfig

        self._LocalMap, self._VoxelMap = LocalMap, VoxelMap
        self._Optimizer = PoseGraphOptimizer
        self._pgo_config = PoseGraphOptimizerConfig(max_iterations=self.config.max_iterations)
        self._closer = LoopCloser(LoopCloserConfig(
            detector=MapClosuresConfig(
                density_map_resolution=self.config.density_map_resolution,
                density_threshold=self.config.density_threshold,
                hamming_distance_threshold=self.config.hamming_distance_threshold,
                inliers_threshold=self.config.inliers_threshold),
            overlap_threshold=self.config.overlap_threshold))
        self._graph = LocalMapGraph()
        self._graph.graph.clear()  # The first real GICP pose fixes the gauge.
        self._poses, self._relative_poses, self._membership = [], [], []
        self._submaps, self._odometry_edges, self._loops = [], [], []
        self._tentative_loops = []
        self._active = None
        self._voxel = None
        self._travel = 0.0
        self._next_query = 0
        self._revision = 0
        self._owner = object()
        self._proposal = None
        self._fatal_error = ''
        self._status = dict(backend='kiss_slam', upstream_tag=UPSTREAM_TAG,
                            upstream_commit=UPSTREAM_COMMIT, state='waiting',
                            runtime={key: runtime[key] for key in ('python', 'versions', 'upstream_commit')},
                            observations=0, submaps=0, candidates=0,
                            geometric_verifications=0, constraints_added=0,
                            optimization_runs=0, committed_optimizations=0,
                            events=[])

    @property
    def observation_count(self):
        return len(self._poses)

    @property
    def status(self):
        result = dict(self._status)
        result['events'] = [dict(event) for event in self._status['events']]
        return result

    def _event(self, stage, **details):
        self._status['events'] = (self._status['events'] + [dict(stage=stage, **details)])[-64:]
        self._status.update(state=stage, **details)

    def append(self, points, pose):
        points = np.array(points, dtype=np.float64, copy=True)
        matrix = _pose(pose)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('KISS observation must be a finite N x 3 body cloud')
        if not len(points):
            raise ValueError('KISS observation cannot be empty')
        if self._poses:
            travel = self._travel + float(np.linalg.norm(matrix[:3, 3]-self._poses[-1][:3, 3]))
        else:
            travel = 0.0
        if self._active is None:
            index = len(self._submaps)
            local_map = self._LocalMap(index, matrix.copy())
            local_map.local_trajectory.clear()
            voxel = self._VoxelMap(self.config.voxel_size)
            voxel.integrate_frame(points, np.eye(4))
            # Do not expose an empty node if voxel construction fails.
            if index:
                previous = self._graph[index-1].keypose
                self._odometry_edges.append(_Constraint(
                    index, index-1, _pose(np.linalg.inv(previous) @ matrix)))
            self._graph.graph[index] = local_map
            self._submaps.append(_Submap(len(self._poses), travel))
            self._active, self._voxel = index, voxel
            relative = _pose(np.eye(4))
        else:
            relative = _pose(np.linalg.inv(self._graph[self._active].keypose) @ matrix)
            self._voxel.integrate_frame(points, relative)
        submap = self._submaps[self._active]
        submap.observations.append(len(self._poses))
        self._graph[self._active].local_trajectory.append(relative)
        self._poses.append(matrix)
        self._relative_poses.append(relative)
        self._membership.append(self._active)
        self._travel = travel
        self._revision += 1
        self._proposal = None
        self._status.update(observations=len(self._poses), submaps=len(self._submaps))
        if (travel-submap.anchor_travel >= self.config.submap_distance or
                len(submap.observations) >= self.config.submap_max_observations):
            self._seal_active()
        return len(self._poses)-1

    def _seal_active(self):
        if self._active is None:
            return False
        # Both native allocations finish before changing the sealed frontier.
        points = self._voxel.point_cloud()
        pcd = self._voxel.open3d_pcd_with_normals()
        current = self._submaps[self._active]
        self._graph[self._active].pcd = pcd
        current.query_points = points
        current.sealed = True
        self._active, self._voxel = None, None
        return True

    def flush(self):
        """Seal the final partial submap. Repeated calls are idempotent.

        Called by the mapping worker, never a ROS service thread. Later accepted
        scans may start a new submap, so a save request does not stop mapping.
        """
        return self._seal_active()

    def _query(self, query):
        submap = self._submaps[query]
        geometry = self._graph[query].pcd
        if len(geometry.point.positions) < self.config.min_geometry_points:
            self._event('rejected', query=query, reason='too few normal-supported submap points')
            return
        # Same official detector and geometric validator as LoopCloser.compute,
        # separated here so candidate/verification outcomes are observable.
        if submap.candidate is None:
            try:
                submap.candidate = self._closer.detector.get_best_closure(query, submap.query_points)
            except Exception as error:
                # The native detector combines lookup and DB insertion. Once
                # it throws, insertion status is unknown; replaying this ID
                # could duplicate native state. Require a new session/replay.
                self._fatal_error = f'MapClosures state unavailable: {error}; restart and replay the journal'
                raise RuntimeError(self._fatal_error) from error
        closure = submap.candidate
        inliers, reference = int(closure.number_of_inliers), int(closure.source_id)
        self._event('candidate_search', query=query, source=reference, inliers=inliers)
        if inliers < self.config.inliers_threshold:
            return
        self._status['candidates'] += 1
        if (reference < 0 or reference >= query or
                query-reference < self.config.min_loop_separation or
                submap.anchor_travel-self._submaps[reference].anchor_travel < self.config.min_loop_travel):
            self._event('rejected', query=query, source=reference,
                        reason='candidate is not a separated historical revisit')
            return
        target_geometry = self._graph[reference].pcd
        if target_geometry is None or len(target_geometry.point.positions) < self.config.min_geometry_points:
            self._event('rejected', query=query, source=reference, reason='historical geometry unavailable')
            return
        try:
            guess = _pose(closure.pose)
        except ValueError:
            self._event('rejected', query=query, source=reference,
                        reason='place recognizer returned an invalid SE(3) proposal')
            return
        self._status['geometric_verifications'] += 1
        try:
            verified, transform = self._closer.validate_closure(target_geometry, geometry, guess.copy())
        except RuntimeError as error:
            self._event('rejected', query=query, source=reference,
                        geometry_accepted=False, reason=f'upstream ICP failed: {error}')
            return
        self._event('geometric_verification', query=query, source=reference,
                    geometry_accepted=bool(verified), inliers=inliers)
        if not verified:
            return
        try:
            measurement = _pose(transform)
        except ValueError:
            self._event('rejected', query=query, source=reference,
                        reason='geometric verification returned invalid SE(3)')
            return
        expected_reference = self._graph[query].keypose @ measurement
        difference = np.linalg.inv(self._graph[reference].keypose) @ expected_reference
        if (np.linalg.norm(difference[:3, 3]) > max(2.0, self._travel*self.config.max_correction_fraction) or
                _angle(difference) > math.radians(self.config.max_correction_rotation_degrees)):
            self._event('rejected', query=query, source=reference,
                        reason='geometric loop exceeds route correction budget')
            return
        if any(edge.source == reference and edge.target == query
               for edge in self._loops+self._tentative_loops):
            return
        self._tentative_loops.append(_Constraint(reference, query, measurement, inliers))
        self._event('constraint_proposed', query=query, source=reference, inliers=inliers)

    def detect_and_optimize(self):
        if self._fatal_error:
            raise RuntimeError(self._fatal_error)
        if self._proposal is not None:
            return self._proposal  # Retry a failed map rebuild, without another DB insertion.
        while self._next_query < len(self._submaps) and self._submaps[self._next_query].sealed:
            self._query(self._next_query)
            self._submaps[self._next_query].query_points = None
            self._next_query += 1
        if not self._tentative_loops:
            return None
        proposal = self._optimize(tuple(self._loops+self._tentative_loops))
        if proposal is None:
            # A failed global consistency gate cannot leave a factor in the
            # durable graph or later silently count as a successful loop.
            self._tentative_loops.clear()
            return None
        self._proposal = proposal
        return proposal

    def _optimize(self, loops):
        optimizer = self._Optimizer(self._pgo_config)
        originals = tuple(self._poses)
        for index in range(len(self._submaps)):
            optimizer.add_variable(index, self._graph[index].keypose.copy())
        optimizer.fix_variable(0)
        # Upstream C++ sets vertex0=target and vertex1=source. A measurement
        # is therefore inv(T_target) @ T_source, including ICP source->target.
        for edge in tuple(self._odometry_edges)+loops:
            optimizer.add_factor(edge.source, edge.target, edge.transform.copy(), np.eye(6))
        self._status['optimization_runs'] += 1
        optimizer.optimize()
        estimates = optimizer.estimates()
        if set(estimates) != set(range(len(self._submaps))):
            self._event('rejected', reason='optimizer returned an incomplete graph')
            return None
        try:
            keyposes = tuple(_pose(estimates[index]) for index in range(len(self._submaps)))
        except ValueError:
            self._event('rejected', reason='optimizer returned invalid SE(3) poses')
            return None
        # Preserve the original map gauge exactly, including nonzero Z/roll/pitch.
        anchor = self._graph[0].keypose
        gauge = anchor @ np.linalg.inv(keyposes[0])
        keyposes = tuple(_pose(gauge @ pose) for pose in keyposes)
        keyposes = (anchor.copy(),) + keyposes[1:]
        for edge in loops:
            residual = np.linalg.inv(edge.transform) @ (np.linalg.inv(keyposes[edge.target]) @ keyposes[edge.source])
            if (np.linalg.norm(residual[:3, 3]) > self.config.graph_loop_translation_limit+1e-8 or
                    _angle(residual) > math.radians(self.config.graph_loop_rotation_degrees)+1e-8):
                self._event('rejected', reason='optimized loop residual exceeds limits')
                return None
        for edge in self._odometry_edges:
            residual = np.linalg.inv(edge.transform) @ (np.linalg.inv(keyposes[edge.target]) @ keyposes[edge.source])
            allowance = .15 + .25*np.linalg.norm(edge.transform[:3, 3])
            if np.linalg.norm(residual[:3, 3]) > allowance or _angle(residual) > math.radians(8):
                self._event('rejected', reason='optimized odometry graph would tear or fold')
                return None
        poses = tuple(_pose(keyposes[node] @ relative)
                      for node, relative in zip(self._membership, self._relative_poses))
        poses = (originals[0],)+poses[1:]
        edges = tuple(dict(source=self._submaps[edge.source].anchor_observation,
                           target=self._submaps[edge.target].anchor_observation,
                           source_submap=edge.source, target_submap=edge.target,
                           inliers=edge.inliers, transform=edge.transform.tolist())
                      for edge in loops)
        self._event('optimized', reason='official KISS-SLAM geometric loop and g2o graph',
                    graph_nodes=len(keyposes), proposed_constraints=len(loops))
        return KissOptimization(poses, originals, edges, self.status, self._revision,
                                self._owner, loops, keyposes)

    def validate(self, result):
        if (not isinstance(result, KissOptimization) or result._owner is not self._owner or
                result._revision != self._revision or len(result.poses) != len(self._poses)):
            raise ValueError('foreign or stale KISS optimization')

    def apply(self, result):
        """Commit after the mapper prepared its complete new cloud/grid/TF state."""
        self.validate(result)
        # Allocate and validate before changing any durable graph references.
        poses = list(result.poses)
        keyposes = tuple(_pose(pose) for pose in result._keyposes)
        loops = list(result._constraints)
        status = dict(result.status)
        status['events'] = [dict(event) for event in status['events']]
        status.update(state='applied', constraints_added=len(loops),
                      committed_optimizations=self._status['committed_optimizations']+1)
        for index, pose in enumerate(keyposes):
            self._graph[index].keypose = pose
        self._poses, self._loops, self._status = poses, loops, status
        self._tentative_loops = []
        self._proposal = None
        self._revision += 1

    def close(self):
        """The ROS adapter owns process lifetime; no live ROS resources here."""
