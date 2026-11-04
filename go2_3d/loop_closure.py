"""ROS-independent, conservative scan-context / SE(3) pose-graph backend.

This module changes neither live GICP acceptance nor stored map resolution.
The mapper retains the original fine observations and rebuilds them using an
accepted Optimization. Only bounded, coarse keyframe clouds are retained here.
All methods belong to one worker thread. An optimization is a proposal until
``apply`` is called after the mapper's complete replacement map was prepared.

Place recognition is independent of the drifting map position. A descriptor
match alone never closes a loop: independent geometry, competing-place checks,
and a second spatially separated, consistent observation are required. Repeated
or featureless scenes can deliberately remain unclosed rather than distort a
map. This is not a guarantee of observability in a particular field.
"""
from dataclasses import dataclass
import math

import numpy as np
import open3d as o3d

from geometry import rotation_angle, transform_points
from registration import blend_correction, cloud


def _frozen(value):
    value = np.array(value, dtype=np.float64, copy=True)
    value.setflags(write=False)
    return value


def _pose(value):
    value = np.asarray(value, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all() or
            not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7) or
            not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-5)):
        raise ValueError('loop pose must be a finite proper SE(3) transform')
    return _frozen(value)


@dataclass(frozen=True)
class LoopConfig:
    keyframe_distance: float = 1.0
    keyframe_rotation_degrees: float = 15.0
    min_loop_travel: float = 20.0
    min_loop_separation: int = 10
    descriptor_radius: float = 20.0
    descriptor_rings: int = 20
    descriptor_sectors: int = 60
    candidate_count: int = 4
    descriptor_distance: float = 0.35
    voxel_size: float = 0.25
    max_cloud_points: int = 6000
    correspondence: float = 0.35
    min_overlap: float = 0.55
    max_rmse: float = 0.18
    confirmation_distance: float = 1.0
    confirmation_max_travel: float = 10.0
    consistency_translation: float = 0.75
    consistency_rotation_degrees: float = 10.0
    ambiguity_margin: float = 0.04
    ambiguity_separation: float = 6.0
    max_correction_fraction: float = 0.30
    max_correction_rotation_degrees: float = 60.0

    def __post_init__(self):
        integers = ('min_loop_separation', 'descriptor_rings', 'descriptor_sectors',
                    'candidate_count', 'max_cloud_points')
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            if name in integers and (int(value) != value or isinstance(value, bool)):
                raise ValueError(f'{name} must be a positive integer')
        if self.min_overlap > 1 or self.descriptor_distance > 1:
            raise ValueError('overlap and descriptor distance must be at most one')
        if self.confirmation_max_travel < self.confirmation_distance:
            raise ValueError('confirmation_max_travel must cover confirmation_distance')


@dataclass(frozen=True)
class Optimization:
    poses: tuple
    original_poses: tuple
    loop_edges: tuple
    status: dict
    _revision: int
    _owner: object
    _constraints: tuple


@dataclass(frozen=True)
class _Keyframe:
    observation: int
    points: np.ndarray
    level: np.ndarray
    descriptor: np.ndarray
    ring_key: np.ndarray
    travel: float


@dataclass(frozen=True)
class _Constraint:
    source: int
    target: int
    transform: np.ndarray
    fitness: float
    rmse: float


def _descriptor(points, config):
    """Height and occupied-bin context; ring key is invariant to yaw."""
    radius = np.linalg.norm(points[:, :2], axis=1)
    valid = (radius > .4) & (radius < config.descriptor_radius)
    points, radius = points[valid], radius[valid]
    shape = (config.descriptor_rings, config.descriptor_sectors)
    heights = np.zeros(shape)
    occupied = np.zeros(shape)
    if len(points):
        floor = np.percentile(points[:, 2], 5)
        # Ground alone supplies no useful place recognition evidence.
        structural = points[:, 2] > floor + .25
        points, radius = points[structural], radius[structural]
        rings = np.minimum((radius / config.descriptor_radius * shape[0]).astype(int), shape[0]-1)
        sectors = ((np.arctan2(points[:, 1], points[:, 0]) + np.pi) /
                   (2*np.pi) * shape[1]).astype(int) % shape[1]
        np.maximum.at(heights, (rings, sectors), np.clip(points[:, 2] - floor, 0, 5))
        occupied[rings, sectors] = 1
    descriptor = np.stack((heights / 5, occupied * .35), axis=0)
    # Hard polar bins are discontinuous under even a metre of translation:
    # the confirming scan would otherwise look unrelated when a tree crosses
    # a radial/sector boundary. Integrate neighbouring bins before retrieval,
    # while retaining geometry as the authority for actual place identity.
    descriptor = sum(np.roll(descriptor, shift, axis=2) for shift in (-1, 0, 1)) / 3
    padded = np.pad(descriptor, ((0, 0), (1, 1), (0, 0)))
    descriptor = sum(padded[:, offset:offset+shape[0], :] for offset in range(3)) / 3
    key = np.r_[heights.mean(axis=1) / 5, occupied.mean(axis=1)]
    norm = np.linalg.norm(key)
    return _frozen(descriptor), _frozen(key / norm if norm else key)


def _descriptor_match(query, target):
    """Return best yaw shift, plus an occupancy-aware cosine distance."""
    best = (float('inf'), 0)
    # Fixed sector count keeps this independent of route/map size.
    for shift in range(query.shape[2]):
        shifted = np.roll(query, shift, axis=2)
        first, second = shifted.reshape(-1), target.reshape(-1)
        norm = np.linalg.norm(first) * np.linalg.norm(second)
        distance = 1 - float(np.dot(first, second) / norm) if norm else 1.0
        best = min(best, (distance, shift))
    return best


def _level_rotation(pose):
    yaw = math.atan2(pose[1, 0], pose[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    yaw_inverse = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    level = np.eye(4)
    level[:3, :3] = yaw_inverse @ pose[:3, :3]
    return level


def _yaw(angle):
    result = np.eye(4)
    c, s = math.cos(angle), math.sin(angle)
    result[:2, :2] = [[c, -s], [s, c]]
    return result


def _bounded_points(points, voxel_size, limit):
    """Deterministic spatial coverage before filling the registration budget.

    Uniform indices into Open3D's hash-map iteration order are neither stable
    nor spatially stratified. Reserve a representative from every occupied
    coarse cell first; then fill remaining slots in a deterministic dispersed
    order. Fine observations used for map rebuilding never pass through this.
    """
    points = np.asarray(cloud(points, voxel_size).points)
    if len(points) <= limit:
        return points
    # Stable input ordering also resolves equal-distance representative ties.
    points = points[np.lexsort((points[:, 2], points[:, 1], points[:, 0]))]
    cell_size = voxel_size
    grid_origin = points.min(axis=0)
    while True:
        cell_size *= 2
        _, inverse = np.unique(np.floor((points-grid_origin)/cell_size).astype(np.int64), axis=0,
                               return_inverse=True)
        count = int(np.max(inverse)) + 1
        if count <= limit:
            break
    sums = np.zeros((count, 3))
    np.add.at(sums, inverse, points)
    centers = sums / np.bincount(inverse)[:, None]
    distance = np.sum((points-centers[inverse])**2, axis=1)
    order = np.lexsort((np.arange(len(points)), distance, inverse))
    starts = np.r_[True, inverse[order][1:] != inverse[order][:-1]]
    chosen = order[starts]
    remaining = np.ones(len(points), dtype=bool)
    remaining[chosen] = False
    extra = np.flatnonzero(remaining)
    # Multiplication modulo 2**32 disperses lexicographic neighbours without
    # random state, platform-dependent Python hashing, or coordinate overflow.
    ranks = (extra.astype(np.uint64)*2654435761) % 4294967296
    extra = extra[np.argsort(ranks, kind='stable')[:limit-len(chosen)]]
    return points[np.r_[chosen, extra]]


def _distances(reference, query):
    tree = o3d.geometry.KDTreeFlann(reference)
    return np.asarray([math.sqrt(tree.search_knn_vector_3d(point, 1)[2][0])
                       for point in np.asarray(query.points)])


def _structural(points):
    floor = np.percentile(points[:, 2], 5)
    return points[points[:, 2] > floor + .25]


def _observable(points):
    """Reject line/flat-ground matches using a scaled point-to-plane Hessian.

    Point-to-point ICP information alone falsely reports a large flat floor as
    constrained. Normal diversity and lever arms must constrain all six axes.
    """
    if len(points) < 100:
        return False
    centered = points - np.mean(points, axis=0)
    if np.linalg.eigvalsh(centered.T @ centered / len(points))[1] < .25:
        return False
    geometry = cloud(points, None)
    geometry.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.8, max_nn=30))
    normals = np.asarray(geometry.normals)
    scale = max(1.0, float(np.sqrt(np.mean(np.sum(centered**2, axis=1)))))
    jacobian = np.c_[np.cross(centered / scale, normals), normals]
    eigen = np.linalg.eigvalsh(jacobian.T @ jacobian / len(points))
    return bool(np.isfinite(eigen).all() and eigen[0] > 1e-4 and eigen[0] / eigen[-1] > 2e-4)


class LoopClosure:
    def __init__(self, config=None):
        self.config = config or LoopConfig()
        self._poses = []
        self._motion = []
        self._keyframes = []
        self._edges = []
        self._loops = []
        self._revision = 0
        self._owner = object()
        self._travel = 0.0
        self._last_checked = -1
        self._pending = None
        self._status = dict(state='waiting', observations=0, keyframes=0,
                            closures=0, candidates=0, reason='waiting for separated keyframes')

    @property
    def status(self):
        return dict(self._status)

    @property
    def observation_count(self):
        return len(self._poses)

    def append(self, points_body, pose_map_body):
        """Append every accepted observation; internally select sparse nodes.

        Does not retain or decimate the mapper's fine scan. Invalid inputs fail
        before updating graph state. Stationary scans still get a pose index.
        """
        pose = _pose(pose_map_body)
        points = np.asarray(points_body, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('loop observation must be finite N x 3 points')
        index = len(self._poses)
        travel = self._travel
        motion = self._motion[-1] if index else 0.0
        if index:
            distance = float(np.linalg.norm(pose[:3, 3] - self._poses[-1][:3, 3]))
            travel += distance
            motion += distance + rotation_angle(np.linalg.inv(self._poses[-1]) @ pose)
        selected = not self._keyframes
        if self._keyframes:
            previous = self._poses[self._keyframes[-1].observation]
            relative = np.linalg.inv(previous) @ pose
            selected = (np.linalg.norm(relative[:3, 3]) >= self.config.keyframe_distance or
                        rotation_angle(relative) >= math.radians(self.config.keyframe_rotation_degrees))
        new_keyframe = None
        if selected:
            level = _level_rotation(pose)
            points = transform_points(points, level)
            distance = np.linalg.norm(points, axis=1)
            points = _bounded_points(points[distance < self.config.descriptor_radius],
                                     self.config.voxel_size, self.config.max_cloud_points)
            descriptor, key = _descriptor(points, self.config)
            new_keyframe = _Keyframe(index, _frozen(points), _frozen(level), descriptor, key, travel)
        self._poses.append(pose)
        self._motion.append(motion)
        self._travel = travel
        self._revision += 1
        if new_keyframe is not None:
            if self._keyframes:
                previous = self._keyframes[-1].observation
                self._edges.append(_frozen(np.linalg.inv(pose) @ self._poses[previous]))
            self._keyframes.append(new_keyframe)
        self._status.update(observations=len(self._poses), keyframes=len(self._keyframes))
        return index

    def _verify(self, source, target, shift):
        config = self.config
        first, second = cloud(source.points, None), cloud(target.points, None)
        if len(first.points) < 100 or len(second.points) < 100:
            return None
        # Local coordinate registration deliberately does not use the drifting
        # world translation. Scan context supplies yaw, multi-scale GICP fits
        # the small physical offset between revisits, independently of drift.
        guesses = []
        angle = shift * 2*np.pi / config.descriptor_sectors
        for offset in (0, -1, 1):
            guess = _yaw(angle + offset * 2*np.pi / config.descriptor_sectors)
            guesses.append(guess)
        best = None
        for guess in guesses:
            transform = guess
            for voxel, distance, iterations in ((.6, 3.0, 35), (.35, 1.0, 30),
                                                 (config.voxel_size, config.correspondence, 30)):
                result = o3d.pipelines.registration.registration_generalized_icp(
                    first.voxel_down_sample(voxel), second.voxel_down_sample(voxel),
                    distance, transform,
                    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations))
                transform = result.transformation
            if not np.isfinite(transform).all():
                continue
            moved = cloud(transform_points(source.points, transform), None)
            forward, reverse = _distances(second, moved), _distances(moved, second)
            overlap = min(float(np.mean(forward < config.correspondence)),
                          float(np.mean(reverse < config.correspondence)))
            accepted_distances = np.r_[forward[forward < config.correspondence],
                                       reverse[reverse < config.correspondence]]
            rmse = float(np.sqrt(np.mean(accepted_distances**2))) if len(accepted_distances) else float('inf')
            if overlap < config.min_overlap or rmse > config.max_rmse:
                continue
            # A floor can dominate overlap even for unrelated field rows.
            # Require overlap specifically on non-ground structure as well.
            structural_first = cloud(_structural(source.points), None)
            structural_second = cloud(_structural(target.points), None)
            structural_first.transform(transform)
            if len(structural_first.points) < 100 or len(structural_second.points) < 100:
                continue
            structural_overlap = min(
                float(np.mean(_distances(structural_second, structural_first) < config.correspondence)),
                float(np.mean(_distances(structural_first, structural_second) < config.correspondence)))
            if structural_overlap < config.min_overlap or not _observable(
                    target.points[reverse < config.correspondence]):
                continue
            body_transform = np.linalg.inv(target.level) @ transform @ source.level
            rank = overlap + structural_overlap - rmse
            if best is None or rank > best[0]:
                best = (rank, _frozen(body_transform), min(overlap, structural_overlap), rmse)
        return best

    def detect_and_optimize(self):
        """Propose a correction after two consistent, unambiguous revisits.

        This CPU work must run on the mapper's worker, never in sensor or TF
        callbacks. Descriptor proposals exclude neighbours by path distance,
        not by drifting world position, so a long loop may close despite drift.
        """
        config = self.config
        current = len(self._keyframes) - 1
        if current < 0 or current <= self._last_checked:
            return None
        self._last_checked = current
        query = self._keyframes[current]
        if (self._pending is not None and
                query.travel-self._pending[2] > config.confirmation_max_travel):
            self._pending = None
        candidates = []
        for index, target in enumerate(self._keyframes[:max(0, current-config.min_loop_separation+1)]):
            if query.travel - target.travel < config.min_loop_travel:
                continue
            if np.linalg.norm(target.ring_key) == 0 or np.linalg.norm(query.ring_key) == 0:
                continue
            distance = 1 - float(np.dot(query.ring_key, target.ring_key))
            candidates.append((distance, index))
        candidates.sort()
        verified = []
        proposals = []
        # Ring-key retrieval is cheap; full yaw search is bounded to a few
        # alternatives and geometry to candidate_count. Keep geographically
        # distinct alternatives so repeated orchard rows are actually tested.
        for _, index in candidates[:max(12, config.candidate_count*3)]:
            target = self._keyframes[index]
            distance, shift = _descriptor_match(query.descriptor, target.descriptor)
            if distance <= config.descriptor_distance:
                proposals.append((distance, index, shift))
        proposals.sort()
        chosen = []
        # Test distinct historical areas first; adjacent nodes in the best
        # area must not consume every geometry slot and hide an orchard alias.
        for proposal in proposals:
            location = self._poses[self._keyframes[proposal[1]].observation][:3, 3]
            if all(np.linalg.norm(location-self._poses[self._keyframes[p[1]].observation][:3, 3]) >=
                   config.ambiguity_separation for p in chosen):
                chosen.append(proposal)
            if len(chosen) == config.candidate_count:
                break
        for proposal in proposals:
            if len(chosen) == config.candidate_count:
                break
            if proposal not in chosen:
                chosen.append(proposal)
        for descriptor_distance, index, shift in chosen:
            match = self._verify(query, self._keyframes[index], shift)
            if match is not None:
                score, transform, overlap, rmse = match
                verified.append((score, index, transform, overlap, rmse, descriptor_distance))
        self._status.update(candidates=len(chosen), state='searching', reason='no verified loop')
        if not verified:
            return None
        verified.sort(key=lambda item: item[0], reverse=True)
        best = verified[0]
        for alternate in verified[1:]:
            separation = np.linalg.norm(self._poses[self._keyframes[best[1]].observation][:3, 3] -
                                        self._poses[self._keyframes[alternate[1]].observation][:3, 3])
            if separation >= config.ambiguity_separation and best[0] - alternate[0] < config.ambiguity_margin:
                self._pending = None
                self._status.update(state='rejected', reason='ambiguous repeated places')
                return None
        _, target, transform, overlap, rmse, _ = best
        edge = _Constraint(current, target, transform, overlap, rmse)
        current_pose = self._poses[query.observation]
        correction = self._poses[self._keyframes[target].observation] @ transform @ np.linalg.inv(current_pose)
        self._status.update(source_observation=int(query.observation),
                            target_observation=int(self._keyframes[target].observation),
                            fitness=float(overlap), rmse=float(rmse))
        if (np.linalg.norm((correction @ current_pose)[:3, 3]-current_pose[:3, 3]) >
                max(2.0, self._travel * config.max_correction_fraction) or
                rotation_angle(correction) > math.radians(config.max_correction_rotation_degrees)):
            self._status.update(state='rejected', reason='loop innovation exceeds route budget')
            return None
        previous = self._pending
        self._pending = (edge, correction, query.travel)
        if previous is None:
            self._status.update(state='confirming', reason='waiting for a second separated revisit')
            return None
        old_edge, old_correction, old_travel = previous
        if query.travel - old_travel < config.confirmation_distance:
            self._pending = previous
            return None
        # Compare at the robot rather than the world origin: a small rotation
        # about a distant loop location must not appear as a translation jump.
        difference = np.linalg.inv(old_correction @ current_pose) @ (correction @ current_pose)
        historical_separation = np.linalg.norm(
            self._poses[self._keyframes[old_edge.target].observation][:3, 3] -
            self._poses[self._keyframes[edge.target].observation][:3, 3])
        if (historical_separation > config.confirmation_max_travel or
                np.linalg.norm(difference[:3, 3]) > config.consistency_translation or
                rotation_angle(difference) > math.radians(config.consistency_rotation_degrees)):
            self._status.update(state='rejected', reason='revisit corrections are inconsistent')
            return None
        proposal = self._optimize(tuple(self._loops) + (old_edge, edge))
        if proposal is None:
            self._status.update(state='rejected', reason='pose graph failed residual checks')
        return proposal

    def _optimize(self, constraints):
        reg = o3d.pipelines.registration
        graph = reg.PoseGraph()
        original = tuple(self._poses)
        for frame in self._keyframes:
            graph.nodes.append(reg.PoseGraphNode(original[frame.observation].copy()))
        for index, measurement in enumerate(self._edges):
            graph.edges.append(reg.PoseGraphEdge(index, index+1, measurement.copy(),
                                                 np.eye(6)*100, uncertain=False))
        for edge in constraints:
            graph.edges.append(reg.PoseGraphEdge(edge.source, edge.target, edge.transform.copy(),
                                                 np.eye(6)*1000, uncertain=True))
        reg.global_optimization(graph, reg.GlobalOptimizationLevenbergMarquardt(),
                                reg.GlobalOptimizationConvergenceCriteria(),
                                reg.GlobalOptimizationOption(max_correspondence_distance=self.config.correspondence,
                                                             edge_prune_threshold=.1,
                                                             preference_loop_closure=5.0,
                                                             reference_node=0))
        optimized = [_pose(node.pose) for node in graph.nodes]
        # Some Open3D versions preserve the first pose only up to numeric
        # precision; explicitly restore the initial body/map frame gauge.
        gauge = original[0] @ np.linalg.inv(optimized[0])
        optimized = [_pose(gauge @ pose) for pose in optimized]
        for constraint in constraints:
            error = np.linalg.inv(constraint.transform) @ (
                np.linalg.inv(optimized[constraint.target]) @ optimized[constraint.source])
            if np.linalg.norm(error[:3, 3]) > .45 or rotation_angle(error) > math.radians(5):
                return None
        for index, measurement in enumerate(self._edges):
            predicted = np.linalg.inv(optimized[index+1]) @ optimized[index]
            error = np.linalg.inv(measurement) @ predicted
            # Reject graph foldovers/tearing instead of using a convincing
            # local match to arbitrarily rewrite a distant route.
            allowance = .15 + .25*np.linalg.norm(measurement[:3, 3])
            if np.linalg.norm(error[:3, 3]) > allowance or rotation_angle(error) > math.radians(8):
                return None
        corrections = [new @ np.linalg.inv(original[frame.observation])
                       for frame, new in zip(self._keyframes, optimized)]
        poses, segment = [], 0
        for index, pose in enumerate(original):
            while segment+1 < len(self._keyframes) and self._keyframes[segment+1].observation <= index:
                segment += 1
            correction = corrections[segment]
            if segment+1 < len(self._keyframes):
                start, end = self._keyframes[segment].observation, self._keyframes[segment+1].observation
                # Sensor arrival rate and standing still must not determine
                # how historical drift is distributed. Interpolate by motion
                # (metres + radians), so repeated static scans stay together.
                denominator = self._motion[end] - self._motion[start]
                alpha = ((self._motion[index]-self._motion[start])/denominator
                         if denominator > 1e-9 else 0.0)
                if alpha > 0:
                    correction = blend_correction(correction, corrections[segment+1], alpha)
            poses.append(_pose(correction @ pose))
        poses[0] = original[0]
        metadata = tuple(dict(source=self._keyframes[e.source].observation,
                              target=self._keyframes[e.target].observation,
                              fitness=float(e.fitness), rmse=float(e.rmse),
                              transform=e.transform.tolist()) for e in constraints)
        status = dict(self._status, state='optimized', reason='confirmed geometric loop and pose graph',
                      closures=len(constraints), graph_nodes=len(self._keyframes))
        return Optimization(tuple(poses), original, metadata, status,
                            self._revision, self._owner, constraints)

    def validate(self, result):
        """Validate a proposal immediately before the mapper transaction."""
        if (not isinstance(result, Optimization) or result._owner is not self._owner or
                result._revision != self._revision or len(result.poses) != len(self._poses)):
            raise ValueError('foreign or stale loop optimization')

    def apply(self, result):
        """Commit only the current proposal after the replacement map succeeds.

        All allocations occur before updating any internal references. Call on
        the same worker as append/detect, inside the mapper commit lock.
        """
        self.validate(result)
        poses, loops, status = list(result.poses), list(result._constraints), dict(result.status)
        self._poses = poses
        self._loops = loops
        self._pending = None
        self._revision += 1
        self._status = status
