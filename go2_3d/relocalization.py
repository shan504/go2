"""Conservative local recapture against any selected map, without ROS.

This is a bounded recovery of the last trusted map/odometry relationship, not
global place recognition. The caller runs ``observe`` on one background worker,
keeps motion paused, and checks scan freshness/generation again on completion.
Only several fresh, geometrically unique, motion-consistent observations can
produce an accepted correction. No method changes map files or normal tracking
acceptance limits. Repeated or poorly constrained rows deliberately stay lost.
"""
from dataclasses import dataclass
import math

import numpy as np
import open3d as o3d

from geometry import rotation_angle, transform_points
from registration import cloud


def _readonly(value):
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def _pose(value):
    value = np.asarray(value, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all() or
            not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7) or
            not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-5)):
        raise ValueError('recovery pose must be finite proper SE(3)')
    return _readonly(value)


@dataclass(frozen=True)
class RecoveryConfig:
    max_translation: float = 2.0
    max_rotation_degrees: float = 20.0
    max_vertical: float = .35
    search_step: float = .5
    yaw_step_degrees: float = 10.0
    candidate_count: int = 4
    voxel_size: float = .20
    source_point_limit: int = 4000
    target_point_limit: int = 20000
    scan_radius: float = 20.0
    correspondence: float = .35
    min_overlap: float = .55
    min_structural_overlap: float = .50
    max_rmse: float = .18
    confirmations: int = 3
    confirmation_interval: float = .15
    confirmation_timeout: float = 2.5
    consistency_translation: float = .18
    consistency_rotation_degrees: float = 3.0
    max_scan_age: float = 1.0
    ambiguity_distance: float = .60
    ambiguity_rotation_degrees: float = 8.0
    ambiguity_margin: float = .05

    def __post_init__(self):
        integers = ('candidate_count', 'source_point_limit', 'target_point_limit', 'confirmations')
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            if name in integers and (int(value) != value or isinstance(value, bool)):
                raise ValueError(f'{name} must be a positive integer')
        if self.confirmations < 2:
            raise ValueError('recovery needs at least two independent scans')
        if min(self.source_point_limit, self.target_point_limit) < 100:
            raise ValueError('recovery point budgets must retain at least 100 points')
        if max(self.min_overlap, self.min_structural_overlap, self.ambiguity_margin) > 1:
            raise ValueError('recovery overlap/margin must be at most one')
        if self.confirmation_timeout < self.confirmation_interval:
            raise ValueError('confirmation timeout must cover the scan interval')
        # Keep a mistaken settings value from silently creating an unbounded
        # hypothesis search. These are supported local-search budgets.
        count = 2*math.ceil(self.max_translation/self.search_step)+1
        yaws = 2*math.ceil(self.max_rotation_degrees/self.yaw_step_degrees)+1
        if count*count*yaws > 625 or not 2 <= self.candidate_count <= 8:
            raise ValueError('recovery hypothesis budget exceeds supported local search')


@dataclass(frozen=True)
class RecoveryResult:
    accepted: bool
    pose: object
    correction: object
    fitness: float
    rmse: float
    confirmations: int
    reason: str


def _bounded(points, voxel_size, limit):
    result = cloud(points, voxel_size)
    # Preserve spatial coverage by increasing backend-only voxel size. This
    # never changes stored maps or the scan used for normal localization.
    while len(result.points) > limit:
        voxel_size *= 1.25
        result = cloud(points, voxel_size)
    return result


def _nearest(reference_tree, points):
    return np.asarray([math.sqrt(reference_tree.search_knn_vector_3d(point, 1)[2][0])
                       for point in points])


def _hypothesis_scores(target_points, score_points, hypotheses, native=True):
    """Score every unchanged search seed with one native CPU NN batch.

    Float64 preserves the scalar KD-tree scoring even far from the map origin.
    The normal budget is 245 x 300 queries (about 1.8 MiB of XYZ coordinates),
    not a target/query distance matrix. The scalar path exists only for older
    Open3D builds lacking the public core nearest-neighbour API, and for direct
    numerical equivalence tests. Native execution failures are not swallowed.
    """
    matrices = np.asarray(hypotheses, dtype=np.float64)
    transformed = (np.einsum('mij,nj->mni', matrices[:, :3, :3], score_points) +
                   matrices[:, None, :3, 3])
    if (native and hasattr(o3d, 'core') and hasattr(o3d.core, 'nns') and
            hasattr(o3d.core.nns, 'NearestNeighborSearch')):
        reference = o3d.core.Tensor(np.ascontiguousarray(target_points), o3d.core.Dtype.Float64)
        search = o3d.core.nns.NearestNeighborSearch(reference)
        if not search.knn_index():
            raise RuntimeError('Open3D CPU recovery nearest-neighbour index failed')
        queries = o3d.core.Tensor(np.ascontiguousarray(transformed.reshape(-1, 3)),
                                  o3d.core.Dtype.Float64)
        _, squared_distances = search.knn_search(queries, 1)
        distances = np.sqrt(np.maximum(squared_distances.numpy(), 0)).reshape(transformed.shape[:2])
    else:
        # Open3D 0.14's legacy index references the cloud's storage. Keep the
        # owner alive for every query; an inline temporary can leave a dangling
        # index and nondeterministic distances on that supported ROS version.
        reference_cloud = cloud(target_points, None)
        tree = o3d.geometry.KDTreeFlann(reference_cloud)
        distances = np.vstack([_nearest(tree, query) for query in transformed])
    return np.mean(np.minimum(distances, 1.5), axis=1)


def _observable(points):
    if len(points) < 100:
        return False
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered / len(points)
    if np.linalg.eigvalsh(covariance)[1] < .25:
        return False
    geometry = cloud(points, None)
    geometry.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.7, max_nn=30))
    normals = np.asarray(geometry.normals)
    scale = max(1., float(np.sqrt(np.mean(np.sum(centered**2, axis=1)))))
    jacobian = np.c_[np.cross(centered/scale, normals), normals]
    eigen = np.linalg.eigvalsh(jacobian.T @ jacobian / len(points))
    return bool(np.isfinite(eigen).all() and eigen[0] > 1e-4 and eigen[0]/eigen[-1] > 2e-4)


class Relocalizer:
    """Read-only selected-map index plus a worker-owned confirmation session."""
    def __init__(self, map_points, config=None):
        self.config = config or RecoveryConfig()
        points = np.asarray(map_points, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or
                not np.isfinite(points).all()):
            raise ValueError('recovery requires a finite map with at least 100 points')
        self._map = cloud(points, self.config.voxel_size)
        self._map_tree = o3d.geometry.KDTreeFlann(self._map)
        self._map_points = np.asarray(self._map.points)
        self._map_points.setflags(write=False)
        self.reset()

    def reset(self):
        """Start a new session after trusted tracking or an explicit initialpose."""
        self._trusted = None
        self._previous = None
        self._last_stamp = None
        self._confirmations = 0

    def _rejected(self, reason, fitness=0., rmse=float('inf'), clear=True):
        if clear:
            self._previous = None
            self._confirmations = 0
        return RecoveryResult(False, None, None, float(fitness), float(rmse),
                              self._confirmations, reason)

    def _hypotheses(self, predicted):
        config = self.config
        steps = math.ceil(config.max_translation/config.search_step)
        yaws = math.ceil(config.max_rotation_degrees/config.yaw_step_degrees)
        hypotheses = []
        for ix in range(-steps, steps+1):
            for iy in range(-steps, steps+1):
                offset = np.array([ix*config.search_step, iy*config.search_step, 0.])
                if np.linalg.norm(offset) > config.max_translation + 1e-8:
                    continue
                for yaw_index in range(-yaws, yaws+1):
                    angle = yaw_index * config.yaw_step_degrees
                    if abs(angle) > config.max_rotation_degrees + 1e-8:
                        continue
                    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
                    guess = predicted.copy()
                    guess[:3, 3] += offset
                    guess[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ predicted[:3, :3]
                    hypotheses.append(guess)
        return hypotheses

    def _within_budget(self, pose, predicted):
        difference = np.linalg.inv(predicted) @ pose
        return (np.linalg.norm(difference[:3, 3]) <= self.config.max_translation + 1e-7 and
                abs(float(pose[2, 3]-predicted[2, 3])) <= self.config.max_vertical + 1e-7 and
                rotation_angle(difference) <= math.radians(self.config.max_rotation_degrees) + 1e-7)

    def _distinct(self, first, second):
        relative = np.linalg.inv(first) @ second
        return (np.linalg.norm(relative[:3, 3]) >= self.config.ambiguity_distance or
                rotation_angle(relative) >= math.radians(self.config.ambiguity_rotation_degrees))

    def _search(self, points, predicted):
        config = self.config
        indices = self._map_tree.search_radius_vector_3d(
            predicted[:3, 3], config.scan_radius + config.max_translation + 1.)[1]
        if len(indices) < 100:
            return [], 'no map geometry near trusted prediction'
        source = _bounded(points, config.voxel_size, config.source_point_limit)
        target = _bounded(self._map_points[np.asarray(indices)], config.voxel_size,
                          config.target_point_limit)
        if min(len(source.points), len(target.points)) < 100:
            return [], 'too few recovery points'
        source_points, target_points = np.asarray(source.points), np.asarray(target.points)
        score_points = source_points[np.linspace(0, len(source_points)-1,
                                                min(300, len(source_points)), dtype=int)]
        hypotheses = self._hypotheses(predicted)
        values = _hypothesis_scores(target_points, score_points, hypotheses)
        scores = [(float(score), guess) for score, guess in zip(values, hypotheses)]
        scores.sort(key=lambda item: item[0])
        # Collapse yaw seeds per spatial cell, then find separate coarse
        # basins. Seeds along the same smooth corridor gradient must not use
        # every ICP slot while a repeated row has its own alternative basin.
        spatial = {}
        for score, guess in scores:
            key = tuple(np.round(guess[:3, 3]-predicted[:3, 3], 8))
            spatial.setdefault(key, (score, guess))
        basins = []
        for position, (score, guess) in spatial.items():
            neighbours = [other_score for other_position, (other_score, _) in spatial.items()
                          if 0 < np.linalg.norm(np.asarray(position)-other_position) <= config.search_step*1.01]
            if not neighbours or score <= min(neighbours)+.01:
                basins.append((score, guess))
        basins.sort(key=lambda item: item[0])
        guesses = []
        # Test different local positions before spending the whole budget on
        # yaw seeds at one position. Otherwise several nearly identical ICP
        # convergences could hide a second, equally plausible tree row.
        for _, guess in basins:
            if all(np.linalg.norm(guess[:3, 3]-selected[:3, 3]) >= config.ambiguity_distance
                   for selected in guesses):
                guesses.append(guess)
            if len(guesses) == config.candidate_count:
                break
        for _, guess in scores:
            if len(guesses) == config.candidate_count:
                break
            if all(self._distinct(guess, selected) for selected in guesses):
                guesses.append(guess)
        # Reuse the coarse clouds. Let GICP construct its regularized surface
        # covariances: raw point-cloud covariances are singular on flat floors
        # and must not be passed to GICP as already conditioned estimates.
        pyramids = []
        for voxel, distance, iterations in ((.5, 1.2, 25),
                                             (config.voxel_size, config.correspondence, 30)):
            first, second = source.voxel_down_sample(voxel), target.voxel_down_sample(voxel)
            pyramids.append((first, second, distance, iterations))
        verified = []
        for guess in guesses:
            transform = guess
            for first, second, distance, iterations in pyramids:
                match = o3d.pipelines.registration.registration_generalized_icp(
                    first, second, distance, transform,
                    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations))
                transform = match.transformation
            if not np.isfinite(transform).all() or not self._within_budget(transform, predicted):
                continue
            verified_match = self._verify(source_points, target_points, transform)
            if verified_match is not None:
                fitness, rmse = verified_match
                verified.append((fitness - rmse, _readonly(transform), fitness, rmse))
        verified.sort(key=lambda item: item[0], reverse=True)
        return verified, 'no unique geometrically verified local recovery'

    def _verify(self, source, target, transform):
        config = self.config
        moved = transform_points(source, transform)
        # Reverse overlap uses only map geometry in the actual scan footprint;
        # unrelated parts of a large selected map cannot dilute this metric.
        lower, upper = moved.min(axis=0)-config.correspondence, moved.max(axis=0)+config.correspondence
        radius = min(config.scan_radius, float(np.percentile(np.linalg.norm(source, axis=1), 98)))
        target = target[(np.linalg.norm(target-transform[:3, 3], axis=1) <= radius+config.correspondence) &
                        np.all((target >= lower) & (target <= upper), axis=1)]
        if len(target) < 100:
            return None
        target_cloud, source_cloud = cloud(target, None), cloud(moved, None)
        forward = np.asarray(source_cloud.compute_point_cloud_distance(target_cloud))
        reverse = np.asarray(target_cloud.compute_point_cloud_distance(source_cloud))
        fitness = min(float(np.mean(forward < config.correspondence)),
                      float(np.mean(reverse < config.correspondence)))
        inliers = np.r_[forward[forward < config.correspondence], reverse[reverse < config.correspondence]]
        rmse = float(np.sqrt(np.mean(inliers**2))) if len(inliers) else float('inf')
        if fitness < config.min_overlap or rmse > config.max_rmse:
            return None
        # Ground-only agreement cannot recover along visually repetitive rows.
        floor = min(float(np.percentile(moved[:, 2], 5)), float(np.percentile(target[:, 2], 5)))
        first = moved[moved[:, 2] > floor+.25]
        second = target[target[:, 2] > floor+.25]
        if min(len(first), len(second)) < 100:
            return None
        first_cloud, second_cloud = cloud(first, None), cloud(second, None)
        structural = min(
            float(np.mean(np.asarray(first_cloud.compute_point_cloud_distance(second_cloud)) < config.correspondence)),
            float(np.mean(np.asarray(second_cloud.compute_point_cloud_distance(first_cloud)) < config.correspondence)))
        if structural < config.min_structural_overlap or not _observable(target[reverse < config.correspondence]):
            return None
        return min(fitness, structural), rmse

    def observe(self, points_body, odom_pose, trusted_correction, stamp, now):
        """Verify one distinct fresh scan on the caller's serial recovery worker.

        ``stamp`` and ``now`` use the same clock in seconds. Predicted position
        always derives from the unchanged last trusted correction and actual
        scan-time odometry. A rejected result never renews trusted localization.
        """
        config = self.config
        odom, trusted = _pose(odom_pose), _pose(trusted_correction)
        points = np.asarray(points_body, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('recovery scan must be finite N x 3 points')
        if not np.isfinite(stamp) or not np.isfinite(now):
            raise ValueError('recovery timestamps must be finite')
        if not -.1 <= now-stamp <= config.max_scan_age:
            return self._rejected('scan is stale or from the future')
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return self._rejected('duplicate or out-of-order scan', clear=False)
        if self._trusted is not None and not np.allclose(trusted, self._trusted, atol=1e-7):
            self.reset()
        self._trusted = trusted
        self._last_stamp = float(stamp)
        if self._previous is not None and stamp-self._previous[2] > config.confirmation_timeout:
            self._previous = None
            self._confirmations = 0
        if self._previous is not None and stamp-self._previous[2] < config.confirmation_interval:
            return self._rejected('waiting for an independent scan timestamp', clear=False)
        points = points[np.linalg.norm(points, axis=1) <= config.scan_radius]
        if len(points) < 100:
            return self._rejected('too few recovery points')
        predicted = trusted @ odom
        matches, reason = self._search(points, predicted)
        if not matches:
            return self._rejected(reason)
        best = matches[0]
        for alternate in matches[1:]:
            if self._distinct(best[1], alternate[1]) and best[0]-alternate[0] < config.ambiguity_margin:
                return self._rejected('ambiguous repeated local geometry', best[2], best[3])
        _, pose, fitness, rmse = best
        correction = pose @ np.linalg.inv(odom)
        if self._previous is not None:
            previous_correction, _, _ = self._previous
            # Evaluating both corrections at this body origin accounts for
            # real intervening odometry without amplifying a far map origin.
            innovation = np.linalg.inv(previous_correction @ odom) @ pose
            if (np.linalg.norm(innovation[:3, 3]) > config.consistency_translation or
                    rotation_angle(innovation) > math.radians(config.consistency_rotation_degrees)):
                self._previous = (correction, odom, float(stamp))
                self._confirmations = 1
                return self._rejected('local recovery changed between scans', fitness, rmse, clear=False)
        self._confirmations += 1
        self._previous = (correction, odom, float(stamp))
        if self._confirmations < config.confirmations:
            return self._rejected('confirming independent local recovery scans', fitness, rmse, clear=False)
        return RecoveryResult(True, _pose(pose), _pose(correction), float(fitness), float(rmse),
                              self._confirmations, 'confirmed unique local recovery')
