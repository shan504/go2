"""Actual Open3D Generalized ICP, with acceptance and innovation limits."""
from dataclasses import dataclass
import numpy as np
import open3d as o3d
from geometry import rotation_angle


def cloud(points, voxel=0.15):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    return result.voxel_down_sample(voxel)


@dataclass
class Match:
    accepted: bool
    transform: np.ndarray
    fitness: float
    rmse: float
    reason: str


def align(source, target, guess, correspondence=0.7, min_fitness=0.55,
          max_rmse=0.20, max_translation=0.6, max_rotation=0.35):
    if len(source.points) < 100 or len(target.points) < 100:
        return Match(False, guess.copy(), 0.0, float('inf'), 'too few points')
    result = o3d.pipelines.registration.registration_generalized_icp(
        source, target, correspondence, guess,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35))
    delta = np.linalg.inv(guess) @ result.transformation
    accepted = (np.isfinite(result.transformation).all() and
                result.fitness >= min_fitness and result.inlier_rmse <= max_rmse and
                np.linalg.norm(delta[:3,3]) <= max_translation and
                rotation_angle(delta) <= max_rotation)
    return Match(accepted, result.transformation, result.fitness, result.inlier_rmse,
                 'accepted' if accepted else 'overlap/residual/innovation rejected')
