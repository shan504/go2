"""Actual Open3D Generalized ICP, with acceptance and innovation limits."""
from dataclasses import dataclass
import numpy as np
import open3d as o3d
from geometry import rotation_angle, quaternion_from_matrix, pose_matrix


def cloud(points, voxel=0.15):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    return result if voxel is None else result.voxel_down_sample(voxel)


@dataclass
class Match:
    accepted: bool
    transform: np.ndarray
    fitness: float
    rmse: float
    reason: str


def align(source, target, guess, correspondence=0.7, min_fitness=0.55,
          max_rmse=0.20, max_translation=0.6, max_rotation=0.35,
          iterations=35, innovation_guess=None):
    if len(source.points) < 100 or len(target.points) < 100:
        return Match(False, guess.copy(), 0.0, float('inf'), 'too few points')
    result = o3d.pipelines.registration.registration_generalized_icp(
        source, target, correspondence, guess,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations))
    reference = guess if innovation_guess is None else innovation_guess
    delta = np.linalg.inv(reference) @ result.transformation
    finite = np.isfinite(result.transformation).all()
    innovation_ok = (finite and np.linalg.norm(delta[:3,3]) <= max_translation and
                     rotation_angle(delta) <= max_rotation)
    quality_ok = result.fitness >= min_fitness and result.inlier_rmse <= max_rmse
    accepted = innovation_ok and quality_ok
    # Distinguish a missing/poor scan from a dangerous pose innovation. Only
    # the former may hold the previous match within its original time and
    # odometry budgets. No rejected result updates that reference/deadline.
    reason = ('nonfinite transform' if not finite else 'innovation rejected' if not innovation_ok
              else 'overlap/residual rejected' if not quality_ok else 'accepted')
    return Match(accepted, result.transformation, result.fitness, result.inlier_rmse,
                 reason)


def odometry_is_stationary(previous_odom, odom, translation_limit=0.02, rotation_limit=0.02):
    """Check body motion between timestamped odometry poses."""
    if previous_odom is None:
        return False
    movement = np.linalg.inv(previous_odom)@odom
    return (np.linalg.norm(movement[:3,3]) < translation_limit and
            rotation_angle(movement) < rotation_limit)


def static_correction_is_consistent(previous_correction, correction, previous_odom, odom,
                                    translation_limit=0.05, rotation_limit=0.035):
    """Reject false scan matches which move the map while odometry is stationary."""
    if previous_odom is None:
        return True  # Bootstrap and an explicit initialpose may establish a new pose.
    stationary = odometry_is_stationary(previous_odom, odom)
    # Compare the robot pose, not the odometry origin. A small rotation around
    # a distant robot can change the correction's translation by metres.
    change = np.linalg.inv(previous_correction@odom)@(correction@odom)
    return not stationary or (np.linalg.norm(change[:3,3]) <= translation_limit and
                              rotation_angle(change) <= rotation_limit)


def blend_correction(previous, measured, alpha=0.25, odom=None):
    """Filter only an already accepted map->odom correction; odometry is not filtered."""
    if not 0.0 < alpha <= 1.0:
        raise ValueError('correction_alpha must be greater than 0 and at most 1')
    if odom is not None:
        previous,measured = previous@odom,measured@odom
    first, second = quaternion_from_matrix(previous), quaternion_from_matrix(measured)
    dot = float(np.dot(first,second))
    if dot < 0:
        second, dot = -second, -dot
    dot = float(np.clip(dot,-1,1))
    if dot > 0.9995:
        quaternion = (1-alpha)*first+alpha*second
    else:
        angle = np.arccos(dot)
        quaternion = (np.sin((1-alpha)*angle)*first+np.sin(alpha*angle)*second)/np.sin(angle)
    filtered = pose_matrix((1-alpha)*previous[:3,3]+alpha*measured[:3,3],quaternion)
    return filtered if odom is None else filtered@np.linalg.inv(odom)
