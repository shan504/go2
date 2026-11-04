"""Loop closure on deterministic synthetic geometry, never robot acceptance."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import pose_matrix, transform_points, rotation_angle
from loop_closure import LoopClosure, LoopConfig


def pose(x=0., y=0., yaw=0., z=0.):
    return pose_matrix([x, y, z], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])


def farm_geometry():
    """Repeated crop rows plus varied trunks and an asymmetric entrance building."""
    rng = np.random.default_rng(702)
    parts = [np.column_stack((rng.uniform(-16, 66, (19000, 2)),
                              np.full(19000, -.3)))]
    for x in np.arange(-10., 66., 6.):
        for y in np.arange(-10., 66., 6.):
            center = np.array([x, y]) + rng.uniform(-.6, .6, 2)
            theta = rng.uniform(-math.pi, math.pi, 150)
            radius = rng.uniform(.15, .3)
            height = rng.uniform(1.4, 2.6)
            parts.append(np.column_stack((center[0] + radius * np.cos(theta),
                                          center[1] + radius * np.sin(theta),
                                          rng.uniform(-.3, height, 150))))
    # Nonparallel walls and differently sized columns distinguish the entrance
    # from translated copies of the orchard's repeating rows.
    parts.extend([
        np.column_stack((rng.uniform(-9, 3, 1700), np.full(1700, -8.),
                         rng.uniform(-.3, 3.3, 1700))),
        np.column_stack((np.full(1200, -9.), rng.uniform(-8, 5, 1200),
                         rng.uniform(-.3, 2.8, 1200))),
        rng.uniform([4, -5, -.3], [5.2, -3, 1.7], (900, 3)),
        rng.uniform([-4, 4, -.3], [-3.4, 4.8, 2.5], (600, 3)),
    ])
    return np.vstack(parts)


def local_scan(world, world_body, radius=15.):
    local = transform_points(world, np.linalg.inv(world_body))
    distance = np.linalg.norm(local[:, :2], axis=1)
    return local[(distance > .4) & (distance < radius)]


def square_route(step=5.):
    """Exactly 200 metres; final pose revisits the entrance with original yaw."""
    result = []
    for x in np.arange(0, 50, step):
        result.append(pose(x, 0, 0))
    for y in np.arange(0, 50, step):
        result.append(pose(50, y, math.pi / 2))
    for x in np.arange(50, 0, -step):
        result.append(pose(x, 50, math.pi))
    for y in np.arange(50, 0, -step):
        result.append(pose(0, y, -math.pi / 2))
    result.append(pose())
    return result


class LoopClosureTests(unittest.TestCase):
    def test_200m_loop_can_be_recognized_when_returning_in_opposite_heading(self):
        world = farm_geometry()
        backend = LoopClosure()
        route = square_route()[:-1] + [pose(0, 0, math.pi), pose(-1.5, 0, math.pi)]
        result = None
        for index, truth in enumerate(route):
            progress = min(index / 40., 1.)
            drift = pose(5. * progress, -2. * progress, math.radians(6) * progress)
            backend.append(local_scan(world, truth), drift @ truth)
            if index >= 37:
                result = backend.detect_and_optimize()
                if result is not None:
                    break
        self.assertIsNotNone(result, f'Opposite-heading loop was not recognized: {backend.status}')
        error = np.linalg.inv(route[index]) @ result.poses[-1]
        self.assertLess(np.linalg.norm(error[:3, 3]), .6)
        self.assertLess(rotation_angle(error), math.radians(5))

    def test_second_lap_without_drift_does_not_apply_the_first_correction_twice(self):
        world = farm_geometry()
        backend = LoopClosure()
        route = square_route() + [pose(1.5, 0)]
        result = None
        for index, truth in enumerate(route):
            progress = min(index / 40., 1.)
            backend.append(local_scan(world, truth),
                           pose(5. * progress, -2. * progress, math.radians(6) * progress) @ truth)
            if index >= 37:
                result = backend.detect_and_optimize()
                if result is not None:
                    break
        self.assertIsNotNone(result)
        backend.apply(result)
        first_count = backend.observation_count
        second = None
        for index, truth in enumerate(route):
            backend.append(local_scan(world, truth), truth)
            if index >= 37:
                second = backend.detect_and_optimize()
                if second is not None:
                    break
        self.assertIsNotNone(second, f'Second lap failed: {backend.status}')
        self.assertGreater(len(second.loop_edges), len(result.loop_edges))
        np.testing.assert_allclose(second.poses[0], np.eye(4), atol=1e-8)
        for original, corrected in zip(second.original_poses[first_count:], second.poses[first_count:]):
            change = np.linalg.inv(original) @ corrected
            self.assertLess(np.linalg.norm(change[:3, 3]), .35,
                            'A no-drift second lap was displaced by a previous lap correction')
            self.assertLess(rotation_angle(change), math.radians(3))
        backend.apply(second)

    def test_200m_loop_recovers_start_and_distributes_drift(self):
        world = farm_geometry()
        # Keep a fine observation between graph nodes: rebuilding must retain
        # and correct it too, rather than dropping it during graph sparsifying.
        route = [pose(), pose(.2, 0)] + square_route()[1:] + [pose(1.5, 0)]
        backend = LoopClosure()
        originals = []
        result = None
        for index, truth in enumerate(route):
            progress = min(index / 40., 1.)
            drift = pose(5. * progress, -2. * progress,
                         math.radians(6.) * progress, .25 * progress)
            measured = drift @ truth
            originals.append(measured.copy())
            scan = local_scan(world, truth)
            original_scan = scan.copy()
            backend.append(scan, measured)
            # The last 15m approach supplies independent temporal confirmation.
            if index >= 37:
                candidate = backend.detect_and_optimize()
                if candidate is not None:
                    result = candidate
                    break
            np.testing.assert_array_equal(scan, original_scan)
        self.assertIsNotNone(result, 'Recognizable entrance did not close the synthetic 200m loop')
        np.testing.assert_allclose(result.poses[0], originals[0], atol=1e-8)
        self.assertEqual(len(result.poses), len(originals))
        self.assertLess(result.status['graph_nodes'], len(result.poses))
        self.assertGreater(len(result.loop_edges), 0)
        for saved, original in zip(result.original_poses, originals):
            np.testing.assert_array_equal(saved, original)
        before_error = np.linalg.norm(originals[-1][:3, 3] - route[len(originals)-1][:3, 3])
        after_error = np.linalg.norm(result.poses[-1][:3, 3] - route[len(originals)-1][:3, 3])
        self.assertGreater(before_error, 4.)
        self.assertLess(after_error, .6)
        middle = len(result.poses) // 2
        self.assertGreater(np.linalg.norm(result.poses[middle][:3, 3] -
                                          originals[middle][:3, 3]), .15,
                           'Only snapping the current pose leaves historical map drift intact')
        for corrected in result.poses:
            self.assertTrue(np.isfinite(corrected).all())
            np.testing.assert_allclose(corrected[:3, :3].T @ corrected[:3, :3],
                                       np.eye(3), atol=1e-7)
            self.assertAlmostEqual(np.linalg.det(corrected[:3, :3]), 1., places=7)
        backend.apply(result)
        self.assertEqual(backend.status['state'], 'optimized')
        with self.assertRaises(ValueError):
            backend.apply(result)
        with self.assertRaises(ValueError):
            LoopClosure().apply(result)

    def test_empty_backend_and_invalid_config_fail_without_corrupting_state(self):
        backend = LoopClosure()
        self.assertIsNone(backend.detect_and_optimize())
        self.assertEqual(backend.observation_count, 0)
        for settings in ({'candidate_count': 0}, {'voxel_size': float('nan')},
                         {'min_overlap': 1.01}, {'descriptor_sectors': 2.5}):
            with self.assertRaises(ValueError):
                LoopConfig(**settings)

    def test_confirmation_from_a_previous_visit_expires_after_leaving_the_place(self):
        world = farm_geometry()
        backend = LoopClosure()
        for index, truth in enumerate(square_route()):
            progress = min(index / 40., 1.)
            backend.append(local_scan(world, truth),
                           pose(5 * progress, -2 * progress, math.radians(6) * progress) @ truth)
        self.assertIsNone(backend.detect_and_optimize())
        self.assertEqual(backend.status['state'], 'confirming')
        drift = pose(5, -2, math.radians(6))
        # No polling is needed during this excursion: path travel itself must
        # expire the old evidence before a new entrance visit is evaluated.
        away = pose(12, 12)
        backend.append(local_scan(world, away), drift @ away)
        backend.append(local_scan(world, pose()), drift)
        self.assertIsNone(backend.detect_and_optimize(),
                          'An old entrance match was combined with a different visit')
        self.assertEqual(backend.status['state'], 'confirming')
        confirmation = pose(1.5, 0)
        backend.append(local_scan(world, confirmation), drift @ confirmation)
        self.assertIsNotNone(backend.detect_and_optimize(),
                             'Two fresh consistent entrance observations should still close')

    def test_repeating_places_without_unique_geometry_do_not_close(self):
        # A local scene copied along identical tree rows can match perfectly
        # at several historical places. Residual alone must not accept it.
        rng = np.random.default_rng(97)
        repeated = rng.uniform([-7., -7., -.3], [7., 7., 2.], (1800, 3))
        backend = LoopClosure()
        for index in range(35):
            backend.append(repeated, pose(index * 3., 0))
        self.assertIsNone(backend.detect_and_optimize())
        backend.append(repeated, pose(105., 0))
        self.assertIsNone(backend.detect_and_optimize())
        self.assertEqual(backend.status['state'], 'rejected')
        self.assertEqual(backend.status['reason'], 'ambiguous repeated places')
        self.assertGreaterEqual(backend.status['candidates'], 2)

    def test_sparse_and_planar_returns_cannot_create_a_loop(self):
        rng = np.random.default_rng(103)
        plane = np.column_stack((rng.uniform(-10, 10, (1800, 2)),
                                 np.full(1800, -.3)))
        for points in (plane, rng.uniform(-1, 1, (30, 3))):
            backend = LoopClosure()
            for index, truth in enumerate(square_route() + [pose(1.5, 0)]):
                backend.append(points, truth)
            self.assertIsNone(backend.detect_and_optimize())
            backend.append(points, pose(3., 0))
            self.assertIsNone(backend.detect_and_optimize())

    def test_invalid_pose_or_nonfinite_cloud_is_rejected(self):
        backend = LoopClosure()
        rng = np.random.default_rng(3)
        points = rng.uniform(-2, 2, (300, 3))
        bad_pose = pose()
        bad_pose[0, 0] = float('nan')
        with self.assertRaises(ValueError):
            backend.append(points, bad_pose)
        bad_cloud = points.copy()
        bad_cloud[0, 1] = float('inf')
        with self.assertRaises(ValueError):
            backend.append(bad_cloud, pose())
        self.assertEqual(backend.observation_count, 0)

    def test_stationary_observations_preserve_density_without_graph_growth(self):
        backend = LoopClosure()
        rng = np.random.default_rng(31)
        points = rng.uniform([-4, -4, -.3], [4, 4, 2], (1200, 3))
        for index in range(50):
            self.assertEqual(backend.append(points, pose()), index)
        self.assertEqual(backend.observation_count, 50)
        self.assertEqual(backend.status['keyframes'], 1)
        self.assertIsNone(backend.detect_and_optimize())


if __name__ == '__main__':
    unittest.main()
