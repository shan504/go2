"""Real pinned KISS-SLAM tests plus explicitly controlled error/transaction cases.

Run with the isolated .venv-kiss. The unmodified native detector/ICP/g2o fixture
is synthetic and does not validate L1 range, farm accuracy or Jetson throughput.
"""
from dataclasses import replace
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'go2_3d'))
sys.path.insert(0, str(ROOT/'tests'))
from kiss_backend import KissBackend, KissConfig, _Constraint
from kiss_backend_fixture import rich_scene, submap_revisit


def pose(x=0, y=0, yaw=0):
    matrix = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    matrix[:2, :2] = [[c, -s], [s, c]]
    matrix[:2, 3] = [x, y]
    return matrix


class KissConfigurationTests(unittest.TestCase):
    def test_old_backend_import_does_not_load_native_experimental_packages(self):
        script = ('import sys; sys.path.insert(0, '+repr(str(ROOT/'go2_3d'))+'); '
                  'from kiss_backend import KissConfig; KissConfig(); '
                  'assert not any(x.startswith(("kiss_slam", "map_closures", "open3d")) '
                  'for x in sys.modules)')
        subprocess.run([sys.executable, '-c', script], check=True)

    def test_invalid_backend_settings_fail_before_importing_native_libraries(self):
        for change in ({'submap_distance': 0}, {'max_iterations': 1.1},
                       {'inliers_threshold': True}, {'overlap_threshold': 1},
                       {'density_map_resolution': float('nan')}, {'voxel_size': .25}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                KissConfig(**change)


@unittest.skipUnless(importlib.util.find_spec('kiss_slam'), 'requires isolated pinned KISS environment')
class KissNativeTests(unittest.TestCase):
    def backend(self, **settings):
        backend = KissBackend(KissConfig(**settings))
        self.addCleanup(backend.close)
        return backend

    def test_actual_detector_icp_g2o_and_two_phase_apply(self):
        backend = self.backend(submap_max_observations=1, max_iterations=40)
        originals = []
        result = None
        for cloud, matrix in submap_revisit():
            originals.append(matrix.copy())
            backend.append(cloud, matrix)
            result = backend.detect_and_optimize()
        self.assertIsNotNone(result, backend.status)
        self.assertGreaterEqual(backend.status['candidates'], 1)
        self.assertGreaterEqual(backend.status['geometric_verifications'], 1)
        self.assertEqual(backend.status['optimization_runs'], 1)
        self.assertEqual(backend.status['constraints_added'], 0)
        self.assertTrue(any(event.get('geometry_accepted') is True
                            for event in backend.status['events']))
        self.assertGreaterEqual(result.loop_edges[-1]['inliers'], 5)
        self.assertLess(np.linalg.norm(result.poses[-1][:3, 3]), .5)
        np.testing.assert_array_equal(result.poses[0], originals[0])
        for actual, expected in zip(backend._poses, originals):
            np.testing.assert_array_equal(actual, expected)  # No premature graph commit.
        with patch.object(backend._closer.detector, 'get_best_closure',
                          side_effect=AssertionError('rebuild retry inserted detector ID again')):
            self.assertIs(backend.detect_and_optimize(), result)
        backend.apply(result)
        self.assertEqual(backend.status['constraints_added'], len(result.loop_edges))
        self.assertEqual(backend.status['committed_optimizations'], 1)
        for actual, expected in zip(backend._poses, result.poses):
            np.testing.assert_array_equal(actual, expected)
        with self.assertRaises(ValueError):
            backend.apply(result)  # A proposal cannot be committed twice.

    def test_actual_geometry_rejects_a_wrong_place(self):
        backend = self.backend(submap_max_observations=1)
        backend.append(rich_scene(17), pose())
        first = backend._graph[0].pcd
        wrong = first.clone()
        wrong.translate([150., 0., 0.])
        accepted, _ = backend._closer.validate_closure(first, wrong, np.eye(4))
        self.assertFalse(accepted)
        self.assertEqual(backend.status['constraints_added'], 0)

    def test_flush_is_idempotent_and_pose_validation_is_atomic(self):
        backend = self.backend()
        cloud = rich_scene(17)
        with self.assertRaises(ValueError):
            backend.append(cloud, np.zeros((4, 4)))
        with self.assertRaises(ValueError):
            backend.append([[np.nan, 1, 2]], pose())
        self.assertEqual(backend.observation_count, 0)
        anchor = pose(8., -2., .4)
        backend.append(cloud, anchor)
        self.assertTrue(backend.flush())
        self.assertFalse(backend.flush())
        self.assertIsNone(backend.detect_and_optimize())
        backend.append(cloud, anchor @ pose(1.))
        self.assertTrue(backend.flush())
        self.assertEqual(len(backend._submaps), 2)
        self.assertEqual(backend.observation_count, 2)
        np.testing.assert_array_equal(backend._poses[0], anchor)

    def test_real_optimizer_factor_direction_and_nonidentity_map_anchor(self):
        # Controlled loop constraint isolates the g2o frame convention. Detection
        # is tested unmocked above; this is deliberately not a detection test.
        backend = self.backend(submap_max_observations=1, max_iterations=40)
        anchor = pose(8., -2., .4)
        for x in (0, 10, 20, 30, 40, 50, 3):
            backend.append(np.array([[0., 0., 0.], [.1, 0., 0.], [0., .1, 0.]]),
                           anchor @ pose(x))
        result = backend._optimize((_Constraint(0, 6, np.eye(4), 50),))
        self.assertIsNotNone(result, backend.status)
        np.testing.assert_array_equal(result.poses[0], anchor)
        endpoint = np.linalg.inv(anchor) @ result.poses[-1]
        np.testing.assert_allclose(endpoint[:3, 3], [3/7, 0, 0], atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(result.poses[-1][:3, :3]), 1.)
        with self.assertRaises(ValueError):
            backend.apply(replace(result, _owner=object()))
        backend.append(np.array([[0., 0., 0.]]), anchor @ pose(4.))
        with self.assertRaises(ValueError):
            backend.apply(result)

    def test_detector_exception_requires_replay_without_duplicate_insertion(self):
        backend = self.backend(submap_max_observations=1)
        backend.append(rich_scene(17), pose())
        with patch.object(backend._closer.detector, 'get_best_closure',
                          side_effect=RuntimeError('controlled detector failure')) as detector:
            for _ in range(2):
                with self.assertRaisesRegex(RuntimeError, 'restart and replay'):
                    backend.detect_and_optimize()
        self.assertEqual(detector.call_count, 1)
        self.assertEqual(backend.status['constraints_added'], 0)

    def test_active_submap_remains_in_its_local_frame_after_graph_commit(self):
        backend = self.backend(submap_max_observations=2, submap_distance=1000., max_iterations=40)
        points = np.array([[0., 0., 0.], [.1, 0., 0.], [0., .1, 0.]])
        for index, x in enumerate((0, 10, 20, 30, 40, 50, 3)):
            for _ in range(1 if index == 6 else 2):
                backend.append(points, pose(x))
        self.assertEqual(backend._active, 6)
        result = backend._optimize((_Constraint(0, 6, np.eye(4), 50),))
        self.assertIsNotNone(result, backend.status)
        correction = result.poses[-1] @ np.linalg.inv(result.original_poses[-1])
        backend.apply(result)
        backend.append(points, correction @ pose(4.))
        np.testing.assert_allclose(backend._relative_poses[-1], pose(1.), atol=1e-6)
        np.testing.assert_allclose(backend._poses[-1], correction @ pose(4.), atol=1e-6)
        self.assertEqual(backend._membership[-1], 6)
        self.assertEqual(len(backend._submaps), 7)

    def test_icp_exception_rejects_candidate_without_reinsertion(self):
        backend = self.backend(submap_max_observations=1)
        cloud = rich_scene(17)
        for x in (0, 10, 20, 30, 40):
            backend.append(cloud, pose(x))
        def candidate(query, points):
            return SimpleNamespace(number_of_inliers=50 if query == 4 else 0,
                                   source_id=0, pose=np.eye(4))
        with patch.object(backend._closer.detector, 'get_best_closure', side_effect=candidate) as detector:
            with patch.object(backend._closer, 'validate_closure',
                              side_effect=RuntimeError('controlled ICP failure')):
                self.assertIsNone(backend.detect_and_optimize())
                self.assertIsNone(backend.detect_and_optimize())
        self.assertEqual(detector.call_count, 5)
        self.assertEqual(backend.status['constraints_added'], 0)
        self.assertIn('controlled ICP failure', backend.status['reason'])


if __name__ == '__main__':
    unittest.main()
