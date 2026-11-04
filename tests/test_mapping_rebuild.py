import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import ObservedGrid, pose_matrix, transform_points
from mapping_rebuild import MappingObservation, corrected_poses, rebuild, append_rebuilt
from mapping_store import MappingStore
from registration import cloud


def pose(x=0., y=0., z=0., yaw=0.):
    return pose_matrix([x, y, z], [0, 0, np.sin(yaw/2), np.cos(yaw/2)])


def scan():
    # Keep a dense feature at the fine retention scale; a coarse cloud cannot
    # substitute for these returns without visibly losing PCD geometry.
    rng = np.random.default_rng(612)
    return np.column_stack((rng.uniform(2, 3, 6000),
                            rng.uniform(-1, 1, 6000),
                            rng.uniform(-.35, 1.2, 6000)))


class MappingRebuildTests(unittest.TestCase):
    def test_observation_copies_inputs_and_rejects_invalid_geometry(self):
        points, matrix = scan(), pose()
        observation = MappingObservation(points, matrix)
        points[:] = 42
        matrix[:3, 3] = 42
        self.assertLess(observation.points_body.max(), 4)
        np.testing.assert_array_equal(observation.pose, np.eye(4))
        self.assertFalse(observation.points_body.flags.writeable)
        self.assertFalse(observation.pose.flags.writeable)
        moved = observation.with_pose(pose(2, 3, .35, .2))
        self.assertIs(moved.points_body, observation.points_body)
        np.testing.assert_array_equal(observation.pose, np.eye(4))
        np.testing.assert_allclose(moved.pose, pose(2, 3, .35, .2))
        self.assertFalse(moved.pose.flags.writeable)
        invalid = np.eye(4); invalid[0, 0] = 2
        for bad in (invalid, np.full((4, 4), np.nan), np.eye(3)):
            with self.assertRaises(ValueError):
                MappingObservation(scan(), bad)
        for bad in (np.full((2, 3), np.nan), np.zeros((3, 4))):
            with self.assertRaises(ValueError):
                MappingObservation(bad, pose())

    def test_nonrigid_prefix_and_tail_preserve_all_relative_motion(self):
        original = [pose(0, 0, .35, .1), pose(5, 2, .4, .3), pose(7, 3, .4, .5)]
        observations = [MappingObservation(scan(), p) for p in original]
        new = [original[0], pose(4, 1, .36, .2)]
        corrected = corrected_poses(observations, original[:2], new)
        np.testing.assert_allclose(corrected[0], original[0])
        np.testing.assert_allclose(corrected[1], new[1])
        np.testing.assert_allclose(np.linalg.inv(corrected[1]) @ corrected[2],
                                   np.linalg.inv(original[1]) @ original[2], atol=1e-12)
        self.assertFalse(corrected[-1].flags.writeable)
        np.testing.assert_array_equal(observations[-1].pose, original[-1])
        with self.assertRaisesRegex(ValueError, 'does not match'):
            corrected_poses(observations, [pose(100)], [pose()])
        for before, after in (([], []), (original, new), (original*2, original*2)):
            with self.assertRaises(ValueError):
                corrected_poses(observations, before, after)

    def test_fine_geometry_free_origins_and_pcd_replay_for_two_map_frames(self):
        points = scan()
        # Raw body-origin and ground-aligned starts must both be preserved;
        # no global floor/start coordinate can be assumed by reconstruction.
        for start in (pose(), pose(4, -2, .37, .4)):
            with self.subTest(start=start.tolist()):
                old = [start, start @ pose(5, .7, .03, .1)]
                new = [start, start @ pose(4, .1, 0., .02)]
                observations = [MappingObservation(points, p) for p in old]
                store, grid = rebuild(observations, new, .02, .15, .10)
                expected_store = MappingStore(.02, .15)
                expected_grid = ObservedGrid(.10)
                stale_grid = ObservedGrid(.10)
                for matrix, old_matrix in zip(new, old):
                    mapped = transform_points(points, matrix)
                    expected_store.commit(expected_store.prepare(mapped))
                    expected_grid.observe(np.asarray(cloud(mapped, .02).points), matrix[:3, 3])
                    stale = transform_points(points, old_matrix)
                    stale_grid.observe(np.asarray(cloud(stale, .02).points), old_matrix[:3, 3])
                np.testing.assert_allclose(store.snapshot(), expected_store.snapshot(), atol=0)
                self.assertEqual(grid.free, expected_grid.free)
                self.assertNotEqual(grid.free, stale_grid.free)
                self.assertGreater(store.count, len(cloud(store.snapshot(), .15).points)*2)
                with tempfile.TemporaryDirectory() as tmp:
                    path = Path(tmp)/'map.pcd'
                    self.assertTrue(o3d.io.write_point_cloud(str(path), cloud(store.snapshot(), None), compressed=True))
                    actual = np.asarray(o3d.io.read_point_cloud(str(path)).points)
                    np.testing.assert_allclose(actual, store.snapshot(), atol=1e-6)
                np.testing.assert_array_equal(observations[0].pose, start)
                # Repeated replay must not apply a correction for a second time.
                repeated, repeated_grid = rebuild(observations, new, .02, .15, .10)
                np.testing.assert_array_equal(repeated.snapshot(), store.snapshot())
                self.assertEqual(repeated_grid.free, grid.free)
                # Catching up an observation arriving during optimization
                # must be identical to a single complete journal replay.
                prefix, prefix_grid = rebuild(observations[:1], new[:1], .02, .15, .10)
                append_rebuilt(prefix, prefix_grid, observations[1], new[1])
                np.testing.assert_array_equal(prefix.snapshot(), store.snapshot())
                self.assertEqual(prefix_grid.free, grid.free)

    def test_rebuild_failure_does_not_modify_original_map_or_journal(self):
        observations = [MappingObservation(scan(), pose()), MappingObservation(scan(), pose(5))]
        original, grid = rebuild(observations, [o.pose for o in observations], .02, .15, .1)
        before, free = original.snapshot().copy(), grid.free.copy()
        prepare = MappingStore.prepare
        calls = 0
        def fail_second(store, points):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise RuntimeError('controlled rebuild failure')
            return prepare(store, points)
        with patch.object(MappingStore, 'prepare', fail_second):
            with self.assertRaisesRegex(RuntimeError, 'controlled rebuild failure'):
                rebuild(observations, [pose(), pose(4)], .02, .15, .1)
        np.testing.assert_array_equal(original.snapshot(), before)
        self.assertEqual(grid.free, free)
        np.testing.assert_array_equal(observations[-1].pose, pose(5))


if __name__ == '__main__':
    unittest.main()
