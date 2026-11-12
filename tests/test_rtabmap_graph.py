"""Synthetic external-graph replay tests; these do not prove real loop closure."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import pose_matrix
from mapping_rebuild import rebuild
from rtabmap_graph import GraphMap


SECOND = 1_000_000_000


def pose(x=0., y=0., yaw=0.):
    return pose_matrix([x, y, .30], [0., 0., np.sin(yaw/2), np.cos(yaw/2)])


def change(x=0., y=0., yaw=0.):
    result = pose(x, y, yaw)
    result[2, 3] = 0.
    return result


def points():
    return np.random.default_rng(370).uniform([1., -.5, -.30], [2., .5, .8], (240, 3))


def graph_map():
    return GraphMap(.02, .15, .10)


def ordered(array):
    return array[np.lexsort((array[:, 2], array[:, 1], array[:, 0]))]


class GraphMapTests(unittest.TestCase):
    def test_three_anchor_correction_rebuilds_dense_cloud_and_free_rays(self):
        journal = graph_map()
        originals = [pose(x) for x in (0., 5., 10., 6.5, 3.)]
        stamps = [value*SECOND for value in (10, 15, 20, 25, 30)]
        for stamp, matrix in zip(stamps, originals):
            journal.append(stamp, points(), matrix)
        before = journal.snapshot()
        anchors = {stamps[0]: pose(0.), stamps[2]: pose(9.), stamps[4]: pose(0.)}
        self.assertTrue(journal.apply_graph(anchors))
        result = journal.snapshot()
        expected = [pose(x) for x in (0., 4.5, 9., 4.5, 0.)]
        reference, reference_grid = rebuild(result.observations, expected, .02, .15, .10)
        np.testing.assert_allclose(ordered(result.points), ordered(reference.snapshot()), atol=1e-12)
        self.assertEqual(result.free, frozenset(reference_grid.free))
        self.assertNotEqual(before.free, result.free)
        self.assertFalse(np.array_equal(before.points, result.points))
        for original, observation, corrected, wanted in zip(originals, result.observations, result.poses, expected):
            np.testing.assert_array_equal(observation.pose, original)
            np.testing.assert_allclose(corrected, wanted)
        np.testing.assert_allclose(result.correction, change(-3.))
        np.testing.assert_allclose(result.start_pose, pose())
        self.assertTrue(journal.ready_for_save())
        self.assertEqual(journal.report()['map_rebuilds'], 1)

    def test_sparse_non_keyframe_anchors_interpolate_corrections_not_path(self):
        journal = graph_map()
        journal.record_pose(10*SECOND, pose(0.))
        middle = pose(8., 3., .3)
        journal.append(20*SECOND, points(), middle)
        journal.record_pose(30*SECOND, pose(2.))
        journal.apply_graph({10*SECOND: pose(0.),
                             30*SECOND: change(-2., 1., np.pi/2) @ pose(2.)})
        expected = change(-1., .5, np.pi/4) @ middle
        np.testing.assert_allclose(journal.snapshot().poses[0], expected, atol=1e-12)
        # Interpolating graph poses directly would erase the original excursion.
        self.assertGreater(journal.snapshot().poses[0][1, 3], 5.)
        self.assertTrue(journal.ready_for_save())

    def test_identity_extension_uses_incremental_updates_without_rebuild(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        with patch('rtabmap_graph.rebuild', side_effect=AssertionError('unneeded rebuild')):
            self.assertFalse(journal.apply_graph({10*SECOND: pose()}))
            journal.append(20*SECOND, points(), pose(1.))
            self.assertFalse(journal.apply_graph({10*SECOND: pose(), 20*SECOND: pose(1.)}))
            self.assertFalse(journal.apply_graph({10*SECOND: pose(), 20*SECOND: pose(1.)}))
        self.assertEqual(journal.report()['map_rebuilds'], 0)
        self.assertEqual(journal.report()['map_revision'], 2)

    def test_float32_pose_roundoff_at_large_coordinates_does_not_rebuild(self):
        journal = graph_map()
        first = pose(300.123456, 1.234567)
        second = pose(301.987654, 2.345678)
        journal.append(10*SECOND, points(), first)
        journal.append(20*SECOND, points(), second)
        with patch('rtabmap_graph.rebuild', side_effect=AssertionError('float32 roundoff rebuilt the map')):
            self.assertFalse(journal.apply_graph({10*SECOND: first.astype(np.float32).astype(float),
                                                  20*SECOND: second.astype(np.float32).astype(float)}))
        np.testing.assert_array_equal(journal.correction, np.eye(4))
        np.testing.assert_array_equal(journal.snapshot().poses[0], first)

    def test_unacknowledged_tail_is_visible_but_cannot_be_saved(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        journal.apply_graph({10*SECOND: change(2.) @ pose()})
        journal.append(20*SECOND, points(), pose(1.))
        snapshot = journal.snapshot()
        np.testing.assert_allclose(snapshot.poses[-1], pose(3.))
        self.assertEqual(journal.report()['pending_after_last_anchor'], 1)
        self.assertFalse(journal.ready_for_save())
        journal.apply_graph({10*SECOND: pose(2.), 20*SECOND: pose(3.)})
        self.assertTrue(journal.ready_for_save())

    def test_initial_unanchored_frames_use_first_correction_and_are_reported(self):
        journal = graph_map()
        for stamp, x in ((10, 0.), (20, 1.), (30, 2.)):
            journal.append(stamp*SECOND, points(), pose(x))
        journal.apply_graph({20*SECOND: pose(5.), 30*SECOND: pose(6.)})
        np.testing.assert_allclose(journal.start_pose, pose(4.))
        self.assertEqual(journal.report()['initial_extrapolated_observations'], 1)
        self.assertTrue(journal.ready_for_save())

    def test_rebuild_failure_preserves_entire_committed_revision(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        journal.apply_graph({10*SECOND: pose()})
        before, report = journal.snapshot(), journal.report()
        with patch('rtabmap_graph.rebuild', side_effect=RuntimeError('allocation failed')):
            with self.assertRaisesRegex(RuntimeError, 'allocation'):
                journal.apply_graph({10*SECOND: pose(3.)})
        after = journal.snapshot()
        self.assertIs(before.points, after.points)
        self.assertEqual(report, journal.report())
        self.assertEqual(before.free, after.free)
        np.testing.assert_array_equal(before.correction, after.correction)

    def test_scan_arriving_during_graph_rebuild_invalidates_proposal(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        proposal = journal.prepare_graph({10*SECOND: pose(2.)})
        journal.append(20*SECOND, points(), pose(1.))
        with self.assertRaisesRegex(ValueError, 'stale'):
            journal.commit_graph(proposal)
        self.assertEqual(journal.report()['graph_revision'], 0)
        self.assertEqual(journal.report()['observations'], 2)
        journal.apply_graph({10*SECOND: pose(2.), 20*SECOND: pose(3.)})
        self.assertEqual(journal.report()['observations'], 2)
        np.testing.assert_allclose(journal.snapshot().poses[-1], pose(3.))

    def test_invalid_missing_partial_or_stale_graph_never_changes_map(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        journal.append(20*SECOND, points(), pose(2.))
        journal.apply_graph({10*SECOND: pose(), 20*SECOND: pose(1.)}, revision=3)
        invalid = pose()
        invalid[0, 0] = 2.
        attempts = [({}, None), ({30*SECOND: pose()}, None),
                    ({10*SECOND: pose()}, None),
                    ({10*SECOND: invalid, 20*SECOND: pose()}, None),
                    ({10*SECOND: pose(), 20*SECOND: pose()}, 2),
                    ({10*SECOND: pose(), 20*SECOND: pose()}, 3)]
        before = journal.snapshot()
        report = journal.report()
        for graph, revision in attempts:
            with self.subTest(graph=list(graph), revision=revision):
                with self.assertRaises(ValueError):
                    journal.apply_graph(graph, revision)
                self.assertEqual(journal.report(), report)
                self.assertIs(journal.snapshot().points, before.points)

    def test_foreign_or_twice_committed_proposal_is_rejected(self):
        first, second = graph_map(), graph_map()
        for journal in (first, second):
            journal.append(SECOND, points(), pose())
        proposal = first.prepare_graph({SECOND: pose(2.)})
        with self.assertRaisesRegex(ValueError, 'another map'):
            second.commit_graph(proposal)
        first.commit_graph(proposal)
        with self.assertRaisesRegex(ValueError, 'stale'):
            first.commit_graph(proposal)

    def test_conflicting_or_nonmonotonic_input_is_rejected(self):
        journal = graph_map()
        journal.append(10*SECOND, points(), pose())
        journal.record_pose(20*SECOND, pose(1.))
        journal.record_pose(20*SECOND, pose(1.))  # An identical duplicate is harmless.
        for action in (lambda: journal.record_pose(20*SECOND, pose(2.)),
                       lambda: journal.record_pose(15*SECOND, pose(2.)),
                       lambda: journal.append(10*SECOND, points(), pose()),
                       lambda: journal.append(20*SECOND, points(), pose(2.)),
                       lambda: journal.append(15*SECOND, points(), pose(2.)),
                       lambda: journal.append(30*SECOND, [[np.nan, 0, 0]], pose()),
                       lambda: journal.append(30*SECOND, np.empty((0, 3)), pose())):
            with self.assertRaises(ValueError):
                action()
        self.assertEqual(journal.report()['observations'], 1)
        journal.append(20*SECOND, points(), pose(1.))
        self.assertEqual(journal.report()['observations'], 2)

    def test_two_independent_map_origins_and_stable_snapshots(self):
        for name, origin in (('farm_alpha', change(10., -2., .4)),
                             ('farm_beta', change(-6., 3., -.2))):
            with self.subTest(map=name):
                journal = graph_map()
                original = origin @ pose()
                journal.append(SECOND, points(), original)
                old = journal.snapshot()
                new_start = change(1., 2., -.1) @ original
                journal.apply_graph({SECOND: new_start})
                np.testing.assert_allclose(journal.start_pose, new_start)
                np.testing.assert_array_equal(old.observations[0].pose, original)
                np.testing.assert_array_equal(old.poses[0], original)
                self.assertFalse(old.points.flags.writeable)
                self.assertFalse(journal.correction.flags.writeable)
                self.assertIsInstance(journal.snapshot().free, frozenset)
                expected, _ = rebuild(old.observations, [new_start], .02, .15, .1)
                np.testing.assert_allclose(ordered(journal.snapshot().points), ordered(expected.snapshot()))

    def test_no_graph_or_empty_journal_cannot_be_saved(self):
        journal = graph_map()
        self.assertFalse(journal.ready_for_save())
        journal.record_pose(SECOND, pose())
        journal.apply_graph({SECOND: pose()})
        self.assertFalse(journal.ready_for_save())
        journal.append(SECOND, points(), pose())
        self.assertTrue(journal.ready_for_save())
        for bad in (True, 0, -1, .5, '100'):
            with self.assertRaises(ValueError):
                journal.record_pose(bad, pose())


if __name__ == '__main__':
    unittest.main()
