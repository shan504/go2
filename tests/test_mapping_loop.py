"""Orchestration tests with real map reconstruction and a controlled graph.

Place recognition and SE(3) optimization are exercised by test_loop_closure;
this file deliberately controls proposals to isolate concurrent journal replay
and commit ordering failures without probabilistic registration timing.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from geometry import pose_matrix
from mapping_loop import MappingLoopWorker
from mapping_rebuild import MappingObservation, rebuild


def pose(x=0, y=0):
    return pose_matrix([x, y, 0], [0, 0, 0, 1])


def observation(x):
    rng = np.random.default_rng(790)
    return MappingObservation(rng.uniform([1., -1., -.3], [3., 1., 1.], (450, 3)), pose(x))


def wait_for(predicate, timeout=4.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('worker condition timed out')


class ControlledBackend:
    def __init__(self, fail_apply=False, propose=True):
        self.poses = []
        self.fail_apply = fail_apply
        self.propose = propose
        self.applied = False
        self.status = {'state': 'waiting'}

    @property
    def observation_count(self):
        return len(self.poses)

    def append(self, points, matrix):
        self.poses.append(matrix.copy())

    def detect_and_optimize(self):
        if self.applied or not self.propose or len(self.poses) < 2:
            return None
        new = [matrix.copy() for matrix in self.poses]
        for index in range(1, len(new)):
            new[index][0, 3] -= 1.
        return SimpleNamespace(original_poses=tuple(self.poses), poses=tuple(new),
                               status={'state': 'optimized', 'closures': 1}, loop_edges=())

    def apply(self, result):
        # Match the real backend contract: validation and allocation failures
        # occur before its internal reference swaps.
        if self.fail_apply:
            raise RuntimeError('controlled backend apply failure')
        poses = [matrix.copy() for matrix in result.poses]
        self.poses, self.applied = poses, True


class MappingLoopTests(unittest.TestCase):
    def test_save_flushes_final_submap_without_foreground_backend_calls(self):
        lock = threading.RLock()
        backend = ControlledBackend(propose=False)
        calls, commits = [], []
        def flush():
            calls.append(threading.get_ident())
            backend.propose = True
        backend.flush = flush
        worker = MappingLoopWorker(lock, None, .02, .15, .10,
                                   lambda *args: lambda: commits.append(args), backend)
        try:
            with lock:
                worker.append(observation(0))
                worker.append(observation(5))
            wait_for(worker.ready_for_save)
            self.assertFalse(commits)
            with lock:
                worker.request_flush()
                self.assertFalse(worker.ready_for_save())
            wait_for(worker.ready_for_save)
            self.assertEqual(len(commits), 1)
            self.assertEqual(calls, [worker.thread.ident])
            with lock:
                worker.request_flush()
                self.assertTrue(worker.ready_for_save())
            self.assertEqual(len(calls), 1)
        finally:
            worker.close()

    def test_failed_final_flush_blocks_save_and_can_retry_without_new_scans(self):
        lock = threading.RLock()
        backend = ControlledBackend(propose=False)
        calls = []
        def flush():
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError('temporary native/submap failure')
        backend.flush = flush
        worker = MappingLoopWorker(lock, None, .02, .15, .10, lambda *a: None, backend)
        try:
            with lock:
                worker.append(observation(0))
                worker.request_flush()
            wait_for(lambda: worker.report()['state'] == 'error')
            with lock:
                self.assertFalse(worker.ready_for_save())
                worker.request_flush()
            wait_for(worker.ready_for_save)
            self.assertEqual(len(calls), 2)
            self.assertEqual(worker.report()['flush_completed'], 1)
        finally:
            worker.close()

    def test_tail_arriving_during_rebuild_is_corrected_and_never_dropped(self):
        lock = threading.RLock()
        backend = ControlledBackend()
        entered, release = threading.Event(), threading.Event()
        committed = []
        def prepare(store, grid, poses, delta):
            candidate = (store, grid, tuple(poses), delta.copy())
            return lambda: committed.append(candidate)
        worker = MappingLoopWorker(lock, None, .02, .15, .10, prepare, backend)
        prefix = [observation(0), observation(5)]
        tail = observation(7)
        def delayed_rebuild(*args):
            entered.set()
            if not release.wait(timeout=4):
                raise RuntimeError('test release timed out')
            return rebuild(*args)
        try:
            with patch('mapping_loop.rebuild', delayed_rebuild):
                with lock:
                    for item in prefix:
                        worker.append(item)
                    self.assertFalse(worker.ready_for_save())
                self.assertTrue(entered.wait(timeout=4))
                with lock:
                    worker.append(tail)
                    self.assertFalse(worker.ready_for_save())
                release.set()
                wait_for(lambda: worker.ready_for_save())
            self.assertEqual(len(committed), 1)
            store, grid, poses, delta = committed[0]
            self.assertEqual(len(poses), 3)
            np.testing.assert_allclose(poses[-1], pose(6))
            np.testing.assert_allclose(delta, pose(-1))
            reference, reference_grid = rebuild(prefix+[tail], [pose(), pose(4), pose(6)],
                                                .02, .15, .1)
            np.testing.assert_array_equal(store.snapshot(), reference.snapshot())
            self.assertEqual(grid.free, reference_grid.free)
            self.assertEqual(worker.checked, 3)
            self.assertEqual(backend.observation_count, 3)
            self.assertEqual(worker.applied_loops, 1)
            self.assertIs(worker.observations[-1].points_body, tail.points_body)
            np.testing.assert_allclose(worker.observations[-1].pose, pose(6))
            np.testing.assert_allclose(backend.poses[-1], pose(6))
            np.testing.assert_allclose(tail.pose, pose(7))
        finally:
            release.set()
            worker.close()

    def test_prepare_or_backend_failure_leaves_live_map_and_journal_unchanged(self):
        for failure in ('prepare', 'backend'):
            with self.subTest(failure=failure):
                lock = threading.RLock()
                backend = ControlledBackend(fail_apply=failure == 'backend')
                committed = []
                def prepare(store, grid, poses, delta):
                    if failure == 'prepare':
                        raise RuntimeError('controlled prepare failure')
                    return lambda: committed.append(store)
                worker = MappingLoopWorker(lock, None, .02, .15, .10, prepare, backend)
                originals = [observation(0), observation(5)]
                try:
                    with lock:
                        for item in originals:
                            worker.append(item)
                    wait_for(lambda: worker.report()['state'] == 'error')
                    with lock:
                        self.assertFalse(worker.ready_for_save())
                        self.assertIn('controlled', worker.error)
                        self.assertEqual(worker.applied_loops, 0)
                        self.assertEqual(committed, [])
                        self.assertFalse(backend.applied)
                        for expected, actual, matrix in zip(originals, worker.observations, backend.poses):
                            self.assertIs(actual, expected)
                            np.testing.assert_array_equal(matrix, expected.pose)
                finally:
                    worker.close()

    def test_no_loop_checks_complete_without_rebuilding_or_committing(self):
        lock = threading.RLock()
        backend = ControlledBackend(propose=False)
        commits = []
        worker = MappingLoopWorker(lock, None, .02, .15, .10,
                                   lambda *args: commits.append(args), backend)
        try:
            with patch('mapping_loop.rebuild', side_effect=AssertionError('unexpected rebuild')):
                with lock:
                    worker.append(observation(0))
                    worker.append(observation(3))
                wait_for(lambda: worker.ready_for_save())
                self.assertEqual(worker.report()['state'], 'idle')
                self.assertEqual(worker.checked, 2)
                self.assertFalse(worker.error)
                self.assertEqual(commits, [])
        finally:
            worker.close()


if __name__ == '__main__':
    unittest.main()
