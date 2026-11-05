"""Recovery worker scheduling/lifecycle tests; geometric proof is tested separately."""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from localization_recovery import RecoveryJob, RecoveryWorker


def job(label, generation=1, serial=1):
    return RecoveryJob(generation, serial, label, None, None, None,
                       int(time.monotonic() * 1e9), time.monotonic())


class Backend:
    def __init__(self, blocked=None, failing=None):
        self.config = SimpleNamespace(max_scan_age=1.0)
        self.blocked, self.failing = blocked, failing
        self.entered, self.release = threading.Event(), threading.Event()
        self.seen = []
        self.resets = 0
        self.count = 0

    def reset(self):
        self.count = 0
        self.resets += 1

    def observe(self, points, odom, correction, received, now):
        self.seen.append(points)
        self.count += 1
        if points == self.blocked:
            self.entered.set()
            if not self.release.wait(3):
                raise RuntimeError('test did not release blocked search')
        if points == self.failing:
            raise ValueError('search failed after provisional confirmation')
        return SimpleNamespace(confirmations=self.count, accepted=False)


class RecoveryWorkerTests(unittest.TestCase):
    def test_only_latest_pending_job_is_processed_and_mapper_receives_old_generation(self):
        backend = Backend(blocked='active')
        finished = threading.Event()
        completed = []

        def callback(item, result, elapsed, error):
            completed.append((item.generation, item.points, result.confirmations, error))
            if item.points == 'latest':
                finished.set()

        worker = RecoveryWorker(backend, callback)
        try:
            worker.submit(job('active', generation=1))
            self.assertTrue(backend.entered.wait(2))
            self.assertTrue(worker.busy, 'Foreground registration cannot see the active CPU reservation')
            worker.submit(job('superseded', generation=2))
            worker.submit(job('latest', generation=2))
            backend.release.set()
            self.assertTrue(finished.wait(2))
            self.assertEqual(backend.seen, ['active', 'latest'])
            # Generation acceptance belongs to Mapper under its state lock;
            # the worker delivers the immutable job identity for that check.
            self.assertEqual(completed, [(1, 'active', 1, None), (2, 'latest', 1, None)])
            self.assertEqual(backend.resets, 2)
        finally:
            backend.release.set()
            worker.close()
            self.assertFalse(worker.busy)

    def test_generation_or_serial_change_resets_confirmation_session(self):
        backend = Backend()
        completed = []
        available = threading.Event()

        def callback(item, result, elapsed, error):
            completed.append(result.confirmations)
            available.set()

        worker = RecoveryWorker(backend, callback)
        try:
            for item in (job('a', 1, 4), job('b', 1, 4), job('c', 2, 4), job('d', 2, 5)):
                available.clear()
                worker.submit(item)
                self.assertTrue(available.wait(2))
            self.assertEqual(completed, [1, 2, 1, 1])
            self.assertEqual(backend.resets, 3)
        finally:
            worker.close()

    def test_search_exception_does_not_reuse_partial_proof_or_stop_worker(self):
        backend = Backend(failing='failure')
        completed = []
        available = threading.Event()

        def callback(item, result, elapsed, error):
            completed.append((item.points, result, error))
            available.set()

        worker = RecoveryWorker(backend, callback)
        try:
            worker.submit(job('failure'))
            self.assertTrue(available.wait(2))
            self.assertIsNone(completed[-1][1])
            self.assertIn('search failed', completed[-1][2])
            available.clear()
            worker.submit(job('fresh'))
            self.assertTrue(available.wait(2))
            self.assertIsNone(completed[-1][2])
            self.assertEqual(completed[-1][1].confirmations, 1)
            self.assertTrue(worker.thread.is_alive())
        finally:
            worker.close()

    def test_completion_exception_does_not_kill_future_recovery(self):
        backend = Backend()
        attempted = threading.Event()
        recovered = threading.Event()
        results = []

        def callback(item, result, elapsed, error):
            if item.points == 'bad_callback':
                attempted.set()
                raise RuntimeError('state callback failed')
            results.append(result.confirmations)
            recovered.set()

        worker = RecoveryWorker(backend, callback)
        try:
            worker.submit(job('bad_callback'))
            self.assertTrue(attempted.wait(2))
            worker.submit(job('fresh'))
            self.assertTrue(recovered.wait(2), 'Completion callback killed the recovery thread')
            self.assertEqual(results, [1], 'A failed state commit retained provisional proof')
            self.assertTrue(worker.thread.is_alive())
        finally:
            worker.close()

    def test_expired_computation_cannot_contribute_to_later_fresh_confirmation(self):
        clock = [100.]
        backend = Backend()
        original = backend.observe

        def observe(points, *args):
            result = original(points, *args)
            if points == 'slow':
                clock[0] += 2.
            return result

        backend.observe = observe
        completed = []
        available = threading.Event()

        def callback(item, result, elapsed, error):
            completed.append((item.points, None if result is None else result.confirmations, error))
            available.set()

        with patch('localization_recovery.time.monotonic', side_effect=lambda: clock[0]):
            worker = RecoveryWorker(backend, callback)
            try:
                worker.submit(job('slow'))
                self.assertTrue(available.wait(2))
                available.clear()
                worker.submit(job('fresh'))
                self.assertTrue(available.wait(2))
                self.assertEqual(completed[0], ('slow', None, 'expired recovery computation'),
                                 'An expired result must never be offered as usable recovery proof')
                self.assertEqual(completed[1], ('fresh', 1, None))
            finally:
                worker.close()

    def test_mapper_ineligible_result_resets_worker_proof(self):
        backend = Backend()
        completed = []
        available = threading.Event()

        def callback(item, result, elapsed, error):
            completed.append(result.confirmations)
            available.set()
            # Mapper can reject source ROS stamp/odometry/generation even
            # when worker computation finished within the monotonic deadline.
            return item.points != 'ineligible'

        worker = RecoveryWorker(backend, callback)
        try:
            worker.submit(job('ineligible'))
            self.assertTrue(available.wait(2))
            available.clear()
            worker.submit(job('fresh'))
            self.assertTrue(available.wait(2))
            self.assertEqual(completed, [1, 1])
        finally:
            worker.close()

    def test_close_discards_pending_work_and_suppresses_active_completion(self):
        backend = Backend(blocked='active')
        completed = []
        closed = threading.Event()
        worker = RecoveryWorker(backend, lambda *args: completed.append(args))
        closer = None
        try:
            worker.submit(job('active'))
            self.assertTrue(backend.entered.wait(2))
            worker.submit(job('pending'))
            closer = threading.Thread(target=lambda: (worker.close(), closed.set()))
            closer.start()
            self.assertTrue(worker.stopping.wait(2))
            self.assertFalse(closed.is_set())
            backend.release.set()
            self.assertTrue(closed.wait(2))
            self.assertEqual(backend.seen, ['active'])
            self.assertEqual(completed, [])
            self.assertFalse(worker.thread.is_alive())
        finally:
            backend.release.set()
            if closer is not None:
                closer.join(3)
            worker.close()


if __name__ == '__main__':
    unittest.main()
