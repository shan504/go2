"""Private-process protocol checks plus a real pinned native backend smoke test.

The controlled endpoints test lost ACKs, malformed replies, timeouts and
proposal ownership. They do not test place recognition. The explicitly named
native test starts the actual installed KISS-SLAM, integrates a small cloud,
flushes a submap and checks crash isolation; it does not assert a loop closure.
"""
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'go2_3d'))
from kiss_backend import KissConfig, UPSTREAM_COMMIT
from kiss_backend_process import KissBackendProcess


_ENDPOINT = r'''
import os
import socket
import struct
import sys
import time
import numpy as np
from kiss_backend_process import _receive, _send

channel = socket.socket(fileno=int(sys.argv[1]))
mode = sys.argv[2]
count, revision = 0, 0
while True:
    try:
        request = _receive(channel)
    except (EOFError, OSError):
        break
    command = request['command']
    value = None
    if command == 'init':
        config = request['config']
    elif command == 'append':
        if mode == 'crash':
            os._exit(71)
        if mode == 'eof':
            channel.close()
            break
        if mode == 'timeout':
            time.sleep(30)
        if mode == 'malformed':
            _send(channel, {'ok': True})
            continue
        if mode == 'wrong_type':
            _send(channel, [])
            continue
        if mode == 'oversize':
            channel.sendall(struct.pack('!Q', 513 * 1024 * 1024))
            continue
        if mode == 'error':
            _send(channel, {'ok': False, 'error': 'controlled native failure'})
            continue
        count += 1
        revision += 1
        value = count - 1
    elif command == 'detect':
        if count:
            poses = tuple(np.eye(4) for _ in range(count))
            value = dict(poses=poses, original_poses=poses, loop_edges=(),
                         status={'state': 'controlled_proposal'},
                         token='proposal-' + str(revision))
            if mode == 'malformed_proposal':
                value = 'not a proposal'
            if mode == 'wrong_pose':
                value['poses'][0][0, 0] = float('nan')
            if mode == 'count_mismatch':
                value['poses'] = ()
            if mode == 'empty_token':
                value['token'] = ''
    elif command == 'apply':
        if request['token'] != 'proposal-' + str(revision):
            _send(channel, {'ok': False, 'error': 'stale controlled token'})
            continue
        revision += 1
    elif command == 'flush':
        value = True
    _send(channel, dict(ok=True, observation_count=count,
                        status={'state': 'controlled', 'config': config}, value=value))
'''


@contextmanager
def controlled_backend(mode='normal'):
    """Launch an actual child, replacing only its native algorithm endpoint."""
    process_factory = subprocess.Popen
    with tempfile.TemporaryDirectory(prefix='kiss-ipc-test-') as directory:
        helper = Path(directory) / 'endpoint.py'
        helper.write_text(_ENDPOINT)

        def spawn(arguments, **kwargs):
            return process_factory([sys.executable, str(helper), arguments[-1], mode],
                                   **kwargs)

        with patch('kiss_backend_process.subprocess.Popen', side_effect=spawn):
            backend = KissBackendProcess(KissConfig(submap_distance=12.5),
                                         python=sys.executable, timeout=5.0)
        try:
            yield backend
        finally:
            backend.close()


class KissProcessProtocolTests(unittest.TestCase):
    def test_configuration_roundtrip_and_successful_append(self):
        with controlled_backend() as backend:
            self.assertEqual(backend.observation_count, 0)
            self.assertEqual(backend.status['config']['submap_distance'], 12.5)
            self.assertEqual(backend.status['isolation'], 'private_process')
            self.assertEqual(backend.append(np.ones((5, 3)), np.eye(4)), 0)
            self.assertEqual(backend.observation_count, 1)
            self.assertTrue(backend.flush())

    def test_proposal_retry_is_readonly_and_apply_is_once_only(self):
        with controlled_backend() as backend:
            backend.append(np.ones((5, 3)), np.eye(4))
            first = backend.detect_and_optimize()
            second = backend.detect_and_optimize()
            self.assertEqual(first.token, second.token)
            self.assertFalse(second.poses[0].flags.writeable)
            self.assertFalse(second.original_poses[0].flags.writeable)
            backend.apply(first)  # Same server proposal may survive a rebuild retry.
            with self.assertRaisesRegex(ValueError, 'stale'):
                backend.apply(second)
            self.assertEqual(backend.observation_count, 1)

    def test_append_invalidates_old_proposal_without_committing_it(self):
        with controlled_backend() as backend:
            backend.append(np.ones((5, 3)), np.eye(4))
            old = backend.detect_and_optimize()
            backend.append(np.ones((5, 3)), np.eye(4))
            with self.assertRaisesRegex(ValueError, 'stale'):
                backend.apply(old)
            self.assertEqual(backend.observation_count, 2)
            self.assertNotEqual(backend.detect_and_optimize().token, old.token)

    def test_foreign_proposal_is_rejected_locally(self):
        with controlled_backend() as first, controlled_backend() as second:
            for backend in (first, second):
                backend.append(np.ones((5, 3)), np.eye(4))
            other = first.detect_and_optimize()
            second.detect_and_optimize()
            with self.assertRaisesRegex(ValueError, 'foreign'):
                second.apply(other)
            self.assertEqual(second.status['state'], 'controlled')

    def test_missing_or_corrupt_ack_poison_process_without_advancing_count(self):
        for mode in ('crash', 'eof', 'malformed', 'wrong_type', 'oversize', 'error'):
            with self.subTest(mode=mode), controlled_backend(mode) as backend:
                with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                    backend.append(np.ones((5, 3)), np.eye(4))
                self.assertEqual(backend.observation_count, 0)
                self.assertEqual(backend.status['state'], 'error')
                with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                    backend.flush()

    def test_timeout_fails_closed_and_close_reaps_child(self):
        with controlled_backend('timeout') as backend:
            process = backend._process
            backend._channel.settimeout(.15)
            started = time.monotonic()
            with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                backend.append(np.ones((5, 3)), np.eye(4))
            self.assertLess(time.monotonic() - started, 4.0)
            self.assertEqual(backend.observation_count, 0)
            backend.close()
            self.assertIsNotNone(process.poll())
            self.assertIsNone(backend._channel)
            backend.close()  # Idempotent, including after a timeout.

    def test_malformed_optimization_proposal_poison_process(self):
        for mode in ('malformed_proposal', 'wrong_pose', 'count_mismatch', 'empty_token'):
            with self.subTest(mode=mode), controlled_backend(mode) as backend:
                backend.append(np.ones((5, 3)), np.eye(4))
                with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                    backend.detect_and_optimize()
                self.assertEqual(backend.status['state'], 'error')
                with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                    backend.flush()

    def test_idle_close_reaps_child_and_reuse_is_explicit_error(self):
        with controlled_backend() as backend:
            process = backend._process
            backend.close()
            self.assertIsNotNone(process.poll())
            with self.assertRaises(RuntimeError):
                backend.append(np.ones((5, 3)), np.eye(4))

    def test_close_interrupts_inflight_native_wait(self):
        with controlled_backend('timeout') as backend:
            process = backend._process
            backend._channel.settimeout(30.0)
            entered = threading.Event()
            errors = []

            def request():
                entered.set()
                try:
                    backend.append(np.ones((5, 3)), np.eye(4))
                except RuntimeError as error:
                    errors.append(error)

            caller = threading.Thread(target=request)
            caller.start()
            self.assertTrue(entered.wait(timeout=2.0))
            time.sleep(.05)
            started = time.monotonic()
            backend.close()
            caller.join(timeout=3.0)
            self.assertFalse(caller.is_alive(), 'shutdown left an RPC waiting for its 30s timeout')
            self.assertLess(time.monotonic()-started, 4.0)
            self.assertEqual(len(errors), 1)
            self.assertIsNotNone(process.poll())

    def test_invalid_interpreter_and_timeout_fail_before_child_launch(self):
        with self.assertRaisesRegex(ValueError, 'interpreter'):
            KissBackendProcess(python='/nonexistent/go2-kiss-python')
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.subTest(timeout=timeout), self.assertRaisesRegex(ValueError, 'timeout'):
                KissBackendProcess(timeout=timeout)


_NATIVE_PYTHON = Path(os.environ.get('GO2_KISS_TEST_PYTHON', ROOT / '.venv-kiss/bin/python'))


@unittest.skipUnless(_NATIVE_PYTHON.is_file(), 'isolated KISS environment is not installed')
class KissProcessNativeTests(unittest.TestCase):
    def test_real_pinned_native_init_voxel_flush_and_crash_isolation(self):
        backend = KissBackendProcess(KissConfig(), python=str(_NATIVE_PYTHON), timeout=60)
        process = backend._process
        try:
            self.assertEqual(backend.status['upstream_commit'], UPSTREAM_COMMIT)
            axis = np.linspace(-2., 2., 41)
            x, y = np.meshgrid(axis, axis)
            points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.3)))
            backend.append(points, np.eye(4))
            pose = np.eye(4)
            pose[0, 3] = .2
            backend.append(points, pose)
            self.assertEqual(backend.observation_count, 2)
            self.assertIsNone(backend.detect_and_optimize())
            self.assertTrue(backend.flush())
            self.assertFalse(backend.flush())
            self.assertIsNone(backend.detect_and_optimize())
            self.assertEqual(backend.status['committed_optimizations'], 0)
            # Actual process death must never be mistaken for a successful scan
            # or graph update; this test does not simulate a field loop closure.
            process.kill()
            process.wait(timeout=3)
            with self.assertRaisesRegex(RuntimeError, 'Restart/replay'):
                backend.append(points, pose)
            self.assertEqual(backend.observation_count, 2)
        finally:
            backend.close()
        self.assertIsNotNone(process.poll())


if __name__ == '__main__':
    unittest.main()
