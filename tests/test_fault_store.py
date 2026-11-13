"""Persistent evidence survives sessions, failures and offline inspection."""
from concurrent.futures import ThreadPoolExecutor
import contextlib
import gzip
import io
import json
import os
import shutil
from pathlib import Path
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from fault_store import (ARTIFACT_BYTES, EventStore, create_session, enforce_limits,
                         export_fault, find_fault, list_faults, read_json)
from fault_cli import main as cli


class FaultStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)/'diagnostic path'
        self.session = create_session(self.root, {'source': {'commit': 'abc123'}})
        self.store = EventStore(self.session)

    def tearDown(self):
        self.temporary.cleanup()

    def fault(self, message='Failed to make progress'):
        return self.store.record_fault(message, {'route': {'current': '5'},
            'commands': {'cmd_vel': {'last': [0.3, 0, 0]}},
            'odometry': {'net_displacement_m': 0.01}}, {'initial_snapshot': {'before': True}})

    def test_fault_initial_trigger_and_frozen_logs_survive_enrichment_and_rotation(self):
        (self.session/'launch.log').write_text('before\nFailed to make progress\n')
        identity = self.fault()
        directory = self.session/'faults'/identity
        self.assertEqual(gzip.decompress((directory/'launch-before.log.gz').read_bytes()),
                         b'before\nFailed to make progress\n')
        (self.session/'launch.log').write_text('later\nGoal failed\n')
        self.store.update_fault(identity, {'route': {'current': '-', 'paused': True}},
                                {'final_action': {'status': 'ABORTED'}, 'events_after': ['stop']})
        summary = read_json(directory/'summary.json')
        evidence = read_json(directory/'evidence.json')
        self.assertEqual(summary['trigger']['code'], 'progress_timeout')
        self.assertEqual(evidence['initial_snapshot'], {'before': True})
        self.assertEqual(evidence['events_after'], ['stop'])
        self.assertEqual(gzip.decompress((directory/'launch-after.log.gz').read_bytes()), b'later\nGoal failed\n')
        (self.session/'launch.log').write_text('unrelated new rotated log')
        self.assertIn('progress_timeout', (directory/'summary.json').read_text())
        self.assertIn('最初触发', (directory/'report.txt').read_text())
        self.assertIn('ABORTED', (directory/'report.txt').read_text())

    def test_latest_fault_ignores_new_idle_session_and_manual_snapshot(self):
        identity = self.fault()
        newer = create_session(self.root)
        EventStore(newer).record_fault({'code': 'manual_snapshot', 'manual': True,
                                       'severity': 'info', 'message': 'user snapshot'}, {})
        create_session(self.root)
        self.assertEqual(find_fault(self.root)['fault_id'], identity)
        self.assertEqual(len(list_faults(self.root)), 2)

    def test_export_keeps_full_fault_context_and_explicit_missing_logs(self):
        identity = self.fault()
        context = self.session/'context'
        context.mkdir()
        (context/'nav2.yaml').write_text('speed: 0.3\n')
        extra = self.session/'faults'/identity/'extra.json'
        extra.write_text('{"retained": true}')
        archive = export_fault(self.root)
        with tarfile.open(archive) as reader:
            names = reader.getnames()
            self.assertIn('fault/extra.json', names)
            self.assertIn('session/manifest.json', names)
            self.assertIn('session/context/nav2.yaml', names)
            summary = json.load(reader.extractfile('fault/summary.json'))
            evidence = json.load(reader.extractfile('fault/evidence.json'))
            self.assertEqual(summary['fault_id'], identity)
            self.assertFalse(evidence['raw_log_before']['available'])
            self.assertIn('不是唯一', reader.extractfile('fault/report.txt').read().decode().replace('不代表唯一', '不是唯一'))
            self.assertFalse(any(name.startswith('/') or '..' in Path(name).parts for name in names))

    def test_cli_works_after_container_is_gone_with_no_ros_dependencies(self):
        identity = self.fault()
        for command in (['list'], ['show'], ['show', identity], ['sessions'], ['export', identity]):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(cli(['--root', str(self.root), *command]), 0)
            self.assertTrue(output.getvalue())
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli(['--root', str(self.root), 'show', '../escape']), 2)

    def test_symlink_and_unmarked_data_are_not_followed_or_replaced(self):
        unknown = self.root/'my-user-folder'
        unknown.mkdir()
        (unknown/'keep.txt').write_text('untouched')
        with self.assertRaises(ValueError):
            EventStore(unknown)
        alias = self.root/'alias'
        alias.symlink_to(self.session, target_is_directory=True)
        with self.assertRaises(ValueError):
            EventStore(alias)
        self.fault()
        context = self.session/'context'
        context.mkdir()
        secret = Path(self.temporary.name)/'unrelated.txt'
        secret.write_text('private-unrelated-value')
        (context/'unsafe.txt').symlink_to(secret)
        archive = export_fault(self.root)
        with tarfile.open(archive) as reader:
            self.assertNotIn('session/context/unsafe.txt', reader.getnames())
            manifest = json.load(reader.extractfile('export_manifest.json'))
            self.assertTrue(any('unsafe.txt' in item for item in manifest['omissions']))
        self.assertEqual((unknown/'keep.txt').read_text(), 'untouched')

    def test_retention_removes_only_marked_closed_sessions_and_keeps_active(self):
        self.store.write_manifest({'ended_utc': '2026-11-13T00:00:00Z'})
        self.fault()
        for _ in range(3):
            session = create_session(self.root)
            EventStore(session).write_manifest({'ended_utc': '2026-11-13T00:00:00Z'})
        active = create_session(self.root, {'capture_pid': os.getpid()})
        unowned = self.root/'keep'
        unowned.mkdir()
        (unowned/'user.txt').write_text('preserve')
        report = enforce_limits(self.root, max_sessions=1, keep_session=active)
        self.assertEqual(report['sessions'], 1)
        self.assertTrue(active.exists())
        self.assertEqual((unowned/'user.txt').read_text(), 'preserve')

    def test_retention_preserves_unowned_export_lookalikes(self):
        exports = self.root/'exports'
        exports.mkdir()
        user_archive = exports/'fault-my-user-file.tar.gz'
        user_archive.write_text('do not delete')
        enforce_limits(self.root, max_bytes=1)
        self.assertEqual(user_archive.read_text(), 'do not delete')

    def test_quiet_old_live_capture_is_protected_and_dead_pid_is_not(self):
        active = create_session(self.root, {'capture_pid': os.getpid()})
        old = __import__('time').time()-7200
        os.utime(active, (old, old))
        self.store.write_manifest({'ended_utc': '2026-11-13T00:00:00Z'})
        report = enforce_limits(self.root, max_sessions=0)
        self.assertTrue(active.exists())
        self.assertTrue(report['over_budget'])
        EventStore(active).write_manifest({'capture_pid': 999999999})
        enforce_limits(self.root, max_sessions=0)
        self.assertFalse(active.exists())

    def test_reused_pid_and_initial_evidence_overwrite_are_rejected(self):
        reused = create_session(self.root, {'capture_pid': os.getpid(),
                                            'capture_process_start': 'not-this-process'})
        enforce_limits(self.root, max_sessions=1, keep_session=self.session)
        self.assertFalse(reused.exists())
        identity = self.fault()
        with self.assertRaises(ValueError):
            self.store.update_fault(identity, evidence={'initial_snapshot': {'changed': True}})
        self.assertEqual(read_json(self.session/'faults'/identity/'evidence.json')['initial_snapshot'],
                         {'before': True})

    def test_bind_mount_basename_does_not_change_canonical_session_identity(self):
        mounted = Path(self.temporary.name)/'diagnostics'
        shutil.copytree(self.session, mounted)
        container_store = EventStore(mounted)
        container_store.write_manifest({'recorder': {'started': True}})
        identity = container_store.record_fault('Failed to make progress', {})
        container_store.update_fault(identity, evidence={'events_after': ['stop']})
        self.assertTrue(identity.startswith(self.session.name+'--'))
        self.assertEqual(read_json(mounted/'manifest.json')['session_id'], self.session.name)
        self.assertEqual(read_json(mounted/'faults'/identity/'summary.json')['session_id'], self.session.name)

    def test_terminal_control_codes_are_escaped_but_raw_evidence_is_preserved(self):
        message = 'bad \x1b[2J\r\u202eerror'
        identity = self.store.record_fault({'message': message}, {})
        directory = self.session/'faults'/identity
        self.assertEqual(read_json(directory/'summary.json')['trigger']['message'], message)
        report = (directory/'report.txt').read_text()
        self.assertNotIn('\x1b', report)
        self.assertIn('\\x1b', report)
        self.assertNotIn('\u202e', report)
        for command in ('list', 'show'):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(cli(['--root', str(self.root), command]), 0)
            self.assertNotIn('\x1b', output.getvalue())
            self.assertNotIn('\u202e', output.getvalue())

    def test_frozen_tail_crosses_log_rotation_and_marks_missing_segments(self):
        (self.session/'launch.log.2').write_bytes(b'older\n')
        (self.session/'launch.log.1').write_bytes(b'previous\n')
        (self.session/'launch.log').write_bytes(b'current\n')
        identity = self.fault()
        directory = self.session/'faults'/identity
        self.assertEqual(gzip.decompress((directory/'launch-before.log.gz').read_bytes()),
                         b'older\nprevious\ncurrent\n')
        metadata = read_json(directory/'evidence.json')['raw_log_before']
        self.assertEqual([item['file'] for item in metadata['sources']],
                         ['launch.log.2', 'launch.log.1', 'launch.log'])
        (self.session/'launch.log.1').unlink()
        self.store.update_fault(identity)
        metadata = read_json(directory/'evidence.json')['raw_log_after']
        self.assertTrue(any('launch.log.1' in gap for gap in metadata['coverage_gaps']))

    def test_frozen_rotated_tail_is_bounded_and_keeps_newest_bytes(self):
        (self.session/'launch.log.1').write_bytes(b'a'*(2*1024*1024))
        (self.session/'launch.log').write_bytes(b'current\n')
        identity = self.fault()
        directory = self.session/'faults'/identity
        captured = gzip.decompress((directory/'launch-before.log.gz').read_bytes())
        self.assertEqual(len(captured), 2*1024*1024)
        self.assertTrue(captured.endswith(b'current\n'))
        metadata = read_json(directory/'evidence.json')['raw_log_before']
        self.assertTrue(metadata['truncated'])
        self.assertEqual(metadata['sources'][0]['source_offset'], len(b'current\n'))

    def test_beijing_times_are_primary_with_utc_reference(self):
        identity = self.store.record_fault('Failed to make progress', {},
                                           now_utc='2026-11-12T16:30:00Z')
        report = (self.session/'faults'/identity/'report.txt').read_text()
        self.assertIn('2026-11-13 00:30:00.000 UTC+08:00', report)
        self.assertIn('UTC 原始时间: 2026-11-12T16:30:00Z', report)
        for command in ('list', 'show', 'sessions'):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(cli(['--root', str(self.root), command]), 0)
            self.assertIn('北京时间', output.getvalue())

    def test_periodics_never_prune_sole_latest_fault_even_over_budget(self):
        self.store.max_bytes = 1
        identity = self.fault()
        self.store.append_summary({'reason': 'periodic'})
        self.assertEqual(find_fault(self.root)['fault_id'], identity)
        storage = read_json(self.session/'manifest.json')['fault_store']
        self.assertTrue(storage['over_budget'])
        self.assertEqual(storage['preserved_fault'], identity)

    def test_recorder_heartbeat_protects_orphan_writer_but_not_closed_session(self):
        orphan = create_session(self.root, {'capture_pid': 999999999})
        EventStore(orphan).heartbeat()
        enforce_limits(self.root, max_sessions=1, keep_session=self.session)
        self.assertTrue(orphan.exists())
        EventStore(orphan).write_manifest({'ended_utc': '2026-11-13T00:00:00Z'})
        enforce_limits(self.root, max_sessions=1, keep_session=self.session)
        self.assertFalse(orphan.exists())

    def test_retry_after_publication_error_is_idempotent(self):
        trigger = {'code': 'progress_timeout', 'message': 'failed', 'recorder_token': 'same-incident'}
        with patch.object(self.store, '_retention', side_effect=OSError('after publication')):
            with self.assertRaises(OSError):
                self.store.record_fault(trigger, {'initial': True}, {'initial_snapshot': {'initial': True}})
        identity = self.store.record_fault(trigger, {'must_not_replace': True})
        self.assertEqual(len(list_faults(self.root)), 1)
        self.assertEqual(read_json(self.session/'faults'/identity/'summary.json')['snapshot'], {'initial': True})

    def test_export_snapshot_is_consistent_and_update_has_no_lock_inversion(self):
        store = EventStore(self.session, root_dir=self.root)
        identity = store.record_fault('failed', {'version': 0}, {'version': 0})
        captured = threading.Event()
        release = threading.Event()
        original = tarfile.TarFile.addfile
        def pause_archive(archive, member, fileobj=None):
            if member.name == 'fault/evidence.json':
                captured.set()
                self.assertTrue(release.wait(5))
            return original(archive, member, fileobj)
        with ThreadPoolExecutor(max_workers=2) as workers, \
                patch('fault_store.tarfile.TarFile.addfile', pause_archive):
            exported = workers.submit(export_fault, self.root, identity)
            self.assertTrue(captured.wait(5))
            updated = workers.submit(store.update_fault, identity, {'version': 1}, {'version': 1})
            time.sleep(0.03)
            self.assertFalse(updated.done())
            release.set()
            archive = exported.result(timeout=5)
            self.assertEqual(updated.result(timeout=5), identity)
        with tarfile.open(archive) as reader:
            self.assertEqual(json.load(reader.extractfile('fault/summary.json'))['snapshot']['version'], 0)
            self.assertEqual(json.load(reader.extractfile('fault/evidence.json'))['version'], 0)
        self.assertEqual(read_json(self.session/'faults'/identity/'summary.json')['snapshot']['version'], 1)

    def test_manifest_concurrent_writers_merge_without_clobber(self):
        def write(index):
            EventStore(self.session).write_manifest({'key'+str(index): index})
        with ThreadPoolExecutor(max_workers=6) as workers:
            list(workers.map(write, range(30)))
        manifest = read_json(self.session/'manifest.json')
        self.assertEqual(manifest['source'], {'commit': 'abc123'})
        for index in range(30):
            self.assertEqual(manifest['key'+str(index)], index)

    def test_low_disk_and_invalid_evidence_preserve_previous_fault(self):
        identity = self.fault()
        with patch('fault_store.shutil.disk_usage', return_value=type('Usage', (), {'free': 0})()):
            with self.assertRaises(OSError):
                self.fault()
        self.assertEqual(find_fault(self.root)['fault_id'], identity)
        with self.assertRaises(ValueError):
            self.store.record_fault('bad', {'x': float('nan')})
        with self.assertRaises(ValueError):
            self.store.record_fault('bad', {}, {'oversized': 'a'*(ARTIFACT_BYTES+1)})
        self.assertEqual(len(list_faults(self.root)), 1)
        self.assertFalse(any(p.name.startswith('.pending') for p in (self.session/'faults').iterdir()))

    def test_container_created_files_adopt_host_owner(self):
        with patch('fault_store.os.geteuid', return_value=0), \
                patch.dict(os.environ, {'GO2_DIAGNOSTICS_UID': '1234', 'GO2_DIAGNOSTICS_GID': '2345'}), \
                patch('fault_store.os.chown') as chown:
            self.fault()
        self.assertTrue(chown.called)
        self.assertTrue(all(call.args[1:] == (1234, 2345) for call in chown.call_args_list))

    def test_process_exit_kind_is_preserved_as_observed_trigger(self):
        identity = self.store.record_fault({'kind': 'process_exit', 'exit_code': 17,
                                           'source': 'launcher', 'message': 'exited 17'}, {})
        summary = read_json(self.session/'faults'/identity/'summary.json')
        self.assertEqual(summary['trigger']['code'], 'process_exit')
        self.assertEqual(summary['trigger']['exit_code'], 17)


if __name__ == '__main__':
    unittest.main()
