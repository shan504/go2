"""Real subprocess/terminal tests for persistent launch capture; no ROS or motion."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
import capture_navigation as capture
from fault_store import EventStore, create_session, enforce_limits, list_faults

SCRIPT = Path(capture.__file__).resolve()


class CaptureTests(unittest.TestCase):
    def project(self, root):
        project = Path(root)/'project with spaces'
        project.mkdir()
        (project/'go2_3d').mkdir()
        (project/'go2_3d/navigation_grid.yaml').write_text('grid_resolution: 0.05\n')
        return project

    def sessions(self, project):
        return sorted(path for path in (project/'runtime/diagnostics').iterdir()
                      if path.is_dir() and (path/'.go2-diagnostic-session').is_file())

    def launch(self, project, code, **kwargs):
        return subprocess.run([sys.executable, str(SCRIPT), '--project', str(project), '--',
            sys.executable, '-c', code], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=15, **kwargs)

    def test_binary_output_exit_fault_and_no_environment_dump(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            environment = dict(os.environ, PRIVATE_CAPTURE_CANARY='never-copy-this-credential')
            result = self.launch(project, "import os; os.write(1,b'out\\x00\\xff\\n'); "
                "os.write(2,b'err\\n'); print(os.environ['GO2_DIAGNOSTICS_CAPTURE_ACTIVE']); "
                "print(os.environ['GO2_DIAGNOSTICS_SESSION']); raise SystemExit(7)", env=environment)
            self.assertEqual(result.returncode, 7, result.stderr)
            session = self.sessions(project)[0]
            self.assertEqual((session/'launch.log').read_bytes(), result.stdout)
            self.assertIn(b'out\x00\xff\nerr\n', result.stdout)
            manifest = json.loads((session/'manifest.json').read_text())
            self.assertEqual(manifest['exit_code'], 7)
            self.assertEqual(manifest['capture']['status'], 'failed')
            self.assertIsInstance(manifest['capture_pid'], int)
            self.assertIn('ended_utc', manifest)
            faults = list_faults(project/'runtime/diagnostics')
            self.assertEqual(len(faults), 1)
            self.assertEqual(faults[0]['trigger']['code'], 'process_exit')
            self.assertEqual(faults[0]['trigger']['exit_code'], 7)
            for path in session.rglob('*'):
                if path.is_file() and not path.name.endswith('.gz'):
                    self.assertNotIn(b'never-copy-this-credential', path.read_bytes())

    def test_context_is_bounded_refreshes_generated_and_rejects_map_escape(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            outside = Path(root)/'outside'
            outside.mkdir()
            (outside/'metadata.yaml').write_text('sensitive-map: must-not-copy\n')
            (project/'maps').mkdir()
            (project/'maps/latest').symlink_to(outside, target_is_directory=True)
            large = project/'go2_3d/gicp.yaml'
            large.write_bytes(b'x'*(capture.CONTEXT_BYTES+1))
            result = self.launch(project, "from pathlib import Path; "
                "p=Path('runtime/config'); p.mkdir(parents=True,exist_ok=True); "
                "(p/'nav2_3d.yaml').write_text('controller_frequency: 10.0\\n'); print('ready')")
            self.assertEqual(result.returncode, 0, result.stderr)
            session = self.sessions(project)[0]
            manifest = json.loads((session/'manifest.json').read_text())
            self.assertIn('unavailable', manifest['context']['selected_map'])
            self.assertIn('go2_3d/gicp.yaml', manifest['context']['omissions'])
            generated = session/'context/runtime/config/nav2_3d.yaml'
            self.assertEqual(generated.read_text(), 'controller_frequency: 10.0\n')
            self.assertFalse((session/'context/selected_map/metadata.yaml').exists())
            self.assertFalse(list_faults(project/'runtime/diagnostics'))

    def test_rotation_preserves_trailing_bytes_and_bound(self):
        with tempfile.TemporaryDirectory() as root:
            value = bytes(range(100))
            writer = capture.RotatingBytes(root, limit=8, count=3)
            writer.write(value[:11])
            writer.write(value[11:])
            writer.close()
            paths = [Path(root)/'launch.log.2', Path(root)/'launch.log.1', Path(root)/'launch.log']
            content = b''.join(path.read_bytes() for path in paths)
            self.assertTrue(value.endswith(content))
            self.assertLessEqual(sum(path.stat().st_size for path in paths), 24)
            self.assertEqual(writer.total_bytes, len(value))
            self.assertEqual(writer.discarded_bytes+len(content), len(value))
            self.assertTrue(writer.summary()['truncated'])

    def test_logging_failure_does_not_replace_child_exit(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            with patch.object(capture.RotatingBytes, 'write', side_effect=OSError('disk full')), \
                    patch.object(capture, 'warn') as warning:
                code = capture.run_capture(project, [sys.executable, '-c', "print('child-completed')"])
            self.assertEqual(code, 0)
            session = self.sessions(project)[0]
            manifest = json.loads((session/'manifest.json').read_text())
            self.assertEqual(manifest['capture']['log_error'], 'disk full')
            self.assertTrue(any('terminal and navigation continue' in str(call) for call in warning.call_args_list))

    def test_slow_disk_is_bounded_and_does_not_delay_relay_or_child_exit(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            blocked, release = threading.Event(), threading.Event()
            original = capture.RotatingBytes.write
            def slow_write(writer, data):
                blocked.set()
                release.wait(5)
                original(writer, data)
            with tempfile.TemporaryFile() as terminal, patch.object(capture, 'warn') as warning:
                saved_stdout = os.dup(1)
                try:
                    os.dup2(terminal.fileno(), 1)
                    with patch.object(capture.RotatingBytes, 'write', slow_write):
                        started = time.monotonic()
                        code = capture.run_capture(project,
                            [sys.executable, '-u', '-c', "print('FAST-TERMINAL-RELAY')"],
                            flush_seconds=0.05)
                        elapsed = time.monotonic()-started
                    self.assertEqual(code, 0)
                    self.assertTrue(blocked.is_set())
                    self.assertLess(elapsed, 1.5)
                    terminal.seek(0)
                    self.assertIn(b'FAST-TERMINAL-RELAY', terminal.read())
                    self.assertTrue(any('flush incomplete' in str(call) for call in warning.call_args_list))
                finally:
                    os.dup2(saved_stdout, 1)
                    os.close(saved_stdout)
                    release.set()
                    for thread in threading.enumerate():
                        if thread.name == 'diagnostic-writer':
                            thread.join(2)
            accepted = []
            blocked, release = threading.Event(), threading.Event()
            def paused(value):
                blocked.set()
                release.wait(3)
                accepted.append(value)
            worker = capture.AsyncWriter(paused, max_bytes=32)
            worker.start()
            try:
                self.assertTrue(worker.enqueue(b'a'*16))
                self.assertTrue(blocked.wait(1))
                self.assertTrue(worker.enqueue(b'b'*16))
                self.assertFalse(worker.enqueue(b'c'))
                self.assertLessEqual(worker.summary()['peak_buffered_bytes'], 32)
                self.assertEqual(worker.summary()['dropped_bytes'], 1)
                worker.finish()
                self.assertFalse(worker.done.wait(0.05))
            finally:
                release.set()
                self.assertTrue(worker.done.wait(2))
            self.assertEqual(b''.join(accepted), b'a'*16+b'b'*16)

    def test_queued_process_death_keeps_receipt_intent_after_later_manual_stop(self):
        records = []
        class Store:
            def record_fault(self, trigger, snapshot, evidence):
                records.append((trigger, snapshot, evidence))
                return 'fault-'+str(len(records))
        state = {'requested_signal': None}
        parser = capture.LaunchDeaths(Store(), state, ['navigation'])
        blocked, release = threading.Event(), threading.Event()
        def slow_disk(data):
            blocked.set()
            release.wait(3)
            parser.feed(data)
        worker = capture.AsyncWriter(slow_disk)
        worker.start()
        try:
            before = b"[ERROR] [launch]: process has died [pid 77, exit code -11, cmd 'recorder']\n"
            after = b"[ERROR] [launch]: process has died [pid 88, exit code -2, cmd 'cleanup']\n"
            worker.enqueue(capture.ReceivedChunk(before, False, '2026-11-13T00:00:00Z'))
            self.assertTrue(blocked.wait(1))
            state['requested_signal'] = signal.SIGINT
            worker.enqueue(capture.ReceivedChunk(after, True, '2026-11-13T00:00:01Z'))
            worker.finish()
        finally:
            release.set()
            self.assertTrue(worker.done.wait(2))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][0]['pid'], 77)
        self.assertEqual(records[0][0]['exit_code'], -11)
        self.assertEqual(records[0][2]['receipt']['received_utc'], '2026-11-13T00:00:00Z')
        self.assertFalse(records[0][2]['receipt']['manual_intent'])

    def test_process_death_saved_while_launch_keeps_running(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            death = "[ERROR] [launch]: process has died [pid 4711, exit code -11, cmd 'diagnostics.py']"
            code = "import time; print("+repr(death)+",flush=True); print('READY',flush=True); time.sleep(60)"
            process = subprocess.Popen([sys.executable, str(SCRIPT), '--project', str(project), '--',
                sys.executable, '-u', '-c', code], stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                self.assertTrue(select.select([process.stdout], [], [], 10)[0])
                self.assertIn(b'process has died', process.stdout.readline())
                deadline = time.monotonic()+3
                faults = []
                while not faults and time.monotonic() < deadline:
                    faults = list_faults(project/'runtime/diagnostics')
                    time.sleep(0.02)
                self.assertTrue(faults)
                self.assertIsNone(process.poll())
                self.assertEqual(faults[0]['trigger']['source'], 'ros_launch')
                self.assertEqual(faults[0]['trigger']['exit_code'], -11)
                process.terminate()
                process.communicate(timeout=5)
                self.assertEqual(len(list_faults(project/'runtime/diagnostics')), 1)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_escaped_runtime_rejected_before_creating_external_directory(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            outside = Path(root)/'external'
            outside.mkdir()
            (project/'runtime').symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'inside the project'):
                capture.run_capture(project, [sys.executable, '-c', 'pass'])
            self.assertFalse((outside/'diagnostics').exists())

    def test_missing_command_is_saved_as_startup_fault(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            with patch.object(capture, 'warn'):
                code = capture.run_capture(project, ['/missing-go2-test-command'])
            self.assertEqual(code, 127)
            faults = list_faults(project/'runtime/diagnostics')
            self.assertEqual(faults[0]['trigger']['exit_code'], 127)

    def test_low_disk_fails_before_starting_child(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            with self.assertRaisesRegex(OSError, 'free disk'):
                capture.run_capture(project, [sys.executable, '-c',
                    "from pathlib import Path; Path('child-started').touch()"], min_free_bytes=2**60)
            self.assertFalse((project/'child-started').exists())

    def test_sigterm_forwards_drains_tail_reaps_child_and_is_not_fault(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            code = ("import os,signal,sys,time,subprocess; "
                "g=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
                "print('READY',g.pid,flush=True); "
                "exec(\"def stop(s,f):\\n g.wait(timeout=3)\\n print('FINAL-TAIL',flush=True)\\n sys.exit(143)\"); "
                "signal.signal(signal.SIGTERM,stop); time.sleep(60)")
            process = subprocess.Popen([sys.executable, str(SCRIPT), '--project', str(project), '--',
                sys.executable, '-u', '-c', code], stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                self.assertTrue(select.select([process.stdout], [], [], 10)[0])
                line = process.stdout.readline()
                self.assertTrue(line.startswith(b'READY'), line)
                grandchild = int(line.split()[1])
                process.terminate()
                output, error = process.communicate(timeout=8)
                self.assertEqual(process.returncode, 143, error)
                self.assertIn(b'FINAL-TAIL', output)
                with self.assertRaises(ProcessLookupError):
                    os.kill(grandchild, 0)
                self.assertFalse(list_faults(project/'runtime/diagnostics'))
                session = self.sessions(project)[0]
                self.assertIn(b'FINAL-TAIL', (session/'launch.log').read_bytes())
                self.assertEqual(json.loads((session/'manifest.json').read_text())['capture']['status'], 'interrupted')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_interactive_tty_ctrl_c_once_and_terminal_restoration(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            master, slave = pty.openpty()
            before = termios.tcgetattr(slave)
            code = ("import os,signal,sys,time; "
                "exec(\"def stop(s,f):\\n print('SIGINT-ONCE',flush=True)\\n sys.exit(130)\"); "
                "signal.signal(signal.SIGINT,stop); "
                "print('TTY',os.isatty(0),os.isatty(1),os.isatty(2),flush=True); time.sleep(60)")
            process = subprocess.Popen([sys.executable, str(SCRIPT), '--project', str(project), '--',
                sys.executable, '-u', '-c', code], stdin=slave, stdout=slave, stderr=slave,
                start_new_session=True)
            output = b''
            try:
                deadline = time.monotonic()+10
                while b'TTY True True True' not in output and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.2)[0]:
                        output += os.read(master, 65536)
                self.assertIn(b'TTY True True True', output)
                os.write(master, b'\x03')
                while process.poll() is None and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.2)[0]:
                        output += os.read(master, 65536)
                process.wait(timeout=3)
                while select.select([master], [], [], 0.05)[0]:
                    output += os.read(master, 65536)
                self.assertEqual(process.returncode, 130, output)
                self.assertEqual(output.count(b'SIGINT-ONCE'), 1, output)
                self.assertEqual(termios.tcgetattr(slave), before)
                self.assertFalse(list_faults(project/'runtime/diagnostics'))
                session = self.sessions(project)[0]
                self.assertEqual(json.loads((session/'manifest.json').read_text())['capture']['requested_signal'], signal.SIGINT)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                os.close(master)
                os.close(slave)

    def test_ignored_raw_ctrl_c_escalates_and_active_session_survives_retention(self):
        with tempfile.TemporaryDirectory() as root:
            project = self.project(root)
            master, slave = pty.openpty()
            # A small grace period is injected into this direct helper entry;
            # the production CLI retains its normal 10-second grace period.
            program = ("import sys; sys.path.insert(0,"+repr(str(SCRIPT.parent))+"); "
                "from capture_navigation import run_capture; "
                "raise SystemExit(run_capture("+repr(str(project))+
                ",[sys.executable,'-u','-c',\"import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); print('IGNORING',flush=True); time.sleep(60)\"],grace_seconds=0.3))")
            process = subprocess.Popen([sys.executable, '-c', program], stdin=slave, stdout=slave,
                                       stderr=slave, start_new_session=True)
            output = b''
            try:
                deadline = time.monotonic()+10
                while b'IGNORING' not in output and time.monotonic() < deadline:
                    if select.select([master], [], [], 0.2)[0]:
                        output += os.read(master, 65536)
                self.assertIn(b'IGNORING', output)
                session = self.sessions(project)[0]
                # A quiet long-running launch remains active through host PID,
                # not directory mtime or a newly written ROS summary.
                old = time.time()-120
                os.utime(session, (old, old))
                limits = enforce_limits(project/'runtime/diagnostics', max_sessions=0)
                self.assertTrue(session.exists())
                self.assertTrue(limits['over_budget'])
                os.write(master, b'\x03')
                process.wait(timeout=5)
                self.assertEqual(process.returncode, 137)
                manifest = json.loads((session/'manifest.json').read_text())
                self.assertTrue(manifest['capture']['forced_group_kill'])
                self.assertEqual(manifest['capture']['requested_signal'], signal.SIGINT)
                self.assertFalse(list_faults(project/'runtime/diagnostics'))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                os.close(master)
                os.close(slave)


if __name__ == '__main__':
    unittest.main()
