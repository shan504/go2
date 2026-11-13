#!/usr/bin/env python3
"""Capture one navigation launch without changing its commands or motion policy.

The host owns the session and terminal. A child PTY preserves Docker -it when
stdin is a terminal; noninteractive callers use a separate child process group.
Only bounded whitelisted context is copied, never process environments or PCDs.
"""
import argparse
from collections import deque
from datetime import datetime, timedelta, timezone
import errno
import fcntl
import hashlib
import os
from pathlib import Path
import platform
import pty
import re
import selectors
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

from fault_store import EventStore, create_session, enforce_limits

LOG_BYTES = 8 * 1024 * 1024
LOG_FILES = 5
CONTEXT_BYTES = 512 * 1024
MIN_FREE_BYTES = 64 * 1024 * 1024
GRACE_SECONDS = 10.0
BEIJING = timezone(timedelta(hours=8), 'Asia/Shanghai')


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def warn(message):
    try:
        os.write(2, ('Navigation diagnostics: ' + message + '\n').encode('utf-8', 'replace'))
    except OSError:
        pass


def below(root, path):
    root, path = Path(root).resolve(), Path(path).resolve()
    return path == root or root in path.parents


def git_output(project, *args):
    try:
        result = subprocess.run(['git', '-C', str(project), *args], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, timeout=5, check=False)
        return result.stdout.decode('utf-8', 'replace').strip()[:65536] if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


class RotatingBytes:
    """Single-writer binary log; retained size never exceeds count * limit."""
    def __init__(self, directory, limit=LOG_BYTES, count=LOG_FILES, on_rotate=None):
        if limit < 1 or count < 1:
            raise ValueError('positive log size and file count required')
        self.path = Path(directory)/'launch.log'
        self.limit, self.count, self.on_rotate = limit, count, on_rotate
        self.total_bytes = self.rotations = self.discarded_bytes = 0
        self.stream = None
        self._open()

    def _open(self):
        flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY | getattr(os, 'O_NOFOLLOW', 0)
        descriptor = os.open(str(self.path), flags, 0o600)
        self.stream = os.fdopen(descriptor, 'ab', buffering=0)
        self.size = os.fstat(descriptor).st_size

    def _rotate(self):
        self.close()
        oldest = self.path if self.count == 1 else Path(str(self.path)+'.'+str(self.count-1))
        if oldest.is_symlink():
            raise OSError('refusing a symlink in capture logs')
        if oldest.exists():
            self.discarded_bytes += oldest.stat().st_size
            oldest.unlink()
        for index in range(self.count-2, 0, -1):
            source = Path(str(self.path)+'.'+str(index))
            if source.is_symlink():
                raise OSError('refusing a symlink in capture logs')
            if source.exists():
                os.replace(source, Path(str(self.path)+'.'+str(index+1)))
        if self.count > 1 and self.path.exists():
            os.replace(self.path, Path(str(self.path)+'.1'))
        self.rotations += 1
        self._open()
        if self.on_rotate:
            self.on_rotate()

    def write(self, value):
        value = memoryview(value)
        while value:
            if self.size >= self.limit:
                self._rotate()
            length = min(len(value), self.limit-self.size)
            chunk = value[:length]
            while chunk:
                written = self.stream.write(chunk)
                if not written:
                    raise OSError('capture log write made no progress')
                self.size += written
                self.total_bytes += written
                chunk = chunk[written:]
            value = value[length:]

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def summary(self):
        return dict(total_bytes=self.total_bytes, rotations=self.rotations,
                    discarded_bytes=self.discarded_bytes, max_file_bytes=self.limit,
                    retained_files=self.count, truncated=self.discarded_bytes > 0)


class ReceivedChunk(bytes):
    """Raw output plus the manual-intent snapshot when relay received it.

    A slow disk worker must not reclassify pre-stop process failures using a
    shutdown signal that arrived only after these bytes were queued.
    """
    def __new__(cls, data, manual_intent, received_utc):
        value = super().__new__(cls, data)
        value.manual_intent = bool(manual_intent)
        value.received_utc = str(received_utc)
        return value


class AsyncWriter:
    """A bounded nonblocking handoff; filesystem/console I/O never runs in relay."""
    def __init__(self, write, max_bytes=8*1024*1024, periodic=None, name='diagnostic-writer', on_error=None):
        self.write, self.max_bytes, self.periodic = write, max_bytes, periodic
        self.on_error = on_error
        self.queue, self.condition = deque(), threading.Condition()
        self.buffered = self.peak = self.dropped_bytes = self.dropped_chunks = 0
        self.error, self.periodic_error, self.closing, self.finish_callback = None, None, False, None
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)

    def start(self):
        self.thread.start()

    def enqueue(self, data):
        with self.condition:
            if self.error or self.closing or self.buffered+len(data) > self.max_bytes:
                self.dropped_bytes += len(data)
                self.dropped_chunks += 1
                return False
            self.queue.append(data if isinstance(data, ReceivedChunk) else bytes(data))
            self.buffered += len(data)
            self.peak = max(self.peak, self.buffered)
            self.condition.notify()
            return True

    def finish(self, callback=None):
        with self.condition:
            self.closing, self.finish_callback = True, callback
            self.condition.notify()

    def summary(self):
        with self.condition:
            return dict(max_buffer_bytes=self.max_bytes, peak_buffered_bytes=self.peak,
                        pending_bytes=self.buffered, dropped_bytes=self.dropped_bytes,
                        dropped_chunks=self.dropped_chunks, error=self.error, periodic_error=self.periodic_error)

    def _run(self):
        periodic_at = 0.0
        try:
            while True:
                with self.condition:
                    if not self.queue and not self.closing:
                        self.condition.wait(0.2)
                    data = self.queue.popleft() if self.queue else None
                    finished = data is None and self.closing
                if data is not None:
                    try:
                        if self.error is None:
                            self.write(data)
                        else:
                            with self.condition:
                                self.dropped_bytes += len(data)
                                self.dropped_chunks += 1
                    except Exception as error:
                        with self.condition:
                            self.error = str(error)
                            self.dropped_bytes += len(data)
                            self.dropped_chunks += 1
                        if self.on_error:
                            self.on_error(str(error))
                    finally:
                        with self.condition:
                            self.buffered -= len(data)
                now = time.monotonic()
                if self.periodic and now >= periodic_at:
                    try:
                        self.periodic()
                    except Exception as error:
                        with self.condition:
                            self.periodic_error = str(error)
                    periodic_at = now+2.0
                if finished:
                    if self.finish_callback:
                        self.finish_callback()
                    break
        finally:
            self.done.set()


class LaunchDeaths:
    """Recognize exact ROS launch process-death reports, not generic ERROR lines."""
    PATTERN = re.compile(r'process has died.*?\[pid\s+(\d+),\s*exit code\s+(-?\d+)', re.I)

    def __init__(self, store, capture, command):
        self.store, self.capture, self.command = store, capture, command
        self.buffer = b''
        self.saved = set()
        self.fault_ids = []

    def feed(self, data):
        # Bound an unterminated or malicious line; raw bytes still live in logs.
        self.buffer = (self.buffer+data)[-131072:]
        manual_intent = getattr(data, 'manual_intent', bool(self.capture.get('requested_signal')))
        received_utc = getattr(data, 'received_utc', utc_now())
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            text = line.decode('utf-8', 'replace').strip()[:32768]
            match = self.PATTERN.search(text)
            if match is None or manual_intent:
                continue
            identity = tuple(map(int, match.groups()))
            if identity in self.saved:
                continue
            if len(self.saved) >= 256:
                self.capture['dropped_process_exit_events'] = self.capture.get('dropped_process_exit_events', 0)+1
                continue
            self.saved.add(identity)
            try:
                fault = self.store.record_fault(dict(code='process_exit', source='ros_launch',
                    message=text, pid=identity[0], exit_code=identity[1]),
                    dict(capture=dict(self.capture)), dict(launch_argv=list(self.command),
                        unexpected_process_death=True,
                        receipt=dict(received_utc=received_utc, manual_intent=manual_intent,
                                     processed_utc=utc_now()),
                        evidence_boundary='Observed ROS launch exit report, not a unique physical cause'))
            except (OSError, ValueError) as error:
                self.capture['process_exit_record_error'] = str(error)[:1024]
                warn('cannot persist observed process death; raw log capture continues: '+str(error))
                continue
            self.fault_ids.append(fault)
            self.capture['unexpected_process_fault_ids'] = list(self.fault_ids)


class ContextCapture:
    SOURCE_FILES = (
        'go2_3d/gicp.yaml', 'go2_3d/gicp_dense.yaml', 'go2_3d/navigation_grid.yaml',
        'go2_3d/navigation_filter.yaml', 'go2_3d/navigate.xml', 'go2_3d/motion_profile.py',
        'Dockerfile', 'requirements-validation.txt')
    GENERATED_FILES = (
        'runtime/config/nav2_3d.yaml', 'runtime/config/go2_edu.urdf',
        'runtime/navigation_map/nav.yaml', 'runtime/navigation_map/metadata.yaml')

    def __init__(self, project, session):
        self.project, self.session = Path(project).resolve(), Path(session).resolve()
        self.records, self.omissions, self.seen = {}, {}, {}

    def copy(self, relative, label=None):
        label = label or relative
        source = self.project/relative
        try:
            resolved = source.resolve(strict=True)
            if not below(self.project, resolved) or not resolved.is_file():
                raise ValueError('source is not a regular file inside the project')
            stat = resolved.stat()
            if stat.st_size > CONTEXT_BYTES:
                raise ValueError('context file exceeds bounded size limit')
            signature = (str(resolved), stat.st_mtime_ns, stat.st_size)
            if self.seen.get(label) == signature:
                return False
            data = resolved.read_bytes()
            if len(data) > CONTEXT_BYTES:
                raise ValueError('context file grew beyond bounded size limit')
            destination = self.session/'context'/label
            if not below(self.session/'context', destination):
                raise ValueError('context destination escapes its directory')
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_name('.'+destination.name+'.capture-tmp')
            if temporary.is_symlink() or destination.is_symlink():
                raise ValueError('context destination is a symlink')
            with temporary.open('wb') as output:
                output.write(data)
            os.replace(temporary, destination)
            self.records[label] = dict(source=str(resolved.relative_to(self.project)),
                bytes=len(data), sha256=hashlib.sha256(data).hexdigest(), observed_utc=utc_now())
            self.seen[label] = signature
            self.omissions.pop(label, None)
            return True
        except (OSError, ValueError) as error:
            reason = str(error)
            changed = self.omissions.get(label) != reason
            self.omissions[label] = reason
            return changed

    def initial(self):
        for relative in self.SOURCE_FILES:
            self.copy(relative)
        selected = self.project/'maps/latest'
        try:
            resolved = selected.resolve(strict=True)
            if not below(self.project/'maps', resolved) or not resolved.is_dir():
                raise ValueError('selected map escapes maps/ or is not a directory')
            self.selected_map = dict(path=str(resolved.relative_to(self.project)))
            for name in ('metadata.yaml', 'nav.yaml'):
                self.copy(str(resolved.relative_to(self.project)/name), 'selected_map/'+name)
            cloud = resolved/'map.pcd'
            if cloud.is_file() and below(resolved, cloud):
                stat = cloud.stat()
                self.selected_map['point_cloud'] = dict(bytes=stat.st_size,
                    mtime_ns=stat.st_mtime_ns, copied=False, hashed=False)
        except (OSError, ValueError) as error:
            self.selected_map = dict(unavailable=str(error))
        self.refresh()
        return self.snapshot()

    def refresh(self):
        return any([self.copy(relative) for relative in self.GENERATED_FILES])

    def snapshot(self):
        return dict(files=dict(self.records), omissions=dict(self.omissions),
                    selected_map=getattr(self, 'selected_map', None), max_file_bytes=CONTEXT_BYTES,
                    point_clouds_copied=False)


def stop_owned_container(session_id):
    """Shutdown fallback only; stop immutable ID after checking our session label."""
    prefix = ['sudo', '-n', 'docker']
    try:
        result = subprocess.run(prefix+['inspect', '--format',
            '{{.Id}} {{index .Config.Labels "go2.diagnostics.session"}}', 'go2-3d'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=4, check=False)
        fields = result.stdout.decode('ascii', 'replace').strip().split()
        if result.returncode == 0 and len(fields) == 2 and fields[1] == session_id:
            return subprocess.run(prefix+['stop', '--time', '3', fields[0]],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6,
                check=False).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        pass
    return False


def run_capture(project, command, max_log_bytes=LOG_BYTES, log_files=LOG_FILES,
                min_free_bytes=MIN_FREE_BYTES, grace_seconds=GRACE_SECONDS, flush_seconds=2.0):
    project = Path(project).resolve(strict=True)
    root = project/'runtime/diagnostics'
    if not below(project, root):
        raise ValueError('diagnostic root must stay inside the project')
    root.mkdir(parents=True, exist_ok=True)
    enforce_limits(root)
    if shutil.disk_usage(root).free < min_free_bytes:
        raise OSError('insufficient free disk space for navigation diagnostics; free space before starting')
    metadata = dict(capture_pid=os.getpid(), source=dict(commit=git_output(project, 'rev-parse', 'HEAD'),
        dirty_paths=(git_output(project, 'status', '--short', '--untracked-files=normal') or '').splitlines()[:250]),
        launch=dict(argv=list(command), working_directory=str(project),
                    configured_image=os.environ.get('GO2_DIAGNOSTICS_IMAGE', 'go2-3d:edu')),
        host=dict(system=platform.system(), machine=platform.machine(), release=platform.release(),
                  python=platform.python_version()))
    session = create_session(root, metadata)
    store = EventStore(session)
    context = ContextCapture(project, session)
    capture = dict(status='starting', started_utc=utc_now(),
        started_beijing=datetime.now(BEIJING).isoformat(timespec='milliseconds'),
        interactive=os.isatty(0), requested_signal=None, forced_group_kill=False,
        log_policy=dict(canonical='bounded launcher stdout+stderr and incident rosout',
                        native_ros_files='not retained; container-local and ephemeral'))
    store.write_manifest(dict(context=context.initial(), capture=dict(capture)))
    warn('saving navigation evidence to '+str(session))
    environment = os.environ.copy()
    environment.update(GO2_DIAGNOSTICS_CAPTURE_ACTIVE='1', GO2_DIAGNOSTICS_SESSION=str(session),
                       GO2_DIAGNOSTICS_UID=str(os.getuid()), GO2_DIAGNOSTICS_GID=str(os.getgid()))
    child_pid, child = None, None
    requested_at, old_handlers = None, {}
    terminal_state, master, output, returncode = None, None, None, None
    writer, disk_worker, display_worker = None, None, None
    deaths = LaunchDeaths(store, capture, command)
    capture['disk_handoff'] = dict(max_buffer_bytes=8*1024*1024)
    io_warning = False
    selector = selectors.DefaultSelector()
    pending_input = bytearray()

    def save_capture():
        if writer:
            capture['logs'] = writer.summary()
        if disk_worker:
            capture['disk_handoff'] = disk_worker.summary()
            if disk_worker.error:
                capture['log_error'] = disk_worker.error
        if display_worker:
            capture['terminal_handoff'] = display_worker.summary()
        try:
            store.write_manifest(dict(capture=dict(capture), context=context.snapshot()))
        except (OSError, ValueError) as error:
            warn('cannot update capture manifest: '+str(error))

    def rotate():
        save_capture()
        try:
            limits = enforce_limits(root, keep_session=session)
            if limits.get('over_budget'):
                warn('protected diagnostic data exceed retention budget; export/free old records')
        except (OSError, ValueError) as error:
            warn('cannot enforce diagnostic retention: '+str(error))

    def finalize(value):
        context.refresh()
        capture.update(status='interrupted' if capture['requested_signal'] else
                       'completed' if value == 0 else 'failed', exit_code=value,
                       ended_utc=utc_now(), ended_beijing=datetime.now(BEIJING).isoformat(timespec='milliseconds'))
        save_capture()
        try:
            store.write_manifest(dict(ended_utc=capture['ended_utc'], exit_code=value))
        except (OSError, ValueError) as error:
            warn('cannot finalize session manifest: '+str(error))
        if value not in (0, 130, 143) and not capture['requested_signal'] and not deaths.fault_ids:
            try:
                store.record_fault(dict(code='process_exit', source='launcher',
                    message='Navigation launcher exited with code '+str(value), exit_code=value),
                    dict(capture=dict(capture), source=metadata['source']),
                    dict(launch_argv=list(command), log_files=[path.name for path in session.glob('launch.log*')]))
            except (OSError, ValueError) as error:
                warn('could not save launch failure event: '+str(error))

    def signal_child(number, frame=None):
        nonlocal requested_at
        if requested_at is None:
            requested_at = time.monotonic()
        capture['requested_signal'] = number
        if child_pid:
            try:
                os.killpg(child_pid, number)
            except ProcessLookupError:
                pass

    def copy_window(number=None, frame=None):
        if master is not None:
            try:
                size = fcntl.ioctl(0, termios.TIOCGWINSZ, b'\0'*8)
                fcntl.ioctl(master, termios.TIOCSWINSZ, size)
            except OSError:
                pass

    try:
        writer = RotatingBytes(session, max_log_bytes, log_files, rotate)
        interactive = capture['interactive']
        if interactive:
            terminal_state = termios.tcgetattr(0)
            child_pid, master = pty.fork()
            if child_pid == 0:
                try:
                    os.chdir(project)
                    os.execvpe(command[0], command, environment)
                except BaseException as error:
                    os.write(2, ('Cannot start navigation command: '+str(error)+'\n').encode())
                    os._exit(127)
            output = master
            os.set_blocking(master, False)
            copy_window()
            tty.setraw(0)
            selector.register(0, selectors.EVENT_READ, 'stdin')
        else:
            try:
                child = subprocess.Popen(command, cwd=str(project), env=environment,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as error:
                returncode = 127
                capture['spawn_error'] = str(error)
                warn('cannot start navigation command: '+str(error))
                finalize(returncode)
                return returncode
            child_pid, output = child.pid, child.stdout.fileno()
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[number] = signal.signal(number, signal_child)
        if interactive:
            old_handlers[signal.SIGWINCH] = signal.signal(signal.SIGWINCH, copy_window)
        selector.register(output, selectors.EVENT_READ, 'output')
        capture['status'] = 'running'

        def persist(data):
            writer.write(data)
            deaths.feed(data)

        def background_context():
            context.refresh()
            save_capture()

        def show(data):
            view = memoryview(data)
            while view:
                written = os.write(1, view)
                if written <= 0:
                    raise OSError('terminal output write made no progress')
                view = view[written:]

        # Start threads only after fork/exec: no inherited Python worker locks.
        disk_worker = AsyncWriter(persist, periodic=background_context, on_error=lambda error:
            warn('file capture stopped; terminal and navigation continue: '+error))
        display_worker = AsyncWriter(show, max_bytes=512*1024, name='diagnostic-terminal')
        disk_worker.start()
        display_worker.start()
        eof, child_exit_at = False, None
        while not eof or returncode is None:
            now = time.monotonic()
            if returncode is None:
                if child:
                    value = child.poll()
                    if value is not None:
                        returncode = value if value >= 0 else 128-value
                else:
                    waited, status = os.waitpid(child_pid, os.WNOHANG)
                    if waited:
                        returncode = (os.WEXITSTATUS(status) if os.WIFEXITED(status)
                                      else 128+os.WTERMSIG(status))
                if returncode is not None:
                    child_exit_at = now
            if requested_at is not None and now-requested_at > grace_seconds and returncode is None:
                capture['forced_group_kill'] = True
                capture['owned_container_stopped'] = stop_owned_container(session.name)
                try:
                    os.killpg(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if child_exit_at is not None and now-child_exit_at > 2.0 and not eof:
                capture['incomplete_output_drain'] = True
                try:
                    os.killpg(child_pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                eof = True
            for key, events in selector.select(0.1):
                if key.data == 'stdin':
                    try:
                        data = os.read(0, 65536)
                    except OSError:
                        data = b''
                    if data:
                        if b'\x03' in data:
                            # Raw terminal control bytes signal the child PTY,
                            # not this wrapper. Remember user intent for reports.
                            capture['requested_signal'] = signal.SIGINT
                            if requested_at is None:
                                requested_at = time.monotonic()
                            pending_input.clear()
                            data = data[data.index(b'\x03'):]
                        if returncode is None:
                            available = max(0, 65536-len(pending_input))
                            pending_input.extend(data[:available])
                            if len(data) > available:
                                capture['stdin_dropped_bytes'] = capture.get('stdin_dropped_bytes', 0)+len(data)-available
                            selector.modify(output, selectors.EVENT_READ | selectors.EVENT_WRITE, 'output')
                    else:
                        selector.unregister(0)
                        signal_child(signal.SIGHUP)
                else:
                    if master is not None and events & selectors.EVENT_WRITE and pending_input:
                        try:
                            written = os.write(master, pending_input)
                            del pending_input[:written]
                        except BlockingIOError:
                            pass
                        except OSError as error:
                            if error.errno != errno.EIO:
                                raise
                            pending_input.clear()
                        if not pending_input:
                            selector.modify(output, selectors.EVENT_READ, 'output')
                    if not events & selectors.EVENT_READ:
                        continue
                    try:
                        data = os.read(output, 65536)
                    except BlockingIOError:
                        continue
                    except OSError as error:
                        if master is not None and error.errno == errno.EIO:
                            data = b''
                        else:
                            raise
                    if not data:
                        eof = True
                        selector.unregister(output)
                        continue
                    disk_ok = disk_worker.enqueue(ReceivedChunk(
                        data, bool(capture.get('requested_signal')), utc_now()))
                    display_worker.enqueue(data)
                    if not disk_ok and not io_warning:
                        io_warning = True
                        display_worker.enqueue(b'\r\nNavigation diagnostics: disk capture queue full/unavailable; '
                            b'bytes dropped, navigation relay continues. See capture manifest.\r\n')
        def finish_disk():
            try:
                finalize(returncode)
            finally:
                writer.close()
        disk_worker.finish(finish_disk)
        display_worker.finish()
        if not disk_worker.done.wait(flush_seconds):
            warn('diagnostic flush incomplete after bounded shutdown wait; pending evidence may be missing')
        if not display_worker.done.wait(min(flush_seconds, 0.5)):
            warn('terminal output drain incomplete; capture manifest records handoff limits')
        return returncode
    finally:
        if returncode is None and child_pid:
            try:
                os.killpg(child_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                if child:
                    child.wait(timeout=3)
                else:
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        if os.waitpid(child_pid, os.WNOHANG)[0]:
                            break
                        time.sleep(0.05)
                    else:
                        os.killpg(child_pid, signal.SIGKILL)
                        os.waitpid(child_pid, 0)
            except (ProcessLookupError, ChildProcessError):
                pass
            except subprocess.TimeoutExpired:
                os.killpg(child_pid, signal.SIGKILL)
                child.wait()
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
        if terminal_state is not None:
            termios.tcsetattr(0, termios.TCSADRAIN, terminal_state)
        selector.close()
        if master is not None:
            os.close(master)
        if child and child.stdout:
            child.stdout.close()
        if writer and (disk_worker is None or disk_worker.done.is_set()):
            writer.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('a navigation command after -- is required')
    try:
        return run_capture(args.project, command)
    except (OSError, ValueError) as error:
        warn('capture startup failed before navigation: '+str(error))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
