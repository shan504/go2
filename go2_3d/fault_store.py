"""Bounded, atomic navigation evidence storage; no ROS or control operations.

Only directories carrying our session marker participate in automatic retention.
The container normally sees one session at /diagnostics; only the host supplies
an explicit diagnostics root for global retention.
"""
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import fcntl
import gzip
import io
import json
import os
from pathlib import Path
import re
import shutil
import tarfile
import tempfile
import uuid
import unicodedata

SCHEMA = 'go2.navigation.diagnostics.session.v1'
FAULT_SCHEMA = 'go2.navigation.diagnostics.fault.v1'
EXPORT_SCHEMA = 'go2.navigation.diagnostics.export.v1'
MARKER = '.go2-diagnostic-session'
MAX_BYTES = 512 * 1024 * 1024
SESSION_BYTES = 64 * 1024 * 1024
ARTIFACT_BYTES = 8 * 1024 * 1024
RESERVE_BYTES = 16 * 1024 * 1024
MAX_SESSIONS = 100
MAX_FAULTS = 200
_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,220}$')


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def client_time(value):
    if value is None:
        return '未提供'
    try:
        parsed = datetime.fromisoformat(str(value).replace('Z', '+00:00'))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        local = parsed.astimezone(timezone(timedelta(hours=8), 'Asia/Shanghai'))
        return local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]+' UTC+08:00（北京时间）'
    except (ValueError, OverflowError):
        return '时间无法解析；原值: '+str(value)


def _owner(path):
    """Give only newly-created recorder outputs back to the launching host user."""
    if os.geteuid() != 0:
        return
    uid, gid = os.environ.get('GO2_DIAGNOSTICS_UID'), os.environ.get('GO2_DIAGNOSTICS_GID')
    if uid is not None and gid is not None and uid.isdecimal() and gid.isdecimal():
        uid, gid = int(uid), int(gid)
        if uid <= 2**31-1 and gid <= 2**31-1:
            os.chown(path, uid, gid, follow_symlinks=False)


def _directory(path, create=False):
    path = Path(path).absolute()
    for part in (path, *path.parents):
        if part.is_symlink():
            raise ValueError(f'Refusing symlink diagnostic path: {part}')
    if create and not path.exists():
        path.mkdir(parents=True, mode=0o755)
        _owner(path)
    if not path.is_dir():
        raise ValueError(f'Diagnostic directory unavailable: {path}')
    return path


def _file(path):
    path = Path(path)
    _directory(path.parent)
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise ValueError(f'Refusing nonregular diagnostic file: {path}')
    return path


@contextmanager
def _lock(directory, name='.store.lock'):
    directory = _directory(directory)
    path = _file(directory/name)
    existed = path.exists()
    descriptor = os.open(path, os.O_CREAT | os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0), 0o644)
    if not existed:
        _owner(path)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        yield
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _sync_directory(directory):
    descriptor = os.open(_directory(directory), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def safe_display(text):
    # Raw evidence remains unchanged; reports must not execute terminal control
    # sequences or hide text using invisible directional formatting characters.
    return ''.join(character if character in '\n\t' or not unicodedata.category(character).startswith('C')
                   else ('\\x%02x' % ord(character) if ord(character) < 256 else
                         '\\u%04x' % ord(character)) for character in str(text))


def _write(path, data, max_bytes=ARTIFACT_BYTES):
    path = _file(path)
    if len(data) > max_bytes:
        raise ValueError(f'Evidence exceeds {max_bytes} byte artifact limit: {path.name}')
    if shutil.disk_usage(path.parent).free < len(data)+RESERVE_BYTES:
        raise OSError('Insufficient diagnostic storage reserve; existing evidence preserved')
    descriptor, temporary = tempfile.mkstemp(prefix='.'+path.name+'.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o644)
        _owner(temporary)
        os.replace(temporary, path)
        _sync_directory(path.parent)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_json(path, value, max_bytes=ARTIFACT_BYTES):
    _write(path, (json.dumps(value, ensure_ascii=False, allow_nan=False,
                            indent=2)+'\n').encode('utf-8'), max_bytes)


def read_json(path):
    path = _file(path)
    if path.stat().st_size > ARTIFACT_BYTES:
        raise ValueError('Diagnostic JSON exceeds bounded artifact size')
    return json.loads(path.read_text())


def _owned_session(path):
    try:
        path = _directory(path)
        marker = _file(path/MARKER)
        return marker.read_text() == '1\n' and read_json(path/'manifest.json').get('schema') == SCHEMA
    except (OSError, ValueError, json.JSONDecodeError):
        return False


def merge_manifest(session_dir, fields):
    session = _directory(session_dir)
    if not _owned_session(session):
        raise ValueError('Refusing an unmarked diagnostic session')
    with _lock(session):
        manifest = read_json(session/'manifest.json')
        reserved = {'schema': SCHEMA, 'session_id': _name(manifest['session_id'])}
        manifest.update(fields)
        manifest.update(reserved)
        atomic_json(session/'manifest.json', manifest)
        return manifest


def _process_identity(pid):
    try:
        text = (Path('/proc')/str(pid)/'stat').read_text()
        return text.rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def _boot_identity():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return None


def create_session(root, manifest=None):
    root = _directory(root, create=True)
    if shutil.disk_usage(root).free < RESERVE_BYTES:
        raise OSError('Insufficient space to start diagnostic recording')
    enforce_limits(root)
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+uuid.uuid4().hex[:8]
    session = root/name
    session.mkdir(mode=0o755)
    _owner(session)
    _write(session/MARKER, b'1\n')
    value = dict(manifest or {})
    if type(value.get('capture_pid')) is int and value['capture_pid'] > 0:
        value.setdefault('capture_process_start', _process_identity(value['capture_pid']))
        value.setdefault('capture_boot_id', _boot_identity())
    value.update(schema=SCHEMA, session_id=name, created_utc=utc_now())
    atomic_json(session/'manifest.json', value)
    _sync_directory(root)
    return session


def _capture_log_tail(session, directory, phase):
    metadata = dict(available=False, source='launch.log', captured_utc=utc_now(),
                    strategy='chronological bounded tail across launch.log.4 through launch.log',
                    sources=[], coverage_gaps=[])
    limit, remaining = 2*1024*1024, 2*1024*1024
    pieces, seen, existing = [], set(), []
    try:
        for index in range(5):  # Newest first, then concatenate oldest first.
            name = 'launch.log'+('.'+str(index) if index else '')
            source = session/name
            try:
                source = _file(source)
                with source.open('rb') as stream:
                    info = os.fstat(stream.fileno())
                    existing.append(index)
                    identity = (info.st_dev, info.st_ino)
                    if identity in seen:
                        metadata['coverage_gaps'].append(name+': duplicate inode during rotation; skipped')
                        continue
                    seen.add(identity)
                    kept = min(info.st_size, remaining)
                    offset = max(0, info.st_size-kept)
                    stream.seek(offset)
                    data = stream.read(kept)
                    pieces.append(data)
                    metadata['sources'].append(dict(file=name, source_bytes=info.st_size,
                        retained_bytes=len(data), source_offset=offset))
                    remaining -= len(data)
            except FileNotFoundError:
                continue
            except (OSError, ValueError) as error:
                metadata['coverage_gaps'].append(name+': '+str(error))
        if not existing:
            metadata['error'] = 'No readable launch log files available at capture time'
            return metadata
        for index in range(max(existing)+1):
            if index not in existing:
                metadata['coverage_gaps'].append('launch.log'+('.'+str(index) if index else '')+': missing rotation segment')
        data = b''.join(reversed(pieces))
        name = 'launch-'+phase+'.log.gz'
        _write(directory/name, gzip.compress(data, compresslevel=3))
        metadata['sources'].reverse()
        metadata.update(available=True, file=name,
                        source_bytes=sum(item['source_bytes'] for item in metadata['sources']),
                        retained_bytes=len(data), byte_limit=limit,
                        truncated=any(item['source_offset'] > 0 for item in metadata['sources']))
    except (OSError, ValueError) as error:
        metadata['error'] = str(error)
    return metadata


def _size(path):
    total = 0
    for parent, directories, files in os.walk(path, followlinks=False):
        directories[:] = [name for name in directories if not (Path(parent)/name).is_symlink()]
        for name in files:
            item = Path(parent)/name
            if not item.is_symlink():
                try:
                    total += item.stat().st_size
                except FileNotFoundError:
                    pass
    return total


def _active(session):
    manifest = read_json(session/'manifest.json')
    capture = manifest.get('capture') if isinstance(manifest.get('capture'), dict) else {}
    closed = any(record.get(key) is not None for record in (manifest, capture)
                 for key in ('ended_utc', 'finished_utc', 'exit_code'))
    liveness = manifest.get('recorder_liveness', {})
    if not closed and liveness.get('active') is True:
        try:
            heartbeat = datetime.fromisoformat(liveness['heartbeat_utc'].replace('Z', '+00:00'))
            if 0 <= (datetime.now(timezone.utc)-heartbeat).total_seconds() <= 15:
                return True  # Host capture died, but the recorder is still writing.
        except (KeyError, TypeError, ValueError):
            pass
    pid = manifest.get('capture_pid', capture.get('pid', capture.get('capture_pid')))
    if type(pid) is int and pid > 0:
        # A live capture may still be atomically writing its final fault after
        # it has recorded the child's exit. Protect it until its own exit.
        boot = manifest.get('capture_boot_id')
        expected = manifest.get('capture_process_start')
        if boot is not None and _boot_identity() not in (None, boot):
            return False
        actual = _process_identity(pid)
        if expected is not None and actual is not None and actual != expected:
            return False  # PID reused by an unrelated process.
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
    if closed:
        return False
    # Do not remove a newly created session while the host sets its PID.
    return __import__('time').time()-session.stat().st_mtime < 60


def _faults_in(session):
    directory = session/'faults'
    if not directory.exists() or directory.is_symlink():
        return []
    result = []
    for item in directory.iterdir():
        if item.is_dir() and not item.is_symlink():
            try:
                summary = read_json(item/'summary.json')
                if summary.get('schema') == FAULT_SCHEMA and summary.get('fault_id') == item.name:
                    result.append((item, summary))
            except (ValueError, OSError, json.JSONDecodeError):
                pass
    return sorted(result, key=lambda item: item[1].get('recorded_utc', ''))


def _owned_export(path):
    try:
        manifest = read_json(path.with_name(path.name+'.json'))
        return manifest.get('schema') == EXPORT_SCHEMA and manifest.get('archive') == path.name
    except (ValueError, OSError, json.JSONDecodeError):
        return False


def enforce_limits(root, keep_session=None, *, max_bytes=MAX_BYTES,
                   max_sessions=MAX_SESSIONS, max_faults=MAX_FAULTS):
    """Prune only marked, inactive sessions; never an active capture or unknown file.

    Completed exports share the byte budget. If protected active data alone is
    over budget, return over_budget instead of deleting it or user files.
    """
    root = _directory(root, create=True)
    removed = []
    with _lock(root, '.retention.lock'):
        sessions = sorted((p for p in root.iterdir() if p.is_dir() and _owned_session(p)),
                          key=lambda p: p.name)
        export_dir = root/'exports'
        exports = ([] if not export_dir.is_dir() or export_dir.is_symlink() else
                   sorted((p for p in export_dir.iterdir() if not p.is_symlink() and p.is_file()
                           and re.fullmatch(r'fault-[A-Za-z0-9_.-]+\.tar\.gz', p.name)
                           and _owned_export(p)),
                          key=lambda p: p.stat().st_mtime))
        def totals():
            return (sum(_size(p) for p in sessions)+sum(p.stat().st_size for p in exports),
                    sum(len(_faults_in(p)) for p in sessions))
        size, faults = totals()
        for session in list(sessions):
            if size <= max_bytes and len(sessions) <= max_sessions and faults <= max_faults:
                break
            if (keep_session is not None and session == Path(keep_session).absolute()) or _active(session):
                continue
            shutil.rmtree(session)
            sessions.remove(session)
            removed.append(session.name)
            size, faults = totals()
        for export in list(exports):
            if size <= max_bytes and len(exports) <= 10:
                break
            size -= export.stat().st_size
            export.unlink()
            _file(export.with_name(export.name+'.json')).unlink(missing_ok=True)
            exports.remove(export)
            removed.append('exports/'+export.name)
        return dict(removed=removed, bytes=size, sessions=len(sessions), faults=faults,
                    over_budget=size > max_bytes or len(sessions) > max_sessions or faults > max_faults)


def normalize_trigger(trigger):
    if isinstance(trigger, str):
        trigger = {'message': trigger}
    trigger = dict(trigger)
    message = str(trigger.get('message', ''))
    if not trigger.get('code') and trigger.get('kind'):
        trigger['code'] = str(trigger['kind'])
    if not trigger.get('code'):
        text = message.lower()
        code = ('progress_timeout' if 'failed to make progress' in text else
                'planner_failed' if any(s in text for s in ('failed to create plan', 'failed to generate a valid path')) else
                'localization_lost' if any(s in text for s in ('localization lost', 'odometry_timeout', 'localization_reported_invalid')) else
                'tf_error' if any(s in text for s in ('extrapolation', 'transform', ' tf ')) else
                'manual_stop' if trigger.get('manual') else 'navigation_failure')
        trigger['code'] = code
    trigger.setdefault('message', message)
    return trigger


def render_report(summary, evidence=None):
    trigger = summary.get('trigger', {})
    snapshot = summary.get('snapshot') or {}
    code = trigger.get('code', 'unknown')
    labels = {'progress_timeout': '控制器进度超时', 'planner_failed': '全局规划失败',
              'planner_failure': '全局规划失败', 'localization_lost': '定位丢失或失效',
              'tf_error': '坐标变换错误', 'manual_stop': '人工停止或取消',
              'manual_cancel': '人工停止或取消', 'process_exit': '导航进程异常退出',
              'navigation_failure': '导航失败（需结合原始证据）',
              'controller_no_valid_trajectory': '控制器没有有效轨迹',
              'controller_alignment_error': '路径对准控制错误',
              'component_error': '组件错误（需查看原始日志）',
              'localization_loss': '定位丢失或失效',
              'localization_heartbeat_stale': '定位有效性心跳超时',
              'startup_failed': '导航启动失败', 'navigation_aborted': '导航任务中止',
              'goal_rejected': '目标被拒绝', 'action_failure': '导航 action 失败',
              'no_motion_observed': '观察到命令与位移不一致（现象，不是唯一根因）',
              'manual_initialpose': '人工重新设置初始位姿', 'manual_snapshot': '人工诊断快照'}
    lines = ['导航故障记录', '记录 ID: '+summary.get('fault_id', '?'),
             '记录时间: '+client_time(summary.get('recorded_utc')),
             'UTC 原始时间: '+summary.get('recorded_utc', '?'),
             '观察到的触发: '+labels.get(code, code),
             '触发来源: '+str(trigger.get('source', '未提供')),
             '触发观察时间: '+client_time(trigger.get('observed_at', trigger.get('time_utc', summary.get('recorded_utc')))),
             '最初触发信息: '+str(trigger.get('message', '未提供')),
             '', '最新现场摘要（最初证据保留在 evidence.initial_snapshot）:']
    route = snapshot.get('route')
    if route:
        lines.append('- 路线状态: '+json.dumps(route, ensure_ascii=False))
    for name, value in snapshot.get('commands', {}).items():
        lines.append('- '+name+': '+json.dumps(value, ensure_ascii=False))
    if snapshot.get('odometry'):
        lines.append('- 实际里程计: '+json.dumps(snapshot['odometry'], ensure_ascii=False))
    for key in ('local_costmap', 'plan', 'tf', 'lifecycle', 'latest'):
        if snapshot.get(key) is not None:
            lines.append('- '+key+': '+json.dumps(snapshot[key], ensure_ascii=False))
    if evidence:
        lines.append('- 证据项目: '+', '.join(sorted(map(str, evidence))))
        context = evidence.get('latest_context') or evidence.get('context') or {}
        if isinstance(context, dict):
            for key in ('transforms', 'lifecycle', 'coverage'):
                if context.get(key) is not None:
                    lines.append('- 最近 '+key+': '+json.dumps(context[key], ensure_ascii=False))
        for key in ('omissions', 'truncated', 'capture_status', 'post_capture_status', 'post_capture',
                    'final_action', 'final_status', 'trigger_timeline', 'raw_log_before', 'raw_log_after', 'errors'):
            if evidence.get(key) is not None:
                lines.append('- '+key+': '+json.dumps(evidence[key], ensure_ascii=False))
    lines += ['', '证据边界:',
              '- 这是已观察到的触发和数据，不代表唯一物理根因已被确认。',
              '- 定位有效不等于底盘有位移；检查控制器命令、平滑后命令、桥状态和真实里程计。',
              '- 消息缺失/过期、窗口截断和采集失败均需保留，不能解释为正常。',
              '- 人工停止/取消与自动故障应分开分析；本记录不会重试目标或启用运动。',
              '', '文件: summary.json、evidence.json、report.txt；导出包还包含启动日志、配置及会话清单。']
    return safe_display('\n'.join(lines)+'\n')


class EventStore:
    def __init__(self, session_dir, *, root_dir=None, max_bytes=SESSION_BYTES,
                 max_sessions=MAX_SESSIONS, max_faults=MAX_FAULTS):
        self.session = _directory(session_dir, create=True)
        if not _owned_session(self.session):
            if any(self.session.iterdir()):
                raise ValueError('Refusing to write into an unmarked nonempty session')
            _write(self.session/MARKER, b'1\n')
            atomic_json(self.session/'manifest.json', dict(schema=SCHEMA,
                session_id=self.session.name, created_utc=utc_now()))
        self.session_id = _name(read_json(self.session/'manifest.json')['session_id'])
        self.root = _directory(root_dir) if root_dir is not None else None
        if self.root is not None and self.session.parent != self.root:
            raise ValueError('Session must be a direct child of diagnostic root')
        self.max_bytes, self.max_sessions, self.max_faults = max_bytes, max_sessions, max_faults
        for name in ('faults',):
            directory = self.session/name
            if not directory.exists():
                directory.mkdir(mode=0o755)
                _owner(directory)
            _directory(directory)

    def write_manifest(self, fields):
        return merge_manifest(self.session, fields)

    def append_summary(self, snapshot):
        data = (json.dumps(snapshot, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n').encode()
        if len(data) > 128*1024:
            raise ValueError('Summary exceeds bounded line size')
        with _lock(self.session):
            path = _file(self.session/'summaries.jsonl')
            if path.exists() and path.stat().st_size+len(data) > 2*1024*1024:
                for index in (2, 1):
                    source = _file(self.session/('summaries.jsonl' if index == 1 else 'summaries.jsonl.1'))
                    target = _file(self.session/('summaries.jsonl.'+str(index)))
                    if source.exists():
                        os.replace(source, target)
                _file(self.session/'summaries.jsonl.3').unlink(missing_ok=True)
            if shutil.disk_usage(self.session).free < RESERVE_BYTES+len(data):
                raise OSError('Insufficient diagnostic storage reserve')
            new = not path.exists()
            descriptor = os.open(path, os.O_CREAT | os.O_APPEND | os.O_WRONLY | getattr(os, 'O_NOFOLLOW', 0), 0o644)
            with os.fdopen(descriptor, 'ab') as output:
                output.write(data)
                output.flush()
                os.fsync(output.fileno())
            _sync_directory(self.session)
            if new:
                _owner(path)
            self._retention()
        self._global_retention()

    def _retention(self, keep_fault=None):
        """Session-local retention; caller holds session lock, never root lock."""
        faults = _faults_in(self.session)
        if keep_fault is None and faults:
            keep_fault = faults[-1][0].name  # Periodics must not delete the sole latest fault.
        removed = []
        while faults and (_size(self.session) > self.max_bytes or len(faults) > self.max_faults):
            candidate = next((item for item in faults if item[0].name != keep_fault), None)
            if candidate is None:
                break
            shutil.rmtree(candidate[0])
            removed.append(candidate[0].name)
            faults.remove(candidate)
        size = _size(self.session)
        manifest = read_json(self.session/'manifest.json')
        previous = manifest.get('fault_store', {})
        manifest['fault_store'] = dict(session_bytes=size, target_bytes=self.max_bytes,
            over_budget=size > self.max_bytes or len(faults) > self.max_faults,
            retained_faults=len(faults), preserved_fault=keep_fault,
            removed_total=previous.get('removed_total', 0)+len(removed),
            removed_recent=(previous.get('removed_recent', [])+removed)[-16:])
        atomic_json(self.session/'manifest.json', manifest)

    def _global_retention(self):
        # Export/host cleanup acquire root -> session. Never acquire root while
        # holding session, otherwise concurrent export and update can deadlock.
        if self.root is not None:
            enforce_limits(self.root, keep_session=self.session,
                           max_sessions=self.max_sessions, max_faults=self.max_faults)

    def heartbeat(self, active=True):
        return self.write_manifest({'recorder_liveness':
                                    dict(active=bool(active), heartbeat_utc=utc_now())})

    def record_fault(self, trigger, snapshot, evidence=None, *, now_utc=None):
        trigger = normalize_trigger(trigger)
        timestamp = str(now_utc or utc_now())
        local_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+uuid.uuid4().hex[:8]
        fault_id = self.session_id+'--'+local_id
        summary = dict(schema=FAULT_SCHEMA, fault_id=fault_id, session_id=self.session_id,
                       recorded_utc=timestamp, trigger=trigger, snapshot=snapshot)
        evidence = dict(evidence or {})
        # Validate before publishing a fault directory, never emit partial JSON.
        for value in (summary, evidence):
            if len(json.dumps(value, ensure_ascii=False, allow_nan=False).encode()) > ARTIFACT_BYTES:
                raise ValueError('Fault artifact exceeds bounded size')
        with _lock(self.session):
            token = trigger.get('recorder_token')
            if isinstance(token, str) and token:
                for existing_path, existing in _faults_in(self.session):
                    if existing.get('trigger', {}).get('recorder_token') == token:
                        return existing['fault_id']
            directory = self.session/'faults'/fault_id
            temporary = self.session/'faults'/('.pending-'+uuid.uuid4().hex)
            temporary.mkdir(mode=0o755)
            _owner(temporary)
            try:
                evidence['raw_log_before'] = _capture_log_tail(self.session, temporary, 'before')
                atomic_json(temporary/'evidence.json', evidence)
                atomic_json(temporary/'summary.json', summary)
                _write(temporary/'report.txt', render_report(summary, evidence).encode())
                os.rename(temporary, directory)
                _sync_directory(directory.parent)
            finally:
                if temporary.exists():
                    shutil.rmtree(temporary)
            self._retention(fault_id)
        self._global_retention()
        return fault_id

    def update_fault(self, fault_id, snapshot=None, evidence=None):
        directory = self.session/'faults'/_name(fault_id)
        _directory(directory)
        with _lock(self.session):
            summary = read_json(directory/'summary.json')
            if summary.get('fault_id') != fault_id:
                raise ValueError('Fault identity mismatch')
            if snapshot is not None:
                summary['snapshot'] = snapshot
            data = read_json(directory/'evidence.json')
            if evidence:
                for key in ('initial_snapshot', 'events_before', 'raw_log_before'):
                    if key in evidence and key in data and evidence[key] != data[key]:
                        raise ValueError('Initial fault evidence is immutable: '+key)
                data.update(evidence)
            summary['updated_utc'] = utc_now()
            data['raw_log_after'] = _capture_log_tail(self.session, directory, 'after')
            atomic_json(directory/'evidence.json', data)
            atomic_json(directory/'summary.json', summary)
            _write(directory/'report.txt', render_report(summary, data).encode())
            self._retention(fault_id)
        self._global_retention()
        return fault_id


def _name(value):
    if not isinstance(value, str) or not _NAME.fullmatch(value) or value in ('.', '..'):
        raise ValueError('Invalid diagnostic ID')
    return value


def list_faults(root):
    root = _directory(root)
    result = []
    for session in root.iterdir():
        if session.is_dir() and _owned_session(session):
            result.extend(dict(summary, session_path=str(session), fault_path=str(path))
                          for path, summary in _faults_in(session))
    return sorted(result, key=lambda item: (item.get('recorded_utc', ''), item['fault_id']), reverse=True)


def find_fault(root, selector='latest'):
    if selector != 'latest':
        _name(selector)
    candidates = list_faults(root)
    if selector == 'latest':
        actual = [item for item in candidates if not item.get('trigger', {}).get('manual') and
                  item.get('trigger', {}).get('severity') != 'info' and
                  item.get('trigger', {}).get('code') not in
                  ('manual_stop', 'manual_cancel', 'manual_snapshot', 'snapshot')]
        result = next(iter(actual or candidates), None)
    else:
        result = next((item for item in candidates if item['fault_id'] == selector), None)
    if result is None:
        raise ValueError('未找到故障记录；先确认已使用新版本启动导航并发生过故障')
    return result


def export_fault(root, selector='latest'):
    root = _directory(root)
    with _lock(root, '.retention.lock'):
        selected = find_fault(root, selector)
        session, fault = Path(selected['session_path']), Path(selected['fault_path'])
        with _lock(session):
            return _export_locked(root, selected, session, fault)


def _export_locked(root, selected, session, fault):
    destination = root/'exports'
    _directory(destination, create=True)
    if shutil.disk_usage(destination).free < SESSION_BYTES+RESERVE_BYTES:
        raise OSError('导出空间不足，需预留最多 64MiB 诊断包及 16MiB 磁盘余量；原记录保留')
    name = 'fault-'+selected['fault_id']+'-'+uuid.uuid4().hex[:6]+'.tar.gz'
    output = destination/name
    descriptor, temporary = tempfile.mkstemp(prefix='.export-', dir=destination)
    omissions = []
    total = 0
    try:
        with os.fdopen(descriptor, 'wb') as raw:
            with tarfile.open(fileobj=raw, mode='w:gz') as archive:
                def add_file(path, relative, limit=ARTIFACT_BYTES):
                    nonlocal total
                    try:
                        path = _file(path)
                        if not path.exists():
                            omissions.append(relative+': unavailable')
                            return
                        size = path.stat().st_size
                        if total+min(size, limit) > SESSION_BYTES:
                            omissions.append(relative+': export byte budget reached')
                            return
                        with path.open('rb') as source:
                            if size > limit:
                                source.seek(-limit, os.SEEK_END)
                                omissions.append(relative+': truncated to trailing '+str(limit)+' bytes')
                            data = source.read(limit)
                        item = tarfile.TarInfo(relative)
                        item.size, item.mode = len(data), 0o644
                        archive.addfile(item, io.BytesIO(data))
                        total += len(data)
                    except (OSError, ValueError) as error:
                        omissions.append(relative+': '+str(error))
                for parent, directories, files in os.walk(fault, followlinks=False):
                    directories[:] = [d for d in directories if not (Path(parent)/d).is_symlink()]
                    for filename in sorted(files):
                        path = Path(parent)/filename
                        add_file(path, 'fault/'+str(path.relative_to(fault)))
                add_file(session/'manifest.json', 'session/manifest.json')
                for pattern in ('launch.log*', 'summaries.jsonl*'):
                    for path in sorted(session.glob(pattern)):
                        add_file(path, 'session/'+path.name)
                for folder in ('context', 'ros'):
                    directory = session/folder
                    if directory.exists() and not directory.is_symlink():
                        for parent, directories, files in os.walk(directory, followlinks=False):
                            directories[:] = [d for d in directories if not (Path(parent)/d).is_symlink()]
                            for filename in sorted(files):
                                path = Path(parent)/filename
                                add_file(path, 'session/'+str(path.relative_to(session)))
                metadata = dict(exported_utc=utc_now(), fault_id=selected['fault_id'],
                                selected_fault_snapshot_consistent=True,
                                live_session_logs_may_change=True, live_session_context_may_change=True,
                                omissions=omissions,
                                data_bytes=total, max_data_bytes=SESSION_BYTES)
                data = json.dumps(metadata, ensure_ascii=False, indent=2).encode()
                item = tarfile.TarInfo('export_manifest.json')
                item.size, item.mode = len(data), 0o644
                archive.addfile(item, io.BytesIO(data))
            raw.flush()
            os.fsync(raw.fileno())
        os.chmod(temporary, 0o644)
        _owner(temporary)
        os.replace(temporary, output)
        _sync_directory(destination)
        atomic_json(output.with_name(output.name+'.json'),
                    dict(schema=EXPORT_SCHEMA, archive=output.name, fault_id=selected['fault_id'],
                         created_utc=utc_now()))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return output


def load_fault(root, selector='latest'):
    root = _directory(root)
    with _lock(root, '.retention.lock'):
        selected = find_fault(root, selector)
        with _lock(Path(selected['session_path'])):
            summary = read_json(Path(selected['fault_path'])/'summary.json')
            summary.update(session_path=selected['session_path'], fault_path=selected['fault_path'])
            evidence = read_json(Path(selected['fault_path'])/'evidence.json')
            manifest = read_json(Path(selected['session_path'])/'manifest.json')
            return summary, evidence, manifest
