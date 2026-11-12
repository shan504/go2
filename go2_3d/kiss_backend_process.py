"""Private-process boundary around the optional native KISS-SLAM backend.

The ROS mapper retains its own Python/Open3D. Only NumPy arrays and graph
proposals cross a private inherited socket; no ROS/DDS or motion exists in the
child. Native crashes, lost acknowledgements and timeouts fail closed. Pickle
is used ONLY between this process and its explicitly launched trusted child,
never for files, network clients or user-supplied serialized objects.
"""
from dataclasses import asdict, dataclass
import os
from pathlib import Path
import pickle
import socket
import struct
import subprocess
import sys
import uuid

import numpy as np

from kiss_backend import KissConfig, _pose

_MAX_MESSAGE = 512 * 1024 * 1024


def _read_exact(channel, count):
    pieces = []
    while count:
        data = channel.recv(min(count, 1024*1024))
        if not data:
            raise EOFError('KISS child closed its private channel')
        pieces.append(data)
        count -= len(data)
    return b''.join(pieces)


def _receive(channel):
    size = struct.unpack('!Q', _read_exact(channel, 8))[0]
    if size > _MAX_MESSAGE:
        raise ValueError('KISS IPC message exceeds 512 MiB')
    return pickle.loads(_read_exact(channel, size))


def _send(channel, message):
    data = pickle.dumps(message, protocol=4)
    if len(data) > _MAX_MESSAGE:
        raise ValueError('KISS IPC message exceeds 512 MiB')
    channel.sendall(struct.pack('!Q', len(data)))
    channel.sendall(data)


@dataclass(frozen=True)
class _Proposal:
    poses: tuple
    original_poses: tuple
    loop_edges: tuple
    status: dict
    token: str
    owner: object


class KissBackendProcess:
    def __init__(self, config=None, python=None, timeout=60.0):
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('KISS process timeout must be finite and positive')
        self._channel = None
        self._process = None
        self._failure = ''
        self._count = 0
        self._status = {}
        self._proposal = None
        self._owner = object()
        self.python = os.path.abspath(os.path.expanduser(python or sys.executable))
        if not os.path.isfile(self.python) or not os.access(self.python, os.X_OK):
            raise ValueError('KISS interpreter does not exist or is not executable: '+self.python)
        parent, child = socket.socketpair()
        self._channel = parent
        parent.settimeout(float(timeout))
        environment = dict(os.environ)
        # KISS needs no ROS Python packages. Do not let inherited ROS/Conda
        # paths override the private venv's NumPy or native dependencies.
        environment.pop('PYTHONHOME', None)
        environment['PYTHONPATH'] = str(Path(__file__).resolve().parent)
        environment.setdefault('OMP_NUM_THREADS', '2')
        environment.setdefault('OPENBLAS_NUM_THREADS', '1')
        try:
            self._process = subprocess.Popen(
                [self.python, str(Path(__file__).resolve()), '--serve', str(child.fileno())],
                pass_fds=(child.fileno(),), env=environment,
                stdin=subprocess.DEVNULL, stdout=sys.stderr, stderr=None)
            child.close()
            self._rpc('init', config=asdict(config or KissConfig()))
        except BaseException:
            child.close()
            self.close()
            raise

    @property
    def observation_count(self):
        return self._count

    @property
    def status(self):
        result = dict(self._status)
        result['isolation'] = 'private_process'
        if self._failure:
            result.update(state='error', error=self._failure)
        return result

    def _rpc(self, command, **arguments):
        if self._failure:
            raise RuntimeError(self._failure)
        channel = self._channel
        if channel is None:
            raise RuntimeError('KISS backend process is closed')
        try:
            _send(channel, dict(command=command, **arguments))
            reply = _receive(channel)
            if not isinstance(reply, dict) or type(reply.get('ok')) is not bool:
                raise ValueError('Malformed KISS response envelope')
            if not reply['ok']:
                raise RuntimeError(reply.get('error', 'KISS backend failed'))
            if (type(reply.get('observation_count')) is not int or reply['observation_count'] < 0 or
                    not isinstance(reply.get('status'), dict)):
                raise ValueError('Malformed KISS response state')
            self._count = reply['observation_count']
            self._status = reply['status']
            return reply.get('value')
        except Exception as error:
            # An append/apply ACK may have been lost. Never silently restart
            # the native graph or claim a successful map revision in that case.
            self._fail(error)

    def _fail(self, error):
        self._failure = ('KISS backend process failed; no new live map revision was acknowledged: '
                         f'{type(error).__name__}: {error}. Restart/replay this session.')
        self.close()
        raise RuntimeError(self._failure) from error

    def append(self, points, pose):
        result = self._rpc('append', points=np.asarray(points), pose=np.asarray(pose))
        self._proposal = None
        return result

    def flush(self):
        return self._rpc('flush')

    def detect_and_optimize(self):
        value = self._rpc('detect')
        if value is None:
            self._proposal = None
            return None
        try:
            if (not isinstance(value, dict) or not isinstance(value.get('status'), dict) or
                    not isinstance(value.get('token'), str) or not value['token'] or
                    not isinstance(value.get('loop_edges'), (list, tuple))):
                raise ValueError('Malformed KISS optimization proposal')
            for key in ('poses', 'original_poses'):
                if len(value[key]) != self._count or not self._count:
                    raise ValueError('KISS proposal does not cover the acknowledged observations')
                value[key] = tuple(_pose(pose) for pose in value[key])
            self._proposal = _Proposal(value['poses'], value['original_poses'],
                                       tuple(value['loop_edges']), value['status'],
                                       value['token'], self._owner)
        except Exception as error:
            self._fail(error)
        return self._proposal

    def apply(self, proposal):
        if (not isinstance(proposal, _Proposal) or proposal.owner is not self._owner or
                self._proposal is None or proposal.token != self._proposal.token):
            raise ValueError('foreign or stale KISS process proposal')
        self._rpc('apply', token=proposal.token)
        self._proposal = None

    def close(self):
        process = self._process
        channel = self._channel
        self._process, self._channel = None, None
        if channel is not None:
            try:
                channel.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            channel.close()
        if process is not None:
            if process.poll() is None:
                # Closing the socket terminates an idle child. A stalled native
                # call cannot block ROS shutdown indefinitely.
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def _serve(fd):
    from kiss_backend import KissBackend
    backend, proposal, token = None, None, None
    channel = socket.socket(fileno=fd)
    try:
        while True:
            try:
                request = _receive(channel)
            except EOFError:
                return
            try:
                command = request['command']
                value = None
                if command == 'init' and backend is None:
                    backend = KissBackend(KissConfig(**request['config']))
                elif backend is None:
                    raise ValueError('KISS backend has not been initialized')
                elif command == 'append':
                    value = backend.append(request['points'], request['pose'])
                    proposal, token = None, None
                elif command == 'flush':
                    value = backend.flush()
                elif command == 'detect':
                    found = backend.detect_and_optimize()
                    if found is not None:
                        if found is not proposal:
                            proposal, token = found, uuid.uuid4().hex
                        value = dict(poses=proposal.poses, original_poses=proposal.original_poses,
                                     loop_edges=proposal.loop_edges, status=proposal.status, token=token)
                elif command == 'apply':
                    if proposal is None or request['token'] != token:
                        raise ValueError('stale or unknown proposal token')
                    backend.apply(proposal)
                    proposal, token = None, None
                else:
                    raise ValueError('Unknown KISS IPC command')
                _send(channel, dict(ok=True, value=value,
                                    observation_count=backend.observation_count, status=backend.status))
            except Exception as error:
                _send(channel, dict(ok=False, error=f'{type(error).__name__}: {error}'))
    finally:
        channel.close()


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] != '--serve':
        raise SystemExit('Private KISS worker; use the mapping/replay entrypoints')
    _serve(int(sys.argv[2]))
