"""Background loop verification and atomic whole-map correction.

The foreground keeps accepting full resolution observations while a private map
is rebuilt. New observations are replayed before the swap; an old registration
result is invalidated by the mapper's generation counter at that same swap.
"""
import threading
import time

import numpy as np

from loop_closure import LoopClosure
from mapping_rebuild import rebuild, append_rebuilt, corrected_poses


class MappingLoopWorker:
    def __init__(self, lock, config, map_voxel_size, registration_voxel_size,
                 grid_resolution, prepare_commit, backend=None):
        self.lock = lock
        self.backend = backend if backend is not None else LoopClosure(config)
        self.resolutions = (map_voxel_size, registration_voxel_size, grid_resolution)
        self.prepare_commit = prepare_commit
        self.observations = []
        self.checked = 0
        self.applied_loops = 0
        self.state = 'waiting'
        self.error = ''
        self.last_result = {}
        self.backend_status = {}
        self.event = threading.Event()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name='mapping-loop', daemon=True)
        self.thread.start()

    def append(self, observation):
        # Caller holds the mapper lock and has prepared all scan payloads.
        self.observations.append(observation)
        self.event.set()

    def report(self):
        with self.lock:
            return dict(state=self.state, applied_loops=self.applied_loops,
                        observations=len(self.observations), checked_observations=self.checked,
                        retained_scan_bytes=sum(o.points_body.nbytes for o in self.observations),
                        error=self.error, backend=dict(self.backend_status),
                        last_loop=dict(self.last_result))

    def ready_for_save(self):
        # Check and snapshot the map within the same caller-held lock.
        return (self.checked == len(self.observations) and
                self.state in ('idle', 'applied', 'waiting') and not self.error)

    def _run(self):
        while not self.stopping.is_set():
            self.event.wait()
            self.event.clear()
            if self.stopping.is_set():
                return
            try:
                with self.lock:
                    snapshot = tuple(self.observations)
                    self.state, self.error = 'checking', ''
                for observation in snapshot[self.backend.observation_count:]:
                    if self.stopping.is_set():
                        return
                    self.backend.append(observation.points_body, observation.pose)
                result = self.backend.detect_and_optimize()
                with self.lock:
                    self.backend_status = dict(self.backend.status)
                if result is None:
                    with self.lock:
                        self.checked = len(snapshot)
                        self.state = 'idle'
                    continue
                self._rebuild_and_commit(snapshot, result)
            except Exception as error:
                # Never disable live mapping or publish a partial correction.
                # A new observation can retry the backend; save reports failure.
                with self.lock:
                    self.state, self.error = 'error', str(error)

    def _rebuild_and_commit(self, snapshot, result):
        started = time.monotonic()
        poses = list(corrected_poses(snapshot, result.original_poses, result.poses))
        delta = poses[-1] @ np.linalg.inv(snapshot[-1].pose)
        with self.lock:
            self.state = 'rebuilding'
        store, grid = rebuild(snapshot, poses, *self.resolutions)
        rebuilt = list(snapshot)
        # The worker owns the candidate. Replay concurrent foreground scans
        # off-lock, then swap only when it contains the entire live journal.
        while not self.stopping.is_set():
            with self.lock:
                tail = tuple(self.observations[len(rebuilt):])
                if not tail:
                    observations = [o.with_pose(p) for o, p in zip(rebuilt, poses)]
                    summary = dict(result.status)
                    summary.update(rebuild_seconds=time.monotonic()-started,
                                   corrected_observations=len(observations),
                                   edges=list(result.loop_edges))
                    publish = self.prepare_commit(store, grid, poses, delta)
                    # apply validates the unchanged backend snapshot. No new
                    # backend observations are added while rebuilding.
                    self.backend.apply(result)
                    publish()
                    self.observations = observations
                    self.checked = len(snapshot)
                    self.applied_loops += 1
                    self.last_result = summary
                    self.backend_status = dict(self.backend.status)
                    self.state, self.error = 'applied', ''
                    # Tail scans need their own backend append/check, even
                    # though their geometry is already in the corrected map.
                    if len(observations) > len(snapshot):
                        self.event.set()
                    return
            for observation in tail:
                if self.stopping.is_set():
                    return
                pose = delta @ observation.pose
                append_rebuilt(store, grid, observation, pose)
                rebuilt.append(observation)
                poses.append(pose)

    def close(self):
        self.stopping.set()
        self.event.set()
        self.thread.join()
