"""Latest-only background recovery; sensor/TF callbacks never run the search."""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class RecoveryJob:
    generation: int
    serial: int
    points: object
    odom: object
    correction: object
    source_odom: object
    stamp_ns: int
    received: float


class RecoveryWorker:
    def __init__(self, backend, completed):
        self.backend, self.completed = backend, completed
        self.lock = threading.Lock()
        self.event, self.stopping = threading.Event(), threading.Event()
        self._busy = threading.Event()
        self.pending = None
        self.token = None
        self.last_error = None
        self.thread = threading.Thread(target=self._run, name='localization-recovery', daemon=True)
        self.thread.start()

    @property
    def busy(self):
        return self._busy.is_set()

    def submit(self, job):
        with self.lock:
            self.pending = job
        self.event.set()

    def _run(self):
        while not self.stopping.is_set():
            self.event.wait()
            self.event.clear()
            with self.lock:
                job, self.pending = self.pending, None
            if self.stopping.is_set():
                return
            if job is None:
                continue
            self._busy.set()
            started = time.monotonic()
            try:
                token = job.generation, job.serial
                if token != self.token:
                    self.backend.reset()
                    self.token = token
                result = self.backend.observe(job.points, job.odom, job.correction,
                                              job.received, time.monotonic())
                maximum_age = getattr(getattr(self.backend, 'config', None), 'max_scan_age', 1.0)
                if time.monotonic()-job.received > maximum_age:
                    # An expensive search cannot contribute a stale frame to
                    # the next frame's temporal recovery proof.
                    self.backend.reset()
                    result, error = None, 'expired recovery computation'
                else:
                    error = None
            except Exception as failure:
                result, error = None, str(failure)
                self.backend.reset()
            if not self.stopping.is_set():
                try:
                    eligible = self.completed(job, result, time.monotonic()-started, error)
                    if eligible is False:
                        self.backend.reset()
                except Exception as failure:
                    # Keep the worker alive after a reporting/commit failure;
                    # no partially verified session may survive the error.
                    self.last_error = str(failure)
                    self.backend.reset()
            self._busy.clear()

    def close(self):
        self.stopping.set()
        self.event.set()
        self.thread.join()
