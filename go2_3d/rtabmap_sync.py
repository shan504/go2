"""Bounded image/laser association before RTAB-Map's exact-time subscriber.

The source exposure timestamp is retained in each returned association. The
caller may relabel the image with the laser timestamp ONLY after this measured
skew plus camera timing uncertainty passes the configured total error budget.
This is bounded association, not a claim of hardware synchronization.
"""
import math


class AssociationGate:
    def __init__(self, max_skew_sec, uncertainty_sec, max_pending=100):
        if (not all(math.isfinite(v) for v in (max_skew_sec, uncertainty_sec)) or
                not 0 < uncertainty_sec < max_skew_sec or max_pending < 1):
            raise ValueError('association requires a positive total budget greater than measured uncertainty')
        self.budget_ns = int((max_skew_sec - uncertainty_sec) * 1e9)
        self.uncertainty_sec = uncertainty_sec
        self.max_pending = max_pending
        self.cameras, self.scans = {}, {}
        self.last_camera = self.last_scan = 0
        self.paired = self.skipped = self.camera_dropped = 0
        self.last_skew_sec = None

    @staticmethod
    def _stamp(value):
        if type(value) is not int or value <= 0:
            raise ValueError('positive integer nanosecond timestamp required')
        return value

    def add_camera(self, stamp_ns, payload):
        stamp_ns = self._stamp(stamp_ns)
        if stamp_ns <= self.last_camera:
            raise ValueError('camera stamps must strictly increase')
        self.last_camera = stamp_ns
        self.cameras[stamp_ns] = payload
        while len(self.cameras) > self.max_pending:
            del self.cameras[min(self.cameras)]
            self.camera_dropped += 1

    def add_scan(self, stamp_ns, payload):
        stamp_ns = self._stamp(stamp_ns)
        if stamp_ns <= self.last_scan:
            raise ValueError('accepted GICP stamps must strictly increase')
        if len(self.scans) >= self.max_pending:
            raise ValueError('image association backlog full; inspect camera timing/rate')
        self.last_scan = stamp_ns
        self.scans[stamp_ns] = payload

    def ready(self):
        result = []
        for scan_stamp in sorted(self.scans):
            candidates = [stamp for stamp in self.cameras
                          if abs(stamp - scan_stamp) <= self.budget_ns]
            if candidates:
                camera_stamp = min(candidates, key=lambda stamp: (abs(stamp - scan_stamp), stamp))
                result.append((scan_stamp, self.scans.pop(scan_stamp), camera_stamp,
                               self.cameras.pop(camera_stamp)))
                self.paired += 1
                self.last_skew_sec = abs(camera_stamp - scan_stamp) / 1e9
            elif self.last_camera > scan_stamp + self.budget_ns:
                del self.scans[scan_stamp]
                self.skipped += 1
            else:
                break
        # A camera exposure older than every possible next scan cannot pair.
        oldest = min(self.scans) if self.scans else self.last_scan
        for stamp in tuple(self.cameras):
            if stamp < oldest - self.budget_ns:
                del self.cameras[stamp]
                self.camera_dropped += 1
        return result

    def expire_before(self, stamp_ns):
        """Bound memory during camera loss; never synthesize a backend input."""
        for stamp in tuple(self.scans):
            if stamp < stamp_ns:
                del self.scans[stamp]
                self.skipped += 1

    def report(self):
        return dict(paired=self.paired, skipped=self.skipped,
                    pending_scans=len(self.scans), pending_images=len(self.cameras),
                    camera_dropped=self.camera_dropped, last_skew_sec=self.last_skew_sec,
                    measured_uncertainty_sec=self.uncertainty_sec,
                    allowed_skew_sec=self.budget_ns / 1e9)
