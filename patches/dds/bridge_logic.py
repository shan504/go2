"""ROS-independent clock and command handling for the EDU adapter."""
import math
import time


class SensorClock:
    """One frozen clock mapping for native odometry, TF and point clouds.

    A first packet can have queued during startup. Calibrate from several
    advancing samples and choose the smallest receipt offset; later transport
    delays must never make an old sample look fresh. Host clock jumps are sticky
    because changing an active offset would invalidate existing TF. Individual
    stale/future samples are dropped without replacing the calibrated mapping.
    """
    CALIBRATION_NS = 500_000_000
    MIN_CALIBRATION_SAMPLES = 5
    MAX_FUTURE_NS = 100_000_000
    MAX_AGE_NS = 1_000_000_000
    MAX_CLOCK_JUMP_NS = 100_000_000
    MAX_CLOCK_SAMPLE_SPAN_NS = 20_000_000

    def __init__(self):
        self.offset_ns = None
        self.last_odom_ns = None
        self.fault_reason = None
        self.fault_detail = None
        self.last_rejection_reason = None
        self.sample_age_ns = None
        self.calibration_samples = 0
        self.calibration_span_ns = 0
        self._candidate_offset_ns = None
        self._calibration_started_ns = None
        self._last_clock_pair = None

    def status(self):
        return dict(state=('fault' if self.fault_reason else
                           'ready' if self.offset_ns is not None else 'calibrating'),
                    offset_ns=self.offset_ns, calibration_samples=self.calibration_samples,
                    calibration_span_s=self.calibration_span_ns/1e9,
                    fault_reason=self.fault_reason, fault_detail=self.fault_detail,
                    last_rejection_reason=self.last_rejection_reason,
                    sample_age_s=None if self.sample_age_ns is None else self.sample_age_ns/1e9)

    def _fault(self, reason, detail):
        self.fault_reason, self.fault_detail = reason, detail
        self.last_rejection_reason = reason
        return False

    def observe_odom_sample(self, sensor_ns, ros_ns, monotonic_before_ns, monotonic_after_ns):
        """Use a bracketed host-clock read, excluding scheduling uncertainty.

        A process can be descheduled between reading ROS and monotonic time.
        Such a sample says nothing reliable about a wall-clock step and must
        not alter the accepted source watermark, calibration or clock pair.
        """
        if self.fault_reason is not None:
            return False
        if (any(not isinstance(value, int) or isinstance(value, bool)
                for value in (monotonic_before_ns, monotonic_after_ns)) or
                not 0 <= monotonic_after_ns-monotonic_before_ns <= self.MAX_CLOCK_SAMPLE_SPAN_NS):
            self.last_rejection_reason = 'host_clock_sample_delayed'
            return False
        return self.observe_odom(sensor_ns, ros_ns, (monotonic_before_ns+monotonic_after_ns)//2)

    def observe_odom(self, sensor_ns, now_ns, monotonic_ns=None):
        if self.fault_reason is not None:
            return False
        if monotonic_ns is None:
            monotonic_ns = time.monotonic_ns()
        if any(not isinstance(value, int) or isinstance(value, bool) or value <= 0
               for value in (sensor_ns, now_ns, monotonic_ns)):
            self.last_rejection_reason = 'invalid_timestamp'
            return False
        # Drop reordered samples. A sensor clock reset requires a bridge restart,
        # rather than making an old/replayed sample appear current.
        if self.last_odom_ns is not None and sensor_ns <= self.last_odom_ns:
            self.last_rejection_reason = ('duplicate_odom' if sensor_ns == self.last_odom_ns
                                          else 'reordered_odom')
            return False
        if self._last_clock_pair is not None:
            last_ros, last_monotonic = self._last_clock_pair
            monotonic_delta = monotonic_ns-last_monotonic
            ros_delta = now_ns-last_ros
            if monotonic_delta <= 0:
                return self._fault('monotonic_clock_regressed', dict(delta_ns=monotonic_delta))
            if abs(ros_delta-monotonic_delta) > self.MAX_CLOCK_JUMP_NS:
                return self._fault('ros_clock_jump', dict(
                    ros_delta_ns=ros_delta, monotonic_delta_ns=monotonic_delta,
                    difference_ns=ros_delta-monotonic_delta))
        self._last_clock_pair = now_ns, monotonic_ns
        if self.offset_ns is None:
            # Bootstrap samples are not published, but must advance strictly
            # to contribute independent clock observations.
            self.last_odom_ns = sensor_ns
            if self._calibration_started_ns is None:
                self._calibration_started_ns = monotonic_ns
            self.calibration_span_ns = monotonic_ns-self._calibration_started_ns
            observed_offset = now_ns-sensor_ns
            self._candidate_offset_ns = (observed_offset if self._candidate_offset_ns is None
                                         else min(self._candidate_offset_ns, observed_offset))
            self.calibration_samples += 1
            if (self.calibration_span_ns < self.CALIBRATION_NS or
                    self.calibration_samples < self.MIN_CALIBRATION_SAMPLES):
                self.last_rejection_reason = 'calibrating'
                return False
            self.offset_ns = self._candidate_offset_ns
        self.sample_age_ns = now_ns-(sensor_ns+self.offset_ns)
        if self.sample_age_ns < -self.MAX_FUTURE_NS:
            self.last_rejection_reason = 'sensor_time_in_future'
            return False
        if self.sample_age_ns > self.MAX_AGE_NS:
            self.last_rejection_reason = 'stale_odom'
            return False
        # Once calibrated, only a sample eligible for publication advances
        # the watermark. A rejected future outlier must not make the next
        # good sample appear reordered or force TF to wait for that outlier.
        self.last_odom_ns = sensor_ns
        self.last_rejection_reason = None
        return True

    def convert(self, sensor_ns):
        if (self.offset_ns is None or self.fault_reason is not None or
                not isinstance(sensor_ns, int) or isinstance(sensor_ns, bool) or sensor_ns <= 0):
            return None
        return sensor_ns + self.offset_ns


class CommandGate:
    def __init__(self, timeout=0.5, max_linear=0.15, max_yaw=0.3):
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_yaw = max_yaw
        self.enabled = False
        self.command = None
        self.received_at = None
        self.active = False

    def set_enabled(self, enabled):
        if enabled == self.enabled:
            return False
        stop = self.active
        self.enabled = enabled
        self.command = None
        self.received_at = None
        self.active = False
        return stop

    def receive(self, x, y, yaw, now):
        if not self.enabled:
            return
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.command = None
            self.received_at = None
            return
        speed = math.hypot(x, y)
        if speed > self.max_linear:
            scale = self.max_linear / speed
            x, y = x * scale, y * scale
        yaw = max(-self.max_yaw, min(self.max_yaw, yaw))
        self.command = (x, y, yaw)
        self.received_at = now

    def poll(self, now, sensors_fresh):
        fresh = self.received_at is not None and 0 <= now - self.received_at <= self.timeout
        if self.enabled and sensors_fresh and fresh and self.command is not None:
            self.active = True
            return "move", self.command
        if self.active:
            self.active = False
            # Require a new command after stale data or a command timeout.
            self.command = None
            self.received_at = None
            return "stop", None
        return None, None
