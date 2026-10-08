"""ROS-independent clock and command handling for the EDU adapter."""
import math


class SensorClock:
    def __init__(self):
        self.offset_ns = None
        self.last_odom_ns = None

    def observe_odom(self, sensor_ns, now_ns):
        if sensor_ns <= 0:
            return False
        # Drop reordered samples. A sensor clock reset requires a bridge restart,
        # rather than making an old/replayed sample appear current.
        if self.last_odom_ns is not None and sensor_ns < self.last_odom_ns:
            return False
        if self.offset_ns is None:
            self.offset_ns = now_ns - sensor_ns
        self.last_odom_ns = sensor_ns
        return True

    def convert(self, sensor_ns):
        if self.offset_ns is None or sensor_ns <= 0:
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
