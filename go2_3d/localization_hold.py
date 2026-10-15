"""Bound dead reckoning by the last accepted scan, never by rejected scans."""
import numpy as np
from geometry import rotation_angle


class OdometryBudget:
    def __init__(self):
        self.reset(None, None)

    def reset(self, odom, stamp):
        self.previous = None if odom is None else odom.copy()
        self.stamp = stamp
        self.distance = self.angle = 0.0

    def observe(self, odom, stamp):
        if self.previous is None or stamp < self.stamp:
            return False
        if stamp > self.stamp:
            delta = np.linalg.inv(self.previous) @ odom
            self.distance += float(np.linalg.norm(delta[:3, 3]))
            self.angle += float(rotation_angle(delta))
            self.previous, self.stamp = odom.copy(), stamp
        return True

    def exceeded(self, distance, angle):
        # Accumulate travel, including reversals; returning to the starting
        # pose cannot replenish the dead-reckoning budget.
        if self.distance > distance:
            return 'odometry_distance_limit'
        if self.angle > angle:
            return 'odometry_angle_limit'
        return ''
