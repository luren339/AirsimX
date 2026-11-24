"""Source-stamp filtering and real-sample selection extracted from acquisition runs.

No ROS dependency. A new simulation run needs a new instance (restart the node).
Selection never interpolates or publishes a source measurement more than once.
"""

import math


class SourceSampleFilter:
    def __init__(self):
        self.last_timestamp = 0
        self.accepted = 0
        self.rejected = 0

    def accept(self, timestamp_ns):
        value = int(timestamp_ns)
        if value <= 0 or value >= (2**31) * 1_000_000_000:
            raise ValueError('sensor timestamp must be positive and fit ROS Time.sec')
        if value <= self.last_timestamp:
            self.rejected += 1
            return False
        self.last_timestamp = value
        self.accepted += 1
        return True


class ImuSampleSelector:
    """Keep actual IMU measurements using the existing source-time accumulator."""

    def __init__(self, target_hz=0.0):
        self.target_hz = float(target_hz)
        if not math.isfinite(self.target_hz) or self.target_hz < 0:
            raise ValueError('imu_target_hz_sim_time must be finite and nonnegative')
        self.last_timestamp = None
        self.phase = 0.0

    def select(self, timestamp_ns):
        value = int(timestamp_ns)
        if self.last_timestamp is not None and value <= self.last_timestamp:
            return False
        previous, self.last_timestamp = self.last_timestamp, value
        if previous is None or self.target_hz == 0.0:
            return True
        self.phase += (value - previous) * self.target_hz / 1_000_000_000.0
        if self.phase >= 1.0 - 1e-12:
            self.phase = max(0.0, self.phase - 1.0)
            return True
        return False
