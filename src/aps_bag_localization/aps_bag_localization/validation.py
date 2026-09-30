"""Pure validation helpers, usable without ROS for regression tests."""
import math


def diagonal_covariance(values, dimension, variance_floor):
    """Use reported diagonal variances with a positive floor, discard correlations.

    Bag covariances are not calibrated. Diagonalization guarantees a positive
    definite observation covariance; the floor is an assumption, not calibration.
    """
    if len(values) != dimension * dimension or variance_floor <= 0:
        raise ValueError('invalid covariance dimensions or variance floor')
    result = [0.0] * len(values)
    for index in range(dimension):
        value = float(values[index * (dimension + 1)])
        result[index * (dimension + 1)] = (
            max(value, variance_floor) if math.isfinite(value) else variance_floor)
    return result


class FreshnessGuard:
    """Latch a stop on the first missing stream or time reversal after activation."""

    STREAMS = ('wheel', 'imu', 'gyro')

    def __init__(self, timeout=0.2, future_tolerance=0.1):
        if timeout <= 0 or future_tolerance < 0:
            raise ValueError('invalid time tolerances')
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.latest = {}
        self.active = False
        self.stop_reason = ''
        self.last_now = None

    def observe(self, stream, stamp):
        if stream not in self.STREAMS or not math.isfinite(stamp) or stamp <= 0:
            return False
        previous = self.latest.get(stream)
        if previous is not None and stamp < previous:
            if self.active:
                self.stop_reason = 'nonmonotonic_' + stream
            return False
        self.latest[stream] = stamp
        return True

    def problem(self, now):
        if not math.isfinite(now) or now <= 0:
            return 'waiting_for_clock'
        if self.last_now is not None and now < self.last_now - 1e-6:
            if self.active:
                self.stop_reason = 'clock_moved_backwards'
            self.last_now = now
            return 'clock_moved_backwards'
        self.last_now = now
        if self.stop_reason:
            return self.stop_reason
        for stream in self.STREAMS:
            stamp = self.latest.get(stream)
            if stamp is None:
                reason = 'waiting_for_' + stream
            elif now - stamp > self.timeout:
                reason = 'stale_' + stream
            elif stamp - now > self.future_tolerance:
                reason = 'future_' + stream
            else:
                continue
            if self.active:
                self.stop_reason = reason
            return reason
        return ''

    def activate(self, now):
        if self.problem(now):
            return False
        self.active = True
        return True
