"""Recorder statistics independent of ROS and of the localization method."""
import math


class TrajectorySummary:
    def __init__(self, min_samples=100, min_span=5.0):
        self.min_samples = min_samples
        self.min_span = min_span
        self.samples = 0
        self.invalid_samples = 0
        self.nonmonotonic_samples = 0
        self.first_stamp = None
        self.last_stamp = None
        self.frame_id = None
        self.last_position = None
        self.last_quaternion = None
        self.path_length = 0.0
        self.max_sample_gap = 0.0

    def add(self, stamp, frame_id, position, quaternion, covariance):
        values = [stamp, *position, *quaternion, *covariance]
        norm = sum(value * value for value in quaternion)
        if (not frame_id or not all(math.isfinite(value) for value in values)
                or abs(norm - 1.0) > 0.01
                or (self.frame_id is not None and frame_id != self.frame_id)):
            self.invalid_samples += 1
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.nonmonotonic_samples += 1
            return False
        if self.first_stamp is None:
            self.first_stamp = stamp
            self.frame_id = frame_id
        if self.last_position is not None:
            self.path_length += math.dist(position, self.last_position)
            self.max_sample_gap = max(self.max_sample_gap, stamp - self.last_stamp)
        self.last_stamp = stamp
        self.last_position = list(position)
        self.last_quaternion = list(quaternion)
        self.samples += 1
        return True

    def report(self):
        span = 0.0 if self.first_stamp is None else self.last_stamp - self.first_stamp
        checks = (self.samples >= self.min_samples and span >= self.min_span
                  and self.invalid_samples == 0 and self.nonmonotonic_samples == 0)
        return {
            'samples': self.samples, 'invalid_samples': self.invalid_samples,
            'nonmonotonic_samples': self.nonmonotonic_samples,
            'first_stamp_sec': self.first_stamp, 'last_stamp_sec': self.last_stamp,
            'span_sec': span, 'frame_id': self.frame_id,
            'path_length_m': self.path_length, 'max_sample_gap_sec': self.max_sample_gap,
            'last_position_xyz': self.last_position,
            'last_quaternion_xyzw': self.last_quaternion,
            'minimum_samples': self.min_samples, 'minimum_span_sec': self.min_span,
            'sample_checks_passed': checks,
            'accuracy_verified': False,
            'note': 'Sample checks measure output continuity and numeric validity, not localization accuracy.',
        }
