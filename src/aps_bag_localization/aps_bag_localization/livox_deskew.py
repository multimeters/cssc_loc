"""Livox parsing and 3-D scan-end deskew, independent of ROS.

Integrate measured rear-center forward speed and rotated gyro as a body twist.
No odometry pose, wheel yaw, IMU acceleration, or invented rate bridges a gap.
Piecewise constant midpoint twists give exact SE(3) motion for constant rates;
motion knots are all wheel/IMU sample times plus the scan boundaries.
"""
from dataclasses import dataclass

import numpy as np


class IncompleteMotion(ValueError):
    """A sensor does not bracket every point with sufficiently close samples."""


def fallback_due(end_ns, now_ns, wait_timeout_s, latest_imu_ns, latest_wheel_ns):
    """Deadlines use acquisition time, so replay rate never changes coverage.

    Once both ordered streams have passed the end, missing old samples cannot
    arrive and the scan need not wait for its configured deadline.
    """
    if now_ns < end_ns:
        return False
    return (now_ns >= end_ns + round(wait_timeout_s * 1e9)
            or (latest_imu_ns is not None and latest_wheel_ns is not None
                and latest_imu_ns >= end_ns and latest_wheel_ns >= end_ns))


@dataclass
class Scan:
    xyz: np.ndarray
    intensity: np.ndarray
    times_ns: np.ndarray
    start_ns: int
    end_ns: int
    input_points: int
    rejected_points: int


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    if not np.all(np.isfinite(quaternion)) or abs(np.dot(quaternion, quaternion) - 1.) > 1e-6:
        raise ValueError('Rotation quaternion must be finite and normalized')
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def valid_mid360_tags(tags):
    """MID-360 bits 0:1/2:3/4:5: accept high/medium, reject low/reserved."""
    tags = np.asarray(tags, dtype=np.uint8)
    return ((tags & 0x03) < 2) & (((tags >> 2) & 0x03) < 2) & (((tags >> 4) & 0x03) < 2)


def parse_scan(message, settings):
    """Keep integer nanoseconds throughout; header uses the raw driver's label.

    The shared livox_frame label does not relocate lidar points to the IMU chip.
    The output coordinates remain at the lidar origin, independently of the IMU
    translation in the static tree.
    """
    if message.header.frame_id != settings['raw_frame']:
        raise ValueError('unexpected raw lidar frame: ' + message.header.frame_id)
    count = len(message.points)
    if count == 0 or count != int(message.point_num):
        raise ValueError('empty scan or CustomMsg.point_num mismatch')
    base = int(message.timebase)
    header = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
    if base <= 0 or base > 2_147_483_646_000_000_000:
        raise ValueError('timebase outside ROS timestamp range')
    if abs(header - base) > round(settings['header_tolerance_s'] * 1e9):
        raise ValueError('header stamp and timebase disagree')
    offsets = np.fromiter((point.offset_time for point in message.points), dtype=np.int64, count=count)
    if np.any(offsets < 0) or int(offsets.max()) > round(settings['max_scan_duration_s'] * 1e9):
        raise ValueError('invalid scan point time offsets')
    # Boundaries precede quality filtering: filtering cannot change scan time.
    start, end = base + int(offsets.min()), base + int(offsets.max())
    xyz = np.fromiter((v for p in message.points for v in (p.x, p.y, p.z)),
                      dtype=np.float64, count=3*count).reshape(-1, 3)
    intensity = np.fromiter((p.reflectivity for p in message.points), dtype=np.float32, count=count)
    tags = np.fromiter((p.tag for p in message.points), dtype=np.uint8, count=count)
    ranges2 = np.einsum('ij,ij->i', xyz, xyz)
    keep = (np.all(np.isfinite(xyz), axis=1) & np.isfinite(intensity)
            & (ranges2 >= settings['min_range_m'] ** 2)
            & (ranges2 <= settings['max_range_m'] ** 2))
    if settings['reject_invalid_tags']:
        keep &= valid_mid360_tags(tags)
    if not np.any(keep):
        raise ValueError('scan has no points after quality and range filtering')
    return Scan(xyz[keep], intensity[keep], base + offsets[keep], start, end, count, int((~keep).sum()))


def _covered_samples(samples, start_ns, end_ns, max_gap_s, label):
    if len(samples) < 2:
        raise IncompleteMotion(label + ': fewer than two samples')
    times = np.array([row[0] for row in samples], dtype=np.int64)
    if np.any(np.diff(times) <= 0):
        raise ValueError(label + ': samples must have strictly increasing timestamps')
    begin = int(np.searchsorted(times, start_ns, side='right')) - 1
    finish = int(np.searchsorted(times, end_ns, side='left'))
    if begin < 0 or finish >= len(times):
        raise IncompleteMotion(label + ': no bracketing samples')
    times = times[begin:finish+1]
    values = np.asarray([row[1:] for row in samples[begin:finish+1]], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError(label + ': non-finite motion sample')
    if len(times) > 1 and np.max(np.diff(times)) > round(max_gap_s * 1e9):
        raise IncompleteMotion(label + ': gap exceeds configured limit')
    return times, values


def _twist_exp(omega, velocity, dt):
    """Batched exp([omega, velocity] * dt), stable through zero angular rate."""
    vector = np.asarray(omega) * np.asarray(dt)[..., None]
    distance = np.asarray(velocity) * np.asarray(dt)[..., None]
    theta2 = np.einsum('...i,...i->...', vector, vector)
    theta = np.sqrt(theta2)
    small = theta2 < 1e-8
    # np.divide where avoids warning/NaN at zero, unlike np.where division.
    a = np.divide(np.sin(theta), theta, out=np.ones_like(theta), where=~small)
    b = np.divide(1.-np.cos(theta), theta2, out=np.full_like(theta, .5), where=~small)
    c = np.divide(theta-np.sin(theta), theta2*theta, out=np.full_like(theta, 1./6), where=~small)
    a = np.where(small, 1-theta2/6+theta2*theta2/120, a)
    b = np.where(small, .5-theta2/24+theta2*theta2/720, b)
    c = np.where(small, 1./6-theta2/120+theta2*theta2/5040, c)
    cross = np.zeros(vector.shape[:-1] + (3, 3))
    cross[..., 0, 1], cross[..., 0, 2] = -vector[..., 2], vector[..., 1]
    cross[..., 1, 0], cross[..., 1, 2] = vector[..., 2], -vector[..., 0]
    cross[..., 2, 0], cross[..., 2, 1] = -vector[..., 1], vector[..., 0]
    square = cross @ cross
    rotation = np.eye(3) + a[..., None, None]*cross + b[..., None, None]*square
    jacobian = np.eye(3) + b[..., None, None]*cross + c[..., None, None]*square
    translation = np.einsum('...ij,...j->...i', jacobian, distance)
    return rotation, translation


def deskew_scan(scan, imu_samples, wheel_samples, lidar_to_base_rotation, lidar_origin_base,
                max_imu_gap_s, max_wheel_gap_s):
    """Return every point in lidar axes at scan.end_ns, or raise IncompleteMotion.

    imu_samples rows are (integer_ns, wx_base, wy_base, wz_base), wheel rows are
    (integer_ns, forward_vx). Input coordinates/time ordering need not be sorted.
    A whole scan is either compensated or unchanged by the caller, never partly
    compensated. Lever-arm motion comes from the full rigid mount transform.
    """
    ti, wi = _covered_samples(imu_samples, scan.start_ns, scan.end_ns, max_imu_gap_s, 'imu')
    tv, vv = _covered_samples(wheel_samples, scan.start_ns, scan.end_ns, max_wheel_gap_s, 'wheel')
    if scan.end_ns == scan.start_ns:
        return scan.xyz.copy()
    knots = np.unique(np.concatenate(([scan.start_ns, scan.end_ns],
        ti[(ti > scan.start_ns) & (ti < scan.end_ns)], tv[(tv > scan.start_ns) & (tv < scan.end_ns)])))
    seconds = (knots - scan.start_ns) * 1e-9
    omega = np.column_stack([np.interp(seconds, (ti-scan.start_ns)*1e-9, wi[:, axis]) for axis in range(3)])
    speed = np.interp(seconds, (tv-scan.start_ns)*1e-9, vv[:, 0])
    omega = (omega[:-1] + omega[1:]) * .5
    velocity = np.zeros_like(omega)
    velocity[:, 0] = (speed[:-1] + speed[1:]) * .5
    increments_r, increments_p = _twist_exp(omega, velocity, np.diff(seconds))
    rotations, positions = [np.eye(3)], [np.zeros(3)]
    for dr, dp in zip(increments_r, increments_p):
        positions.append(positions[-1] + rotations[-1] @ dp)
        rotations.append(rotations[-1] @ dr)
    rotations, positions = np.asarray(rotations), np.asarray(positions)
    bins = np.clip(np.searchsorted(knots, scan.times_ns, side='right') - 1, 0, len(knots)-2)
    partial_r, partial_p = _twist_exp(omega[bins], velocity[bins], (scan.times_ns-knots[bins])*1e-9)
    point_rotations = rotations[bins] @ partial_r
    point_positions = positions[bins] + np.einsum('nij,nj->ni', rotations[bins], partial_p)
    mount_r, mount_p = np.asarray(lidar_to_base_rotation), np.asarray(lidar_origin_base)
    base_points = scan.xyz @ mount_r.T + mount_p
    world_points = np.einsum('nij,nj->ni', point_rotations, base_points) + point_positions
    end_base_points = (world_points - positions[-1]) @ rotations[-1]
    return (end_base_points - mount_p) @ mount_r
